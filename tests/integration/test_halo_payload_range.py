"""Plan 045: the sized Halo flown off-design (fixed design) and its payload-range points."""
import unittest

from examples.halo_payload_range import sized_reference
from examples.halo_sizing import HaloRequirements, fuel_factor, solve_halo_off_design


class OffDesignArgumentTests(unittest.TestCase):
    def test_needs_exactly_one_of_range_and_payload(self):
        with self.assertRaises(ValueError):
            solve_halo_off_design(None)
        with self.assertRaises(ValueError):
            solve_halo_off_design(None, range_nm=400.0, mass_payload_kg=500.0)


class PayloadRangeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sized = sized_reference()        # the cached reference, or solved (v3.6 defaults)
        range_design_nm = HaloRequirements().range_m / 1852.0
        cls.design = solve_halo_off_design(cls.sized, range_nm=range_design_nm)
        cls.half = solve_halo_off_design(cls.sized, mass_payload_kg=0.5 * cls.sized.mass_payload_kg)
        cls.ferry = solve_halo_off_design(cls.sized, mass_payload_kg=0.0)
        cls.points = (cls.design, cls.half, cls.ferry)

    def test_design_point_reproduces_the_sizing(self):
        s, d = self.sized, self.design
        self.assertAlmostEqual(d.mass_payload_kg, s.mass_payload_kg, delta=0.5)
        self.assertAlmostEqual(d.mass_takeoff_kg, s.mass_takeoff_kg, delta=0.5)
        self.assertAlmostEqual(d.mass_fuel_kg, s.mass_fuel_kg, delta=0.5)
        self.assertAlmostEqual(d.mass_empty_kg, s.mass_empty_kg, delta=0.1)

    def test_empty_mass_is_fixed(self):
        for point in self.points:
            self.assertAlmostEqual(point.mass_empty_kg, self.sized.mass_empty_kg, delta=0.1)

    def test_range_rises_as_payload_falls_along_the_tanks(self):
        self.assertLess(self.design.range_nm, self.half.range_nm)
        self.assertLess(self.half.range_nm, self.ferry.range_nm)
        self.assertAlmostEqual(self.ferry.mass_payload_kg, 0.0, delta=1e-6)
        for point in (self.half, self.ferry):
            self.assertAlmostEqual(point.mass_fuel_kg, self.sized.mass_fuel_kg, delta=0.5)   # tanks full
            self.assertLess(point.mass_takeoff_kg, self.sized.mass_takeoff_kg)

    def test_fuel_and_mass_accounting(self):
        for point in self.points:
            burnt_kg = sum(s["fuel_kg"] for s in point.result.segments)
            self.assertAlmostEqual(point.mass_fuel_burnt_kg, burnt_kg, delta=1e-3)
            self.assertAlmostEqual(point.mass_fuel_kg, fuel_factor * burnt_kg, delta=1e-3)
            self.assertAlmostEqual(point.mass_takeoff_kg,
                                   point.mass_empty_kg + point.mass_payload_kg + point.mass_fuel_kg, delta=1e-2)
            self.assertLessEqual(point.mass_takeoff_kg, self.sized.mass_takeoff_kg + 1e-3)
            self.assertLessEqual(point.mass_fuel_kg, self.sized.mass_fuel_kg + 1e-3)

    def test_every_requirement_still_holds(self):
        for point in self.points:
            self.assertGreater(point.result.min_margin, -1e-6)
            self.assertLess(abs(point.result.offset_cg_trim_m), 1e-3)


if __name__ == "__main__":
    unittest.main()
