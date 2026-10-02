import unittest

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.aircraft_mass_closure import build_reference_aircraft, solve_mass_closure


class MassClosureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = solve_mass_closure()
        cls.light = solve_mass_closure(mass_payload_kg=200.0)
        cls.heavy = solve_mass_closure(mass_payload_kg=400.0)

    def test_closure_residual_vanishes(self):
        self.assertLess(abs(self.reference.closure_residual_kg), 1e-6)

    def test_closed_mass_is_a_fixed_point_of_the_buildup(self):
        aircraft = build_reference_aircraft(self.reference.x_le_wing_m)
        rebuilt_kg = float(aircraft.get_mass(StructuralDesignCondition(self.reference.mass_takeoff_kg)))
        self.assertAlmostEqual(rebuilt_kg, self.reference.mass_takeoff_kg, places=5)

    def test_cg_placed_at_quarter_mac(self):
        self.assertAlmostEqual(self.reference.cg_fraction_mac, 0.25, places=8)

    def test_growth_factor_exceeds_one(self):
        growth_factor = (self.heavy.mass_takeoff_kg - self.light.mass_takeoff_kg) / 200.0
        self.assertGreater(growth_factor, 1.0)
        self.assertLess(self.light.mass_takeoff_kg, self.reference.mass_takeoff_kg)
        self.assertLess(self.reference.mass_takeoff_kg, self.heavy.mass_takeoff_kg)

    def test_masses_sum_and_are_positive(self):
        masses = dict(self.reference.component_masses_kg)
        self.assertTrue(all(mass_kg > 0 for mass_kg in masses.values()))
        self.assertAlmostEqual(sum(masses.values()), self.reference.mass_takeoff_kg, places=6)
        self.assertAlmostEqual(self.reference.mass_empty_kg, self.reference.mass_takeoff_kg - 300.0, places=6)


if __name__ == "__main__":
    unittest.main()
