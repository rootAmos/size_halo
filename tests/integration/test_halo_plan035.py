"""The Halo default reference: every model on (whole real machine units, redundancy, drag corrections, drawn layout, wing corrections)."""
import unittest

import aerosandbox.tools.units as u

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_plan030, solve_halo_sizing,
                                  whirl_flutter_cleared)


class Plan035ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = solve_halo_sizing()

    def test_defaults(self):
        a = HaloAssumptions()
        self.assertTrue(a.drag_corrections and a.redundancy and a.gearbox_stages and a.thermal_model)
        self.assertEqual(a.machine_mass_model, "units")
        self.assertEqual(HaloRequirements().mass_payload_kg, 900.0)
        self.assertFalse(assumptions_plan030.redundancy)
        self.assertEqual(assumptions_plan030.machine_mass_model, "torque_density")

    def test_closes_at_900_kg(self):
        r = self.result
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-3)
        self.assertEqual(r.mass_payload_kg, 900.0)
        # v3.11: plan 061 wing aspect ratio free under the 24 m overall-width limit (v3.10, AR 6.12, was 14,350 lb;
        # v3.7 14,357 lb, v3.6 15,179 lb).
        self.assertAlmostEqual(r.mass_takeoff_kg / u.lbm, 14163, delta=20)
        self.assertGreater(r.design.aspect_ratio_wing, 6.12)
        self.assertLessEqual(r.span_m + 2 * r.radius_rotor_m, HaloRequirements().max_width_overall_m * (1 + 1e-6))
        self.assertTrue(whirl_flutter_cleared(r, HaloRequirements(), HaloAssumptions()))

    def test_machines_are_whole_units_on_two_lanes(self):
        r = self.result
        self.assertEqual(r.count_lanes_motor, 2)
        for name, (count, fixed) in r.machine_units.items():
            self.assertIsInstance(fixed, int, msg=name)
            self.assertAlmostEqual(count, fixed, places=9, msg=name)
        self.assertLessEqual(r.design.reduction_ratio, 5.2)        # low-speed units: a single gear stage


if __name__ == "__main__":
    unittest.main()
