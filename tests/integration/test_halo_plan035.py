"""The Halo default reference: every model on (whole real machine units, redundancy, drag corrections, drawn layout, wing corrections)."""
import unittest

import aerosandbox.tools.units as u

from examples.halo_sizing import HaloAssumptions, HaloRequirements, assumptions_plan030, solve_halo_sizing


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
        self.assertAlmostEqual(r.mass_takeoff_kg / u.lbm, 15179, delta=20)   # with the layout line (plans 037-038)

    def test_machines_are_whole_units_on_two_lanes(self):
        r = self.result
        self.assertEqual(r.count_lanes_motor, 2)
        for name, (count, fixed) in r.machine_units.items():
            self.assertIsInstance(fixed, int, msg=name)
            self.assertAlmostEqual(count, fixed, places=9, msg=name)
        self.assertLessEqual(r.design.reduction_ratio, 5.2)        # low-speed units: a single gear stage


if __name__ == "__main__":
    unittest.main()
