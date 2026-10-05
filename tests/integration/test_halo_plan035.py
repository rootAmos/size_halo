"""Plan 035: AFDD spar-cap depth and minimum gauge, pylon inertia build-up, layout turbogenerator station."""
import unittest

import aerosandbox.tools.units as u

from examples.halo_sizing import HaloAssumptions, HaloRequirements, build_halo_aircraft, solve_halo_sizing


class Plan035ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sized = solve_halo_sizing()                     # the defaults are the plan 035 reference
        cls.wing_model = build_halo_aircraft(cls.sized.design, HaloRequirements(), HaloAssumptions()).wing.mass_model

    def test_closes_with_all_margins(self):
        self.assertGreater(self.sized.min_margin, -1e-6)
        self.assertLess(abs(self.sized.closure_residual_kg), 1e-3)
        self.assertAlmostEqual(self.sized.mass_takeoff_kg / u.lbm, 13038, delta=10)

    def test_wing_options_take_the_layout_values(self):
        self.assertAlmostEqual(self.wing_model.ratio_depth_spar_cap, 0.826, delta=0.002)
        self.assertEqual(self.wing_model.thickness_min_torque_box_m, 0.001)
        self.assertAlmostEqual(float(self.wing_model.radius_gyration_pylon_m), 0.842, delta=0.01)


if __name__ == "__main__":
    unittest.main()
