"""Plan 032: the Halo reference with the layout-anchored fuselage factor."""
import unittest

import aerosandbox.tools.units as u

import examples.halo_sizing as halo
from examples.halo_sizing import HaloAssumptions, solve_halo_sizing


class FuselageFactorTests(unittest.TestCase):
    def test_default_and_legacy_sets(self):
        self.assertEqual(HaloAssumptions().mass_factor_fuselage, 1.70)
        legacy = [name for name in dir(halo) if name.startswith("assumptions_") and
                  isinstance(getattr(halo, name), HaloAssumptions)]
        self.assertGreaterEqual(len(legacy), 10)
        for name in legacy:
            self.assertIsNone(getattr(halo, name).mass_factor_fuselage, msg=name)


class Plan032ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sized = solve_halo_sizing()            # the defaults are the plan 032 reference

    def test_closes_with_all_margins(self):
        self.assertGreater(self.sized.min_margin, -1e-6)
        self.assertLess(abs(self.sized.closure_residual_kg), 1e-3)
        self.assertAlmostEqual(self.sized.mass_takeoff_kg / u.lbm, 13307, delta=10)

    def test_fuselage_is_lighter_than_the_xv15_calibrated_one(self):
        masses = dict(self.sized.component_masses_kg)
        self.assertAlmostEqual(masses["fuselage"], 521.8, delta=2.0)


if __name__ == "__main__":
    unittest.main()
