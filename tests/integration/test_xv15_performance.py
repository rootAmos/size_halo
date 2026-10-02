import unittest

import aerosandbox as asb
import aerosandbox.tools.units as u

from examples.xv15_performance import (Xv15PowerData, calibrate_figure_of_merit, fit_lapse_exponent,
                                       hover_ceiling_m, hover_mass_kg, part_power_sfc_ratios,
                                       thermal_efficiency_from_sfc, tier9_hover_power_ratio, xv15_engine)
from examples.xv15_reference import Xv15Reference

data = Xv15PowerData()


class Xv15PerformanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.n = fit_lapse_exponent()
        cls.figure_of_merit = calibrate_figure_of_merit(cls.n)

    def test_lapse_fit_within_six_percent(self):
        self.assertTrue(0.6 < self.n < 1.0)
        for h, p in zip(data.altitude_power_m, data.power_available_rotor_W):
            model = float(xv15_engine(self.n).power_available_W(asb.Atmosphere(altitude=h)))
            self.assertLess(abs(model / p - 1), 0.06, msg=f"{h / u.foot:.0f} ft")

    def test_calibration_reproduces_sea_level_hover(self):
        self.assertAlmostEqual(hover_mass_kg(data.altitude_hover_m[0], self.figure_of_merit, self.n)
                               / data.mass_hover_kg[0], 1.0, places=6)
        self.assertTrue(0.6 < self.figure_of_merit < 0.75)

    def test_hover_weights_predicted_within_five_percent(self):
        for h, m in zip(data.altitude_hover_m[1:], data.mass_hover_kg[1:]):
            self.assertLess(abs(hover_mass_kg(h, self.figure_of_merit, self.n) / m - 1), 0.05,
                            msg=f"{h / u.foot:.0f} ft")

    def test_hover_ceiling_at_design_weight(self):
        """Figure take-off line ~7,800 ft; SP-4517 8,650 ft (rating unstated): within 1,000 ft of that band."""
        ceiling_ft = hover_ceiling_m(Xv15Reference().mass_design_kg, self.figure_of_merit, self.n) / u.foot
        self.assertTrue(7800 - 1000 < ceiling_ft < 8650 + 1000)

    def test_part_power_sfc_within_two_percent(self):
        for throttle, published, model in part_power_sfc_ratios():
            self.assertLess(abs(model / published - 1), 0.02, msg=f"throttle {throttle:.2f}")

    def test_sfc_to_efficiency(self):
        self.assertAlmostEqual(thermal_efficiency_from_sfc(0.564), 0.2440, places=3)

    def test_tier9_hover_was_optimistic(self):
        self.assertGreater(tier9_hover_power_ratio(self.figure_of_merit), 1.2)


if __name__ == "__main__":
    unittest.main()
