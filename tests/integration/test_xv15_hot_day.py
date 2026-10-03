"""Tier 16: XV-15 95 F take-off power fit and hover-weight prediction (TM X-62407 figs. 6.2.2, 5.1.2)."""
import unittest

import aerosandbox as asb
import aerosandbox.tools.units as u

from examples.xv15_hot_day import (Xv15HotDayData, fit_temperature_lapse_exponent, hot_atmosphere, hover_mass_kg,
                                   temperature_from_fahrenheit_K, temperature_offset_K, xv15_engine_hot,
                                   xv15_lapse_model)
from examples.xv15_performance import Xv15PowerData, calibrate_figure_of_merit, fit_lapse_exponent

data, standard = Xv15HotDayData(), Xv15PowerData()


class Xv15HotDayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.n = fit_lapse_exponent()
        cls.model = xv15_lapse_model(cls.n)
        cls.figure_of_merit = calibrate_figure_of_merit(cls.n)   # Tier 10b, standard day: not refitted

    def test_temperature_definitions(self):
        self.assertAlmostEqual(temperature_from_fahrenheit_K(95.0), 308.15, places=10)
        self.assertAlmostEqual(float(temperature_offset_K(0.0, 308.15)), 20.0, places=6)
        offset_K = float(temperature_offset_K(4000 * u.foot, 308.15))
        self.assertAlmostEqual(offset_K, 27.66, delta=0.01)      # "4k/95" = ISA + 27.7 K
        self.assertAlmostEqual(float(hot_atmosphere(4000 * u.foot).temperature()), 308.15, places=6)

    def test_digitized_data_are_monotonic_and_below_standard_day(self):
        powers, masses = data.power_available_rotor_W, data.mass_hover_kg
        self.assertTrue(all(b < a for a, b in zip(powers, powers[1:])))
        self.assertTrue(all(b < a for a, b in zip(masses, masses[1:])))
        hot = dict(zip(data.altitude_power_m, powers))
        for h, p in zip(standard.altitude_power_m, standard.power_available_rotor_W):
            if h in hot:
                self.assertLess(hot[h], 0.85 * p)

    def test_temperature_exponent_fit(self):
        m = fit_temperature_lapse_exponent(self.n)
        self.assertAlmostEqual(m, self.model.lapse_exponent_temperature, places=12)
        self.assertTrue(2.0 < m < 3.0)
        # Paired 95 F / standard-day ratios at 0, 4,000, 8,000, 12,000 ft: residuals within 1.5 %.
        hot = dict(zip(data.altitude_power_m, data.power_available_rotor_W))
        for h, p in zip(standard.altitude_power_m, standard.power_available_rotor_W):
            if h in hot:
                a = hot_atmosphere(h)
                ratio_model = float((a.temperature() / (a.temperature() - a.temperature_deviation))**-(self.n + m))
                self.assertLess(abs(ratio_model / (hot[h] / p) - 1), 0.015, msg=f"{h / u.foot:.0f} ft")

    def test_hot_power_available_within_five_percent(self):
        """Residuals include the Tier 10b standard-day lapse error (up to about 4 % between 4,000 and 12,000 ft)."""
        engine = xv15_engine_hot(self.model)
        for h, p in zip(data.altitude_power_m, data.power_available_rotor_W):
            self.assertLess(abs(float(engine.power_available_W(hot_atmosphere(h))) / p - 1), 0.05,
                            msg=f"{h / u.foot:.0f} ft")
        self.assertAlmostEqual(float(engine.power_available_W(hot_atmosphere(0.0))) / data.power_available_rotor_W[0],
                               1.0, delta=0.01)

    def test_hot_hover_weights_predicted(self):
        """Prediction (no refit): within 3 % where the 95 F power curve exists, 5 % above it."""
        for h, m in zip(data.altitude_hover_m, data.mass_hover_kg):
            error = hover_mass_kg(hot_atmosphere(h), self.figure_of_merit, self.model) / m - 1
            band = 0.03 if h <= data.altitude_power_data_max_m + 1 else 0.05
            self.assertLess(abs(error), band, msg=f"{h / u.foot:.0f} ft: {error:+.3f}")

    def test_hot_day_lowers_hover_weight(self):
        standard_kg = hover_mass_kg(asb.Atmosphere(altitude=4000 * u.foot), self.figure_of_merit, self.model)
        hot_kg = hover_mass_kg(hot_atmosphere(4000 * u.foot), self.figure_of_merit, self.model)
        self.assertLess(hot_kg, 0.9 * standard_kg)


if __name__ == "__main__":
    unittest.main()
