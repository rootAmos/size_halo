"""Tier 16: turboshaft lapse in density and temperature, ISA + offset atmosphere."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.powertrain.components.turboshaft import (DensityTemperatureLapse, SimpleTurboshaft,
                                                               density_sea_level_kg_m3)

altitudes_m = (0.0, 500.0, 1219.2, 2500.0, 4000.0)


class TemperatureLapseTests(unittest.TestCase):
    def test_standard_day_reproduces_density_lapse_exactly(self):
        legacy = SimpleTurboshaft(power_rated_W=8e5, lapse_exponent=0.797)
        lapsed = SimpleTurboshaft(power_rated_W=8e5, lapse_exponent=0.797,
                                  lapse_model=DensityTemperatureLapse(0.797, 2.49))
        for h in altitudes_m:
            for atmosphere in (asb.Atmosphere(altitude=h), asb.Atmosphere(altitude=h, temperature_deviation=0.0)):
                self.assertEqual(float(lapsed.power_available_W(atmosphere)),
                                 float(legacy.power_available_W(atmosphere)))
        self.assertEqual(lapsed.power_available_W(None), 8e5)

    def test_default_turboshaft_unchanged(self):
        engine = SimpleTurboshaft(lapse_exponent=0.8)
        self.assertIsNone(engine.lapse_model)
        atmosphere = asb.Atmosphere(altitude=3000.0, temperature_deviation=20.0)
        self.assertAlmostEqual(float(engine.power_available_W(atmosphere)),
                               150000.0 * float(atmosphere.density() / density_sea_level_kg_m3)**0.8, places=6)

    def test_zero_temperature_exponent_is_density_lapse_on_any_day(self):
        model = DensityTemperatureLapse(0.797, 0.0)
        for offset_K in (-15.0, 10.0, 30.0):
            atmosphere = asb.Atmosphere(altitude=1000.0, temperature_deviation=offset_K)
            self.assertAlmostEqual(float(model.power_ratio(atmosphere)),
                                   float(atmosphere.density() / density_sea_level_kg_m3)**0.797, places=12)

    def test_fixed_pressure_ratio_closed_form(self):
        """At one pressure altitude, hot / standard = (T / T_ISA)^-(n + m), since density ~ 1 / T."""
        n, m = 0.797, 2.49
        model = DensityTemperatureLapse(n, m)
        for h in altitudes_m:
            standard, hot = asb.Atmosphere(altitude=h), asb.Atmosphere(altitude=h, temperature_deviation=27.7)
            self.assertAlmostEqual(float(hot.pressure() / standard.pressure()), 1.0, places=12)
            theta = float(hot.temperature() / standard.temperature())
            self.assertAlmostEqual(float(model.power_ratio(hot) / model.power_ratio(standard)), theta**-(n + m),
                                   places=10)

    def test_hotter_means_less_power_colder_more(self):
        model = DensityTemperatureLapse(0.797, 2.49)
        ratios = [float(model.power_ratio(asb.Atmosphere(altitude=1219.2, temperature_deviation=dt)))
                  for dt in (-20.0, 0.0, 10.0, 27.7, 40.0)]
        self.assertTrue(all(b < a for a, b in zip(ratios, ratios[1:])))
        # The temperature term adds to the density term: hotter costs more than density alone.
        density_only = DensityTemperatureLapse(0.797, 0.0)
        hot = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
        self.assertLess(float(model.power_ratio(hot)), float(density_only.power_ratio(hot)))

    def test_symbolic_offset_and_altitude(self):
        """Find the offset at which sea-level power is 80 % of rated: T = T0 0.8^(-1 / (n + m))."""
        n, m = 0.797, 2.49
        engine = SimpleTurboshaft(power_rated_W=1e6, lapse_model=DensityTemperatureLapse(n, m))
        opti = asb.Opti()
        offset_K = opti.variable(init_guess=5.0)
        altitude_m = opti.variable(init_guess=100.0)
        opti.subject_to([altitude_m == 0.0,
                         engine.power_available_W(asb.Atmosphere(altitude=altitude_m,
                                                                 temperature_deviation=offset_K)) / 1e6 == 0.8])
        solution = opti.solve(verbose=False)
        temperature_sea_level_K = float(asb.Atmosphere(altitude=0.0).temperature())
        self.assertAlmostEqual(float(solution.value(offset_K)),
                               temperature_sea_level_K * (0.8**(-1 / (n + m)) - 1), places=5)


if __name__ == "__main__":
    unittest.main()
