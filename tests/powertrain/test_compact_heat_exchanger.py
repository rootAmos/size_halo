"""Plan 042: offset-strip-fin core by the ε-NTU method."""
import math
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.powertrain.components.compact_heat_exchanger import (
    OffsetStripFinCore, air_properties, effectiveness_cross_flow_unmixed, offset_strip_fin_factors,
    water_glycol_properties)

hot_day = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
air = air_properties(hot_day)
temperature_coolant_K = 273.15 + 60.0
halo_core = OffsetStripFinCore(count_wide=600.0, count_tall=40.0, count_long=15.0)


def counterflow_effectiveness(ntu, ratio):
    return (1 - math.exp(-ntu * (1 - ratio))) / (1 - ratio * math.exp(-ntu * (1 - ratio)))


class EffectivenessTests(unittest.TestCase):
    def test_single_stream_limit(self):
        # C_r -> 0 (one fluid at constant temperature): every arrangement gives 1 - exp(-NTU).
        for ntu in (0.2, 1.0, 3.0):
            self.assertAlmostEqual(effectiveness_cross_flow_unmixed(ntu, 1e-6), 1 - math.exp(-ntu), places=4)

    def test_small_ntu_is_ntu(self):
        self.assertAlmostEqual(effectiveness_cross_flow_unmixed(1e-3, 0.5) / 1e-3, 1.0, delta=2e-3)

    def test_increasing_and_below_counterflow(self):
        values = [effectiveness_cross_flow_unmixed(ntu, 0.7) for ntu in (0.5, 1.0, 2.0, 4.0)]
        self.assertEqual(values, sorted(values))
        for ntu, value in zip((0.5, 1.0, 2.0, 4.0), values):
            self.assertLess(value, counterflow_effectiveness(ntu, 0.7))

    def test_kays_london_table(self):
        # Kays and London, cross flow both unmixed, C_r = 1, NTU = 1: 0.476 (exact series value 0.4764).
        self.assertAlmostEqual(effectiveness_cross_flow_unmixed(1.0, 1.0), 0.476, delta=0.01)


class OffsetStripFinFactorTests(unittest.TestCase):
    geometry = (0.134, 0.012, 0.041)

    def test_decrease_with_reynolds(self):
        j_low, f_low = offset_strip_fin_factors(300.0, *self.geometry)
        j_high, f_high = offset_strip_fin_factors(5000.0, *self.geometry)
        self.assertGreater(j_low, j_high)
        self.assertGreater(f_low, f_high)

    def test_smooth_through_transition(self):
        # One correlation spans laminar to turbulent: no jump between neighbouring Reynolds numbers.
        reynolds = np.logspace(np.log10(200.0), np.log10(8000.0), 400)
        j, f = offset_strip_fin_factors(reynolds, *self.geometry)
        self.assertLess(np.max(np.abs(np.diff(np.log(j)))), 0.02)
        self.assertLess(np.max(np.abs(np.diff(np.log(f)))), 0.02)

    def test_laminar_asymptote(self):
        # At low Re the second factor tends to 1: j -> 0.6522 Re^-0.5403 x geometry.
        reynolds, (a, d, g) = 150.0, self.geometry
        j, _ = offset_strip_fin_factors(reynolds, a, d, g)
        self.assertAlmostEqual(j / (0.6522 * reynolds**-0.5403 * a**-0.1541 * d**0.1499 * g**-0.0678), 1.0,
                               delta=0.01)


class CoreTests(unittest.TestCase):
    def test_dimensions_and_air_passage(self):
        core = OffsetStripFinCore(count_wide=10.0, count_long=2.0, count_tall=3.0)
        self.assertAlmostEqual(core.width_m(), 10 * (0.102e-3 + 1.35e-3), places=12)
        self.assertAlmostEqual(core.height_m(), 3 * (2 * (0.102e-3 + 0.2e-3) + 14e-3 + 1e-3), places=12)
        self.assertAlmostEqual(core.length_m(), 12e-3, places=12)
        passage = core.passage_air()
        self.assertAlmostEqual(passage.area_flow_m2, 1.35e-3 * 14e-3 * 30, places=12)
        self.assertAlmostEqual(passage.area_heat_m2, 2 * (1.35e-3 + 14e-3) * 6e-3 * 2 * 30, places=12)

    def test_energy_balance(self):
        r = halo_core.evaluate(8.0, 3.0, hot_day.temperature(), temperature_coolant_K, air)
        coolant = water_glycol_properties()
        self.assertAlmostEqual(r.power_heat_W, 8.0 * air.specific_heat_J_kg_K
                               * (r.temperature_air_out_K - hot_day.temperature()), delta=1e-6 * r.power_heat_W)
        self.assertAlmostEqual(r.power_heat_W, 3.0 * coolant.specific_heat_J_kg_K
                               * (temperature_coolant_K - r.temperature_coolant_out_K), delta=1e-6 * r.power_heat_W)
        self.assertLess(r.temperature_air_out_K, temperature_coolant_K)

    def test_bigger_core_rejects_more_and_weighs_more(self):
        small = replace(halo_core, count_long=8.0)
        q_small = small.evaluate(8.0, 3.0, hot_day.temperature(), temperature_coolant_K, air).power_heat_W
        q_big = halo_core.evaluate(8.0, 3.0, hot_day.temperature(), temperature_coolant_K, air).power_heat_W
        self.assertGreater(q_big, q_small)
        self.assertGreater(halo_core.get_mass(), small.get_mass())

    def test_more_air_more_heat_more_pressure_drop(self):
        low = halo_core.evaluate(5.0, 3.0, hot_day.temperature(), temperature_coolant_K, air)
        high = halo_core.evaluate(10.0, 3.0, hot_day.temperature(), temperature_coolant_K, air)
        self.assertGreater(high.power_heat_W, low.power_heat_W)
        self.assertGreater(high.air.pressure_drop_Pa, low.air.pressure_drop_Pa)

    def test_halo_scale(self):
        # Order of magnitude against Kellermann 2021 (96 kW core about 0.48 x 0.73 x 0.18 m): a core under a
        # cubic metre rejects about 100 kW on the hot day for tens of kilograms, at a kilopascal of air loss.
        r = halo_core.evaluate(8.0, 3.0, hot_day.temperature(), temperature_coolant_K, air)
        self.assertTrue(80e3 < r.power_heat_W < 200e3)
        self.assertTrue(5.0 < halo_core.get_mass() < 60.0)
        self.assertTrue(200.0 < r.air.pressure_drop_Pa < 3000.0)

    def test_sizing_inside_opti(self):
        # The counts are design variables: lightest core rejecting 120 kW within a 1 kPa air-side loss.
        opti = asb.Opti()
        core = OffsetStripFinCore(count_wide=opti.variable(init_guess=500.0, scale=100.0, lower_bound=50.0),
                                  count_long=opti.variable(init_guess=10.0, lower_bound=2.0),
                                  count_tall=opti.variable(init_guess=30.0, scale=10.0, lower_bound=5.0))
        mass_flow_air_kg_s = opti.variable(init_guess=8.0, lower_bound=1.0, upper_bound=12.0)
        r = core.evaluate(mass_flow_air_kg_s, 3.0, hot_day.temperature(), temperature_coolant_K, air)
        opti.subject_to([r.power_heat_W >= 120e3, r.air.pressure_drop_Pa <= 1000.0,
                         r.coolant.pressure_drop_Pa <= 80e3, core.height_m() <= 0.8, core.width_m() <= 1.0])
        opti.minimize(core.get_mass())
        solution = opti.solve(verbose=False)
        self.assertGreater(solution(r.power_heat_W), 120e3 * (1 - 1e-6))
        self.assertLess(solution(r.air.pressure_drop_Pa), 1000.0 * (1 + 1e-6))
        self.assertTrue(5.0 < solution(core.get_mass()) < 60.0)


if __name__ == "__main__":
    unittest.main()
