import os
import tempfile
import unittest
from pathlib import Path

import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.powertrain.components.turboshaft import (deck_1120hp_part_power_model, deck_1120hp_power_fraction,
                                                               deck_1120hp_sfc_ratio)
from aircraft_closure.powertrain.decks import fit_cubic_part_power, load_gasp_turboshaft_deck, part_power_curve
from examples.xv15_performance import part_power_sfc_ratios

raw_deck_path = Path(__file__).resolve().parents[2] / "data" / "engines" / "turboshaft_1120hp.csv"

synthetic_deck = """# created for tests
# sls_horsepower: 100.0
# t4max: 50.0

Mach Number (input), Altitude (ft, input),   Throttle (input), Shaft Power Corrected (hp, output), Tailpipe Thrust (lbf, output), Fuel Flow (lb/h, output)
0.0, 0.0, 20.0, 20.0, 1.0, 30.0
0.0, 0.0, 50.0, 100.0, 2.0, 60.0
0.0, 1000.0, 20.0, 25.0, 1.0, 40.0
0.0, 1000.0, 50.0, 100.0, 2.0, 70.0
"""


class DeckLoaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        handle, cls.path = tempfile.mkstemp(suffix=".csv")
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(synthetic_deck)
        cls.deck = load_gasp_turboshaft_deck(cls.path)

    @classmethod
    def tearDownClass(cls):
        os.remove(cls.path)

    def test_metadata_and_si_conversion(self):
        self.assertEqual(self.deck.metadata["sls_horsepower"], "100.0")
        self.assertEqual(len(self.deck.mach), 4)
        self.assertAlmostEqual(self.deck.altitude_m[2], 1000 * u.foot, places=12)
        self.assertAlmostEqual(self.deck.power_shaft_corrected_W[1], 100 * u.hp, places=9)
        self.assertAlmostEqual(self.deck.fuel_flow_corrected_kg_s[1], 60 * u.lbm / 3600, places=12)

    def test_part_power_curve_is_median_of_normalised_rows(self):
        # Row 1: power fraction 0.2 -> sfc ratio (30/20)/(60/100) = 2.5; row 2: 0.25 -> (40/25)/(70/100).
        curve = part_power_curve(self.deck, [0.25, 1.0])
        row_1 = np.interp(0.25, [0.2, 1.0], [2.5, 1.0])
        row_2 = (40 / 25) / (70 / 100)
        self.assertAlmostEqual(curve.sfc_ratio_median[0], (row_1 + row_2) / 2, places=12)
        self.assertAlmostEqual(curve.sfc_ratio_min[0], min(row_1, row_2), places=12)
        self.assertAlmostEqual(curve.sfc_ratio_median[1], 1.0, places=12)

    def test_cubic_fit_recovers_a_cubic(self):
        fractions = np.linspace(0.1, 1.0, 10)
        a, b, c = 0.3, 0.2, 0.8
        x = fractions - 1
        sfc_ratio = 1 / (1 + a * x + b * x**2 + c * x**3)
        fitted = fit_cubic_part_power(fractions, sfc_ratio)
        for value, expected in zip(fitted, (a, b, c)):
            self.assertAlmostEqual(value, expected, places=9)

    def test_embedded_cubic_is_the_fit_of_the_embedded_table(self):
        fitted = fit_cubic_part_power(deck_1120hp_power_fraction, deck_1120hp_sfc_ratio)
        for value, expected in zip(fitted, deck_1120hp_part_power_model().coefficients):
            self.assertAlmostEqual(value, expected, places=6)

    def test_deck_reproduces_xv15_part_power_sfc(self):
        """Reference operating points: LTC1K-4K ratings (contingency = maximum), within 3 %."""
        for throttle, published, model in part_power_sfc_ratios(part_power_model=deck_1120hp_part_power_model()):
            self.assertLess(abs(model / published - 1), 0.03, msg=f"throttle {throttle:.3f}")

    @unittest.skipUnless(raw_deck_path.exists(), "raw user deck not present (not committed, plan 014)")
    def test_embedded_table_rederives_from_raw_deck(self):
        curve = part_power_curve(load_gasp_turboshaft_deck(raw_deck_path), list(deck_1120hp_power_fraction))
        for value, expected in zip(curve.sfc_ratio_median, deck_1120hp_sfc_ratio):
            self.assertAlmostEqual(value, expected, places=4)


if __name__ == "__main__":
    unittest.main()
