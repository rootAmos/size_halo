"""Blade-element momentum rotor against full-scale JVX data (plan 047), on every 8th test point to keep it short.

Full-set numbers (examples/jvx_rotor_bemt.py, drag factor 0.964): hover FM RMS 0.040 on Table D-1 (0.018 for
C_T/sigma 0.04-0.14), airplane-mode efficiency RMS 0.025, max 0.059.
"""
import unittest

import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import BladeElementRotor
from examples.jvx_rotor_bemt import bemt_jvx_report, jvx_rotor


class JvxBemtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = bemt_jvx_report(drag_factor=BladeElementRotor().drag_factor, stride=8)

    def test_geometry_reproduces_test_solidity(self):
        self.assertAlmostEqual(float(jvx_rotor().solidity_thrust_weighted()), 0.1138, delta=0.001)

    def test_hover_figure_of_merit(self):
        self.assertLess(self.report.hover_fm_rms, 0.04)

    def test_airplane_mode_efficiency(self):
        self.assertLess(self.report.airplane_eta_rms, 0.035)
        self.assertLess(self.report.airplane_eta_max, 0.06)

    def test_most_points_trim(self):
        self.assertLessEqual(self.report.count_unsolved, 1)

    def test_figure_of_merit_rises_with_blade_loading(self):
        fm = self.report.hover_fm_predicted
        load = self.report.hover_blade_loading
        keep = onp.isfinite(fm) & (load < 0.10)
        order = onp.argsort(load[keep])
        self.assertTrue(onp.all(onp.diff(fm[keep][order]) > -0.01))


if __name__ == "__main__":
    unittest.main()
