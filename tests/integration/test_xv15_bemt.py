"""Blade-element momentum rotor on the full-scale XV-15 rotor (plan 052), a few points of the curves in
examples/proprotor_bemt_curves.py (full set: hover FM RMS 0.030 for C_T/sigma <= 0.13; airplane-mode efficiency
0.07 below the 1971 data on average).
"""
import unittest

import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import AirState, OperatingPoint, solve_point_stepped
from examples.proprotor_bemt_curves import (_rows, radius_xv15_m, solidity_xv15, xv15_conditions, xv15_rotor)


class Xv15BemtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rotor = xv15_rotor()
        cls.hover, cls.airplane = xv15_conditions()

    def test_data_files(self):
        self.assertEqual(len(_rows("xv15_hover.csv")), 160)
        self.assertEqual(len(_rows("xv15_airplane.csv")), 46)

    def test_geometry_reproduces_test_solidity(self):
        self.assertAlmostEqual(float(self.rotor.solidity_thrust_weighted()), solidity_xv15, delta=0.001)

    def test_hover_figure_of_merit_at_mid_loading(self):
        condition = self.hover[1]                  # Mtip 0.69, 122 points
        keep = onp.abs(condition.measured_x - 0.10) < 0.01
        blade_loading = 0.10
        thrust_N = blade_loading * solidity_xv15 * condition.density_kg_m3 * onp.pi * radius_xv15_m**2 \
            * condition.speed_tip_m_s**2
        point = OperatingPoint("hover", 0.0, 0.0, condition.speed_tip_m_s / radius_xv15_m, thrust_N,
                               air=AirState(condition.density_kg_m3, condition.speed_sound_m_s))
        result, _ = solve_point_stepped(self.rotor, point)
        self.assertLess(abs(float(result.figure_of_merit) - float(onp.mean(condition.measured_y[keep]))), 0.04)

    def test_airplane_mode_trims_below_momentum_limit(self):
        condition = next(c for c in self.airplane if c.label.startswith("160 kt, 600"))
        thrust_N = float(condition.thrust_grid_N[-2])        # near the highest measured thrust
        point = OperatingPoint("airplane", condition.axial_velocity_m_s, 0.0, condition.speed_tip_m_s / radius_xv15_m,
                               thrust_N, air=AirState(condition.density_kg_m3, condition.speed_sound_m_s))
        result, _ = solve_point_stepped(self.rotor, point)
        loading = thrust_N / (0.5 * condition.density_kg_m3 * condition.axial_velocity_m_s**2 * onp.pi * radius_xv15_m**2)
        self.assertGreater(float(result.efficiency_propulsive), 0.75)
        self.assertLess(float(result.efficiency_propulsive), 2 / (1 + onp.sqrt(1 + loading)))


if __name__ == "__main__":
    unittest.main()
