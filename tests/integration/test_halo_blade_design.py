"""Two-point Halo blade design with the blade-element rotor (plan 047)."""
import unittest

import numpy as onp

from examples.halo_blade_design import (blade_loading_max, current_model_numbers, design_blade, evaluate_blade,
                                        fraction_stall_check, halo_points, halo_rotor, margin_stall_deg,
                                        reference_blade, stall_angles_deg, start_blade)


class HaloBladeDesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points = halo_points()
        cls.current = current_model_numbers(cls.points)
        cls.start = evaluate_blade(start_blade(), cls.points)
        cls.optimum = design_blade(cls.points, cls.start, cls.current)

    def test_operating_points_match_the_baseline(self):
        self.assertAlmostEqual(self.points.hover.thrust_N, 38004.4, delta=1.0)
        self.assertAlmostEqual(self.current["hover"]["figure_of_merit"], 0.7903, delta=1e-3)
        self.assertAlmostEqual(self.current["cruise"]["efficiency"], 0.8803, delta=1e-3)
        self.assertGreater(self.points.max_speed.thrust_N, self.points.cruise.thrust_N)

    def test_optimum_meets_hover_floor_and_limits(self):
        o = self.optimum
        self.assertGreaterEqual(float(o.hover.figure_of_merit), self.current["hover"]["figure_of_merit"] - 1e-6)
        self.assertLessEqual(float(o.hover.blade_loading), blade_loading_max + 1e-6)
        limit = stall_angles_deg(halo_rotor(self.start.blade), self.points.hover) - margin_stall_deg
        check = o.hover.fraction_radius >= fraction_stall_check
        self.assertTrue(onp.all(o.hover.angle_attack_deg[check] <= limit[check] + 1e-4))
        self.assertLess(float(o.max_speed.mach_tip_helical), 0.80)

    def test_optimum_beats_the_starting_blade_in_cruise(self):
        self.assertLess(float(self.optimum.cruise.shaft_power_W), float(self.start.cruise.shaft_power_W))
        self.assertTrue(0.85 < float(self.optimum.cruise.efficiency_propulsive) < 0.90)

    def test_cruise_trims_with_the_sizing_solidity_blade(self):
        """The XV-15-like blade at the sizing's solidity trims in cruise; its hover is checked in the study."""
        reference = evaluate_blade(reference_blade(), self.points)
        self.assertIsNotNone(reference.cruise)
        self.assertAlmostEqual(float(reference.cruise.efficiency_propulsive), self.current["cruise"]["efficiency"],
                               delta=0.02)


if __name__ == "__main__":
    unittest.main()
