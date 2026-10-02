import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.controls.stability import (DirectionalStability, LongitudinalStability, flap_effectiveness,
                                                 failed_propulsor_yaw_moment_Nm)
from examples.aircraft_mass_closure import build_reference_aircraft

aircraft = build_reference_aircraft(3.23)
aero = SimpleAerodynamics()
longitudinal = LongitudinalStability()
directional = DirectionalStability()
x_cg_m = 3.52


class FlapEffectivenessTests(unittest.TestCase):
    def test_limits_and_monotonicity(self):
        self.assertAlmostEqual(float(flap_effectiveness(0.0, correction=1.0)), 0.0, places=12)
        self.assertAlmostEqual(float(flap_effectiveness(1.0, correction=1.0)), 1.0, places=12)
        values = [float(flap_effectiveness(c)) for c in (0.1, 0.2, 0.3, 0.4)]
        self.assertTrue(all(a < b for a, b in zip(values, values[1:])))
        self.assertAlmostEqual(float(flap_effectiveness(0.3)), 0.8 * float(flap_effectiveness(0.3, 1.0)), places=12)


class LongitudinalTests(unittest.TestCase):
    def test_tail_moves_neutral_point_aft(self):
        small = replace(aircraft, horizontal_tail=replace(aircraft.horizontal_tail, area_m2=0.5))
        self.assertLess(float(longitudinal.neutral_point_x_m(small, aero, 60, 1000)),
                        float(longitudinal.neutral_point_x_m(aircraft, aero, 60, 1000)))

    def test_fuselage_term_is_destabilizing(self):
        without = replace(longitudinal, fuselage_pitch_factor_per_deg=0.0)
        self.assertGreater(float(without.neutral_point_x_m(aircraft, aero, 60, 1000)),
                           float(longitudinal.neutral_point_x_m(aircraft, aero, 60, 1000)))

    def test_cm_alpha_equals_minus_static_margin_times_lift_slope(self):
        h = 1e-4
        cm_plus = float(longitudinal.evaluate(aircraft, aero, x_cg_m, 60, 1000, 3 + h, 0).cm)
        cm_minus = float(longitudinal.evaluate(aircraft, aero, x_cg_m, 60, 1000, 3 - h, 0).cm)
        cm_alpha_per_rad = (cm_plus - cm_minus) / np.radians(2 * h)
        slope_total = float(longitudinal.lift_curve_slope_total_per_rad(aircraft, aero, 60, 1000))
        static_margin = float(longitudinal.static_margin(aircraft, aero, x_cg_m, 60, 1000))
        self.assertAlmostEqual(cm_alpha_per_rad, -static_margin * slope_total, places=6)

    def test_cg_at_neutral_point_is_neutrally_stable(self):
        x_np_m = float(longitudinal.neutral_point_x_m(aircraft, aero, 60, 1000))
        self.assertAlmostEqual(float(longitudinal.static_margin(aircraft, aero, x_np_m, 60, 1000)), 0.0, places=12)

    def test_trim_and_trim_drag(self):
        opti = asb.Opti()
        alpha_deg = opti.variable(init_guess=3)
        elevator_deg = opti.variable(init_guess=0)
        result = longitudinal.evaluate(aircraft, aero, x_cg_m, 60, 1000, alpha_deg, elevator_deg)
        self.assertIsInstance(result.cm, cas.MX)
        opti.subject_to([result.cm == 0, result.lift_N == 15000])
        solution = opti.solve(verbose=False)
        self.assertLess(abs(float(solution.value(result.cm))), 1e-9)
        self.assertGreater(float(solution.value(result.trim_drag.cd)), 0)
        # Nose-down wing moment and aft CG of the ac: the tail must push down (negative lift).
        self.assertLess(float(solution.value(result.cl_tail)), 0)
        self.assertLess(float(solution.value(elevator_deg)), 0)

    def test_elevator_raises_tail_lift(self):
        up = longitudinal.evaluate(aircraft, aero, x_cg_m, 60, 1000, 3, 5)
        down = longitudinal.evaluate(aircraft, aero, x_cg_m, 60, 1000, 3, -5)
        self.assertGreater(float(up.cl_tail), float(down.cl_tail))
        self.assertLess(float(up.cm), float(down.cm))


class DirectionalTests(unittest.TestCase):
    def test_cn_beta_grows_with_fin_area_and_arm(self):
        big = replace(aircraft, vertical_tail=replace(aircraft.vertical_tail, area_m2=2.4))
        self.assertGreater(float(directional.cn_beta_per_rad(big, aero, x_cg_m, 60, 1000)),
                           float(directional.cn_beta_per_rad(aircraft, aero, x_cg_m, 60, 1000)))
        self.assertGreater(float(directional.cn_beta_per_rad(aircraft, aero, x_cg_m - 0.5, 60, 1000)),
                           float(directional.cn_beta_per_rad(aircraft, aero, x_cg_m, 60, 1000)))

    def test_fuselage_is_directionally_destabilizing(self):
        tiny = replace(aircraft, vertical_tail=replace(aircraft.vertical_tail, area_m2=1e-6))
        self.assertLess(float(directional.cn_beta_per_rad(tiny, aero, x_cg_m, 60, 1000)), 0)

    def test_rudder_scaling(self):
        base = float(directional.rudder_for_yaw_moment_deg(aircraft, aero, x_cg_m, 45, 0, 2000))
        self.assertAlmostEqual(float(directional.rudder_for_yaw_moment_deg(aircraft, aero, x_cg_m, 45, 0, 4000)),
                               2 * base, places=9)
        self.assertAlmostEqual(float(directional.rudder_for_yaw_moment_deg(aircraft, aero, x_cg_m, 90, 0, 2000)),
                               base / 4, delta=base * 0.01)
        bigger = replace(aircraft, vertical_tail=replace(aircraft.vertical_tail, area_m2=3.2))
        self.assertLess(float(directional.rudder_for_yaw_moment_deg(bigger, aero, x_cg_m, 45, 0, 2000)), base)

    def test_failed_propulsor_moment(self):
        self.assertEqual(failed_propulsor_yaw_moment_Nm(1000, 5), 5000)
        self.assertEqual(failed_propulsor_yaw_moment_Nm(1000, 5, drag_failed_propulsor_N=200), 6000)


if __name__ == "__main__":
    unittest.main()
