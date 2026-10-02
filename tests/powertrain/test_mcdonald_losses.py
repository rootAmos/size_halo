import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import (McDonaldMotorLossModel, Motor, SimpleMotorLossModel,
                                                          rubber_machine)


def efficiency(model, speed_rad_s, torque_Nm):
    power_shaft_W = speed_rad_s * torque_Nm
    return power_shaft_W / (power_shaft_W + model.evaluate(speed_rad_s, torque_Nm, 800))


class McDonaldLossTests(unittest.TestCase):
    """McDonald, AIAA 2015-1676, eqs. 1-4."""

    def test_peak_efficiency_value_and_location(self):
        model = McDonaldMotorLossModel(500.0, 300.0, 0.95, 0.5)
        self.assertAlmostEqual(efficiency(model, 500.0, 300.0), 0.95, places=12)
        step = 1e-4
        for d_speed, d_torque in ((step, 0), (-step, 0), (0, step), (0, -step)):
            self.assertLess(efficiency(model, 500.0 * (1 + d_speed), 300.0 * (1 + d_torque)), 0.95)

    def test_peak_is_stationary(self):
        model = McDonaldMotorLossModel(500.0, 300.0, 0.95, 0.5)
        h = 1e-3
        d_speed = (efficiency(model, 500 + h, 300) - efficiency(model, 500 - h, 300)) / (2 * h)
        d_torque = (efficiency(model, 500, 300 + h) - efficiency(model, 500, 300 - h)) / (2 * h)
        self.assertLess(abs(d_speed), 1e-9)
        self.assertLess(abs(d_torque), 1e-9)

    def test_coefficients_closed_form(self):
        c = McDonaldMotorLossModel(500.0, 300.0, 0.95, 0.5).coefficients()
        loss_ratio = 0.05 / 0.95
        self.assertAlmostEqual(c.constant_W, 0.5 * 500 * 300 / 6 * loss_ratio)
        self.assertAlmostEqual(c.linear_W_s_rad, 300 * loss_ratio * (1 - 0.5) / 4)
        self.assertAlmostEqual(c.torque_W_Nm2, 500 * loss_ratio / (2 * 300))

    def test_zero_parasite_ratio_removes_constant_loss(self):
        c = McDonaldMotorLossModel(parasite_loss_ratio=0.0).coefficients()
        self.assertEqual(c.constant_W, 0)
        self.assertAlmostEqual(efficiency(McDonaldMotorLossModel(parasite_loss_ratio=0.0), 400, 200), 0.96)

    def test_coefficients_non_negative_on_valid_domain(self):
        for k0 in np.linspace(0, 1, 11):
            c = McDonaldMotorLossModel(parasite_loss_ratio=k0).coefficients()
            self.assertTrue(min(c.constant_W, c.linear_W_s_rad, c.cubic_W_s3_rad3, c.torque_W_Nm2) >= 0)

    def test_rubber_torque_scaling(self):
        base = McDonaldMotorLossModel(400.0, 200.0)
        doubled = McDonaldMotorLossModel(400.0, 400.0)
        for speed_rad_s, torque_Nm in ((100, 50), (400, 200), (800, 400)):
            self.assertAlmostEqual(doubled.evaluate(speed_rad_s, 2 * torque_Nm, 800),
                                   2 * base.evaluate(speed_rad_s, torque_Nm, 800), places=9)

    def test_rubber_machine_ratings(self):
        motor = rubber_machine(Motor, 500.0, 300.0, 0.95, 0.5, torque_ratio=2, power_ratio=2, speed_ratio=2)
        self.assertEqual((motor.max_torque_Nm, motor.power_rated_W, motor.max_speed_rad_s), (600.0, 300000.0, 1000.0))
        generator = rubber_machine(Generator, 400.0, 200.0, specific_power_W_kg=5000.0)
        self.assertIsInstance(generator, Generator)
        self.assertEqual(generator.specific_power_W_kg, 5000.0)

    def test_efficiency_trends(self):
        model = McDonaldMotorLossModel()
        self.assertLess(efficiency(model, 400, 20), efficiency(model, 400, 200))
        self.assertLess(efficiency(model, 1000, 200), efficiency(model, 400, 200))

    def test_simple_model_still_available(self):
        self.assertEqual(Motor(loss_model=SimpleMotorLossModel()).evaluate(400, 200, 800).power_loss_W, 960)

    def test_symbolic_parameters(self):
        opti = asb.Opti()
        torque_peak_Nm = opti.variable(init_guess=150, lower_bound=1)
        loss_W = McDonaldMotorLossModel(torque_peak_efficiency_Nm=torque_peak_Nm).evaluate(400, 200, 800)
        self.assertIsInstance(loss_W, cas.MX)
        opti.minimize(loss_W)
        solution = opti.solve(verbose=False)
        # Loss at a fixed point is minimized when that point is the peak-efficiency point.
        self.assertAlmostEqual(float(solution.value(torque_peak_Nm)), 200.0, delta=1e-3)


if __name__ == "__main__":
    unittest.main()
