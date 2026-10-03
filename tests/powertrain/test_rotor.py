import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor, profile_integral

sea_level = asb.Atmosphere(altitude=0)
rho = float(sea_level.density())
rotor = MomentumProfileRotor(area_disk_m2=45.6, solidity=0.11)


def coefficients(result, speed_rad_s, model=rotor):
    tip = speed_rad_s * float(model.radius_m())
    scale = rho * model.area_disk_m2 * tip**2
    return float(result.thrust_N) / scale, float(result.shaft_power_W) / (scale * tip)


class IdentityTests(unittest.TestCase):
    def test_profile_integral_closed_form_and_hover_limit(self):
        x = np.linspace(0, 1, 20001)
        for lam in (0.05, 0.3, 0.6, 1.0):
            numeric = np.trapezoid(x**2 * np.sqrt(x**2 + lam**2), x)
            self.assertAlmostEqual(float(profile_integral(lam)), numeric, places=7)
        self.assertAlmostEqual(float(profile_integral(1e-6)), 0.25, places=9)

    def test_hover_power_identity(self):
        result = rotor.evaluate(0.0, sea_level, thrust_N=50000.0, speed_rad_s=55.0)
        ct, cp = coefficients(result, 55.0)
        d0, d1, d2 = rotor.drag_polar
        x = ct / rotor.solidity
        expected = rotor.kappa_hover * ct**1.5 / 2**0.5 + rotor.solidity / 8 * (d0 + d1 * x + d2 * x**2)
        self.assertAlmostEqual(cp, expected, places=12)
        self.assertAlmostEqual(float(result.blade_loading), x, places=12)

    def test_airplane_power_identity(self):
        cruise = rotor.in_airplane_mode()
        velocity, speed = 90.0, 40.0
        result = cruise.evaluate(velocity, sea_level, thrust_N=8000.0, speed_rad_s=speed)
        ct, cp = coefficients(result, speed, cruise)
        lam = velocity / (speed * float(cruise.radius_m()))
        inflow = -lam / 2 + np.sqrt(lam**2 / 4 + ct / 2)
        d0, d1, d2 = cruise.drag_polar
        a, b = cruise.drag_increment_airplane
        x = ct / cruise.solidity / (1 + 3 * lam**2)
        cd = d0 + d1 * x + d2 * x**2 + a + b * lam**2
        expected = ct * lam + cruise.kappa_airplane * ct * inflow + cruise.solidity / 2 * cd * profile_integral(lam)
        self.assertAlmostEqual(cp, float(expected), places=12)
        self.assertAlmostEqual(float(result.advance_ratio), lam, places=12)

    def test_helical_tip_mach(self):
        result = rotor.in_airplane_mode().evaluate(100.0, sea_level, thrust_N=5000.0, speed_rad_s=50.0)
        tip = 50.0 * float(rotor.radius_m())
        self.assertAlmostEqual(float(result.mach_tip_helical), (tip**2 + 100.0**2)**0.5 / float(sea_level.speed_of_sound()),
                               places=12)


class LimitAndTrendTests(unittest.TestCase):
    def test_zero_thrust_hover_is_profile_power_only(self):
        result = rotor.evaluate(0.0, sea_level, thrust_N=0.0, speed_rad_s=55.0)
        _, cp = coefficients(result, 55.0)
        self.assertAlmostEqual(cp, rotor.solidity / 8 * rotor.drag_polar[0], places=12)

    def test_frictionless_rotor_reaches_ideal_momentum(self):
        ideal = MomentumProfileRotor(area_disk_m2=45.6, solidity=0.11, kappa_hover=1.0, drag_polar=(0.0, 0.0, 0.0))
        result = ideal.evaluate(0.0, sea_level, thrust_N=50000.0, speed_rad_s=55.0)
        ideal_power = 50000.0**1.5 / (2 * rho * 45.6)**0.5
        self.assertAlmostEqual(float(result.shaft_power_W) / ideal_power, 1.0, places=10)

    def test_slower_rotor_saves_profile_power_at_light_load(self):
        fast = rotor.evaluate(0.0, sea_level, thrust_N=10000.0, speed_rad_s=60.0).shaft_power_W
        slow = rotor.evaluate(0.0, sea_level, thrust_N=10000.0, speed_rad_s=45.0).shaft_power_W
        self.assertLess(float(slow), float(fast))

    def test_cruise_power_falls_with_rotor_speed_to_the_validity_bound(self):
        """Documented limitation (plan 016): no interior cruise optimum inside the calibrated range, so the
        lambda <= 0.60 bound sets cruise rotor speed."""
        cruise = rotor.in_airplane_mode()
        slowest = 90.0 / (cruise.advance_ratio_max * float(cruise.radius_m()))
        speeds = np.linspace(slowest, 70.0, 30)
        powers = [float(cruise.evaluate(90.0, sea_level, thrust_N=9000.0, speed_rad_s=s).shaft_power_W) for s in speeds]
        self.assertTrue(all(b > a for a, b in zip(powers, powers[1:])))

    def test_rotor_speed_does_not_matter_to_actuator_disk(self):
        from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
        disk = ActuatorDiskPropulsor()
        a = disk.evaluate(0.0, sea_level, thrust_N=5000.0, speed_rad_s=30.0).shaft_power_W
        b = disk.evaluate(0.0, sea_level, thrust_N=5000.0).shaft_power_W
        self.assertEqual(a, b)


class SymbolicTests(unittest.TestCase):
    def test_opti_variables(self):
        opti = asb.Opti()
        speed = opti.variable(init_guess=50.0)
        thrust = opti.variable(init_guess=8000.0)
        result = rotor.in_airplane_mode().evaluate(90.0, sea_level, thrust_N=thrust, speed_rad_s=speed)
        self.assertIsInstance(result.shaft_power_W, cas.MX)
        opti.subject_to([speed == 45.0, thrust == 8000.0])
        solution = opti.solve(verbose=False)
        numeric = rotor.in_airplane_mode().evaluate(90.0, sea_level, thrust_N=8000.0, speed_rad_s=45.0).shaft_power_W
        self.assertAlmostEqual(float(solution.value(result.shaft_power_W)), float(numeric), places=6)


if __name__ == "__main__":
    unittest.main()
