"""Tier 14 (plan 019): tiltrotor trajectory model, collocation and the two Halo trajectory problems."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
import casadi as cas

from aircraft_closure.performance.flight_point import FlightCondition, acceleration_gravity_m_s2, build_flight_point
from aircraft_closure.trajectory.tiltrotor import (ConversionCorridor, TrajectoryGuess, TrajectoryLimits,
                                                   build_tiltrotor_trajectory)
from examples.trajectory_optimization import (altitude_band_transition_m, altitude_transition_m, halo_trajectory_case,
                                              ratio_transition_end_stall, solve_min_energy_transition,
                                              solve_min_time_climb, solve_prescribed_transition)

g_m_s2 = acceleration_gravity_m_s2
_cases = {}


def halo_case():
    """The sized Halo reference, sized once per test run."""
    if "halo" not in _cases:
        _cases["halo"] = halo_trajectory_case()
    return _cases["halo"]


def trapezoid(values, time_s):
    return np.concatenate([[0.0], np.cumsum(0.5 * (values[1:] + values[:-1]) * np.diff(time_s))])


def solve_flight_point(case, condition):
    opti = asb.Opti()
    point = build_flight_point(opti, case.model.aircraft, case.model.aerodynamics, condition, case.mass_kg, 0.0)
    opti.minimize(point.fuel_flow_kg_s * 100)
    solution = opti.solve(verbose=False)
    return point, solution


class TrajectoryModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.case = halo_case()
        cls.weight_N = cls.case.mass_kg * g_m_s2
        cls.cruise_condition = FlightCondition(mode="airplane", velocity_m_s=80.0, altitude_m=1000.0, label="cruise")
        cls.cruise, cls.cruise_solution = solve_flight_point(cls.case, cls.cruise_condition)
        cls.hover, cls.hover_solution = solve_flight_point(
            cls.case, FlightCondition(mode="hover", altitude_m=0.0, thrust_to_weight=1.0, label="hover"))

    def test_airplane_mode_identity_with_flight_point(self):
        """Thrust along the flight path (tilt = -alpha): the quasi-steady airplane point balances exactly."""
        s, p = self.cruise_solution, self.cruise
        alpha_deg = s.value(p.alpha_deg)
        forces = self.case.model.evaluate(80.0, 1000.0, alpha_deg, -alpha_deg,
                                          thrust_per_rotor_N=s.value(p.thrust_per_rotor_N),
                                          speed_rotor_rad_s=s.value(p.speed_rotor_rad_s))
        self.assertAlmostEqual(forces.force_x_wind_N / self.weight_N, 0.0, places=9)
        self.assertAlmostEqual((forces.force_z_wind_N + self.weight_N) / self.weight_N, 0.0, places=9)
        self.assertAlmostEqual(forces.rotor.shaft_power_W / s.value(p.power_shaft_rotor_W), 1.0, places=9)
        self.assertAlmostEqual(forces.power_electric_motors_W / s.value(p.power_electric_motors_W), 1.0, places=9)

    def test_hover_reproduces_flight_point_hover(self):
        """Tilt 90 deg at zero speed: thrust (with the full download) carries the weight; hover power recovered."""
        s, p = self.hover_solution, self.hover
        thrust_per_rotor_N = s.value(p.thrust_per_rotor_N)
        forces = self.case.model.evaluate(0.0, 0.0, 0.0, 90.0, thrust_per_rotor_N=thrust_per_rotor_N,
                                          speed_rotor_rad_s=s.value(p.speed_rotor_rad_s))
        self.assertAlmostEqual(forces.download_fraction, self.case.model.aerodynamics.download_fraction_hover, places=12)
        self.assertAlmostEqual((forces.force_z_wind_N + self.weight_N) / self.weight_N, 0.0, places=9)
        self.assertAlmostEqual(forces.force_x_wind_N / self.weight_N, 0.0, places=9)
        self.assertAlmostEqual(forces.lift_N, 0.0, places=9)
        self.assertAlmostEqual(forces.rotor.shaft_power_W / s.value(p.power_shaft_rotor_W), 1.0, places=5)
        self.assertAlmostEqual(forces.power_electric_motors_W / s.value(p.power_electric_motors_W), 1.0, places=5)

    def frozen_airplane_mode(self, thrust_along_flight_path):
        """Collocated level trajectory at the flight point's speed and altitude, nacelles frozen."""
        opti = asb.Opti()
        guess = TrajectoryGuess(duration_s=10.0, velocity_m_s=80.0, altitude_m=1000.0, tilt_deg=0.0,
                                thrust_per_rotor_N=self.cruise_solution.value(self.cruise.thrust_per_rotor_N))
        t = build_tiltrotor_trajectory(opti, self.case.model, mass_initial_kg=self.case.mass_kg, soc_initial=0.8,
                                       count_nodes=5, duration_bounds_s=(10.0, 10.0), guess=guess,
                                       limits=TrajectoryLimits(tilt_min_deg=-20.0))
        # Steady at every node (zero rates), from the flight point's speed and altitude; states then stay constant.
        opti.subject_to([t.tilt_deg == (-t.alpha_deg if thrust_along_flight_path else 0.0), t.acceleration_m_s2 == 0,
                         t.rate_gamma_rad_s == 0, t.velocity_m_s[0] == 80.0, t.altitude_m[0] == 1000.0,
                         t.gamma_rad[0] == 0])
        opti.minimize(t.energy_bus_J[-1] / 1e7)
        s = opti.solve(verbose=False)
        power_W = s.value(t.forces.power_electric_motors_W)
        np.testing.assert_allclose(s.value(t.gamma_rad), 0.0, atol=1e-9)
        np.testing.assert_allclose(s.value(t.velocity_m_s), 80.0, atol=1e-7)
        np.testing.assert_allclose(s.value(t.altitude_m), 1000.0, atol=1e-6)
        np.testing.assert_allclose(s.value(t.energy_bus_J)[-1], np.mean(power_W) * 10.0, rtol=1e-6)
        return power_W / self.cruise_solution.value(self.cruise.power_electric_motors_W)

    def test_frozen_trajectory_recovers_quasi_steady_airplane_power(self):
        """Thrust along the flight path (the flight point's assumption): the collocated trajectory, with its own
        trim and rotor-speed choice, recovers the quasi-steady power. The 10 s of fuel burn (0.02 % of mass) is the
        only difference."""
        np.testing.assert_allclose(self.frozen_airplane_mode(True), 1.0, atol=5e-4)

    def test_frozen_tilt_zero_differs_only_by_thrust_inclination(self):
        """Tilt 0 (thrust along the fuselage, alpha above the path): its lift component unloads the wing, so
        power is slightly lower than the flight point's, here by 1.4 %."""
        ratio = self.frozen_airplane_mode(False)
        self.assertTrue(np.all(ratio < 1.0))
        np.testing.assert_allclose(ratio, 1.0, atol=0.02)

    def test_thrust_direction_and_signs(self):
        model = self.case.model
        hover = model.evaluate(0.0, 0.0, 0.0, 90.0, thrust_per_rotor_N=3e4, speed_rotor_rad_s=50.0)
        airplane = model.evaluate(80.0, 0.0, 0.0, 0.0, thrust_per_rotor_N=3e3, speed_rotor_rad_s=40.0)
        tilted = model.evaluate(40.0, 0.0, 0.0, 45.0, thrust_per_rotor_N=1e4, speed_rotor_rad_s=45.0)
        self.assertLess(hover.force_z_wind_N, 0)              # z down: thrust up is negative
        self.assertGreater(airplane.force_x_wind_N + airplane.drag_N, 0)
        self.assertAlmostEqual(airplane.velocity_axial_m_s, 80.0, places=9)
        self.assertAlmostEqual(tilted.velocity_axial_m_s, 40.0 * np.cosd(45.0), places=9)
        self.assertAlmostEqual(airplane.download_fraction, 0.0, places=12)
        self.assertLess(tilted.download_fraction, hover.download_fraction)

    def test_axial_climb_power_rises_with_inflow(self):
        """Physical trend: at fixed thrust and rotor speed, power rises with axial (climb) velocity."""
        powers = [self.case.model.evaluate(v, 0.0, 0.0, 0.0, thrust_per_rotor_N=1e4, speed_rotor_rad_s=40.0)
                  .rotor.shaft_power_W for v in (10.0, 40.0, 80.0)]
        self.assertTrue(powers[0] < powers[1] < powers[2])

    def test_symbolic_and_vectorized(self):
        opti = asb.Opti()
        n = 4
        velocity_m_s = opti.variable(init_guess=np.linspace(5, 80, n))
        forces = self.case.model.evaluate(velocity_m_s, 500.0, opti.variable(init_guess=4.0 * np.ones(n)),
                                          opti.variable(init_guess=np.linspace(90, 0, n)),
                                          thrust_per_rotor_N=opti.variable(init_guess=2e4 * np.ones(n)),
                                          speed_rotor_rad_s=opti.variable(init_guess=45.0 * np.ones(n)))
        for expression in (forces.force_x_wind_N, forces.force_z_wind_N, forces.power_electric_motors_W):
            self.assertIsInstance(expression, cas.MX)
            self.assertEqual(expression.shape, (n, 1))
            self.assertEqual(cas.jacobian(expression, opti.x).shape, (n, opti.nx))
        numeric = self.case.model.evaluate(np.linspace(5, 80, n), 500.0, 4.0 * np.ones(n), np.linspace(90, 0, n),
                                           thrust_per_rotor_N=2e4 * np.ones(n), speed_rotor_rad_s=45.0 * np.ones(n))
        self.assertEqual(np.asarray(numeric.power_electric_motors_W).shape, (n,))
        self.assertTrue(np.all(np.isfinite(numeric.power_electric_motors_W)))

    def test_corridor_shape(self):
        corridor = ConversionCorridor(velocity_stall_m_s=50.0)
        self.assertAlmostEqual(corridor.velocity_min_m_s(0.0), 50.0)
        self.assertAlmostEqual(corridor.velocity_min_m_s(90.0), 0.0, places=12)
        self.assertAlmostEqual(corridor.velocity_max_m_s(90.0), 115 * u.knot)
        self.assertAlmostEqual(corridor.velocity_max_m_s(0.0), 180 * u.knot)
        self.assertLess(corridor.velocity_max_m_s(60.0), corridor.velocity_max_m_s(30.0))


class CollocationTests(unittest.TestCase):
    def test_constant_acceleration_point_mass(self):
        """AeroSandbox speed-gamma dynamics with the builder's free-final-time grid: exact for constant acceleration."""
        opti = asb.Opti()
        n, mass_kg, acceleration_m_s2 = 11, 1000.0, 2.0
        duration_s = opti.variable(init_guess=5.0, lower_bound=1.0)
        time_s = np.linspace(0, duration_s, n)
        speed_m_s = opti.variable(init_guess=np.linspace(10, 30, n))
        dynamics = asb.DynamicsPointMass2DSpeedGamma(mass_props=asb.MassProperties(mass=mass_kg),
                                                     x_e=opti.variable(init_guess=np.zeros(n)),
                                                     z_e=opti.variable(init_guess=np.zeros(n)), speed=speed_m_s,
                                                     gamma=opti.variable(init_guess=np.zeros(n)))
        dynamics.add_force(Fx=mass_kg * acceleration_m_s2, Fz=-mass_kg * g_m_s2, axes="wind")
        dynamics.add_gravity_force(g=g_m_s2)
        dynamics.constrain_derivatives(opti, time_s)
        opti.subject_to([dynamics.x_e[0] == 0, dynamics.z_e[0] == 0, dynamics.gamma[0] == 0, speed_m_s[0] == 10.0,
                         speed_m_s[-1] == 30.0])
        s = opti.solve(verbose=False)
        t_s = s.value(time_s)
        self.assertAlmostEqual(s.value(duration_s), 10.0, places=8)
        np.testing.assert_allclose(s.value(dynamics.x_e), 10.0 * t_s + 0.5 * acceleration_m_s2 * t_s**2, atol=1e-6)
        np.testing.assert_allclose(s.value(dynamics.z_e), 0.0, atol=1e-8)


class HaloTrajectoryProblemTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.case = halo_case()
        cls.transition = solve_min_energy_transition(cls.case)
        cls.prescribed = solve_prescribed_transition(cls.case)
        cls.climb = solve_min_time_climb(cls.case)
        cls.limits = TrajectoryLimits()

    def assert_operating_limits(self, r):
        tolerance = 1e-6
        self.assertTrue(np.all(np.abs(r.tilt_rate_deg_s) <= self.limits.tilt_rate_max_deg_s + tolerance))
        self.assertTrue(np.all(r.velocity_m_s >= r.velocity_corridor_min_m_s - tolerance))
        self.assertTrue(np.all(r.velocity_m_s <= r.velocity_corridor_max_m_s + tolerance))
        self.assertTrue(np.all(r.alpha_deg <= r.alpha_stall_deg + tolerance))
        self.assertTrue(np.all(r.power_bus_W <= r.power_available_bus_W * (1 + tolerance)))
        self.assertTrue(np.all(r.power_shaft_generator_W <= r.power_available_turboshaft_W * (1 + tolerance)))
        self.assertTrue(np.all(r.soc >= self.limits.soc_min - tolerance))
        self.assertTrue(np.all(r.power_battery_W >= -1.0))
        self.assertTrue(np.all(r.blade_loading <= 0.14 + tolerance))
        self.assertTrue(np.all(r.advance_ratio <= 0.60 + tolerance))

    def test_transition_meets_its_constraints(self):
        r = self.transition
        self.assert_operating_limits(r)
        self.assertAlmostEqual(r.tilt_deg[0], 90.0, places=6)
        self.assertAlmostEqual(r.tilt_deg[-1], 0.0, places=6)
        self.assertAlmostEqual(r.velocity_m_s[-1] / (ratio_transition_end_stall * self.case.corridor.velocity_stall_m_s),
                               1.0, places=6)
        self.assertTrue(np.all(np.abs(r.altitude_m - altitude_transition_m) <= altitude_band_transition_m + 1e-4))
        self.assertAlmostEqual(r.acceleration_m_s2[-1], 0.0, places=6)
        # The nacelle rate bound sets a floor on the conversion time.
        self.assertGreaterEqual(r.duration_s, 90.0 / self.limits.tilt_rate_max_deg_s - 1e-6)

    def test_optimized_transition_beats_the_prescribed_profile(self):
        self.assert_operating_limits(self.prescribed)
        self.assertLess(self.transition.energy_bus_J[-1], 0.7 * self.prescribed.energy_bus_J[-1])

    def test_energy_and_kinematics_are_consistent(self):
        """States agree with independent trapezoidal integrals of the model's rates (signs and axes)."""
        for r in (self.transition, self.climb):
            np.testing.assert_allclose(r.energy_bus_J, trapezoid(r.power_bus_W, r.time_s), rtol=1e-6, atol=1.0)
            np.testing.assert_allclose(r.altitude_m - r.altitude_m[0],
                                       trapezoid(r.velocity_m_s * np.sind(r.gamma_deg), r.time_s), atol=1e-5)
            np.testing.assert_allclose(r.x_m, trapezoid(r.velocity_m_s * np.cosd(r.gamma_deg), r.time_s), atol=1e-5)

    def test_time_to_climb(self):
        r = self.climb
        self.assert_operating_limits(r)
        self.assertAlmostEqual(r.altitude_m[-1], 10000 * u.foot, places=4)
        self.assertAlmostEqual(r.velocity_m_s[-1], self.case.sizing.velocity_cruise_m_s, places=6)
        self.assertAlmostEqual(r.tilt_deg[-1], 0.0, places=6)
        # Faster than the sizing mission's prescribed 6 m/s quasi-steady climb.
        self.assertLess(r.duration_s, 10000 * u.foot / 6.0)
        self.assertTrue(np.all(r.altitude_m >= -1e-6))


if __name__ == "__main__":
    unittest.main()
