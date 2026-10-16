"""Plan 068: battery-only tiltrotor trajectory and the transition-climb segment on the notional aircraft."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.trajectory.segments import (energy_fixed_power_rule_J, solve_steady_level_power,
                                                  solve_transition_climb)
from aircraft_closure.trajectory.tiltrotor import TrajectoryLimits
from examples.all_electric_transition_climb import build_notional_trajectory_model

mass_kg = 1600.0
soc_initial = 0.95
altitude_final_m = 500.0
velocity_final_m_s = 70.0
count_nodes = 21


def trapezoid(values, time_s):
    return np.concatenate([[0.0], np.cumsum(0.5 * (values[1:] + values[:-1]) * np.diff(time_s))])


class BatteryOnlyModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_notional_trajectory_model()

    def test_topology_is_battery_only_and_direct_drive(self):
        self.assertTrue(self.model.is_battery_only())
        self.assertFalse(self.model.has_instance("gearbox"))
        self.assertEqual(self.model.count("motor"), 3 * self.model.count("propulsor"))

    def test_supply_is_the_battery_alone(self):
        supply = self.model.evaluate_supply(0.0, current_battery_A=200.0, torque_generator_Nm=None, soc=0.9)
        self.assertIsNone(supply.generator)
        self.assertIsNone(supply.engine)
        self.assertEqual(supply.fuel_flow_kg_s, 0.0)
        self.assertEqual(supply.power_shaft_generator_W, 0.0)
        self.assertAlmostEqual(supply.power_bus_W, supply.battery.power_electric_W, places=9)

    def test_direct_drive_motor_speed_and_channel_torque(self):
        """No gearbox: motor speed is rotor speed; the three channels share the rotor torque equally."""
        forces = self.model.evaluate(0.0, 0.0, 0.0, 90.0, thrust_per_rotor_N=8500.0, speed_rotor_rad_s=88.0)
        self.assertAlmostEqual(forces.speed_motor_rad_s, 88.0, places=12)
        self.assertAlmostEqual(3 * forces.torque_motor_Nm * 88.0 / forces.rotor.shaft_power_W, 1.0, places=12)

    def test_symbolic_compatibility(self):
        opti = asb.Opti()
        n = 4
        current_A = opti.variable(init_guess=100.0 * np.ones(n))
        soc = opti.variable(init_guess=0.9 * np.ones(n))
        supply = self.model.evaluate_supply(opti.variable(init_guess=np.zeros(n)), current_battery_A=current_A,
                                            torque_generator_Nm=None, soc=soc)
        forces = self.model.evaluate(opti.variable(init_guess=np.linspace(5, 70, n)), 300.0,
                                     opti.variable(init_guess=4.0 * np.ones(n)),
                                     opti.variable(init_guess=np.linspace(90, 0, n)),
                                     thrust_per_rotor_N=opti.variable(init_guess=5e3 * np.ones(n)),
                                     speed_rotor_rad_s=opti.variable(init_guess=70.0 * np.ones(n)),
                                     voltage_bus_V=supply.battery.voltage_V)
        for expression in (supply.power_bus_W, supply.fuel_flow_kg_s, forces.power_electric_motors_W,
                           forces.force_z_wind_N):
            self.assertIsInstance(expression, cas.MX)
            self.assertEqual(expression.shape, (n, 1))
            self.assertEqual(cas.jacobian(expression, opti.x).shape, (n, opti.nx))


class TransitionClimbTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_notional_trajectory_model()
        kwargs = dict(mass_kg=mass_kg, soc_initial=soc_initial, altitude_final_m=altitude_final_m,
                      velocity_final_m_s=velocity_final_m_s, count_nodes=count_nodes)
        cls.energy = solve_transition_climb(cls.model, **kwargs)
        cls.time = solve_transition_climb(cls.model, objective="time", **kwargs)
        cls.battery = cls.model.instance("battery")

    def test_boundary_conditions_and_limits(self):
        limits = TrajectoryLimits()
        for r in (self.energy, self.time):
            self.assertAlmostEqual(r.altitude_m[0], 0.0, places=6)
            self.assertAlmostEqual(r.altitude_m[-1], altitude_final_m, places=4)
            self.assertAlmostEqual(r.velocity_m_s[-1], velocity_final_m_s, places=6)
            self.assertAlmostEqual(r.tilt_deg[0], 90.0, places=6)
            self.assertAlmostEqual(r.tilt_deg[-1], 0.0, places=6)
            self.assertTrue(np.all(r.altitude_m >= -1e-6))
            self.assertTrue(np.all(r.soc >= limits.soc_min - 1e-9))
            self.assertTrue(np.all(r.power_battery_W <= self.battery.max_discharge_power_W * (1 + 1e-6)))
            self.assertTrue(np.all(r.power_battery_W >= -1.0))
            self.assertTrue(np.all(np.abs(np.diff(r.tilt_deg) / np.diff(r.time_s))
                                   <= limits.tilt_rate_max_deg_s + 1e-6))

    def test_mass_is_constant_and_no_fuel(self):
        np.testing.assert_allclose(self.energy.mass_kg, mass_kg, rtol=1e-12)
        self.assertAlmostEqual(self.energy.mass_fuel_burnt_kg, 0.0, places=9)

    def test_energy_accounting_identity(self):
        """SOC drop x capacity = integral of chemical (OCV x current) power; the bus energy state is the integral
        of battery terminal power; their difference is the resistive loss, positive and small."""
        for r in (self.energy, self.time):
            energy_soc_J = (r.soc[0] - r.soc[-1]) * self.battery.energy_capacity_J
            self.assertAlmostEqual(r.energy_battery_chemical_J / energy_soc_J, 1.0, places=6)
            np.testing.assert_allclose(r.energy_bus_J, trapezoid(r.power_battery_W, r.time_s), rtol=1e-6, atol=1.0)
            self.assertAlmostEqual(r.energy_electric_J / r.energy_battery_terminal_J, 1.0, places=6)
            loss_fraction = (r.energy_battery_chemical_J - r.energy_battery_terminal_J) / r.energy_battery_chemical_J
            self.assertGreater(loss_fraction, 0.0)
            self.assertLess(loss_fraction, 0.02)

    def test_kinematics_consistent(self):
        r = self.energy
        np.testing.assert_allclose(r.altitude_m - r.altitude_m[0],
                                   trapezoid(r.velocity_m_s * np.sind(r.gamma_deg), r.time_s), atol=1e-5)
        self.assertAlmostEqual(r.distance_m, trapezoid(r.velocity_m_s * np.cosd(r.gamma_deg), r.time_s)[-1],
                               places=4)

    def test_objectives_trade_energy_and_time(self):
        self.assertLessEqual(self.time.duration_s, self.energy.duration_s * (1 + 1e-6))
        self.assertLessEqual(self.energy.energy_electric_J, self.time.energy_electric_J * (1 + 1e-6))
        # Climbing 500 m costs at least the potential energy.
        self.assertGreater(self.energy.energy_electric_J, mass_kg * 9.80665 * altitude_final_m)

    def test_duration_bound(self):
        """Equal duration bounds fix the time; a slower segment than the optimum costs more energy."""
        duration_s = 1.3 * self.energy.duration_s
        r = solve_transition_climb(self.model, mass_kg=mass_kg, soc_initial=soc_initial,
                                   altitude_final_m=altitude_final_m, velocity_final_m_s=velocity_final_m_s,
                                   count_nodes=count_nodes, duration_bounds_s=(duration_s, duration_s))
        self.assertAlmostEqual(r.duration_s / duration_s, 1.0, places=6)
        self.assertGreaterEqual(r.energy_electric_J, self.energy.energy_electric_J * (1 - 1e-6))

    def test_objective_extra(self):
        """An extra objective term trades the main objective: a penalty on the nacelle angle converts sooner (lower
        mean nacelle angle) and costs energy (never less than the optimum)."""
        r = solve_transition_climb(self.model, mass_kg=mass_kg, soc_initial=soc_initial,
                                   altitude_final_m=altitude_final_m, velocity_final_m_s=velocity_final_m_s,
                                   count_nodes=count_nodes,
                                   objective_extra=lambda opti, t: np.mean(t.tilt_deg / 90))
        self.assertLess(np.mean(r.tilt_deg), np.mean(self.energy.tilt_deg) - 0.5)
        self.assertGreaterEqual(r.energy_electric_J, self.energy.energy_electric_J * (1 - 1e-6))

    def test_steady_level_power_and_rule(self):
        power_W = solve_steady_level_power(self.model, mass_kg=mass_kg, soc=soc_initial, altitude_m=altitude_final_m,
                                           velocity_m_s=velocity_final_m_s)
        # The segment ends in the same steady flight point.
        self.assertAlmostEqual(self.energy.power_bus_W[-1] / power_W, 1.0, places=3)
        self.assertAlmostEqual(energy_fixed_power_rule_J(power_W, 1.5, 180.0), 270.0 * power_W)


if __name__ == "__main__":
    unittest.main()
