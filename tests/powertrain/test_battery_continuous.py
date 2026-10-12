"""Plan 060: continuous battery voltage (instantaneous terminal voltage, RC rates, collocated RC states)."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
from scipy.optimize import brentq

from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.trajectory.tiltrotor import battery_rc_states


def halo_pack():
    """The sizing's pack settings (plan 021): 210s x 20p, 5x power density, end of life."""
    return EquivalentCircuitBattery(count_series=210, count_parallel=20.0, factor_power_density=5.0,
                                    factor_capacity_ageing=0.8, factor_resistance_ageing=1.5)


def constant_power_discharge(pack, power_W, duration_s, soc_start, count_points):
    """The sizing mission's point chain (constant current per point, exact RC update) from steady polarization:
    returns (mid-SOC end voltage, end-SOC end voltage) of the last point."""
    dt_s = duration_s / count_points
    soc = soc_start
    voltage_rc_V = None
    for _ in range(count_points):
        # "steady" start: the first point's RC voltages at their steady values (as `polarization_start`).
        start = (lambda i: voltage_rc_V) if voltage_rc_V is not None else (
            lambda i: tuple(i * r for r in pack.resistances_ohm(soc)[1:]))
        current_A = brentq(lambda i: float(pack.evaluate(i, soc, dt_s, start(i)).power_electric_W) - power_W,
                           1.0, 1200.0)       # below the matched-load current: the low-current root
        result = pack.evaluate(current_A, soc, dt_s, start(current_A))
        soc, voltage_rc_V = float(result.soc_next), tuple(float(v) for v in result.voltage_rc_end_V)
    return float(result.voltage_end_V), float(pack.voltage_terminal_V(current_A, soc, voltage_rc_V))


class InstantaneousVoltageTests(unittest.TestCase):
    def setUp(self):
        self.pack = halo_pack()

    def test_equals_zero_duration_evaluation(self):
        result = self.pack.evaluate(400.0, 0.4, 0.0, (3.0, 20.0))
        self.assertAlmostEqual(self.pack.voltage_terminal_V(400.0, 0.4, (3.0, 20.0)), result.voltage_V, places=9)
        # At rest: the open-circuit voltage.
        self.assertAlmostEqual(self.pack.voltage_terminal_V(0.0, 0.4, (0.0, 0.0)),
                               self.pack.voltage_open_circuit_V(0.4), places=9)

    def test_rc_rates(self):
        pack, current_A, soc = self.pack, 300.0, 0.5
        steady = tuple(current_A * r for r in pack.resistances_ohm(soc)[1:])
        np.testing.assert_allclose(pack.derivative_voltage_rc_V_s(current_A, soc, steady), 0.0, atol=1e-12)
        # Below steady the branch charges (positive rate); after the load drops it recovers (negative rate).
        self.assertTrue(all(r > 0 for r in pack.derivative_voltage_rc_V_s(current_A, soc, (0.0, 0.0))))
        self.assertTrue(all(r < 0 for r in pack.derivative_voltage_rc_V_s(0.0, soc, steady)))

    def test_exact_update_satisfies_the_ode(self):
        """d/dt of the exact constant-current update equals the rate at the end (central difference)."""
        pack, current_A, soc, start = self.pack, 300.0, 0.5, (1.0, 2.0)
        h_s, t_s = 1e-3, 20.0
        end = lambda dt: pack.evaluate(current_A, soc, dt, start).voltage_rc_end_V  # noqa: E731
        # The update takes R_k at the mid-interval SOC, which moves with dt; evaluate the rate there.
        soc_mid = soc - current_A * t_s / (2 * pack.capacity_As)
        rates = pack.derivative_voltage_rc_V_s(current_A, soc_mid, end(t_s))
        finite = [(a - b) / (2 * h_s) for a, b in zip(end(t_s + h_s), end(t_s - h_s))]
        # What remains is the R_k change with SOC over the interval (dR/dt term), under 1 %.
        np.testing.assert_allclose(finite, rates, rtol=1e-2)

    def test_symbolic(self):
        opti = asb.Opti()
        current_A = opti.variable(init_guess=100.0)
        voltage_V = self.pack.voltage_terminal_V(current_A, 0.5, (0.0, 0.0))
        opti.subject_to(voltage_V == 700.0)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(self.pack.voltage_terminal_V(solution.value(current_A), 0.5, (0.0, 0.0))),
                               700.0, places=6)


class EndpointMinimumTests(unittest.TestCase):
    """Constant current over an interval: the terminal voltage's minimum is at an end of the interval."""

    def history_V(self, current_A, soc_start, start_V, duration_s=60.0):
        pack = halo_pack()
        time_s = np.linspace(0.0, duration_s, 601)
        soc = soc_start - current_A * time_s / pack.capacity_As
        resistances = pack.resistances_ohm(soc_start)[1:]
        taus = pack.cell.resistance_model.time_constants_s()
        rc_V = [current_A * r + (v0 - current_A * r) * np.exp(-time_s / tau)
                for r, tau, v0 in zip(resistances, taus, start_V)]
        return pack.voltage_terminal_V(current_A, soc, rc_V)

    def test_step_up_from_rest(self):
        voltage_V = self.history_V(800.0, 0.3, (0.0, 0.0))
        self.assertEqual(int(np.argmin(voltage_V)), len(voltage_V) - 1)
        self.assertTrue(np.all(np.diff(voltage_V) < 0))

    def test_step_down_from_developed_polarization(self):
        pack = halo_pack()
        developed = tuple(900.0 * r for r in pack.resistances_ohm(0.3)[1:])
        for current_A in (0.0, 200.0, 600.0):
            voltage_V = self.history_V(current_A, 0.3, developed)
            self.assertIn(int(np.argmin(voltage_V)), (0, len(voltage_V) - 1))


class EndVoltageConvergenceTests(unittest.TestCase):
    """The engine-out-like case: 500 kW for 60 s from SOC 0.3 with developed polarization."""

    @classmethod
    def setUpClass(cls):
        pack = halo_pack()
        cls.runs = {n: constant_power_discharge(pack, 500e3, 60.0, 0.3, n) for n in (1, 2, 3, 6, 12, 96)}

    def test_mid_soc_end_voltage_overstates(self):
        for n in (1, 3, 12):
            quasi_steady_V, continuous_V = self.runs[n]
            self.assertGreater(quasi_steady_V, continuous_V)

    def test_continuous_end_voltage_converges(self):
        reference_V = self.runs[96][1]
        errors = [abs(self.runs[n][1] - reference_V) for n in (1, 2, 3, 6, 12)]
        self.assertTrue(all(a > b for a, b in zip(errors, errors[1:])), errors)
        # First order: the current is constant within a point while the constant-power current rises.
        self.assertAlmostEqual(errors[3] / errors[4], 2.0, delta=0.3)
        self.assertLess(errors[-1], 1.5)
        # The mid-SOC end voltage converges to the same value, about four times more slowly.
        self.assertGreater(abs(self.runs[12][0] - reference_V), 3 * errors[-1])


class CollocatedRcStateTests(unittest.TestCase):
    """The trajectory's RC states (trapezoidal collocation) against the closed-form constant-current response."""

    def solve(self, current_A, start_V, count_nodes, duration_s=60.0):
        pack = halo_pack()
        opti = asb.Opti()
        time_s = np.linspace(0.0, duration_s, count_nodes)
        states, constraints = battery_rc_states(opti, pack, current_A * np.ones(count_nodes),
                                                0.5 * np.ones(count_nodes), time_s, count_nodes, start_V)
        opti.subject_to(constraints)
        opti.minimize(0)
        solution = opti.solve(verbose=False)
        resistances = pack.resistances_ohm(0.5)[1:]
        taus = pack.cell.resistance_model.time_constants_s()
        exact = [current_A * r + (v0 - current_A * r) * np.exp(-time_s / tau)
                 for r, tau, v0 in zip(resistances, taus, start_V)]
        return [np.max(np.abs(solution.value(s) - e)) / max(np.max(np.abs(e)), 1e-9) for s, e in zip(states, exact)]

    def test_rise_from_rest(self):
        errors = self.solve(600.0, (0.0, 0.0), 31)       # 2 s spacing, tau_1 = 8 s
        self.assertLess(max(errors), 1e-2)

    def test_relaxation_after_the_load_drops(self):
        errors = self.solve(0.0, (5.0, 40.0), 31)
        self.assertLess(max(errors), 1e-2)

    def test_second_order_convergence(self):
        coarse = self.solve(600.0, (0.0, 0.0), 16)
        fine = self.solve(600.0, (0.0, 0.0), 31)
        for a, b in zip(coarse, fine):
            self.assertGreater(a / b, 3.0)


if __name__ == "__main__":
    unittest.main()
