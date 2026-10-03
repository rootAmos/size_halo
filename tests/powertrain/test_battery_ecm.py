"""Tier 17 equivalent-circuit battery: identities, limits, trends, symbolic use, tabulated OCV and data fits."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.core.ports import Direction, ElectricalPortValue
from aircraft_closure.core.topology import Topology
from aircraft_closure.powertrain import cells
from aircraft_closure.powertrain.compatibility import operating_margins, port_envelope
from aircraft_closure.powertrain.components.battery_ecm import (EquivalentCircuitBattery, PolynomialOcvModel,
                                                                inr21700_50g_cell)
from aircraft_closure.powertrain.ports import port_specs_for

cell = inr21700_50g_cell()
ocv_table = cells.load_ocv_table()
dcir_table = cells.load_dcir_table()


def single_cell(**kwargs):
    return EquivalentCircuitBattery(count_series=1, count_parallel=1.0, **kwargs)


class CellDataTests(unittest.TestCase):
    def test_tables_are_complete(self):
        """Fig. 8 vector data: 6 temperatures x 4 pulses x 2 directions; 13 SOC points (12 at -10 C)."""
        self.assertEqual(len(dcir_table.soc), (5 * 13 + 12) * 4 * 2)
        soc, voltage_V = ocv_table.at_temperature(30.0)
        np.testing.assert_allclose(soc, np.linspace(0, 1, 21), atol=1e-9)
        self.assertTrue(np.all(np.diff(voltage_V) > 0))
        self.assertTrue(np.all((dcir_table.resistance_dc_ohm > 0.01) & (dcir_table.resistance_dc_ohm < 0.2)))

    def test_reference_points(self):
        """Read off the paper's figures: OCV(50 %, 30 C) ~3.72 V; 2 s discharge DCIR 23.5 mOhm at 30 C."""
        soc, voltage_V = ocv_table.at_temperature(30.0)
        self.assertAlmostEqual(float(voltage_V[10]), 3.72, delta=0.02)
        self.assertAlmostEqual(float(voltage_V[0]), 2.90, delta=0.02)
        self.assertAlmostEqual(float(voltage_V[-1]), 4.19, delta=0.02)
        mask = ((dcir_table.temperature_C == 30) & (dcir_table.soc == 0.5) & (dcir_table.duration_pulse_s == 2)
                & dcir_table.discharge)
        self.assertAlmostEqual(float(dcir_table.resistance_dc_ohm[mask][0]), 0.0235, delta=0.0005)

    def test_digitized_power_is_consistent_with_dcir_and_ocv(self):
        """Paudel eq. (12): P = 2.5 V (OCV - 2.5 V) / DCIR reproduces the separately digitized Fig. 8 power
        to about 1 %, a cross-check of both digitizations (OCV raster, DCIR vector)."""
        discharge = dcir_table.select(True)
        mask = discharge.temperature_C >= 20      # OCV curves coincide there (Fig. 7)
        power_W = cells.pulse_power_W(cell.ocv_model.voltage_V(discharge.soc[mask]), discharge.resistance_dc_ohm[mask])
        relative = power_W / discharge.power_pulse_W[mask] - 1
        self.assertLess(np.sqrt(np.mean(relative**2)), 0.015)
        self.assertLess(np.max(np.abs(relative)), 0.05)

    def test_ocv_polynomial_fit(self):
        soc, voltage_V = ocv_table.at_temperature(30.0)
        refit = cells.fit_ocv_polynomial(soc, voltage_V)
        np.testing.assert_allclose(refit, cell.ocv_model.coefficients, rtol=1e-6)
        residual_V = cell.ocv_model.voltage_V(soc) - voltage_V
        self.assertLess(np.sqrt(np.mean(residual_V**2)), 0.010)
        self.assertLess(np.max(np.abs(residual_V)), 0.025)
        grid = np.linspace(0, 1, 501)
        self.assertTrue(np.all(np.diff(cell.ocv_model.voltage_V(grid)) > 0))   # monotonic

    def test_ocv_temperature_dependence_is_small_above_30_percent(self):
        """The model's OCV has no temperature term; Fig. 7 curves at 10-45 C agree within ~35 mV above 30 %."""
        for temperature_C in (10.0, 20.0, 45.0):
            soc, voltage_V = ocv_table.at_temperature(temperature_C)
            mask = soc >= 0.3
            self.assertLess(np.max(np.abs(cell.ocv_model.voltage_V(soc[mask]) - voltage_V[mask])), 0.04)

    def test_resistance_refit_reproduces_the_coefficients(self):
        model, residual = cells.fit_resistance_model()
        for fitted, coded in zip(model.branches, cell.resistance_model.branches):
            self.assertAlmostEqual(fitted.resistance_ref_ohm / coded.resistance_ref_ohm, 1.0, places=3)
        self.assertLess(np.sqrt(np.mean(residual**2)), 0.05)     # 4.7 % rms over 308 points
        self.assertLess(np.max(np.abs(residual)), 0.22)

    def test_resistance_fit_by_temperature_and_pulse(self):
        discharge = dcir_table.select(True)
        target = cells.resistance_dc_corrected_ohm(discharge, cell.ocv_model, cell.capacity_As)
        model = cell.resistance_model.resistance_dc_ohm(discharge.soc, discharge.temperature_C,
                                                        discharge.duration_pulse_s)
        relative = model / target - 1
        for temperature_C in (-10, 0, 10, 20, 30, 45):
            mask = discharge.temperature_C == temperature_C
            self.assertLess(np.sqrt(np.mean(relative[mask]**2)), 0.08, msg=temperature_C)
        # The ECM time constants put long pulses above short ones (more polarization).
        self.assertGreater(float(cell.resistance_model.resistance_dc_ohm(0.5, 25.0, 180.0)),
                           float(cell.resistance_model.resistance_dc_ohm(0.5, 25.0, 2.0)))


class TabulatedOcvTests(unittest.TestCase):
    table = cells.tabulated_ocv_model(30.0)

    def test_grid_node_recovery(self):
        np.testing.assert_allclose(self.table.voltage_V(np.array(self.table.soc)),
                                   self.table.voltage_open_circuit_V, atol=1e-9)

    def test_interpolation_between_nodes(self):
        for soc in (0.425, 0.575, 0.825):
            low = int(soc / 0.05)
            bounds = sorted((self.table.voltage_open_circuit_V[low], self.table.voltage_open_circuit_V[low + 1]))
            self.assertTrue(bounds[0] - 0.01 <= float(self.table.voltage_V(soc)) <= bounds[1] + 0.01)
            self.assertAlmostEqual(float(self.table.voltage_V(soc)), float(cell.ocv_model.voltage_V(soc)), delta=0.025)

    def test_boundary_holds_end_values(self):
        self.assertAlmostEqual(float(self.table.voltage_V(-0.1)), self.table.voltage_open_circuit_V[0], places=9)
        self.assertAlmostEqual(float(self.table.voltage_V(1.2)), self.table.voltage_open_circuit_V[-1], places=9)

    def test_interchangeable_with_the_polynomial(self):
        tabulated = single_cell(cell=replace(cell, ocv_model=self.table))
        polynomial = single_cell()
        self.assertAlmostEqual(float(tabulated.energy_capacity_J / polynomial.energy_capacity_J), 1.0, delta=0.005)
        self.assertAlmostEqual(float(tabulated.evaluate(5.0, 0.5).voltage_V),
                               float(polynomial.evaluate(5.0, 0.5).voltage_V), delta=0.02)


class EquivalentCircuitIdentityTests(unittest.TestCase):
    pack = EquivalentCircuitBattery(count_series=210, count_parallel=20.0, factor_power_density=5.0)

    def test_steady_state_ohm_law_and_power_split(self):
        result = self.pack.evaluate(300.0, 0.6)
        r0, r1, r2 = self.pack.resistances_ohm(0.6)
        ocv_V = float(self.pack.voltage_open_circuit_V(0.6))
        self.assertAlmostEqual(float(result.voltage_V), ocv_V - 300.0 * (r0 + r1 + r2), places=9)
        self.assertAlmostEqual(float(result.power_chemical_W), ocv_V * 300.0, places=6)
        self.assertAlmostEqual(float(result.power_loss_W), 300.0**2 * (r0 + r1 + r2), places=6)
        self.assertAlmostEqual(float(result.power_electric_W + result.power_loss_W), float(result.power_chemical_W))
        self.assertEqual(result.soc_next, 0.6)                       # no duration, no depletion

    def test_pack_scaling(self):
        one = single_cell()
        r_cell = one.resistances_ohm(0.5)
        r_pack = EquivalentCircuitBattery(count_series=210, count_parallel=20.0).resistances_ohm(0.5)
        for a, b in zip(r_cell, r_pack):
            self.assertAlmostEqual(b / a, 210 / 20.0)
        self.assertAlmostEqual(float(self.pack.voltage_open_circuit_V(0.5) / one.voltage_open_circuit_V(0.5)), 210)
        self.assertAlmostEqual(float(self.pack.get_mass()), 210 * 20 * 0.069 / 0.7)
        self.assertAlmostEqual(float(one.energy_capacity_J) / 3600, 18.15, delta=0.1)   # 4.9 Ah x 3.70 V mean

    def test_scale_factor_one_is_the_50g(self):
        one = single_cell()
        self.assertEqual(one.resistances_ohm(0.5, 30.0), cell.resistance_model.resistances_ohm(0.5, 30.0))
        self.assertEqual(one.get_limits().max_discharge_current_A, 9.8)
        five = single_cell(factor_power_density=5.0)
        self.assertAlmostEqual(five.resistances_ohm(0.5)[0] * 5, one.resistances_ohm(0.5)[0])
        self.assertAlmostEqual(five.get_limits().max_discharge_current_A, 49.0)
        self.assertEqual(five.get_mass(), one.get_mass())           # power density costs no mass (assumption)

    def test_ageing_factors(self):
        aged = replace(self.pack, factor_capacity_ageing=0.8, factor_resistance_ageing=1.5)
        self.assertAlmostEqual(float(aged.energy_capacity_J / self.pack.energy_capacity_J), 0.8)
        self.assertAlmostEqual(aged.resistances_ohm(0.5)[0] / self.pack.resistances_ohm(0.5)[0], 1.5)
        self.assertLess(float(aged.evaluate(400.0, 0.5).voltage_V), float(self.pack.evaluate(400.0, 0.5).voltage_V))

    def test_max_power_and_low_root(self):
        """P = V I with V = V* - I R: the low root is the physical one and V >= V*/2 there; P_max = V*^2/(4R)."""
        result = self.pack.evaluate(0.0, 0.3)
        v_star_V, r_ohm = float(result.voltage_driving_V), float(result.resistance_effective_ohm)
        power_W = 0.6 * float(result.power_max_W)
        discriminant = v_star_V**2 - 4 * r_ohm * power_W
        low_A, high_A = (v_star_V - np.sqrt(discriminant)) / (2 * r_ohm), (v_star_V + np.sqrt(discriminant)) / (2 * r_ohm)
        for current_A in (low_A, high_A):
            self.assertAlmostEqual(float(self.pack.evaluate(current_A, 0.3).power_electric_W) / power_W, 1.0, places=9)
        self.assertGreaterEqual(float(self.pack.evaluate(low_A, 0.3).voltage_V), v_star_V / 2)
        self.assertLess(float(self.pack.evaluate(high_A, 0.3).voltage_V), v_star_V / 2)
        self.assertAlmostEqual(float(result.power_max_W), v_star_V**2 / (4 * r_ohm))
        peak = self.pack.evaluate(v_star_V / (2 * r_ohm), 0.3)
        self.assertAlmostEqual(float(peak.power_electric_W / result.power_max_W), 1.0, places=9)

    def test_rc_transient_limits(self):
        """From rest: zero duration gives R0 only (instant step); long durations reach the steady state."""
        rest = (0.0, 0.0)
        instant = self.pack.evaluate(300.0, 0.5, 0.0, rest)
        r0 = self.pack.resistances_ohm(0.5)[0]
        self.assertAlmostEqual(float(instant.voltage_V), float(self.pack.voltage_open_circuit_V(0.5)) - 300.0 * r0)
        pack = replace(self.pack, count_parallel=1e6)                # negligible SOC change over the hold
        long_hold = pack.evaluate(300.0, 0.5, 3000.0, rest)
        steady = pack.evaluate(300.0, 0.5)
        for a, b in zip(long_hold.voltage_rc_end_V, steady.voltage_rc_V):
            self.assertAlmostEqual(float(a), float(b), places=6)

    def test_rc_exact_propagation(self):
        """Two 30 s steps equal one 60 s step (exact exponential update at constant current)."""
        pack = replace(self.pack, count_parallel=1e6)                # negligible SOC change: OCV/R constant
        one = pack.evaluate(200.0, 0.5, 60.0, (1.0, 2.0))
        half = pack.evaluate(200.0, 0.5, 30.0, (1.0, 2.0))
        two = pack.evaluate(200.0, 0.5, 30.0, half.voltage_rc_end_V)
        for a, b in zip(one.voltage_rc_end_V, two.voltage_rc_end_V):
            self.assertAlmostEqual(float(a), float(b), places=9)
        # The interval mean is the average of the two halves' means.
        for a, b, c in zip(one.voltage_rc_V, half.voltage_rc_V, two.voltage_rc_V):
            self.assertAlmostEqual(float(a), float((b + c) / 2), places=9)

    def test_coulomb_counting_and_signs(self):
        result = self.pack.evaluate(98.0, 0.8, 3600.0)
        self.assertAlmostEqual(float(result.soc_next), 0.8 - 98.0 * 3600.0 / (20 * 4.9 * 3600))
        charge = self.pack.evaluate(-50.0, 0.5, 60.0, (0.0, 0.0))
        self.assertGreater(float(charge.soc_next), 0.5)
        self.assertGreater(float(charge.voltage_V), float(charge.voltage_open_circuit_V))
        self.assertGreater(float(charge.power_loss_W), 0)           # Joule loss positive while charging
        self.assertLess(float(charge.power_electric_W), 0)


class TrendTests(unittest.TestCase):
    pack = EquivalentCircuitBattery(count_series=210, count_parallel=20.0)

    def test_discharge_curve_shape(self):
        """Constant-power discharge: the bus voltage falls with SOC, steeply below 20 % (the user's curve)."""
        voltages = [float(self.pack.evaluate(200.0, soc).voltage_V) for soc in (0.9, 0.5, 0.2, 0.1, 0.05)]
        self.assertTrue(all(a > b for a, b in zip(voltages, voltages[1:])))
        self.assertGreater(voltages[3] - voltages[4], (voltages[0] - voltages[1]) / 8)
        slope_mid = voltages[1] - voltages[2]
        self.assertGreater(voltages[2] - voltages[3], slope_mid / 3 * 1.5)   # steeper per unit SOC below 0.2

    def test_sag_grows_at_low_soc_and_low_temperature(self):
        def sag(soc, temperature_C=None):
            result = self.pack.evaluate(200.0, soc, temperature_C=temperature_C)
            return float(result.voltage_open_circuit_V - result.voltage_V)
        self.assertGreater(sag(0.1), sag(0.5))
        self.assertGreater(sag(0.5, 0.0), sag(0.5, 25.0))
        self.assertGreater(sag(0.5, -10.0), 2.5 * sag(0.5, 25.0))

    def test_extrapolation_policy(self):
        """Low SOC: resistance keeps rising (conservative); above SOC 0.8 the high-SOC term is held."""
        model = cell.resistance_model
        self.assertGreater(sum(model.resistances_ohm(0.05, 25.0)), 1.5 * sum(model.resistances_ohm(0.2, 25.0)))
        held = [sum(model.resistances_ohm(soc, 25.0)) for soc in (0.85, 0.95, 1.0)]
        self.assertLess(max(held) / min(held) - 1, 0.02)
        self.assertLess(held[-1] / sum(model.resistances_ohm(0.8, 25.0)) - 1, 0.1)


class SymbolicTests(unittest.TestCase):
    def test_sized_pack_in_opti(self):
        """count_parallel and current as Opti variables; P = V I solved on the low-current branch."""
        opti = asb.Opti()
        count_parallel = opti.variable(init_guess=10.0, lower_bound=1.0)
        current_A = opti.variable(init_guess=50.0)
        pack = EquivalentCircuitBattery(count_series=210, count_parallel=count_parallel, factor_power_density=5.0,
                                        factor_capacity_ageing=0.8, factor_resistance_ageing=1.5)
        result = pack.evaluate(current_A, 0.3, 60.0, (0.0, 0.0))
        opti.subject_to([result.power_electric_W == 400e3, result.voltage_V / result.voltage_driving_V >= 0.5,
                         result.voltage_end_V >= pack.get_limits().min_voltage_V,
                         current_A <= pack.get_limits().max_discharge_current_A])
        opti.minimize(pack.get_mass())
        solution = opti.solve(verbose=False)
        voltage_V = solution.value(result.voltage_V)
        self.assertAlmostEqual(voltage_V * solution.value(current_A) / 400e3, 1.0, places=6)
        self.assertGreater(voltage_V, 0.5 * solution.value(result.voltage_driving_V))
        self.assertGreater(solution.value(count_parallel), 1.0)

    def test_numeric_arrays(self):
        pack = EquivalentCircuitBattery()
        result = pack.evaluate(np.array([0.0, 100.0, 200.0]), np.array([0.9, 0.5, 0.2]), 10.0, (0.0, 0.0))
        self.assertEqual(np.shape(result.voltage_V), (3,))
        self.assertTrue(np.all(np.diff(result.voltage_V) < 0))


class PortAndMarginTests(unittest.TestCase):
    def test_ports_envelope_and_margins(self):
        pack = EquivalentCircuitBattery(count_series=210, count_parallel=20.0, factor_power_density=5.0)
        (port,) = port_specs_for(pack)
        self.assertEqual((port.name, port.direction), ("electrical", Direction.OUT))
        envelope = port_envelope(pack, "electrical")
        self.assertTrue(envelope.sets_voltage)
        self.assertAlmostEqual(envelope.min_voltage_V, 210 * 2.5)
        self.assertAlmostEqual(envelope.max_voltage_V, 210 * 4.2)
        self.assertAlmostEqual(envelope.max_power_W, 20 * 9.8 * 5 * 210 * 3.6)
        topology = Topology()
        topology.add("battery", pack, port_specs_for(pack))
        margins = {m.label: m.value for m in operating_margins(
            topology, {"battery.electrical": ElectricalPortValue(600.0, 490.0)})}
        self.assertAlmostEqual(margins["battery discharge_current_A"], 0.5)
        self.assertAlmostEqual(margins["battery min_voltage_V"], 600.0 / 525.0 - 1)
        self.assertAlmostEqual(margins["battery charge_current_A"], 1 + 490.0 / (20 * 4.9 * 5))


if __name__ == "__main__":
    unittest.main()
