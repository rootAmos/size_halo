"""Tier 18 (plan 032): redundancy architecture in the topology, multiplicity masses and degraded states."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from aircraft_closure.performance.flight_point import FlightCondition
from aircraft_closure.powertrain.compatibility import design_margins
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.components.cable import Cable
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor, TorqueDensityMassModel, rubber_machine
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.thermal import LumpedThermalModel
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.redundancy import (battery_for_condition, battery_with_strings, degraded_state,
                                                    redundancy_counts)
from aircraft_closure.powertrain.topologies import RedundancyLayer, build_series_hybrid
from aircraft_closure.thermal.heat import thermal_parameters
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation

mass_model = TorqueDensityMassModel(15.0, 10000.0)


def lane_motor(count_lanes, torque_rotor_Nm=2000.0):
    return rubber_machine(Motor, 400.0, torque_rotor_Nm / count_lanes, torque_ratio=2.5, power_ratio=1.25,
                          speed_ratio=2.5, mass_model=mass_model)


def hardware(count_lanes=2, count_buses=2, count_strings=2):
    return RedundancyLayer(count_lanes=count_lanes, count_buses=count_buses, count_strings_battery=count_strings,
                           protection_string=ProtectionUnit(max_current_A=300.0) if count_strings > 1 else None,
                           protection_bus_tie=ProtectionUnit(max_current_A=800.0) if count_buses > 1 else None,
                           cable_bus_tie=Cable(length_m=2.0, max_current_A=800.0) if count_buses > 1 else None)


def topology(redundancy=None, count_lanes=1, battery=None):
    return build_series_hybrid(lane_motor(count_lanes), Generator(), battery or Battery(), SimpleTurboshaft(),
                               Gearbox(power_rated_W=1e6), ActuatorDiskPropulsor(), count_rotors=2,
                               count_turbogenerators=2, redundancy=redundancy)


def installed_masses(t):
    locations = tuple(InstalledInstance(name, x_m=0.0) for name in t.instances)
    return {item.instance_name: item.mass_properties.mass
            for item in PowertrainInstallation(t, locations).get_instance_mass_properties()}


class ArchitectureTests(unittest.TestCase):
    def test_all_ones_builds_the_plain_topology(self):
        plain = topology()
        ones = topology(RedundancyLayer())
        self.assertEqual({n: i.count for n, i in plain.instances.items()},
                         {n: i.count for n, i in ones.instances.items()})
        self.assertEqual(plain.connections, ones.connections)
        self.assertEqual(ones.buses["bus"].count, 1)
        self.assertEqual(redundancy_counts(ones).count_ties, 0)

    def test_counts_in_the_topology(self):
        t = topology(hardware(count_lanes=2, count_buses=2, count_strings=3), count_lanes=2)
        counts = {n: i.count for n, i in t.instances.items()}
        self.assertEqual(counts["motor"], 4)
        self.assertEqual(counts["gearbox"], 2)
        self.assertEqual(counts["protection_string"], 3)
        self.assertEqual(counts["protection_bus_tie"], 1)
        self.assertEqual(t.buses["bus"].count, 2)
        self.assertEqual(redundancy_counts(t), redundancy_counts(t).__class__(2, 2, 3, 1))
        combined = {(c.source, c.target) for c in t.connections if c.combine}
        self.assertEqual(combined, {("motor.shaft", "gearbox.shaft_in"),
                                    ("battery.electrical", "protection_string.input")})

    def test_validation(self):
        with self.assertRaisesRegex(ValueError, "multiple of count_buses"):
            RedundancyLayer(count_lanes=3, count_buses=2, protection_bus_tie=ProtectionUnit(), cable_bus_tie=Cable())
        with self.assertRaisesRegex(ValueError, "protection_string"):
            RedundancyLayer(count_strings_battery=2)
        with self.assertRaisesRegex(ValueError, "bus_tie"):
            RedundancyLayer(count_lanes=2, count_buses=2)
        with self.assertRaisesRegex(ValueError, "positive integer"):
            RedundancyLayer(count_lanes=0)


class MultiplicityMassTests(unittest.TestCase):
    def test_lane_motors_total_one_machine_under_linear_torque_density(self):
        """N lane motors at 1/N of the torque weigh exactly one machine of the full torque (linear mass model)."""
        single = installed_masses(topology())["motor"]
        for lanes in (2, 3, 4):
            masses = installed_masses(topology(hardware(count_lanes=lanes, count_buses=1, count_strings=1),
                                               count_lanes=lanes))
            self.assertAlmostEqual(masses["motor"], 2 * lanes * lane_motor(lanes).get_mass(), places=9)
            self.assertAlmostEqual(masses["motor"] / single, 1.0, places=2)   # softmax smoothing only

    def test_hardware_masses_are_component_times_count(self):
        h = hardware(count_lanes=4, count_buses=2, count_strings=3)
        masses = installed_masses(topology(h, count_lanes=4))
        self.assertAlmostEqual(masses["protection_string"], 3 * h.protection_string.get_mass())
        self.assertAlmostEqual(masses["protection_bus_tie"], h.protection_bus_tie.get_mass())
        self.assertAlmostEqual(masses["cable_bus_tie"], h.cable_bus_tie.get_mass())
        # Two poles of 0.2 kg + 1.3 g/A each.
        self.assertAlmostEqual(h.protection_string.get_mass(), 2 * (0.2 + 1.3e-3 * 300.0))

    def test_more_lanes_lower_per_lane_rating(self):
        ratings = [lane_motor(n).power_rated_W for n in (1, 2, 3, 4)]
        self.assertTrue(all(a > b for a, b in zip(ratings, ratings[1:])))
        self.assertAlmostEqual(ratings[0] / ratings[3], 4.0)

    def test_design_margins_sum_lanes_at_the_combiner(self):
        t = topology(hardware(count_lanes=2, count_buses=1, count_strings=1), count_lanes=2)
        margins = {m.label: m.value for m in design_margins(t)}
        lane = lane_motor(2)
        # Two lanes of rated power against the 1 MW gearbox input.
        self.assertAlmostEqual(margins["motor.shaft->gearbox.shaft_in max_power_W"],
                               (1e6 - 2 * lane.power_rated_W) / 1e6)


class StringTests(unittest.TestCase):
    def test_equivalent_circuit_pack_scales_with_active_strings(self):
        pack = EquivalentCircuitBattery(count_parallel=20.0)
        half = battery_with_strings(pack, 0.5)
        self.assertAlmostEqual(half.energy_capacity_J / pack.energy_capacity_J, 0.5)
        self.assertAlmostEqual(half.get_limits().max_discharge_current_A / pack.get_limits().max_discharge_current_A,
                               0.5)
        self.assertAlmostEqual(sum(half.resistances_ohm(0.5)) / sum(pack.resistances_ohm(0.5)), 2.0)
        self.assertEqual(half.get_limits().max_voltage_V, pack.get_limits().max_voltage_V)
        self.assertIs(battery_with_strings(pack, 1), pack)

    def test_constant_pack_scales(self):
        pack = Battery()
        part = battery_with_strings(pack, 2 / 3)
        self.assertAlmostEqual(part.energy_capacity_J, pack.energy_capacity_J * 2 / 3)
        self.assertAlmostEqual(part.resistance_ohm, pack.resistance_ohm * 1.5)
        self.assertAlmostEqual(part.max_discharge_power_W, pack.max_discharge_power_W * 2 / 3)
        self.assertEqual(part.voltage_open_circuit_V, pack.voltage_open_circuit_V)

    def test_isolated_string_keeps_the_thermal_time_constant(self):
        """Capacitance scales with the strings' mass, resistance inversely: the time constant is unchanged."""
        pack = EquivalentCircuitBattery(count_parallel=20.0, thermal_model=LumpedThermalModel(1000.0, 60.0, 25.0))
        full, half = thermal_parameters(pack), thermal_parameters(battery_with_strings(pack, 0.5))
        self.assertAlmostEqual(half.capacity_J_K / full.capacity_J_K, 0.5)
        self.assertAlmostEqual(half.resistance_K_W / full.resistance_K_W, 2.0)
        self.assertAlmostEqual(half.capacity_J_K * half.resistance_K_W, full.capacity_J_K * full.resistance_K_W)

    def test_symbolic_parallel_count(self):
        opti = asb.Opti()
        count_parallel = opti.variable(init_guess=10.0, lower_bound=1.0)
        part = battery_with_strings(EquivalentCircuitBattery(count_parallel=count_parallel), 0.5)
        opti.subject_to(part.get_limits().max_discharge_current_A == 500.0)
        solution = opti.solve(verbose=False)
        cell_A = EquivalentCircuitBattery(count_parallel=1.0).get_limits().max_discharge_current_A
        self.assertAlmostEqual(float(solution.value(count_parallel)), 2 * 500.0 / cell_A, places=6)


class DegradedStateTests(unittest.TestCase):
    def setUp(self):
        self.t = topology(hardware(count_lanes=4, count_buses=2, count_strings=4), count_lanes=4)

    def test_normal_state(self):
        state = degraded_state(self.t, FlightCondition())
        self.assertEqual((state.count_lanes_active, state.count_strings_active, state.count_buses_failed), (4, 4, 0))
        self.assertEqual(state.fraction_strings_active, 1.0)

    def test_lane_bus_and_string_out(self):
        self.assertEqual(degraded_state(self.t, FlightCondition(active_lane_count=3)).count_lanes_active, 3)
        # A failed bus takes its 4 / 2 lanes per rotor.
        self.assertEqual(degraded_state(self.t, FlightCondition(count_buses_failed=1)).count_lanes_active, 2)
        state = degraded_state(self.t, FlightCondition(active_battery_string_count=3))
        self.assertAlmostEqual(state.fraction_strings_active, 0.75)
        battery = battery_for_condition(self.t, FlightCondition(active_battery_string_count=3))
        self.assertAlmostEqual(battery.energy_capacity_J, 0.75 * Battery().energy_capacity_J)

    def test_invalid_states_rejected(self):
        for condition in (FlightCondition(active_lane_count=0), FlightCondition(active_lane_count=5),
                          FlightCondition(active_battery_string_count=0), FlightCondition(count_buses_failed=2)):
            with self.assertRaises(ValueError):
                degraded_state(self.t, condition)
        plain = topology()
        with self.assertRaisesRegex(ValueError, "bus ties"):
            degraded_state(plain, FlightCondition(count_buses_failed=1))


if __name__ == "__main__":
    unittest.main()
