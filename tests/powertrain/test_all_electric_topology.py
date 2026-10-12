"""Plan 066: all-electric topology with per-rotor motor channels, one-channel-out degraded state."""
import unittest
from dataclasses import dataclass

import aerosandbox as asb

from aircraft_closure.core.topology import connection_residuals
from aircraft_closure.powertrain.compatibility import design_margins
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.cable import Cable
from aircraft_closure.powertrain.components.converters import Inverter
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.redundancy import degraded_state, redundancy_counts
from aircraft_closure.powertrain.topologies import (ElectricalLayer, RedundancyLayer, build_all_electric,
                                                    build_series_hybrid)
from examples import all_electric_tiltrotor as notional


@dataclass(frozen=True)
class Condition:
    active_lane_count: int = None
    count_rotors_lane_out: int = None
    count_buses_failed: int = 0
    active_battery_string_count: int = None


def electrical_layer():
    return ElectricalLayer(inverter_motor=Inverter(power_rated_W=100e3), cable_motor=Cable(max_current_A=200.0),
                           protection_motor=ProtectionUnit(max_current_A=200.0), inverter_generator=None,
                           cable_generator=None, protection_generator=None, cable_battery=Cable(length_m=2.0),
                           protection_battery=ProtectionUnit(max_current_A=1000.0))


def counts_of(topology):
    return {name: instance.count for name, instance in topology.instances.items()}


class StructureTests(unittest.TestCase):
    def test_plain_counts_and_links(self):
        for channels in range(2, 7):
            topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(),
                                          count_channels_per_rotor=channels)
            self.assertEqual(counts_of(topology), {"battery": 1, "motor": 2 * channels, "propulsor": 2})
            self.assertEqual(set(topology.buses), {"bus"})
            self.assertEqual({(c.source, c.target, c.combine) for c in topology.connections},
                             {("battery.electrical", "bus", False), ("motor.electrical", "bus", False),
                              ("motor.shaft", "propulsor.shaft", True)})
            self.assertEqual(redundancy_counts(topology).count_lanes, channels)

    def test_single_channel_is_a_plain_shaft(self):
        topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=1)
        self.assertEqual(counts_of(topology)["motor"], 2)
        self.assertFalse(any(c.combine for c in topology.connections))

    def test_electrical_layer_feeders_per_channel_and_no_generator(self):
        topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_rotors=2,
                                      count_channels_per_rotor=3, electrical=electrical_layer())
        counts = counts_of(topology)
        for name in ("motor", "inverter_motor", "cable_motor", "protection_motor"):
            self.assertEqual(counts[name], 6, name)
        self.assertEqual(counts["propulsor"], 2)
        self.assertEqual(counts["cable_battery"], 1)
        for name in ("generator", "inverter_generator", "cable_generator", "protection_generator", "turboshaft"):
            self.assertNotIn(name, counts)
        links = {(c.source, c.target) for c in topology.connections}
        self.assertIn(("protection_motor.input", "bus"), links)
        self.assertIn(("cable_battery.output", "bus"), links)
        self.assertIn(("inverter_motor.ac", "motor.electrical"), links)

    def test_gearbox_combines_channels_on_its_input(self):
        topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=2,
                                      gearbox=Gearbox(power_rated_W=500e3))
        self.assertEqual(counts_of(topology)["gearbox"], 2)
        self.assertEqual({(c.source, c.target, c.combine) for c in topology.connections if c.target != "bus"},
                         {("motor.shaft", "gearbox.shaft_in", True), ("gearbox.shaft_out", "propulsor.shaft", False)})

    def test_strings_and_cross_strapped_buses(self):
        redundancy = RedundancyLayer(count_lanes=4, count_buses=2, count_strings_battery=3,
                                     protection_string=ProtectionUnit(), protection_bus_tie=ProtectionUnit(),
                                     cable_bus_tie=Cable())
        topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=4,
                                      electrical=electrical_layer(), redundancy=redundancy)
        self.assertEqual(redundancy_counts(topology), redundancy_counts(topology).__class__(4, 2, 3, 1))
        self.assertIn(("protection_string.output", "protection_battery.input"),
                      {(c.source, c.target) for c in topology.connections})

    def test_mismatched_redundancy_rejected(self):
        with self.assertRaisesRegex(ValueError, "count_channels_per_rotor"):
            build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=3,
                               redundancy=RedundancyLayer(count_lanes=2))

    def test_series_hybrid_unchanged_by_shared_helpers(self):
        """Same instances, counts and connection order as the series-hybrid builder had before plan 066."""
        e = ElectricalLayer(Inverter(), Cable(), ProtectionUnit(), Inverter(), Cable(), ProtectionUnit(), Cable(),
                            ProtectionUnit())
        r = RedundancyLayer(count_lanes=2, count_buses=2, count_strings_battery=2, protection_string=ProtectionUnit(),
                            protection_bus_tie=ProtectionUnit(), cable_bus_tie=Cable())
        topology = build_series_hybrid(Motor(), Generator(), Battery(), SimpleTurboshaft(), Gearbox(),
                                       ActuatorDiskPropulsor(), count_rotors=2, count_turbogenerators=2,
                                       electrical=e, redundancy=r)
        self.assertEqual(list(counts_of(topology).items()), [
            ("turboshaft", 2), ("generator", 2), ("battery", 1), ("motor", 4), ("gearbox", 2), ("propulsor", 2),
            ("protection_string", 2), ("protection_bus_tie", 1), ("cable_bus_tie", 1), ("protection_motor", 4),
            ("cable_motor", 4), ("inverter_motor", 4), ("cable_generator", 2), ("protection_generator", 2),
            ("protection_battery", 1), ("cable_battery", 1), ("inverter_generator", 2)])
        self.assertEqual([(c.source, c.target) for c in topology.connections], [
            ("turboshaft.shaft", "generator.shaft"), ("battery.electrical", "protection_string.input"),
            ("protection_bus_tie.output", "cable_bus_tie.input"), ("protection_motor.input", "bus"),
            ("protection_motor.output", "cable_motor.input"), ("cable_motor.output", "inverter_motor.dc"),
            ("inverter_motor.ac", "motor.electrical"), ("generator.electrical", "inverter_generator.ac"),
            ("inverter_generator.dc", "cable_generator.input"), ("cable_generator.output", "protection_generator.input"),
            ("protection_generator.output", "bus"), ("protection_string.output", "protection_battery.input"),
            ("protection_battery.output", "cable_battery.input"), ("cable_battery.output", "bus"),
            ("motor.shaft", "gearbox.shaft_in"), ("gearbox.shaft_out", "propulsor.shaft")])


class DegradedStateTests(unittest.TestCase):
    def setUp(self):
        self.topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=3)

    def test_one_channel_out_on_one_rotor(self):
        state = degraded_state(self.topology, Condition(active_lane_count=2, count_rotors_lane_out=1))
        self.assertEqual(state.count_lanes_active, 2)
        self.assertEqual(state.count_lanes_active_healthy, 3)
        self.assertEqual(state.count_rotors_degraded, 1)

    def test_default_is_symmetric_and_unchanged(self):
        state = degraded_state(self.topology, Condition(active_lane_count=2))
        self.assertIsNone(state.count_rotors_degraded)
        self.assertEqual(state.count_lanes_active_healthy, 2)
        self.assertEqual(degraded_state(self.topology, Condition()).count_lanes_active, 3)

    def test_validation(self):
        for bad in (0, 3, 1.0):
            with self.assertRaisesRegex(ValueError, "count_rotors_lane_out"):
                degraded_state(self.topology, Condition(active_lane_count=2, count_rotors_lane_out=bad))

    def test_failed_bus_on_top_of_a_channel_out(self):
        redundancy = RedundancyLayer(count_lanes=4, count_buses=2, protection_bus_tie=ProtectionUnit(),
                                     cable_bus_tie=Cable())
        topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(), count_channels_per_rotor=4,
                                      redundancy=redundancy)
        state = degraded_state(topology, Condition(active_lane_count=3, count_rotors_lane_out=1, count_buses_failed=1))
        self.assertEqual(state.count_lanes_active, 1)
        self.assertEqual(state.count_lanes_active_healthy, 2)

    def test_channel_power_identity(self):
        power_rotor_W = 150e3
        for channels in range(2, 7):
            topology = build_all_electric(Motor(), Battery(), ActuatorDiskPropulsor(),
                                          count_channels_per_rotor=channels)
            state = degraded_state(topology, Condition(active_lane_count=channels - 1, count_rotors_lane_out=1))
            self.assertAlmostEqual(state.power_shaft_lane_W(power_rotor_W), power_rotor_W / (channels - 1))
            self.assertAlmostEqual(state.power_shaft_lane_W(power_rotor_W, degraded=False), power_rotor_W / channels)
            # The failed rotor's channels carry n / (n - 1) of their healthy share.
            self.assertAlmostEqual(state.power_shaft_lane_W(1.0) / state.power_shaft_lane_W(1.0, degraded=False),
                                   channels / (channels - 1))

    def test_channel_power_symbolic(self):
        opti = asb.Opti()
        power_rotor_W = opti.variable(init_guess=100e3, scale=1e5)
        state = degraded_state(self.topology, Condition(active_lane_count=2, count_rotors_lane_out=1))
        power_channel_W = state.power_shaft_lane_W(power_rotor_W)
        opti.subject_to(power_channel_W == 60e3)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution(power_rotor_W), 120e3, places=3)


class NotionalTiltrotorTests(unittest.TestCase):
    def setUp(self):
        self.data = notional.NotionalAllElectricTiltrotor()

    def test_connection_residuals_close_at_hover(self):
        topology = notional.build_notional_all_electric_tiltrotor(3, self.data)
        voltage_V, current_A, residual_max = notional.solve_hover_voltage(topology, self.data)
        self.assertLess(residual_max, 1e-6)
        self.assertTrue(0.9 * self.data.voltage_open_circuit_V < voltage_V < self.data.voltage_open_circuit_V)
        # Battery output covers the six motors' shaft power plus losses (drive efficiency below one).
        power_shaft_W = 2 * notional.power_shaft_hover_rotor_W(self.data)
        power_battery_W = topology.instances["battery"].component.evaluate(current_A, 0.9).power_electric_W
        self.assertTrue(0.85 < power_shaft_W / power_battery_W < 1.0)

    def test_channel_rating_covers_one_out_hover(self):
        power_rotor_W = notional.power_shaft_hover_rotor_W(self.data)
        for channels in range(2, 7):
            topology = notional.build_notional_all_electric_tiltrotor(channels, self.data)
            motor = topology.instances["motor"].component
            state = degraded_state(topology, notional.OneChannelOut(active_lane_count=channels - 1))
            self.assertAlmostEqual(motor.power_rated_W, state.power_shaft_lane_W(power_rotor_W))

    def test_installed_power_falls_as_n_over_n_minus_one(self):
        installed = {}
        for channels in range(2, 7):
            topology = notional.build_notional_all_electric_tiltrotor(channels, self.data)
            installed[channels] = topology.instances["motor"].count * topology.instances["motor"].component.power_rated_W
        power_rotor_W = notional.power_shaft_hover_rotor_W(self.data)
        for channels, power_W in installed.items():
            self.assertAlmostEqual(power_W / (2 * power_rotor_W), channels / (channels - 1))
        self.assertEqual(sorted(installed.values(), reverse=True), [installed[n] for n in range(2, 7)])

    def test_design_margins_finite_and_symbolic_ratings(self):
        margins = design_margins(notional.build_notional_all_electric_tiltrotor(3, self.data))
        self.assertTrue(all(abs(float(margin.value)) < 1e3 for margin in margins))
        opti = asb.Opti()
        power_rated_W = opti.variable(init_guess=80e3, scale=1e5)
        topology = build_all_electric(Motor(power_rated_W=power_rated_W), Battery(), ActuatorDiskPropulsor(),
                                      count_channels_per_rotor=3)
        margins = {m.label: m.value for m in design_margins(topology)}
        opti.subject_to(margins["motor.shaft->propulsor.shaft max_power_W"] == 0)
        solution = opti.solve(verbose=False)
        # Three channels on a 100 kW rotor (ActuatorDiskPropulsor default) -> 33.3 kW each.
        self.assertAlmostEqual(solution(power_rated_W), 100e3 / 3, places=2)

    def test_residuals_symbolic(self):
        topology = notional.build_notional_all_electric_tiltrotor(4, self.data)
        opti = asb.Opti()
        values = notional.hover_port_values(topology, self.data, opti.variable(init_guess=790.0),
                                            opti.variable(init_guess=400.0))
        residuals = connection_residuals(topology, values)
        self.assertTrue(any(hasattr(r.value, "is_zero") for r in residuals))


if __name__ == "__main__":
    unittest.main()
