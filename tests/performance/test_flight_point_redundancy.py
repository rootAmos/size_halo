"""Tier 18 (plan 032): degraded flight points (lane out, bus out, string out) on the four-rotor reference."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import HoverSegment
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point
from aircraft_closure.powertrain.components.cable import Cable
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.topologies import RedundancyLayer, build_series_hybrid
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance
from examples.aircraft_mass_closure import build_reference_aircraft

base = build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92)
aero = SimpleAerodynamics()
mass_kg = 1553.0
hover = FlightCondition(mode="hover", altitude_m=0.0, label="hover")
string_contactor = ProtectionUnit(max_current_A=400.0)
tie_contactor, tie_cable = ProtectionUnit(max_current_A=600.0), Cable(length_m=2.0, max_current_A=600.0)


def lane_of(motor, count_lanes):
    """One of `count_lanes` lane motors sharing the reference motor's torque (rubber scaling)."""
    return replace(motor, power_rated_W=motor.power_rated_W / count_lanes, max_torque_Nm=motor.max_torque_Nm / count_lanes,
                   loss_model=replace(motor.loss_model, torque_peak_efficiency_Nm=motor.loss_model.torque_peak_efficiency_Nm
                                      / count_lanes))


def redundant_aircraft(count_lanes=1, count_buses=1, count_strings=1, plain=False):
    instances = {name: i.component for name, i in base.powertrain.topology.instances.items()}
    redundancy = None if plain else RedundancyLayer(
        count_lanes=count_lanes, count_buses=count_buses, count_strings_battery=count_strings,
        protection_string=string_contactor if count_strings > 1 else None,
        protection_bus_tie=tie_contactor if count_buses > 1 else None,
        cable_bus_tie=tie_cable if count_buses > 1 else None)
    topology = build_series_hybrid(lane_of(instances["motor"], count_lanes), instances["generator"],
                                   instances["battery"], instances["turboshaft"], instances["gearbox"],
                                   instances["propulsor"], count_rotors=4, redundancy=redundancy)
    located = {location.instance_name: location for location in base.powertrain.locations}
    locations = tuple(located.get(name, InstalledInstance(name, x_m=2.6, z_m=-0.3)) for name in topology.instances)
    return replace(base, powertrain=replace(base.powertrain, topology=topology, locations=locations))


def solve(aircraft, condition=hover, hybridization=0.2):
    opti = asb.Opti()
    point = build_flight_point(opti, aircraft, aero, condition, mass_kg, hybridization)
    opti.minimize(point.fuel_flow_kg_s * 100)
    return point, opti.solve(verbose=False)


def number(solution, expression):
    return float(solution.value(expression))


class LaneTests(unittest.TestCase):
    def test_all_ones_equals_the_plain_point(self):
        plain, s0 = solve(redundant_aircraft(plain=True))
        ones, s1 = solve(redundant_aircraft())
        self.assertIsNone(plain.redundancy)
        self.assertIsNone(ones.redundancy)
        self.assertEqual(number(s0, plain.fuel_flow_kg_s), number(s1, ones.fuel_flow_kg_s))
        self.assertEqual([m.label for m in plain.margins], [m.label for m in ones.margins])

    def test_healthy_lanes_share_torque_and_match_one_machine(self):
        """Rubber lanes at 1/N of the torque: each carries 1/N of the gearbox torque and the losses add up to
        exactly one machine's (McDonald coefficients scale with the rating)."""
        single, s1 = solve(redundant_aircraft(plain=True))
        for lanes in (2, 3):
            point, s = solve(redundant_aircraft(count_lanes=lanes))
            self.assertAlmostEqual(number(s, point.torque_motor_Nm * lanes),
                                   number(s, point.redundancy.torque_gearbox_input_Nm), places=9)
            self.assertAlmostEqual(number(s, point.torque_motor_Nm * lanes), number(s1, single.torque_motor_Nm),
                                   places=6)
            self.assertAlmostEqual(number(s, point.power_electric_motors_W) / number(s1, single.power_electric_motors_W),
                                   1.0, places=9)
            loads = {load.source: load for load in point.heat_loads}
            self.assertEqual(loads["motor"].count, 4 * lanes)

    def test_lane_out_raises_lane_torque(self):
        healthy, s0 = solve(redundant_aircraft(count_lanes=2))
        out, s1 = solve(redundant_aircraft(count_lanes=2), replace(hover, active_lane_count=1, label="lane out"))
        self.assertEqual(out.redundancy.count_lanes_active, 1)
        # The one active lane carries the whole gearbox input (rotor speed is free, so compare shaft powers).
        self.assertEqual(number(s1, out.torque_motor_Nm), number(s1, out.redundancy.torque_gearbox_input_Nm))
        power_lane_out_W = number(s1, out.speed_motor_rad_s * out.torque_motor_Nm)
        power_lane_healthy_W = number(s0, healthy.speed_motor_rad_s * healthy.torque_motor_Nm)
        self.assertAlmostEqual(power_lane_out_W / power_lane_healthy_W, 2.0, delta=0.05)
        self.assertAlmostEqual(power_lane_out_W, number(s1, out.power_shaft_rotor_W) / 0.97, places=3)
        # The same rotor power through half the copper: more loss, more electrical power.
        self.assertGreater(number(s1, out.power_electric_motors_W), number(s0, healthy.power_electric_motors_W))
        self.assertEqual({load.source: load.count for load in out.heat_loads}["motor"], 4)
        torque = {m.label: number(s1, m.value) for m in out.margins}["lane out: motor torque_Nm"]
        lane = redundant_aircraft(count_lanes=2).powertrain.topology.instances["motor"].component
        self.assertAlmostEqual(torque, 1 - number(s1, out.torque_motor_Nm) / lane.max_torque_Nm, places=9)


class BusTests(unittest.TestCase):
    def test_normal_operation_ties_open(self):
        point, s = solve(redundant_aircraft(count_lanes=2, count_buses=2))
        self.assertEqual(point.redundancy.current_tie_A, 0.0)
        self.assertFalse(any("bus_tie" in m.label for m in point.margins))
        self.assertFalse(any("bus_tie" in load.source for load in point.heat_loads))

    def test_bus_out_loses_its_lanes_and_feeds_through_the_tie(self):
        aircraft = redundant_aircraft(count_lanes=2, count_buses=2)
        point, s = solve(aircraft, replace(hover, count_buses_failed=1, label="bus out"))
        self.assertEqual(point.redundancy.count_lanes_active, 1)
        voltage_V = number(s, point.battery.voltage_V)
        motors_W = number(s, point.power_electric_motors_W)
        current_A = number(s, point.redundancy.current_tie_A)
        self.assertAlmostEqual(current_A, motors_W / (2 * voltage_V), places=6)
        loss_W = number(s, point.redundancy.power_loss_tie_W)
        resistance_ohm = tie_contactor.resistance_ohm() + tie_cable.resistance_ohm()
        self.assertAlmostEqual(loss_W, current_A**2 * resistance_ohm, places=6)
        # The sources deliver the motors' demand plus the tie loss.
        supplied_W = number(s, point.battery.power_electric_W + 1 * point.generator.power_electric_W)
        self.assertAlmostEqual(supplied_W, motors_W + loss_W, places=3)
        labels = {m.label for m in point.margins}
        self.assertIn("bus out: protection_bus_tie current_A (squared)", labels)
        self.assertIn("bus out: cable_bus_tie partial_discharge_V", labels)
        self.assertEqual({load.source for load in point.heat_loads} & {"protection_bus_tie", "cable_bus_tie"},
                         {"protection_bus_tie", "cable_bus_tie"})


class StringTests(unittest.TestCase):
    def test_string_contactors_in_series(self):
        point, s = solve(redundant_aircraft(count_strings=2))
        current_A = number(s, point.battery.current_A)
        self.assertAlmostEqual(number(s, point.redundancy.current_string_A), current_A / 2, places=9)
        loss_W = number(s, point.redundancy.power_loss_strings_W)
        self.assertAlmostEqual(loss_W, 2 * string_contactor.resistance_ohm() * (current_A / 2)**2, places=9)
        # Bus side = terminal power less the contactor loss.
        bus_W = number(s, point.power_electric_motors_W) * 0.2
        self.assertAlmostEqual(bus_W, number(s, point.battery.power_electric_W) - loss_W, places=3)

    def test_string_out_uses_the_remaining_strings(self):
        aircraft = redundant_aircraft(count_strings=2)
        normal, s0 = solve(aircraft)
        out, s1 = solve(aircraft, replace(hover, active_battery_string_count=1, label="string out"))
        battery = aircraft.powertrain.topology.instances["battery"].component
        margins = {m.label: number(s1, m.value) for m in out.margins}
        power_W = number(s1, out.battery.power_electric_W)
        self.assertAlmostEqual(margins["string out: battery discharge_power_W"],
                               1 - power_W / (0.5 * battery.max_discharge_power_W), places=9)
        # Half the conductance: a deeper terminal sag at about the same power.
        self.assertLess(number(s1, out.battery.voltage_V), number(s0, normal.battery.voltage_V))
        self.assertAlmostEqual(number(s1, out.redundancy.current_string_A), number(s1, out.battery.current_A),
                               places=9)

    def test_string_out_mission_depletes_the_remaining_strings(self):
        aircraft = redundant_aircraft(count_strings=2)
        ends = []
        for strings in (None, 1):
            opti = asb.Opti()
            flown = build_mission(opti, aircraft, aero, Mission((HoverSegment(
                60.0, 0.0, 0.5, "hover", active_battery_string_count=strings),)), mass_kg, 0.9)
            opti.minimize(flown.mass_fuel_burnt_kg)
            s = opti.solve(verbose=False)
            ends.append((number(s, flown.soc_end), number(s, flown.energy_battery_chemical_J)))
        (soc_all, energy_all_J), (soc_one, energy_one_J) = ends
        capacity_J = aircraft.powertrain.topology.instances["battery"].component.energy_capacity_J
        self.assertAlmostEqual(0.9 - soc_all, energy_all_J / capacity_J, places=9)
        self.assertAlmostEqual(0.9 - soc_one, energy_one_J / (0.5 * capacity_J), places=9)


if __name__ == "__main__":
    unittest.main()
