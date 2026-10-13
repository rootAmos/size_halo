"""Plan 040: the per-bus engine-out hover (two buses, a turbogenerator lost on bus A, tie open or closed)."""
import unittest

import aerosandbox as asb

from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import HoverSegment
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.topologies import RedundancyLayer, build_series_hybrid
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance
from dataclasses import replace
from tests.performance.test_flight_point_redundancy import (aero, base, lane_of, mass_kg, number, string_contactor,
                                                            tie_cable, tie_contactor)

pack = EquivalentCircuitBattery(count_series=210, count_parallel=12.0, factor_power_density=5.0)


def two_bus_aircraft(count_buses=2):
    instances = {name: i.component for name, i in base.powertrain.topology.instances.items()}
    redundancy = RedundancyLayer(count_lanes=2, count_buses=count_buses, count_strings_battery=2,
                                 protection_string=string_contactor,
                                 protection_bus_tie=tie_contactor if count_buses > 1 else None,
                                 cable_bus_tie=tie_cable if count_buses > 1 else None)
    topology = build_series_hybrid(lane_of(instances["motor"], 2), instances["generator"], pack,
                                   instances["turboshaft"], instances["gearbox"], instances["propulsor"],
                                   count_rotors=4, count_turbogenerators=2, redundancy=redundancy)
    located = {location.instance_name: location for location in base.powertrain.locations}
    locations = tuple(located.get(name, InstalledInstance(name, x_m=2.6, z_m=-0.3)) for name in topology.instances)
    return replace(base, powertrain=replace(base.powertrain, topology=topology, locations=locations))


aircraft = two_bus_aircraft()


def engine_out(bus_tie, **kwargs):
    return FlightCondition(mode="hover", altitude_m=0.0, active_generator_count=1, soc=0.6, bus_tie=bus_tie,
                           label=f"engine-out {bus_tie}", **kwargs)


def solve_point(condition, plane=aircraft):
    """Battery used only as needed (least chemical power, no recharging), every margin held, as in a reserve."""
    opti = asb.Opti()
    point = build_flight_point(opti, plane, aero, condition, mass_kg, duration_s=20.0)
    opti.subject_to([m.value >= 0 for m in point.margins])
    opti.minimize(point.battery.power_chemical_W / 1e5 if point.split is None else
                  sum(s.battery.power_chemical_W for s in point.split.buses.values()) / 1e5)
    return point, opti.solve(verbose=False)


class SplitPointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.open, cls.s_open = solve_point(engine_out("open"))
        cls.closed, cls.s_closed = solve_point(engine_out("closed"))

    def test_lanes_carry_the_gearbox_torque(self):
        for point, s in ((self.open, self.s_open), (self.closed, self.s_closed)):
            A, B = point.split.buses["A"], point.split.buses["B"]
            self.assertAlmostEqual(number(s, A.torque_motor_Nm + B.torque_motor_Nm),
                                   number(s, point.redundancy.torque_gearbox_input_Nm), places=6)
            self.assertEqual((A.count_generators, B.count_generators), (0, 1))
            self.assertEqual((A.count_motors, B.count_motors), (4, 4))

    def test_each_bus_balances_on_its_own_sources_with_the_tie_open(self):
        s, split = self.s_open, self.open.split
        self.assertEqual(split.current_tie_A, 0.0)
        A, B = split.buses["A"], split.buses["B"]
        self.assertAlmostEqual(number(s, A.power_battery_bus_W) / number(s, A.power_motors_W), 1.0, places=6)
        self.assertAlmostEqual(number(s, B.power_battery_bus_W + B.power_generators_bus_W)
                               / number(s, B.power_motors_W), 1.0, places=6)

    def test_open_tie_shifts_torque_to_the_healthy_bus(self):
        """Bus A has only its string group; the least-battery answer loads the lanes fed by the generator."""
        self.assertLess(number(self.s_open, self.open.split.share_torque_A), 0.5)

    def test_closed_tie_feeds_the_failed_bus_through_its_resistance(self):
        s, split = self.s_closed, self.closed.split
        A, B = split.buses["A"], split.buses["B"]
        resistance_ohm = tie_contactor.resistance_ohm() + tie_cable.resistance_ohm()
        self.assertAlmostEqual(number(s, B.voltage_V - A.voltage_V), number(s, split.current_tie_A) * resistance_ohm,
                               places=6)
        self.assertGreater(number(s, split.current_tie_A), 0.0)
        labels = [m.label for m in self.closed.margins]
        self.assertIn("engine-out closed: protection_bus_tie current_A (squared)", labels)
        self.assertNotIn("engine-out open: protection_bus_tie current_A (squared)", [m.label for m in self.open.margins])

    def test_closed_tie_uses_less_battery_than_open(self):
        """Tie closed, the healthy generator can serve both buses: the battery supplies less."""
        battery = lambda point, s: number(s, point.power_battery_W)
        self.assertLess(battery(self.closed, self.s_closed), battery(self.open, self.s_open))

    def test_per_bus_margins_and_heat_sources(self):
        labels = [m.label for m in self.open.margins]
        for b in ("A", "B"):
            self.assertIn(f"engine-out open: battery {b} min_voltage_V", labels)
            self.assertIn(f"engine-out open: motor {b} torque_Nm", labels)
        self.assertIn("engine-out open: generator B torque_Nm", labels)
        self.assertNotIn("engine-out open: generator A torque_Nm", labels)
        sources = {load.source for load in self.open.heat_loads}
        self.assertTrue({"motor", "motor@A", "battery", "battery@A", "gearbox", "generator"} <= sources)
        self.assertNotIn("generator@A", sources)


class SplitMissionTests(unittest.TestCase):
    def test_string_groups_carry_their_own_soc(self):
        opti = asb.Opti()
        result = build_mission(opti, aircraft, aero, Mission((HoverSegment(60.0, 0.0, None, "engine-out",
                               active_generator_count=1, bus_tie="open"),)), mass_kg, 0.6, subsegments=3,
                               polarization_start="steady")
        points = result.segments[0].subsegments
        opti.minimize(-result.soc_end)
        s = opti.solve(verbose=False)
        by_bus = {b: number(s, v) for b, v in result.soc_end_by_bus.items()}
        self.assertAlmostEqual(number(s, result.soc_end), (by_bus["A"] + by_bus["B"]) / 2, places=9)
        self.assertNotAlmostEqual(by_bus["A"], by_bus["B"], places=4)
        # The chain: each point starts where the previous one ended, per bus.
        for previous, point in zip(points[:-1], points[1:]):
            for b in ("A", "B"):
                self.assertAlmostEqual(number(s, previous.soc_end[b]), number(s, point.soc_start[b]), places=12)
        labels = [m.label for m in result.margins]
        self.assertIn("engine-out 3/3: battery A end voltage_V", labels)
        self.assertIn("engine-out 3/3: battery B end voltage_V", labels)


class ScopeTests(unittest.TestCase):
    def test_unsupported_configurations_raise(self):
        with self.assertRaises(ValueError):
            solve_point(engine_out("open"), plane=two_bus_aircraft(count_buses=1))
        with self.assertRaises(ValueError):
            solve_point(engine_out("ajar"))
        with self.assertRaises(ValueError):
            solve_point(engine_out("open", active_lane_count=1))


if __name__ == "__main__":
    unittest.main()
