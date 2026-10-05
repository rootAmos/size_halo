"""Tier 18 (plan 032): the Halo with lane motors, cross-strapped buses, battery strings and failure hovers."""
import unittest
from dataclasses import replace

from examples.halo_sizing import (HaloAssumptions, assumptions_tier18, build_halo_aircraft, count_lanes_motor,
                                  failure_hover_cases, solve_halo_sizing)
from tests.integration.test_halo_thermal import numeric_design

# Plan 035 changed the defaults; these tests reproduce their tier on the plan 030 settings.
from functools import partial  # noqa: E402
from examples.halo_sizing import pre_plan035  # noqa: E402
HaloAssumptions = partial(HaloAssumptions, **pre_plan035)
build_halo_aircraft = partial(build_halo_aircraft, assumptions=HaloAssumptions())
solve_halo_sizing = partial(solve_halo_sizing, assumptions=HaloAssumptions())

ones = HaloAssumptions(redundancy=True, count_lanes_motor=1, count_buses=1, count_strings_battery=1,
                       failure_lane_out=False, failure_bus_out=False, failure_string_out=False)


def masses(aircraft):
    return {item.instance_name: float(item.mass_properties.mass)
            for item in aircraft.powertrain.get_instance_mass_properties()}


class HaloRedundancySwitchTests(unittest.TestCase):
    def test_off_by_default_plain_topology(self):
        self.assertFalse(HaloAssumptions().redundancy)
        self.assertEqual(count_lanes_motor(HaloAssumptions()), 1)
        topology = build_halo_aircraft(numeric_design()).powertrain.topology
        self.assertEqual(topology.instances["motor"].count, 2)
        self.assertEqual(topology.buses["bus"].count, 1)
        self.assertFalse({"protection_string", "protection_bus_tie", "cable_bus_tie"} & set(topology.instances))

    def test_all_ones_is_the_plain_aircraft(self):
        plain = build_halo_aircraft(numeric_design())
        same = build_halo_aircraft(numeric_design(), assumptions=ones)
        self.assertEqual(masses(plain), masses(same))
        self.assertEqual(plain.powertrain.topology.connections, same.powertrain.topology.connections)

    def test_sensible_set_hardware(self):
        design = replace(numeric_design(), torque_peak_motor_Nm=200.0)          # half of the 400 N.m per lane
        aircraft = build_halo_aircraft(design, assumptions=assumptions_tier18)
        instances = aircraft.powertrain.topology.instances
        self.assertEqual(instances["motor"].count, 4)
        self.assertEqual(instances["protection_string"].count, 2)
        self.assertEqual(instances["protection_bus_tie"].count, 1)
        self.assertEqual(aircraft.powertrain.topology.buses["bus"].count, 2)
        plain = masses(build_halo_aircraft(numeric_design()))
        redundant = masses(aircraft)
        # Two lanes of half the torque weigh one machine (linear torque density), up to the smooth maximum's
        # 2 kg x ln 2 per machine (4 lane motors against 2 machines).
        self.assertLessEqual(abs(redundant["motor"] - plain["motor"]), 4 * 2.0 * 0.6931472)
        for name in ("protection_string", "protection_bus_tie", "cable_bus_tie"):
            self.assertGreater(redundant[name], 0.0)
        # String contactors at 125 % of a string's discharge rating.
        battery = instances["battery"].component
        self.assertAlmostEqual(float(instances["protection_string"].component.max_current_A),
                               float(1.25 * battery.get_limits().max_discharge_current_A / 2))

    def test_bus_out_covers_lane_out_with_one_lane_per_bus(self):
        labels = [label for label, _ in failure_hover_cases(assumptions_tier18)]
        self.assertEqual(labels, ["bus-out hover", "string-out hover"])
        four = replace(assumptions_tier18, count_lanes_motor=4)
        self.assertEqual([label for label, _ in failure_hover_cases(four)][:2], ["lane-out hover", "bus-out hover"])

    def test_a_case_needs_something_to_fail_onto(self):
        with self.assertRaisesRegex(ValueError, "lane-out"):
            solve_halo_sizing(assumptions=replace(ones, failure_lane_out=True), max_iter=1)


class HaloRedundancySizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = solve_halo_sizing()                          # plan 030 reference (14,037 lb)
        cls.limit = solve_halo_sizing(assumptions=ones, initial=cls.reference)
        cls.redundant = solve_halo_sizing(assumptions=assumptions_tier18, initial=cls.reference)

    def test_one_lane_without_cases_is_the_reference(self):
        self.assertAlmostEqual(self.limit.mass_takeoff_kg / self.reference.mass_takeoff_kg, 1.0, places=5)
        self.assertEqual(self.limit.failure_cases, ())
        self.assertEqual(self.limit.count_lanes_motor, 1)

    def test_redundant_aircraft_closes_with_every_case(self):
        r = self.redundant
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-3)
        # One lane per rotor on each bus: the bus out covers the lane out (same lanes lost, plus the tie).
        self.assertEqual([c["label"] for c in r.failure_cases], ["bus-out hover", "string-out hover"])
        for case in r.failure_cases:
            self.assertGreater(case["min_margin"], -1e-6)
            self.assertGreaterEqual(case["soc_end"], 0.1 - 1e-6)

    def test_failure_states(self):
        cases = {c["label"]: c for c in self.redundant.failure_cases}
        self.assertEqual(cases["bus-out hover"]["count_lanes_active"], 1)
        self.assertEqual(cases["bus-out hover"]["count_buses_failed"], 1)
        self.assertGreater(cases["bus-out hover"]["current_tie_A"], 0.0)
        self.assertEqual(cases["string-out hover"]["current_tie_A"], 0.0)
        self.assertEqual(cases["string-out hover"]["count_lanes_active"], 2)
        self.assertEqual(cases["string-out hover"]["count_strings_active"], 1)
        # With thermal on, a lane may exceed its continuous rating for the 60 s hover but stays in its torque limit.
        for case in cases.values():
            self.assertLessEqual(case["torque_lane_to_max"], 1 + 1e-6)
            self.assertLessEqual(case["temperature_end_motor_C"], 150.0 + 1e-4)

    def test_trends(self):
        """Lanes: lower per-lane rating, more installed motor rating and a heavier aircraft."""
        r, ref = self.redundant, self.reference
        self.assertEqual(r.count_lanes_motor, 2)
        self.assertLess(r.power_rated_motor_W, ref.power_rated_motor_W)
        self.assertGreater(2 * r.power_rated_motor_W, ref.power_rated_motor_W)
        self.assertGreater(r.mass_takeoff_kg, ref.mass_takeoff_kg)
        motors = dict(r.powertrain_masses_kg)["motor"]
        self.assertGreater(motors, dict(ref.powertrain_masses_kg)["motor"])

    def test_heat_exchanger_covers_failure_points(self):
        rows = [row for row in self.redundant.thermal_trace if "bus-out" in row["label"]]
        self.assertTrue(rows)
        rating_W = self.redundant.heat_exchanger["power_rated_W"]
        for row in self.redundant.thermal_trace:
            self.assertLessEqual(row["power_heat_equivalent_W"], rating_W * (1 + 1e-6))


if __name__ == "__main__":
    unittest.main()
