import unittest

import aerosandbox as asb
import casadi as cas

from aircraft_closure.core.ports import (Direction, Domain, ElectricalPortValue, FuelPortValue,
                                         MechanicalPortValue, PortSpec)
from aircraft_closure.core.topology import Topology, connection_residuals

shaft_out = PortSpec("shaft", Domain.MECHANICAL, Direction.OUT)
shaft_in = PortSpec("shaft", Domain.MECHANICAL, Direction.IN)
electrical_out = PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT)
electrical_in = PortSpec("electrical", Domain.ELECTRICAL, Direction.IN)
fuel_in = PortSpec("fuel", Domain.FUEL, Direction.IN)
fuel_out = PortSpec("fuel", Domain.FUEL, Direction.OUT)


def residual_map(residuals):
    return {residual.label: residual.value for residual in residuals}


class TopologyStructureTests(unittest.TestCase):
    def test_direct_connection_is_normalized_out_to_in(self):
        topology = Topology()
        topology.add("driver", object(), (shaft_out,))
        topology.add("load", object(), (shaft_in,))
        connection = topology.connect("load.shaft", "driver.shaft")
        self.assertEqual((connection.source, connection.target), ("driver.shaft", "load.shaft"))

    def test_domain_mismatch_rejected(self):
        topology = Topology()
        topology.add("driver", object(), (shaft_out,))
        topology.add("load", object(), (electrical_in,))
        with self.assertRaisesRegex(ValueError, "Domain mismatch"):
            topology.connect("driver.shaft", "load.electrical")

    def test_same_direction_rejected(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        topology.add("b", object(), (shaft_out,))
        with self.assertRaisesRegex(ValueError, "one OUT and one IN"):
            topology.connect("a.shaft", "b.shaft")

    def test_duplicate_and_invalid_names_rejected(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        with self.assertRaisesRegex(ValueError, "already used"):
            topology.add("a", object(), (shaft_in,))
        with self.assertRaisesRegex(ValueError, "already used"):
            topology.add_bus("a")
        with self.assertRaisesRegex(ValueError, "must not contain"):
            topology.add("b.c", object(), (shaft_in,))
        with self.assertRaisesRegex(ValueError, "duplicate port"):
            topology.add("d", object(), (shaft_in, shaft_out))

    def test_count_must_be_positive_integer(self):
        topology = Topology()
        for count in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                topology.add(f"x{count!s}".replace(".", "_").replace("-", "m"), object(), (shaft_in,), count=count)

    def test_unknown_port_or_instance_rejected(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        topology.add("b", object(), (shaft_in,))
        with self.assertRaises(KeyError):
            topology.connect("a.missing", "b.shaft")
        with self.assertRaises(KeyError):
            topology.connect("ghost.shaft", "b.shaft")
        with self.assertRaises(KeyError):
            topology.connect("a", "b.shaft")

    def test_shaft_port_connects_once(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        topology.add("b", object(), (shaft_in,))
        topology.add("c", object(), (shaft_in,))
        topology.connect("a.shaft", "b.shaft")
        with self.assertRaisesRegex(ValueError, "already connected"):
            topology.connect("a.shaft", "c.shaft")

    def test_count_mismatch_on_direct_connection_rejected(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,), count=1)
        topology.add("b", object(), (shaft_in,), count=2)
        with self.assertRaisesRegex(ValueError, "splitter"):
            topology.connect("a.shaft", "b.shaft")

    def test_bus_accepts_only_electrical_ports(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        topology.add_bus("bus")
        with self.assertRaisesRegex(ValueError, "electrical bus"):
            topology.connect("a.shaft", "bus")
        with self.assertRaisesRegex(ValueError, "Only electrical"):
            topology.add_bus("shaft_bus", Domain.MECHANICAL)

    def test_topology_holds_no_symbolic_state(self):
        topology = Topology()
        topology.add("a", object(), (electrical_out,))
        topology.add_bus("bus")
        topology.connect("a.electrical", "bus")
        self.assertEqual(set(vars(topology)), {"_instances", "_buses", "_connections", "_connected_ports"})


class ConnectionResidualTests(unittest.TestCase):
    def bus_topology(self, count_load=1):
        topology = Topology()
        topology.add("source", object(), (electrical_out,))
        topology.add("storage", object(), (electrical_out,))
        topology.add("load", object(), (electrical_in,), count=count_load)
        topology.add_bus("bus")
        for name in ("source", "storage", "load"):
            topology.connect(f"{name}.electrical", "bus")
        return topology

    def test_shaft_residuals_are_exact_differences(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,))
        topology.add("b", object(), (shaft_in,))
        topology.connect("a.shaft", "b.shaft")
        residuals = residual_map(connection_residuals(topology, {
            "a.shaft": MechanicalPortValue(400, 210), "b.shaft": MechanicalPortValue(390, 200)}))
        self.assertEqual(residuals, {"a.shaft->b.shaft speed_rad_s": 10, "a.shaft->b.shaft torque_Nm": 10})

    def test_balanced_bus_has_zero_residuals(self):
        residuals = residual_map(connection_residuals(self.bus_topology(), {
            "source.electrical": ElectricalPortValue(800, 80),
            "storage.electrical": ElectricalPortValue(800, 20),
            "load.electrical": ElectricalPortValue(800, 100)}))
        self.assertTrue(all(value == 0 for value in residuals.values()))
        self.assertEqual(len(residuals), 3)

    def test_unbalanced_bus_reports_exact_imbalance(self):
        residuals = residual_map(connection_residuals(self.bus_topology(), {
            "source.electrical": ElectricalPortValue(800, 80),
            "storage.electrical": ElectricalPortValue(795, 25),
            "load.electrical": ElectricalPortValue(800, 100)}))
        self.assertEqual(residuals["bus current_A"], 5)
        self.assertEqual(residuals["bus storage.electrical voltage_V"], -5)
        self.assertEqual(residuals["bus load.electrical voltage_V"], 0)

    def test_multiplicity_scales_bus_current(self):
        residuals = residual_map(connection_residuals(self.bus_topology(count_load=4), {
            "source.electrical": ElectricalPortValue(800, 320),
            "storage.electrical": ElectricalPortValue(800, 80),
            "load.electrical": ElectricalPortValue(800, 100)}))
        self.assertEqual(residuals["bus current_A"], 0)

    def test_negative_storage_current_is_charging(self):
        # Source feeds the load and charges storage: storage current is negative.
        residuals = residual_map(connection_residuals(self.bus_topology(), {
            "source.electrical": ElectricalPortValue(800, 130),
            "storage.electrical": ElectricalPortValue(800, -30),
            "load.electrical": ElectricalPortValue(800, 100)}))
        self.assertEqual(residuals["bus current_A"], 0)

    def test_fuel_connection_residual(self):
        topology = Topology()
        topology.add("tank", object(), (fuel_out,))
        topology.add("engine", object(), (fuel_in,))
        topology.connect("tank.fuel", "engine.fuel")
        residuals = residual_map(connection_residuals(topology, {
            "tank.fuel": FuelPortValue(0.006), "engine.fuel": FuelPortValue(0.005)}))
        self.assertAlmostEqual(residuals["tank.fuel->engine.fuel fuel_flow_kg_s"], 0.001)

    def test_unconnected_ports_need_no_values(self):
        topology = Topology()
        topology.add("engine", object(), (fuel_in, shaft_out))
        self.assertEqual(connection_residuals(topology, {}), [])

    def test_missing_or_wrong_type_value_rejected(self):
        topology = self.bus_topology()
        values = {"source.electrical": ElectricalPortValue(800, 80),
                  "storage.electrical": ElectricalPortValue(800, 20)}
        with self.assertRaisesRegex(KeyError, "load.electrical"):
            connection_residuals(topology, values)
        values["load.electrical"] = MechanicalPortValue(400, 200)
        with self.assertRaisesRegex(TypeError, "ElectricalPortValue"):
            connection_residuals(topology, values)

    def test_residuals_with_opti_variables_solve(self):
        # Toy cyclic network: source and storage share a load through one bus.
        opti = asb.Opti()
        current_source_A = opti.variable(init_guess=10)
        current_storage_A = opti.variable(init_guess=10)
        voltage_storage_V = opti.variable(init_guess=700)
        residuals = connection_residuals(self.bus_topology(count_load=3), {
            "source.electrical": ElectricalPortValue(800, current_source_A),
            "storage.electrical": ElectricalPortValue(voltage_storage_V, current_storage_A),
            "load.electrical": ElectricalPortValue(800, 50)})
        self.assertTrue(all(isinstance(r.value, cas.MX) for r in residuals if r.label != "bus load.electrical voltage_V"))
        opti.subject_to([r.value == 0 for r in residuals])
        opti.subject_to(current_storage_A == 0.25 * 150)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(current_source_A)), 112.5, places=6)
        self.assertAlmostEqual(float(solution.value(voltage_storage_V)), 800, places=6)


class CombinerSplitterTests(unittest.TestCase):
    """Tier 18: direct connections between counts (lane motors on one gearbox input, pack into strings)."""

    def lanes(self, count_lanes=2, count_rotors=2):
        topology = Topology()
        topology.add("motor", object(), (shaft_out,), count=count_rotors * count_lanes)
        topology.add("gearbox", object(), (shaft_in,), count=count_rotors)
        topology.connect("motor.shaft", "gearbox.shaft", combine=True)
        return topology

    def test_combiner_needs_flag_and_a_multiple(self):
        topology = Topology()
        topology.add("a", object(), (shaft_out,), count=3)
        topology.add("b", object(), (shaft_in,), count=2)
        with self.assertRaisesRegex(ValueError, "multiple"):
            topology.connect("a.shaft", "b.shaft", combine=True)
        self.assertTrue(self.lanes().connections[0].combine)

    def test_combiner_sums_torque_at_equal_speed(self):
        residuals = residual_map(connection_residuals(self.lanes(count_lanes=2), {
            "motor.shaft": MechanicalPortValue(400, 150), "gearbox.shaft": MechanicalPortValue(400, 300)}))
        self.assertEqual(residuals, {"motor.shaft->gearbox.shaft speed_rad_s": 0,
                                     "motor.shaft->gearbox.shaft torque_Nm (total)": 0})
        residuals = residual_map(connection_residuals(self.lanes(count_lanes=3), {
            "motor.shaft": MechanicalPortValue(400, 100), "gearbox.shaft": MechanicalPortValue(390, 290)}))
        self.assertEqual(residuals["motor.shaft->gearbox.shaft speed_rad_s"], 10)
        self.assertEqual(residuals["motor.shaft->gearbox.shaft torque_Nm (total)"], 6 * 100 - 2 * 290)

    def test_splitter_conserves_current(self):
        topology = Topology()
        topology.add("battery", object(), (electrical_out,))
        topology.add("string", object(), (electrical_in,), count=4)
        topology.connect("battery.electrical", "string.electrical", combine=True)
        residuals = residual_map(connection_residuals(topology, {
            "battery.electrical": ElectricalPortValue(700, 200), "string.electrical": ElectricalPortValue(700, 50)}))
        self.assertEqual(residuals["battery.electrical->string.electrical current_A (total)"], 0)
        self.assertEqual(residuals["battery.electrical->string.electrical voltage_V"], 0)

    def test_bus_count(self):
        topology = Topology()
        topology.add_bus("bus", count=2)
        self.assertEqual(topology.buses["bus"].count, 2)
        with self.assertRaisesRegex(ValueError, "positive integer"):
            topology.add_bus("other", count=0)
        self.assertEqual(Topology().add_bus("bus") and 1, 1)

    def test_combiner_residual_symbolic(self):
        opti = asb.Opti()
        torque_lane_Nm = opti.variable(init_guess=10.0)
        residuals = connection_residuals(self.lanes(count_lanes=2), {
            "motor.shaft": MechanicalPortValue(400, torque_lane_Nm), "gearbox.shaft": MechanicalPortValue(400, 300)})
        opti.subject_to([r.value == 0 for r in residuals if isinstance(r.value, cas.MX)])
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(torque_lane_Nm)), 150.0, places=6)


if __name__ == "__main__":
    unittest.main()
