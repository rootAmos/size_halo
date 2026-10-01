import unittest

from aircraft_closure.core.ports import Direction, Domain
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.ports import port_specs_for
from aircraft_closure.powertrain.topologies import build_series_hybrid


def components():
    return Motor(), Generator(), Battery(), SimpleTurboshaft(), Gearbox(), ActuatorDiskPropulsor()


class PortDeclarationTests(unittest.TestCase):
    def test_every_component_declares_ports(self):
        for component in components():
            self.assertTrue(port_specs_for(component), type(component).__name__)

    def test_power_flow_directions(self):
        def ports(component):
            return {(p.name, p.domain, p.direction) for p in port_specs_for(component)}
        self.assertEqual(ports(Motor()), {("electrical", Domain.ELECTRICAL, Direction.IN),
                                          ("shaft", Domain.MECHANICAL, Direction.OUT)})
        self.assertEqual(ports(Generator()), {("shaft", Domain.MECHANICAL, Direction.IN),
                                              ("electrical", Domain.ELECTRICAL, Direction.OUT)})
        # Battery current is positive on discharge, i.e. out of the pack.
        self.assertEqual(ports(Battery()), {("electrical", Domain.ELECTRICAL, Direction.OUT)})
        self.assertEqual(ports(SimpleTurboshaft()), {("fuel", Domain.FUEL, Direction.IN),
                                                     ("shaft", Domain.MECHANICAL, Direction.OUT)})
        self.assertEqual(ports(Gearbox()), {("shaft_in", Domain.MECHANICAL, Direction.IN),
                                            ("shaft_out", Domain.MECHANICAL, Direction.OUT)})
        self.assertEqual(ports(ActuatorDiskPropulsor()), {("shaft", Domain.MECHANICAL, Direction.IN)})

    def test_subclass_inherits_ports_and_unknown_rejected(self):
        class TunedMotor(Motor):
            pass
        self.assertEqual(port_specs_for(TunedMotor()), port_specs_for(Motor()))
        with self.assertRaises(TypeError):
            port_specs_for(object())


class SeriesHybridTopologyTests(unittest.TestCase):
    def test_structure(self):
        topology = build_series_hybrid(*components(), count_rotors=3)
        self.assertEqual(set(topology.instances), {"turboshaft", "generator", "battery", "motor",
                                                   "gearbox", "propulsor"})
        self.assertEqual(set(topology.buses), {"bus"})
        counts = {name: instance.count for name, instance in topology.instances.items()}
        self.assertEqual(counts, {"turboshaft": 1, "generator": 1, "battery": 1,
                                  "motor": 3, "gearbox": 3, "propulsor": 3})
        links = {(c.source, c.target) for c in topology.connections}
        self.assertEqual(links, {("turboshaft.shaft", "generator.shaft"),
                                 ("generator.electrical", "bus"), ("battery.electrical", "bus"),
                                 ("motor.electrical", "bus"), ("motor.shaft", "gearbox.shaft_in"),
                                 ("gearbox.shaft_out", "propulsor.shaft")})

    def test_fuel_port_is_a_boundary(self):
        topology = build_series_hybrid(*components())
        referenced = {r for c in topology.connections for r in (c.source, c.target)}
        self.assertNotIn("turboshaft.fuel", referenced)

    def test_components_are_held_not_copied(self):
        parts = components()
        topology = build_series_hybrid(*parts)
        self.assertIs(topology.instances["motor"].component, parts[0])


if __name__ == "__main__":
    unittest.main()
