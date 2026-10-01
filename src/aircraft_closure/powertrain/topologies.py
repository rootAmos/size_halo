"""Reference powertrain topologies; descriptions only, never solved here."""
from aircraft_closure.core.topology import Topology
from .ports import port_specs_for


def build_series_hybrid(motor, generator, battery, turboshaft, gearbox, propulsor, count_rotors=1):
    """Turboshaft -> generator -> bus <- battery; bus -> n x (motor -> gearbox -> rotor).

    The turboshaft fuel port is left unconnected as a boundary port.
    """
    topology = Topology()
    topology.add("turboshaft", turboshaft, port_specs_for(turboshaft))
    topology.add("generator", generator, port_specs_for(generator))
    topology.add("battery", battery, port_specs_for(battery))
    topology.add_bus("bus")
    topology.add("motor", motor, port_specs_for(motor), count=count_rotors)
    topology.add("gearbox", gearbox, port_specs_for(gearbox), count=count_rotors)
    topology.add("propulsor", propulsor, port_specs_for(propulsor), count=count_rotors)
    topology.connect("turboshaft.shaft", "generator.shaft")
    topology.connect("generator.electrical", "bus")
    topology.connect("battery.electrical", "bus")
    topology.connect("motor.electrical", "bus")
    topology.connect("motor.shaft", "gearbox.shaft_in")
    topology.connect("gearbox.shaft_out", "propulsor.shaft")
    return topology
