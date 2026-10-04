"""Port declarations for Tier 1 components, kept outside the component classes.

Signs follow the Tier 1 conventions: motor current and generator current are
positive in their working quadrants, battery current is positive on discharge,
and gearbox/propulsor torque is positive in forward power transfer.
"""
from aircraft_closure.core.ports import Direction, Domain, PortSpec
from .components.battery import Battery
from .components.battery_ecm import EquivalentCircuitBattery
from .components.cable import Cable
from .components.converters import DcDcConverter, Inverter
from .components.gearbox import Gearbox
from .components.generator import Generator
from .components.motor import Motor
from .components.protection import ProtectionUnit
from .components.propulsor import ActuatorDiskPropulsor
from .components.rotor import MomentumProfileRotor
from .components.turboshaft import SimpleTurboshaft

_port_specs_by_type = {
    Motor: (PortSpec("electrical", Domain.ELECTRICAL, Direction.IN),
            PortSpec("shaft", Domain.MECHANICAL, Direction.OUT)),
    Generator: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),
                PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT)),
    Battery: (PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT),),
    EquivalentCircuitBattery: (PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT),),
    SimpleTurboshaft: (PortSpec("fuel", Domain.FUEL, Direction.IN),
                       PortSpec("shaft", Domain.MECHANICAL, Direction.OUT)),
    Gearbox: (PortSpec("shaft_in", Domain.MECHANICAL, Direction.IN),
              PortSpec("shaft_out", Domain.MECHANICAL, Direction.OUT)),
    ActuatorDiskPropulsor: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),),
    MomentumProfileRotor: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),),
    # Tier 15 electrical layer: two-ports named in the nominal power-flow direction. A generator's active
    # rectifier is an Inverter with `rectifier_port_specs()`.
    Inverter: (PortSpec("dc", Domain.ELECTRICAL, Direction.IN),
               PortSpec("ac", Domain.ELECTRICAL, Direction.OUT)),
    Cable: (PortSpec("input", Domain.ELECTRICAL, Direction.IN),
            PortSpec("output", Domain.ELECTRICAL, Direction.OUT)),
    ProtectionUnit: (PortSpec("input", Domain.ELECTRICAL, Direction.IN),
                     PortSpec("output", Domain.ELECTRICAL, Direction.OUT)),
    DcDcConverter: (PortSpec("input", Domain.ELECTRICAL, Direction.IN),
                    PortSpec("output", Domain.ELECTRICAL, Direction.OUT)),
}


def rectifier_port_specs():
    """An Inverter operated as an active rectifier: AC in from a generator, DC out to the bus."""
    return (PortSpec("ac", Domain.ELECTRICAL, Direction.IN), PortSpec("dc", Domain.ELECTRICAL, Direction.OUT))


def port_specs_for(component):
    """Ports of a component instance; subclasses inherit their parent's ports."""
    for component_type in type(component).__mro__:
        if component_type in _port_specs_by_type:
            return _port_specs_by_type[component_type]
    raise TypeError(f"No port declaration for {type(component).__name__}.")
