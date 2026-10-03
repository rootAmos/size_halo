"""Port declarations for Tier 1 components, kept outside the component classes.

Signs follow the Tier 1 conventions: motor current and generator current are
positive in their working quadrants, battery current is positive on discharge,
and gearbox/propulsor torque is positive in forward power transfer.
"""
from aircraft_closure.core.ports import Direction, Domain, PortSpec
from .components.battery import Battery
from .components.gearbox import Gearbox
from .components.generator import Generator
from .components.motor import Motor
from .components.propulsor import ActuatorDiskPropulsor
from .components.rotor import MomentumProfileRotor
from .components.turboshaft import SimpleTurboshaft

_port_specs_by_type = {
    Motor: (PortSpec("electrical", Domain.ELECTRICAL, Direction.IN),
            PortSpec("shaft", Domain.MECHANICAL, Direction.OUT)),
    Generator: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),
                PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT)),
    Battery: (PortSpec("electrical", Domain.ELECTRICAL, Direction.OUT),),
    SimpleTurboshaft: (PortSpec("fuel", Domain.FUEL, Direction.IN),
                       PortSpec("shaft", Domain.MECHANICAL, Direction.OUT)),
    Gearbox: (PortSpec("shaft_in", Domain.MECHANICAL, Direction.IN),
              PortSpec("shaft_out", Domain.MECHANICAL, Direction.OUT)),
    ActuatorDiskPropulsor: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),),
    MomentumProfileRotor: (PortSpec("shaft", Domain.MECHANICAL, Direction.IN),),
}


def port_specs_for(component):
    """Ports of a component instance; subclasses inherit their parent's ports."""
    for component_type in type(component).__mro__:
        if component_type in _port_specs_by_type:
            return _port_specs_by_type[component_type]
    raise TypeError(f"No port declaration for {type(component).__name__}.")
