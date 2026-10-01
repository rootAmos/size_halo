"""Network description only: no variables, constraints, ordering or solves.

A direct connection joins one OUT port to one IN port of the same domain and
instance count. A bus is a junction for any number of electrical ports: all
attached voltages are equal and currents obey Kirchhoff's current law. Cycles
are allowed; `connection_residuals` returns expressions the caller constrains
to zero in its own `asb.Opti`.

An instance with `count` n represents n identical copies sharing one set of
port values (symmetric operation). Its flow into a bus is scaled by n.
"""
from dataclasses import dataclass, fields
from typing import Any

from .ports import Direction, Domain, PortSpec, port_value_types


@dataclass(frozen=True)
class Instance:
    name: str
    component: Any
    ports: tuple
    count: int

    def get_port(self, port_name):
        for port in self.ports:
            if port.name == port_name:
                return port
        raise KeyError(f"Instance '{self.name}' has no port '{port_name}'.")


@dataclass(frozen=True)
class Bus:
    name: str
    domain: Domain


@dataclass(frozen=True)
class Connection:
    """Direct: `source` OUT port -> `target` IN port. Bus: `source` is the attached
    port (either direction; see its PortSpec) and `target` is the bus name."""
    source: str
    target: str


@dataclass(frozen=True)
class Residual:
    """One equality the caller drives to zero; units follow the label suffix."""
    label: str
    value: Any


class Topology:
    def __init__(self):
        self._instances = {}
        self._buses = {}
        self._connections = []
        self._connected_ports = set()

    @property
    def instances(self):
        return dict(self._instances)

    @property
    def buses(self):
        return dict(self._buses)

    @property
    def connections(self):
        return tuple(self._connections)

    def add(self, name, component, ports, count=1):
        self._check_new_name(name)
        if "." in name:
            raise ValueError(f"Instance name '{name}' must not contain '.'.")
        if not isinstance(count, int) or isinstance(count, bool) or count < 1:
            raise ValueError(f"Instance count must be a positive integer, got {count!r}.")
        port_names = [port.name for port in ports]
        if len(set(port_names)) != len(port_names):
            raise ValueError(f"Instance '{name}' declares duplicate port names {port_names}.")
        self._instances[name] = Instance(name, component, tuple(ports), count)
        return name

    def add_bus(self, name, domain=Domain.ELECTRICAL):
        self._check_new_name(name)
        if domain is not Domain.ELECTRICAL:
            raise ValueError("Only electrical buses are supported; splitters/combiners are deferred.")
        self._buses[name] = Bus(name, domain)
        return name

    def connect(self, reference_a, reference_b):
        """Join two ports (either order) or a port and a bus; validated immediately."""
        if reference_b in self._buses or reference_a in self._buses:
            port_reference, bus_name = ((reference_a, reference_b) if reference_b in self._buses
                                        else (reference_b, reference_a))
            instance, port = self._resolve(port_reference)
            bus = self._buses[bus_name]
            if port.domain is not bus.domain:
                raise ValueError(f"Cannot connect {port.domain.value} port '{port_reference}' "
                                 f"to {bus.domain.value} bus '{bus_name}'.")
            connection = Connection(port_reference, bus_name)
        else:
            instance_a, port_a = self._resolve(reference_a)
            instance_b, port_b = self._resolve(reference_b)
            if port_a.domain is not port_b.domain:
                raise ValueError(f"Domain mismatch: '{reference_a}' is {port_a.domain.value}, "
                                 f"'{reference_b}' is {port_b.domain.value}.")
            if {port_a.direction, port_b.direction} != {Direction.OUT, Direction.IN}:
                raise ValueError(f"Direct connections join one OUT and one IN port: "
                                 f"'{reference_a}' and '{reference_b}' are both {port_a.direction.value}.")
            if instance_a.count != instance_b.count:
                raise ValueError(f"Direct connection between counts {instance_a.count} and "
                                 f"{instance_b.count} needs a splitter/combiner (deferred) or a bus.")
            connection = (Connection(reference_a, reference_b) if port_a.direction is Direction.OUT
                          else Connection(reference_b, reference_a))
        for reference in (connection.source, connection.target):
            if reference in self._connected_ports:
                raise ValueError(f"Port '{reference}' is already connected; use a bus for junctions.")
        self._connected_ports.update(r for r in (connection.source, connection.target) if r not in self._buses)
        self._connections.append(connection)
        return connection

    def get_port(self, reference):
        return self._resolve(reference)[1]

    def _resolve(self, reference):
        instance_name, separator, port_name = reference.partition(".")
        if not separator or instance_name not in self._instances:
            raise KeyError(f"'{reference}' is not an 'instance.port' reference in this topology.")
        instance = self._instances[instance_name]
        return instance, instance.get_port(port_name)

    def _check_new_name(self, name):
        if name in self._instances or name in self._buses:
            raise ValueError(f"Name '{name}' is already used in this topology.")


def connection_residuals(topology, port_values):
    """Return labelled residuals for every connection; the caller applies them.

    `port_values` maps "instance.port" to the matching port-value dataclass for
    every connected port. Values may be numeric or CasADi expressions.
    """
    residuals = []
    bus_ports = {name: [] for name in topology.buses}
    for connection in topology.connections:
        if connection.target in bus_ports:
            bus_ports[connection.target].append(connection.source)
            continue
        source = _port_value(topology, port_values, connection.source)
        target = _port_value(topology, port_values, connection.target)
        link = f"{connection.source}->{connection.target}"
        for field_name in (field.name for field in fields(source)):
            residuals.append(Residual(f"{link} {field_name}",
                                      getattr(source, field_name) - getattr(target, field_name)))

    for bus_name, references in bus_ports.items():
        if not references:
            continue
        values = [_port_value(topology, port_values, reference) for reference in references]
        for reference, value in zip(references[1:], values[1:]):
            residuals.append(Residual(f"{bus_name} {reference} voltage_V",
                                      value.voltage_V - values[0].voltage_V))
        # OUT ports deliver current into the bus; IN ports draw it.
        current_net_A = 0
        for reference, value in zip(references, values):
            instance_name = reference.partition(".")[0]
            sign = 1 if topology.get_port(reference).direction is Direction.OUT else -1
            current_net_A = current_net_A + sign * topology.instances[instance_name].count * value.current_A
        residuals.append(Residual(f"{bus_name} current_A", current_net_A))
    return residuals


def _port_value(topology, port_values, reference):
    if reference not in port_values:
        raise KeyError(f"No port value supplied for connected port '{reference}'.")
    value = port_values[reference]
    expected_type = port_value_types[topology.get_port(reference).domain]
    if not isinstance(value, expected_type):
        raise TypeError(f"Port '{reference}' needs {expected_type.__name__}, got {type(value).__name__}.")
    return value
