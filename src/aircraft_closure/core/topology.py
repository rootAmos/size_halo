"""Network description only: no variables, constraints, ordering or solves.

A direct connection joins one OUT port to one IN port of the same domain and
instance count. A bus is a junction for any number of electrical ports: all
attached voltages are equal and currents obey Kirchhoff's current law. Cycles
are allowed; `connection_residuals` returns expressions the caller constrains
to zero in its own `asb.Opti`.

An instance with `count` n represents n identical copies sharing one set of
port values (symmetric operation). Its flow into a bus is scaled by n.

Tier 18: a direct connection made with `combine=True` may join counts where
one is an integer multiple of the other: a combiner (several lane motors on
one gearbox input) or a splitter (one pack into isolated strings). Efforts
(speed, voltage) are equal and the total flow is conserved,
count_source x flow_source = count_target x flow_target. A bus may have a
`count` too: that many identical buses (cross-strapped, each with an equal
share of the attached copies), lumped as one junction under symmetric
operation; the count is read by callers that model a bus failure.
"""
from dataclasses import dataclass, fields
from typing import Any

from .ports import Direction, Domain, PortSpec, port_flow_fields, port_value_types


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
    count: int = 1          # Tier 18: identical cross-strapped buses lumped as one junction


@dataclass(frozen=True)
class Connection:
    """Direct: `source` OUT port -> `target` IN port. Bus: `source` is the attached
    port (either direction; see its PortSpec) and `target` is the bus name. `combine`
    (Tier 18) marks a direct connection between different counts (combiner or splitter)."""
    source: str
    target: str
    combine: bool = False


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

    def add_bus(self, name, domain=Domain.ELECTRICAL, count=1):
        self._check_new_name(name)
        if domain is not Domain.ELECTRICAL:
            raise ValueError("Only electrical buses are supported; shaft combiners use connect(..., combine=True).")
        if not isinstance(count, int) or isinstance(count, bool) or count < 1:
            raise ValueError(f"Bus count must be a positive integer, got {count!r}.")
        self._buses[name] = Bus(name, domain, count)
        return name

    def connect(self, reference_a, reference_b, combine=False):
        """Join two ports (either order) or a port and a bus; validated immediately.

        `combine=True` (Tier 18) allows a direct connection between counts where one is an integer multiple of
        the other (a combiner or splitter); without it the counts must be equal.
        """
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
            counts = sorted((instance_a.count, instance_b.count))
            if not combine and counts[0] != counts[1]:
                raise ValueError(f"Direct connection between counts {instance_a.count} and "
                                 f"{instance_b.count} needs combine=True (a combiner or splitter) or a bus.")
            if combine and counts[1] % counts[0] != 0:
                raise ValueError(f"A combiner or splitter needs one count to be a multiple of the other, got "
                                 f"{instance_a.count} and {instance_b.count}.")
            connection = (Connection(reference_a, reference_b, combine) if port_a.direction is Direction.OUT
                          else Connection(reference_b, reference_a, combine))
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
        count_source = count_target = 1
        flow_field = None
        if connection.combine:
            # Combiner or splitter: efforts equal, total flow conserved.
            count_source = topology.instances[connection.source.partition(".")[0]].count
            count_target = topology.instances[connection.target.partition(".")[0]].count
            flow_field = port_flow_fields[topology.get_port(connection.source).domain]
        for field_name in (field.name for field in fields(source)):
            if field_name == flow_field:
                residuals.append(Residual(f"{link} {field_name} (total)",
                                          count_source * getattr(source, field_name)
                                          - count_target * getattr(target, field_name)))
            else:
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
