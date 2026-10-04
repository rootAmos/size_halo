"""Powertrain compatibility margins; report only, never resize or clip.

Operating margins compare each instance's port values with its own limits.
Design margins compare adjacent ratings across connections and buses and use
component parameters only. Max speed, max torque and rated power are
independent bounds; no corner point combining them is assumed.
"""
from dataclasses import dataclass, fields
from typing import Any

import aerosandbox.numpy as np

from aircraft_closure.core.margins import margin_above, margin_below
from aircraft_closure.core.ports import Direction
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


@dataclass(frozen=True)
class MechanicalEnvelope:
    """Shaft capability at one port; None means the port declares no bound."""
    max_speed_rad_s: Any = None
    max_torque_Nm: Any = None
    max_power_W: Any = None


@dataclass(frozen=True)
class ElectricalEnvelope:
    """Voltage window (or achievable range when `sets_voltage`) and power bound."""
    min_voltage_V: Any
    max_voltage_V: Any
    max_power_W: Any
    sets_voltage: bool = False


def battery_voltage_range_V(battery):
    """Terminal voltage at max discharge and at max charge power.

    Roots of V = OCV - I R with V I = P. Discharge beyond OCV^2 / (4 R) has no
    real root (the expression becomes NaN), which flags an infeasible rating.
    """
    ocv_V = battery.voltage_open_circuit_V
    resistance_ohm = battery.resistance_ohm
    min_voltage_V = (ocv_V + np.sqrt(ocv_V**2 - 4 * resistance_ohm * battery.max_discharge_power_W)) / 2
    max_voltage_V = (ocv_V + np.sqrt(ocv_V**2 + 4 * resistance_ohm * battery.max_charge_power_W)) / 2
    return min_voltage_V, max_voltage_V


def port_envelope(component, port_name):
    """Envelope of one port; electrical demand/supply bounds are loss-free."""
    if isinstance(component, (Motor, Generator)):
        if port_name == "shaft":
            return MechanicalEnvelope(component.max_speed_rad_s, component.max_torque_Nm, component.power_rated_W)
        if port_name == "electrical":
            # Shaft rating bounds motor demand from below and generator output
            # from above, so bus power margins built on it are optimistic.
            return ElectricalEnvelope(component.min_voltage_V, component.max_voltage_V, component.power_rated_W)
    elif isinstance(component, Battery) and port_name == "electrical":
        min_voltage_V, max_voltage_V = battery_voltage_range_V(component)
        return ElectricalEnvelope(min_voltage_V, max_voltage_V, component.max_discharge_power_W, sets_voltage=True)
    elif isinstance(component, EquivalentCircuitBattery) and port_name == "electrical":
        # Tier 17: the cell voltage window x count_series (2.5-4.2 V per 50G cell); rated power at nominal voltage.
        limits = component.get_limits()
        return ElectricalEnvelope(limits.min_voltage_V, limits.max_voltage_V, component.power_max_discharge_W,
                                  sets_voltage=True)
    elif isinstance(component, SimpleTurboshaft):
        if port_name == "shaft":
            return MechanicalEnvelope(max_power_W=component.power_rated_W)
        if port_name == "fuel":
            return None
    elif isinstance(component, Gearbox):
        if port_name == "shaft_in":
            return MechanicalEnvelope(max_power_W=component.power_rated_W)
        if port_name == "shaft_out":
            return MechanicalEnvelope(max_power_W=component.power_rated_W * component.efficiency)
    elif isinstance(component, (ActuatorDiskPropulsor, MomentumProfileRotor)) and port_name == "shaft":
        return MechanicalEnvelope(max_power_W=component.max_shaft_power_W)
    elif isinstance(component, Inverter) and port_name in ("dc", "ac"):
        limits = component.get_limits()
        return ElectricalEnvelope(limits.min_voltage_V, limits.max_voltage_V, limits.power_rated_W)
    elif isinstance(component, (Cable, ProtectionUnit)) and port_name in ("input", "output"):
        return ElectricalEnvelope(0.0, component.max_voltage_V, component.max_current_A * component.max_voltage_V)
    elif isinstance(component, DcDcConverter):
        limits = component.get_limits()
        if port_name == "input":
            return ElectricalEnvelope(limits.min_voltage_input_V, limits.max_voltage_input_V, limits.power_rated_W)
        if port_name == "output":
            return ElectricalEnvelope(limits.voltage_output_V, limits.voltage_output_V, limits.power_rated_W,
                                      sets_voltage=True)
    raise KeyError(f"No envelope for port '{port_name}' of {type(component).__name__}.")


def operating_margins(topology, port_values, atmosphere=None):
    """Per-instance margins from the port values used for connection residuals.

    `atmosphere` is the operating point's; it sets turboshaft power available
    (sea-level rating when None). Other limits do not depend on altitude here.
    """
    margins = []
    for name, instance in topology.instances.items():
        component = instance.component

        def value(port_name):
            reference = f"{name}.{port_name}"
            if reference not in port_values:
                raise KeyError(f"No port value supplied for '{reference}'.")
            return port_values[reference]

        if isinstance(component, (Motor, Generator)):
            shaft = value("shaft")
            electrical = value("electrical")
            if component.thermal_model is None:
                # Tier 19: with a thermal model the rating is continuous; the caller limits temperature instead.
                margins.append(margin_below(f"{name} power_shaft_W", shaft.speed_rad_s * shaft.torque_Nm,
                                            component.power_rated_W))
            margins += [
                margin_below(f"{name} torque_Nm", shaft.torque_Nm, component.max_torque_Nm),
                margin_below(f"{name} speed_rad_s", shaft.speed_rad_s, component.max_speed_rad_s),
                margin_above(f"{name} min_voltage_V", electrical.voltage_V, component.min_voltage_V),
                margin_below(f"{name} max_voltage_V", electrical.voltage_V, component.max_voltage_V),
            ]
        elif isinstance(component, Battery):
            electrical = value("electrical")
            power_terminal_W = electrical.voltage_V * electrical.current_A
            margins += [
                margin_below(f"{name} discharge_power_W", power_terminal_W, component.max_discharge_power_W),
                margin_below(f"{name} charge_power_W", -power_terminal_W, component.max_charge_power_W),
            ]
        elif isinstance(component, EquivalentCircuitBattery):
            electrical = value("electrical")
            limits = component.get_limits()
            margins += [
                margin_below(f"{name} discharge_current_A", electrical.current_A, limits.max_discharge_current_A),
                margin_below(f"{name} charge_current_A", -electrical.current_A, limits.max_charge_current_A),
                margin_above(f"{name} min_voltage_V", electrical.voltage_V, limits.min_voltage_V),
                margin_below(f"{name} max_voltage_V", electrical.voltage_V, limits.max_voltage_V),
            ]
        elif isinstance(component, SimpleTurboshaft):
            shaft = value("shaft")
            margins.append(margin_below(f"{name} power_shaft_W", shaft.speed_rad_s * shaft.torque_Nm,
                                        component.power_available_W(atmosphere)))
        elif isinstance(component, Gearbox):
            shaft = value("shaft_in")
            margins.append(margin_below(f"{name} power_input_W", shaft.speed_rad_s * shaft.torque_Nm,
                                        component.power_rated_W))
        elif isinstance(component, (ActuatorDiskPropulsor, MomentumProfileRotor)):
            shaft = value("shaft")
            margins.append(margin_below(f"{name} power_shaft_W", shaft.speed_rad_s * shaft.torque_Nm,
                                        component.max_shaft_power_W))
        elif isinstance(component, Inverter):
            # Tier 15: AC power against the rating (either direction: port currents are positive in the
            # instance's working direction) and the DC link inside the derated blocking voltage.
            ac, dc = value("ac"), value("dc")
            limits = component.get_limits()
            margins += [
                margin_below(f"{name} power_ac_W", ac.voltage_V * ac.current_A, limits.power_rated_W),
                margin_above(f"{name} min_voltage_V", dc.voltage_V, limits.min_voltage_V),
                margin_below(f"{name} max_voltage_V", dc.voltage_V, limits.max_voltage_V),
            ]
        elif isinstance(component, (Cable, ProtectionUnit)):
            # Thermal current rating for either current sign (squared, so smooth through zero).
            electrical = value("input")
            margins.append(margin_below(f"{name} current_A (squared)", electrical.current_A**2,
                                        component.max_current_A**2))
            if isinstance(component, Cable):
                # Partial discharge at the point's pressure: PDIV >= factor_safety x operating voltage.
                ratio = None
                if atmosphere is not None:
                    ratio = atmosphere.pressure() / 101325.0
                margins.append(margin_below(f"{name} partial_discharge_V",
                                            component.partial_discharge.factor_safety * electrical.voltage_V,
                                            component.voltage_inception_V(ratio)))
        elif isinstance(component, DcDcConverter):
            electrical = value("input")
            limits = component.get_limits()
            margins += [
                margin_below(f"{name} power_input_W (squared)", (electrical.voltage_V * electrical.current_A)**2,
                             limits.power_rated_W**2),
                margin_above(f"{name} min_voltage_input_V", electrical.voltage_V, limits.min_voltage_input_V),
                margin_below(f"{name} max_voltage_input_V", electrical.voltage_V, limits.max_voltage_input_V),
            ]
        else:
            raise TypeError(f"No operating margins for {type(component).__name__}.")
    return tuple(margins)


def design_margins(topology):
    """Adjacent-rating margins across direct shaft connections and buses."""
    margins = []
    bus_ports = {name: [] for name in topology.buses}
    for connection in topology.connections:
        if connection.target in bus_ports:
            bus_ports[connection.target].append(connection.source)
            continue
        upstream = _envelope(topology, connection.source)
        downstream = _envelope(topology, connection.target)
        if isinstance(upstream, ElectricalEnvelope):
            # Tier 15 direct electrical connections (feeders): downstream tolerates the upstream power.
            margins.append(margin_below(f"{connection.source}->{connection.target} max_power_W",
                                        upstream.max_power_W, downstream.max_power_W))
            continue
        for field in fields(MechanicalEnvelope):
            upstream_max = getattr(upstream, field.name)
            downstream_max = getattr(downstream, field.name)
            if upstream_max is not None and downstream_max is not None:
                # Downstream must tolerate the upstream maximum.
                margins.append(margin_below(f"{connection.source}->{connection.target} {field.name}",
                                            upstream_max, downstream_max))

    for bus_name, references in bus_ports.items():
        envelopes = {reference: _envelope(topology, reference) for reference in references}
        for setter, setter_envelope in envelopes.items():
            if not setter_envelope.sets_voltage:
                continue
            for reference, envelope in envelopes.items():
                if reference == setter or envelope.sets_voltage:
                    continue
                margins += [
                    margin_above(f"{bus_name} {setter}->{reference} min_voltage_V",
                                 setter_envelope.min_voltage_V, envelope.min_voltage_V),
                    margin_below(f"{bus_name} {setter}->{reference} max_voltage_V",
                                 setter_envelope.max_voltage_V, envelope.max_voltage_V),
                ]
        power_supply_W = 0
        power_demand_W = 0
        for reference, envelope in envelopes.items():
            count = topology.instances[reference.partition(".")[0]].count
            if topology.get_port(reference).direction is Direction.OUT:
                power_supply_W = power_supply_W + count * envelope.max_power_W
            else:
                power_demand_W = power_demand_W + count * envelope.max_power_W
        has_supply_and_demand = {topology.get_port(r).direction for r in references} == {Direction.OUT, Direction.IN}
        if has_supply_and_demand:
            margins.append(margin_above(f"{bus_name} power_W (loss-free)", power_supply_W, power_demand_W))
    return tuple(margins)


def _envelope(topology, reference):
    instance_name, _, port_name = reference.partition(".")
    return port_envelope(topology.instances[instance_name].component, port_name)
