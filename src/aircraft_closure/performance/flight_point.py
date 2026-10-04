"""Quasi-steady flight points coupling aerodynamics and the series-hybrid powertrain.

Orchestration layer: this module creates the per-point Opti variables (angle
of attack, rotor speed, battery current, generator torque and, if not given,
the electric power fraction) and the per-point equalities, because a flight
point is a caller-side coupling of several disciplines. Components remain
equation-only. The powertrain must be the series-hybrid reference topology
(instances turboshaft, generator, battery, motor, gearbox, propulsor).

Hover: thrust per active rotor = (T/W) W / ((1 - download) n_active) at zero
airspeed; the download fraction belongs to the aerodynamics model.
Airplane mode (tiltrotor cruise, rotors as propellers): wing lift = W cos(gamma),
thrust = D + W sin(gamma), sin(gamma) = climb rate / V.

Equivalent-circuit battery (Tier 17): the point holds the current for
`duration_s` from the RC state `voltage_rc_start_V` (None: steady-state
polarization), and the bus voltage is the interval-mean terminal voltage. The
power balance R_eff I^2 - V* I + P = 0 is an equality like any other; the
branch constraint V >= V*/2 selects its physical low-current root.

Electrical layer (Tier 15, when the topology has an `inverter_motor` instance): each motor draws its AC
power through an inverter, a cable and a protection unit; each generator delivers through an active
rectifier, a cable and a protection unit; the battery through a protection unit, a cable and optionally a
DC/DC converter that regulates the bus. The power balance and the hybridization share are then on the bus
side (`power_electric_motors_W` is what the motor feeders draw from the bus). Feeder currents for the cable
and contactor losses use the bus voltage (first order; drops are below 1 %); the battery feeder is exact,
bus voltage = terminal voltage - I (R_cable + R_protection) without a DC/DC converter.

Thermal (Tier 19): every point returns its named heat loads (motor, gearbox,
generator, generator_gearbox, battery; per unit, with the active count). With
an installed heat exchanger (`aircraft.powertrain.cooling`) the fan power
joins the bus demand in hover, and in airplane mode the cooling drag is a
per-point variable added to the thrust and tied to the exchanger's drag by
an equality (drag depends on heat, heat on thrust). Components with a thermal
model get an end-of-interval temperature from `temperature_start_C` (None:
steady state) over `duration_s`, with margins against their limits.

Redundancy (Tier 18, `topologies.RedundancyLayer`): with lane motors (motor count = rotors x lanes) the
rotor gearbox input torque is shared by the active lanes of each rotor; `FlightCondition.active_lane_count`
(per rotor), `active_battery_string_count` and `count_buses_failed` select a degraded state, applied to every
rotor alike (symmetric multiplicity; conservative for power and heat, no roll trim needed); a failed lane is
declutched (no spinning loss). String
contactors sit in series with the pack (exact drop at the string current); an isolated string removes its
share of the pack (`redundancy.battery_with_strings`), whose limits and thermal state are used at the point.
A failed bus loses its lanes and its sources feed the other buses through a tie, whose current is the
failed bus's share of the motor feeders' demand at the bus voltage (first order) and whose loss joins the
bus demand. Ties carry no current, and have no margins, in normal operation.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.slipstream import RotorState
from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.powertrain.compatibility import operating_margins
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.redundancy import battery_with_strings, degraded_state
from aircraft_closure.thermal.heat import HeatLoad, evaluate_point_thermal

acceleration_gravity_m_s2 = 9.80665


@dataclass(frozen=True)
class FlightCondition:
    mode: str = "airplane"
    velocity_m_s: Any = 60.0
    altitude_m: Any = 1000.0
    climb_rate_m_s: Any = 0.0
    thrust_to_weight: Any = 1.0
    active_rotor_count: Any = None
    active_generator_count: Any = None
    soc: Any = 0.8
    hybridization_electric: Any = None
    label: str = "point"
    # Tier 16: ambient temperature minus ISA at the (pressure) altitude; 0 is a standard day.
    temperature_offset_K: Any = 0.0
    # Tier 18 degraded states (Python integers): active lanes per rotor (None: all), active battery strings
    # (None: all) and failed buses (their lanes lost, their sources through a bus tie).
    active_lane_count: Any = None
    active_battery_string_count: Any = None
    count_buses_failed: int = 0

    def __post_init__(self):
        if self.mode not in ("hover", "airplane"):
            raise ValueError(f"Unknown flight mode '{self.mode}'.")


@dataclass(frozen=True)
class FlightPoint:
    condition: Any
    weight_N: Any
    alpha_deg: Any
    aero: Any
    thrust_per_rotor_N: Any
    power_shaft_rotor_W: Any
    speed_rotor_rad_s: Any
    speed_motor_rad_s: Any
    torque_motor_Nm: Any
    power_electric_motors_W: Any
    hybridization_electric: Any
    battery: Any
    generator: Any
    engine: Any
    power_battery_W: Any
    fuel_flow_kg_s: Any
    margins: tuple
    heat_loads: tuple = ()                 # Tier 19: HeatLoad per loss source
    thermal: Any = None                    # Tier 19: PointThermal
    electrical: Any = None          # ElectricalLayerResult (Tier 15) or None without the electrical layer
    redundancy: Any = None          # RedundancyPointResult (Tier 18) or None without redundancy hardware or lanes


@dataclass(frozen=True)
class RedundancyPointResult:
    """Tier 18 per-point state: active units, string contactors and the bus tie (currents per unit)."""
    count_lanes_active: int          # per rotor
    count_strings_active: int
    count_buses_failed: int
    torque_gearbox_input_Nm: Any     # per rotor, shared by the active lanes
    current_string_A: Any            # per active string (0 without string contactors)
    power_loss_strings_W: Any        # total over the active string contactors
    current_tie_A: Any               # per tie (0 in normal operation)
    power_loss_tie_W: Any            # contactor + cable of the one tie in use


@dataclass(frozen=True)
class ElectricalLayerResult:
    """Tier 15 per-point electrical state; powers are per instance unless they are named totals."""
    voltage_bus_V: Any
    voltage_battery_V: Any
    inverter_motor: Any
    inverter_generator: Any
    cable_motor: Any
    cable_generator: Any
    cable_battery: Any
    dcdc: Any
    power_motor_bus_W: Any          # drawn from the bus per motor feeder
    power_generator_bus_W: Any      # delivered to the bus per generator feeder
    power_battery_bus_W: Any        # delivered to the bus by the battery feeder
    power_loss_inverters_W: Any     # totals over the active instances
    power_loss_cables_W: Any
    power_loss_protection_W: Any
    power_loss_dcdc_W: Any
    heat_loads: tuple = ()          # Tier 19 HeatLoad per electrical-layer instance (per unit, active count)

    @property
    def power_loss_total_W(self):
        return (self.power_loss_inverters_W + self.power_loss_cables_W + self.power_loss_protection_W
                + self.power_loss_dcdc_W)


def build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg, hybridization_electric=None, *,
                       hybridization_electric_min=0.0,
                       drag_increments=(), duration_s=0.0, voltage_rc_start_V=None, temperature_start_C=None):
    topology = aircraft.powertrain.topology
    instances = topology.instances
    motor_model = instances["motor"].component
    generator_model = instances["generator"].component
    battery_model = instances["battery"].component
    gearbox_model = instances["gearbox"].component
    rotor_model = instances["propulsor"].component
    turboshaft_model = instances["turboshaft"].component
    active_rotor_count = condition.active_rotor_count or instances["propulsor"].count
    active_generator_count = condition.active_generator_count or instances["generator"].count
    # Tier 18: lanes per rotor, active strings and failed buses (all ones for the plain topology).
    state = degraded_state(topology, condition)
    count_lanes_active = state.count_lanes_active
    count_motors_active = active_rotor_count * count_lanes_active
    has_strings = "protection_string" in instances
    has_redundancy = has_strings or state.counts.count_lanes > 1 or state.counts.count_buses > 1
    battery_model = battery_with_strings(battery_model, state.fraction_strings_active)
    components_override = {"battery": battery_model} if state.fraction_strings_active != 1 else {}
    weight_N = mass_kg * acceleration_gravity_m_s2
    temperature_offset_K = condition.temperature_offset_K
    atmosphere = asb.Atmosphere(altitude=condition.altitude_m, temperature_deviation=temperature_offset_K)
    has_cooling = getattr(aircraft.powertrain, "cooling", None) is not None
    drag_cooling_N = 0.0

    if condition.mode == "hover":
        alpha_deg, aero = None, None
        speed_rotor_rad_s = opti.variable(init_guess=100.0, scale=100.0, lower_bound=10.0)
        velocity_m_s = 0.0
        download_fraction = aerodynamics.hover_download_fraction(aircraft)
        thrust_total_N = condition.thrust_to_weight * weight_N / (1 - download_fraction)
    else:
        velocity_m_s = condition.velocity_m_s
        alpha_deg = opti.variable(init_guess=4.0, lower_bound=-5.0, upper_bound=20.0)
        speed_rotor_rad_s = opti.variable(init_guess=100.0, scale=100.0, lower_bound=10.0)
        rotor_state = None
        if getattr(aerodynamics, "blown_wing", None) is not None:
            # Tier 21 blown wing: the slipstream depends on thrust, thrust on drag. Thrust per rotor is a variable
            # tied to the drag by an equality below (no iteration).
            rotor_state = RotorState(opti.variable(init_guess=5000.0, scale=5000.0, lower_bound=0.0),
                                     speed_rotor_rad_s)
        aero = aerodynamics.evaluate(aircraft, velocity_m_s, condition.altitude_m, alpha_deg, drag_increments,
                                     temperature_offset_K=temperature_offset_K, rotor_state=rotor_state)
        sin_gamma = condition.climb_rate_m_s / velocity_m_s
        # Equalities are normalized (lift by weight, powers by installed motor
        # rating) so IPOPT sees O(1) residuals in large coupled problems.
        opti.subject_to([aero.lift_N / weight_N == np.sqrt(1 - sin_gamma**2),
                         alpha_deg <= aerodynamics.alpha_stall_deg(aircraft, velocity_m_s, condition.altitude_m,
                                                                   temperature_offset_K=temperature_offset_K,
                                                                   aero=aero)])
        if has_cooling:
            # Tier 19: cooling drag as a variable, tied to the heat exchanger's drag below.
            drag_cooling_N = opti.variable(init_guess=100.0, scale=100.0, lower_bound=0.0)
        thrust_total_N = aero.drag_N + drag_cooling_N + weight_N * sin_gamma
        if rotor_state is not None:
            opti.subject_to((active_rotor_count * rotor_state.thrust_per_rotor_N - thrust_total_N) / weight_N == 0)
            thrust_total_N = active_rotor_count * rotor_state.thrust_per_rotor_N

    thrust_per_rotor_N = thrust_total_N / active_rotor_count
    if condition.mode == "airplane":
        rotor_model = rotor_model.in_airplane_mode()
    speed_motor_limit_rad_s = motor_model.max_speed_rad_s
    rotor = rotor_model.evaluate(velocity_m_s, atmosphere, thrust_N=thrust_per_rotor_N, speed_rad_s=speed_rotor_rad_s)
    # Rotor-speed physics models (Tier 12) return blade loading and helical tip Mach to bound.
    if getattr(rotor_model, "blade_loading_max", None) is not None:
        opti.subject_to(rotor.blade_loading <= rotor_model.blade_loading_max)
    if getattr(rotor_model, "mach_tip_helical_max", None) is not None:
        opti.subject_to(rotor.mach_tip_helical <= rotor_model.mach_tip_helical_max)
    if condition.mode == "airplane" and getattr(rotor_model, "advance_ratio_max", None) is not None:
        opti.subject_to(rotor.advance_ratio <= rotor_model.advance_ratio_max)
    speed_motor_rad_s = gearbox_model.reduction_ratio * speed_rotor_rad_s
    # Gearbox loss is taken from torque, so input torque = P_out / (eta * omega_in).
    torque_gearbox_input_Nm = rotor.shaft_power_W / (gearbox_model.efficiency * speed_motor_rad_s)
    # Tier 18: the active lanes of a rotor share its gearbox input torque.
    torque_motor_Nm = (torque_gearbox_input_Nm if count_lanes_active == 1
                       else torque_gearbox_input_Nm / count_lanes_active)
    gear = gearbox_model.evaluate(speed_motor_rad_s, torque_gearbox_input_Nm)

    # With cooling (Tier 19) start at zero current: 50 A held through a long cruise drives the coulomb-counted SOC
    # chain far outside the cell data, and the battery loss there, through the exchanger's cubic pumping law,
    # swamps IPOPT. Without cooling the earlier 50 A start is kept (reference results unchanged).
    current_battery_A = opti.variable(init_guess=0.0 if has_cooling else 50.0, scale=100.0)
    if isinstance(battery_model, EquivalentCircuitBattery):
        battery = battery_model.evaluate(current_battery_A, condition.soc, duration_s, voltage_rc_start_V)
        # Low-current root of R_eff I^2 - V* I + P = 0; also P <= V*^2 / (4 R_eff).
        opti.subject_to(battery.voltage_V / battery.voltage_driving_V >= 0.5)
    else:
        battery = battery_model.evaluate(current_battery_A, condition.soc)
    has_layer = "inverter_motor" in instances
    layer = {name: instance.component for name, instance in instances.items()}
    dcdc_model = layer.get("dcdc") if has_layer else None
    resistances_feeder_ohm = []
    if has_layer:
        resistances_feeder_ohm.append(layer["cable_battery"].resistance_ohm()
                                      + layer["protection_battery"].resistance_ohm())
    if has_strings:
        # Tier 18: the active strings' contactors in parallel, in series with the pack.
        current_string_A = current_battery_A / state.count_strings_active
        protection_string = layer["protection_string"].evaluate(current_string_A)
        resistances_feeder_ohm.append(layer["protection_string"].resistance_ohm() / state.count_strings_active)
    if resistances_feeder_ohm:
        resistance_battery_feeder_ohm = (resistances_feeder_ohm[0] if len(resistances_feeder_ohm) == 1
                                         else resistances_feeder_ohm[0] + resistances_feeder_ohm[1])
        # Battery side of the feeder: exact series drop at the battery current.
        voltage_feeder_battery_V = battery.voltage_V - current_battery_A * resistance_battery_feeder_ohm
        voltage_bus_V = dcdc_model.voltage_output_V if dcdc_model is not None else voltage_feeder_battery_V
    else:
        voltage_bus_V = battery.voltage_V
    motor = motor_model.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
    power_electric_motors_W = count_motors_active * motor.power_electric_W
    if has_layer:
        inverter_motor = layer["inverter_motor"].evaluate(motor.power_electric_W, voltage_bus_V)
        current_motor_feeder_A = inverter_motor.power_dc_W / voltage_bus_V
        cable_motor = layer["cable_motor"].evaluate(current_motor_feeder_A)
        protection_motor = layer["protection_motor"].evaluate(current_motor_feeder_A)
        power_motor_bus_W = inverter_motor.power_dc_W + cable_motor.power_loss_W + protection_motor.power_loss_W
        power_electric_motors_W = count_motors_active * power_motor_bus_W
    # Tier 18 bus tie: a failed bus's sources feed the others through one tie (normally open: no current).
    current_tie_A, power_loss_tie_W, tie_ports = 0.0, 0.0, {}
    if state.counts.count_ties:
        if state.count_buses_failed:
            current_tie_A = (state.count_buses_failed * power_electric_motors_W
                             / (state.counts.count_buses * voltage_bus_V))
            protection_tie = layer["protection_bus_tie"].evaluate(current_tie_A)
            cable_tie = layer["cable_bus_tie"].evaluate(current_tie_A)
            power_loss_tie_W = protection_tie.power_loss_W + cable_tie.power_loss_W
        tie_ports = {f"{name}.{port}": ElectricalPortValue(voltage_bus_V, current_tie_A)
                     for name in ("protection_bus_tie", "cable_bus_tie") for port in ("input", "output")}
    if hybridization_electric is None:
        hybridization_electric = condition.hybridization_electric
    if hybridization_electric is None:
        # A negative share means the generators also recharge the battery (bounded by its charge rating).
        hybridization_electric = opti.variable(init_guess=0.3 if hybridization_electric_min >= 0 else 0.0,
                                               lower_bound=hybridization_electric_min, upper_bound=1.0)
    speed_generator_rad_s = generator_model.loss_model.speed_peak_efficiency_rad_s
    torque_generator_Nm = opti.variable(init_guess=500.0, scale=500.0, lower_bound=0.0)
    generator = generator_model.evaluate(speed_generator_rad_s, torque_generator_Nm, voltage_bus_V)
    electrical = None
    power_battery_bus_W = battery.power_electric_W
    if has_strings and not has_layer:
        power_battery_bus_W = voltage_feeder_battery_V * current_battery_A
    power_generator_bus_W = generator.power_electric_W
    if has_layer:
        # Rectifier: positive inverter power flows DC -> AC, so generation is negative AC power.
        inverter_generator = layer["inverter_generator"].evaluate(-generator.power_electric_W, voltage_bus_V)
        current_generator_feeder_A = -inverter_generator.power_dc_W / voltage_bus_V
        cable_generator = layer["cable_generator"].evaluate(current_generator_feeder_A)
        protection_generator = layer["protection_generator"].evaluate(current_generator_feeder_A)
        power_generator_bus_W = (-inverter_generator.power_dc_W - cable_generator.power_loss_W
                                 - protection_generator.power_loss_W)
        cable_battery = layer["cable_battery"].evaluate(current_battery_A)
        protection_battery = layer["protection_battery"].evaluate(current_battery_A)
        power_battery_feeder_W = voltage_feeder_battery_V * current_battery_A
        dcdc = None
        power_loss_dcdc_W = 0.0
        power_battery_bus_W = power_battery_feeder_W
        if dcdc_model is not None:
            dcdc = dcdc_model.evaluate(power_battery_feeder_W, voltage_feeder_battery_V)
            power_battery_bus_W = dcdc.power_output_W
            power_loss_dcdc_W = dcdc.power_loss_W
        electrical = ElectricalLayerResult(
            voltage_bus_V=voltage_bus_V, voltage_battery_V=battery.voltage_V, inverter_motor=inverter_motor,
            inverter_generator=inverter_generator, cable_motor=cable_motor, cable_generator=cable_generator,
            cable_battery=cable_battery, dcdc=dcdc, power_motor_bus_W=power_motor_bus_W,
            power_generator_bus_W=power_generator_bus_W, power_battery_bus_W=power_battery_bus_W,
            power_loss_inverters_W=(count_motors_active * inverter_motor.power_loss_W
                                    + active_generator_count * inverter_generator.power_loss_W),
            power_loss_cables_W=(count_motors_active * cable_motor.power_loss_W
                                 + active_generator_count * cable_generator.power_loss_W + cable_battery.power_loss_W),
            power_loss_protection_W=(count_motors_active * protection_motor.power_loss_W
                                     + active_generator_count * protection_generator.power_loss_W
                                     + protection_battery.power_loss_W),
            power_loss_dcdc_W=power_loss_dcdc_W,
            heat_loads=(
                HeatLoad("inverter_motor", inverter_motor.power_loss_W, count_motors_active),
                HeatLoad("cable_motor", cable_motor.power_loss_W, count_motors_active),
                HeatLoad("protection_motor", protection_motor.power_loss_W, count_motors_active),
                HeatLoad("inverter_generator", inverter_generator.power_loss_W, active_generator_count),
                HeatLoad("cable_generator", cable_generator.power_loss_W, active_generator_count),
                HeatLoad("protection_generator", protection_generator.power_loss_W, active_generator_count),
                HeatLoad("cable_battery", cable_battery.power_loss_W, 1),
                HeatLoad("protection_battery", protection_battery.power_loss_W, 1),
            ) + ((HeatLoad("dcdc", power_loss_dcdc_W, 1),) if dcdc is not None else ()))
    # Optional step-up gearbox between turboshaft output and generator (Tier 13): ratio = input / output speed.
    speed_engine_rad_s, torque_engine_Nm = speed_generator_rad_s, torque_generator_Nm
    generator_gear_ports = {}
    if "generator_gearbox" in instances:
        generator_gearbox_model = instances["generator_gearbox"].component
        speed_engine_rad_s = speed_generator_rad_s * generator_gearbox_model.reduction_ratio
        torque_engine_Nm = torque_generator_Nm / (generator_gearbox_model.reduction_ratio * generator_gearbox_model.efficiency)
        generator_gear = generator_gearbox_model.evaluate(speed_engine_rad_s, torque_engine_Nm)
        generator_gear_ports = {
            "generator_gearbox.shaft_in": MechanicalPortValue(speed_engine_rad_s, torque_engine_Nm),
            "generator_gearbox.shaft_out": MechanicalPortValue(generator_gear.speed_output_rad_s,
                                                               generator_gear.torque_output_Nm),
        }
    engine = turboshaft_model.evaluate(speed_engine_rad_s * torque_engine_Nm, atmosphere)
    # Tier 19: heat loads by source (per unit, active count); new loss sources add a HeatLoad here.
    heat_loads = (HeatLoad("motor", motor.power_loss_W, count_motors_active),
                  HeatLoad("gearbox", gear.power_loss_W, active_rotor_count),
                  HeatLoad("generator", generator.power_loss_W, active_generator_count),
                  HeatLoad("battery", battery.power_loss_W, 1))
    if generator_gear_ports:
        heat_loads += (HeatLoad("generator_gearbox", generator_gear.power_loss_W, active_generator_count),)
    if electrical is not None:
        heat_loads += electrical.heat_loads          # Tier 15: inverters, cables, protection, DC/DC by instance
    redundancy = None
    if has_redundancy:
        power_loss_strings_W = state.count_strings_active * protection_string.power_loss_W if has_strings else 0.0
        if has_strings:
            heat_loads += (HeatLoad("protection_string", protection_string.power_loss_W, state.count_strings_active),)
        if state.count_buses_failed:
            heat_loads += (HeatLoad("protection_bus_tie", protection_tie.power_loss_W, 1),
                           HeatLoad("cable_bus_tie", cable_tie.power_loss_W, 1))
        redundancy = RedundancyPointResult(
            count_lanes_active=count_lanes_active, count_strings_active=state.count_strings_active,
            count_buses_failed=state.count_buses_failed, torque_gearbox_input_Nm=torque_gearbox_input_Nm,
            current_string_A=current_string_A if has_strings else 0.0, power_loss_strings_W=power_loss_strings_W,
            current_tie_A=current_tie_A, power_loss_tie_W=power_loss_tie_W)
    thermal = evaluate_point_thermal(aircraft.powertrain, heat_loads, atmosphere, velocity_m_s, condition.mode,
                                     duration_s, temperature_start_C, condition.label, components=components_override)
    power_electric_demand_W = power_electric_motors_W
    if state.count_buses_failed:
        power_electric_demand_W = power_electric_motors_W + power_loss_tie_W
    if thermal.cooling is not None:
        power_electric_demand_W = power_electric_demand_W + thermal.cooling.power_fan_W
        if condition.mode == "airplane":
            opti.subject_to((drag_cooling_N - thermal.cooling.drag_N) / (0.01 * weight_N) == 0)
    # Each active turbogenerator carries an equal share of the generator power.
    power_scale_W = active_rotor_count * motor_model.power_rated_W
    if state.counts.count_lanes > 1:
        power_scale_W = power_scale_W * state.counts.count_lanes
    opti.subject_to([
        (power_battery_bus_W - hybridization_electric * power_electric_demand_W) / power_scale_W == 0,
        (active_generator_count * power_generator_bus_W - (1 - hybridization_electric) * power_electric_demand_W)
        / power_scale_W == 0,
        speed_motor_rad_s <= speed_motor_limit_rad_s,
    ])
    if rotor_model.speed_tip_max_m_s is not None:
        opti.subject_to(speed_rotor_rad_s * rotor_model.radius_m() <= rotor_model.speed_tip_max_m_s)

    port_values = {
        "turboshaft.shaft": MechanicalPortValue(speed_engine_rad_s, torque_engine_Nm),
        **generator_gear_ports,
        "generator.shaft": MechanicalPortValue(speed_generator_rad_s, torque_generator_Nm),
        "generator.electrical": ElectricalPortValue(voltage_bus_V, generator.current_A),
        "battery.electrical": ElectricalPortValue(battery.voltage_V, current_battery_A),
        "motor.electrical": ElectricalPortValue(voltage_bus_V, motor.current_A),
        "motor.shaft": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
        "gearbox.shaft_in": MechanicalPortValue(speed_motor_rad_s, torque_gearbox_input_Nm),
        "gearbox.shaft_out": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
        "propulsor.shaft": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
    }
    if has_layer:
        port_values.update({
            "inverter_motor.ac": ElectricalPortValue(voltage_bus_V, motor.current_A),
            "inverter_motor.dc": ElectricalPortValue(voltage_bus_V, current_motor_feeder_A),
            "cable_motor.input": ElectricalPortValue(voltage_bus_V, current_motor_feeder_A),
            "cable_motor.output": ElectricalPortValue(voltage_bus_V, current_motor_feeder_A),
            "protection_motor.input": ElectricalPortValue(voltage_bus_V, current_motor_feeder_A),
            "protection_motor.output": ElectricalPortValue(voltage_bus_V, current_motor_feeder_A),
            "inverter_generator.ac": ElectricalPortValue(voltage_bus_V, generator.current_A),
            "inverter_generator.dc": ElectricalPortValue(voltage_bus_V, current_generator_feeder_A),
            "cable_generator.input": ElectricalPortValue(voltage_bus_V, current_generator_feeder_A),
            "cable_generator.output": ElectricalPortValue(voltage_bus_V, current_generator_feeder_A),
            "protection_generator.input": ElectricalPortValue(voltage_bus_V, current_generator_feeder_A),
            "protection_generator.output": ElectricalPortValue(voltage_bus_V, current_generator_feeder_A),
            "protection_battery.input": ElectricalPortValue(battery.voltage_V, current_battery_A),
            "protection_battery.output": ElectricalPortValue(voltage_feeder_battery_V, current_battery_A),
            "cable_battery.input": ElectricalPortValue(battery.voltage_V, current_battery_A),
            "cable_battery.output": ElectricalPortValue(voltage_feeder_battery_V, current_battery_A),
        })
        if dcdc_model is not None:
            port_values.update({
                "dcdc.input": ElectricalPortValue(voltage_feeder_battery_V, current_battery_A),
                "dcdc.output": ElectricalPortValue(voltage_bus_V, dcdc.current_output_A),
            })
    if has_strings:
        port_values.update({
            "protection_string.input": ElectricalPortValue(battery.voltage_V, current_string_A),
            "protection_string.output": ElectricalPortValue(battery.voltage_V - protection_string.voltage_drop_V,
                                                            current_string_A),
        })
    port_values.update(tie_ports)
    margins_operating = operating_margins(topology, port_values, atmosphere, components=components_override)
    if state.counts.count_ties and not state.count_buses_failed:
        # Normally open ties: de-energized, no current, no margins.
        margins_operating = tuple(m for m in margins_operating
                                  if not m.label.startswith(("protection_bus_tie ", "cable_bus_tie ")))
    margins = tuple(type(m)(f"{condition.label}: {m.label}", m.value) for m in margins_operating) + thermal.margins
    return FlightPoint(condition=condition, weight_N=weight_N, alpha_deg=alpha_deg, aero=aero,
                       thrust_per_rotor_N=thrust_per_rotor_N, power_shaft_rotor_W=rotor.shaft_power_W,
                       speed_rotor_rad_s=speed_rotor_rad_s, speed_motor_rad_s=speed_motor_rad_s,
                       torque_motor_Nm=torque_motor_Nm, power_electric_motors_W=power_electric_motors_W,
                       hybridization_electric=hybridization_electric, battery=battery, generator=generator,
                       engine=engine, power_battery_W=battery.power_electric_W,
                       fuel_flow_kg_s=active_generator_count * engine.fuel_flow_kg_s,
                       margins=margins, heat_loads=heat_loads, thermal=thermal, electrical=electrical,
                       redundancy=redundancy)
