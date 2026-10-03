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
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.slipstream import RotorState
from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.powertrain.compatibility import operating_margins
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery

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
    electrical: Any = None          # ElectricalLayerResult (Tier 15) or None without the electrical layer


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

    @property
    def power_loss_total_W(self):
        return (self.power_loss_inverters_W + self.power_loss_cables_W + self.power_loss_protection_W
                + self.power_loss_dcdc_W)


def build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg, hybridization_electric=None, *,
                       hybridization_electric_min=0.0,
                       drag_increments=(), duration_s=0.0, voltage_rc_start_V=None):
    instances = aircraft.powertrain.topology.instances
    motor_model = instances["motor"].component
    generator_model = instances["generator"].component
    battery_model = instances["battery"].component
    gearbox_model = instances["gearbox"].component
    rotor_model = instances["propulsor"].component
    turboshaft_model = instances["turboshaft"].component
    active_rotor_count = condition.active_rotor_count or instances["propulsor"].count
    active_generator_count = condition.active_generator_count or instances["generator"].count
    weight_N = mass_kg * acceleration_gravity_m_s2
    temperature_offset_K = condition.temperature_offset_K
    atmosphere = asb.Atmosphere(altitude=condition.altitude_m, temperature_deviation=temperature_offset_K)

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
        thrust_total_N = aero.drag_N + weight_N * sin_gamma
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
    torque_motor_Nm = rotor.shaft_power_W / (gearbox_model.efficiency * speed_motor_rad_s)
    gear = gearbox_model.evaluate(speed_motor_rad_s, torque_motor_Nm)

    current_battery_A = opti.variable(init_guess=50.0, scale=100.0)
    if isinstance(battery_model, EquivalentCircuitBattery):
        battery = battery_model.evaluate(current_battery_A, condition.soc, duration_s, voltage_rc_start_V)
        # Low-current root of R_eff I^2 - V* I + P = 0; also P <= V*^2 / (4 R_eff).
        opti.subject_to(battery.voltage_V / battery.voltage_driving_V >= 0.5)
    else:
        battery = battery_model.evaluate(current_battery_A, condition.soc)
    has_layer = "inverter_motor" in instances
    if has_layer:
        layer = {name: instance.component for name, instance in instances.items()}
        dcdc_model = layer.get("dcdc")
        resistance_battery_feeder_ohm = (layer["cable_battery"].resistance_ohm()
                                         + layer["protection_battery"].resistance_ohm())
        # Battery side of the feeder: exact series drop at the battery current.
        voltage_feeder_battery_V = battery.voltage_V - current_battery_A * resistance_battery_feeder_ohm
        voltage_bus_V = dcdc_model.voltage_output_V if dcdc_model is not None else voltage_feeder_battery_V
    else:
        voltage_bus_V = battery.voltage_V
    motor = motor_model.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
    power_electric_motors_W = active_rotor_count * motor.power_electric_W
    if has_layer:
        inverter_motor = layer["inverter_motor"].evaluate(motor.power_electric_W, voltage_bus_V)
        current_motor_feeder_A = inverter_motor.power_dc_W / voltage_bus_V
        cable_motor = layer["cable_motor"].evaluate(current_motor_feeder_A)
        protection_motor = layer["protection_motor"].evaluate(current_motor_feeder_A)
        power_motor_bus_W = inverter_motor.power_dc_W + cable_motor.power_loss_W + protection_motor.power_loss_W
        power_electric_motors_W = active_rotor_count * power_motor_bus_W
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
            power_loss_inverters_W=(active_rotor_count * inverter_motor.power_loss_W
                                    + active_generator_count * inverter_generator.power_loss_W),
            power_loss_cables_W=(active_rotor_count * cable_motor.power_loss_W
                                 + active_generator_count * cable_generator.power_loss_W + cable_battery.power_loss_W),
            power_loss_protection_W=(active_rotor_count * protection_motor.power_loss_W
                                     + active_generator_count * protection_generator.power_loss_W
                                     + protection_battery.power_loss_W),
            power_loss_dcdc_W=power_loss_dcdc_W)
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
    # Each active turbogenerator carries an equal share of the generator power.
    power_scale_W = active_rotor_count * motor_model.power_rated_W
    opti.subject_to([
        (power_battery_bus_W - hybridization_electric * power_electric_motors_W) / power_scale_W == 0,
        (active_generator_count * power_generator_bus_W - (1 - hybridization_electric) * power_electric_motors_W)
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
        "gearbox.shaft_in": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
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
    margins = tuple(type(m)(f"{condition.label}: {m.label}", m.value)
                    for m in operating_margins(aircraft.powertrain.topology, port_values, atmosphere))
    return FlightPoint(condition=condition, weight_N=weight_N, alpha_deg=alpha_deg, aero=aero,
                       thrust_per_rotor_N=thrust_per_rotor_N, power_shaft_rotor_W=rotor.shaft_power_W,
                       speed_rotor_rad_s=speed_rotor_rad_s, speed_motor_rad_s=speed_motor_rad_s,
                       torque_motor_Nm=torque_motor_Nm, power_electric_motors_W=power_electric_motors_W,
                       hybridization_electric=hybridization_electric, battery=battery, generator=generator,
                       engine=engine, power_battery_W=battery.power_electric_W,
                       fuel_flow_kg_s=active_generator_count * engine.fuel_flow_kg_s,
                       margins=margins, electrical=electrical)
