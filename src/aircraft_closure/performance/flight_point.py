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
    voltage_bus_V = battery.voltage_V
    motor = motor_model.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
    power_electric_motors_W = active_rotor_count * motor.power_electric_W
    if hybridization_electric is None:
        hybridization_electric = condition.hybridization_electric
    if hybridization_electric is None:
        # A negative share means the generators also recharge the battery (bounded by its charge rating).
        hybridization_electric = opti.variable(init_guess=0.3 if hybridization_electric_min >= 0 else 0.0,
                                               lower_bound=hybridization_electric_min, upper_bound=1.0)
    speed_generator_rad_s = generator_model.loss_model.speed_peak_efficiency_rad_s
    torque_generator_Nm = opti.variable(init_guess=500.0, scale=500.0, lower_bound=0.0)
    generator = generator_model.evaluate(speed_generator_rad_s, torque_generator_Nm, voltage_bus_V)
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
        (battery.power_electric_W - hybridization_electric * power_electric_motors_W) / power_scale_W == 0,
        (active_generator_count * generator.power_electric_W - (1 - hybridization_electric) * power_electric_motors_W)
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
        "battery.electrical": ElectricalPortValue(voltage_bus_V, current_battery_A),
        "motor.electrical": ElectricalPortValue(voltage_bus_V, motor.current_A),
        "motor.shaft": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
        "gearbox.shaft_in": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
        "gearbox.shaft_out": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
        "propulsor.shaft": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
    }
    margins = tuple(type(m)(f"{condition.label}: {m.label}", m.value)
                    for m in operating_margins(aircraft.powertrain.topology, port_values, atmosphere))
    return FlightPoint(condition=condition, weight_N=weight_N, alpha_deg=alpha_deg, aero=aero,
                       thrust_per_rotor_N=thrust_per_rotor_N, power_shaft_rotor_W=rotor.shaft_power_W,
                       speed_rotor_rad_s=speed_rotor_rad_s, speed_motor_rad_s=speed_motor_rad_s,
                       torque_motor_Nm=torque_motor_Nm, power_electric_motors_W=power_electric_motors_W,
                       hybridization_electric=hybridization_electric, battery=battery, generator=generator,
                       engine=engine, power_battery_W=battery.power_electric_W,
                       fuel_flow_kg_s=active_generator_count * engine.fuel_flow_kg_s,
                       margins=margins)
