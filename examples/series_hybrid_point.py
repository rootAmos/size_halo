"""Series-hybrid point coupled through the Tier 2 topology description.

The topology states what is connected; this caller owns every Opti variable,
applies the generated connection residuals, and adds the operating and rating
constraints. With one rotor it reproduces `series_hybrid_point_explicit.py`.
Illustrative, not an aircraft sizing or mission model.
"""
from dataclasses import dataclass

import aerosandbox as asb

from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.core.topology import connection_residuals
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.topologies import build_series_hybrid


@dataclass(frozen=True)
class TopologyPointResult:
    thrust_N: float
    shaft_power_W: float
    battery_power_W: float
    fuel_flow_kg_s: float
    bus_power_residual_W: float
    rotor_power_residual_W: float
    current_motor_A: float
    current_motors_total_A: float
    voltage_bus_V: float
    max_abs_connection_residual: float


def build_reference_topology(count_rotors=1):
    """One shared source side sized for `count_rotors` identical rotor strings.

    The battery is n reference packs in parallel (capacity and power x n,
    resistance / n) so bus voltage, and therefore per-motor current, is the same
    for every n. Generator and turboshaft ratings scale with n; their loss and
    efficiency parameters do not, so fuel flow is not exactly n times larger.
    """
    reference_battery = Battery()
    reference_generator = Generator()
    battery = Battery(energy_capacity_J=reference_battery.energy_capacity_J * count_rotors,
                      resistance_ohm=reference_battery.resistance_ohm / count_rotors,
                      max_discharge_power_W=reference_battery.max_discharge_power_W * count_rotors,
                      max_charge_power_W=reference_battery.max_charge_power_W * count_rotors)
    generator = Generator(power_rated_W=reference_generator.power_rated_W * count_rotors,
                          max_torque_Nm=reference_generator.max_torque_Nm * count_rotors)
    turboshaft = SimpleTurboshaft(power_rated_W=SimpleTurboshaft().power_rated_W * count_rotors)
    return build_series_hybrid(Motor(), generator, battery, turboshaft, Gearbox(),
                               ActuatorDiskPropulsor(), count_rotors=count_rotors)


def solve_topology_point(count_rotors=1, thrust_per_rotor_N=5000.0, hybridization_electric=0.2):
    """Hover point with `hybridization_electric` of motor electrical input from the battery."""
    topology = build_reference_topology(count_rotors)
    instances = topology.instances
    motor_component = instances["motor"].component
    generator_component = instances["generator"].component
    battery_component = instances["battery"].component
    turboshaft_component = instances["turboshaft"].component
    gearbox_component = instances["gearbox"].component
    rotor_component = instances["propulsor"].component

    opti = asb.Opti()
    # Per-instance operating variables. Generator speed and motor speed are
    # prescribed, as in the Tier 1 reference; everything else is solved.
    speed_motor_rad_s = 400.0
    speed_generator_rad_s = 400.0
    torque_motor_Nm = opti.variable(init_guess=200, lower_bound=0)
    torque_generator_Nm = opti.variable(init_guess=200 * count_rotors, lower_bound=0)
    torque_turboshaft_Nm = opti.variable(init_guess=200 * count_rotors, lower_bound=0)
    speed_rotor_rad_s = opti.variable(init_guess=100, lower_bound=1)
    torque_rotor_Nm = opti.variable(init_guess=800, lower_bound=0)
    current_battery_A = opti.variable(init_guess=20 * count_rotors)
    voltage_bus_V = opti.variable(init_guess=800, lower_bound=1)
    induced_velocity_m_s = opti.variable(init_guess=10, lower_bound=0)

    pack = battery_component.evaluate(current_battery_A, soc=0.9)
    motor = motor_component.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
    generator = generator_component.evaluate(speed_generator_rad_s, torque_generator_Nm, voltage_bus_V)
    engine = turboshaft_component.evaluate(speed_generator_rad_s * torque_turboshaft_Nm)
    gear = gearbox_component.evaluate(speed_motor_rad_s, torque_motor_Nm)
    rotor = rotor_component.evaluate(0, asb.Atmosphere(altitude=0),
                                     shaft_power_W=speed_rotor_rad_s * torque_rotor_Nm,
                                     induced_velocity_m_s=induced_velocity_m_s)

    port_values = {
        "turboshaft.shaft": MechanicalPortValue(speed_generator_rad_s, torque_turboshaft_Nm),
        "generator.shaft": MechanicalPortValue(speed_generator_rad_s, torque_generator_Nm),
        "generator.electrical": ElectricalPortValue(voltage_bus_V, generator.current_A),
        "battery.electrical": ElectricalPortValue(pack.voltage_V, pack.current_A),
        "motor.electrical": ElectricalPortValue(voltage_bus_V, motor.current_A),
        "motor.shaft": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
        "gearbox.shaft_in": MechanicalPortValue(speed_motor_rad_s, torque_motor_Nm),
        "gearbox.shaft_out": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
        "propulsor.shaft": MechanicalPortValue(speed_rotor_rad_s, torque_rotor_Nm),
    }
    residuals = connection_residuals(topology, port_values)
    power_motors_electric_W = count_rotors * motor.power_electric_W

    opti.subject_to([residual.value == 0 for residual in residuals])
    opti.subject_to([
        # Momentum theory, specified thrust and the prescribed source split.
        rotor.power_residual_W == 0,
        rotor.thrust_N == thrust_per_rotor_N,
        pack.power_electric_W == hybridization_electric * power_motors_electric_W,
        # Per-instance hardware ratings, applied by the caller as in Tier 1.
        motor.power_shaft_W <= motor_component.get_limits().power_rated_W,
        torque_motor_Nm <= motor_component.get_limits().max_torque_Nm,
        speed_motor_rad_s <= motor_component.get_limits().max_speed_rad_s,
        voltage_bus_V >= motor_component.get_limits().min_voltage_V,
        voltage_bus_V <= motor_component.get_limits().max_voltage_V,
        generator.power_shaft_W <= generator_component.get_limits().power_rated_W,
        torque_generator_Nm <= generator_component.get_limits().max_torque_Nm,
        speed_generator_rad_s <= generator_component.get_limits().max_speed_rad_s,
        voltage_bus_V >= generator_component.get_limits().min_voltage_V,
        voltage_bus_V <= generator_component.get_limits().max_voltage_V,
        generator.power_electric_W >= 0,
        engine.power_shaft_W <= turboshaft_component.get_limits().power_rated_W,
        gear.power_input_W <= gearbox_component.get_limits().power_rated_W,
        rotor.shaft_power_W <= rotor_component.get_limits().max_shaft_power_W,
        pack.power_electric_W <= battery_component.get_limits().max_discharge_power_W,
    ])
    solution = opti.solve(verbose=False)
    # Conversion to ordinary scalars is confined to post-solve reporting.
    return TopologyPointResult(
        thrust_N=float(solution.value(count_rotors * rotor.thrust_N)),
        shaft_power_W=float(solution.value(rotor.shaft_power_W)),
        battery_power_W=float(solution.value(pack.power_electric_W)),
        fuel_flow_kg_s=float(solution.value(engine.fuel_flow_kg_s)),
        bus_power_residual_W=float(solution.value(
            generator.power_electric_W + pack.power_electric_W - power_motors_electric_W)),
        rotor_power_residual_W=float(solution.value(rotor.power_residual_W)),
        current_motor_A=float(solution.value(motor.current_A)),
        current_motors_total_A=float(solution.value(count_rotors * motor.current_A)),
        voltage_bus_V=float(solution.value(voltage_bus_V)),
        max_abs_connection_residual=max(abs(float(solution.value(r.value))) for r in residuals),
    )


if __name__ == "__main__":
    for count in (1, 4):
        print(f"count_rotors={count}:", solve_topology_point(count))
