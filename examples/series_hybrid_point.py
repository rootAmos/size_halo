"""Series-hybrid point coupled through the Tier 2 topology description.

The topology states what is connected; this caller owns every Opti variable,
applies the generated connection residuals and constrains the Tier 3 operating
margins to be non-negative in place of a hand-written rating list. With one rotor it reproduces `series_hybrid_point_explicit.py`.
Illustrative, not an aircraft sizing or mission model.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb

from aircraft_closure.core.margins import margin_report
from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.core.topology import connection_residuals
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.compatibility import operating_margins
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
    min_operating_margin: float
    binding_margin_label: str


def build_reference_topology(count_rotors=1, motor=None):
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
    return build_series_hybrid(motor or Motor(), generator, battery, turboshaft, Gearbox(),
                               ActuatorDiskPropulsor(), count_rotors=count_rotors)


@dataclass(frozen=True)
class PointProblem:
    """Expressions of one topology-coupled point; the Opti owns all variables."""
    topology: Any
    rotor: Any
    pack: Any
    motor: Any
    generator: Any
    engine: Any
    voltage_bus_V: Any
    power_motors_electric_W: Any
    residuals: tuple
    margins: tuple


def build_point_problem(opti, count_rotors=1, hybridization_electric=0.2, motor=None):
    """Add the coupled hover point to `opti`; thrust is left to the caller.

    Connection equalities come from the topology, hardware ratings from
    operating margins (>= 0). Only the generator domain constraint, the
    momentum residual and the prescribed source split are written here.
    """
    topology = build_reference_topology(count_rotors, motor)
    instances = topology.instances
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

    pack = instances["battery"].component.evaluate(current_battery_A, soc=0.9)
    motor = instances["motor"].component.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
    generator = instances["generator"].component.evaluate(speed_generator_rad_s, torque_generator_Nm, voltage_bus_V)
    engine = instances["turboshaft"].component.evaluate(speed_generator_rad_s * torque_turboshaft_Nm)
    gear = instances["gearbox"].component.evaluate(speed_motor_rad_s, torque_motor_Nm)
    rotor = instances["propulsor"].component.evaluate(0, asb.Atmosphere(altitude=0),
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
    residuals = tuple(connection_residuals(topology, port_values))
    margins = operating_margins(topology, port_values)
    power_motors_electric_W = count_rotors * motor.power_electric_W

    opti.subject_to([residual.value == 0 for residual in residuals])
    opti.subject_to([margin.value >= 0 for margin in margins])
    opti.subject_to([
        rotor.power_residual_W == 0,
        pack.power_electric_W == hybridization_electric * power_motors_electric_W,
        generator.power_electric_W >= 0,
    ])
    return PointProblem(topology, rotor, pack, motor, generator, engine, voltage_bus_V,
                        power_motors_electric_W, residuals, margins)


def solve_topology_point(count_rotors=1, thrust_per_rotor_N=5000.0, hybridization_electric=0.2):
    """Hover point with `hybridization_electric` of motor electrical input from the battery."""
    opti = asb.Opti()
    problem = build_point_problem(opti, count_rotors, hybridization_electric)
    opti.subject_to(problem.rotor.thrust_N == thrust_per_rotor_N)
    solution = opti.solve(verbose=False)
    return summarize_point(problem, solution, count_rotors)


def summarize_point(problem, solution, count_rotors):
    # Conversion to ordinary scalars is confined to post-solve reporting.
    report = margin_report(problem.margins, solution.value)
    return TopologyPointResult(
        thrust_N=float(solution.value(count_rotors * problem.rotor.thrust_N)),
        shaft_power_W=float(solution.value(problem.rotor.shaft_power_W)),
        battery_power_W=float(solution.value(problem.pack.power_electric_W)),
        fuel_flow_kg_s=float(solution.value(problem.engine.fuel_flow_kg_s)),
        bus_power_residual_W=float(solution.value(problem.generator.power_electric_W + problem.pack.power_electric_W
                                                  - problem.power_motors_electric_W)),
        rotor_power_residual_W=float(solution.value(problem.rotor.power_residual_W)),
        current_motor_A=float(solution.value(problem.motor.current_A)),
        current_motors_total_A=float(solution.value(count_rotors * problem.motor.current_A)),
        voltage_bus_V=float(solution.value(problem.voltage_bus_V)),
        max_abs_connection_residual=max(abs(float(solution.value(r.value))) for r in problem.residuals),
        min_operating_margin=float(report[0].value),
        binding_margin_label=report[0].label,
    )


if __name__ == "__main__":
    for count in (1, 4):
        print(f"count_rotors={count}:", solve_topology_point(count))
