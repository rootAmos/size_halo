"""Illustrative component coupling, not an aircraft sizing or mission model."""
import aerosandbox as asb
from dataclasses import dataclass
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor


@dataclass(frozen=True)
class ReferencePointResult:
    thrust_N: float
    shaft_power_W: float
    battery_power_W: float
    fuel_flow_kg_s: float
    bus_power_residual_W: float
    rotor_power_residual_W: float


def solve_reference_point():
    """Couple six components with caller-owned variables and constraints.

    One rotor represents one illustrative installation, not Halo geometry.
    The battery provides 20% of motor electrical input at this point. No time
    integration is performed, so this case makes no endurance or reserve claim.
    """
    opti = asb.Opti()
    speed_motor_rad_s = 400.0
    torque_motor_Nm = opti.variable(init_guess=200, lower_bound=0)
    torque_generator_Nm = opti.variable(init_guess=200, lower_bound=0)
    current_battery_A = opti.variable(init_guess=20, lower_bound=0)
    induced_velocity_m_s = opti.variable(init_guess=10, lower_bound=0)
    battery = Battery()
    pack = battery.evaluate(current_battery_A, soc=0.9)
    motor_component = Motor()
    motor = motor_component.evaluate(speed_motor_rad_s, torque_motor_Nm, pack.voltage_V)
    generator_component = Generator()
    generator = generator_component.evaluate(400, torque_generator_Nm, pack.voltage_V)
    engine_component = SimpleTurboshaft()
    engine = engine_component.evaluate(generator.power_shaft_W)
    gear_component = Gearbox()
    gear = gear_component.evaluate(speed_motor_rad_s, torque_motor_Nm)
    rotor_component = ActuatorDiskPropulsor()
    rotor = rotor_component.evaluate(0, asb.Atmosphere(altitude=0),
                                    shaft_power_W=gear.power_output_W,
                                    induced_velocity_m_s=induced_velocity_m_s)
    opti.subject_to([
        # Momentum theory, specified thrust, source split and electrical balance.
        rotor.power_residual_W == 0,
        rotor.thrust_N == 5000,
        pack.power_electric_W == 0.2 * motor.power_electric_W,
        generator.power_electric_W + pack.power_electric_W == motor.power_electric_W,
        # Enforce hardware ratings separately from the component equations.
        motor.power_shaft_W <= motor_component.get_limits().power_rated_W,
        torque_motor_Nm <= motor_component.get_limits().max_torque_Nm,
        speed_motor_rad_s <= motor_component.get_limits().max_speed_rad_s,
        pack.voltage_V >= motor_component.get_limits().min_voltage_V,
        pack.voltage_V <= motor_component.get_limits().max_voltage_V,
        generator.power_shaft_W <= generator_component.get_limits().power_rated_W,
        torque_generator_Nm <= generator_component.get_limits().max_torque_Nm,
        400 <= generator_component.get_limits().max_speed_rad_s,
        pack.voltage_V >= generator_component.get_limits().min_voltage_V,
        pack.voltage_V <= generator_component.get_limits().max_voltage_V,
        generator.power_electric_W >= 0,
        engine.power_shaft_W <= engine_component.get_limits().power_rated_W,
        gear.power_input_W <= gear_component.get_limits().power_rated_W,
        rotor.shaft_power_W <= rotor_component.get_limits().max_shaft_power_W,
        pack.power_electric_W <= battery.get_limits().max_discharge_power_W,
        pack.voltage_V > 0,
    ])
    solution = opti.solve(verbose=False)
    # Conversion to ordinary scalars is confined to post-solve reporting.
    return ReferencePointResult(
        thrust_N=float(solution.value(rotor.thrust_N)),
        shaft_power_W=float(solution.value(rotor.shaft_power_W)),
        battery_power_W=float(solution.value(pack.power_electric_W)),
        fuel_flow_kg_s=float(solution.value(engine.fuel_flow_kg_s)),
        bus_power_residual_W=float(solution.value(generator.power_electric_W + pack.power_electric_W - motor.power_electric_W)),
        rotor_power_residual_W=float(solution.value(rotor.power_residual_W)),
    )


if __name__ == "__main__":
    print(solve_reference_point())
