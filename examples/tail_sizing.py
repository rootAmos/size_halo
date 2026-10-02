"""Tail sizing inside the mass closure: stability, trim and failed-rotor yaw.

One Opti, minimum take-off mass, over the wing position and both tail areas:

* mass closure and hover pitch trim (CG under the common rotor station);
* cruise trim at 60 m/s, 1000 m: Cm = 0 and trimmed lift = weight, elevator
  within +/-15 deg;
* static margin >= 10 % MAC;
* directional stability Cn_beta >= 0.06 /rad;
* outboard rotor failed at 1.2 V_stall (sea level, wing-tip arm): rudder
  <= 20 deg, with the remaining rotors supplying level-flight drag.

All limits are expressed as Tier 3 margins so the report names what binds.
Illustrative requirements and configuration, not Halo data.
"""
from dataclasses import dataclass, fields

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.controls.stability import (DirectionalStability, LongitudinalStability,
                                                 failed_propulsor_yaw_moment_Nm)
from aircraft_closure.core.margins import margin_above, margin_below, margin_report
from aircraft_closure.vehicle.aircraft import MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.aircraft_mass_closure import acceleration_gravity_m_s2, build_reference_aircraft


@dataclass(frozen=True)
class TailSizingRequirements:
    static_margin_min: float = 0.10
    elevator_max_deg: float = 15.0
    cn_beta_min_per_rad: float = 0.06
    rudder_max_deg: float = 20.0
    speed_factor_minimum_control: float = 1.2
    velocity_cruise_m_s: float = 60.0
    altitude_cruise_m: float = 1000.0


@dataclass(frozen=True)
class TailSizingResult:
    mass_takeoff_kg: float
    x_le_wing_m: float
    area_horizontal_tail_m2: float
    area_vertical_tail_m2: float
    static_margin: float
    alpha_cruise_deg: float
    elevator_cruise_deg: float
    cn_beta_per_rad: float
    velocity_minimum_control_m_s: float
    rudder_failed_rotor_deg: float
    lift_to_drag_trimmed: float
    trim_drag_cd: float
    margins: tuple
    component_masses_kg: tuple


def solve_tail_sizing(requirements=TailSizingRequirements(), mass_payload_kg=300.0, count_rotors=4,
                      aerodynamics=None, longitudinal=None, directional=None):
    aerodynamics = aerodynamics or SimpleAerodynamics()
    longitudinal = longitudinal or LongitudinalStability()
    directional = directional or DirectionalStability()
    r = requirements
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=1550, lower_bound=100)
    x_le_wing_m = opti.variable(init_guess=3.2, lower_bound=0.5, upper_bound=5.5)
    area_horizontal_tail_m2 = opti.variable(init_guess=2.4, lower_bound=0.3)
    area_vertical_tail_m2 = opti.variable(init_guess=1.6, lower_bound=0.3)
    alpha_cruise_deg = opti.variable(init_guess=3.5, lower_bound=-5, upper_bound=20)
    elevator_cruise_deg = opti.variable(init_guess=-5, lower_bound=-30, upper_bound=30)
    alpha_minimum_control_deg = opti.variable(init_guess=8, lower_bound=-5, upper_bound=20)

    aircraft = build_reference_aircraft(x_le_wing_m, mass_payload_kg, count_rotors,
                                        area_horizontal_tail_m2, area_vertical_tail_m2)
    weight_N = mass_takeoff_kg * acceleration_gravity_m_s2
    # The fuselage correlation takes the untrimmed (wing) cruise L/D: the trimmed
    # value needs the CG, which needs the masses. Its exponent is -0.072, so the
    # difference is negligible; everything else uses the trimmed state.
    cruise_wing = aerodynamics.evaluate(aircraft, r.velocity_cruise_m_s, r.altitude_cruise_m, alpha_cruise_deg)
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg, velocity_cruise_m_s=r.velocity_cruise_m_s,
                                          altitude_cruise_m=r.altitude_cruise_m,
                                          lift_to_drag_cruise=cruise_wing.lift_to_drag)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    x_cg_m = total.x_cg
    trim = longitudinal.evaluate(aircraft, aerodynamics, x_cg_m, r.velocity_cruise_m_s, r.altitude_cruise_m,
                                 alpha_cruise_deg, elevator_cruise_deg)
    cruise = aerodynamics.evaluate(aircraft, r.velocity_cruise_m_s, r.altitude_cruise_m, alpha_cruise_deg,
                                   drag_increments=(trim.trim_drag,))
    static_margin = longitudinal.static_margin(aircraft, aerodynamics, x_cg_m, r.velocity_cruise_m_s,
                                               r.altitude_cruise_m)
    cn_beta_per_rad = directional.cn_beta_per_rad(aircraft, aerodynamics, x_cg_m, r.velocity_cruise_m_s,
                                                  r.altitude_cruise_m)

    # Failed outboard rotor at 1.2 V_stall, sea level, wing-tip arm.
    density_sea_level_kg_m3 = asb.Atmosphere(altitude=0).density()
    velocity_stall_m_s = np.sqrt(2 * weight_N / (density_sea_level_kg_m3 * aircraft.wing.area_m2 * aerodynamics.cl_max))
    velocity_minimum_control_m_s = r.speed_factor_minimum_control * velocity_stall_m_s
    slow = aerodynamics.evaluate(aircraft, velocity_minimum_control_m_s, 0.0, alpha_minimum_control_deg)
    yaw_moment_Nm = failed_propulsor_yaw_moment_Nm(slow.drag_N / (count_rotors - 1), aircraft.wing.span_m() / 2)
    rudder_deg = directional.rudder_for_yaw_moment_deg(aircraft, aerodynamics, x_cg_m, velocity_minimum_control_m_s,
                                                       0.0, yaw_moment_Nm)
    x_rotor_m = x_le_wing_m + 0.25 * aircraft.wing.chord_root_m()

    margins = (
        margin_above("static_margin", static_margin, r.static_margin_min),
        margin_below("elevator_cruise_up_deg", -elevator_cruise_deg, r.elevator_max_deg),
        margin_below("elevator_cruise_down_deg", elevator_cruise_deg, r.elevator_max_deg),
        margin_above("cn_beta_per_rad", cn_beta_per_rad, r.cn_beta_min_per_rad),
        margin_below("rudder_failed_rotor_deg", rudder_deg, r.rudder_max_deg),
        margin_below("alpha_cruise_deg", alpha_cruise_deg, aerodynamics.alpha_stall_deg(
            aircraft, r.velocity_cruise_m_s, r.altitude_cruise_m)),
    )
    opti.subject_to([
        mass_takeoff_kg == total.mass,
        x_cg_m == x_rotor_m,  # hover pitch trim with equal rotor thrust
        trim.cm == 0,
        trim.lift_N == weight_N,
        slow.lift_N == weight_N,
    ])
    opti.subject_to([margin.value >= 0 for margin in margins])
    opti.minimize(mass_takeoff_kg / 1000)
    solution = opti.solve(verbose=False)
    value = lambda expression: float(solution.value(expression))
    return TailSizingResult(
        mass_takeoff_kg=value(mass_takeoff_kg), x_le_wing_m=value(x_le_wing_m),
        area_horizontal_tail_m2=value(area_horizontal_tail_m2), area_vertical_tail_m2=value(area_vertical_tail_m2),
        static_margin=value(static_margin), alpha_cruise_deg=value(alpha_cruise_deg),
        elevator_cruise_deg=value(elevator_cruise_deg), cn_beta_per_rad=value(cn_beta_per_rad),
        velocity_minimum_control_m_s=value(velocity_minimum_control_m_s), rudder_failed_rotor_deg=value(rudder_deg),
        lift_to_drag_trimmed=value(trim.lift_N / (cruise.dynamic_pressure_Pa * aircraft.wing.area_m2 * cruise.cd)),
        trim_drag_cd=value(trim.trim_drag.cd),
        margins=tuple((e.label, float(e.value)) for e in margin_report(margins, solution.value)),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
    )


if __name__ == "__main__":
    result = solve_tail_sizing()
    for label, margin in result.margins:
        print(f"{label:<26}{margin:+.4f}")
    print(result)
