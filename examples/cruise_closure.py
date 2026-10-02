"""Mass closure coupled to a cruise equilibrium through simple aerodynamics.

Extends `aircraft_mass_closure.py`: the fuselage correlation's cruise
lift-to-drag is no longer assumed but comes from the Tier 5 model at the angle
of attack where lift equals weight, solved in the same Opti. Illustrative.
"""
from dataclasses import dataclass, fields

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.vehicle.aircraft import MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.aircraft_mass_closure import acceleration_gravity_m_s2, build_reference_aircraft


@dataclass(frozen=True)
class CruiseClosureResult:
    mass_takeoff_kg: float
    x_le_wing_m: float
    alpha_cruise_deg: float
    alpha_stall_deg: float
    cl_cruise: float
    cd_cruise: float
    cd0: float
    oswald_efficiency: float
    lift_to_drag_cruise: float
    drag_cruise_N: float
    power_drag_cruise_W: float
    closure_residual_kg: float
    lift_residual_N: float
    component_masses_kg: tuple


def solve_cruise_closure(velocity_cruise_m_s=60.0, altitude_cruise_m=1000.0, mass_payload_kg=300.0,
                         cg_fraction_mac=0.25, aerodynamics=None):
    aerodynamics = aerodynamics or SimpleAerodynamics()
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=1550, lower_bound=100)
    x_le_wing_m = opti.variable(init_guess=3.2, lower_bound=0.5, upper_bound=5.5)
    alpha_cruise_deg = opti.variable(init_guess=3, lower_bound=-5, upper_bound=20)
    aircraft = build_reference_aircraft(x_le_wing_m, mass_payload_kg)
    cruise = aerodynamics.evaluate(aircraft, velocity_cruise_m_s, altitude_cruise_m, alpha_cruise_deg)
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg, velocity_cruise_m_s=velocity_cruise_m_s,
                                          altitude_cruise_m=altitude_cruise_m, lift_to_drag_cruise=cruise.lift_to_drag)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    chord_mac_m = aircraft.wing.to_asb().mean_aerodynamic_chord()
    alpha_stall_deg = aerodynamics.alpha_stall_deg(aircraft, velocity_cruise_m_s, altitude_cruise_m)
    weight_N = mass_takeoff_kg * acceleration_gravity_m_s2
    opti.subject_to([
        mass_takeoff_kg == total.mass,
        total.x_cg == x_le_wing_m + cg_fraction_mac * chord_mac_m,
        # Cruise equilibrium in the vertical direction (wing lift only, Tier 5).
        cruise.lift_N == weight_N,
        alpha_cruise_deg <= alpha_stall_deg,
    ])
    solution = opti.solve(verbose=False)
    value = lambda expression: float(solution.value(expression))
    return CruiseClosureResult(
        mass_takeoff_kg=value(mass_takeoff_kg), x_le_wing_m=value(x_le_wing_m),
        alpha_cruise_deg=value(alpha_cruise_deg), alpha_stall_deg=value(alpha_stall_deg),
        cl_cruise=value(cruise.cl), cd_cruise=value(cruise.cd), cd0=value(cruise.cd0),
        oswald_efficiency=value(cruise.oswald_efficiency), lift_to_drag_cruise=value(cruise.lift_to_drag),
        drag_cruise_N=value(cruise.drag_N), power_drag_cruise_W=value(cruise.drag_N * velocity_cruise_m_s),
        closure_residual_kg=value(mass_takeoff_kg - total.mass), lift_residual_N=value(cruise.lift_N - weight_N),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
    )


if __name__ == "__main__":
    print(solve_cruise_closure())
