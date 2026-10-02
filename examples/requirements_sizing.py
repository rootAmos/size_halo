"""Size the series-hybrid powertrain to capability requirements in the closure.

Minimum take-off mass over wing position and powertrain ratings (rubber motor
and generator peak torques, turboshaft rating, battery discharge power, rotor
disk area), subject to mass closure, hover pitch trim and every requirement
flight point having non-negative Tier 3 operating margins. Battery energy is
held at the reference (missions size it in Tier 8/9). Tail areas take the
Tier 6 sized values. Illustrative requirements, not Halo data.
"""
from dataclasses import dataclass, fields

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.core.margins import margin_report
from aircraft_closure.performance.flight_point import build_flight_point
from aircraft_closure.powertrain.topologies import SeriesHybridSizing, build_series_hybrid_from_sizing
from aircraft_closure.requirements.capability import RequirementSet
from aircraft_closure.vehicle.aircraft import MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.aircraft_mass_closure import build_reference_aircraft

area_disk_max_m2 = 15.0  # geometric limit for wing-mounted rotors (assumption)


@dataclass(frozen=True)
class RequirementsSizingResult:
    mass_takeoff_kg: float
    x_le_wing_m: float
    power_rated_motor_W: float
    power_rated_generator_W: float
    power_rated_turboshaft_W: float
    power_max_discharge_battery_W: float
    area_disk_m2: float
    hybridization_hover: float
    point_powers_W: tuple
    margins: tuple
    component_masses_kg: tuple


def build_sizing_variables(opti, initial=SeriesHybridSizing()):
    return SeriesHybridSizing(
        torque_peak_motor_Nm=opti.variable(init_guess=initial.torque_peak_motor_Nm, lower_bound=20),
        torque_peak_generator_Nm=opti.variable(init_guess=initial.torque_peak_generator_Nm, lower_bound=50),
        power_rated_turboshaft_W=opti.variable(init_guess=initial.power_rated_turboshaft_W, lower_bound=1e4),
        power_max_discharge_battery_W=opti.variable(init_guess=initial.power_max_discharge_battery_W, lower_bound=1e4),
        energy_capacity_battery_J=initial.energy_capacity_battery_J,
        area_disk_m2=opti.variable(init_guess=initial.area_disk_m2, lower_bound=2.0, upper_bound=area_disk_max_m2),
        count_rotors=initial.count_rotors,
    )


def solve_requirements_sizing(requirements=RequirementSet(), aerodynamics=None):
    aerodynamics = aerodynamics or SimpleAerodynamics()
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=1550, lower_bound=100)
    x_le_wing_m = opti.variable(init_guess=3.2, lower_bound=0.5, upper_bound=5.5)
    sizing = build_sizing_variables(opti)
    topology = build_series_hybrid_from_sizing(sizing)
    aircraft = build_reference_aircraft(x_le_wing_m, requirements.mass_payload_kg, sizing.count_rotors,
                                        area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92, topology=topology)
    breakdown = aircraft.get_mass_breakdown(StructuralDesignCondition(mass_design_kg=mass_takeoff_kg))
    total = breakdown.total()
    points = [build_flight_point(opti, aircraft, aerodynamics, condition, mass_takeoff_kg)
              for condition in requirements.flight_conditions()]
    margins = tuple(m for point in points for m in point.margins)
    opti.subject_to([
        mass_takeoff_kg == total.mass,
        total.x_cg == x_le_wing_m + 0.25 * aircraft.wing.chord_root_m(),  # hover pitch trim
    ])
    opti.subject_to([m.value >= 0 for m in margins])
    opti.minimize(mass_takeoff_kg / 1000)
    solution = opti.solve(verbose=False)
    value = lambda expression: float(solution.value(expression))
    instances = topology.instances
    hover = next(p for p in points if p.condition.mode == "hover")
    return RequirementsSizingResult(
        mass_takeoff_kg=value(mass_takeoff_kg), x_le_wing_m=value(x_le_wing_m),
        power_rated_motor_W=value(instances["motor"].component.power_rated_W),
        power_rated_generator_W=value(instances["generator"].component.power_rated_W),
        power_rated_turboshaft_W=value(sizing.power_rated_turboshaft_W),
        power_max_discharge_battery_W=value(sizing.power_max_discharge_battery_W),
        area_disk_m2=value(sizing.area_disk_m2),
        hybridization_hover=value(hover.hybridization_electric),
        point_powers_W=tuple((p.condition.label, value(p.power_shaft_rotor_W), value(p.power_battery_W),
                              value(p.generator.power_shaft_W)) for p in points),
        margins=tuple((e.label, float(e.value)) for e in margin_report(margins, solution.value)),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
    )


if __name__ == "__main__":
    result = solve_requirements_sizing()
    for label, margin in result.margins[:8]:
        print(f"{label:<40}{margin:+.4f}")
    print(result)
