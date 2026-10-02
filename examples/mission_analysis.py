"""Mission analysis on a fixed aircraft: prescribed (M2) and semi-free (M3).

The aircraft geometry and powertrain are fixed (Tier 6 tails, reference
series-hybrid ratings). Take-off mass, fuel load and wing position close with
the mission in one Opti: fuel load = 1.1 x mission fuel, final SOC >= 0.3,
hover pitch trim, and every segment's operating margins >= 0.

* Prescribed: hover segments use 70 % battery power, all others none, and
  every speed is fixed; the solver only picks rotor speeds (minimum fuel).
* Semi-free: cruise speed and every segment's electric power fraction are free;
  minimum fuel spends the battery down to its SOC floor where it saves most.
Illustrative mission, not Halo data.
"""
from dataclasses import dataclass, replace

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import (ClimbSegment, CruiseSegment, DescentSegment, HoverSegment,
                                               LoiterSegment)
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.aircraft_mass_closure import build_reference_aircraft

fuel_factor = 1.1
soc_take_off = 0.95
soc_minimum = 0.3


def reference_mission(hybridization_hover=0.7, hybridization_other=0.0, velocity_cruise_m_s=60.0):
    return Mission((
        HoverSegment(60.0, 0.0, hybridization_hover, "take-off hover"),
        ClimbSegment(0.0, 1000.0, 4.0, 50.0, hybridization_other, "climb"),
        CruiseSegment(100000.0, 1000.0, velocity_cruise_m_s, hybridization_other, "cruise"),
        LoiterSegment(1200.0, 1000.0, 45.0, hybridization_other, "loiter"),
        DescentSegment(1000.0, 0.0, 3.0, 50.0, hybridization_other, "descent"),
        HoverSegment(60.0, 0.0, hybridization_hover, "landing hover"),
    ))


@dataclass(frozen=True)
class MissionAnalysisResult:
    mass_takeoff_kg: float
    mass_fuel_kg: float
    mass_fuel_burnt_kg: float
    soc_end: float
    duration_s: float
    velocity_cruise_m_s: float
    segments: tuple
    min_margin: float
    min_margin_label: str


def solve_mission(mission, aerodynamics=None, mass_payload_kg=300.0, topology=None):
    aerodynamics = aerodynamics or SimpleAerodynamics()
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=1650, lower_bound=100)
    mass_fuel_kg = opti.variable(init_guess=100, lower_bound=0)
    x_le_wing_m = opti.variable(init_guess=3.2, lower_bound=0.5, upper_bound=5.5)
    segments = tuple(_free_variables(opti, segment) for segment in mission.segments)
    aircraft = build_reference_aircraft(x_le_wing_m, mass_payload_kg, area_horizontal_tail_m2=2.16,
                                        area_vertical_tail_m2=1.92, topology=topology, mass_fuel_kg=mass_fuel_kg)
    total = aircraft.get_mass_breakdown(StructuralDesignCondition(mass_design_kg=mass_takeoff_kg)).total()
    flown = build_mission(opti, aircraft, aerodynamics, Mission(segments), mass_takeoff_kg, soc_take_off)
    opti.subject_to([
        mass_takeoff_kg == total.mass,
        total.x_cg == x_le_wing_m + 0.25 * aircraft.wing.chord_root_m(),
        mass_fuel_kg == fuel_factor * flown.mass_fuel_burnt_kg,
        flown.soc_end >= soc_minimum,
    ])
    opti.subject_to([m.value >= 0 for m in flown.margins])
    opti.minimize(mass_fuel_kg / 100)
    solution = opti.solve(verbose=False)
    value = lambda expression: float(solution.value(expression))
    margins = sorted(((value(m.value), m.label) for m in flown.margins))
    cruise = next(s for s in segments if isinstance(s, CruiseSegment))
    return MissionAnalysisResult(
        mass_takeoff_kg=value(mass_takeoff_kg), mass_fuel_kg=value(mass_fuel_kg),
        mass_fuel_burnt_kg=value(flown.mass_fuel_burnt_kg), soc_end=value(flown.soc_end),
        duration_s=value(flown.duration_s), velocity_cruise_m_s=value(cruise.velocity_m_s),
        segments=tuple((r.segment.label, value(r.duration_s), value(r.mass_fuel_burnt_kg),
                        value(r.energy_battery_chemical_J), value(r.soc_end), value(r.point.hybridization_electric),
                        value(r.point.power_shaft_rotor_W)) for r in flown.segments),
        min_margin=margins[0][0], min_margin_label=margins[0][1],
    )


def _free_variables(opti, segment):
    """Semi-free mission: replace None speeds/fractions marked free by variables."""
    if isinstance(segment, CruiseSegment) and segment.velocity_m_s is None:
        segment = replace(segment, velocity_m_s=opti.variable(init_guess=60.0, lower_bound=40.0, upper_bound=85.0))
    return segment


def solve_prescribed_mission(**kwargs):
    return solve_mission(reference_mission(), **kwargs)


def solve_semi_free_mission(**kwargs):
    return solve_mission(reference_mission(hybridization_hover=None, hybridization_other=None,
                                           velocity_cruise_m_s=None), **kwargs)


if __name__ == "__main__":
    for name, solve in (("prescribed", solve_prescribed_mission), ("semi-free", solve_semi_free_mission)):
        result = solve()
        print(f"\n{name}: MTOM {result.mass_takeoff_kg:.1f} kg, fuel {result.mass_fuel_kg:.2f} kg, "
              f"SOC end {result.soc_end:.3f}, cruise {result.velocity_cruise_m_s:.1f} m/s, "
              f"min margin {result.min_margin:+.3f} ({result.min_margin_label})")
        for label, duration, fuel, energy, soc, h_e, shaft in result.segments:
            print(f"  {label:<15}{duration:8.0f} s {fuel:7.2f} kg {energy / 3.6e6:7.2f} kWh SOC {soc:.3f} "
                  f"h_e {h_e:.2f} {shaft / 1e3:6.1f} kW/rotor")
