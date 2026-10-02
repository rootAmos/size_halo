"""Reference aircraft mass closure and CG placement in one explicit Opti.

The take-off mass is an Opti variable that feeds every empirical correlation
through the structural design condition; one equality closes the loop. The
wing position is solved so the CG sits at a chosen fraction of the MAC (a
reference placement, not a stability criterion: that is Tier 6).
Illustrative configuration, not Halo data; no aerodynamics or mission yet.
"""
from dataclasses import dataclass, fields

import aerosandbox as asb

from aircraft_closure.vehicle.aircraft import Aircraft, MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.items import LandingGear, Payload, Systems
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation
from aircraft_closure.vehicle.surfaces import HorizontalTail, VerticalTail, Wing
from examples.series_hybrid_point import build_reference_topology

acceleration_gravity_m_s2 = 9.80665


@dataclass(frozen=True)
class ClosureResult:
    mass_takeoff_kg: float
    mass_empty_kg: float
    mass_payload_kg: float
    x_le_wing_m: float
    x_cg_m: float
    cg_fraction_mac: float
    closure_residual_kg: float
    thrust_hover_per_rotor_N: float
    component_masses_kg: tuple


def build_reference_aircraft(x_le_wing_m=2.5, mass_payload_kg=300.0, count_rotors=4,
                             area_horizontal_tail_m2=2.4, area_vertical_tail_m2=1.6):
    """Tiltrotor-like layout: rotor strings at the wing quarter chord, engine aft.

    All rotors share one x station, so hover pitch trim with equal thrust needs
    the CG under that station (25 % of the rectangular wing's MAC).
    """
    wing = Wing(x_le_root_m=x_le_wing_m)
    x_rotor_m = x_le_wing_m + 0.25 * wing.chord_root_m()
    topology = build_reference_topology(count_rotors)
    locations = (
        InstalledInstance("turboshaft", x_m=4.6, z_m=0.1),
        InstalledInstance("generator", x_m=4.1, z_m=0.1),
        InstalledInstance("battery", x_m=2.6, z_m=-0.3),
        InstalledInstance("motor", x_m=x_rotor_m, z_m=wing.z_m),
        InstalledInstance("gearbox", x_m=x_rotor_m, z_m=wing.z_m),
        InstalledInstance("propulsor", x_m=x_rotor_m, z_m=wing.z_m + 0.4),
    )
    return Aircraft(
        wing=wing,
        horizontal_tail=HorizontalTail(area_m2=area_horizontal_tail_m2),
        vertical_tail=VerticalTail(area_m2=area_vertical_tail_m2),
        fuselage=Fuselage(),
        landing_gear=LandingGear(),
        systems=Systems(),
        powertrain=PowertrainInstallation(topology, locations, installation_factor=1.1),
        payload=Payload(mass_kg=mass_payload_kg),
    )


def solve_mass_closure(mass_payload_kg=300.0, cg_fraction_mac=0.25, count_rotors=4):
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=1800, lower_bound=100)
    x_le_wing_m = opti.variable(init_guess=2.5, lower_bound=0.5, upper_bound=5.5)
    aircraft = build_reference_aircraft(x_le_wing_m, mass_payload_kg, count_rotors)
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    chord_mac_m = aircraft.wing.to_asb().mean_aerodynamic_chord()
    opti.subject_to([
        # Mass closure: the design mass every correlation used equals the sum.
        mass_takeoff_kg == total.mass,
        # Reference CG placement by moving the wing (and the rotors with it).
        total.x_cg == x_le_wing_m + cg_fraction_mac * chord_mac_m,
    ])
    solution = opti.solve(verbose=False)
    # Conversion to ordinary scalars is confined to post-solve reporting.
    value = lambda expression: float(solution.value(expression))
    return ClosureResult(
        mass_takeoff_kg=value(mass_takeoff_kg),
        mass_empty_kg=value(breakdown.mass_empty_kg()),
        mass_payload_kg=value(breakdown.payload.mass),
        x_le_wing_m=value(x_le_wing_m),
        x_cg_m=value(total.x_cg),
        cg_fraction_mac=value((total.x_cg - x_le_wing_m) / chord_mac_m),
        closure_residual_kg=value(mass_takeoff_kg - total.mass),
        thrust_hover_per_rotor_N=value(mass_takeoff_kg * acceleration_gravity_m_s2 / count_rotors),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
    )


if __name__ == "__main__":
    result = solve_mass_closure()
    for name, mass_kg in result.component_masses_kg:
        print(f"{name:<16}{mass_kg:9.1f} kg")
    print(result)
