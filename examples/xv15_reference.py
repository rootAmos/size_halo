"""Bell XV-15 reference: the framework's mass models checked group by group.

The XV-15 is a published twin-tiltrotor of the size the Halo-class study
targets (13,000 lb design gross weight). Its group weight statement lets each
mass model be compared with what was built, and gives one calibration factor
per group: actual / predicted at the design gross weight.

Sources (accessed 2026-10-02):
  NASA TM X-62407 (1975), XV-15 Familiarization Document, sec. 3.1.2 group
  weights (Nov 1974), 3.3 geometry, 3.7-3.8 rotor and tip speeds, 4.2 load
  factors, 6.1-6.2 drive and engines, fig. 7.1.1 rotor collective modes.
  https://ntrs.nasa.gov/citations/19750016648
  NASA SP-4517 (2000), appendix A, XV-15 characteristics (length, cruise).
  W. Johnson, NDARC Theory, NASA/TP-2009-215402, ch. 19 (AFDD equations).
Values marked "assumed" are not published; plan 011 lists the reasoning.
"""
from dataclasses import dataclass, fields, replace

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
from aerosandbox.library.power_turboshaft import power_turboshaft

from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.topologies import build_mechanical_tiltrotor
from aircraft_closure.vehicle.aircraft import Aircraft
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.items import (FixedEquipment, FuelLoad, InterconnectShaft, LandingGear, Nacelles,
                                            Payload, Systems)
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation
from aircraft_closure.vehicle.surfaces import HorizontalTail, VerticalTail, Wing
from aircraft_closure.weights import afdd


@dataclass(frozen=True)
class Xv15GroupMasses:
    """Mass per weight group in kg; the comparison and calibration unit."""
    rotor: float
    wing: float
    tail: float
    fuselage: float
    alighting_gear: float
    flight_controls: float
    powerplant: float
    transmission: float
    equipment: float

    def total(self):
        return sum(getattr(self, f.name) for f in fields(self))


# Nov 1974 statement (TM X-62407 sec. 3.1.2). "flight_controls" is the
# hydraulics and flight controls group; "transmission" the transmission and
# conversion systems group; "equipment" sums heating and air conditioning (100),
# electrical (396), instrumentation (91) and miscellaneous (436).
published_groups = Xv15GroupMasses(
    rotor=1070 * u.lbm, wing=873 * u.lbm, tail=209 * u.lbm, fuselage=1442 * u.lbm, alighting_gear=508 * u.lbm,
    flight_controls=934 * u.lbm, powerplant=1754 * u.lbm, transmission=1263 * u.lbm, equipment=1023 * u.lbm)


@dataclass(frozen=True)
class Xv15Reference:
    """Published XV-15 data in SI, plus the stated assumptions."""
    mass_design_kg: float = 13000 * u.lbm
    mass_basic_empty_kg: float = 9076 * u.lbm
    # Useful load at design gross weight: crew 400, additional (research)
    # equipment 997, oil and trapped fluids 138, payload 899; fuel 1,490.
    mass_useful_non_fuel_kg: float = (400 + 997 + 138 + 899) * u.lbm
    mass_fuel_kg: float = 1490 * u.lbm
    load_factor_ultimate: float = 1.5 * 3.0              # limit 3.0 at design weight, factor of safety 1.5
    velocity_cruise_m_s: float = 200 * u.knot            # SP-4517: 200 kt cruise at 20,000 ft
    altitude_cruise_m: float = 20000 * u.foot
    lift_to_drag_cruise: float = 8.0                     # assumed; Raymer fuselage exponent is -0.072
    area_wing_m2: float = 169.0 * u.foot**2
    aspect_ratio_wing: float = 6.12
    area_horizontal_tail_m2: float = 50.25 * u.foot**2
    aspect_ratio_horizontal_tail: float = 3.27
    area_vertical_tail_m2: float = 50.5 * u.foot**2      # two fins, modelled as one of equal area
    aspect_ratio_vertical_tail: float = 2.33
    length_tail_m: float = 22.4 * u.foot                 # wing to horizontal-tail quarter chords
    length_fuselage_m: float = 42.1 * u.foot
    diameter_fuselage_m: float = 5.5 * u.foot            # assumed: 5 ft cabin width plus structure
    length_main_gear_m: float = 3.0 * u.foot             # assumed strut lengths
    length_nose_gear_m: float = 3.0 * u.foot
    count_rotors: int = 2
    count_blades: int = 3
    radius_rotor_m: float = 12.5 * u.foot
    chord_blade_m: float = 14.0 * u.inch
    speed_tip_hover_m_s: float = 740 * u.foot
    speed_rotor_hover_rad_s: float = 565 * u.rpm
    frequency_coning_per_rev: float = 1.55               # fig. 7.1.1: ~880 cpm at 565 rpm; read off a plot
    power_takeoff_engine_W: float = 1550 * u.hp
    speed_engine_rad_s: float = 20000 * u.rpm
    count_gearboxes: int = 3                             # assumed: two proprotor gearboxes plus interconnect set
    length_interconnect_m: float = 32.17 * u.foot        # rotor centreline spacing
    area_wetted_nacelles_m2: float = 2 * 95 * u.foot**2  # assumed: ~9 ft x 3.3 ft cylinders
    mass_fixed_equipment_kg: float = 1023 * u.lbm


@dataclass(frozen=True)
class Xv15MassFactors:
    """Calibration (technology) factor per group; 1.0 is the uncalibrated model."""
    rotor: float = 1.0
    wing: float = 1.0
    tail: float = 1.0
    fuselage: float = 1.0
    alighting_gear: float = 1.0
    flight_controls: float = 1.0
    powerplant: float = 1.0
    transmission: float = 1.0


def mass_turboshaft_from_power_kg(power_W):
    """Invert AeroSandbox's turboshaft mass-power regression by an explicit equality solve."""
    opti = asb.Opti()
    mass_kg = opti.variable(init_guess=250.0, lower_bound=1.0)
    opti.subject_to(power_turboshaft(mass_kg) / power_W == 1)
    return float(opti.solve(verbose=False).value(mass_kg))


def build_xv15_aircraft(reference=Xv15Reference(), factors=Xv15MassFactors(), mass_engine_kg=None):
    """XV-15 geometry from framework components; CG stations are approximate (not validated here)."""
    r = reference
    mass_engine_kg = mass_engine_kg if mass_engine_kg is not None else mass_turboshaft_from_power_kg(
        r.power_takeoff_engine_W)
    power_drive_W = r.count_rotors * r.power_takeoff_engine_W
    mass_rotor_group_kg = afdd.mass_rotor_group_afdd82_kg(r.count_rotors, r.count_blades, r.radius_rotor_m,
                                                          r.chord_blade_m, r.speed_tip_hover_m_s,
                                                          r.frequency_coning_per_rev)
    mass_gearboxes_kg = afdd.mass_gearbox_rotor_shaft_afdd83_kg(power_drive_W, r.speed_rotor_hover_rad_s,
                                                                r.speed_engine_rad_s, r.count_gearboxes, 0.6)
    turboshaft = SimpleTurboshaft(power_rated_W=r.power_takeoff_engine_W,
                                  specific_power_W_kg=r.power_takeoff_engine_W / (factors.powerplant * mass_engine_kg))
    gearbox = Gearbox(reduction_ratio=r.speed_engine_rad_s / r.speed_rotor_hover_rad_s,
                      power_rated_W=r.power_takeoff_engine_W,
                      specific_power_W_kg=r.power_takeoff_engine_W
                      / (factors.transmission * mass_gearboxes_kg / r.count_rotors))
    rotor = ActuatorDiskPropulsor(area_disk_m2=np.pi * r.radius_rotor_m**2,
                                  mass_kg=factors.rotor * mass_rotor_group_kg / r.count_rotors,
                                  max_shaft_power_W=r.power_takeoff_engine_W)
    topology = build_mechanical_tiltrotor(turboshaft, gearbox, rotor, r.count_rotors)

    wing = Wing(area_m2=r.area_wing_m2, aspect_ratio=r.aspect_ratio_wing, taper_ratio=1.0, x_le_root_m=5.6,
                z_m=1.2, airfoil=asb.Airfoil("naca2423"), mass_factor=factors.wing)
    x_quarter_chord_wing_m = wing.x_le_root_m + 0.25 * wing.chord_root_m()
    horizontal_tail = HorizontalTail(area_m2=r.area_horizontal_tail_m2, aspect_ratio=r.aspect_ratio_horizontal_tail,
                                     taper_ratio=1.0, x_le_root_m=0.0, z_m=0.8, airfoil=asb.Airfoil("naca0015"),
                                     mass_factor=factors.tail)
    horizontal_tail = replace(horizontal_tail, x_le_root_m=x_quarter_chord_wing_m + r.length_tail_m
                              - 0.25 * horizontal_tail.chord_root_m())
    vertical_tail = VerticalTail(area_m2=r.area_vertical_tail_m2, aspect_ratio=r.aspect_ratio_vertical_tail,
                                 taper_ratio=0.6, x_le_root_m=horizontal_tail.x_le_root_m, z_root_m=0.8,
                                 airfoil=asb.Airfoil("naca0009"), mass_factor=factors.tail)
    x_rotor_m = x_quarter_chord_wing_m
    locations = (InstalledInstance("turboshaft", x_m=x_rotor_m + 0.5, z_m=wing.z_m + 0.3),
                 InstalledInstance("gearbox", x_m=x_rotor_m, z_m=wing.z_m + 0.5),
                 InstalledInstance("propulsor", x_m=x_rotor_m, z_m=wing.z_m + 1.5))
    return Aircraft(
        wing=wing,
        horizontal_tail=horizontal_tail,
        vertical_tail=vertical_tail,
        fuselage=Fuselage(length_m=r.length_fuselage_m, diameter_m=r.diameter_fuselage_m, mass_factor=factors.fuselage),
        landing_gear=LandingGear(length_main_m=r.length_main_gear_m, length_nose_m=r.length_nose_gear_m,
                                 x_main_m=x_rotor_m + 0.6, x_nose_m=1.2, z_m=-0.9, is_retractable=True,
                                 mass_factor=factors.alighting_gear),
        systems=Systems(mass_avionics_uninstalled_kg=0.0, x_m=3.0, mass_factor=factors.flight_controls),
        powertrain=PowertrainInstallation(topology, locations, installation_factor=1.0),
        payload=Payload(mass_kg=r.mass_useful_non_fuel_kg, x_m=3.5),
        fuel=FuelLoad(mass_kg=r.mass_fuel_kg, x_m=x_rotor_m, z_m=wing.z_m),
        nacelles=Nacelles(mass_engines_kg=r.count_rotors * mass_engine_kg,
                          count_engines=r.count_rotors, area_wetted_m2=r.area_wetted_nacelles_m2,
                          x_m=x_rotor_m, z_m=wing.z_m + 0.3, mass_factor=factors.powerplant),
        drive_shaft=InterconnectShaft(power_drive_limit_W=power_drive_W, speed_rotor_rad_s=r.speed_rotor_hover_rad_s,
                                      length_m=r.length_interconnect_m, x_m=x_rotor_m, z_m=wing.z_m,
                                      mass_factor=factors.transmission),
        equipment=FixedEquipment(mass_kg=r.mass_fixed_equipment_kg, x_m=4.0),
    )


def design_condition(mass_design_kg, reference=Xv15Reference()):
    return StructuralDesignCondition(mass_design_kg=mass_design_kg, load_factor_ultimate=reference.load_factor_ultimate,
                                     velocity_cruise_m_s=reference.velocity_cruise_m_s,
                                     altitude_cruise_m=reference.altitude_cruise_m,
                                     lift_to_drag_cruise=reference.lift_to_drag_cruise)


def group_masses(aircraft, condition):
    """Framework mass breakdown mapped onto the XV-15 weight groups (may be symbolic)."""
    breakdown = aircraft.get_mass_breakdown(condition)
    installed = {item.instance_name: item.mass_properties.mass
                 for item in aircraft.powertrain.get_instance_mass_properties()}
    return Xv15GroupMasses(
        rotor=installed["propulsor"],
        wing=breakdown.wing.mass,
        tail=breakdown.horizontal_tail.mass + breakdown.vertical_tail.mass,
        fuselage=breakdown.fuselage.mass,
        alighting_gear=breakdown.landing_gear.mass,
        flight_controls=breakdown.systems.mass,
        powerplant=installed["turboshaft"] + breakdown.nacelles.mass,
        transmission=installed["gearbox"] + breakdown.drive_shaft.mass,
        equipment=breakdown.equipment.mass,
    )


def compare_groups(reference=Xv15Reference(), factors=Xv15MassFactors(), mass_engine_kg=None):
    """Predicted group masses at the published design gross weight."""
    aircraft = build_xv15_aircraft(reference, factors, mass_engine_kg)
    predicted = group_masses(aircraft, design_condition(reference.mass_design_kg, reference))
    return Xv15GroupMasses(**{f.name: float(getattr(predicted, f.name)) for f in fields(Xv15GroupMasses)})


def calibration_factors(reference=Xv15Reference(), mass_engine_kg=None):
    """actual / predicted per group, so the calibrated model reproduces the statement at design weight."""
    predicted = compare_groups(reference, Xv15MassFactors(), mass_engine_kg)
    return Xv15MassFactors(**{f.name: getattr(published_groups, f.name) / getattr(predicted, f.name)
                              for f in fields(Xv15MassFactors)})


@dataclass(frozen=True)
class Xv15ClosureResult:
    mass_takeoff_kg: float
    mass_empty_kg: float
    closure_residual_kg: float
    groups: Xv15GroupMasses


def solve_xv15_closure(reference=Xv15Reference(), factors=Xv15MassFactors(), mass_engine_kg=None):
    """Take-off mass that closes the XV-15 useful load on the framework's mass models.

    Geometry, engines and rotors stay at their published sizes (an analysis,
    not a sizing): only the take-off mass feeding the correlations is solved.
    """
    aircraft = build_xv15_aircraft(reference, factors, mass_engine_kg)
    opti = asb.Opti()
    mass_takeoff_kg = opti.variable(init_guess=reference.mass_design_kg, scale=1000, lower_bound=1000)
    condition = design_condition(mass_takeoff_kg, reference)
    total = aircraft.get_mass_breakdown(condition).total()
    opti.subject_to(mass_takeoff_kg / total.mass == 1)
    solution = opti.solve(verbose=False)
    value = lambda expression: float(solution.value(expression))
    groups = group_masses(aircraft, condition)
    return Xv15ClosureResult(
        mass_takeoff_kg=value(mass_takeoff_kg),
        mass_empty_kg=value(aircraft.get_mass_breakdown(condition).mass_empty_kg()),
        closure_residual_kg=value(mass_takeoff_kg - total.mass),
        groups=Xv15GroupMasses(**{f.name: value(getattr(groups, f.name)) for f in fields(Xv15GroupMasses)}),
    )


if __name__ == "__main__":
    predicted = compare_groups()
    print(f"{'group':<17}{'predicted lb':>13}{'actual lb':>11}{'factor':>8}")
    for f in fields(Xv15GroupMasses):
        p, a = getattr(predicted, f.name), getattr(published_groups, f.name)
        print(f"{f.name:<17}{p / u.lbm:13.0f}{a / u.lbm:11.0f}{a / p:8.2f}")
    print(f"{'basic empty':<17}{predicted.total() / u.lbm:13.0f}{published_groups.total() / u.lbm:11.0f}")
    for label, factors in (("uncalibrated", Xv15MassFactors()), ("calibrated", calibration_factors())):
        closure = solve_xv15_closure(factors=factors)
        print(f"{label} closure: take-off {closure.mass_takeoff_kg / u.lbm:,.0f} lb, "
              f"empty {closure.mass_empty_kg / u.lbm:,.0f} lb")
