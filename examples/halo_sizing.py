"""Halo-class two-rotor series-hybrid tiltrotor: simultaneous sizing, mission and energy allocation.

The Tier 9 formulation, re-baselined on the Bell XV-15 (plans 011-013):
two tip-mounted rotors, two turbogenerators on one bus with a battery, and the
validated Tier 10 models.

Requirements (user-approved 2026-10-02): 900 kg payload, 445 nm mission,
250 kt at 10,000 ft, 13,000 ft ceiling, OGE hover at 4,000 ft, engine-out
hover on one turbogenerator plus battery, stall at most 120 kt.

Models carried from Tier 10:
  * weights: Raymer GA airframe x XV-15 calibration factors; AFDD82 rotor group
    and AFDD83 gearboxes; AFDD82 engine section; AeroSandbox turboshaft
    mass-power regression as an explicit Opti equality;
  * engine: lapse sigma^0.797 and part-power knockdown (XV-15 validated);
  * hover: figure of merit 0.67 with 7 % download (XV-15 calibrated).
Assumptions specific to this case are fields of `HaloAssumptions` and are
listed in plan 013. Illustrative study, not Archer data.
"""
from dataclasses import dataclass, field, fields, replace
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
from aerosandbox.library.power_turboshaft import power_turboshaft, thermal_efficiency_turboshaft

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.controls.stability import DirectionalStability, LongitudinalStability
from aircraft_closure.core.margins import margin_above, margin_below, margin_report
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import (ClimbSegment, CruiseSegment, DescentSegment, HoverSegment,
                                               LoiterSegment, ground_distance_m)
from aircraft_closure.performance.flight_point import FlightCondition, acceleration_gravity_m_s2, build_flight_point
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor, rubber_machine
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft, deck_1120hp_part_power_model
from aircraft_closure.powertrain.topologies import build_series_hybrid
from aircraft_closure.requirements.capability import (CeilingRequirement, ClimbRequirement, HoverRequirement,
                                                      RequirementSet, SpeedRequirement)
from aircraft_closure.vehicle.aircraft import Aircraft, MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.items import FixedEquipment, FuelLoad, LandingGear, Nacelles, Payload, Systems
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation
from aircraft_closure.vehicle.surfaces import HorizontalTail, VerticalTail, Wing
from aircraft_closure.weights import afdd
from examples.xv15_performance import fit_lapse_exponent
from examples.xv15_reference import Xv15MassFactors, calibration_factors

fuel_factor = 1.1
soc_take_off = 0.95
soc_minimum = 0.3
soc_emergency_floor = 0.1


@dataclass(frozen=True)
class HaloRequirements:
    mass_payload_kg: float = 900.0
    range_m: float = 445 * 1852.0
    altitude_cruise_m: float = 10000 * u.foot
    velocity_max_m_s: float = 250 * u.knot
    altitude_hover_m: float = 4000 * u.foot
    thrust_to_weight_hover: float = 1.05
    altitude_ceiling_m: float = 13000 * u.foot
    climb_rate_ceiling_m_s: float = 0.5
    climb_rate_m_s: float = 7.5
    velocity_climb_m_s: float = 80.0
    velocity_stall_max_m_s: float = 120 * u.knot
    cl_max: float = 1.5
    duration_loiter_s: float = 1200.0
    duration_hover_s: float = 60.0
    duration_engine_out_hover_s: float = 60.0

    def requirement_set(self):
        return RequirementSet(
            mass_payload_kg=self.mass_payload_kg,
            hover=HoverRequirement(altitude_m=self.altitude_hover_m, thrust_to_weight=self.thrust_to_weight_hover),
            climb=ClimbRequirement(climb_rate_m_s=self.climb_rate_m_s, velocity_m_s=self.velocity_climb_m_s,
                                   altitude_m=1000.0),
            speed=SpeedRequirement(velocity_m_s=self.velocity_max_m_s, altitude_m=self.altitude_cruise_m),
            ceiling=CeilingRequirement(altitude_m=self.altitude_ceiling_m, climb_rate_m_s=self.climb_rate_ceiling_m_s,
                                       velocity_m_s=self.velocity_climb_m_s),
        )


@dataclass(frozen=True)
class HaloAssumptions:
    count_rotors: int = 2
    count_turbogenerators: int = 2
    count_blades: int = 3
    solidity: float = 0.089                       # XV-15
    speed_tip_m_s: float = 740 * u.foot           # XV-15 hover tip speed, also the bound
    frequency_coning_per_rev: float = 1.55        # XV-15 reading; the rotor factor calibrates it
    figure_of_merit: float = 0.67                 # XV-15 calibrated (Tier 10b)
    coefficient_airplane: float = 0.87            # assumed; ~0.85 propulsive efficiency in cruise
    download_fraction_hover: float = 0.07         # XV-15 TM X-62407 sec. 5.1
    reduction_ratio: float = 7.0                  # motor near peak-efficiency speed in hover
    speed_peak_motor_rad_s: float = 400.0
    speed_peak_generator_rad_s: float = 400.0
    aspect_ratio_wing: float = 6.12               # XV-15
    length_fuselage_m: float = 42.1 * u.foot      # XV-15
    diameter_fuselage_m: float = 5.5 * u.foot     # XV-15 (Tier 10a assumption)
    clearance_rotor_fuselage_m: float = 0.3       # XV-15 ~1 ft
    x_horizontal_tail_m: float = 11.4
    drag_area_misc_m2: float = 0.8                # assumed: tip nacelles, spinners, gear fairings
    load_factor_ultimate: float = 4.5             # XV-15
    mass_equipment_kg: float = (396 + 91 + 100) * u.lbm  # XV-15 electrical + instrumentation + autonomy
    area_wetted_nacelles_m2: float = 2 * 95 * u.foot**2
    resistance_energy_product_ohm_J: float = 1.8e6
    battery_mass_smoothing_kg: float = 10.0
    # Part-power fuel curve: the user-supplied 1,120 hp GASP deck (plan 014); GeissPartPowerModel() is the
    # XV-15-validated alternative (within 1 % of it above 50 % power).
    part_power_model: Any = field(default_factory=deck_1120hp_part_power_model)


@dataclass(frozen=True)
class HaloDesign:
    """Every sized quantity; fields may be Opti variables or numbers."""
    x_le_wing_m: Any
    area_wing_m2: Any
    area_horizontal_tail_m2: Any
    area_vertical_tail_m2: Any
    torque_peak_motor_Nm: Any
    torque_peak_generator_Nm: Any
    power_rated_turboshaft_W: Any
    mass_turboshaft_bare_kg: Any
    power_max_discharge_battery_W: Any
    energy_capacity_battery_J: Any
    area_disk_m2: Any
    mass_fuel_kg: Any


def build_halo_aircraft(design, requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                        lapse_exponent=None):
    a, d = assumptions, design
    factors = factors if factors is not None else calibration_factors()
    lapse_exponent = lapse_exponent if lapse_exponent is not None else fit_lapse_exponent()
    ratios = dict(torque_ratio=2.5, power_ratio=1.25, speed_ratio=2.5)
    motor = rubber_machine(Motor, a.speed_peak_motor_rad_s, d.torque_peak_motor_Nm, **ratios)
    generator = rubber_machine(Generator, a.speed_peak_generator_rad_s, d.torque_peak_generator_Nm, **ratios)
    radius_m = np.sqrt(d.area_disk_m2 / np.pi)
    chord_m = a.solidity * np.pi * radius_m / a.count_blades
    speed_rotor_design_rad_s = a.speed_tip_m_s / radius_m
    mass_rotors_kg = factors.rotor * afdd.mass_rotor_group_afdd82_kg(a.count_rotors, a.count_blades, radius_m, chord_m,
                                                                     a.speed_tip_m_s, a.frequency_coning_per_rev)
    mass_gearboxes_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd83_kg(
        a.count_rotors * motor.power_rated_W, speed_rotor_design_rad_s, a.speed_peak_motor_rad_s, a.count_rotors, 0.6)
    gearbox = Gearbox(reduction_ratio=a.reduction_ratio, power_rated_W=motor.power_rated_W,
                      specific_power_W_kg=motor.power_rated_W / (mass_gearboxes_kg / a.count_rotors))
    rotor = ActuatorDiskPropulsor(area_disk_m2=d.area_disk_m2, mass_kg=mass_rotors_kg / a.count_rotors,
                                  coefficient_of_performance=a.figure_of_merit,
                                  coefficient_of_performance_airplane=a.coefficient_airplane,
                                  max_shaft_power_W=gearbox.power_rated_W * gearbox.efficiency,
                                  speed_tip_max_m_s=a.speed_tip_m_s)
    battery = Battery(energy_capacity_J=d.energy_capacity_battery_J,
                      resistance_ohm=a.resistance_energy_product_ohm_J / d.energy_capacity_battery_J,
                      max_discharge_power_W=d.power_max_discharge_battery_W,
                      max_charge_power_W=0.5 * d.power_max_discharge_battery_W,
                      mass_smoothing_kg=a.battery_mass_smoothing_kg)
    turboshaft = SimpleTurboshaft(power_rated_W=d.power_rated_turboshaft_W,
                                  mass_kg=factors.powerplant * d.mass_turboshaft_bare_kg,
                                  thermal_efficiency=thermal_efficiency_turboshaft(d.mass_turboshaft_bare_kg),
                                  lapse_exponent=lapse_exponent, part_power_model=a.part_power_model)
    topology = build_series_hybrid(motor, generator, battery, turboshaft, gearbox, rotor, count_rotors=a.count_rotors,
                                   count_turbogenerators=a.count_turbogenerators)

    wing = Wing(area_m2=d.area_wing_m2, aspect_ratio=a.aspect_ratio_wing, taper_ratio=1.0, x_le_root_m=d.x_le_wing_m,
                z_m=1.2, airfoil=asb.Airfoil("naca2423"), mass_factor=factors.wing)
    x_rotor_m = d.x_le_wing_m + 0.25 * wing.chord_root_m()
    z_rotor_m = wing.z_m + 1.0
    locations = (InstalledInstance("turboshaft", x_m=x_rotor_m, z_m=wing.z_m),
                 InstalledInstance("generator", x_m=x_rotor_m, z_m=wing.z_m),
                 InstalledInstance("battery", x_m=x_rotor_m - 0.8, z_m=-0.2),
                 InstalledInstance("motor", x_m=x_rotor_m, z_m=z_rotor_m),
                 InstalledInstance("gearbox", x_m=x_rotor_m, z_m=z_rotor_m),
                 InstalledInstance("propulsor", x_m=x_rotor_m, z_m=z_rotor_m + 0.5))
    return Aircraft(
        wing=wing,
        horizontal_tail=HorizontalTail(area_m2=d.area_horizontal_tail_m2, aspect_ratio=3.27, taper_ratio=1.0,
                                       x_le_root_m=a.x_horizontal_tail_m, z_m=0.8, airfoil=asb.Airfoil("naca0015"),
                                       mass_factor=factors.tail),
        vertical_tail=VerticalTail(area_m2=d.area_vertical_tail_m2, aspect_ratio=2.33, taper_ratio=0.6,
                                   x_le_root_m=a.x_horizontal_tail_m, z_root_m=0.8, airfoil=asb.Airfoil("naca0009"),
                                   mass_factor=factors.tail),
        fuselage=Fuselage(length_m=a.length_fuselage_m, diameter_m=a.diameter_fuselage_m, mass_factor=factors.fuselage),
        landing_gear=LandingGear(length_main_m=3.0 * u.foot, length_nose_m=3.0 * u.foot, x_main_m=x_rotor_m + 0.6,
                                 x_nose_m=1.5, z_m=-0.9, is_retractable=True, mass_factor=factors.alighting_gear),
        systems=Systems(mass_avionics_uninstalled_kg=0.0, x_m=3.0, mass_factor=factors.flight_controls),
        powertrain=PowertrainInstallation(topology, locations, installation_factor=1.0),
        payload=Payload(mass_kg=requirements.mass_payload_kg, x_m=x_rotor_m),
        fuel=FuelLoad(mass_kg=d.mass_fuel_kg, x_m=x_rotor_m, z_m=wing.z_m),
        nacelles=Nacelles(mass_engines_kg=a.count_turbogenerators * d.mass_turboshaft_bare_kg,
                          count_engines=a.count_turbogenerators, area_wetted_m2=a.area_wetted_nacelles_m2,
                          x_m=x_rotor_m, z_m=wing.z_m, mass_factor=factors.powerplant),
        equipment=FixedEquipment(mass_kg=a.mass_equipment_kg, x_m=3.5),
    )


def halo_mission(requirements=HaloRequirements(), velocity_cruise_m_s=None, velocity_loiter_m_s=None):
    r = requirements
    climb = ClimbSegment(0.0, r.altitude_cruise_m, 6.0, 80.0, None, "climb")
    descent = DescentSegment(r.altitude_cruise_m, 0.0, 5.0, 80.0, None, "descent")
    distance_cruise_m = r.range_m - ground_distance_m(climb) - ground_distance_m(descent)
    return Mission((
        HoverSegment(r.duration_hover_s, 0.0, None, "take-off hover"),
        climb,
        CruiseSegment(distance_cruise_m, r.altitude_cruise_m, velocity_cruise_m_s, None, "cruise"),
        LoiterSegment(r.duration_loiter_s, r.altitude_cruise_m, velocity_loiter_m_s, None, "reserve loiter"),
        descent,
        HoverSegment(r.duration_hover_s, 0.0, None, "landing hover"),
    ))


@dataclass(frozen=True)
class HaloSizingResult:
    mass_takeoff_kg: float
    mass_empty_kg: float
    mass_fuel_kg: float
    mass_fuel_burnt_kg: float
    design: HaloDesign
    energy_battery_kWh: float
    radius_rotor_m: float
    span_m: float
    disk_loading_kg_m2: float
    wing_loading_kg_m2: float
    power_rated_motor_W: float
    power_rated_generator_W: float
    velocity_cruise_m_s: float
    velocity_loiter_m_s: float
    lift_to_drag_cruise: float
    cl_cruise: float
    static_margin: float
    cn_beta_per_rad: float
    soc_end: float
    soc_after_engine_out: float
    closure_residual_kg: float
    turboshaft_mass_power_residual: float
    thermal_efficiency_turboshaft: float
    segments: tuple
    binding: tuple
    min_margin: float
    component_masses_kg: tuple
    powertrain_masses_kg: tuple


def solve_halo_sizing(requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None, verbose=False,
                      max_iter=3000, initial=None):
    """`initial`: an earlier `HaloSizingResult` used as the initial guess (sensitivity studies)."""
    factors = factors if factors is not None else calibration_factors()
    lapse_exponent = fit_lapse_exponent()
    a, r = assumptions, requirements
    aerodynamics = SimpleAerodynamics(drag_area_misc_m2=a.drag_area_misc_m2,
                                      download_fraction_hover=a.download_fraction_hover, cl_max=r.cl_max)
    longitudinal, directional = LongitudinalStability(), DirectionalStability()
    opti = asb.Opti()

    # ---- Design variables -------------------------------------------------------------
    guess = initial.design if initial is not None else HaloDesign(
        x_le_wing_m=5.5, area_wing_m2=18.0, area_horizontal_tail_m2=4.7, area_vertical_tail_m2=4.7,
        torque_peak_motor_Nm=2500.0, torque_peak_generator_Nm=2000.0, power_rated_turboshaft_W=1.0e6,
        mass_turboshaft_bare_kg=230.0, power_max_discharge_battery_W=1.0e6, energy_capacity_battery_J=1.8e8,
        area_disk_m2=40.0, mass_fuel_kg=1000.0)
    mass_takeoff_kg = opti.variable(init_guess=initial.mass_takeoff_kg if initial else 6000.0, scale=1000.0,
                                    lower_bound=1000.0)
    design = HaloDesign(
        x_le_wing_m=opti.variable(init_guess=guess.x_le_wing_m, lower_bound=3.0, upper_bound=8.5),
        area_wing_m2=opti.variable(init_guess=guess.area_wing_m2, scale=10.0, lower_bound=5.0, upper_bound=60.0),
        area_horizontal_tail_m2=opti.variable(init_guess=guess.area_horizontal_tail_m2, lower_bound=0.5),
        area_vertical_tail_m2=opti.variable(init_guess=guess.area_vertical_tail_m2, lower_bound=0.5),
        torque_peak_motor_Nm=opti.variable(init_guess=guess.torque_peak_motor_Nm, scale=1000.0, lower_bound=100.0),
        torque_peak_generator_Nm=opti.variable(init_guess=guess.torque_peak_generator_Nm, scale=1000.0,
                                               lower_bound=100.0),
        power_rated_turboshaft_W=opti.variable(init_guess=guess.power_rated_turboshaft_W, scale=1e6, lower_bound=5e4),
        mass_turboshaft_bare_kg=opti.variable(init_guess=guess.mass_turboshaft_bare_kg, scale=100.0, lower_bound=10.0),
        power_max_discharge_battery_W=opti.variable(init_guess=guess.power_max_discharge_battery_W, scale=1e6,
                                                    lower_bound=1e4),
        energy_capacity_battery_J=opti.variable(init_guess=guess.energy_capacity_battery_J, scale=1e8, lower_bound=1e6),
        area_disk_m2=opti.variable(init_guess=guess.area_disk_m2, scale=10.0, lower_bound=5.0, upper_bound=120.0),
        mass_fuel_kg=opti.variable(init_guess=guess.mass_fuel_kg, scale=500.0, lower_bound=0.0),
    )
    mission = halo_mission(r, velocity_cruise_m_s=opti.variable(
        init_guess=initial.velocity_cruise_m_s if initial else 110.0, scale=50.0, lower_bound=60.0,
        upper_bound=r.velocity_max_m_s), velocity_loiter_m_s=opti.variable(
        init_guess=initial.velocity_loiter_m_s if initial else 80.0, scale=50.0, lower_bound=55.0,
        upper_bound=r.velocity_max_m_s))

    # ---- Aircraft, mission, requirements ----------------------------------------------
    aircraft = build_halo_aircraft(design, r, a, factors, lapse_exponent)
    instances = aircraft.powertrain.topology.instances
    flown = build_mission(opti, aircraft, aerodynamics, mission, mass_takeoff_kg, soc_take_off)
    requirement_points = [build_flight_point(opti, aircraft, aerodynamics, c, mass_takeoff_kg)
                          for c in r.requirement_set().flight_conditions()]
    cruise = next(s for s in flown.segments if isinstance(s.segment, CruiseSegment))
    loiter = next(s for s in flown.segments if isinstance(s.segment, LoiterSegment))
    # Engine-out hover: one turbogenerator plus the battery, at MTOM, from the SOC floor.
    engine_out = build_flight_point(opti, aircraft, aerodynamics, FlightCondition(
        mode="hover", altitude_m=0.0, soc=soc_minimum, active_generator_count=a.count_turbogenerators - 1,
        label="engine-out hover"), mass_takeoff_kg)
    soc_after_reserve = soc_minimum - (engine_out.battery.power_chemical_W * r.duration_engine_out_hover_s
                                       / design.energy_capacity_battery_J)

    # ---- Mass closure ------------------------------------------------------------------
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg, load_factor_ultimate=a.load_factor_ultimate,
                                          velocity_cruise_m_s=cruise.segment.velocity_m_s,
                                          altitude_cruise_m=r.altitude_cruise_m,
                                          lift_to_drag_cruise=cruise.point.aero.lift_to_drag)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    x_cg_m = total.x_cg

    # ---- Stability, geometry, stall -----------------------------------------------------
    static_margin = longitudinal.static_margin(aircraft, aerodynamics, x_cg_m, 80.0, 1000.0)
    cn_beta_per_rad = directional.cn_beta_per_rad(aircraft, aerodynamics, x_cg_m, 80.0, 1000.0)
    radius_m = instances["propulsor"].component.radius_m()
    span_m = aircraft.wing.span_m()
    density_sea_level_kg_m3 = asb.Atmosphere(altitude=0).density()
    lift_stall_N = 0.5 * density_sea_level_kg_m3 * r.velocity_stall_max_m_s**2 * design.area_wing_m2 * r.cl_max
    weight_N = mass_takeoff_kg * acceleration_gravity_m_s2

    design_margins = (
        margin_above("static_margin", static_margin, 0.10),
        margin_above("cn_beta_per_rad", cn_beta_per_rad, 0.06),
        margin_above("soc_end", flown.soc_end, soc_minimum),
        margin_above("soc_after_engine_out_hover", soc_after_reserve, soc_emergency_floor),
        margin_above("stall_lift_at_120kt", lift_stall_N, weight_N),
        margin_below("rotor_radius_m", radius_m, (span_m - a.diameter_fuselage_m) / 2 - a.clearance_rotor_fuselage_m),
    )
    all_margins = (design_margins + flown.margins + engine_out.margins
                   + tuple(m for p in requirement_points for m in p.margins))

    opti.subject_to([
        mass_takeoff_kg / total.mass == 1,                                           # mass closure
        x_cg_m == aircraft.wing.x_le_root_m + 0.25 * aircraft.wing.chord_root_m(),   # hover pitch trim
        design.mass_fuel_kg == fuel_factor * flown.mass_fuel_burnt_kg,               # fuel with reserve
        power_turboshaft(design.mass_turboshaft_bare_kg) / design.power_rated_turboshaft_W == 1,  # engine regression
    ])
    opti.subject_to([m.value >= 0 for m in all_margins])
    opti.minimize(mass_takeoff_kg / 1000)

    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    value = lambda expression: float(solution.value(expression))
    report = margin_report(all_margins, solution.value)
    altitudes = [(0, 0), (0, r.altitude_cruise_m), (r.altitude_cruise_m,) * 2, (r.altitude_cruise_m,) * 2,
                 (r.altitude_cruise_m, 0), (0, 0)]
    return HaloSizingResult(
        mass_takeoff_kg=value(mass_takeoff_kg), mass_empty_kg=value(breakdown.mass_empty_kg()),
        mass_fuel_kg=value(design.mass_fuel_kg), mass_fuel_burnt_kg=value(flown.mass_fuel_burnt_kg),
        design=HaloDesign(**{f.name: value(getattr(design, f.name)) for f in fields(HaloDesign)}),
        energy_battery_kWh=value(design.energy_capacity_battery_J) / 3.6e6,
        radius_rotor_m=value(radius_m), span_m=value(span_m),
        disk_loading_kg_m2=value(mass_takeoff_kg / (a.count_rotors * design.area_disk_m2)),
        wing_loading_kg_m2=value(mass_takeoff_kg / design.area_wing_m2),
        power_rated_motor_W=value(instances["motor"].component.power_rated_W),
        power_rated_generator_W=value(instances["generator"].component.power_rated_W),
        velocity_cruise_m_s=value(cruise.segment.velocity_m_s), velocity_loiter_m_s=value(loiter.segment.velocity_m_s),
        lift_to_drag_cruise=value(cruise.point.aero.lift_to_drag), cl_cruise=value(cruise.point.aero.cl),
        static_margin=value(static_margin), cn_beta_per_rad=value(cn_beta_per_rad), soc_end=value(flown.soc_end),
        soc_after_engine_out=value(soc_after_reserve), closure_residual_kg=value(mass_takeoff_kg - total.mass),
        turboshaft_mass_power_residual=value(power_turboshaft(design.mass_turboshaft_bare_kg)
                                             / design.power_rated_turboshaft_W - 1),
        thermal_efficiency_turboshaft=value(thermal_efficiency_turboshaft(design.mass_turboshaft_bare_kg)),
        segments=tuple(dict(label=s.segment.label, duration_s=value(s.duration_s), fuel_kg=value(s.mass_fuel_burnt_kg),
                            battery_kWh=value(s.energy_battery_chemical_J) / 3.6e6, soc_end=value(s.soc_end),
                            hybridization=value(s.point.hybridization_electric),
                            power_rotors_W=value(a.count_rotors * s.point.power_shaft_rotor_W),
                            speed_motor_rad_s=value(s.point.speed_motor_rad_s),
                            torque_motor_Nm=value(s.point.torque_motor_Nm),
                            altitude_start_m=h[0], altitude_end_m=h[1])
                       for s, h in zip(flown.segments, altitudes)),
        binding=tuple(e.label for e in report if abs(float(e.value)) < 1e-4),
        min_margin=float(report[0].value),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
        powertrain_masses_kg=tuple((item.instance_name, value(item.mass_properties.mass))
                                   for item in aircraft.powertrain.get_instance_mass_properties()),
    )


if __name__ == "__main__":
    result = solve_halo_sizing(verbose=False)
    d = result.design
    print(f"MTOM {result.mass_takeoff_kg:,.0f} kg ({result.mass_takeoff_kg / u.lbm:,.0f} lb), empty "
          f"{result.mass_empty_kg:,.0f} kg, fuel {result.mass_fuel_kg:,.0f} kg, battery {result.energy_battery_kWh:.1f} kWh")
    print(f"wing {d.area_wing_m2:.1f} m2 (span {result.span_m:.2f} m), rotor R {result.radius_rotor_m:.2f} m, "
          f"DL {result.disk_loading_kg_m2:.0f} kg/m2, W/S {result.wing_loading_kg_m2:.0f} kg/m2")
    print(f"motors {result.power_rated_motor_W / 1e3:,.0f} kW x2, turboshafts {d.power_rated_turboshaft_W / 1e3:,.0f} kW x2 "
          f"({d.mass_turboshaft_bare_kg:.0f} kg each, eta {result.thermal_efficiency_turboshaft:.3f}), battery "
          f"{d.power_max_discharge_battery_W / 1e3:,.0f} kW")
    print(f"cruise {result.velocity_cruise_m_s / u.knot:.0f} kt, L/D {result.lift_to_drag_cruise:.1f}, CL {result.cl_cruise:.2f}")
    print("binding:", ", ".join(result.binding))
    for name, mass_kg in result.component_masses_kg:
        print(f"  {name:<16}{mass_kg:9.1f} kg")
    for s in result.segments:
        print(f"  {s['label']:<15}{s['duration_s']:7.0f} s {s['fuel_kg']:7.1f} kg SOC {s['soc_end']:.3f} "
              f"h_e {s['hybridization']:.2f} {s['power_rotors_W'] / 1e3:7.0f} kW")
