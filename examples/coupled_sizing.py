"""Simultaneous aircraft sizing, mission optimization and energy allocation.

One explicit Opti, readable top to bottom (architecture: the aircraft-level
formulation must stay visible):

Design variables      MTOM, fuel load, wing position and area, tail areas,
                      motor and generator peak torques (McDonald rubber
                      machines), turboshaft rating, battery power and energy,
                      rotor disk area.
Mission variables     cruise speed; electric power fraction of every segment
                      (energy allocation); per-point alpha, rotor speed,
                      battery current and generator torque (flight points).
Constraints           mass closure; hover pitch trim; Tier 7 requirements at
                      MTOM; Tier 8 reference mission with fuel reserve and SOC
                      floor; engine-out reserve (60 s battery-only hover from
                      the SOC floor down to 0.10); static margin, Cn_beta and
                      failed-rotor rudder; every operating margin >= 0.

On mass alone a battery never pays for itself here (about 0.9 MJ/kg against
about 12.9 MJ/kg of shaft energy from fuel at 30 % thermal efficiency); the
engine-out reserve is what sizes it, as redundancy does in a series hybrid.
Objective             minimum MTOM, or minimum mission fuel.

Illustrative requirements and mission, not Halo data.
"""
from dataclasses import dataclass, fields, replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.controls.stability import (DirectionalStability, LongitudinalStability,
                                                 failed_propulsor_yaw_moment_Nm)
from aircraft_closure.core.margins import margin_above, margin_below, margin_report
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import CruiseSegment
from aircraft_closure.performance.flight_point import FlightCondition, acceleration_gravity_m_s2, build_flight_point
from aircraft_closure.powertrain.topologies import SeriesHybridSizing, build_series_hybrid_from_sizing
from aircraft_closure.requirements.capability import RequirementSet
from aircraft_closure.vehicle.aircraft import MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.surfaces import Wing
from examples.aircraft_mass_closure import build_reference_aircraft
from examples.mission_analysis import fuel_factor, reference_mission, soc_minimum, soc_take_off

duration_engine_out_hover_s = 60.0
soc_emergency_floor = 0.1


@dataclass(frozen=True)
class CoupledSizingResult:
    objective: str
    mass_takeoff_kg: float
    mass_empty_kg: float
    mass_fuel_kg: float
    mass_fuel_burnt_kg: float
    energy_battery_kWh: float
    area_wing_m2: float
    area_horizontal_tail_m2: float
    area_vertical_tail_m2: float
    area_disk_m2: float
    power_rated_motor_W: float
    power_rated_generator_W: float
    power_rated_turboshaft_W: float
    power_max_discharge_battery_W: float
    velocity_cruise_m_s: float
    lift_to_drag_cruise: float
    static_margin: float
    cn_beta_per_rad: float
    rudder_failed_rotor_deg: float
    soc_end: float
    closure_residual_kg: float
    segments: tuple
    binding: tuple
    min_margin: float
    component_masses_kg: tuple
    powertrain_masses_kg: tuple
    torque_peak_motor_Nm: float
    torque_peak_generator_Nm: float


def solve_coupled_sizing(objective="mass_takeoff", requirements=RequirementSet(), mission=None,
                         aerodynamics=None, verbose=False, max_iter=3000):
    aerodynamics = aerodynamics or SimpleAerodynamics()
    longitudinal, directional = LongitudinalStability(), DirectionalStability()
    mission = mission or reference_mission(hybridization_hover=None, hybridization_other=None,
                                           velocity_cruise_m_s=None)
    opti = asb.Opti()

    # ---- Design variables -------------------------------------------------------------
    mass_takeoff_kg = opti.variable(init_guess=1300, scale=1000, lower_bound=300)
    mass_fuel_kg = opti.variable(init_guess=30, scale=30, lower_bound=0)
    x_le_wing_m = opti.variable(init_guess=3.0, lower_bound=0.5, upper_bound=5.5)
    area_wing_m2 = opti.variable(init_guess=11, lower_bound=3, upper_bound=30)
    area_horizontal_tail_m2 = opti.variable(init_guess=2.0, lower_bound=0.3)
    area_vertical_tail_m2 = opti.variable(init_guess=1.8, lower_bound=0.3)
    sizing = SeriesHybridSizing(
        torque_peak_motor_Nm=opti.variable(init_guess=150, scale=100, lower_bound=20),
        torque_peak_generator_Nm=opti.variable(init_guess=400, scale=300, lower_bound=50),
        power_rated_turboshaft_W=opti.variable(init_guess=2.5e5, scale=1e5, lower_bound=1e4),
        power_max_discharge_battery_W=opti.variable(init_guess=2.5e5, scale=1e5, lower_bound=1e4),
        energy_capacity_battery_J=opti.variable(init_guess=7e7, scale=5e7, lower_bound=1e6),
        area_disk_m2=opti.variable(init_guess=8, lower_bound=2, upper_bound=15),
        # Energy and power ratings are sized together, so the battery mass uses a
        # smooth maximum (overestimate <= 2 kg x ln 2) instead of the exact kink.
        battery_mass_smoothing_kg=2.0,
    )
    # Mission variable: cruise speed (the semi-free mission leaves it as None).
    segments = tuple(replace(s, velocity_m_s=opti.variable(init_guess=55, lower_bound=40, upper_bound=85))
                     if isinstance(s, CruiseSegment) and s.velocity_m_s is None else s for s in mission.segments)

    # ---- Aircraft built from the design variables ------------------------------------
    topology = build_series_hybrid_from_sizing(sizing)
    aircraft = build_reference_aircraft(x_le_wing_m, requirements.mass_payload_kg, sizing.count_rotors,
                                        area_horizontal_tail_m2, area_vertical_tail_m2, topology=topology,
                                        wing=Wing(area_m2=area_wing_m2), mass_fuel_kg=mass_fuel_kg)

    # ---- Mission (energy allocation per segment) and requirements ---------------------
    flown = build_mission(opti, aircraft, aerodynamics, Mission(segments), mass_takeoff_kg, soc_take_off)
    requirement_points = [build_flight_point(opti, aircraft, aerodynamics, c, mass_takeoff_kg)
                          for c in requirements.flight_conditions()]
    cruise = next(r for r in flown.segments if isinstance(r.segment, CruiseSegment))
    # Engine-out reserve: hover on battery alone at MTOM, starting from the
    # mission SOC floor, for the reserve time without passing the emergency floor.
    engine_out = build_flight_point(opti, aircraft, aerodynamics, FlightCondition(
        mode="hover", altitude_m=0.0, soc=soc_minimum, hybridization_electric=1.0, label="engine-out hover"),
        mass_takeoff_kg)
    soc_after_reserve = soc_minimum - (engine_out.battery.power_chemical_W * duration_engine_out_hover_s
                                       / sizing.energy_capacity_battery_J)

    # ---- Mass closure, with the structural condition fed by the solved cruise ---------
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg, velocity_cruise_m_s=cruise.segment.velocity_m_s,
                                          lift_to_drag_cruise=cruise.point.aero.lift_to_drag)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    x_cg_m = total.x_cg

    # ---- Stability and failed-rotor control -------------------------------------------
    static_margin = longitudinal.static_margin(aircraft, aerodynamics, x_cg_m, 60.0, 1000.0)
    cn_beta_per_rad = directional.cn_beta_per_rad(aircraft, aerodynamics, x_cg_m, 60.0, 1000.0)
    weight_N = mass_takeoff_kg * acceleration_gravity_m_s2
    velocity_stall_m_s = np.sqrt(2 * weight_N / (asb.Atmosphere(altitude=0).density() * area_wing_m2
                                                 * aerodynamics.cl_max))
    velocity_minimum_control_m_s = 1.2 * velocity_stall_m_s
    alpha_minimum_control_deg = opti.variable(init_guess=8, lower_bound=-5, upper_bound=20)
    slow = aerodynamics.evaluate(aircraft, velocity_minimum_control_m_s, 0.0, alpha_minimum_control_deg)
    rudder_deg = directional.rudder_for_yaw_moment_deg(
        aircraft, aerodynamics, x_cg_m, velocity_minimum_control_m_s, 0.0,
        failed_propulsor_yaw_moment_Nm(slow.drag_N / (sizing.count_rotors - 1), aircraft.wing.span_m() / 2))

    design_margins = (
        margin_above("static_margin", static_margin, 0.10),
        margin_above("cn_beta_per_rad", cn_beta_per_rad, 0.06),
        margin_below("rudder_failed_rotor_deg", rudder_deg, 20.0),
        margin_above("soc_end", flown.soc_end, soc_minimum),
        margin_above("soc_after_engine_out_hover", soc_after_reserve, soc_emergency_floor),
    )
    all_margins = (design_margins + flown.margins + engine_out.margins
                   + tuple(m for p in requirement_points for m in p.margins))

    opti.subject_to([
        mass_takeoff_kg / total.mass == 1,                                  # mass closure
        x_cg_m == x_le_wing_m + 0.25 * aircraft.wing.chord_root_m(),        # hover pitch trim
        mass_fuel_kg == fuel_factor * flown.mass_fuel_burnt_kg,             # fuel load with reserve
        slow.lift_N / weight_N == 1,                                        # level flight at V_mc
    ])
    opti.subject_to([m.value >= 0 for m in all_margins])
    opti.minimize(mass_takeoff_kg / 1000 if objective == "mass_takeoff" else mass_fuel_kg / 10)

    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    value = lambda expression: float(solution.value(expression))
    report = margin_report(all_margins, solution.value)
    instances = topology.instances
    return CoupledSizingResult(
        objective=objective,
        mass_takeoff_kg=value(mass_takeoff_kg), mass_empty_kg=value(breakdown.mass_empty_kg()),
        mass_fuel_kg=value(mass_fuel_kg), mass_fuel_burnt_kg=value(flown.mass_fuel_burnt_kg),
        energy_battery_kWh=value(sizing.energy_capacity_battery_J) / 3.6e6,
        area_wing_m2=value(area_wing_m2), area_horizontal_tail_m2=value(area_horizontal_tail_m2),
        area_vertical_tail_m2=value(area_vertical_tail_m2), area_disk_m2=value(sizing.area_disk_m2),
        power_rated_motor_W=value(instances["motor"].component.power_rated_W),
        power_rated_generator_W=value(instances["generator"].component.power_rated_W),
        power_rated_turboshaft_W=value(sizing.power_rated_turboshaft_W),
        power_max_discharge_battery_W=value(sizing.power_max_discharge_battery_W),
        velocity_cruise_m_s=value(cruise.segment.velocity_m_s),
        lift_to_drag_cruise=value(cruise.point.aero.lift_to_drag),
        static_margin=value(static_margin), cn_beta_per_rad=value(cn_beta_per_rad),
        rudder_failed_rotor_deg=value(rudder_deg), soc_end=value(flown.soc_end),
        closure_residual_kg=value(mass_takeoff_kg - total.mass),
        segments=tuple((r.segment.label, value(r.duration_s), value(r.mass_fuel_burnt_kg),
                        value(r.energy_battery_chemical_J), value(r.soc_end), value(r.point.hybridization_electric),
                        value(r.point.power_shaft_rotor_W), value(r.point.speed_motor_rad_s),
                        value(r.point.torque_motor_Nm)) for r in flown.segments),
        binding=tuple(e.label for e in report if abs(float(e.value)) < 1e-5),
        min_margin=float(report[0].value),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
        powertrain_masses_kg=tuple((item.instance_name, value(item.mass_properties.mass))
                                   for item in aircraft.powertrain.get_instance_mass_properties()),
        torque_peak_motor_Nm=value(sizing.torque_peak_motor_Nm),
        torque_peak_generator_Nm=value(sizing.torque_peak_generator_Nm),
    )


if __name__ == "__main__":
    for objective in ("mass_takeoff", "fuel"):
        r = solve_coupled_sizing(objective)
        print(f"\n=== minimum {objective}: MTOM {r.mass_takeoff_kg:.1f} kg, empty {r.mass_empty_kg:.1f} kg, "
              f"fuel {r.mass_fuel_kg:.2f} kg, battery {r.energy_battery_kWh:.1f} kWh")
        print(f"wing {r.area_wing_m2:.2f} m2, tails {r.area_horizontal_tail_m2:.2f}/{r.area_vertical_tail_m2:.2f} m2, "
              f"disk {r.area_disk_m2:.2f} m2, motor {r.power_rated_motor_W / 1e3:.1f} kW, turboshaft "
              f"{r.power_rated_turboshaft_W / 1e3:.1f} kW, cruise {r.velocity_cruise_m_s:.1f} m/s, L/D {r.lift_to_drag_cruise:.2f}")
        print("binding:", ", ".join(r.binding))
        for label, duration, fuel, energy, soc, h_e, shaft, *_ in r.segments:
            print(f"  {label:<15}{duration:7.0f} s {fuel:6.2f} kg {energy / 3.6e6:6.2f} kWh SOC {soc:.3f} h_e {h_e:.2f}")
