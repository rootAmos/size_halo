"""Tier 14 trajectory optimization on the sized Halo reference aircraft (plan 019).

The aircraft is fixed: `solve_halo_sizing()` (plan 036: the trajectory model carries the reference's thermal,
redundancy and unit-machine features; earlier: 780 kg payload, 210 kt, fixed 2 x 1,120 hp
turboshafts, equivalent-circuit battery since plan 022; Tier 14 itself used the 900 kg constant-battery
aircraft, `requirements_tier16` / `assumptions_tier16`) is sized first, then `build_halo_aircraft(result.design)` gives the numeric aircraft
that every trajectory flies. Each problem is its own `asb.Opti`, separate from sizing.

Problems:
  * minimum-energy transition: hover at 500 ft to wing-borne flight at 1.3 x the airplane-mode
    stall speed, altitude held within +/-30 m, free final time, nacelle rate <= 8 deg/s, through
    the conversion corridor; compared with a naive prescribed conversion (linear nacelle schedule,
    constant acceleration, level flight);
  * minimum time to climb: from hover at sea level to 10,000 ft at the sizing cruise speed, power
    limited by the lapsed turboshafts (through the generators) plus the battery discharge rating.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.trajectory.tiltrotor import (ConversionCorridor, TiltrotorPointMass, TrajectoryGuess,
                                                   TrajectoryLimits, build_tiltrotor_trajectory)
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, build_halo_aerodynamics, build_halo_aircraft,
                                  soc_take_off, solve_halo_sizing)

altitude_transition_m = 500 * u.foot
altitude_band_transition_m = 30.0
velocity_hover_m_s = 1.0          # "hover": the speed-gamma states need a nonzero speed
ratio_transition_end_stall = 1.3


@dataclass(frozen=True)
class HaloTrajectoryCase:
    sizing: Any
    model: Any
    mass_kg: float
    corridor: Any


def halo_trajectory_case(sizing=None, requirements=HaloRequirements(), assumptions=HaloAssumptions(), corridor=None):
    """The sized Halo aircraft as a trajectory model (numeric design, no sizing variables).

    `corridor`: None flies the assumed XV-15-shaped `ConversionCorridor`; pass a `ComputedCorridor` (plan 039)
    to fly the corridor computed from trim (`examples/halo_trajectory_computed_corridor.py`)."""
    r, a = requirements, assumptions
    sizing = sizing if sizing is not None else solve_halo_sizing(r, a)
    aerodynamics = build_halo_aerodynamics(r, a)   # the trajectory uses the unblown polar (no rotor state)
    model = TiltrotorPointMass(build_halo_aircraft(sizing.design, r, a), aerodynamics)
    mass_kg = sizing.mass_takeoff_kg
    corridor = corridor if corridor is not None else ConversionCorridor(
        velocity_stall_m_s=float(model.velocity_stall_m_s(mass_kg, 0.0)))
    return HaloTrajectoryCase(sizing, model, mass_kg, corridor)


@dataclass(frozen=True)
class TrajectoryResult:
    """Numeric node histories of a solved trajectory."""
    label: str
    duration_s: float
    time_s: Any
    x_m: Any
    altitude_m: Any
    velocity_m_s: Any
    gamma_deg: Any
    alpha_deg: Any
    pitch_deg: Any
    tilt_deg: Any
    tilt_rate_deg_s: Any
    thrust_per_rotor_N: Any
    speed_rotor_rad_s: Any
    speed_tip_m_s: Any
    blade_loading: Any
    advance_ratio: Any
    mach_tip_helical: Any
    download_fraction: Any
    lift_N: Any
    power_shaft_rotors_W: Any
    power_bus_W: Any
    power_battery_W: Any
    power_generators_W: Any
    power_shaft_generator_W: Any
    power_available_turboshaft_W: Any
    power_available_bus_W: Any
    velocity_corridor_min_m_s: Any
    velocity_corridor_max_m_s: Any
    alpha_stall_deg: Any
    soc: Any
    mass_kg: Any
    energy_bus_J: Any
    mass_fuel_burnt_kg: float
    acceleration_m_s2: Any
    rate_gamma_rad_s: Any
    voltage_bus_V: Any = None             # battery terminal voltage (plan 022)
    temperatures_C: Any = None            # plan 036: instance name -> node temperatures (thermal-modelled parts)
    drag_cooling_N: Any = None            # plan 036: ram-air cooling drag and fan power at each node
    power_fan_W: Any = None


def power_available_bus_W(model, altitude_m):
    """Most the bus can draw: every generator at min(lapsed turboshaft, generator rating) plus battery discharge."""
    generator = model.instance("generator")
    speed_rad_s = generator.loss_model.speed_peak_efficiency_rad_s
    power_shaft_W = np.fmin(model.instance("turboshaft").power_available_W(asb.Atmosphere(altitude=altitude_m)),
                            generator.power_rated_W)
    battery = model.instance("battery")
    if isinstance(battery, EquivalentCircuitBattery):
        voltage_V, power_battery_W = battery.voltage_open_circuit_V(soc_take_off), battery.power_max_discharge_W
    else:
        voltage_V, power_battery_W = battery.voltage_open_circuit_V, battery.max_discharge_power_W
    return (model.count("generator") * generator.evaluate(speed_rad_s, power_shaft_W / speed_rad_s,
                                                          voltage_V).power_electric_W + power_battery_W)


def _result(label, solution, model, trajectory):
    v = lambda expression: np.array(solution.value(expression), dtype=float).reshape(-1)  # noqa: E731
    t = trajectory
    count_rotors = model.count("propulsor")
    altitude_m = v(t.altitude_m)
    mass_kg = v(t.mass_kg)
    return TrajectoryResult(
        label=label, duration_s=v(t.duration_s)[0], time_s=v(t.time_s), x_m=v(t.x_m), altitude_m=altitude_m,
        velocity_m_s=v(t.velocity_m_s), gamma_deg=np.degrees(v(t.gamma_rad)), alpha_deg=v(t.alpha_deg),
        pitch_deg=v(t.pitch_deg), tilt_deg=v(t.tilt_deg), tilt_rate_deg_s=v(t.tilt_rate_deg_s),
        thrust_per_rotor_N=v(t.thrust_per_rotor_N), speed_rotor_rad_s=v(t.speed_rotor_rad_s),
        speed_tip_m_s=v(t.speed_rotor_rad_s) * v(model.instance("propulsor").radius_m()),
        blade_loading=v(t.forces.rotor.blade_loading), advance_ratio=v(t.forces.rotor.advance_ratio),
        mach_tip_helical=v(t.forces.rotor.mach_tip_helical), download_fraction=v(t.forces.download_fraction),
        lift_N=v(t.forces.lift_N), power_shaft_rotors_W=count_rotors * v(t.forces.rotor.shaft_power_W),
        power_bus_W=v(t.forces.power_electric_motors_W), power_battery_W=v(t.supply.battery.power_electric_W),
        power_generators_W=model.count("generator") * v(t.supply.generator.power_electric_W),
        power_shaft_generator_W=v(t.supply.power_shaft_generator_W),
        power_available_turboshaft_W=v(t.supply.power_available_turboshaft_W),
        power_available_bus_W=v(power_available_bus_W(model, altitude_m)),
        velocity_corridor_min_m_s=v(t.corridor.velocity_min_m_s(v(t.tilt_deg))),
        velocity_corridor_max_m_s=v(t.corridor.velocity_max_m_s(v(t.tilt_deg))),
        alpha_stall_deg=v(t.forces.alpha_stall_deg), soc=v(t.soc), mass_kg=mass_kg, energy_bus_J=v(t.energy_bus_J),
        mass_fuel_burnt_kg=mass_kg[0] - mass_kg[-1], acceleration_m_s2=v(t.acceleration_m_s2),
        rate_gamma_rad_s=v(t.rate_gamma_rad_s), voltage_bus_V=v(t.supply.battery.voltage_V),
        temperatures_C={name: v(temperature) for name, temperature in t.thermal.temperatures_C.items()},
        drag_cooling_N=v(t.thermal.drag_cooling_N), power_fan_W=v(t.thermal.power_fan_W))


def _trimmed(opti, trajectory, index):
    """Steady flight at one node: no along-path acceleration and no flight-path rotation."""
    opti.subject_to([trajectory.acceleration_m_s2[index] == 0, trajectory.rate_gamma_rad_s[index] == 0])


def _regularization(trajectory, model):
    """Small smoothing terms: battery share (turbines first) and control slew; below 0.1 % of the objectives."""
    t = trajectory
    scale_power_W = model.count("propulsor") * model.instance("motor").power_rated_W
    return (np.mean((t.supply.battery.power_electric_W / scale_power_W) ** 2)
            + np.mean(np.diff(t.alpha_deg / 5) ** 2) + np.mean(np.diff(t.thrust_per_rotor_N / 1e4) ** 2)
            + np.mean(np.diff(t.speed_rotor_rad_s / 10) ** 2))


def transition_guess(case, duration_s=30.0, count_nodes=41):
    velocity_end_m_s = ratio_transition_end_stall * case.corridor.velocity_stall_m_s
    weight_N = case.mass_kg * 9.80665
    fraction = np.linspace(0, 1, count_nodes)
    return TrajectoryGuess(duration_s=duration_s,
                           velocity_m_s=velocity_hover_m_s + (velocity_end_m_s - velocity_hover_m_s) * fraction,
                           altitude_m=altitude_transition_m, tilt_deg=90 * (1 - fraction), alpha_deg=4.0,
                           thrust_per_rotor_N=weight_N / 2 * (1 - 0.85 * fraction), speed_rotor_rad_s=50.0,
                           torque_generator_Nm=1500.0)


def _transition_problem(case, count_nodes, guess, limits, trimmed_ends=True):
    opti = asb.Opti()
    velocity_end_m_s = ratio_transition_end_stall * case.corridor.velocity_stall_m_s
    t = build_tiltrotor_trajectory(opti, case.model, mass_initial_kg=case.mass_kg, soc_initial=soc_take_off,
                                   count_nodes=count_nodes, duration_bounds_s=(5.0, 120.0), guess=guess, limits=limits,
                                   corridor=case.corridor)
    opti.subject_to([
        t.altitude_m >= altitude_transition_m - altitude_band_transition_m,
        t.altitude_m <= altitude_transition_m + altitude_band_transition_m,
        t.altitude_m[0] == altitude_transition_m,
        t.velocity_m_s[0] == velocity_hover_m_s, t.gamma_rad[0] == 0, t.tilt_deg[0] == 90,
        t.velocity_m_s[-1] == velocity_end_m_s, t.gamma_rad[-1] == 0, t.tilt_deg[-1] == 0,
    ])
    if trimmed_ends:
        _trimmed(opti, t, 0)
        _trimmed(opti, t, -1)
    return opti, t


def solve_min_energy_transition(case, count_nodes=41, verbose=False, limits=TrajectoryLimits()):
    """Hover to wing-borne flight at 1.3 V_stall with minimum bus (motor electrical) energy."""
    opti, t = _transition_problem(case, count_nodes, transition_guess(case, count_nodes=count_nodes), limits)
    opti.minimize(t.energy_bus_J[-1] / 1e7 + 1e-3 * _regularization(t, case.model))
    solution = opti.solve(verbose=verbose, max_iter=3000)
    return _result("minimum-energy transition", solution, case.model, t)


def solve_prescribed_transition(case, duration_s=60.0, fraction_hold=0.25, count_nodes=41, verbose=False,
                                limits=TrajectoryLimits()):
    """Naive reference: level flight at constant acceleration over `duration_s`; nacelles held at 90 deg for
    the first `fraction_hold` of the time, then rotated linearly to 0 deg (a plain linear schedule from
    t = 0 leaves the conversion corridor at low speed). Assumed corridor only: inside the computed corridor no
    level, constant-acceleration conversion exists (see `examples/halo_trajectory_computed_corridor.py`).

    Only the trim controls (alpha, thrust, rotor speed, power split) are solved, to fly the prescribed
    profile; they minimize the same energy, so the comparison isolates the profile. The end points are
    not trimmed: a prescribed constant acceleration starts and ends abruptly.
    """
    opti, t = _transition_problem(case, count_nodes, transition_guess(case, duration_s, count_nodes), limits,
                                  trimmed_ends=False)
    fraction = np.linspace(0, 1, count_nodes)
    velocity_end_m_s = ratio_transition_end_stall * case.corridor.velocity_stall_m_s
    tilt_schedule_deg = 90 * np.fmin(1.0, (1 - fraction) / (1 - fraction_hold))
    opti.subject_to([t.duration_s == duration_s, t.tilt_deg == tilt_schedule_deg,
                     t.velocity_m_s == velocity_hover_m_s + (velocity_end_m_s - velocity_hover_m_s) * fraction,
                     t.gamma_rad == 0])
    opti.minimize(t.energy_bus_J[-1] / 1e7 + 1e-3 * _regularization(t, case.model))
    solution = opti.solve(verbose=verbose, max_iter=3000)
    return _result("prescribed transition", solution, case.model, t)


def solve_min_time_climb(case, altitude_final_m=10000 * u.foot, velocity_final_m_s=None, count_nodes=61,
                         verbose=False, limits=TrajectoryLimits()):
    """Hover at sea level to steady level flight at `altitude_final_m` and the sizing cruise speed, in minimum time."""
    velocity_final_m_s = velocity_final_m_s if velocity_final_m_s is not None else case.sizing.velocity_cruise_m_s
    fraction = np.linspace(0, 1, count_nodes)
    ramp = np.fmin(fraction / 0.2, 1.0)
    weight_N = case.mass_kg * 9.80665
    guess = TrajectoryGuess(duration_s=300.0, velocity_m_s=velocity_hover_m_s + (velocity_final_m_s - velocity_hover_m_s)
                            * ramp, altitude_m=altitude_final_m * fraction, tilt_deg=90 * (1 - ramp), alpha_deg=4.0,
                            gamma_deg=5.0 * np.sin(np.pi * fraction), thrust_per_rotor_N=weight_N / 2 * (1 - 0.85 * ramp),
                            speed_rotor_rad_s=50.0, torque_generator_Nm=1500.0)
    opti = asb.Opti()
    t = build_tiltrotor_trajectory(opti, case.model, mass_initial_kg=case.mass_kg, soc_initial=soc_take_off,
                                   count_nodes=count_nodes, duration_bounds_s=(30.0, 1500.0), guess=guess,
                                   limits=limits, corridor=case.corridor)
    opti.subject_to([
        t.altitude_m >= 0,
        t.altitude_m[0] == 0, t.velocity_m_s[0] == velocity_hover_m_s, t.gamma_rad[0] == 0, t.tilt_deg[0] == 90,
        t.altitude_m[-1] == altitude_final_m, t.velocity_m_s[-1] == velocity_final_m_s, t.gamma_rad[-1] == 0,
        t.tilt_deg[-1] == 0,
    ])
    _trimmed(opti, t, 0)
    _trimmed(opti, t, -1)
    opti.minimize(t.duration_s / 100 + 1e-3 * _regularization(t, case.model))
    solution = opti.solve(verbose=verbose, max_iter=3000)
    return _result("minimum time to climb", solution, case.model, t)


def describe(result):
    return (f"{result.label}: {result.duration_s:.1f} s, bus energy {result.energy_bus_J[-1] / 3.6e6:.2f} kWh, "
            f"fuel {result.mass_fuel_burnt_kg:.1f} kg, SOC {result.soc[0]:.3f} -> {result.soc[-1]:.3f}, "
            f"peak bus power {np.max(result.power_bus_W) / 1e3:,.0f} kW")


if __name__ == "__main__":
    case = halo_trajectory_case()
    print(f"Halo {case.mass_kg:,.0f} kg, airplane-mode stall {case.corridor.velocity_stall_m_s:.1f} m/s")
    for solve in (solve_min_energy_transition, solve_prescribed_transition, solve_min_time_climb):
        print(describe(solve(case)))
