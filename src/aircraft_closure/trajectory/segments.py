"""Reusable trajectory mission segments on a fixed tiltrotor (plan 068).

Orchestration: each function creates its own `asb.Opti`, builds the collocated trajectory with
`build_tiltrotor_trajectory`, sets the boundary conditions and the objective, solves and returns numbers. The
model (`TiltrotorPointMass`) stays equation-only. Works on the series hybrid and on a battery-only topology
(no generator instance, e.g. `build_all_electric`).

* `solve_transition_climb`: hover at the ground (or a given altitude) -> conversion -> climb to a final altitude
  and airspeed in steady level airplane-mode flight, minimizing the bus (battery-terminal, when battery only)
  energy or the time.
* `solve_steady_level_power`: the bus power of steady level flight at a speed and altitude, the "cruise power" a
  fixed-power rule multiplies.
* `energy_fixed_power_rule_J`: duration x factor x reference power, the rule-of-thumb allowance to compare with.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.performance.flight_point import acceleration_gravity_m_s2
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.trajectory.tiltrotor import (ConversionCorridor, TrajectoryGuess, TrajectoryLimits,
                                                   build_tiltrotor_trajectory)


@dataclass(frozen=True)
class TransitionClimbResult:
    """Numeric node histories and totals of a solved segment (SI)."""
    objective: str
    duration_s: float
    distance_m: float                     # horizontal ground distance
    energy_electric_J: float              # bus energy: the battery terminal energy when battery only
    energy_battery_terminal_J: float      # trapezoidal integral of the battery terminal power
    energy_battery_chemical_J: float      # trapezoidal integral of OCV x current (SOC drop x capacity)
    power_battery_max_W: float
    power_bus_max_W: float
    mass_fuel_burnt_kg: float             # zero when battery only
    time_s: Any
    x_m: Any
    altitude_m: Any
    velocity_m_s: Any
    gamma_deg: Any
    alpha_deg: Any
    pitch_deg: Any
    tilt_deg: Any
    thrust_per_rotor_N: Any
    speed_rotor_rad_s: Any
    power_shaft_rotors_W: Any
    power_bus_W: Any
    power_battery_W: Any
    current_battery_A: Any
    voltage_battery_V: Any
    soc: Any
    energy_bus_J: Any
    mass_kg: Any


def energy_fixed_power_rule_J(power_reference_W, factor=1.5, duration_s=180.0):
    """Rule-of-thumb segment energy: `duration_s` at `factor` x a reference (e.g. cruise) power."""
    return duration_s * factor * power_reference_W


def _trapezoid_J(power_W, time_s):
    return float(np.sum(0.5 * (power_W[1:] + power_W[:-1]) * np.diff(time_s)))


def _voltage_open_circuit_V(battery, soc):
    return battery.voltage_open_circuit_V(soc) if isinstance(battery, EquivalentCircuitBattery) \
        else battery.voltage_open_circuit_V


def _hover_estimates(model, mass_kg, altitude_m):
    """Rotor speed at C_T/sigma 0.10 and momentum hover power (figure of merit 0.75): initial guesses only."""
    rotor = model.instance("propulsor")
    count_rotors = model.count("propulsor")
    density_kg_m3 = float(asb.Atmosphere(altitude=altitude_m).density())
    thrust_N = mass_kg * acceleration_gravity_m_s2 / count_rotors
    area_m2 = float(rotor.area_disk_m2)
    speed_tip_m_s = np.sqrt(thrust_N / (density_kg_m3 * area_m2 * float(rotor.solidity) * 0.10))
    power_W = count_rotors * thrust_N ** 1.5 / np.sqrt(2 * density_kg_m3 * area_m2) / 0.75
    return thrust_N, float(speed_tip_m_s / float(rotor.radius_m())), float(power_W)


def _smoothing(trajectory):
    """Small control-slew terms (well below 0.1 % of the objectives)."""
    t = trajectory
    return (np.mean(np.diff(t.alpha_deg / 5) ** 2) + np.mean(np.diff(t.thrust_per_rotor_N / 1e4) ** 2)
            + np.mean(np.diff(t.speed_rotor_rad_s / 10) ** 2))


def _trimmed(opti, trajectory, index):
    opti.subject_to([trajectory.acceleration_m_s2[index] == 0, trajectory.rate_gamma_rad_s[index] == 0])


def _result(objective, solution, model, trajectory):
    v = lambda expression: np.array(solution.value(expression), dtype=float).reshape(-1)  # noqa: E731
    t = trajectory
    time_s = v(t.time_s)
    power_battery_W = v(t.supply.battery.power_electric_W)
    power_chemical_W = v(t.supply.battery.power_chemical_W)
    power_bus_W = v(t.supply.power_bus_W)
    energy_bus_J = v(t.energy_bus_J)
    x_m, mass_kg = v(t.x_m), v(t.mass_kg)
    return TransitionClimbResult(
        objective=objective, duration_s=float(v(t.duration_s)[0]), distance_m=float(x_m[-1] - x_m[0]),
        energy_electric_J=float(energy_bus_J[-1]), energy_battery_terminal_J=_trapezoid_J(power_battery_W, time_s),
        energy_battery_chemical_J=_trapezoid_J(power_chemical_W, time_s),
        power_battery_max_W=float(np.max(power_battery_W)), power_bus_max_W=float(np.max(power_bus_W)),
        mass_fuel_burnt_kg=float(mass_kg[0] - mass_kg[-1]), time_s=time_s, x_m=x_m, altitude_m=v(t.altitude_m),
        velocity_m_s=v(t.velocity_m_s), gamma_deg=np.degrees(v(t.gamma_rad)), alpha_deg=v(t.alpha_deg),
        pitch_deg=v(t.pitch_deg), tilt_deg=v(t.tilt_deg), thrust_per_rotor_N=v(t.thrust_per_rotor_N),
        speed_rotor_rad_s=v(t.speed_rotor_rad_s),
        power_shaft_rotors_W=model.count("propulsor") * v(t.forces.rotor.shaft_power_W), power_bus_W=power_bus_W,
        power_battery_W=power_battery_W, current_battery_A=v(t.current_battery_A),
        voltage_battery_V=v(t.supply.battery.voltage_V), soc=v(t.soc), energy_bus_J=energy_bus_J, mass_kg=mass_kg)


def default_corridor(model, mass_kg):
    """The assumed XV-15-shaped conversion corridor (plan 019) on the aircraft's own sea-level stall speed."""
    return ConversionCorridor(velocity_stall_m_s=float(model.velocity_stall_m_s(mass_kg, 0.0)))


def solve_transition_climb(model, *, mass_kg, soc_initial, altitude_final_m, velocity_final_m_s,
                           altitude_initial_m=0.0, duration_bounds_s=(20.0, 900.0), objective="energy",
                           corridor=None, limits=TrajectoryLimits(), count_nodes=41, velocity_hover_m_s=1.0,
                           duration_guess_s=None, configure=None, objective_extra=None, verbose=False,
                           max_iter=3000):
    """Take-off -> conversion -> climb segment on a fixed aircraft (orchestration: owns its `asb.Opti`).

    Starts in trimmed hover (nacelles 90 deg, `velocity_hover_m_s`, the speed-gamma states need a nonzero speed)
    at `altitude_initial_m` and ends in trimmed level airplane-mode flight (nacelles 0 deg) at `altitude_final_m`
    and `velocity_final_m_s`; the altitude never goes below the start. The final time is free within
    `duration_bounds_s` (equal bounds fix it).

    `objective`: "energy" minimizes the bus electric energy, which is the battery terminal energy on a battery-only
    aircraft; "time" minimizes the duration. `corridor`: None flies `default_corridor`. `configure(opti,
    trajectory)` may add constraints (e.g. a vertical climb to an obstacle height) before the solve.
    `objective_extra(opti, trajectory)`: an optional dimensionless term added to the objective (e.g. a slew penalty
    that picks the smooth member of a family of near-equal optima); None adds nothing.
    Returns a `TransitionClimbResult`.
    """
    if objective not in ("energy", "time"):
        raise ValueError(f"Unknown objective '{objective}'.")
    corridor = corridor if corridor is not None else default_corridor(model, mass_kg)
    n = count_nodes
    thrust_hover_N, speed_rotor_rad_s, power_hover_W = _hover_estimates(model, mass_kg, altitude_initial_m)
    duration_guess_s = duration_guess_s if duration_guess_s is not None else float(np.clip(
        (altitude_final_m - altitude_initial_m) / 5.0 + 60.0, duration_bounds_s[0], duration_bounds_s[1]))
    fraction = np.linspace(0, 1, n)
    ramp = np.fmin(fraction / 0.25, 1.0)
    voltage_V = float(_voltage_open_circuit_V(model.instance("battery"), soc_initial))
    guess = TrajectoryGuess(
        duration_s=duration_guess_s,
        velocity_m_s=velocity_hover_m_s + (velocity_final_m_s - velocity_hover_m_s) * ramp,
        altitude_m=altitude_initial_m + (altitude_final_m - altitude_initial_m) * fraction,
        tilt_deg=90 * (1 - ramp), alpha_deg=4.0, gamma_deg=5.0 * np.sin(np.pi * fraction),
        thrust_per_rotor_N=thrust_hover_N * (1 - 0.85 * ramp), speed_rotor_rad_s=speed_rotor_rad_s,
        current_battery_A=(power_hover_W * (1 - 0.5 * ramp) / voltage_V if model.is_battery_only() else 0.0))
    opti = asb.Opti()
    t = build_tiltrotor_trajectory(opti, model, mass_initial_kg=mass_kg, soc_initial=soc_initial, count_nodes=n,
                                   duration_bounds_s=duration_bounds_s, guess=guess, limits=limits,
                                   corridor=corridor)
    opti.subject_to([
        t.altitude_m >= altitude_initial_m,
        t.altitude_m[0] == altitude_initial_m, t.velocity_m_s[0] == velocity_hover_m_s, t.gamma_rad[0] == 0,
        t.tilt_deg[0] == 90,
        t.altitude_m[-1] == altitude_final_m, t.velocity_m_s[-1] == velocity_final_m_s, t.gamma_rad[-1] == 0,
        t.tilt_deg[-1] == 0,
    ])
    _trimmed(opti, t, 0)
    _trimmed(opti, t, -1)
    if configure is not None:
        configure(opti, t)
    extra = objective_extra(opti, t) if objective_extra is not None else 0.0
    if objective == "energy":
        scale_J = power_hover_W * duration_guess_s
        opti.minimize(t.energy_bus_J[-1] / scale_J + 1e-4 * _smoothing(t) + extra)
    else:
        opti.minimize(t.duration_s / duration_guess_s + 1e-4 * _smoothing(t) + extra)
    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    return _result(objective, solution, model, t)


def solve_steady_level_power(model, *, mass_kg, soc, altitude_m, velocity_m_s, limits=TrajectoryLimits(),
                             verbose=False):
    """Bus power of steady level airplane-mode flight (nacelles 0 deg) at `velocity_m_s` and `altitude_m`.

    A three-node, 10 s collocated segment held steady at every node (zero acceleration and flight-path rate) with
    the least energy, so the trim, rotor speed and power chain are the trajectory model's own. Returns the
    mean bus power in W (the battery terminal power when battery only)."""
    _, speed_rotor_rad_s, _ = _hover_estimates(model, mass_kg, altitude_m)
    guess = TrajectoryGuess(duration_s=10.0, velocity_m_s=velocity_m_s, altitude_m=altitude_m, tilt_deg=0.0,
                            thrust_per_rotor_N=0.1 * mass_kg * acceleration_gravity_m_s2 / model.count("propulsor"),
                            speed_rotor_rad_s=0.8 * speed_rotor_rad_s)
    opti = asb.Opti()
    t = build_tiltrotor_trajectory(opti, model, mass_initial_kg=mass_kg, soc_initial=soc, count_nodes=3,
                                   duration_bounds_s=(10.0, 10.0), guess=guess, limits=limits)
    opti.subject_to([t.tilt_deg == 0.0, t.acceleration_m_s2 == 0, t.rate_gamma_rad_s == 0,
                     t.velocity_m_s[0] == velocity_m_s, t.altitude_m[0] == altitude_m, t.gamma_rad[0] == 0])
    opti.minimize(t.energy_bus_J[-1] / (1e3 * mass_kg))
    solution = opti.solve(verbose=verbose)
    return float(np.mean(np.array(solution.value(t.supply.power_bus_W), dtype=float)))
