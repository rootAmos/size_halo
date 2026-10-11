"""Two-point proprotor blade design for the Halo rotor with the blade-element momentum rotor (plan 047).

Rotor: radius 4.832 m, 3 blades, root cutout 0.10 R, the JVX-like section schedule (`SectionSchedule`), the
JVX-calibrated section drag factor. Operating points come from the cached 15,179 lb baseline solve
(`halo-closure-system/output/ledger/baseline.pkl`). Rotor speeds are the baseline's; thrusts are backed out of the
baseline rotor powers with the sizing's own `MomentumProfileRotor`, so both rotor models see the same points:

    hover 4,000 ft ISA       49.29 rad/s (tip 238.2 m/s, blade loading 0.14), 1,484.7 kW for two rotors
    cruise 10,000 ft         85.13 m/s, 29.36 rad/s, 714.2 kW for two rotors
    reserve loiter 10,000 ft 68.89 m/s, 23.76 rad/s, 571.1 kW for two rotors
    maximum speed 10,000 ft  210 kt, 39.03 rad/s; thrust from D = a V^2 + b / V^2 through cruise and loiter

Design: chord at r/R = 0.10, 0.40, 0.75, 1.0 and twist at 0.10, 0.40, 1.0 (zero at 0.75 R by definition), cubic
spline, plus one collective and one inflow angle per station per point. Objective: cruise shaft power. Constraints:
hover power no more than a stated multiple of the reference blade's (figure of merit floor), maximum-speed power no
more than the reference blade's, hover C_T/sigma <= 0.14 (thrust-weighted solidity), hover section angle of attack
at least 2 deg below the first peak of the 2-D NeuralFoil lift curve (stall or
shock-buffet onset) outboard of 0.40 R (inboard, rotational stall delay applies), helical tip Mach <= 0.80, chord between
0.10 and 0.80 m and tip chord at least 40 % of root chord (structure, not modelled).

Reference: the XV-15-like blade of the OpenVSP snapshot: constant chord at the sizing's solidity (0.060), linear
twist 40 deg from root to tip.
"""
from dataclasses import dataclass, replace

import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp

from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor
from aircraft_closure.powertrain.components.rotor_bemt import (BladeElementRotor, BladeGeometry, OperatingPoint,
                                                               add_operating_point, collective_guess_deg,
                                                               numeric_result, solve_point,
                                                               solve_point_stepped)

knot_m_s = 0.514444
radius_halo_m = 4.832308625995513
area_disk_halo_m2 = 73.35997928568187
solidity_halo = 0.06
count_blades_halo = 3
fraction_root_halo = 0.10
drag_factor_calibrated = 0.964        # JVX fit, hover and airplane mode together (examples/jvx_rotor_bemt.py, plan 047)
blade_loading_max = 0.14
mach_tip_helical_max = 0.80
margin_stall_deg = 2.0
fraction_stall_check = 0.40       # inboard of this the model's rotational stall delay lifts the 2-D limit
twist_reference_deg = 40.0


@dataclass(frozen=True)
class HaloRotorPoints:
    hover: OperatingPoint
    cruise: OperatingPoint
    max_speed: OperatingPoint
    duration_hover_s: float
    duration_cruise_s: float


def _thrust_from_power(rotor, velocity_m_s, altitude_m, speed_rad_s, power_W):
    """Thrust at which `MomentumProfileRotor` draws `power_W` (bisection on a monotone numeric function)."""
    atmosphere = asb.Atmosphere(altitude=altitude_m)
    model = rotor.in_airplane_mode() if velocity_m_s > 0 else rotor
    low, high = 10.0, 1.0e5
    for _ in range(80):
        middle = (low + high) / 2
        power = float(model.evaluate(velocity_m_s, atmosphere, thrust_N=middle, speed_rad_s=speed_rad_s).shaft_power_W)
        low, high = (middle, high) if power < power_W else (low, middle)
    return (low + high) / 2


def halo_points():
    rotor = MomentumProfileRotor(area_disk_m2=area_disk_halo_m2, solidity=solidity_halo)
    altitude_cruise_m, altitude_hover_m = 3048.0, 1219.2
    speed_hover_rad_s = 238.20587783149807 / radius_halo_m
    thrust_hover_N = _thrust_from_power(rotor, 0.0, altitude_hover_m, speed_hover_rad_s, 1484655.1017 / 2)
    velocity_cruise_m_s, speed_cruise_rad_s = 85.13186022578407, 29.362038385073514
    thrust_cruise_N = _thrust_from_power(rotor, velocity_cruise_m_s, altitude_cruise_m, speed_cruise_rad_s, 714187.8943 / 2)
    velocity_loiter_m_s = 68.89185957524931
    thrust_loiter_N = _thrust_from_power(rotor, velocity_loiter_m_s, altitude_cruise_m, 23.76085076737679, 571140.7313 / 2)
    # D = a V^2 + b / V^2 through the cruise and loiter thrusts (per rotor), extrapolated to 210 kt.
    matrix = onp.array([[velocity_cruise_m_s**2, velocity_cruise_m_s**-2], [velocity_loiter_m_s**2, velocity_loiter_m_s**-2]])
    a, b = onp.linalg.solve(matrix, [thrust_cruise_N, thrust_loiter_N])
    velocity_max_m_s = 210 * knot_m_s
    thrust_max_N = a * velocity_max_m_s**2 + b / velocity_max_m_s**2
    return HaloRotorPoints(
        hover=OperatingPoint("hover 4,000 ft", 0.0, altitude_hover_m, speed_hover_rad_s, thrust_hover_N),
        cruise=OperatingPoint("cruise 165 kt", velocity_cruise_m_s, altitude_cruise_m, speed_cruise_rad_s, thrust_cruise_N),
        max_speed=OperatingPoint("210 kt", velocity_max_m_s, altitude_cruise_m, 39.03247173774434, thrust_max_N),
        duration_hover_s=120.0, duration_cruise_s=8632.98)


def current_model_numbers(points):
    """The sizing's `MomentumProfileRotor` at the same points: hover FM and airplane-mode efficiency."""
    rotor = MomentumProfileRotor(area_disk_m2=area_disk_halo_m2, solidity=solidity_halo)
    numbers = {}
    for name in ("hover", "cruise", "max_speed"):
        point = getattr(points, name)
        model = rotor if point.axial_velocity_m_s == 0 else rotor.in_airplane_mode()
        result = model.evaluate(point.axial_velocity_m_s, point.atmosphere(), thrust_N=point.thrust_N,
                                speed_rad_s=point.speed_rad_s)
        power_W = float(result.shaft_power_W)
        density_kg_m3 = float(point.atmosphere().density())
        if point.axial_velocity_m_s == 0:
            numbers[name] = dict(power_W=power_W, figure_of_merit=point.thrust_N**1.5
                                 / onp.sqrt(2 * density_kg_m3 * area_disk_halo_m2) / power_W)
        else:
            numbers[name] = dict(power_W=power_W, efficiency=point.thrust_N * point.axial_velocity_m_s / power_W)
    return numbers


def reference_blade():
    chord_m = solidity_halo * onp.pi * radius_halo_m / count_blades_halo
    fractions = (fraction_root_halo, 0.40, 0.75, 1.0)
    rate_deg = -twist_reference_deg / (1 - fraction_root_halo)
    return BladeGeometry(fraction_control=fractions, chord_control_m=(chord_m,) * 4,
                         twist_control_deg=tuple(rate_deg * (x - 0.75) for x in fractions))


def halo_rotor(blade=None, drag_factor=drag_factor_calibrated):
    return BladeElementRotor(radius_m=radius_halo_m, count_blades=count_blades_halo, fraction_root=fraction_root_halo,
                             blade=blade or reference_blade(), drag_factor=drag_factor)


def stall_angles_deg(rotor, point, chord_m=0.40):
    """First peak of the 2-D NeuralFoil lift curve per station at the point's Mach and Reynolds number (numeric):
    stall at low Mach, shock-buffet onset (AeroSandbox's buffet factor beyond the drag-divergence Mach) at high
    Mach, whichever comes first."""
    fractions, _ = rotor.stations()
    atmosphere = point.atmosphere()
    alphas = onp.arange(0.0, 25.0, 0.25)
    result = []
    for fraction, airfoil in zip(fractions, rotor.sections.airfoils(fractions)):
        speed_m_s = point.speed_rad_s * float(rotor.radius_m) * fraction
        mach = speed_m_s / float(atmosphere.speed_of_sound())
        reynolds = float(atmosphere.density()) * speed_m_s * chord_m / float(atmosphere.dynamic_viscosity())
        transition = rotor.sections.fraction_chord_transition
        cl = onp.ravel(airfoil.get_aero_from_neuralfoil(alpha=alphas, Re=reynolds, mach=mach, xtr_upper=transition,
                                                        xtr_lower=transition, model_size=rotor.sections.model_size)["CL"])
        falling = onp.flatnonzero(onp.diff(cl) < 0)
        result.append(float(alphas[falling[0]]) if len(falling) else float(alphas[-1]))
    return onp.array(result)


@dataclass(frozen=True)
class BladeEvaluation:
    blade: BladeGeometry
    hover: object
    cruise: object
    max_speed: object
    collective_deg: tuple
    solidity_thrust_weighted: float


def evaluate_blade(blade, points, drag_factor=drag_factor_calibrated):
    """Trim the blade at the three points; hover through the stepped thrust list (`solve_point_stepped`)."""
    rotor = halo_rotor(blade, drag_factor)
    results, collectives = [], []
    for point in (points.hover, points.cruise, points.max_speed):
        try:
            result, collective = (solve_point_stepped(rotor, point) if point.axial_velocity_m_s == 0
                                  else solve_point(rotor, point))
        except RuntimeError:
            result, collective = None, onp.nan
        results.append(result)
        collectives.append(collective)
    return BladeEvaluation(blade, *results, tuple(collectives), float(rotor.solidity_thrust_weighted()))


def start_blade():
    """Starting guess between ideal hover twist (inflow 0.065) and ideal cruise twist (advance ratio 0.6), constant
    chord at the sizing's solidity."""
    chord_m = solidity_halo * onp.pi * radius_halo_m / count_blades_halo
    chord_m = 0.40       # sigma_tw 0.079: the sizing's 0.060 stalls inboard in hover at C_T/sigma 0.14 (plan 047)
    return BladeGeometry(fraction_control=(fraction_root_halo, 0.40, 0.75, 1.0), chord_control_m=(chord_m, chord_m),
                         twist_control_deg=(35.0, 10.0, 0.0, -5.0), fraction_control_chord=(fraction_root_halo, 1.0),
                         interpolation_chord="linear")


def design_blade(points, start, current, *, figure_of_merit_min=None, power_max_speed_max_W=None,
                 drag_factor=drag_factor_calibrated, verbose=False, max_iter=600):
    """Minimum cruise power subject to a hover figure-of-merit floor (default: the sizing model's FM at the same
    point, i.e. no more hover power than the sizing assumes), maximum-speed power no more than the sizing model's,
    and the limits in the module docstring. Chord: linear taper (root and tip chord); twist: cubic spline."""
    if figure_of_merit_min is None:
        figure_of_merit_min = current["hover"]["figure_of_merit"]
    opti = asb.Opti()
    chord_root_m = opti.variable(init_guess=start.blade.chord_control_m[0], lower_bound=0.10, upper_bound=0.80)
    chord_tip_m = opti.variable(init_guess=start.blade.chord_control_m[-1], lower_bound=0.10, upper_bound=0.80)
    twist_start = start.blade.twist_control_deg
    twist_deg = [opti.variable(init_guess=twist_start[0], lower_bound=-10.0, upper_bound=70.0),
                 opti.variable(init_guess=twist_start[1], lower_bound=-20.0, upper_bound=50.0),
                 0.0,
                 opti.variable(init_guess=twist_start[3], lower_bound=-40.0, upper_bound=15.0)]
    blade = replace(start.blade, chord_control_m=[chord_root_m, chord_tip_m], twist_control_deg=twist_deg)
    rotor = halo_rotor(blade, drag_factor)
    outputs = {}
    for name, collective_start in zip(("hover", "cruise", "max_speed"), start.collective_deg):
        result, collective = add_operating_point(opti, rotor, getattr(points, name),
                                                 collective_guess_deg=collective_start,
                                                 angle_inflow_guess_rad=getattr(start, name).angle_inflow_rad)
        outputs[name] = (result, collective)
    hover, cruise, max_speed = (outputs[k][0] for k in ("hover", "cruise", "max_speed"))
    alpha_stall = stall_angles_deg(halo_rotor(start.blade, drag_factor), points.hover)
    fractions, _ = rotor.stations()
    check = onp.flatnonzero(fractions >= fraction_stall_check)
    if power_max_speed_max_W is not None:
        opti.subject_to(max_speed.shaft_power_W / power_max_speed_max_W <= 1.0)
    opti.subject_to([
        hover.figure_of_merit >= figure_of_merit_min,
        hover.blade_loading <= blade_loading_max,
        max_speed.mach_tip_helical <= mach_tip_helical_max,
        hover.angle_attack_deg[check] <= alpha_stall[check] - margin_stall_deg,
        chord_tip_m >= 0.5 * chord_root_m,
        chord_tip_m <= chord_root_m,
    ])
    opti.minimize(cruise.shaft_power_W / current["cruise"]["power_W"])
    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    numeric_blade = replace(start.blade, chord_control_m=(float(solution(chord_root_m)), float(solution(chord_tip_m))),
                            twist_control_deg=tuple(t if isinstance(t, float) else float(solution(t)) for t in twist_deg))
    return BladeEvaluation(numeric_blade, numeric_result(solution, hover), numeric_result(solution, cruise),
                           numeric_result(solution, max_speed),
                           tuple(float(solution(outputs[k][1])) for k in ("hover", "cruise", "max_speed")),
                           float(solution(rotor.solidity_thrust_weighted())))


def summary_line(label, evaluation):
    parts = [f"{label:<30} sigma_tw {evaluation.solidity_thrust_weighted:.4f}"]
    if evaluation.hover is not None:
        parts.append(f"hover FM {float(evaluation.hover.figure_of_merit):.3f} "
                     f"({float(evaluation.hover.shaft_power_W) / 1e3:.0f} kW, CT/s {float(evaluation.hover.blade_loading):.3f}, "
                     f"max alpha(>0.25R) {float(onp.max(evaluation.hover.angle_attack_deg[evaluation.hover.fraction_radius >= 0.25])):.1f})")
    for name, label_point in (("cruise", "cruise"), ("max_speed", "210 kt")):
        result = getattr(evaluation, name)
        if result is not None:
            parts.append(f"{label_point} eta {float(result.efficiency_propulsive):.3f} ({float(result.shaft_power_W) / 1e3:.0f} kW)")
    return "  ".join(parts)


@dataclass(frozen=True)
class BladeDesignStudy:
    points: HaloRotorPoints
    current: dict
    reference: BladeEvaluation
    start: BladeEvaluation
    optimum: BladeEvaluation                 # minimum cruise power, hover FM floor = the sizing model's
    front: tuple                             # (FM floor, BladeEvaluation or None) along the hover-cruise trade


def run_study(figure_of_merit_floors=(0.70, 0.76, 0.78, 0.80, 0.82), verbose=False):
    """Reference and start blades, the design at the sizing model's hover FM, and the trade along FM floors.
    Each design starts from the previous converged one (a fixed, ordered list of starts; plan 029)."""
    points = halo_points()
    current = current_model_numbers(points)
    reference = evaluate_blade(reference_blade(), points)
    start = evaluate_blade(start_blade(), points)
    front, guess = [], start
    for floor in figure_of_merit_floors:
        try:
            design = design_blade(points, guess, current, figure_of_merit_min=floor, verbose=verbose)
            guess = design
        except RuntimeError:
            design = None
        front.append((floor, design))
        if verbose:
            print(floor, summary_line("design", design) if design else "not feasible / not converged")
    try:
        optimum = design_blade(points, start, current, verbose=verbose)
    except RuntimeError:
        optimum = None
    return BladeDesignStudy(points, current, reference, start, optimum, tuple(front))


if __name__ == "__main__":
    import pickle
    from pathlib import Path
    study = run_study()
    print("current model:", {k: {q: round(float(v), 4) for q, v in d.items()} for k, d in study.current.items()})
    print(summary_line("reference (linear 40 deg)", study.reference))
    print(summary_line("start", study.start))
    for floor, design in study.front:
        print(summary_line(f"FM >= {floor:.2f}", design) if design else f"FM >= {floor:.2f}: no solution")
    if study.optimum is not None:
        print(summary_line("optimum (sizing hover FM)", study.optimum))
        print("chord", study.optimum.blade.chord_control_m, "twist", study.optimum.blade.twist_control_deg)
    output = Path(__file__).resolve().parents[1] / "output"
    output.mkdir(exist_ok=True)
    pickle.dump(study, open(output / "blade_design_study.pkl", "wb"))
