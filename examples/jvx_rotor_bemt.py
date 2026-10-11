"""JVX check of the blade-element momentum rotor (plan 047).

Same full-scale JVX data as the momentum-profile calibration (plan 016, `examples/jvx_rotor_calibration.py`):
NASA/TM-2016-219070 Tables D-1/D-2 (hover) and D-7 (airplane mode). Geometry (NASA/TM-2008-214581 and Acree,
AHS 2008, Table 1): radius 150 in, 3 blades, linear chord with tip chord 15.79 in and taper 0.65 referred to the
rotor centre (thrust-weighted solidity 0.1141 against the quoted 0.1138); bilinear twist -64 / -34 deg/R with
transition at 0.45 R, zero at 0.75 R (47.5 deg over the radius; assumed rates); root cutout 0.10 R (assumed).

Each test point is trimmed to its measured C_T at its tip speed, density and speed of sound; the prediction is
figure of merit (hover) or propulsive efficiency (airplane mode). `drag_factor` (the one calibration constant,
multiplying NeuralFoil section drag) is fitted by least squares on hover figure of merit (Table D-1) from two
evaluations, since FM is close to linear in it; D-2 and airplane mode are then predictions.
"""
from dataclasses import dataclass, replace

import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import (AirState, BladeElementRotor, BladeGeometry,
                                                               OperatingPoint, solve_point, solve_point_stepped)
from examples.jvx_rotor_calibration import load_airplane, load_hover, solidity_jvx

ft_m = 0.3048
slug_ft3_kg_m3 = 515.378818
radius_jvx_m = 150 * 0.0254
chord_tip_jvx_m = 15.79 * 0.0254
taper_jvx = 0.65
fraction_root_jvx = 0.10


def jvx_blade():
    """Linear chord (taper 0.65 referred to r = 0) and bilinear twist through (0.10, 0.45, 1.0) R."""
    fractions = (fraction_root_jvx, 0.45, 1.0)
    chord_m = tuple(chord_tip_jvx_m * (1 + (1 / taper_jvx - 1) * (1 - x)) for x in fractions)
    rate_inboard_deg, rate_outboard_deg = -64.0, -34.0
    twist_deg = (rate_outboard_deg * (0.45 - 0.75) + rate_inboard_deg * (fraction_root_jvx - 0.45),
                 rate_outboard_deg * (0.45 - 0.75), rate_outboard_deg * (1.0 - 0.75))
    return BladeGeometry(fraction_control=fractions, chord_control_m=chord_m, twist_control_deg=twist_deg,
                         interpolation="linear")


def jvx_rotor(drag_factor=1.0, **kwargs):
    return BladeElementRotor(radius_m=radius_jvx_m, count_blades=3, fraction_root=fraction_root_jvx,
                             blade=jvx_blade(), drag_factor=drag_factor, **kwargs)


def _point(label, blade_loading, speed_tip_ft_s, mach_tip, density_slug_ft3, velocity_m_s=0.0):
    speed_tip_m_s = speed_tip_ft_s * ft_m
    density_kg_m3 = density_slug_ft3 * slug_ft3_kg_m3
    thrust_N = blade_loading * solidity_jvx * density_kg_m3 * onp.pi * radius_jvx_m**2 * speed_tip_m_s**2
    return OperatingPoint(label, velocity_m_s, 0.0, speed_tip_m_s / radius_jvx_m, thrust_N,
                          air=AirState(density_kg_m3, speed_tip_m_s / mach_tip))


def hover_points(table=None):
    import csv
    from examples.jvx_rotor_calibration import data_dir
    lines = [line for line in open(data_dir / "jvx_hover.csv", encoding="utf-8") if not line.startswith("#")]
    reader = list(csv.DictReader(lines))
    points, measured = [], []
    for row in reader:
        if table is not None and row["table"] != table:
            continue
        points.append(_point(f"{row['table']} {row['run']}-{row['point']}", float(row["ct_sigma"]),
                             float(row["vtip_ft_s"]), float(row["mtip"]), float(row["rho_slug_ft3"])))
        measured.append(float(row["fm"]))
    return points, onp.array(measured)


def airplane_points():
    import csv
    from examples.jvx_rotor_calibration import data_dir
    lines = [line for line in open(data_dir / "jvx_airplane.csv", encoding="utf-8") if not line.startswith("#")]
    points, measured, advance = [], [], []
    for row in csv.DictReader(lines):
        speed_tip_m_s = float(row["vtip_ft_s"]) * ft_m
        points.append(_point(f"{row['run']}-{row['point']}", float(row["ct_sigma"]), float(row["vtip_ft_s"]),
                             float(row["mtip"]), float(row["rho_slug_ft3"]),
                             velocity_m_s=float(row["lambda"]) * speed_tip_m_s))
        measured.append(float(row["eta"]))
        advance.append(float(row["lambda"]))
    return points, onp.array(measured), onp.array(advance)


def _ideal_efficiency(point):
    """Momentum limit of propulsive efficiency for the point's thrust and speed (whole disk)."""
    density_kg_m3 = point.atmosphere().density()
    loading = point.thrust_N / (0.5 * density_kg_m3 * point.axial_velocity_m_s**2 * onp.pi * radius_jvx_m**2)
    return 2 / (1 + onp.sqrt(1 + loading))


def predict(rotor, points, quantity):
    """Trim each point (stepped thrust list, then a direct start if that fails). A solution above the momentum
    limit (FM > 1 or efficiency above ideal) is a spurious branch and is reported as NaN, like a failed solve."""
    values = []
    for point in points:
        value = onp.nan
        for solver in (solve_point_stepped, solve_point):
            try:
                result, _ = solver(rotor, point)
            except RuntimeError:
                continue
            candidate = float(getattr(result, quantity))
            limit = 1.0 if point.axial_velocity_m_s == 0 else _ideal_efficiency(point)
            if 0 < candidate <= limit + 1e-3:
                value = candidate
                break
        values.append(value)
    return onp.array(values)


@dataclass(frozen=True)
class BemtJvxReport:
    drag_factor: float
    hover_blade_loading: object
    hover_fm_measured: object
    hover_fm_predicted: object
    hover_fm_rms: float
    hover_d2_fm_error_max: float
    airplane_advance_ratio: object
    airplane_blade_loading: object
    airplane_eta_measured: object
    airplane_eta_predicted: object
    airplane_eta_rms: float
    airplane_eta_max: float
    count_unsolved: int


def _rms(error):
    error = error[onp.isfinite(error)]
    return float(onp.sqrt(onp.mean(error**2)))


def fit_drag_factor(hover, fm_measured, airplane, eta_measured, factors=(0.9, 1.2)):
    """One drag factor for both modes: FM and efficiency are close to linear in it, so two evaluations and a
    linear least-squares fit over the hover (Table D-1) and airplane-mode points together."""
    def stacked(factor):
        rotor = jvx_rotor(factor)
        return onp.concatenate([predict(rotor, hover, "figure_of_merit"),
                                predict(rotor, airplane, "efficiency_propulsive")])
    measured = onp.concatenate([fm_measured, eta_measured])
    first, second = stacked(factors[0]), stacked(factors[1])
    slope = (second - first) / (factors[1] - factors[0])
    keep = onp.isfinite(slope)
    delta = onp.sum(slope[keep] * (measured[keep] - first[keep])) / onp.sum(slope[keep]**2)
    return factors[0] + float(delta)


def bemt_jvx_report(drag_factor=None, stride=1):
    """Predictions against every `stride`-th point (sorted by thrust within hover tables)."""
    hover, fm_measured = hover_points()
    order = onp.argsort([p.thrust_N for p in hover])[::stride]
    hover, fm_measured = [hover[i] for i in order], fm_measured[order]
    blade_loading_hover = onp.array([float(l) for l in load_hover().blade_loading])[order]
    is_d1 = onp.array([p.label.startswith("D-1") for p in hover])
    airplane, eta_measured, advance = airplane_points()
    airplane, eta_measured, advance = airplane[::stride], eta_measured[::stride], advance[::stride]
    blade_loading_airplane = onp.asarray(load_airplane().blade_loading)[::stride]
    if drag_factor is None:
        drag_factor = fit_drag_factor([p for p, keep in zip(hover, is_d1) if keep], fm_measured[is_d1],
                                      airplane, eta_measured)
    rotor = jvx_rotor(drag_factor)
    fm_predicted = predict(rotor, hover, "figure_of_merit")
    eta_predicted = predict(rotor, airplane, "efficiency_propulsive")
    error_eta = eta_predicted - eta_measured
    error_d2 = (fm_predicted - fm_measured)[~is_d1]
    return BemtJvxReport(
        drag_factor, blade_loading_hover, fm_measured, fm_predicted, _rms((fm_predicted - fm_measured)[is_d1]),
        float(onp.nanmax(onp.abs(error_d2))) if onp.isfinite(error_d2).any() else onp.nan, advance,
        blade_loading_airplane, eta_measured, eta_predicted, _rms(error_eta), float(onp.nanmax(onp.abs(error_eta))),
        int(onp.isnan(fm_predicted).sum() + onp.isnan(eta_predicted).sum()))


if __name__ == "__main__":
    import time
    start = time.time()
    report = bemt_jvx_report()
    print(f"drag factor {report.drag_factor:.3f}  ({time.time() - start:.0f} s)")
    print(f"hover FM RMS (D-1) {report.hover_fm_rms:.4f}, D-2 max |error| {report.hover_d2_fm_error_max:.4f}")
    print(f"airplane eta RMS {report.airplane_eta_rms:.4f}, max |error| {report.airplane_eta_max:.4f}, "
          f"unsolved points {report.count_unsolved}")
    for lam in sorted(set(onp.round(report.airplane_advance_ratio, 2))):
        mask = onp.round(report.airplane_advance_ratio, 2) == lam
        print(f"  lambda {lam:.2f}: mean error {onp.nanmean(report.airplane_eta_predicted[mask] - report.airplane_eta_measured[mask]):+.4f}")
    order = onp.argsort(report.hover_blade_loading)
    for i in order[::max(1, len(order) // 10)]:
        print(f"  CT/sigma {report.hover_blade_loading[i]:.3f}: FM test {report.hover_fm_measured[i]:.3f}, "
              f"BEM {report.hover_fm_predicted[i]:.3f}")
