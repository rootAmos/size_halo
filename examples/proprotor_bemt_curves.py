"""BEM performance curves against full-scale JVX and XV-15 proprotor data, hover and airplane mode (plan 052).

The blade-element momentum rotor (plan 047, `rotor_bemt`) is swept along each test condition's thrust range, with
continuation from one trimmed point to the next, and drawn over the measurements:

- JVX (NASA/TM-2016-219070, `data/rotors/jvx_*.csv`): hover FM against C_T/sigma at Mtip 0.68 and 0.73; airplane-
  mode efficiency against C_T/sigma at five advance ratios. The JVX blade is plan 047's (`examples/jvx_rotor_bemt.py`).
- XV-15 metal blades, hover (NASA/TM-86833, Felker et al. 1985, `data/rotors/xv15_hover.csv`): FM against C_T/sigma
  at Mtip 0.60, 0.69 and 0.73.
- XV-15 (Bell Model 300) rotor, airplane mode (NASA CR-114363, 1971, `data/rotors/xv15_airplane.csv`): efficiency
  against the propeller power coefficient at 120, 160 and 185 kt and 400-740 ft/s tip speed, as the report plots it.

The drag factor stays at the JVX fit (0.964): the XV-15 is a prediction, not a fit.

XV-15 blade (TM-86833 Table I and figures 5-8; CR-114363 Table III-1): radius 150 in, 3 blades, solidity 0.0891;
chord 14 in, with the root cuff tapering from 16.6 in at 0.091 R to 14 in at 0.25 R; twist digitized from figure 5
(about 36 deg from 0.10 R to the tip, nonlinear); t/c from figure 6 (0.32 at the root to 0.08 at the tip). Sections are
the blade's own NACA 64-935 to 64-208 sections (a = 0.3 mean line at each design lift coefficient, `airfoils="64-series"`,
the default) or plan 047's single 64A210 camber line (`airfoils="plan-047"`). Root cutout 0.09 R.

Model error is read off the curve: each measured point is compared with the curve interpolated at its own C_T/sigma
(hover, JVX airplane mode) or C_P (XV-15 airplane mode).
"""
import csv
import pickle
from dataclasses import dataclass
from pathlib import Path

import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import (AirState, BladeElementRotor, BladeGeometry,
                                                               OperatingPoint, SectionSchedule, solve_point,
                                                               solve_point_stepped)
from examples.jvx_rotor_bemt import jvx_rotor, radius_jvx_m, slug_ft3_kg_m3
from examples.jvx_rotor_calibration import data_dir, solidity_jvx

root = Path(__file__).resolve().parents[1]
ft_m = 0.3048
kt_m_s = 0.514444
radius_xv15_m = 150 * 0.0254
solidity_xv15 = 0.0891
density_sea_level_kg_m3 = 1.225

# TM-86833 figure 5, digitized at 0.05 R (deg, absolute; 1.0 R extrapolated from 0.85-0.95 R).
twist_xv15_fraction = (0.09, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75,
                       0.80, 0.85, 0.90, 0.95, 1.00)
twist_xv15_deg = (30.3, 29.7, 26.5, 23.4, 20.3, 17.3, 14.4, 11.8, 9.4, 7.4, 5.7, 4.2, 3.0, 1.7, 0.4,
                  -1.0, -2.4, -3.8, -5.1, -6.3)


def xv15_blade():
    """Linear interpolation through the digitized twist; chord 16.6 in at 0.091 R tapering to 14 in at 0.25 R."""
    chord_root_m, chord_m = 16.6 * 0.0254, 14.0 * 0.0254
    chord_at_cutout_m = chord_root_m + (chord_root_m - chord_m) * (0.091 - 0.09) / (0.25 - 0.091)
    return BladeGeometry(fraction_control=twist_xv15_fraction, chord_control_m=(chord_at_cutout_m, chord_m, chord_m),
                         twist_control_deg=twist_xv15_deg, interpolation="linear",
                         fraction_control_chord=(0.09, 0.25, 1.0), interpolation_chord="linear")


# XV-15 airfoils (TM-86833 fig. 8; CR-114363 Table III-1, a = 0.3 mean line): NACA 64-935, 64-528, 64-118,
# 64-(1.5)12 and 64-208 at 0.09, 0.17, 0.51, 0.80 and 1.0 R; design lift coefficient = first digit after the dash / 10.
airfoil_xv15_fraction = (0.09, 0.17, 0.51, 0.80, 1.00)
lift_design_xv15 = (0.9, 0.5, 0.1, 0.15, 0.2)


def xv15_rotor(drag_factor=None, airfoils="64-series", **kwargs):
    """`airfoils` "64-series": the blade's own NACA 64-series sections (a = 0.3 mean line at each station's design
    lift coefficient, t/c from fig. 6); "plan-047": the single NACA 64A210 camber line of plan 047."""
    fractions = (0.09, 0.17, 0.30, 0.51, 0.80, 1.00)
    thickness = (0.32, 0.28, 0.235, 0.185, 0.125, 0.082)
    if airfoils == "64-series":
        sections = SectionSchedule(fraction_radius=fractions, thickness_to_chord=thickness,
                                   lift_design=tuple(onp.interp(fractions, airfoil_xv15_fraction, lift_design_xv15)),
                                   mean_line_a=0.3)
    else:
        sections = SectionSchedule(fraction_radius=fractions, thickness_to_chord=thickness)
    extra = {} if drag_factor is None else {"drag_factor": drag_factor}
    return BladeElementRotor(radius_m=radius_xv15_m, count_blades=3, fraction_root=0.09, blade=xv15_blade(),
                             sections=sections, **extra, **kwargs)


def _rows(name):
    lines = [line for line in open(data_dir / name, encoding="utf-8") if not line.startswith("#")]
    return list(csv.DictReader(lines))


@dataclass(frozen=True)
class Condition:
    """One test condition: a curve is swept at fixed tip speed, inflow and air."""
    label: str
    speed_tip_m_s: float
    axial_velocity_m_s: float
    density_kg_m3: float
    speed_sound_m_s: float
    measured_x: object          # C_T/sigma (hover, JVX airplane mode) or C_P propeller (XV-15 airplane mode)
    measured_y: object          # FM or efficiency
    thrust_grid_N: object


def _thrust_from_blade_loading(blade_loading, solidity, density_kg_m3, radius_m, speed_tip_m_s):
    return blade_loading * solidity * density_kg_m3 * onp.pi * radius_m**2 * speed_tip_m_s**2


def jvx_conditions():
    hover = []
    for table, mtip_label in (("D-1", "0.68"), ("D-2", "0.73")):
        rows = [r for r in _rows("jvx_hover.csv") if r["table"] == table]
        speed_tip_m_s = float(onp.median([float(r["vtip_ft_s"]) for r in rows])) * ft_m
        mach = float(onp.median([float(r["mtip"]) for r in rows]))
        density = float(onp.median([float(r["rho_slug_ft3"]) for r in rows])) * slug_ft3_kg_m3
        x = onp.array([float(r["ct_sigma"]) for r in rows])
        grid = _thrust_from_blade_loading(onp.linspace(0.02, x.max() + 0.005, 11), solidity_jvx, density,
                                          radius_jvx_m, speed_tip_m_s)
        hover.append(Condition(f"JVX hover, Mtip {mtip_label} ({table})", speed_tip_m_s, 0.0, density,
                               speed_tip_m_s / mach, x, onp.array([float(r["fm"]) for r in rows]), grid))
    airplane = []
    rows = _rows("jvx_airplane.csv")
    advance = onp.array([float(r["lambda"]) for r in rows])
    centres = onp.array([0.26, 0.35, 0.44, 0.525, 0.56])
    nearest = onp.argmin(onp.abs(advance[:, None] - centres[None, :]), axis=1)
    for k in range(len(centres)):
        group = [r for r, j in zip(rows, nearest) if j == k]
        lam = float(onp.median([float(r["lambda"]) for r in group]))
        speed_tip_m_s = float(onp.median([float(r["vtip_ft_s"]) for r in group])) * ft_m
        mach = float(onp.median([float(r["mtip"]) for r in group]))
        density = float(onp.median([float(r["rho_slug_ft3"]) for r in group])) * slug_ft3_kg_m3
        x = onp.array([float(r["ct_sigma"]) for r in group])
        grid = _thrust_from_blade_loading(onp.linspace(0.6 * x.min(), 1.15 * x.max(), 9), solidity_jvx, density,
                                          radius_jvx_m, speed_tip_m_s)
        airplane.append(Condition(f"$\\lambda$ {lam:.2f}, Mtip {mach:.2f}", speed_tip_m_s, lam * speed_tip_m_s,
                                  density, speed_tip_m_s / mach, x,
                                  onp.array([float(r["eta"]) for r in group]), grid))
    return hover, airplane


def xv15_conditions():
    rows = [r for r in _rows("xv15_hover.csv") if float(r["ct_sigma"]) > 0.01]
    hover = []
    for mach_nominal in (0.60, 0.69, 0.73):
        group = [r for r in rows if abs(float(r["mtip"]) - mach_nominal) < 0.006]
        speed_tip_m_s = float(onp.median([float(r["vtip_m_s"]) for r in group]))
        mach = float(onp.median([float(r["mtip"]) for r in group]))
        density = float(onp.median([float(r["rho_kg_m3"]) for r in group]))
        x = onp.array([float(r["ct_sigma"]) for r in group])
        grid = _thrust_from_blade_loading(onp.linspace(0.02, x.max() + 0.005, 11), solidity_xv15, density,
                                          radius_xv15_m, speed_tip_m_s)
        hover.append(Condition(f"XV-15 hover, Mtip {mach_nominal:.2f}", speed_tip_m_s, 0.0, density,
                               speed_tip_m_s / mach, x, onp.array([float(r["fm"]) for r in group]), grid))
    airplane = []
    rows = _rows("xv15_airplane.csv")
    keys = sorted({(int(r["v_kt"]), int(r["vtip_ft_s"])) for r in rows}, key=lambda k: (k[0], -k[1]))
    for v_kt, vtip_ft_s in keys:
        group = [r for r in rows if int(r["v_kt"]) == v_kt and int(r["vtip_ft_s"]) == vtip_ft_s]
        speed_tip_m_s, velocity_m_s = vtip_ft_s * ft_m, v_kt * kt_m_s
        mach = float(group[0]["mtip_helical"])
        cp = onp.array([float(r["cp_prop"]) for r in group])
        eta = onp.array([float(r["eta"]) for r in group])
        advance_j = float(group[0]["j"])
        ct_prop = eta * cp / advance_j
        n_rev_s = speed_tip_m_s / (2 * onp.pi * radius_xv15_m)
        thrust_scale_N = density_sea_level_kg_m3 * n_rev_s**2 * (2 * radius_xv15_m)**4
        grid = onp.geomspace(0.6 * ct_prop.min(), 1.25 * ct_prop.max(), 10) * thrust_scale_N
        airplane.append(Condition(f"{v_kt} kt, {vtip_ft_s} ft/s (J {advance_j:.2f})", speed_tip_m_s, velocity_m_s,
                                  density_sea_level_kg_m3, onp.hypot(speed_tip_m_s, velocity_m_s) / mach, cp, eta,
                                  grid))
    return hover, airplane


def _ideal_efficiency(thrust_N, velocity_m_s, density_kg_m3, radius_m):
    loading = thrust_N / (0.5 * density_kg_m3 * velocity_m_s**2 * onp.pi * radius_m**2)
    return 2 / (1 + onp.sqrt(1 + loading))


def sweep(rotor, condition, start_fraction=0.6):
    """Trim each thrust in `condition.thrust_grid_N`, starting near `start_fraction` of the grid with the stepped
    solve and continuing up and then down from the neighbouring solution (a fixed list of starting points, not an
    iteration). Spurious branches above the momentum limit, and failed solves, are NaN."""
    radius_m = float(rotor.radius_m)
    air = AirState(condition.density_kg_m3, condition.speed_sound_m_s)
    grid = onp.asarray(condition.thrust_grid_N, dtype=float)
    results = [None] * len(grid)

    def valid(result, thrust_N):
        if condition.axial_velocity_m_s == 0:
            return 0 < result.figure_of_merit <= 1.0
        limit = _ideal_efficiency(thrust_N, condition.axial_velocity_m_s, condition.density_kg_m3, radius_m)
        return 0 < result.efficiency_propulsive <= limit + 1e-3

    def solve(index, guess):
        point = OperatingPoint(condition.label, condition.axial_velocity_m_s, 0.0, condition.speed_tip_m_s / radius_m,
                               float(grid[index]), air=air)
        attempts = ([lambda: solve_point(rotor, point, guess=guess)] if guess is not None else []) + \
                   [lambda: solve_point_stepped(rotor, point), lambda: solve_point(rotor, point)]
        # Near hover stall there are two solutions, attached and stalled, and continuation can drift onto the
        # stalled one. In hover every start is tried and the lowest-power (highest-FM) solution kept; in airplane
        # mode the first valid one is kept.
        best = None
        for attempt in attempts:
            try:
                result, collective = attempt()
            except RuntimeError:
                continue
            if not valid(result, grid[index]):
                continue
            if condition.axial_velocity_m_s != 0:
                return result, collective
            if best is None or result.figure_of_merit > best[0].figure_of_merit:
                best = (result, collective)
        return best

    start = int(round(start_fraction * (len(grid) - 1)))
    for order in (range(start, len(grid)), range(start - 1, -1, -1)):
        guess = results[start] if order.start != start else None
        for index in order:
            results[index] = solve(index, guess)
            guess = results[index] or guess
    return results


@dataclass(frozen=True)
class Curve:
    condition: Condition
    x: object                   # C_T/sigma or C_P (propeller), as the measured_x
    y: object                   # FM or efficiency
    error: object               # curve minus measurement at each measured x (NaN outside the curve)


def _curve(rotor, condition, solidity, kind):
    results = sweep(rotor, condition)
    radius_m = float(rotor.radius_m)
    area_m2 = onp.pi * radius_m**2
    x, y = [], []
    for entry in results:
        if entry is None:
            x.append(onp.nan), y.append(onp.nan)
            continue
        result = entry[0]
        if kind == "power_propeller":
            n_rev_s = condition.speed_tip_m_s / (2 * onp.pi * radius_m)
            x.append(float(result.shaft_power_W) / (condition.density_kg_m3 * n_rev_s**3 * (2 * radius_m)**5))
        else:
            x.append(float(result.thrust_N) / (condition.density_kg_m3 * area_m2 * condition.speed_tip_m_s**2) / solidity)
        y.append(float(result.figure_of_merit if condition.axial_velocity_m_s == 0 else result.efficiency_propulsive))
    x, y = onp.array(x), onp.array(y)
    keep = onp.isfinite(x)
    order = onp.argsort(x[keep])
    xs, ys = x[keep][order], y[keep][order]
    inside = (condition.measured_x >= xs.min()) & (condition.measured_x <= xs.max()) if len(xs) > 1 else \
        onp.zeros(len(condition.measured_x), bool)
    error = onp.where(inside, onp.interp(condition.measured_x, xs, ys) - condition.measured_y, onp.nan)
    return Curve(condition, x, y, error)


groups = ("jvx_hover", "jvx_airplane", "xv15_hover", "xv15_airplane", "xv15_hover_plan047", "xv15_airplane_plan047")
cache_dir = root / "output"


def study_group(key):
    """Sweep one group of curves and cache it (the four groups are independent and can run in parallel)."""
    if key.startswith("jvx"):
        rotor = jvx_rotor(BladeElementRotor().drag_factor)
    else:
        rotor = xv15_rotor(airfoils="plan-047" if key.endswith("plan047") else "64-series")
    solidity = solidity_jvx if key.startswith("jvx") else solidity_xv15
    conditions = dict(zip(groups, jvx_conditions() + xv15_conditions() + xv15_conditions()))[key]
    kind = "power_propeller" if key.startswith("xv15_airplane") else "blade_loading"
    curves = [_curve(rotor, condition, solidity, kind) for condition in conditions]
    cache_dir.mkdir(exist_ok=True)
    with open(cache_dir / f"proprotor_bemt_curves_{key}.pkl", "wb") as handle:
        pickle.dump(curves, handle)
    return curves


def load_curves():
    curves = {key: pickle.load(open(cache_dir / f"proprotor_bemt_curves_{key}.pkl", "rb")) for key in groups}
    curves["xv15_solidity_thrust_weighted"] = float(xv15_rotor().solidity_thrust_weighted())
    return curves


def rms(curves, mask=None):
    error = onp.concatenate([c.error if mask is None else c.error[mask(c)] for c in curves])
    error = error[onp.isfinite(error)]
    return float(onp.sqrt(onp.mean(error**2))), float(onp.mean(error)), len(error)


blade_loading_attached = 0.13      # above about this the BEM hover curves drop (outboard shock-buffet onset, plan 052)


def attached(curve):
    return curve.condition.measured_x <= blade_loading_attached


palette = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4")
markers = ("o", "s", "^", "D", "v")
ink, ink_muted = "#0b0b0b", "#52514e"


def make_figure(curves, path=root / "docs" / "figures" / "proprotor_bemt_curves.png"):
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 12, "axes.edgecolor": ink_muted, "axes.labelcolor": ink, "xtick.color": ink_muted,
                         "ytick.color": ink_muted, "axes.spines.top": False, "axes.spines.right": False})
    figure = plt.figure(figsize=(18, 11), facecolor="white")
    grid = figure.add_gridspec(2, 6, hspace=0.38, wspace=0.55)
    panels = [(figure.add_subplot(grid[0, 0:3]), curves["jvx_hover"], "JVX hover (TM-2016-219070)",
               "$C_T/\\sigma$", "Figure of merit"),
              (figure.add_subplot(grid[0, 3:6]), curves["jvx_airplane"], "JVX airplane mode",
               "$C_T/\\sigma$", "Propulsive efficiency"),
              (figure.add_subplot(grid[1, 0:3]), curves["xv15_hover"], "XV-15 hover (TM-86833), 64-series airfoils",
               "$C_T/\\sigma$ ($\\sigma$ 0.0891)", "Figure of merit")]
    for ax, group, title, xlabel, ylabel in panels:
        if group is curves["xv15_hover"] and "xv15_hover_plan047" in curves:
            for k, curve in enumerate(curves["xv15_hover_plan047"]):
                ax.plot(curve.x, curve.y, "--", color=palette[k], lw=1, alpha=0.7)
            ax.plot([], [], "--", color=ink_muted, lw=1, label="plan 047 single camber line")
        for k, curve in enumerate(group):
            c = curve.condition
            ax.plot(c.measured_x, c.measured_y, markers[k], color=palette[k], ms=7, mfc="none", mew=1.4)
            ax.plot(curve.x, curve.y, "-", color=palette[k], lw=2,
                    label=c.label.split(", ", 1)[-1] if c.axial_velocity_m_s == 0 else c.label)
        r, bias, n = rms(group)
        title = f"{title}: RMS {r:.3f}, bias {bias:+.3f} ({n} points)"
        if group[0].condition.axial_velocity_m_s == 0:
            r_attached, _, n_attached = rms(group, attached)
            title += (f"\nRMS {r_attached:.3f} for $C_T/\\sigma \\leq$ {blade_loading_attached:.2f} ({n_attached} pts); "
                      "drop above: outboard shock-buffet in the section model")
        ax.set(xlabel=xlabel, ylabel=ylabel, title=title)
        ax.legend(frameon=False, fontsize=10, loc="lower right", title="Line BEM, markers test", title_fontsize=10)
    airplane = curves["xv15_airplane"]
    speeds = sorted({int(c.condition.label.split(" kt")[0]) for c in airplane})
    tips = sorted({int(c.condition.label.split(", ")[1].split(" ")[0]) for c in airplane}, reverse=True)
    for j, v_kt in enumerate(speeds):
        ax = figure.add_subplot(grid[1, 3 + j])
        for curve in [c for c in curves.get("xv15_airplane_plan047", []) if c.condition.label.startswith(f"{v_kt} kt")]:
            tip = int(curve.condition.label.split(", ")[1].split(" ")[0])
            ax.plot(curve.x, curve.y, "--", color=palette[tips.index(tip)], lw=1, alpha=0.7)
        for curve in [c for c in airplane if c.condition.label.startswith(f"{v_kt} kt")]:
            tip = int(curve.condition.label.split(", ")[1].split(" ")[0])
            k = tips.index(tip)
            ax.plot(curve.condition.measured_x, curve.condition.measured_y, markers[k], color=palette[k], ms=7,
                    mfc="none", mew=1.4)
            ax.plot(curve.x, curve.y, "-", color=palette[k], lw=2, label=f"{tip} ft/s")
        ax.set(xlabel="$C_P = P/\\rho n^3 D^5$", title=f"XV-15 airplane, {v_kt} kt", ylim=(0.3, 1.0))
        if j == 0:
            ax.set_ylabel("Propulsive efficiency")
        ax.legend(frameon=False, fontsize=9, loc="lower right")
    r, bias, n = rms(airplane)
    figure.text(0.74, 0.475, f"XV-15 airplane mode (CR-114363, digitized), 64-series airfoils (dashed: plan 047 camber): RMS {r:.3f}, bias {bias:+.3f} ({n} points)",
                ha="center", fontsize=12, color=ink)
    figure.suptitle("Blade-element momentum rotor (plan 047, one drag factor fitted on JVX) against full-scale data",
                    fontsize=15, color=ink)
    figure.savefig(path, dpi=110, bbox_inches="tight")
    return path


if __name__ == "__main__":
    import sys
    import time
    start = time.time()
    for key in [k for k in groups if k in sys.argv[1:]] or ([] if "--cached" in sys.argv else groups):
        study_group(key)
    if "--sweep-only" in sys.argv:
        print(f"sweeps {time.time() - start:.0f} s")
        sys.exit()
    curves = load_curves()
    print(f"XV-15 sigma_tw {curves['xv15_solidity_thrust_weighted']:.4f}")
    for key in groups:
        r, bias, n = rms(curves[key])
        if key.endswith("hover"):
            r_attached, bias_attached, n_attached = rms(curves[key], attached)
            print(f"{key:14s} C_T/sigma <= {blade_loading_attached}: RMS {r_attached:.4f} bias {bias_attached:+.4f} "
                  f"over {n_attached} points")
        unsolved = sum(int(onp.isnan(c.x).sum()) for c in curves[key])
        print(f"{key:14s} RMS {r:.4f} bias {bias:+.4f} over {n} points; unsolved curve points {unsolved}")
        for curve in curves[key]:
            e = curve.error[onp.isfinite(curve.error)]
            print(f"   {curve.condition.label:40s} n {len(e):3d}  mean {onp.mean(e) if len(e) else onp.nan:+.3f}  "
                  f"max |e| {onp.max(onp.abs(e)) if len(e) else onp.nan:.3f}")
    print(make_figure(curves))
