"""Plan 054 figure: Halo airframe L/D against equivalent airspeed, with the XV-15 for comparison.

    python -m examples.halo_lift_to_drag_figure output/halo_v3_9.pkl

Reads a `HaloSizingResult` pickle (the default solve) and writes docs/figures/halo_lift_to_drag.png.

- Halo: the sizing aerodynamics (AeroBuildup with the v3.9 calibration, tail-load trim drag, miscellaneous drag
  area) on the sized aircraft at take-off mass and the 10,000 ft cruise altitude, trimmed (lift = weight) at each
  speed. Airframe L/D, rotors in airplane mode feathered out of the drag (no slipstream increments).
- XV-15: L/D = W / (q D/q + W^2 / (q pi b^2 e)) from its published total drag area, weight and span (13,000 lb,
  32.17 ft, D/q 9.25 ft2; Johnson 2010, Table 1), with an assumed span efficiency e = 0.8 (not published).

Equivalent airspeed equals calibrated (and indicated, with no instrument error) airspeed below Mach 0.3; the
drag-area curves neglect compressibility.
"""
import pickle
import sys
from pathlib import Path

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import brentq

from examples.halo_sizing import HaloAssumptions, HaloRequirements, build_halo_aerodynamics, build_halo_aircraft

path_figure = Path("docs/figures/halo_lift_to_drag.png")
density_sea_level = asb.Atmosphere(altitude=0.0).density()
oswald_assumed = 0.8
surface, ink, muted, grid = "#FFFFFF", "#1D2433", "#5A6474", "#E3E1DA"
blue, orange = "#2A6FC2", "#C2611F"
# name, weight lb, span ft, D/q ft2, cruise (TAS kt, altitude ft) or None, colour, source
comparators = (("XV-15", 13000.0, 32.17, 9.25, None, orange, "Johnson 2010"),)


def eas_from_tas(tas_m_s, altitude_m):
    return tas_m_s * np.sqrt(asb.Atmosphere(altitude=altitude_m).density() / density_sea_level)


def drag_area_curve(weight_lb, span_ft, drag_area_ft2, eas_kt):
    """L/D from a total drag area and span, at equivalent airspeed (incompressible, e assumed)."""
    weight_N, span_m, drag_area_m2 = weight_lb * u.lbm * 9.81, span_ft * u.foot, drag_area_ft2 * u.foot**2
    q = 0.5 * density_sea_level * (eas_kt * u.knot)**2
    drag_N = q * drag_area_m2 + weight_N**2 / (q * np.pi * span_m**2 * oswald_assumed)
    return weight_N / drag_N


def halo_curve(result, eas_kt):
    """Trimmed airframe L/D of the sized Halo at take-off mass and the cruise altitude."""
    requirements, assumptions = HaloRequirements(), HaloAssumptions()
    aircraft = build_halo_aircraft(result.design, requirements, assumptions)
    aerodynamics = build_halo_aerodynamics(requirements, assumptions)
    altitude_m, weight_N = requirements.altitude_cruise_m, result.mass_takeoff_kg * 9.81
    ratio = float(eas_from_tas(1.0, altitude_m))
    values = []
    for eas in eas_kt:
        tas_m_s = eas * u.knot / ratio
        lift_excess = lambda alpha: float(np.asarray(
            aerodynamics.evaluate(aircraft, tas_m_s, altitude_m, alpha).lift_N).ravel()[0]) - weight_N
        if lift_excess(12.0) < 0.0:          # below the stall speed at this weight
            values.append(np.nan)
            continue
        alpha = brentq(lift_excess, -4.0, 12.0, xtol=1e-4)
        values.append(float(np.asarray(aerodynamics.evaluate(aircraft, tas_m_s, altitude_m, alpha)
                                       .lift_to_drag).ravel()[0]))
    return np.array(values), ratio


def plot(result, path):
    eas_kt = np.linspace(90.0, 260.0, 35)
    halo_ld, ratio = halo_curve(result, eas_kt)
    figure, ax = plt.subplots(figsize=(12, 6.2), dpi=150)
    figure.patch.set_facecolor(surface)
    ax.set_facecolor(surface)
    ax.plot(eas_kt, halo_ld, color=blue, linewidth=2.5,
            label=f"Halo v3.9 (sizing aero model, {result.mass_takeoff_kg / u.lbm:,.0f} lb, 10,000 ft)")
    cruise_eas = float(result.velocity_cruise_m_s / u.knot * ratio)
    cruise_ld = float(np.interp(cruise_eas, eas_kt, halo_ld))
    ax.plot(cruise_eas, cruise_ld, "o", color=blue, markersize=9, markeredgecolor=surface, markeredgewidth=2)
    ax.annotate(f"mission cruise {result.velocity_cruise_m_s / u.knot:.0f} kt TAS\n({cruise_eas:.0f} kt EAS), "
                f"L/D {cruise_ld:.1f}", (cruise_eas, cruise_ld), xytext=(-10, 18), textcoords="offset points",
                ha="right", fontsize=11, color=ink)
    dash_eas = 210.0 * ratio
    dash_ld = float(np.interp(dash_eas, eas_kt, halo_ld))
    ax.plot(dash_eas, dash_ld, "s", color=blue, markersize=8, markeredgecolor=surface, markeredgewidth=2)
    ax.annotate(f"210 kt TAS dash, L/D {dash_ld:.1f}", (dash_eas, dash_ld), xytext=(10, 10),
                textcoords="offset points", fontsize=11, color=ink)
    for name, weight_lb, span_ft, drag_area_ft2, cruise, colour, source in comparators:
        values = drag_area_curve(weight_lb, span_ft, drag_area_ft2, eas_kt)
        ax.plot(eas_kt, values, color=colour, linewidth=2, linestyle=(0, (6, 3)),
                label=f"{name}: {weight_lb:,.0f} lb, D/q {drag_area_ft2} ft², e {oswald_assumed} assumed ({source})")
        best = int(np.nanargmax(values))
        ax.plot(eas_kt[best], values[best], "D", color=colour, markersize=7, markeredgecolor=surface)
        ax.annotate(f"max {values[best]:.1f}", (eas_kt[best], values[best]), xytext=(0, 9 if values[best] > 12 else -20),
                    textcoords="offset points", ha="center", fontsize=10.5, color=ink)
        if cruise is not None:
            eas = float(eas_from_tas(cruise[0] * u.knot, cruise[1] * u.foot) / u.knot)
            value = float(drag_area_curve(weight_lb, span_ft, drag_area_ft2, eas))
            ax.plot(eas, value, "o", color=colour, markersize=8, markeredgecolor=surface, markeredgewidth=2)
            ax.annotate(f"design cruise {cruise[0]:.0f} kt TAS at {cruise[1]:,.0f} ft\n({eas:.0f} kt EAS), "
                        f"L/D {value:.1f}", (eas, value), xytext=(10, -34), textcoords="offset points",
                        fontsize=10.5, color=ink)
    best = int(np.nanargmax(halo_ld))
    ax.plot(eas_kt[best], halo_ld[best], "D", color=blue, markersize=7, markeredgecolor=surface)
    ax.annotate(f"max {halo_ld[best]:.1f}", (eas_kt[best], halo_ld[best]), xytext=(0, 9), textcoords="offset points",
                ha="center", fontsize=10.5, color=ink)
    ax.set_xlabel("Equivalent airspeed (≈ indicated), kt", fontsize=12, color=muted)
    ax.set_ylabel("Airframe L/D", fontsize=12, color=muted)
    ax.set_xlim(eas_kt[0], eas_kt[-1])
    ax.set_ylim(0, 1.18 * np.nanmax(halo_ld))
    ax.tick_params(colors=muted, labelsize=11)
    ax.grid(color=grid, linewidth=0.8)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color("#C9C7BF")
    ax.legend(loc="lower center", frameon=False, fontsize=10.5)
    figure.suptitle("Airplane-mode L/D against airspeed: Halo v3.9 and the XV-15", x=0.02, ha="left",
                    fontsize=16, color=ink, fontweight="bold")
    figure.text(0.02, 0.015, "Airframe only (no rotor or propulsive losses). XV-15 from its NDARC total drag area D/q "
                "and span with span efficiency 0.8 assumed; incompressible.", fontsize=9.5, color=muted)
    figure.tight_layout(rect=(0, 0.03, 1, 0.96))
    figure.savefig(path, facecolor=surface)
    plt.close(figure)


if __name__ == "__main__":
    with open(sys.argv[1], "rb") as file:
        result = pickle.load(file)
    plot(result, path_figure)
    print(f"wrote {path_figure}")
