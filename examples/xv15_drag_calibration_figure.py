"""Plan 054 figure: the excrescence correction each drag model needs to match the NDARC XV-15 components.

    python -m examples.xv15_drag_calibration_figure

Writes two figures of the XV-15 cruise CD0 of the wing, tails, fuselage and pylons in drag
counts on the 169 ft2 wing (alpha 0, no induced, trim or fittings drag), from AeroBuildup and the
DATCOM/Raymer/Scholz build-up on `build_xv15_drag_aircraft` with the sizing's transition setting, against NDARC
(Johnson 2010, Table 1): docs/figures/xv15_drag_calibration.png, each model's clean components and the excrescence
factor that brings them to NDARC's total; docs/figures/xv15_drag_components.png, the clean-component delta against
NDARC, component by component.
"""
from pathlib import Path

import aerosandbox.numpy as np
import aerosandbox.tools.units as u
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.scholz import InterferenceFactors, ScholzAerodynamics
from examples.halo_sizing import HaloAssumptions
from examples.xv15_reference import Xv15Reference, build_xv15_drag_aircraft

path_totals = Path("docs/figures/xv15_drag_calibration.png")
path_components = Path("docs/figures/xv15_drag_components.png")
source = ("Source: W. Johnson, NDARC validation, AHS Aeromechanics 2010, Table 1 (components D/q 6.25 ft²); "
          "NASA TM X-62407 sec. 3.3. Calibration: tests/aerodynamics/test_drag_corrections.py")
# NDARC XV-15 cruise drag areas, ft2 (pylons: 2 x 0.76).
ndarc_ft2 = dict(wing=2.18, fuselage=1.56, nacelles=1.52, horizontal_tail=0.63, vertical_tail=0.36)
names = dict(wing="Wing", fuselage="Fuselage", nacelles="Nacelles (pylons)", horizontal_tail="Horizontal tail",
             vertical_tail="Vertical tail")
surface, ink, muted = "#FFFFFF", "#1D2433", "#5A6474"
models = (("AeroBuildup\n(Baseline in Sizing Model)", "#7FB2EC", "#2A6FC2"),
          ("DATCOM/Raymer/Scholz\n(hand check)", "#F0B98D", "#C2611F"))


def breakdowns():
    """NDARC and the two clean-component breakdowns, counts on the XV-15 wing; the two excrescence factors."""
    reference, a = Xv15Reference(), HaloAssumptions()
    xv15 = build_xv15_drag_aircraft(reference)
    tails = InterferenceFactors(horizontal_tail=1.08, vertical_tail=1.08)
    flight = (reference.velocity_cruise_m_s, reference.altitude_cruise_m)
    buildup = BuildupAerodynamics(interference=tails, xtr_upper=a.fraction_chord_transition,
                                  xtr_lower=a.fraction_chord_transition).evaluate_buildup(xv15, *flight, 0.0).breakdown
    scholz = ScholzAerodynamics(interference=tails).parasite_drag_breakdown(xv15, *flight)
    to_counts = lambda items: {i.label: 1e4 * float(np.asarray(i.cd0).ravel()[0]) for i in items if i.label in ndarc_ft2}
    ndarc = {k: 1e4 * v * u.foot**2 / xv15.wing.area_m2 for k, v in ndarc_ft2.items()}
    return ndarc, (to_counts(buildup), to_counts(scholz)), (a.factor_excrescence_buildup, a.factor_excrescence_scholz)


def style(ax):
    ax.set_facecolor(surface)
    ax.tick_params(axis="x", colors=muted, labelsize=11)
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="x", color="#E3E1DA", linewidth=0.8)
    ax.set_axisbelow(True)
    for spine in ("top", "right", "left"):
        ax.spines[spine].set_visible(False)
    ax.spines["bottom"].set_color("#C9C7BF")


def plot_totals(ax, ndarc, components, factors):
    target = sum(ndarc.values())
    labels = ["NDARC XV-15\n(Johnson 2010)"] + [m[0] for m in models]
    height, edge = 0.56, dict(edgecolor=surface, linewidth=2)
    top = len(labels) - 1
    ax.barh(top, target, height, color=ink, **edge)
    ax.text(target / 2, top, f"{target:.0f}", ha="center", va="center", fontsize=12, color=surface)
    ax.text(target + 8, top, f"{target:.0f} counts", ha="left", va="center", fontsize=13, color=ink,
            fontweight="bold")
    for index, ((_, colour_clean, colour_correction), parts, factor) in enumerate(zip(models, components, factors)):
        y, clean = top - 1 - index, sum(parts.values())
        ax.barh(y, clean, height, color=colour_clean, **edge)
        ax.text(clean / 2, y, f"{clean:.0f}",
                ha="center", va="center", fontsize=12, color=ink)
        ax.barh(y, target - clean, height, left=clean, color=colour_correction, **edge)
        ax.text((clean + target) / 2, y, f"+{round(target) - round(clean)}", ha="center", va="center",  # adds up
                fontsize=12, color="white", fontweight="bold")
        ax.text(target + 8, y, f"{target:.0f} (×{factor:.2f})", ha="left", va="center", fontsize=13, color=ink)
    ax.axvline(target, color=ink, linestyle=(0, (4, 3)), linewidth=1.2, zorder=0)
    ax.set_yticks(range(len(labels)), labels[::-1], fontsize=12, color=ink)
    ax.set_xlim(0, 470)
    ax.set_ylim(-0.5, len(labels) - 0.5)
    ax.set_xlabel("CD0, drag counts on the XV-15 wing (169 ft²)", fontsize=12, color=muted)
    style(ax)


def plot_components(ax, ndarc, components):
    keys = list(ndarc_ft2)
    height = 0.26
    for row, key in enumerate(keys):
        y = len(keys) - 1 - row
        ax.barh(y + height, ndarc[key], height, color=ink, edgecolor=surface, linewidth=1.5,
                label="NDARC" if row == 0 else None)
        for index, ((label, colour, _), parts) in enumerate(zip(models, components)):
            value, offset = parts[key], -index * height
            ax.barh(y + offset, value, height, color=colour, edgecolor=surface, linewidth=1.5,
                    label=label.splitlines()[0] if row == 0 else None)
            delta = value - ndarc[key]
            text = f"{delta:+.0f} ({delta / ndarc[key]:+.0%})".replace("-", "−").replace("%", " %")
            ax.text(max(value, ndarc[key]) + 3, y + offset, text, ha="left", va="center", fontsize=10.5, color=ink)
        ax.text(ndarc[key] + 3, y + height, f"{ndarc[key]:.0f}", ha="left", va="center", fontsize=10.5, color=muted)
    ax.set_yticks(range(len(keys)), [names[k] for k in keys[::-1]], fontsize=12, color=ink)
    ax.set_xlim(0, 175)
    ax.set_xlabel("CD0, drag counts on the XV-15 wing (169 ft²)", fontsize=12, color=muted)
    ax.legend(loc="lower right", frameon=False, fontsize=11)
    style(ax)


def save(figure, title, path):
    figure.patch.set_facecolor(surface)
    figure.suptitle(title, x=0.02, ha="left", fontsize=16, color=ink, fontweight="bold")
    figure.text(0.02, 0.015, source, fontsize=9.5, color=muted)
    figure.tight_layout(rect=(0, 0.035, 1, 0.97))
    figure.savefig(path, facecolor=surface)
    plt.close(figure)


def plot(ndarc, components, factors):
    figure, ax = plt.subplots(figsize=(13, 5.0), dpi=150)
    plot_totals(ax, ndarc, components, factors)
    save(figure, "XV-15 cruise parasite drag: excrescence correction needed to match NDARC", path_totals)
    figure, ax = plt.subplots(figsize=(13, 7.0), dpi=150)
    plot_components(ax, ndarc, components)
    save(figure, "XV-15 cruise parasite drag by component: model delta against NDARC, before the correction",
         path_components)


if __name__ == "__main__":
    ndarc, components, factors = breakdowns()
    for (label, _, _), parts, factor in zip(models, components, factors):
        print(f"{label.splitlines()[0]:<22} " + ", ".join(f"{k} {v - ndarc[k]:+.0f}" for k, v in parts.items())
              + f"; total {sum(parts.values()):.1f} x{factor:.2f} (NDARC {sum(ndarc.values()):.1f})")
    plot(ndarc, components, factors)
    print(f"wrote {path_totals} and {path_components}")
