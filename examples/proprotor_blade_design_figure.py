"""Figure for plan 047: docs/figures/proprotor_blade_design.png.

Reads the saved study (`examples/halo_blade_design.py` -> output/blade_design_study.pkl) and the JVX report
(`examples/jvx_rotor_bemt.py`, saved to output/jvx_bemt_report.pkl).
"""
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as onp

root = Path(__file__).resolve().parents[1]
blue, orange, gray, ink, ink_muted = "#2a78d6", "#eb6834", "#8a8984", "#0b0b0b", "#52514e"


def _blade_curves(evaluation, fractions):
    blade = evaluation.blade
    return onp.asarray(blade.twist_deg(fractions), dtype=float), onp.asarray(blade.chord_m(fractions), dtype=float)


def _label(name, evaluation):
    fm = float(evaluation.hover.figure_of_merit)
    eta = float(evaluation.cruise.efficiency_propulsive)
    return f"{name}: FM {fm:.3f}, cruise $\\eta$ {eta:.3f}"


def make_figure(study, jvx, path):
    plt.rcParams.update({"font.size": 13, "axes.edgecolor": ink_muted, "axes.labelcolor": ink, "xtick.color": ink_muted,
                         "ytick.color": ink_muted, "axes.spines.top": False, "axes.spines.right": False})
    figure, axes = plt.subplots(2, 2, figsize=(16, 10), facecolor="white")
    fractions = onp.linspace(0.10, 1.0, 91)
    floor_leaning, hover_leaning = next(((f, d) for f, d in reversed(study.front) if d is not None), (None, None))
    curves = [(study.optimum, blue, "-", "Optimum, FM floor = sizing model"),
              (study.reference, gray, "--", "XV-15-like: constant chord, linear 40 deg")]
    if hover_leaning is not None and hover_leaning is not study.optimum:
        curves.insert(1, (hover_leaning, orange, "-", f"Hover-leaning, FM $\\geq$ {floor_leaning:.2f}"))

    for evaluation, color, style, name in curves:
        twist, chord = _blade_curves(evaluation, fractions)
        label = f"{name} (stalls in hover)" if evaluation.hover is None else _label(name, evaluation)
        axes[0, 0].plot(fractions, twist, style, color=color, lw=2.2, label=label)
        axes[0, 1].plot(fractions, chord, style, color=color, lw=2.2, label=label)
    axes[0, 0].set(xlabel="Radial station r/R", ylabel="Twist relative to 0.75 R (deg)", title="Twist")
    axes[0, 1].set(xlabel="Radial station r/R", ylabel="Chord (m)", title="Chord (thrust-weighted solidity in legend)")
    axes[0, 1].set_ylim(bottom=0)
    axes[0, 0].legend(frameon=False, fontsize=11, loc="upper right")
    handles = [plt.Line2D([], [], color=c, ls=s, lw=2.2) for e, c, s, n in curves]
    axes[0, 1].legend(handles, [f"$\\sigma_{{tw}}$ = {e.solidity_thrust_weighted:.3f}" for e, c, s, n in curves],
                      frameon=False, fontsize=11, loc="upper right")

    ax = axes[1, 0]
    front = sorted([d for _, d in study.front if d is not None] + [study.optimum],
                   key=lambda d: float(d.hover.figure_of_merit))
    fm = [float(d.hover.figure_of_merit) for d in front]
    eta = [float(d.cruise.efficiency_propulsive) for d in front]
    ax.plot(eta, fm, "-o", color=blue, lw=2, ms=8, label="BEM designs (min cruise power at each FM floor)")
    ax.plot([float(study.optimum.cruise.efficiency_propulsive)], [float(study.optimum.hover.figure_of_merit)], "o",
            color=blue, ms=15, mfc="none", mew=2.5)
    ax.plot([study.current["cruise"]["efficiency"]], [study.current["hover"]["figure_of_merit"]], "D", color=ink,
            ms=9, label="Sizing model (momentum + profile, $\\sigma$ 0.060)")
    eta_reference = float(study.reference.cruise.efficiency_propulsive)
    ax.axvline(eta_reference, color=gray, ls="--", lw=1.5,
               label=f"XV-15-like blade, $\\sigma$ 0.060: cruise $\\eta$ {eta_reference:.3f}, hover stalls")
    optimum = study.optimum
    ax.annotate(f"Optimum at the sizing FM:\nFM {float(optimum.hover.figure_of_merit):.3f}, "
                f"$\\eta$ {float(optimum.cruise.efficiency_propulsive):.3f}, $\\sigma_{{tw}}$ {optimum.solidity_thrust_weighted:.3f}",
                (float(optimum.cruise.efficiency_propulsive), float(optimum.hover.figure_of_merit)), xytext=(-170, 10),
                textcoords="offset points", ha="right", fontsize=11, color=ink,
                arrowprops=dict(arrowstyle="-", color=ink_muted, lw=1))
    ax.set(xlabel="Cruise propulsive efficiency, 165 kt at 10,000 ft", ylabel="Hover figure of merit, 4,000 ft",
           title="Hover-cruise trade for the Halo rotor (R 4.83 m, 3 blades)")
    ax.legend(frameon=False, fontsize=10.5, loc="lower left")

    ax = axes[1, 1]
    ok_h = onp.isfinite(jvx.hover_fm_predicted)
    ok_a = onp.isfinite(jvx.airplane_eta_predicted)
    ax.plot([0.2, 1.0], [0.2, 1.0], color=gray, lw=1)
    ax.fill_between([0.2, 1.0], [0.17, 0.97], [0.23, 1.03], color=gray, alpha=0.12, lw=0)
    ax.plot(jvx.hover_fm_measured[ok_h], jvx.hover_fm_predicted[ok_h], "o", color=blue, ms=7, mec="white",
            label=f"Hover FM (D-1 RMS {jvx.hover_fm_rms:.3f})")
    ax.plot(jvx.airplane_eta_measured[ok_a], jvx.airplane_eta_predicted[ok_a], "s", color=orange, ms=7, mec="white",
            label=f"Airplane-mode $\\eta$ (RMS {jvx.airplane_eta_rms:.3f})")
    ax.set(xlim=(0.25, 0.95), ylim=(0.25, 0.95), xlabel="JVX test (NASA/TM-2016-219070)", ylabel="BEM prediction",
           title=f"JVX validation (drag factor {jvx.drag_factor:.2f}; band $\\pm$0.03)")
    ax.legend(frameon=False, fontsize=11, loc="upper left")
    for ax in axes.ravel():
        ax.grid(color="#e4e3df", lw=0.8)
    figure.suptitle("Proprotor blade design with blade-element momentum and NeuralFoil sections (plan 047)",
                    fontsize=16, color=ink)
    figure.tight_layout()
    figure.savefig(path, dpi=120, facecolor="white")


if __name__ == "__main__":
    study = pickle.load(open(root / "output" / "blade_design_study.pkl", "rb"))
    jvx = pickle.load(open(root / "output" / "jvx_bemt_report.pkl", "rb"))
    make_figure(study, jvx, root / "docs" / "figures" / "proprotor_blade_design.png")
