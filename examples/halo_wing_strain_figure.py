"""Plan 043 figure: the negative spar-cap strain margin found by FE, closed in the sizing.

Left: peak front spar-cap strain margin at ultimate (1 - strain / allowable) for each case, reduced model and
CalculiX. Right: the sized wing's spanwise cap strain near the root, CalculiX elements against the reduced model.
Reads the plan 043 outputs (`examples/halo_wing_strain.py`, `examples/halo_wing_fe.py`) and the plan 035 FE run
(`--fe-v335`). Run from the repo root: `python -m examples.halo_wing_strain_figure --fe-v335 <dir>`.
"""
import argparse
import json
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from aircraft_closure.weights import afdd  # noqa: E402

color_model, color_fe, ink, muted, grid = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e4e3df"


def cap_strains(directory):
    """(|y| m, x/c, spanwise strain) of every cap element in the printed root band of a jump deck."""
    nodes, caps, mode, current = {}, {}, None, None
    for line in (directory / "halo_wing_jump.inp").read_text().splitlines():
        if not line.strip() or line.startswith("**"):
            continue
        if line.startswith("*"):
            mode = line.split(",")[0].strip().upper()
            match = re.search(r"ELSET=([^,\s]+)", line, re.I)
            current = match.group(1) if match else None
            continue
        values = [v.strip() for v in line.split(",") if v.strip()]
        if mode == "*NODE":
            nodes[int(values[0])] = np.array([float(v) for v in values[1:4]])
        elif mode == "*ELEMENT" and current == "ECAP":
            caps[int(values[0])] = [int(v) for v in values[1:]]
    text = (directory / "halo_wing_jump.dat").read_text().split("strains (elem, integ.pnt.,exx,eyy,ezz,exy,exz,eyz)")[1]
    strains = {}
    for m in re.finditer(r"^\s*(\d+)\s+\d+\s+[-\d.E+]+\s+([-\d.E+]+)", text, re.M):
        strains.setdefault(int(m.group(1)), []).append(float(m.group(2)))
    return np.array([(*np.mean([nodes[n] for n in caps[e]], axis=0)[:2], np.mean(v)) for e, v in strains.items()])


def model_strain(reference, factor_peak, distance_m, factor_load_carrying=1.0):
    r, w, material = reference, reference["wing_afdd"], reference["material_wing"]
    moment_Nm = afdd.moment_jump_station_ultimate_Nm(r["mass_takeoff_kg"], r["mass_tip_kg"], r["span_wing_m"],
                                                     r["width_fuselage_m"], distance_m,
                                                     load_factor_jump=r["load_factor_jump"])
    return afdd.strain_cap_jump_ultimate(r["area_wing_m2"] / r["span_wing_m"], r["thickness_to_chord_wing"],
                                         r["fraction_chord_torque_box"], r["ratio_depth_spar_cap"],
                                         w["area_torque_box_m2"], w["area_spar_m2"], material["modulus_torque_box_Pa"],
                                         material["modulus_spar_Pa"], moment_Nm, factor_load_carrying, factor_peak)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fe-v335", required=True, help="plan 035 output folder (structure/ and fe_wing/)")
    parser.add_argument("--output", default="docs/figures/wing_strain_margin.png")
    args = parser.parse_args()
    factor_peak, allowable = 1.3324, 0.0047
    v335 = Path(args.fe_v335)
    fe_v335 = json.loads((v335 / "fe_wing" / "summary.json").read_text())["strain_max_jump"]
    reference_v335 = json.loads((v335 / "structure" / "reference.json").read_text())
    half, none = Path("output/wing_strain/share05"), Path("output/wing_strain/share00p")
    summary_half, summary_none = (json.loads((p / "summary.json").read_text()) for p in (half, none))
    path_fe_half = Path("output/fe_wing/share05/summary.json")
    fe_half = json.loads(path_fe_half.read_text())["strain_max_jump"] if path_fe_half.exists() else None
    fe_none = json.loads(Path("output/fe_wing/share00p/summary.json").read_text())["strain_max_jump"]
    reference_none = json.loads((none / "reference.json").read_text())
    s_half = summary_half["sized"]["wing_strain"]
    strain_v36_raw = s_half["strain_peak"] * s_half["factor_load_carrying"]   # same wing, no credit
    delta_lb = summary_none["sized"]["mass_takeoff_lb"] - summary_none["initial"]["mass_takeoff_lb"]
    delta_wing_kg = (summary_none["sized"]["component_masses_kg"]["wing"]
                     - summary_none["initial"]["component_masses_kg"]["wing"])
    cases = [  # label, model strain, FE strain (None: not run)
        ("v3.3.5 wing, AFDD gauges\n(the FE finding; calibration point)",
         model_strain(reference_v335, factor_peak, 0.3)[1], fe_v335),
        ("v3.6 reference (15,179 lb),\nAFDD gauges only", strain_v36_raw, None),
        ("v3.6 reference, half the calibration\nmaterial load-carrying (assumed)", s_half["strain_peak"], fe_half),
        (f"Sized with the margin, no credit\n(+{delta_wing_kg:.0f} kg wing, +{delta_lb:.0f} lb take-off)",
         summary_none["sized"]["wing_strain"]["strain_peak"], fe_none),
    ]

    plt.rcParams.update({"font.size": 13, "axes.edgecolor": muted, "axes.labelcolor": ink, "xtick.color": muted,
                         "ytick.color": muted, "font.family": "DejaVu Sans"})
    figure, (left, right) = plt.subplots(1, 2, figsize=(16, 10), gridspec_kw=dict(width_ratios=(1.25, 1)))
    figure.patch.set_facecolor("white")
    rows = np.arange(len(cases))[::-1]
    for row, (label, model, fe) in zip(rows, cases):
        left.plot([0, 100 * (1 - model / allowable)], [row, row], color=grid, lw=2, zorder=1)
        left.scatter(100 * (1 - model / allowable), row, s=140, color=color_model, zorder=3,
                     edgecolor="white", linewidth=2)
        margin_model = round(100 * (1 - model / allowable), 1) + 0.0      # no "-0.0"
        left.annotate(f"{margin_model:+.1f} %", (100 * (1 - model / allowable), row),
                      xytext=(0, 14), textcoords="offset points", ha="center", color=ink, fontsize=12)
        if fe is not None:
            left.scatter(100 * (1 - fe / allowable), row, s=150, marker="D", color=color_fe, zorder=4,
                         edgecolor="white", linewidth=2)
            left.annotate(f"{100 * (1 - fe / allowable):+.1f} %", (100 * (1 - fe / allowable), row),
                          xytext=(0, -24), textcoords="offset points", ha="center", color=ink, fontsize=12)
    left.axvline(0, color=ink, lw=1.2)
    left.text(0.3, rows[0] + 0.55, "allowable", color=ink, fontsize=12)
    left.set_yticks(rows, [c[0] for c in cases], fontsize=12)
    left.set_ylim(-0.7, len(cases) - 0.2)
    left.set_xlim(-14, 14)
    left.set_xlabel("Ultimate spar-cap strain margin, 1 - strain / allowable (%)")
    left.scatter([], [], s=140, color=color_model, label="Reduced model in the sizing")
    left.scatter([], [], s=150, marker="D", color=color_fe, label="CalculiX shell model")
    left.legend(loc="lower left", frameon=False, fontsize=12)
    left.grid(axis="x", color=grid, lw=0.8)
    left.set_axisbelow(True)
    for side in ("top", "right"):
        left.spines[side].set_visible(False)
    left.set_title("Peak front spar-cap strain, ultimate jump take-off", loc="left", color=ink, fontsize=15)

    elements = cap_strains(Path("output/fe_wing/share00p"))
    r = reference_none
    chord_m, x_le_m = r["area_wing_m2"] / r["span_wing_m"], r["x_le_wing_m"]
    outboard_m = np.abs(elements[:, 1]) - r["width_fuselage_m"] / 2
    front = (elements[:, 0] - x_le_m) / chord_m < 0.375
    right.scatter(outboard_m[front], 1e3 * np.abs(elements[front, 2]), s=22, color=color_fe, alpha=0.75,
                  label="CalculiX, front cap elements", edgecolor="none")
    right.scatter(outboard_m[~front], 1e3 * np.abs(elements[~front, 2]), s=22, color=muted, alpha=0.45,
                  label="CalculiX, rear cap elements", edgecolor="none")
    distances_m = np.linspace(0.0, 1.2, 50)
    beam = np.array([model_strain(r, factor_peak, d)[0] for d in distances_m])
    right.plot(distances_m, 1e3 * beam, color=color_model, lw=2, ls="--", label="Beam theory (AFDD section)")
    right.plot(distances_m, 1e3 * factor_peak * beam, color=color_model, lw=2,
               label=f"Reduced model, x {factor_peak:.2f} peak factor")
    right.scatter([0.3], [1e3 * model_strain(r, factor_peak, 0.3)[1]], s=140, color=color_model, zorder=4,
                  edgecolor="white", linewidth=2)
    right.axhline(1e3 * allowable, color=ink, lw=1.2)
    right.text(1.18, 1e3 * allowable + 0.05, "allowable 4.7", ha="right", va="bottom", color=ink, fontsize=12)
    right.axvspan(0.3, 1.0, color=grid, alpha=0.5, lw=0)
    right.text(0.65, 1.15, "FE strain band", ha="center", color=muted, fontsize=12)
    right.set_xlim(0, 1.2)
    right.set_ylim(1.0, 6.0)
    right.set_xlabel("Distance outboard of the fuselage side (m)")
    right.set_ylabel("Spanwise cap strain at ultimate (millistrain)")
    right.legend(loc="upper right", frameon=False, fontsize=12)
    for side in ("top", "right"):
        right.spines[side].set_visible(False)
    right.set_title("Sized wing (no credit): FE against the reduced model", loc="left", color=ink, fontsize=15)
    figure.suptitle("Wing strength: a negative margin found by FE, closed in the sizing (plan 043)",
                    x=0.01, ha="left", fontsize=18, color=ink)
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=120, facecolor="white")
    print(f"Wrote {args.output}")
