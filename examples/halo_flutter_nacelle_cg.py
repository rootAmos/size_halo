"""Plan 056: Halo classic wing flutter and whirl flutter against the nacelle CG position relative to the elastic axis.

Nacelle CG from 0.75 m ahead of the wing elastic axis to 0.5 m behind it (mass and pitch inertia about the CG held,
as in plan 040; hub 1.0 m ahead):
- classic: beam finite elements with the nacelle as a rigid tip body, Theodorsen strip theory (2-D lift slope, and a
  Yates-type finite-span lift slope), no rotor, no knockdown;
- whirl: the plan 040 model (three assumed wing shapes, no wing aerodynamics), calibrated (x 0.84), delta-3 -18 and
  -10 deg, the lower of the two critical rotor speeds.
Writes output/whirl/nacelle_cg_sweep.json and docs/figures/flutter_nacelle_cg.png. Run from the repo root:
`python -m examples.halo_flutter_nacelle_cg` (needs output/structure/reference.json, as plan 040).
"""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as onp

from aircraft_closure.aeroelasticity import wing_flutter as wf
from examples import halo_whirl_flutter as hw
from examples import halo_whirl_flutter_levers as lv

root = Path(__file__).resolve().parents[1]
r = lv.reference
required_kt = 1.15 * 1.25 * r["velocity_max_m_s"] / hw.knot_m_s
length_m = (r["span_wing_m"] - r["width_fuselage_m"]) / 2
chord_m = r["chord_wing_m"]
mass_wing_kg_m = lv.mass_wing_kg_m
wing = wf.UniformWing(length_m=length_m, chord_m=chord_m, stiffness_beam_Nm2=lv.wing["stiffness_beam_Nm2"],
                      stiffness_torsion_Nm2=lv.wing["stiffness_torsion_Nm2"], mass_kg_m=mass_wing_kg_m,
                      inertia_torsion_kg_m=mass_wing_kg_m * (0.25 * chord_m) ** 2, fraction_elastic_axis=0.375)
aspect_ratio = r["span_wing_m"] / chord_m
factor_yates = float(wf.lift_slope_factor_helmbold(aspect_ratio))
offsets_m = [0.75, 0.5, 0.25, 0.0, -0.25, -0.5]
velocities_m_s = onp.arange(100, 1601, 10) * hw.knot_m_s
delta3_m10 = onp.tan(onp.radians(-10.0))


def tip(x_cg_m):
    m = lv.mass_tip_kg
    return wf.TipBody(m, x_cg_m, inertia_pitch_kg_m2=lv.inertia_cg_kg_m2, inertia_roll_kg_m2=m * 0.3**2)


def kt(v_m_s):
    return float("inf") if v_m_s is None else v_m_s / hw.knot_m_s


def sweep(count_elements=20):
    rows = []
    for x in offsets_m:
        row = dict(offset_cg_m=x)
        for name, factor in (("classic_2d", 1.0), ("classic_yates", factor_yates)):
            v, w, label = wf.flutter_speed(wing, tip(x), hw.density_kg_m3, velocities_m_s, count_elements, factor)
            row[name] = dict(kt=kt(v), frequency_rad_s=w, mode=label)
        v, w, label = wf.flutter_speed(wing, tip(x), hw.density_kg_m3, velocities_m_s, count_elements,
                                       basis=wf.uncoupled_basis(wing, count_elements))
        row["classic_2d_two_mode"] = dict(kt=kt(v), frequency_rad_s=w, mode=label)
        for name, changes in (("whirl_d3_18", {}), ("whirl_d3_10", dict(pitch_flap_coupling=delta3_m10))):
            row[name] = [dict(speed_rotor_rad_s=s, kt=float(v_kt), mode=m) for s, (v_kt, m) in
                         zip(lv.speeds_rad_s, lv.flutter_kt(lv.airframe(x_cg_m=x), **changes))]
        rows.append(row)
        print(f"CG {x:+.2f} m: classic 2-D {row['classic_2d']['kt']:.0f} kt ({row['classic_2d']['mode']}), "
              f"Yates {row['classic_yates']['kt']:.0f} kt, 2-mode {row['classic_2d_two_mode']['kt']:.0f} kt; whirl "
              f"d3 -18 {[round(c['kt']) for c in row['whirl_d3_18']]}, d3 -10 {[round(c['kt']) for c in row['whirl_d3_10']]}",
              flush=True)
    return rows


def figure(rows, path):
    plt.rcParams.update({"font.size": 13, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(11.5, 6.4), facecolor="white")
    ax.set_facecolor("white")
    x = [row["offset_cg_m"] for row in rows]
    whirl = lambda key: [min(c["kt"] for c in row[key]) for row in rows]
    ax.plot(x, whirl("whirl_d3_18"), "-o", color="#4A78D0", lw=2.2, ms=7, label="Whirl flutter, δ3 −18° (baseline hub)")
    ax.plot(x, whirl("whirl_d3_10"), "-o", color="#3E7D4F", lw=2.2, ms=7, label="Whirl flutter, δ3 −10°")
    for key, style, label in (("classic_2d", "-^", "Classic wing flutter (2-D strip theory)"),
                              ("classic_yates", "--^", "Classic wing flutter (finite-span lift slope)")):
        y = [row[key]["kt"] for row in rows]
        ax.plot(x, y, style, color="#A94F14", lw=2.0, ms=7, mfc="white" if "--" in style else "#A94F14", label=label)
        if key == "classic_2d":
            first = next((i for i, row in enumerate(rows) if row[key]["mode"] == "divergence"), None)
            if first is not None:
                ax.annotate("divergence before flutter", (x[first], y[first]), xytext=(0, -18),
                            textcoords="offset points", ha="left", fontsize=10, color="#A94F14")
    ax.axhline(required_kt, color="#A94F14", ls=":", lw=1.8)
    ax.text(x[0] + 0.02, required_kt + 42, f"required 1.15 V$_D$ = {required_kt:.0f} kt", color="#A94F14", fontsize=12,
            ha="left")
    ax.axvline(0, color="#2F5D8A", ls="--", lw=1.2)
    ax.text(0.02, 0.98, "wing elastic axis", transform=ax.get_xaxis_transform(), color="#2F5D8A", fontsize=11,
            va="top")
    ax.invert_xaxis()
    ax.set_xlabel("Nacelle CG relative to the wing elastic axis (m): ahead ←  → behind")
    ax.set_ylabel("Flutter speed (kt)")
    ax.set_ylim(150, None)
    ax.grid(color="#E4E3DF", lw=0.8)
    ax.legend(frameon=False, fontsize=11, loc="upper right", bbox_to_anchor=(1.0, 0.92))
    fig.tight_layout()
    fig.savefig(path, dpi=130, facecolor="white")


if __name__ == "__main__":
    rows = sweep()
    out = root / "output" / "whirl"
    out.mkdir(parents=True, exist_ok=True)
    (out / "nacelle_cg_sweep.json").write_text(json.dumps(dict(required_kt=required_kt, aspect_ratio=aspect_ratio,
                                                               factor_lift_slope_yates=factor_yates, rows=rows),
                                                          indent=1))
    figure(rows, root / "docs" / "figures" / "flutter_nacelle_cg.png")
