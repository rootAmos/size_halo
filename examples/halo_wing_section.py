"""Halo wing section thickness as a sizing trade (plan 046).

The wing section is the CST family around NACA 2423 (`aircraft_closure.aerodynamics.wing_section`): NeuralFoil
polars in AeroBuildup at every flight point, section maximum lift scaling the stall requirement, and the same
thickness ratio sizing the AFDD wing (box depth, caps, stiffness) and setting the whirl frequency margins.

Each case is one coupled solve from the cached v3.6 baseline (`output/baseline_plain.pkl`, the plain fields of the
15,179 lb result) with the plan 035 machine unit counts held at the baseline's (2 motor, 3 generator units), so
there is no relaxed-then-integer double solve:

    python -m examples.halo_wing_section solve t23        # family section at 23 % (the reference)
    python -m examples.halo_wing_section solve free       # thickness free, frequency-placement margins only
    python -m examples.halo_wing_section solve free_stiff # thickness free, GJ and EI at least the 23 % wing's
    python -m examples.halo_wing_section solve t18|t21|t25
    python -m examples.halo_wing_section solve free_stiff t25 free_stiff_t25   # same case from the t25 result
    python -m examples.halo_wing_section report           # table, coupled whirl-flutter check, figure

Results go to `output/wing_section/<case>.pkl`.
"""
import json
import pickle
import sys
from dataclasses import fields, replace
from pathlib import Path

import aerosandbox as asb
import numpy as onp

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import (HaloAssumptions, HaloDesign, HaloRequirements, HaloSizingResult,
                                  build_halo_aerodynamics, build_halo_aircraft, halo_wing_section, solve_halo_sizing)

path_baseline = Path("output/baseline_plain.pkl")
path_results = Path("output/wing_section")
path_figure = Path("docs/figures/neuralfoil_wing_section.png")


def baseline_initial(path=path_baseline):
    """The cached baseline as a `HaloSizingResult` of this branch (fields it does not have are dropped)."""
    plain = pickle.loads(path.read_bytes())
    names = {f.name for f in fields(HaloDesign)}
    design = HaloDesign(**{k: v for k, v in plain["design"].items() if k in names})
    kept = {f.name: plain[f.name] for f in fields(HaloSizingResult) if f.name in plain and f.name != "design"}
    return HaloSizingResult(design=design, **kept)


def assumptions_for(case, stiffness_min_Nm2=None):
    """Family section with unit counts fixed; `case` "tNN" fixes thickness NN %, "free" frees it, "free_stiff"
    frees it with the realized (torsion, beam) stiffness floor `stiffness_min_Nm2`."""
    a = replace(HaloAssumptions(), wing_section_model="family", count_units_motor=2, count_units_generator=3)
    if case.startswith("t"):
        return replace(a, thickness_to_chord_wing=int(case[1:]) / 100)
    if case == "free":
        return replace(a, free_thickness_to_chord_wing=True)
    if case.startswith("free_stiff"):
        return replace(a, free_thickness_to_chord_wing=True, stiffness_min_wing_Nm2=stiffness_min_Nm2)
    raise ValueError(case)


def thickness_of(result, assumptions):
    return float(halo_wing_section(result.design, assumptions)[0])


def condition_of(result, requirements, assumptions):
    return StructuralDesignCondition(mass_design_kg=result.mass_takeoff_kg,
                                     load_factor_ultimate=assumptions.load_factor_ultimate,
                                     velocity_cruise_m_s=result.velocity_cruise_m_s,
                                     altitude_cruise_m=requirements.altitude_cruise_m,
                                     lift_to_drag_cruise=result.lift_to_drag_cruise)


def wing_reference(result, requirements, assumptions):
    """Wing, tip and rotor inputs of the plan 040 whirl-flutter check (as `examples.halo_structure_reference`)."""
    aircraft = build_halo_aircraft(result.design, requirements, assumptions)
    condition = condition_of(result, requirements, assumptions)
    wing, model = aircraft.wing, aircraft.wing.mass_model
    masses = model.masses(wing, condition)
    return dict(
        thickness_to_chord_wing=float(model.thickness_to_chord),
        span_wing_m=float(wing.span_m()), chord_wing_m=float(wing.chord_mean_m()),
        width_fuselage_m=float(model.width_fuselage_m), mass_tip_kg=float(model.mass_tip_kg),
        radius_gyration_pylon_m=float(model.radius_gyration_pylon_m), radius_rotor_m=float(result.radius_rotor_m),
        mass_wing_kg=float(aircraft.get_mass_breakdown(condition).wing.mass),
        wing_afdd={f.name: float(getattr(masses, f.name)) for f in fields(masses)},
        count_blades=int(assumptions.count_blades),
        solidity=float(result.design.solidity if result.design.solidity is not None else assumptions.solidity),
        length_mast_m=float(assumptions.length_mast_m), velocity_max_m_s=float(requirements.velocity_max_m_s),
        whirl_points=[dict(label=w["label"], speed_rotor_rad_s=float(w["speed_rotor_rad_s"]))
                      for w in result.whirl_flutter])


def cruise_drag(result, requirements, assumptions):
    """(wing profile CD0 incl. interference, total parasite CD0) at the cruise speed and lift coefficient, from
    AeroBuildup on the solved aircraft (no blown-wing increments; alpha found by linear interpolation)."""
    aircraft = build_halo_aircraft(result.design, requirements, assumptions)
    aerodynamics = build_halo_aerodynamics(requirements, assumptions)
    args = (aircraft, result.velocity_cruise_m_s, requirements.altitude_cruise_m)
    cl0, cl4 = (float(onp.ravel(aerodynamics.evaluate_buildup(*args, alpha).cl_aerobuildup)[0]) for alpha in (0, 4))
    alpha = 4 * (result.cl_cruise - cl0) / (cl4 - cl0)
    out = aerodynamics.evaluate_buildup(*args, alpha)
    wing_cd0 = next(float(onp.ravel(i.cd0)[0]) for i in out.breakdown if i.label == "wing")
    return wing_cd0, float(onp.ravel(out.aero.cd0)[0])


def solve_case(case, start=None, label=None):
    """One solve of `case` from the cached baseline, or from the saved case `start`; saved as `label` (or case)."""
    requirements = HaloRequirements()
    initial = baseline_initial() if start is None else pickle.loads((path_results / f"{start}.pkl").read_bytes())
    stiffness = None
    if case.startswith("free_stiff"):
        reference = pickle.loads((path_results / "t23.pkl").read_bytes())
        wing = wing_reference(reference, requirements, assumptions_for("t23"))["wing_afdd"]
        stiffness = (wing["stiffness_torsion_Nm2"], wing["stiffness_beam_Nm2"])
        print(f"stiffness floor GJ {stiffness[0]:.4g} N m2, EI beam {stiffness[1]:.4g} N m2 (the 23 % wing)")
    assumptions = assumptions_for(case, stiffness)
    result = solve_halo_sizing(requirements, assumptions, initial=initial)
    path_results.mkdir(parents=True, exist_ok=True)
    label = label or case
    (path_results / f"{label}.pkl").write_bytes(pickle.dumps(result))
    print(f"{label}: t/c {thickness_of(result, assumptions):.4f}  take-off {result.mass_takeoff_kg:.1f} kg "
          f"({result.mass_takeoff_kg / 0.45359237:.0f} lb)  L/D {result.lift_to_drag_cruise:.3f}")
    print("binding:", result.binding)


def load_cases():
    """Saved cases by label; of several solves of one free case from different starts, the lightest is kept under
    the case name and the others under "<label> (local)"."""
    cases = {path.stem: pickle.loads(path.read_bytes()) for path in sorted(path_results.glob("*.pkl"))}
    for case in ("free_stiff", "free"):
        labels = [k for k in cases if k == case or k.startswith(case + "_t")]
        if len(labels) > 1:
            best = min(labels, key=lambda k: cases[k].mass_takeoff_kg)
            results = {k: cases.pop(k) for k in labels}
            cases[case] = results[best]
            cases.update({f"{k} (local)": v for k, v in results.items() if k != best})
    return cases


def summarize(cases):
    """Per case: thickness, masses, L/D, drag, frequency margins, binding wing/whirl constraints."""
    from examples.halo_whirl_flutter import halo as whirl_halo
    requirements = HaloRequirements()
    stiffness = None
    if "t23" in cases:
        wing = wing_reference(cases["t23"], requirements, assumptions_for("t23"))["wing_afdd"]
        stiffness = (wing["stiffness_torsion_Nm2"], wing["stiffness_beam_Nm2"])
    rows = {}
    for case, result in cases.items():
        a = assumptions_for(case.split(" ")[0], stiffness)
        reference = wing_reference(result, requirements, a)
        wing_cd0, cd0 = cruise_drag(result, requirements, a)
        a_ = a
        torsion = min(w["torsion_per_rev"] for w in result.whirl_flutter) / a_.frequency_torsion_wing_per_rev - 1
        beam = min(w["beam_per_rev"] for w in result.whirl_flutter) / a_.frequency_beam_wing_per_rev - 1
        flutter_kt, mode, speed = whirl_halo(reference)
        rows[case] = dict(
            thickness=reference["thickness_to_chord_wing"], mass_takeoff_kg=result.mass_takeoff_kg,
            mass_wing_kg=reference["mass_wing_kg"], lift_to_drag=result.lift_to_drag_cruise,
            cd0_wing=wing_cd0, cd0=cd0, margin_torsion_per_rev=torsion, margin_beam_per_rev=beam,
            stiffness_torsion_Nm2=reference["wing_afdd"]["stiffness_torsion_Nm2"],
            stiffness_beam_Nm2=reference["wing_afdd"]["stiffness_beam_Nm2"],
            mass_spar_jump_kg=reference["wing_afdd"]["mass_spar_jump_kg"],
            mass_torque_box_kg=reference["wing_afdd"]["mass_torque_box_kg"],
            area_wing_m2=result.design.area_wing_m2, flutter_kt=flutter_kt, flutter_mode=mode,
            flutter_rotor_rad_s=speed, binding=[b for b in result.binding],
            required_flutter_kt=1.15 * 1.25 * requirements.velocity_max_m_s / 0.514444)
    return rows


def figure(rows, path=path_figure):
    """2 x 2 slide figure: take-off weight, coupled flutter speed (check), wing mass, wing profile drag."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fixed = sorted((r for c, r in rows.items() if c.startswith("t")), key=lambda r: r["thickness"])
    t = onp.array([r["thickness"] for r in fixed]) * 100
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), sharex=True)
    lb = 1 / 0.45359237
    panels = (("Take-off weight (lb)", lambda r: r["mass_takeoff_kg"] * lb),
              ("Whirl-flutter speed, plan 040 model, calibrated (kt)", lambda r: r["flutter_kt"]),
              ("Wing mass (kg)", lambda r: r["mass_wing_kg"]),
              ("Wing profile drag at cruise, CD0 x 10^4", lambda r: r["cd0_wing"] * 1e4))
    for ax, (label, f) in zip(axes.flat, panels):
        ax.plot(t, [f(r) for r in fixed], "-o", color="#2a6fb0", lw=2.5, ms=8,
                label="t/c fixed (frequency-placement margins)")
        if "free" in rows:
            r = rows["free"]
            ax.plot(r["thickness"] * 100, f(r), "*", color="#d9822b", ms=24, zorder=5,
                    label="t/c free, frequency margins: optimum 21.5 %")
        if "free_stiff" in rows:
            r = rows["free_stiff"]
            ax.plot(r["thickness"] * 100, f(r), "D", color="#3a8f3a", ms=13, zorder=6,
                    label="t/c free, GJ and EI floors at the 23 % wing (binding): optimum 23 %")
        ax.set_title(label, fontsize=16)
        ax.tick_params(labelsize=13)
        ax.grid(alpha=0.3)
    required_kt = fixed[0]["required_flutter_kt"]
    axes[0, 1].axhline(required_kt, color="#b03030", ls="--", lw=2)
    axes[0, 1].text(t[0], required_kt - 4, f"required 1.15 V_D = {required_kt:.0f} kt", color="#b03030",
                    fontsize=13, va="top")
    axes[0, 1].legend(*axes[0, 0].get_legend_handles_labels(), fontsize=12, loc="lower right")
    for ax in axes[1]:
        ax.set_xlabel("Wing thickness ratio t/c (%)", fontsize=15)
    r = rows.get("free")
    if r is not None:
        axes[0, 0].annotate("frequency margins slack; box at minimum gauge;\njump caps against section drag",
                            (r["thickness"] * 100, r["mass_takeoff_kg"] * lb), xytext=(15.2, 15290),
                            fontsize=12, arrowprops=dict(arrowstyle="->", color="0.4"))
    fig.suptitle("Halo wing thickness: under 0.1 % take-off weight at stake; thinning costs flutter speed",
                 fontsize=19)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.9, bottom=0.08, hspace=0.25, wspace=0.18)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)
    print("wrote", path)


def binding_label(row):
    """The wing-related constraints at their limit, else the thickness bound."""
    wing = [b for b in row["binding"] if "stiffness" in b or "whirl" in b or "stall" in b]
    return ", ".join(wing) if wing else "none of the wing margins (aerodynamics against wing mass only)"


if __name__ == "__main__":
    if sys.argv[1] == "solve":
        solve_case(*sys.argv[2:])
    elif sys.argv[1] == "report":
        rows = summarize(load_cases())
        print(json.dumps(rows, indent=1, default=str))
        (path_results / "summary.json").write_text(json.dumps(rows, indent=1, default=str))
        figure(rows)
