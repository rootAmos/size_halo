"""CalculiX check of the Halo wing box against the AFDD wing model (plan 031, deferred FE step).

Meshes the as-sized (constant-chord) wing box in OpenVSP, maps the AFDD
breakdown of `output/structure/reference.json` (from
`examples.halo_structure_reference`) to shell gauges, adds the rigid nacelles
with the tip mass and pylon inertia, and runs CalculiX: symmetric beam, chord
and torsion frequencies against the AFDD single-mode estimates, and the
ultimate jump take-off (tip deflection, largest spar-cap strain against the
AFDD strain allowable). Run from the repo root:
`python -m examples.halo_wing_fe`. Needs OpenVSP and CalculiX (`ccx`; set
`CCX` or install it at the default path below).
"""
import json
import os
from dataclasses import replace
from pathlib import Path

from aircraft_closure.export.openvsp.fe_wing import (FuselageMass, TipMass, properties_from_afdd, read_mesh,
                                                      read_modes, read_static, run_calculix, write_deck)
from aircraft_closure.export.openvsp.snapshot import SurfaceSnapshot, halo_plan027_snapshot
from aircraft_closure.export.openvsp.structure import StructureLayout, build_structure, export_structure_meshes

directory = Path("output/fe_wing")
path_ccx = Path(os.environ.get("CCX", "C:/Users/alexa/Documents/Xenon/Software/CalculiX/CalculiX-2.22.0-win-x64/bin/ccx.exe"))
ratio_radius_gyration_pylon = 0.222      # HaloAssumptions default (XV-15)
# Rest of the aircraft for the free-free modes (assumed): CG at the wing quarter chord, 1 m below the wing;
# pitch radius of gyration 0.3 of the 11 m fuselage, roll radius of gyration 0.6 m.
radius_gyration_pitch_fuselage_m, radius_gyration_roll_fuselage_m, drop_cg_m = 3.3, 0.6, 1.0
g_m_s2 = 9.80665

if __name__ == "__main__":
    reference = json.loads(Path("output/structure/reference.json").read_text())
    r, afdd, layout = reference, reference["wing_afdd"], StructureLayout()
    chord_m = r["area_wing_m2"] / r["span_wing_m"]
    wing = SurfaceSnapshot(span_m=r["span_wing_m"], chord_root_m=chord_m, chord_tip_m=chord_m,
                           x_le_root_m=r["x_le_wing_m"], z_m=1.2, thickness_to_chord=r["thickness_to_chord_wing"],
                           camber=0.02)
    snapshot = replace(halo_plan027_snapshot(), wing=wing)
    directory.mkdir(parents=True, exist_ok=True)
    geoms, structures = build_structure(snapshot, layout)
    paths = export_structure_meshes({"WingBox": structures["WingBox"]}, directory, kinds=("calculix",))
    nodes, elsets = read_mesh(paths[("WingBox", "calculix")])

    properties = properties_from_afdd(afdd, r["material_wing"], chord_m, r["thickness_to_chord_wing"],
                                      layout.fraction_chord_front_spar, layout.fraction_chord_rear_spar,
                                      r["span_wing_m"], r["width_fuselage_m"], width_cap_strip_fraction=0.10)
    radius_gyration_m = ratio_radius_gyration_pylon * r["radius_rotor_m"]
    tip = TipMass(mass_kg=r["mass_tip_kg"], radius_gyration_pitch_m=radius_gyration_m,
                  xyz_spindle_m=snapshot.spindle_xyz_m())
    weight_N = r["mass_takeoff_kg"] * g_m_s2
    force_jump_N = 1.5 * (r["load_factor_jump"] * weight_N / 2 - r["load_factor_jump"] * r["mass_tip_kg"] * g_m_s2)
    deck = dict(nodes=nodes, elsets=elsets, properties=properties, tip=tip, x_le_m=wing.x_le_root_m,
                chord_m=chord_m, fraction_front_spar=layout.fraction_chord_front_spar,
                fraction_rear_spar=layout.fraction_chord_rear_spar, width_fuselage_m=r["width_fuselage_m"])
    mass_wing_kg = afdd["mass_torque_box_kg"] + afdd["mass_spar_stiffness_kg"] + afdd["mass_spar_jump_kg"]         + afdd["mass_fairing_kg"]
    x_spindle_m, _, z_spindle_m = tip.xyz_spindle_m
    fuselage = FuselageMass(mass_kg=r["mass_takeoff_kg"] - 2 * r["mass_tip_kg"] - mass_wing_kg,
                            xyz_cg_m=(x_spindle_m, 0.0, z_spindle_m - drop_cg_m),
                            radius_gyration_pitch_m=radius_gyration_pitch_fuselage_m,
                            radius_gyration_roll_m=radius_gyration_roll_fuselage_m)
    path_modal, path_jump = directory / "halo_wing_modes.inp", directory / "halo_wing_jump.inp"
    references, groups = write_deck(path_modal, fuselage=fuselage, **deck)
    write_deck(path_jump, force_tip_jump_N=force_jump_N, **deck)
    modes = read_modes(run_calculix(path_modal, path_ccx), references, radius_gyration_m)
    deflection_m, strain_max = read_static(run_calculix(path_jump, path_ccx), references)

    print(f"Gauges: box walls {1e3 * properties.thickness_box_m:.2f} mm, cap pads +{1e3 * properties.thickness_cap_pad_m:.2f}"
          f" mm over {1e3 * properties.width_cap_strip_m:.0f} mm strips, ribs {1e3 * properties.thickness_rib_m:.1f} mm; "
          f"elements box {len(groups['BOX'])}, caps {len(groups['CAP'])}, fairing {len(groups['FAIRING'])}, "
          f"webs {len(groups['WEB'])}, ribs {len(groups['RIB'])}")
    print(f"Tip mass {tip.mass_kg:.0f} kg each, pitch radius of gyration {radius_gyration_m:.2f} m; fuselage body "
          f"{fuselage.mass_kg:.0f} kg (free-free modes)")
    print("Modes (rad/s): " + ", ".join(f"{m.frequency_rad_s:.1f} {m.kind}{'' if m.symmetric else ' (anti)'}"
                                        for m in modes))
    rows = []
    for kind, key in (("beam", "frequency_beam_rad_s"), ("chord", "frequency_chord_rad_s"),
                      ("torsion", "frequency_torsion_rad_s")):
        symmetric = [m.frequency_rad_s for m in modes if m.kind == kind and m.symmetric]
        fe = symmetric[0] if symmetric else float("nan")
        rows.append((kind, fe, afdd[key]))
        print(f"  symmetric {kind:<8} FE {fe:7.1f} rad/s   AFDD {afdd[key]:7.1f} rad/s   ratio {fe / afdd[key]:.2f}")
    allowable = r["material_wing"]["strain_ultimate"]
    print(f"Jump take-off (ultimate, {force_jump_N / 1e3:.1f} kN up at each spindle): tip deflection "
          f"{deflection_m:.3f} m, spanwise cap strain 0.3-1.0 m outboard of the clamp {strain_max:.5f} vs AFDD "
          f"allowable {allowable:.4f} "
          f"({strain_max / allowable:.2f})")
    (directory / "summary.json").write_text(json.dumps(dict(
        modes=[dict(frequency_rad_s=m.frequency_rad_s, kind=m.kind, symmetric=m.symmetric) for m in modes],
        comparison=[dict(kind=k, fe_rad_s=fe, afdd_rad_s=a) for k, fe, a in rows],
        deflection_tip_jump_m=deflection_m, strain_max_jump=strain_max, strain_allowable=allowable,
        thickness_box_m=properties.thickness_box_m, thickness_cap_pad_m=properties.thickness_cap_pad_m), indent=1))
