"""CalculiX check of the bottom-up wing box layout (plan 054).

Reads `output/structure/wing_box_layout.json` (`examples.halo_wing_box`), builds the OpenVSP wing box at the sized
rib pitch, gives every element the sized gauges of its spanwise band (orthotropic covers: skin plus smeared
stringers spanwise, skin only chordwise and in shear; cap pads over each spar), and runs the ultimate jump
take-off and the free-free symmetric modes. Reports the root spar-cap strain as a margin against the allowable
and the FE frequencies against the bottom-up predictions. Run from the repo root:
`python -m examples.halo_wing_box_fe`. Needs OpenVSP and CalculiX (`ccx`; set `CCX` or the default path below).
"""
import json
import os
from dataclasses import replace
from pathlib import Path

from aircraft_closure.export.openvsp.fe_wing import (FuselageMass, TipMass, bands_from_wing_box, read_mesh,
                                                      locate_buckling_modes, read_buckling, read_modes, read_static, read_twist,
                                                      run_calculix,
                                                      write_deck)
from aircraft_closure.weights.afdd import wing_tiltrotor_mode_inertias
from aircraft_closure.export.openvsp.snapshot import SurfaceSnapshot, halo_plan027_snapshot
from aircraft_closure.export.openvsp.structure import StructureLayout, build_structure, export_structure_meshes

directory = Path("output/fe_wing_box")
path_layout = Path("output/structure/wing_box_layout.json")
path_ccx = Path(os.environ.get("CCX", "C:/Users/alexa/Documents/Xenon/Software/CalculiX/CalculiX-2.22.0-win-x64/bin/ccx.exe"))
g_m_s2 = 9.80665
# Rest of the aircraft for the free-free modes, as in `examples.halo_wing_fe` (assumed).
radius_gyration_pitch_fuselage_m, radius_gyration_roll_fuselage_m, drop_cg_m = 3.3, 0.6, 1.0

if __name__ == "__main__":
    r = json.loads(path_layout.read_text())
    chord_m = r["chord_wing_m"]
    wing = SurfaceSnapshot(span_m=r["span_wing_m"], chord_root_m=chord_m, chord_tip_m=chord_m,
                           x_le_root_m=r["x_le_wing_m"], z_m=1.2, thickness_to_chord=r["thickness_to_chord_wing"],
                           camber=r.get("camber_wing", 0.02),
                           section_name=None if r.get("section_wing", "naca2423") == "naca2423" else r["section_wing"])
    snapshot = replace(halo_plan027_snapshot(), wing=wing)
    layout = replace(StructureLayout(), fraction_chord_front_spar=r["fraction_chord_front_spar"],
                     fraction_chord_rear_spar=r["fraction_chord_rear_spar"], pitch_rib_wing_m=r["pitch_rib_m"],
                     ribs_between_attach_ribs=True)
    directory.mkdir(parents=True, exist_ok=True)
    geoms, structures = build_structure(snapshot, layout)
    paths = export_structure_meshes({"WingBox": structures["WingBox"]}, directory, kinds=("calculix",))
    nodes, elsets = read_mesh(paths[("WingBox", "calculix")])

    bands = bands_from_wing_box(r)
    tip = TipMass(mass_kg=r["mass_tip_kg"], radius_gyration_pitch_m=r["radius_gyration_pylon_m"],
                  xyz_spindle_m=snapshot.spindle_xyz_m())
    weight_N = r["mass_takeoff_kg"] * g_m_s2
    force_jump_N = 1.5 * r["load_factor_jump"] * (weight_N / 2 - r["mass_tip_kg"] * g_m_s2)
    deck = dict(nodes=nodes, elsets=elsets, properties=bands, tip=tip, x_le_m=wing.x_le_root_m, chord_m=chord_m,
                fraction_front_spar=layout.fraction_chord_front_spar,
                fraction_rear_spar=layout.fraction_chord_rear_spar, width_fuselage_m=r["width_fuselage_m"])
    x_spindle_m, _, z_spindle_m = tip.xyz_spindle_m
    fuselage = FuselageMass(mass_kg=r["mass_takeoff_kg"] - 2 * r["mass_tip_kg"] - r["mass_primary_kg"]
                            - r["mass_fairing_kg"], xyz_cg_m=(x_spindle_m, 0.0, z_spindle_m - drop_cg_m),
                            radius_gyration_pitch_m=radius_gyration_pitch_fuselage_m,
                            radius_gyration_roll_m=radius_gyration_roll_fuselage_m)
    path_modal, path_jump = directory / "wing_box_modes.inp", directory / "wing_box_jump.inp"
    references, groups = write_deck(path_jump, force_tip_jump_N=force_jump_N, **deck)
    deflection_m, strain_max = read_static(run_calculix(path_jump, path_ccx), references)
    # Realized torsion stiffness: a pure tip torque on the clamped box, GJ = T L / twist over the panel length.
    torque_Nm = 10e3
    path_torsion = directory / "wing_box_torsion.inp"
    write_deck(path_torsion, force_tip_jump_N=0.0, torque_tip_Nm=torque_Nm, **deck)
    twist_rad = read_twist(run_calculix(path_torsion, path_ccx), references)
    length_panel_m = r["span_wing_m"] / 2 - r["width_fuselage_m"] / 2
    stiffness_torsion_fe_Nm2 = torque_Nm * length_panel_m / abs(twist_rad)
    _, torsion_inertia_kg_m3 = wing_tiltrotor_mode_inertias(r["span_wing_m"], r["mass_tip_kg"],
                                                             r["radius_gyration_pylon_m"], r["mass_takeoff_kg"])
    # Modes with plain orthotropic covers: the layered covers (stringer-web core) add a spurious soft mechanism
    # to the free-free dynamics (CalculiX expands offset layered shells to solids); they stay for the static,
    # torque and buckling cases, where the stringers' lever arm matters.
    deck_modes = dict(deck, properties=replace(bands, pitch_stringer_upper_m=None, height_stringer_upper_m=None,
                                                  thickness_core_web_m=None))
    write_deck(path_modal, fuselage=fuselage, count_modes=90, **deck_modes)
    modes = read_modes(run_calculix(path_modal, path_ccx), references, r["radius_gyration_pylon_m"])
    # Root boundary: the same modes with an immovable fuselage (the NDARC single-mode relations assume a clamp).
    path_clamped = directory / "wing_box_modes_clamped.inp"
    write_deck(path_clamped, fuselage=replace(fuselage, mass_kg=1e9, radius_gyration_pitch_m=1e3,
                                              radius_gyration_roll_m=1e3), count_modes=90, **deck_modes)
    modes_clamped = read_modes(run_calculix(path_clamped, path_ccx), references, r["radius_gyration_pylon_m"])
    # Panel stability: linear buckling of the stiffened upper covers under the ultimate jump take-off.
    path_buckle = directory / "wing_box_buckle.inp"
    write_deck(path_buckle, force_tip_jump_N=force_jump_N, count_buckling_modes=5, **deck)
    path_buckle_dat = run_calculix(path_buckle, path_ccx)
    factors_buckling = read_buckling(path_buckle_dat)
    groups_of_node = {}
    connectivity = {e: c for _, (kind, elements) in elsets.items() for e, c in elements}
    for group, ids in groups.items():
        for element_id in ids:
            for n in connectivity[element_id]:
                groups_of_node.setdefault(n, set()).add(group)
    located_buckling = locate_buckling_modes(path_buckle_dat, nodes, groups_of_node)

    allowable = r["material"]["strain_tension"]
    print(f"Layout: rib pitch {r['pitch_rib_m']:.2f} m, rib {1e3 * r['thickness_rib_m']:.2f} mm; root upper skin "
          f"{1e3 * r['thickness_skin_upper_m'][0]:.2f} + stringers {1e3 * r['thickness_stringers_upper_m'][0]:.2f} mm, "
          f"web {1e3 * r['thickness_web_m'][0]:.2f} mm (core {1e3 * r.get('thickness_core_web_m', [0])[0]:.1f} mm), caps {1e4 * r['area_caps_m2'][0]:.1f} cm2; elements box "
          f"{len(groups['BOX'])}, caps {len(groups['CAP'])}, webs {len(groups['WEB'])}, ribs {len(groups['RIB'])}")
    margin_fe = 1 - strain_max / allowable
    margin_sizing = 1 - r["strain_root_lower_cover"] / allowable
    print(f"Jump take-off (ultimate, {force_jump_N / 1e3:.1f} kN up at each spindle): tip deflection {deflection_m:.3f} m;"
          f" root cap strain FE {strain_max:.5f} (margin {100 * margin_fe:+.1f} %), sizing {r['strain_root_lower_cover']:.5f}"
          f" (margin {100 * margin_sizing:+.1f} %), allowable {allowable:.4f}")
    print(f"Torsion: FE GJ {stiffness_torsion_fe_Nm2 / 1e6:.2f} MN m2 against bottom-up {r['stiffness_torsion_Nm2'] / 1e6:.2f}"
          f" ({stiffness_torsion_fe_Nm2 / r['stiffness_torsion_Nm2']:.2f}); NDARC single-mode frequency from the FE GJ "
          f"{(stiffness_torsion_fe_Nm2 / torsion_inertia_kg_m3) ** 0.5:.1f} rad/s")
    rows = []
    for kind, key in (("beam", "frequency_beam_rad_s"), ("chord", "frequency_chord_rad_s"),
                      ("torsion", "frequency_torsion_rad_s")):
        symmetric = [m.frequency_rad_s for m in modes if m.kind == kind and m.symmetric]
        fe = symmetric[0] if symmetric else float("nan")
        clamped = [m.frequency_rad_s for m in modes_clamped if m.kind == kind and m.symmetric]
        fe_clamped = clamped[0] if clamped else float("nan")
        rows.append(dict(kind=kind, fe_rad_s=fe, fe_clamped_rad_s=fe_clamped, sizing_rad_s=r[key]))
        print(f"  symmetric {kind:<8} FE free-free {fe:7.1f}  clamped {fe_clamped:7.1f} rad/s   bottom-up "
              f"{r[key]:7.1f} rad/s   ratio {fe / r[key]:.2f}")
    for factor, (node, xyz, where) in zip(factors_buckling, located_buckling):
        print(f"  buckling factor {factor:.2f}: peak at node {node}, x {xyz[0]:.2f} y {xyz[1]:.2f} z {xyz[2]:.2f} m, "
              f"in {', '.join(where)}")
    knockdown = r["material"].get("knockdown_modulus_environment", 1.0)
    print(f"Hot/wet (modulus x {knockdown:.2f}, buckling load proportional): first factor "
          f"{knockdown * factors_buckling[0]:.2f}; webs {', '.join(f'{knockdown * f:.2f}' for f, (_, _, w) in zip(factors_buckling, located_buckling) if 'WEB' in w) or 'none in the first modes'}")
    print(f"Buckling under the ultimate jump take-off: factors {', '.join(f'{f:.2f}' for f in factors_buckling[:3])}"
          f" (margin {100 * (factors_buckling[0] - 1) if factors_buckling else float('nan'):+.0f} % on the first)")
    (directory / "summary.json").write_text(json.dumps(dict(
        deflection_tip_jump_m=deflection_m, strain_max_jump=strain_max, strain_sizing=r["strain_root_lower_cover"],
        strain_allowable=allowable, factors_buckling=factors_buckling, stiffness_torsion_fe_Nm2=stiffness_torsion_fe_Nm2,
        stiffness_torsion_sizing_Nm2=r["stiffness_torsion_Nm2"], margin_fe=margin_fe, margin_sizing=margin_sizing, comparison=rows,
        modes=[dict(frequency_rad_s=m.frequency_rad_s, kind=m.kind, symmetric=m.symmetric) for m in modes]),
        indent=1))
