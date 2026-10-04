"""Back-check of the sizing's primary-structure weights against the drawn layout (plan 031).

Reads `output/structure/reference.json` (from `examples.halo_structure_reference`)
and the OpenVSP structure STLs in `output/structure/` (from
`examples.halo_structure`), sizes gauges from simple ultimate loads and the
AFDD stiffness requirements on the layout, and compares the primary-structure
masses with what the sizing uses (AFDD wing, Raymer tails and fuselage).
Run from the repo root: `python -m examples.halo_structure_check`.
"""
import json
import math
from pathlib import Path

import aerosandbox as asb
import aerosandbox.tools.units as u
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot, v_tail_equivalent
from aircraft_closure.export.openvsp.structure import StructureLayout
from aircraft_closure.export.openvsp.structure_check import (BoxMaterial, FuselageMaterial, SurfaceLoads,
                                                             area_by_part_m2, elliptic_panel_loads,
                                                             fuselage_secondary_items, size_box, size_fuselage,
                                                             torsion_stiffness_min_mass_Nm2)
from examples.xv15_reference import calibration_factors

directory = Path("output/structure")
factor_safety = 1.5
count_stations = 61
# Fuselage secondary-structure quantities (assumed, from the Halo stills and the OpenVSP model).
area_doors_m2 = 2 * 2.0 * 1.3            # one large side cargo door each side
area_fairings_m2 = 12.0                  # exposed dorsal-fairing area (OpenVSP parasite-drag tool, plan 031)
area_access_panels_m2 = 4.0              # nose-bay access panels and sensor radome
fraction_fasteners_paint = 0.06          # fasteners, sealant, paint on primary + secondary structure


def wing_check(reference, layout, taper_ratio, area_ribs_m2, torsion="uniform"):
    """Layout wing box over both panels; the box runs through over the fuselage (constant moment inboard).

    `torsion`: "uniform" applies the AFDD torsion stiffness everywhere; "min_mass" keeps its tip twist
    flexibility with GJ proportional to chord (the lightest distribution for a tapered box).
    """
    r = reference
    material_afdd = r["material_wing"]
    material = BoxMaterial(density_box_kg_m3=material_afdd["density_torque_box_kg_m3"],
                           modulus_box_Pa=material_afdd["modulus_torque_box_Pa"],
                           modulus_shear_box_Pa=material_afdd["modulus_shear_torque_box_Pa"],
                           density_cap_kg_m3=material_afdd["density_spar_kg_m3"],
                           modulus_cap_Pa=material_afdd["modulus_spar_Pa"],
                           strain_ultimate=material_afdd["strain_ultimate"])
    semispan_m = r["span_wing_m"] / 2
    chord_root_m = 2 * r["area_wing_m2"] / (r["span_wing_m"] * (1 + taper_ratio))
    y_m = np.linspace(0.0, semispan_m, count_stations)
    chord_m = chord_root_m * (1 - (1 - taper_ratio) * y_m / semispan_m)
    weight_N = r["mass_takeoff_kg"] * 9.80665
    # Flight: ultimate load factor (already includes 1.5), elliptic lift, tip-mass inertia relief.
    moment_flight_Nm, shear_flight_N = elliptic_panel_loads(
        y_m, semispan_m, r["load_factor_ultimate"] * weight_N / 2,
        point_loads=((semispan_m, -r["load_factor_ultimate"] * r["mass_tip_kg"] * 9.80665),))
    # Jump take-off (AFDD's case): rotor thrust at the tip less tip inertia, times 1.5.
    thrust_N = r["load_factor_jump"] * weight_N / 2
    moment_jump_Nm, shear_jump_N = elliptic_panel_loads(
        y_m, semispan_m, 0.0, point_loads=((semispan_m, factor_safety * (
            thrust_N - r["load_factor_jump"] * r["mass_tip_kg"] * 9.80665)),))
    y_side_m = r["width_fuselage_m"] / 2
    inboard = y_m < y_side_m
    for array in (moment_flight_Nm, moment_jump_Nm):
        array[inboard] = np.interp(y_side_m, y_m, array)
    for array in (shear_flight_N, shear_jump_N):
        array[inboard] = 0.0
    jump_governs = np.abs(moment_jump_Nm) >= np.abs(moment_flight_Nm)
    loads = SurfaceLoads(y_m=y_m, moment_Nm=np.where(jump_governs, moment_jump_Nm, moment_flight_Nm),
                         shear_N=np.where(jump_governs, shear_jump_N, shear_flight_N),
                         case=tuple("jump" if j else "flight" for j in jump_governs))
    afdd = r["wing_afdd"]
    stiffness_torsion_Nm2 = (afdd["stiffness_torsion_Nm2"] if torsion == "uniform" else
                             torsion_stiffness_min_mass_Nm2(y_m, chord_m, afdd["stiffness_torsion_Nm2"], y_side_m))
    box = size_box(y_m, chord_m, r["thickness_to_chord_wing"], layout.fraction_chord_front_spar,
                   layout.fraction_chord_rear_spar, loads, material,
                   stiffness_torsion_Nm2=stiffness_torsion_Nm2, stiffness_beam_Nm2=afdd["stiffness_beam_Nm2"],
                   stiffness_chord_Nm2=afdd["stiffness_chord_Nm2"], area_ribs_m2=area_ribs_m2)
    return box, loads, material


def v_tail_check(reference, material, area_ribs_m2, velocity_max_m_s, coefficient_lift_max=1.0):
    """V-tail panels at the drawn sizing rule; ultimate normal force at CL max and dive speed (1.25 V max)."""
    tail = v_tail_equivalent(reference["area_horizontal_tail_m2"], reference["area_vertical_tail_m2"],
                             aspect_ratio=3.27, taper_ratio=0.5, sweep_le_deg=40.0, x_le_root_m=0.0, z_m=0.0,
                             thickness_to_chord=0.12)
    semispan_m = tail.span_m / 2
    area_panel_m2 = 0.5 * (tail.chord_root_m + tail.chord_tip_m) * semispan_m
    pressure_dynamic_Pa = 0.5 * asb.Atmosphere(altitude=0.0).density() * (1.25 * velocity_max_m_s) ** 2
    force_N = factor_safety * coefficient_lift_max * pressure_dynamic_Pa * area_panel_m2
    y_m = np.linspace(0.0, semispan_m, count_stations)
    chord_m = tail.chord_root_m * (1 - 0.5 * y_m / semispan_m)
    moment_Nm, shear_N = elliptic_panel_loads(y_m, semispan_m, force_N)
    loads = SurfaceLoads(y_m=y_m, moment_Nm=moment_Nm, shear_N=shear_N, case=("tail lift",) * len(y_m))
    box = size_box(y_m, chord_m, 0.12, 0.20, 0.65, loads, material, area_ribs_m2=area_ribs_m2)
    return box, float(force_N), tail


def section_perimeter_m(width_m, height_m, exponent):
    angle = np.linspace(0.0, 2 * math.pi, 361)
    x = 0.5 * width_m * np.sign(np.cos(angle)) * np.abs(np.cos(angle)) ** (2 / exponent)
    z = 0.5 * height_m * np.sign(np.sin(angle)) * np.abs(np.sin(angle)) ** (2 / exponent)
    return float(np.sum(np.hypot(np.diff(x), np.diff(z))))


def fuselage_check(reference, snapshot, layout, force_tail_N, dihedral_deg):
    stations = snapshot.fuselage.stations
    x_stations_m = np.array([s.x_m for s in stations])
    length_m = x_stations_m[-1]
    x_frames_m = np.arange(layout.pitch_frame_m, length_m - 0.3 + 1e-9, layout.pitch_frame_m)
    length_frames_m = sum(section_perimeter_m(np.interp(x, x_stations_m, [s.width_m for s in stations]),
                                              np.interp(x, x_stations_m, [s.height_m for s in stations]),
                                              np.interp(x, x_stations_m, [s.exponent for s in stations]))
                          for x in x_frames_m)
    area_section_frame_m2 = (2 * layout.width_flange_frame_m * layout.thickness_frame_m
                             + (layout.depth_frame_m - 2 * layout.thickness_frame_m) * layout.thickness_frame_m)
    path = directory / "Fuselage.stl"
    area_skin_m2 = area_by_part_m2(path, "Skin")
    area_bulkheads_m2 = sum(area_by_part_m2(path, name) for name in
                            ("NoseBulkhead", "FrontSparBulkhead", "RearSparBulkhead", "CabinEndBulkhead"))
    area_floor_m2 = area_by_part_m2(path, "Floor")
    # Bending at the wing rear spar: tail lift (vertical part of both panels) and aft-body inertia at the
    # ultimate load factor; the aft fuselage is its length share of the Raymer fuselage mass at mid-arm.
    wing = snapshot.wing
    x_rear_spar_m = wing.x_le_root_m + layout.fraction_chord_rear_spar * wing.chord_root_m
    x_tail_m = snapshot.v_tail.x_le_root_m + 0.4 * snapshot.v_tail.chord_root_m
    arm_tail_m = x_tail_m - x_rear_spar_m
    mass_tails_kg = reference["mass_horizontal_tail_kg"] + reference["mass_vertical_tail_kg"]
    mass_aft_fuselage_kg = reference["mass_fuselage_kg"] * (length_m - x_rear_spar_m) / length_m
    moment_Nm = (arm_tail_m * (2 * force_tail_N * math.cos(math.radians(dihedral_deg))
                               + reference["load_factor_ultimate"] * mass_tails_kg * 9.80665)
                 + 0.5 * (length_m - x_rear_spar_m) * reference["load_factor_ultimate"] * mass_aft_fuselage_kg
                 * 9.80665)
    width_m = max(s.width_m for s in stations)
    height_m = max(s.height_m for s in stations)
    sizing = size_fuselage(area_skin_m2, length_frames_m, area_section_frame_m2, area_bulkheads_m2, area_floor_m2,
                           width_m, height_m, moment_Nm, FuselageMaterial())
    return sizing, dict(area_skin_m2=area_skin_m2, length_frames_m=length_frames_m, count_frames=len(x_frames_m),
                        area_bulkheads_m2=area_bulkheads_m2, area_floor_m2=area_floor_m2)


def plot(rows, path_png):
    labels = [row[0] for row in rows]
    x = np.arange(len(rows))
    figure, axis = plt.subplots(figsize=(10, 5))
    width = 0.38
    axis.bar(x - width / 2, [row[1] for row in rows], width - 0.02, color="#2a78d6", label="Sizing correlation")
    axis.bar(x + width / 2, [row[2] for row in rows], width - 0.02, color="#eb6834", label="Layout-based estimate")
    for index, row in enumerate(rows):
        axis.text(index + width / 2, row[2], f"{row[2] / row[1]:.2f}x", ha="center", va="bottom", fontsize=9,
                  color="#333333")
    axis.set_xticks(x, labels)
    axis.set_ylabel("Mass (kg)")
    axis.set_title("Halo primary structure: sizing correlations vs layout-based estimate\n"
                   "wing: primary structure; fuselage: primary + secondary (layout) vs the Raymer group",
                   fontsize=11)
    axis.grid(True, axis="y", color="#e5e5e2")
    axis.set_axisbelow(True)
    for spine in ("top", "right"):
        axis.spines[spine].set_visible(False)
    axis.legend(frameon=False)
    figure.tight_layout()
    figure.savefig(path_png, dpi=120)
    plt.close(figure)


if __name__ == "__main__":
    reference = json.loads((directory / "reference.json").read_text())
    snapshot, layout = halo_plan027_snapshot(), StructureLayout()
    area_ribs_wing_m2 = sum(area_by_part_m2(directory / "WingBox.stl", name)
                            for name in ("Ribs", "FairingAttachRib", "NacelleRib"))
    area_ribs_tail_m2 = area_by_part_m2(directory / "VTail.stl", "Ribs")

    afdd = reference["wing_afdd"]
    mass_afdd_primary_kg = afdd["mass_torque_box_kg"] + afdd["mass_spar_stiffness_kg"] + afdd["mass_spar_jump_kg"]
    print(f"Design mass {reference['mass_takeoff_kg']:.0f} kg, n_ult {reference['load_factor_ultimate']}, "
          f"jump n {reference['load_factor_jump']}, tip mass {reference['mass_tip_kg']:.0f} kg")
    print(f"AFDD wing: torque box {afdd['mass_torque_box_kg']:.1f}, spar (stiffness) {afdd['mass_spar_stiffness_kg']:.1f},"
          f" spar (jump) {afdd['mass_spar_jump_kg']:.1f} -> primary {mass_afdd_primary_kg:.1f} kg; "
          f"fairing {afdd['mass_fairing_kg']:.1f}, fittings {afdd['mass_fittings_kg']:.1f}; "
          f"sized wing (calibrated) {reference['mass_wing_kg']:.1f} kg; AFDD jump M_ult "
          f"{afdd['moment_jump_ultimate_Nm'] / 1e3:.0f} kN m")
    results = {}
    for name, taper, torsion in (("as sized (constant chord)", 1.0, "uniform"),
                                 ("as drawn (taper 0.6), uniform GJ", 0.6, "uniform"),
                                 ("as drawn (taper 0.6), GJ ~ chord", 0.6, "min_mass")):
        box, loads, material = wing_check(reference, layout, taper, area_ribs_wing_m2, torsion)
        results[name] = box
        index_side = np.searchsorted(box.y_m, reference["width_fuselage_m"] / 2)
        print(f"\nWing {name}: root M {loads.moment_Nm[0] / 1e3:.0f} kN m ({loads.case[0]}), "
              f"skin {1e3 * box.thickness_skin_m[0]:.2f}-{1e3 * box.thickness_skin_m[-1]:.2f} mm, "
              f"web {1e3 * box.thickness_web_m[index_side]:.2f} mm at the fuselage side, caps at root "
              f"{1e4 * box.area_caps_m2[0]:.1f} cm2 ({box.driver_caps[0]})")
        print(f"  skins {box.mass_skins_kg:.1f}, webs {box.mass_webs_kg:.1f}, caps {box.mass_caps_kg:.1f}, ribs "
              f"{box.mass_ribs_kg:.1f} -> primary {box.mass_primary_kg():.1f} kg "
              f"({box.mass_primary_kg() / mass_afdd_primary_kg:.2f} x AFDD primary)")

    tail_box, force_tail_N, tail = v_tail_check(reference, material, area_ribs_tail_m2, 210 * u.knot)
    mass_tails_raymer_kg = reference["mass_horizontal_tail_kg"] + reference["mass_vertical_tail_kg"]
    print(f"\nV-tail: panel ultimate normal force {force_tail_N / 1e3:.1f} kN, skin "
          f"{1e3 * tail_box.thickness_skin_m[0]:.2f} mm, caps at root {1e4 * tail_box.area_caps_m2[0]:.2f} cm2 "
          f"-> primary {tail_box.mass_primary_kg():.1f} kg vs Raymer tails {mass_tails_raymer_kg:.1f} kg")

    fuselage, measured = fuselage_check(reference, snapshot, layout, force_tail_N, tail.dihedral_deg)
    print(f"\nFuselage: bending M_ult {fuselage.moment_bending_ultimate_Nm / 1e3:.0f} kN m -> skin "
          f"{1e3 * fuselage.thickness_skin_bending_m:.2f} mm needed, {1e3 * fuselage.thickness_skin_m:.2f} mm used; "
          f"skin {measured['area_skin_m2']:.1f} m2, {measured['count_frames']} frames "
          f"({measured['length_frames_m']:.1f} m), bulkheads {measured['area_bulkheads_m2']:.1f} m2, floor "
          f"{measured['area_floor_m2']:.1f} m2")
    print(f"  skin {fuselage.mass_skin_kg:.1f}, stringers {fuselage.mass_stringers_kg:.1f}, frames "
          f"{fuselage.mass_frames_kg:.1f}, bulkheads {fuselage.mass_bulkheads_kg:.1f}, floor {fuselage.mass_floor_kg:.1f}"
          f" -> primary {fuselage.mass_primary_kg():.1f} kg vs Raymer fuselage {reference['mass_fuselage_kg']:.1f} kg")

    factors = calibration_factors()
    mass_fuselage_raymer_raw_kg = reference["mass_fuselage_kg"] / factors.fuselage
    items = fuselage_secondary_items(area_doors_m2, afdd["mass_fittings_kg"], area_fairings_m2, area_access_panels_m2,
                                     measured["area_floor_m2"])
    mass_secondary_kg = sum(item.mass_kg() for item in items)
    mass_fasteners_kg = fraction_fasteners_paint * (fuselage.mass_primary_kg() + mass_secondary_kg)
    mass_fuselage_layout_kg = fuselage.mass_primary_kg() + mass_secondary_kg + mass_fasteners_kg
    print("\nFuselage secondary structure (layout, assumed unit masses):")
    for item in items:
        print(f"  {item.name:<38}{item.quantity:7.1f} {item.unit:<3} x {item.unit_mass_kg:5.1f} kg/{item.unit:<3}"
              f"= {item.mass_kg():6.1f} kg  ({item.basis})")
    print(f"  {'fasteners, sealant, paint':<38}{100 * fraction_fasteners_paint:7.1f} %{'':24}= {mass_fasteners_kg:6.1f} kg")
    print(f"Fuselage layout total {mass_fuselage_layout_kg:.1f} kg (primary {fuselage.mass_primary_kg():.1f} + "
          f"secondary {mass_secondary_kg + mass_fasteners_kg:.1f}) vs Raymer GA unpressurized "
          f"{mass_fuselage_raymer_raw_kg:.1f} kg, x {factors.fuselage:.2f} XV-15 calibration = "
          f"{reference['mass_fuselage_kg']:.1f} kg")

    rows = [("Wing, as sized\n(vs AFDD primary)", mass_afdd_primary_kg,
             results["as sized (constant chord)"].mass_primary_kg()),
            ("Wing, as drawn\n(GJ ~ chord)", mass_afdd_primary_kg,
             results["as drawn (taper 0.6), GJ ~ chord"].mass_primary_kg()),
            ("Fuselage\n(vs raw Raymer)", mass_fuselage_raymer_raw_kg, mass_fuselage_layout_kg),
            ("Fuselage\n(vs XV-15-calibrated)", reference["mass_fuselage_kg"], mass_fuselage_layout_kg)]
    plot(rows, directory / "structure_check.png")
