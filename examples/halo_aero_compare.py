"""Halo cruise aerodynamics: VSPAERO and the OpenVSP parasite-drag tool against AeroSandbox (plan 031).

Both tools analyse the same drawn geometry (airplane mode, no rotors) at the 210 kt / 10,000 ft speed
requirement. The geometry is the current reference drawn as Halo (`halo_reference_snapshot` from
`output/structure/reference.json`, written by `examples.halo_structure_reference`); `--plan027` uses the
plan 031 snapshot instead.

- lift curve and pitching moment: VSPAERO thin / mixed / panel, AeroSandbox vortex lattice (lifting
  surfaces only), lifting line (NeuralFoil sections, bodies from AeroBuildup) and AeroBuildup;
- induced drag: VSPAERO thin (Trefftz plane), AeroSandbox vortex lattice and AeroBuildup;
- total drag polar: AeroBuildup against the lifting line (the two viscous methods);
- profile drag: OpenVSP parasite-drag build-up against AeroBuildup, by component.

Run from the repo root: `python -m examples.halo_aero_compare`. Outputs go to
`output/aero_compare/`. Needs OpenVSP and matplotlib.
"""
import csv
import json
import sys
from pathlib import Path

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from aircraft_closure.export.openvsp.aero import run_parasite_drag, run_vspaero_sweep
from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot, halo_reference_snapshot

directory_output = Path("output/aero_compare")
altitude_m, velocity_m_s = 10000 * u.foot, 210 * u.knot
alpha_deg = np.arange(-2.0, 10.01, 2.0)
range_fit_deg = (-2.0, 6.0)     # linear range for slopes

# Fixed source order and colours (dataviz reference palette, light; validated 2026-10-04).
sources = (("VSPAERO thin (VLM)", "#2a78d6", "o"), ("VSPAERO mixed (VLM + panel bodies)", "#eb6834", "s"),
           ("VSPAERO panel", "#1baf7a", "^"), ("AeroSandbox VLM", "#eda100", "D"),
           ("AeroSandbox AeroBuildup", "#e87ba4", "v"), ("AeroSandbox lifting line", "#7d52c7", "P"))
colour_parasite_tool = "#008300"


def aerosandbox_polars(airplane, op_point, xyz_ref_m):
    """AeroBuildup and the lifting line on the whole airplane, the vortex lattice on its lifting surfaces, per alpha."""
    buildup = asb.AeroBuildup(airplane=airplane, op_point=op_point, xyz_ref=list(xyz_ref_m)).run()
    dynamic_pressure_Pa = op_point.dynamic_pressure()
    wings_only = asb.Airplane(wings=airplane.wings, s_ref=airplane.s_ref, c_ref=airplane.c_ref, b_ref=airplane.b_ref)
    vlm = [asb.VortexLatticeMethod(airplane=wings_only, op_point=op_point[index], xyz_ref=list(xyz_ref_m),
                                   spanwise_resolution=12, chordwise_resolution=8).run()
           for index in range(len(op_point.alpha))]
    lifting_line = [asb.LiftingLine(airplane=airplane, op_point=op_point[index], xyz_ref=list(xyz_ref_m),
                                    spanwise_resolution=8).run()
                    for index in range(len(op_point.alpha))]
    return (dict(cl=np.array(buildup["CL"]), cm=np.array(buildup["Cm"]), cd=np.array(buildup["CD"]),
                 cdi=np.array(buildup["D_induced"]) / (dynamic_pressure_Pa * airplane.s_ref),
                 cd_profile=np.array(buildup["D_profile"]) / (dynamic_pressure_Pa * airplane.s_ref)),
            dict(cl=np.array([r["CL"] for r in vlm]), cm=np.array([r["Cm"] for r in vlm]),
                 cdi=np.array([r["CD"] for r in vlm])),
            dict(cl=np.array([float(r["CL"]) for r in lifting_line]),
                 cm=np.array([float(r["Cm"]) for r in lifting_line]),
                 cd=np.array([float(r["CD"]) for r in lifting_line])))


def aerosandbox_profile_by_component(airplane, op_point_zero):
    """AeroBuildup profile drag coefficient of each component alone (alpha 0)."""
    dynamic_pressure_Pa = op_point_zero.dynamic_pressure()
    parts = {}
    for wing in airplane.wings:
        parts[wing.name] = asb.Airplane(wings=[wing], s_ref=airplane.s_ref, c_ref=airplane.c_ref, b_ref=airplane.b_ref)
    for fuselage in airplane.fuselages:
        parts[fuselage.name] = asb.Airplane(fuselages=[fuselage], s_ref=airplane.s_ref, c_ref=airplane.c_ref,
                                            b_ref=airplane.b_ref)
    # Wings return a one-element array, bodies a scalar.
    return {name: np.asarray(asb.AeroBuildup(airplane=part, op_point=op_point_zero).run()["D_profile"]).item()
            / (dynamic_pressure_Pa * airplane.s_ref) for name, part in parts.items()}


def slopes(alpha_deg, cl, cm, cdi=None, aspect_ratio=None):
    """Linear-range CL_alpha (per deg), CL at 0 deg, dCm/dCL, and Oswald e from CDi = CL^2 / (pi AR e)."""
    mask = (alpha_deg >= range_fit_deg[0] - 1e-9) & (alpha_deg <= range_fit_deg[1] + 1e-9)
    cl_alpha_per_deg, cl_zero = np.polyfit(alpha_deg[mask], cl[mask], 1)
    dcm_dcl = np.polyfit(cl[mask], cm[mask], 1)[0]
    oswald = None
    if cdi is not None:
        k = np.polyfit(cl[mask] ** 2, cdi[mask], 1)[0]
        oswald = 1 / (np.pi * aspect_ratio * k)
    return dict(cl_alpha_per_deg=cl_alpha_per_deg, cl_zero=cl_zero, dcm_dcl=dcm_dcl, oswald=oswald)


def plot(results, metrics, parasite, profile_asb, path_png, title_geometry):
    figure, axes = plt.subplots(2, 3, figsize=(19, 9.5))
    ax_cl, ax_cm, ax_table, ax_cdi, ax_cd, ax_cd0 = axes.ravel()
    for (label, colour, marker) in sources:
        r = results[label]
        style = dict(color=colour, marker=marker, markersize=6, linewidth=2, label=label)
        ax_cl.plot(r["alpha_deg"], r["cl"], **style)
        ax_cm.plot(r["alpha_deg"], r["cm"], **style)
        if r.get("cdi") is not None:
            ax_cdi.plot(r["cl"], r["cdi"], **style)
        if r.get("cd") is not None:
            ax_cd.plot(r["cl"], r["cd"], **style)
    ax_cl.set(xlabel="alpha (deg)", ylabel="CL", title="Lift curve")
    ax_cm.set(xlabel="alpha (deg)", ylabel="Cm about wing MAC quarter chord", title="Pitching moment")
    ax_cdi.set(xlabel="CL", ylabel="CDi", title="Induced drag (Trefftz / VLM / AeroBuildup)")
    ax_cd.set(xlabel="CL", ylabel="CD", title="Total drag polar, viscous methods (no sizing corrections)")

    rows = [[label.replace("AeroSandbox ", "ASB ").replace(" (VLM + panel bodies)", ""),
             f"{m['cl_alpha_per_deg']:.4f}", f"{m['cl_zero']:.3f}", f"{m['dcm_dcl']:.3f}",
             f"{m['x_neutral_point_m']:.2f}", "" if m["oswald"] is None else f"{m['oswald']:.2f}"]
            for label, m in metrics.items()]
    ax_table.axis("off")
    table = ax_table.table(cellText=rows, colLabels=["source", "CLa /deg", "CL0", "dCm/dCL", "x_np m", "e"],
                           colWidths=[0.34, 0.14, 0.11, 0.14, 0.13, 0.08], loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.6)
    ax_table.set_title("Fit over alpha -2 to 6 deg (e from CDi; lifting line: from total CD)")

    names = list(parasite["labels"])
    x = np.arange(len(names))
    width = 0.38
    ax_cd0.bar(x - width / 2, 1e4 * np.array(parasite["cd0"]), width - 0.02, color=colour_parasite_tool,
               label="OpenVSP parasite-drag tool")
    ax_cd0.bar(x + width / 2, [1e4 * profile_asb[name] for name in names], width - 0.02, color=sources[4][1],
               label="AeroBuildup profile drag")
    ax_cd0.set_xticks(x, names)
    ax_cd0.set(ylabel="CD0 (counts)", title="Profile drag by component, alpha 0")
    for ax in (ax_cl, ax_cm, ax_cdi, ax_cd, ax_cd0):
        ax.grid(True, color="#e5e5e2", linewidth=0.8)
        ax.set_axisbelow(True)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        ax.legend(frameon=False, fontsize=8)
    figure.suptitle(f"Halo cruise aerodynamics, 210 kt / 10,000 ft: OpenVSP vs AeroSandbox ({title_geometry})")
    figure.tight_layout()
    figure.savefig(path_png, dpi=120)
    plt.close(figure)


if __name__ == "__main__":
    directory_output.mkdir(parents=True, exist_ok=True)
    if "--plan027" in sys.argv:
        snapshot, name_geometry, title_geometry = halo_plan027_snapshot(), "plan027", "plan 027 snapshot"
    else:
        reference = json.loads(Path("output/structure/reference.json").read_text())
        snapshot, name_geometry = halo_reference_snapshot(reference), "reference"
        title_geometry = f"current reference, {reference['mass_takeoff_kg'] / u.lbm:,.0f} lb"
    airplane = snapshot.to_asb()
    wing = snapshot.wing
    x_quarter_mac_m = wing.x_le_root_m + 0.25 * wing.chord_root_m   # quarter-chord line is unswept
    xyz_ref_m = (x_quarter_mac_m, 0.0, wing.z_m)
    atmosphere = asb.Atmosphere(altitude=altitude_m)
    op_point = asb.OperatingPoint(atmosphere=atmosphere, velocity=velocity_m_s, alpha=alpha_deg)
    mach = float(op_point.mach())
    reynolds_cref = float(op_point.reynolds(airplane.c_ref))
    aspect_ratio = wing.span_m ** 2 / airplane.s_ref

    # VSPAERO takes minutes; `--reuse-vspaero` reads the previous run's polars instead.
    path_cache = directory_output / f"vspaero_polars_{name_geometry}.json"
    if "--reuse-vspaero" in sys.argv and path_cache.exists():
        results = {label: {key: (None if value is None else np.array(value)) for key, value in r.items()}
                   for label, r in json.loads(path_cache.read_text()).items()}
    else:
        results = {}
        for (label, _, _), model in zip(sources[:3], ("thin", "mixed", "panel")):
            polar = run_vspaero_sweep(snapshot, directory_output / "vspaero", model, alpha_deg, mach,
                                      reynolds_cref, xyz_ref_m)
            results[label] = dict(alpha_deg=polar.alpha_deg, cl=polar.cl, cm=polar.cm,
                                  cdi=polar.cdi_wake if model == "thin" else None)
        path_cache.write_text(json.dumps({label: {key: (None if value is None else list(map(float, value)))
                                                  for key, value in r.items()} for label, r in results.items()}))
    buildup, vlm, lifting_line = aerosandbox_polars(airplane, op_point, xyz_ref_m)
    results[sources[3][0]] = dict(alpha_deg=alpha_deg, **vlm)
    results[sources[4][0]] = dict(alpha_deg=alpha_deg, cl=buildup["cl"], cm=buildup["cm"], cdi=buildup["cdi"],
                                  cd=buildup["cd"])
    results[sources[5][0]] = dict(alpha_deg=alpha_deg, **lifting_line)

    parasite_tool = run_parasite_drag(snapshot, altitude_m, velocity_m_s)
    labels_asb = dict(Fuselage="fuselage", Wing="wing", Nacelle=("nacelle_right", "nacelle_left"),
                      WingFairing="wing_fairing", VTail="v_tail")
    profile_by_part = aerosandbox_profile_by_component(
        airplane, asb.OperatingPoint(atmosphere=atmosphere, velocity=velocity_m_s, alpha=0.0))
    profile_asb = {label: sum(profile_by_part[name] for name in (names if isinstance(names, tuple) else (names,)))
                   for label, names in labels_asb.items()}
    parasite = dict(labels=parasite_tool.labels, cd0=parasite_tool.cd0)

    metrics = {}
    print(f"Mach {mach:.3f}, Re(MAC) {reynolds_cref:.3g}, reference x {x_quarter_mac_m:.3f} m, "
          f"MAC {airplane.c_ref:.3f} m, AR {aspect_ratio:.2f}")
    print(f"{'source':<38}{'CLa /deg':>9}{'CL0':>8}{'dCm/dCL':>9}{'x_np m':>8}{'e':>7}")
    for label, _, _ in sources:
        r = results[label]
        cd_fit = r.get("cdi") if r.get("cdi") is not None else r.get("cd")
        m = slopes(np.asarray(r["alpha_deg"]), np.asarray(r["cl"]), np.asarray(r["cm"]),
                   None if cd_fit is None else np.asarray(cd_fit), aspect_ratio)
        m["x_neutral_point_m"] = x_quarter_mac_m - m["dcm_dcl"] * airplane.c_ref
        metrics[label] = m
        oswald = "" if m["oswald"] is None else f"{m['oswald']:.3f}"
        print(f"{label:<38}{m['cl_alpha_per_deg']:9.4f}{m['cl_zero']:8.3f}{m['dcm_dcl']:9.3f}"
              f"{m['x_neutral_point_m']:8.3f}{oswald:>7}")
    print(f"\n{'component':<14}{'Swet m2 (VSP)':>14}{'FF (VSP)':>10}{'CD0 VSP':>10}{'CD0 ASB':>10}")
    for index, label in enumerate(parasite_tool.labels):
        print(f"{label:<14}{parasite_tool.area_wetted_m2[index]:14.2f}{parasite_tool.form_factor[index]:10.3f}"
              f"{parasite_tool.cd0[index]:10.5f}{profile_asb[label]:10.5f}")
    print(f"{'total':<14}{'':>14}{'':>10}{parasite_tool.cd0_total:10.5f}{sum(profile_asb.values()):10.5f}")
    print(f"AeroBuildup whole-airplane profile CD at alpha 0: {float(np.interp(0.0, alpha_deg, buildup['cd_profile'])):.5f}")

    with open(directory_output / "polars.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["source", "alpha_deg", "CL", "Cm", "CDi", "CD"])
        for label, _, _ in sources:
            r = results[label]
            for index, alpha in enumerate(r["alpha_deg"]):
                writer.writerow([label, alpha, r["cl"][index], r["cm"][index],
                                 "" if r.get("cdi") is None else r["cdi"][index],
                                 "" if r.get("cd") is None else r["cd"][index]])
    plot(results, metrics, parasite, profile_asb, directory_output / f"aero_compare_{name_geometry}.png",
         title_geometry)
