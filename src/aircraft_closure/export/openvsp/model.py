"""OpenVSP outer mold line of a tiltrotor from a numeric `GeometrySnapshot`.

The model tree follows the author-supplied tiltrotor skeleton (plan 031):

    Fuselage (smoothly skinned Fuselage geom)
    |- WingFairing (dorsal fairing, optional)
    |- Wing
    |  `- NacelleTilt (Hinge about y at the conversion spindle)
    |     `- Nacelle (Fuselage geom)
    |        `- Rotor (Propeller)
    |           `- RotorTipPath (Auxiliary)
    `- HorizontalTail and VerticalTail, or VTail

Units are metres. OpenVSP is imported here only; nothing in `vehicle/` or the
sizing code imports this module.
"""
import math

import openvsp as vsp

from aircraft_closure.export.openvsp.snapshot import BodySnapshot

# Propeller blade curve indices (OpenVSP 3.53.1): chord c/R and twist in degrees over r/R.
_pcurve_chord = 0
_pcurve_twist = 1


def _set(geom_id, name, group, value):
    if not vsp.SetParmVal(geom_id, name, group, value) and vsp.FindParm(geom_id, name, group) == "":
        raise KeyError(f"OpenVSP parameter {group}:{name} not found on {vsp.GetGeomName(geom_id)}.")


def _set_xsec(xsec_id, name, value):
    parm_id = vsp.GetXSecParm(xsec_id, name)
    if parm_id == "":
        raise KeyError(f"OpenVSP cross-section parameter {name} not found.")
    vsp.SetParmVal(parm_id, value)


def _attach_to_parent(geom_id, x_rel_m=0.0, y_rel_m=0.0, z_rel_m=0.0):
    _set(geom_id, "Trans_Attach_Flag", "Attach", vsp.ATTACH_TRANS_COMP)
    _set(geom_id, "Rots_Attach_Flag", "Attach", vsp.ATTACH_ROT_COMP)
    _set(geom_id, "X_Rel_Location", "XForm", x_rel_m)
    _set(geom_id, "Y_Rel_Location", "XForm", y_rel_m)
    _set(geom_id, "Z_Rel_Location", "XForm", z_rel_m)


def _place_absolute(geom_id, x_m, y_m, z_m, x_rotation_deg=0.0):
    """Position in the global frame (no attachment to the parent)."""
    _set(geom_id, "Trans_Attach_Flag", "Attach", vsp.ATTACH_TRANS_NONE)
    _set(geom_id, "Rots_Attach_Flag", "Attach", vsp.ATTACH_ROT_NONE)
    _set(geom_id, "X_Rel_Location", "XForm", x_m)
    _set(geom_id, "Y_Rel_Location", "XForm", y_m)
    _set(geom_id, "Z_Rel_Location", "XForm", z_m)
    _set(geom_id, "X_Rel_Rotation", "XForm", x_rotation_deg)


def _slope_deg(x_m, values, index):
    """Tangent angle of a station profile: weighted central difference inside, one-sided at the ends."""
    if index == 0:
        slope = (values[1] - values[0]) / (x_m[1] - x_m[0])
    elif index == len(values) - 1:
        slope = (values[-1] - values[-2]) / (x_m[-1] - x_m[-2])
    else:
        dx_before_m, dx_after_m = x_m[index] - x_m[index - 1], x_m[index + 1] - x_m[index]
        slope_before = (values[index] - values[index - 1]) / dx_before_m
        slope_after = (values[index + 1] - values[index]) / dx_after_m
        slope = (slope_before * dx_after_m + slope_after * dx_before_m) / (dx_before_m + dx_after_m)
    return math.degrees(math.atan(slope))


def _add_body(name, body, parent_id=None, tessellation_section=6):
    """Smoothly skinned body (OpenVSP Fuselage geom, super-ellipse sections) with round end caps.

    Each section's skinning tangents follow the local slopes of the top line,
    bottom line and half-width line (positive opens outward), so the surface
    flows through the stations instead of flattening at each one.
    """
    stations = body.stations
    geom_id = vsp.AddGeom("FUSELAGE", parent_id) if parent_id else vsp.AddGeom("FUSELAGE")
    vsp.SetGeomName(geom_id, name)
    x_m = [station.x_m for station in stations]
    length_m = x_m[-1] - x_m[0]
    _set(geom_id, "Length", "Design", length_m)
    surface_id = vsp.GetXSecSurf(geom_id, 0)
    while vsp.GetNumXSec(surface_id) > len(stations):
        vsp.CutXSec(geom_id, vsp.GetNumXSec(surface_id) - 2)
    while vsp.GetNumXSec(surface_id) < len(stations):
        vsp.InsertXSec(geom_id, vsp.GetNumXSec(surface_id) - 2, vsp.XS_ROUNDED_RECTANGLE)
    z_top_m = [station.z_m + station.height_m / 2 for station in stations]
    z_bottom_down_m = [-(station.z_m - station.height_m / 2) for station in stations]
    half_width_m = [station.width_m / 2 for station in stations]
    # Positions are clamped between neighbours, so sweep until they settle.
    for _ in range(3):
        for index, station in enumerate(stations):
            xsec_id = vsp.GetXSec(surface_id, index)
            _set_xsec(xsec_id, "XLocPercent", (station.x_m - x_m[0]) / length_m)
            _set_xsec(xsec_id, "ZLocPercent", (station.z_m - stations[0].z_m) / length_m)
    for index, station in enumerate(stations):
        vsp.ChangeXSecShape(surface_id, index, vsp.XS_SUPER_ELLIPSE)
        xsec_id = vsp.GetXSec(surface_id, index)
        exponent_bottom = station.exponent if station.exponent_bottom is None else station.exponent_bottom
        _set_xsec(xsec_id, "Super_TopBotSym", 0)
        for name_parm, value in (("Super_Width", station.width_m), ("Super_Height", station.height_m),
                                 ("Super_M", station.exponent), ("Super_N", station.exponent),
                                 ("Super_M_bot", exponent_bottom), ("Super_N_bot", exponent_bottom)):
            _set_xsec(xsec_id, name_parm, value)
        _set_xsec(xsec_id, "SectTess_U", tessellation_section)
        _set_xsec(xsec_id, "AllSym", 0)
        _set_xsec(xsec_id, "TBSym", 0)
        _set_xsec(xsec_id, "RLSym", 1)
        angle_side_deg = _slope_deg(x_m, half_width_m, index)
        vsp.SetXSecTanAngles(xsec_id, vsp.XSEC_BOTH_SIDES, _slope_deg(x_m, z_top_m, index), angle_side_deg,
                             _slope_deg(x_m, z_bottom_down_m, index), angle_side_deg)
    for option in ("CapUMinOption", "CapUMaxOption"):
        _set(geom_id, option, "EndCap", vsp.ROUND_END_CAP)
    _set(geom_id, "Tess_W", "Shape", 41)
    _place_absolute(geom_id, body.x_nose_m + x_m[0], 0.0, stations[0].z_m)
    return geom_id


def _add_surface(name, surface, parent_id, symmetric=True, vertical=False):
    s = surface
    geom_id = vsp.AddGeom("WING", parent_id)
    vsp.SetGeomName(geom_id, name)
    _place_absolute(geom_id, s.x_le_root_m, 0.0, s.z_m, x_rotation_deg=90.0 if vertical else 0.0)
    _set(geom_id, "Sym_Planar_Flag", "Sym", vsp.SYM_XZ if symmetric else 0)
    span_panel_m = s.span_m / 2 if symmetric else s.span_m
    for name_parm, value in (("Span", span_panel_m), ("Root_Chord", s.chord_root_m), ("Tip_Chord", s.chord_tip_m),
                             ("Sweep", s.sweep_deg), ("Sweep_Location", s.fraction_chord_sweep),
                             ("Dihedral", s.dihedral_deg), ("SectTess_U", 12)):
        _set(geom_id, name_parm, "XSec_1", value)
    _set(geom_id, "Tess_W", "Shape", 33)
    surface_id = vsp.GetXSecSurf(geom_id, 0)
    for index in range(vsp.GetNumXSec(surface_id)):
        vsp.ChangeXSecShape(surface_id, index, vsp.XS_FOUR_SERIES)
        xsec_id = vsp.GetXSec(surface_id, index)
        _set_xsec(xsec_id, "ThickChord", s.thickness_to_chord)
        _set_xsec(xsec_id, "Camber", s.camber)
        _set_xsec(xsec_id, "CamberLoc", s.camber_location)
    return geom_id


def _add_nacelle_group(snapshot, wing_id, angle_nacelle_deg):
    n, r = snapshot.nacelle, snapshot.rotor
    x_spindle_m, y_spindle_m, z_spindle_m = snapshot.spindle_xyz_m()

    hinge_id = vsp.AddGeom("HINGE", wing_id)
    vsp.SetGeomName(hinge_id, "NacelleTilt")
    _place_absolute(hinge_id, x_spindle_m, y_spindle_m, z_spindle_m)
    # Hinge axis along +y, as in the skeleton (primary direction y, secondary z).
    for name_parm, value in (("PrimaryDir", 1), ("SecondaryDir", 2), ("PrimXVec", 0.0), ("PrimYVec", 1.0),
                             ("PrimZVec", 0.0), ("JointRotateFlag", 1)):
        _set(hinge_id, name_parm, "Hinge", value)
    _set(hinge_id, "JointRotate", "Hinge", angle_nacelle_deg)

    nacelle_id = _add_body("Nacelle", BodySnapshot(stations=n.stations()), hinge_id)
    _attach_to_parent(nacelle_id, x_rel_m=-n.length_mast_m - n.offset_nose_m)

    rotor_id = vsp.AddGeom("PROP", nacelle_id)
    vsp.SetGeomName(rotor_id, "Rotor")
    _attach_to_parent(rotor_id, x_rel_m=n.offset_nose_m)
    _set(rotor_id, "Diameter", "Design", 2 * r.radius_m)
    _set(rotor_id, "NumBlade", "Design", r.count_blades)
    _set(rotor_id, "Precone", "Design", r.angle_precone_deg)
    _set(rotor_id, "Beta34", "Design", r.angle_collective_deg)
    # Blade curves over r/R: constant chord c/R = solidity pi / blades; linear twist about 0.75 R.
    fraction_radius_root = 0.2
    chord_over_radius = r.solidity * math.pi / r.count_blades
    vsp.SetPCurve(rotor_id, _pcurve_chord, [fraction_radius_root, 1.0], [chord_over_radius] * 2, vsp.LINEAR)
    stations = [fraction_radius_root, 0.75, 1.0]
    twist_deg = [r.angle_twist_deg * (station - 0.75) / (1.0 - fraction_radius_root) for station in stations]
    vsp.SetPCurve(rotor_id, _pcurve_twist, stations, twist_deg, vsp.LINEAR)

    tip_path_id = vsp.AddGeom("AUXILIARY", rotor_id)
    vsp.SetGeomName(tip_path_id, "RotorTipPath")
    _set(tip_path_id, "AuxiliaryGeomType", "Design", vsp.AUX_GEOM_ROTOR_TIP_PATH)
    _attach_to_parent(tip_path_id)
    # A hinge has no surface, so each child is mirrored itself, about the global x-z plane (ancestor 0).
    for geom_id in (nacelle_id, rotor_id, tip_path_id):
        _set(geom_id, "Sym_Planar_Flag", "Sym", vsp.SYM_XZ)
        _set(geom_id, "Sym_Ancestor", "Sym", 0)
    return dict(NacelleTilt=hinge_id, Nacelle=nacelle_id, Rotor=rotor_id, RotorTipPath=tip_path_id)


# User sets: the wetted outer mold line (exports, CompGeom), clearance envelopes (checks only), and the
# airframe / rotor split of the outer mold line (renders).
set_outer_mold_line = vsp.SET_FIRST_USER
set_clearance = vsp.SET_FIRST_USER + 1
set_airframe = vsp.SET_FIRST_USER + 2
set_rotors = vsp.SET_FIRST_USER + 3


def build_openvsp_model(snapshot, angle_nacelle_deg=90.0):
    """Clear the OpenVSP model and build the outer mold line; returns {name: geom id}."""
    vsp.VSPRenew()
    vsp.SetSetName(set_outer_mold_line, "OuterMoldLine")
    vsp.SetSetName(set_clearance, "Clearance")
    vsp.SetSetName(set_airframe, "Airframe")
    vsp.SetSetName(set_rotors, "Rotors")
    fuselage_id = _add_body("Fuselage", snapshot.fuselage)
    wing_id = _add_surface("Wing", snapshot.wing, fuselage_id)
    geoms = dict(Fuselage=fuselage_id, Wing=wing_id)
    if snapshot.wing_fairing is not None:
        geoms["WingFairing"] = _add_body("WingFairing", snapshot.wing_fairing, fuselage_id)
    if snapshot.horizontal_tail is not None:
        geoms["HorizontalTail"] = _add_surface("HorizontalTail", snapshot.horizontal_tail, fuselage_id)
    if snapshot.vertical_tail is not None:
        geoms["VerticalTail"] = _add_surface("VerticalTail", snapshot.vertical_tail, fuselage_id, symmetric=False,
                                             vertical=True)
    if snapshot.v_tail is not None:
        geoms["VTail"] = _add_surface("VTail", snapshot.v_tail, fuselage_id)
    geoms.update(_add_nacelle_group(snapshot, wing_id, angle_nacelle_deg))
    for name, geom_id in geoms.items():
        if name == "RotorTipPath":
            vsp.SetSetFlag(geom_id, set_clearance, True)
        elif name != "NacelleTilt":
            vsp.SetSetFlag(geom_id, set_outer_mold_line, True)
            vsp.SetSetFlag(geom_id, set_rotors if name == "Rotor" else set_airframe, True)
    vsp.Update()
    return geoms


# Nacelle angles of the saved Modes (XV-15 convention: 90 deg helicopter mode).
angles_nacelle_modes_deg = (("hover", 90.0), ("conversion", 45.0), ("cruise", 0.0))


def add_nacelle_modes(geoms):
    """OpenVSP Modes (variable presets on the hinge angle) for use in the GUI; returns {name: mode id}."""
    parm_id = vsp.FindParm(geoms["NacelleTilt"], "JointRotate", "Hinge")
    group_id = vsp.AddVarPresetGroup("NacelleAngle")
    vsp.AddVarPresetParm(group_id, parm_id)
    modes = {}
    for name, angle_nacelle_deg in angles_nacelle_modes_deg:
        setting_id = vsp.AddVarPresetSetting(group_id, name)
        vsp.SetVarPresetParmVal(group_id, setting_id, parm_id, angle_nacelle_deg)
        modes[name] = vsp.CreateAndAddMode(name, set_outer_mold_line, vsp.SET_NONE)
        vsp.ModeAddGroupSetting(modes[name], group_id, setting_id)
    return modes


def export_outer_mold_line(snapshot, directory, name="halo", angles_nacelle_deg=(90.0, 45.0, 0.0)):
    """Write `<name>.vsp3` (hover, with Modes) and STEP and STL files per nacelle angle; returns the paths.

    OpenVSP 3.53.1 keeps the first export's tessellation of the hinge children
    when only the hinge angle changes (also through Modes), so each angle is
    exported from a fresh build.
    """
    from pathlib import Path

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for angle_nacelle_deg in angles_nacelle_deg:
        build_openvsp_model(snapshot, angle_nacelle_deg)
        tag = f"{name}_nacelle{angle_nacelle_deg:02.0f}"
        for kind, set_index, file_type, suffix in (
                ("stl", set_outer_mold_line, vsp.EXPORT_STL, ".stl"), ("step", set_outer_mold_line, vsp.EXPORT_STEP, ".stp"),
                ("stl_airframe", set_airframe, vsp.EXPORT_STL, "_airframe.stl"),
                ("stl_rotors", set_rotors, vsp.EXPORT_STL, "_rotors.stl")):
            path = directory / f"{tag}{suffix}"
            vsp.ExportFile(str(path), set_index, file_type)
            paths[(kind, angle_nacelle_deg)] = path
    geoms = build_openvsp_model(snapshot, 90.0)
    add_nacelle_modes(geoms)
    paths["vsp3"] = directory / f"{name}.vsp3"
    vsp.WriteVSPFile(str(paths["vsp3"]))
    return paths
