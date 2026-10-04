"""OpenVSP internal structure (FEA structures) of a `GeometrySnapshot`, meshed and exported (plan 031, milestone C).

Structures remain mass estimation in this framework; this layout is for CAD
(STEP), for later finite-element checks (CalculiX / Nastran decks) and for
packaging. Nothing here feeds the sizing.

Layout (first pass, placeholder gauges):
- Wing (right half; the left is its mirror): front and rear spars, a rib
  array, attach ribs at the dorsal-fairing sides and a nacelle rib at the
  spindle, upper and lower skins.
- Fuselage: a 1.0 m frame array, bulkheads at the nose bay, both wing-spar stations
  and the cabin end, a cabin floor, skin.
- V-tail (one panel): two spars and a rib array, skin.
"""
from dataclasses import dataclass
from pathlib import Path

import openvsp as vsp

from aircraft_closure.export.openvsp.model import build_openvsp_model


@dataclass(frozen=True)
class StructureLayout:
    """Placement of the internal structure; lengths in metres, chord stations as fractions."""
    fraction_chord_front_spar: float = 0.15
    fraction_chord_rear_spar: float = 0.60
    pitch_rib_wing_m: float = 0.5
    pitch_frame_m: float = 1.0          # meshing cost grows with each frame (about 10-60 s each in OpenVSP 3.53.1)
    height_floor_above_bottom_m: float = 0.35
    x_bulkhead_nose_m: float = 1.6
    x_cabin_end_m: float = 5.06          # end of the constant cabin (0.46 of the 11 m fuselage)
    pitch_rib_tail_m: float = 0.4
    thickness_skin_m: float = 0.002      # placeholder gauges until the AFDD wing masses are mapped
    thickness_spar_web_m: float = 0.004
    thickness_rib_m: float = 0.002
    thickness_frame_m: float = 0.0015
    thickness_floor_m: float = 0.003


def _parm(container_id, name, group, value):
    parm_id = vsp.FindParm(container_id, name, group)
    if parm_id == "":
        raise KeyError(f"OpenVSP FEA parameter {group}:{name} not found.")
    vsp.SetParmVal(parm_id, value)


def _material(name, density_kg_m3, modulus_Pa, poisson):
    material_id = vsp.AddFeaMaterial()
    vsp.SetParmVal(vsp.FindParm(material_id, "MassDensity", "FeaMaterial"), density_kg_m3)
    vsp.SetParmVal(vsp.FindParm(material_id, "ElasticModulus", "FeaMaterial"), modulus_Pa)
    vsp.SetParmVal(vsp.FindParm(material_id, "PoissonRatio", "FeaMaterial"), poisson)
    vsp.SetParmContainerName(material_id, name) if hasattr(vsp, "SetParmContainerName") else None
    return len(vsp.GetFeaMaterialIDVec()) - 1


def _property(name, index_material, thickness_m):
    property_id = vsp.AddFeaProperty()
    vsp.SetParmVal(vsp.FindParm(property_id, "FeaMaterialIndex", "FeaProperty"), index_material)
    vsp.SetParmVal(vsp.FindParm(property_id, "Thickness", "FeaProperty"), thickness_m)
    return len(vsp.GetFeaPropertyIDVec()) - 1


def _part(geom_id, index_struct, part_type, name, index_property, **parms):
    part_id = vsp.AddFeaPart(geom_id, index_struct, part_type)
    vsp.SetFeaPartName(part_id, name)
    _parm(part_id, "IncludedElements", "FeaPart", vsp.FEA_SHELL)
    _parm(part_id, "FeaPropertyIndex", "FeaPart", index_property)
    for key, value in parms.items():
        group, name_parm = key.split("__")
        _parm(part_id, name_parm, group, value)
    return part_id


def _skin(geom_id, index_struct, index_property):
    struct_id = vsp.GetFeaStructID(geom_id, index_struct)
    skin_id = vsp.GetFeaPartIDVec(struct_id)[0]
    _parm(skin_id, "FeaPropertyIndex", "FeaPart", index_property)
    return skin_id


def build_structure(snapshot, layout=StructureLayout()):
    """Build the outer mold line (airplane mode) and its FEA structures; returns (geoms, {name: struct id})."""
    geoms = build_openvsp_model(snapshot, 0.0)
    vehicle_id = vsp.FindContainer("Vehicle", 0)
    vsp.SetParmVal(vsp.FindParm(vehicle_id, "StructUnit", "FeaStructure"), vsp.SI_UNIT)
    # Placeholder materials: quasi-isotropic carbon for the lifting surfaces, 7075 aluminium for the fuselage.
    carbon = _material("CarbonQuasiIsotropic", 1600.0, 60e9, 0.30)
    aluminium = _material("Aluminium7075", 2810.0, 71.7e9, 0.33)
    skin_carbon = _property("SkinCarbon", carbon, layout.thickness_skin_m)
    web = _property("SparWeb", carbon, layout.thickness_spar_web_m)
    rib = _property("Rib", carbon, layout.thickness_rib_m)
    skin_aluminium = _property("SkinAluminium", aluminium, layout.thickness_skin_m)
    frame = _property("Frame", aluminium, layout.thickness_frame_m)
    floor = _property("Floor", aluminium, layout.thickness_floor_m)
    structures = {}

    # ---- Wing box (right half) ---------------------------------------------------------
    wing_id = geoms["Wing"]
    index = vsp.AddFeaStruct(wing_id, True, 0)
    vsp.SetFeaStructName(wing_id, index, "WingBox")
    _skin(wing_id, index, skin_carbon)
    semispan_m = snapshot.wing.span_m / 2
    for name, fraction in (("FrontSpar", layout.fraction_chord_front_spar),
                           ("RearSpar", layout.fraction_chord_rear_spar)):
        _part(wing_id, index, vsp.FEA_SPAR, name, web, FeaPart__RelCenterLocation=fraction)
    _part(wing_id, index, vsp.FEA_RIB_ARRAY, "Ribs", rib, FeaRibArray__RibAbsSpacing=layout.pitch_rib_wing_m,
          FeaPart__AbsRelParmFlag=vsp.ABS, FeaRibArray__RelStartLocation=0.0, FeaRibArray__RelEndLocation=1.0)
    half_width_fairing_m = 0.5 * max(station.width_m for station in snapshot.wing_fairing.stations) \
        if snapshot.wing_fairing is not None else 0.0
    _part(wing_id, index, vsp.FEA_RIB, "FairingAttachRib", rib, FeaPart__AbsRelParmFlag=vsp.ABS,
          FeaPart__AbsCenterLocation=half_width_fairing_m)
    _part(wing_id, index, vsp.FEA_RIB, "NacelleRib", rib, FeaPart__AbsRelParmFlag=vsp.ABS,
          FeaPart__AbsCenterLocation=semispan_m - 0.5 * snapshot.nacelle.width_m)
    structures["WingBox"] = vsp.GetFeaStructID(wing_id, index)

    # ---- Fuselage ----------------------------------------------------------------------
    fuselage_id = geoms["Fuselage"]
    index = vsp.AddFeaStruct(fuselage_id, True, 0)
    vsp.SetFeaStructName(fuselage_id, index, "Fuselage")
    _skin(fuselage_id, index, skin_aluminium)
    length_m = snapshot.fuselage.stations[-1].x_m - snapshot.fuselage.stations[0].x_m
    _part(fuselage_id, index, vsp.FEA_SLICE_ARRAY, "Frames", frame, FeaSliceArray__OrientationPlane=vsp.YZ_BODY,
          FeaPart__AbsRelParmFlag=vsp.ABS, FeaSliceArray__SliceAbsSpacing=layout.pitch_frame_m,
          FeaSliceArray__AbsStartLocation=layout.pitch_frame_m, FeaSliceArray__AbsEndLocation=length_m - 0.3)
    wing = snapshot.wing
    x_root_spars_m = [wing.x_le_root_m + fraction * wing.chord_root_m
                      for fraction in (layout.fraction_chord_front_spar, layout.fraction_chord_rear_spar)]
    for name, x_m in (("NoseBulkhead", layout.x_bulkhead_nose_m), ("FrontSparBulkhead", x_root_spars_m[0]),
                      ("RearSparBulkhead", x_root_spars_m[1]), ("CabinEndBulkhead", layout.x_cabin_end_m)):
        _part(fuselage_id, index, vsp.FEA_SLICE, name, frame, FeaSlice__OrientationPlane=vsp.YZ_BODY,
              FeaPart__AbsRelParmFlag=vsp.ABS, FeaPart__AbsCenterLocation=x_m)
    _part(fuselage_id, index, vsp.FEA_SLICE, "Floor", floor, FeaSlice__OrientationPlane=vsp.XY_BODY,
          FeaPart__AbsRelParmFlag=vsp.ABS, FeaPart__AbsCenterLocation=layout.height_floor_above_bottom_m)
    structures["Fuselage"] = vsp.GetFeaStructID(fuselage_id, index)

    # ---- V-tail (right panel) ------------------------------------------------------------
    if "VTail" in geoms:
        tail_id = geoms["VTail"]
        index = vsp.AddFeaStruct(tail_id, True, 0)
        vsp.SetFeaStructName(tail_id, index, "VTail")
        _skin(tail_id, index, skin_carbon)
        for name, fraction in (("FrontSpar", 0.2), ("RearSpar", 0.65)):
            _part(tail_id, index, vsp.FEA_SPAR, name, web, FeaPart__RelCenterLocation=fraction)
        _part(tail_id, index, vsp.FEA_RIB_ARRAY, "Ribs", rib, FeaRibArray__RibAbsSpacing=layout.pitch_rib_tail_m,
              FeaPart__AbsRelParmFlag=vsp.ABS, FeaRibArray__RelStartLocation=0.0, FeaRibArray__RelEndLocation=1.0)
        structures["VTail"] = vsp.GetFeaStructID(tail_id, index)
    vsp.Update()
    return geoms, structures


def export_structure_meshes(structures, directory, length_max_m=0.15, length_min_m=0.03,
                            kinds=("stl", "calculix", "nastran", "mass")):
    """Mesh each structure and write the requested files; returns {(structure, kind): path}.

    `kinds`: any of "stl", "calculix", "nastran", "mass", "step". Each kind is a
    separate `ComputeFeaMesh` call, which re-meshes (OpenVSP 3.53.1 writes one
    file type per call, whatever the export flags), so ask only for what is
    needed: the fuselage takes minutes per kind. The "FeaMeshAnalysis" wrapper,
    which writes several files from one mesh, did not finish on this model, and
    the structural STEP export stalled on the wing box.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    all_kinds = (("stl", vsp.FEA_STL_FILE_NAME, "stl"), ("calculix", vsp.FEA_CALCULIX_FILE_NAME, "inp"),
                 ("nastran", vsp.FEA_NASTRAN_FILE_NAME, "dat"), ("mass", vsp.FEA_MASS_FILE_NAME, "txt"),
                 ("step", vsp.FEA_STEP_FILE_NAME, "stp"))
    file_kinds = tuple(kind for kind in all_kinds if kind[0] in kinds)
    paths = {}
    for name, struct_id in structures.items():
        geom_id, index = vsp.GetFeaStructParentGeomID(struct_id), vsp.GetFeaStructIndex(struct_id)
        vsp.SetFeaMeshVal(geom_id, index, vsp.CFD_MAX_EDGE_LEN, length_max_m)
        vsp.SetFeaMeshVal(geom_id, index, vsp.CFD_MIN_EDGE_LEN, length_min_m)
        for kind, file_type, suffix in file_kinds:
            path = directory / f"{name}.{suffix}"
            vsp.SetFeaMeshFileName(geom_id, index, file_type, str(path))
            paths[(name, kind)] = path
        for kind, file_type, _ in file_kinds:
            vsp.ComputeFeaMesh(geom_id, index, file_type)
    return paths
