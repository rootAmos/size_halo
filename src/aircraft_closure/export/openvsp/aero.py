"""VSPAERO and OpenVSP parasite-drag analyses of a `GeometrySnapshot`, as a cross-check of AeroSandbox.

Checks only: nothing here feeds the sizing. The airplane-mode (0 deg) outer
mold line is analysed without rotors.

VSPAERO models (`model`):
- "thin": vortex lattice on the lifting surfaces only (wing, tails).
- "mixed": thin lifting surfaces plus panelled (thick) bodies. A vortex-lattice
  body is a flat plate on its centre plane, which coincides with the wing tip
  inside a tip nacelle and makes the solve singular, so bodies are panelled.
- "panel": every component panelled (thick).

Induced drag is the wake (Trefftz-plane) value `CDiw`. With thick bodies it is
not reliable in VSPAERO 7 (near-field CDi can even be negative), so induced
drag is compared from the "thin" model only.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import openvsp as vsp

from aircraft_closure.export.openvsp.model import build_openvsp_model, set_airframe

_set_thin = vsp.SET_FIRST_USER + 4
_set_thick = vsp.SET_FIRST_USER + 5
_lifting_surfaces = ("Wing", "HorizontalTail", "VerticalTail", "VTail")


@dataclass(frozen=True)
class VspaeroPolar:
    model: str
    alpha_deg: np.ndarray
    cl: np.ndarray
    cdi_wake: np.ndarray
    cm: np.ndarray
    cd0_vspaero: np.ndarray       # VSPAERO's own parasite estimate (not its parasite-drag tool)


@dataclass(frozen=True)
class ParasiteDragBuildup:
    """OpenVSP parasite-drag tool: per-component wetted area, form factor and CD0 contribution."""
    labels: tuple
    area_wetted_m2: np.ndarray
    form_factor: np.ndarray
    cd0: np.ndarray
    cd0_total: float


def _assign_sets(geoms, model):
    """Thin and thick VSPAERO sets for the model; returns (thick set, thin set)."""
    for name, geom_id in geoms.items():
        if not vsp.GetSetFlag(geom_id, set_airframe):
            continue
        lifting = name in _lifting_surfaces
        if model == "thin" and lifting or model == "mixed" and lifting:
            vsp.SetSetFlag(geom_id, _set_thin, True)
        elif model == "mixed" and not lifting or model == "panel":
            vsp.SetSetFlag(geom_id, _set_thick, True)
    vsp.Update()
    thick = _set_thick if model in ("mixed", "panel") else vsp.SET_NONE
    thin = _set_thin if model in ("thin", "mixed") else vsp.SET_NONE
    return thick, thin


def run_vspaero_sweep(snapshot, directory, model, alpha_deg, mach, reynolds_cref, xyz_ref_m, count_cpu=8):
    """VSPAERO alpha sweep about `xyz_ref_m` with the wing reference area and span and the wing MAC."""
    if model not in ("thin", "mixed", "panel"):
        raise ValueError(f"Unknown VSPAERO model '{model}'.")
    directory = Path(directory) / model
    directory.mkdir(parents=True, exist_ok=True)
    geoms = build_openvsp_model(snapshot, 0.0)
    thick, thin = _assign_sets(geoms, model)
    # VSPAERO writes its files next to the model file.
    vsp.SetVSP3FileName(str(directory / "halo_aero.vsp3"))
    vsp.WriteVSPFile(str(directory / "halo_aero.vsp3"))

    geometry = "VSPAEROComputeGeometry"
    vsp.SetAnalysisInputDefaults(geometry)
    vsp.SetIntAnalysisInput(geometry, "GeomSet", [thick])
    vsp.SetIntAnalysisInput(geometry, "ThinGeomSet", [thin])
    vsp.SetIntAnalysisInput(geometry, "Symmetry", [0])
    vsp.ExecAnalysis(geometry)

    sweep = "VSPAEROSweep"
    vsp.SetAnalysisInputDefaults(sweep)
    alpha_deg = np.asarray(alpha_deg, dtype=float)
    wing = snapshot.wing
    for name, value in (("GeomSet", thick), ("ThinGeomSet", thin), ("RefFlag", 0), ("AlphaNpts", len(alpha_deg)),
                        ("MachNpts", 1), ("BetaNpts", 1), ("Symmetry", 0), ("NCPU", count_cpu),
                        ("WakeNumIter", 3)):
        vsp.SetIntAnalysisInput(sweep, name, [value])
    for name, value in (("AlphaStart", alpha_deg[0]), ("AlphaEnd", alpha_deg[-1]), ("MachStart", mach),
                        ("ReCref", reynolds_cref), ("Sref", 0.5 * (wing.chord_root_m + wing.chord_tip_m) * wing.span_m),
                        ("bref", wing.span_m), ("cref", wing.mean_aerodynamic_chord_m()),
                        ("Xcg", xyz_ref_m[0]), ("Ycg", xyz_ref_m[1]), ("Zcg", xyz_ref_m[2])):
        vsp.SetDoubleAnalysisInput(sweep, name, [float(value)])
    vsp.ExecAnalysis(sweep)
    polar = vsp.FindLatestResultsID("VSPAERO_Polar")

    def results(name):
        return np.array(vsp.GetDoubleResults(polar, name))

    return VspaeroPolar(model=model, alpha_deg=results("Alpha"), cl=results("CLtot"), cdi_wake=results("CDiw"),
                        cm=results("CMytot"), cd0_vspaero=results("CDo"))


def run_parasite_drag(snapshot, altitude_m, velocity_m_s):
    """OpenVSP component parasite-drag build-up (US 1976 atmosphere, compressible Schlichting Cf)."""
    build_openvsp_model(snapshot, 0.0)
    analysis = "ParasiteDrag"
    vsp.SetAnalysisInputDefaults(analysis)
    wing = snapshot.wing
    for name, value in (("GeomSet", set_airframe), ("LengthUnit", vsp.LEN_M), ("AltLengthUnit", vsp.PD_UNITS_METRIC),
                        ("VelocityUnit", vsp.V_UNIT_M_S), ("TempUnit", vsp.TEMP_UNIT_K),
                        ("FreestreamPropChoice", vsp.ATMOS_TYPE_US_STANDARD_1976)):
        vsp.SetIntAnalysisInput(analysis, name, [value])
    for name, value in (("Altitude", altitude_m), ("Vinf", velocity_m_s),
                        ("Sref", 0.5 * (wing.chord_root_m + wing.chord_tip_m) * wing.span_m)):
        vsp.SetDoubleAnalysisInput(analysis, name, [float(value)])
    result = vsp.ExecAnalysis(analysis)
    return ParasiteDragBuildup(labels=tuple(vsp.GetStringResults(result, "Comp_Label")),
                               area_wetted_m2=np.array(vsp.GetDoubleResults(result, "Comp_Swet")),
                               form_factor=np.array(vsp.GetDoubleResults(result, "Comp_FFOut")),
                               cd0=np.array(vsp.GetDoubleResults(result, "Comp_CD")),
                               cd0_total=vsp.GetDoubleResults(result, "Total_CD_Total")[0])
