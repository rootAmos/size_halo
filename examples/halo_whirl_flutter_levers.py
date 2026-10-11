"""Plan 040: cheaper levers on the Halo whirl-flutter speed than wing torsion stiffness.

One change at a time from the reference (`halo_whirl_flutter.halo` inputs), calibrated flutter speed at the two
critical airplane-mode rotor speeds (reserve loiter, the lowest, and maximum speed):
- nacelle CG ahead of the spindle (pylon pitch/yaw inertia about the CG held at the reference value);
- hub ahead of the spindle (mast length);
- delta-3 (pitch-flap coupling) and pitch-lag coupling.
Run from the repo root: `python -m examples.halo_whirl_flutter_levers`.
"""
import json
from dataclasses import replace
from pathlib import Path

import numpy as onp

from aircraft_closure.aeroelasticity.whirl_flutter import cantilever_wing_nacelle_airframe
from examples import halo_whirl_flutter as hw

reference = json.loads((Path(__file__).resolve().parents[1] / "output/structure/reference.json").read_text())
wing = reference["wing_afdd"]
mass_tip_kg, chord_m = reference["mass_tip_kg"], reference["chord_wing_m"]
mass_wing_kg_m = reference["mass_wing_kg"] / reference["span_wing_m"]
inertia_cg_kg_m2 = mass_tip_kg * (reference["radius_gyration_pylon_m"]**2 - 0.5**2)   # reference: CG 0.5 m ahead
speeds_rad_s = sorted({round(p["speed_rotor_rad_s"], 3) for p in reference["whirl_points"]})
speeds_rad_s = [speeds_rad_s[0], speeds_rad_s[-1]]


def airframe(x_cg_m=0.5, x_hub_m=reference["length_mast_m"]):
    return cantilever_wing_nacelle_airframe(
        (reference["span_wing_m"] - reference["width_fuselage_m"]) / 2, wing["stiffness_beam_Nm2"],
        wing["stiffness_chord_Nm2"], wing["stiffness_torsion_Nm2"], mass_wing_kg_m, mass_wing_kg_m * (0.25 * chord_m)**2,
        mass_tip_kg, x_cg_m, (mass_tip_kg * 0.3**2, inertia_cg_kg_m2, inertia_cg_kg_m2), x_hub_m,
        damping_structural=(0.03, 0.03, 0.03))


def flutter_kt(frame=None, speeds=None, **rotor_changes):
    """[(calibrated kt, mode)] at each rotor speed (default: the two critical ones) for one change."""
    frame = frame or airframe()
    results = []
    for speed in speeds or speeds_rad_s:
        rotor = replace(hw.rotor_for(reference["radius_rotor_m"], reference["count_blades"], reference["solidity"],
                                     speed), **rotor_changes)
        results.append(hw.calibrated_kt(frame, rotor, speed))
    return results


def row(name, frame=None, **rotor_changes):
    cells = [f"{v_kt:4.0f} kt ({label})" for v_kt, label in flutter_kt(frame, **rotor_changes)]
    print(f"{name:28s} " + "   ".join(cells), flush=True)


if __name__ == "__main__":
    required_kt = 1.15 * 1.25 * reference["velocity_max_m_s"] / hw.knot_m_s
    print(f"required {required_kt:.0f} kt; rotor speeds {speeds_rad_s} rad/s")
    base = hw.rotor_for(reference["radius_rotor_m"], reference["count_blades"], reference["solidity"], 30.0)
    row("reference")
    for x_cg_m in (0.0, 0.25, 0.75):
        row(f"nacelle CG {x_cg_m:.2f} m ahead", airframe(x_cg_m=x_cg_m))
    for x_hub_m in (0.6, 0.8, 1.2):
        row(f"hub {x_hub_m:.1f} m ahead", airframe(x_hub_m=x_hub_m))
    for delta3_deg in (-30.0, -10.0, 0.0):
        row(f"delta-3 {delta3_deg:+.0f} deg", pitch_flap_coupling=onp.tan(onp.radians(delta3_deg)))
    for k in (-0.4, 0.0, 0.2):
        row(f"pitch-lag {k:+.1f} (ref {base.pitch_lag_coupling:+.1f})", pitch_lag_coupling=k)
    # Combinations: the levers above are one change at a time.
    delta3_m10 = onp.tan(onp.radians(-10.0))
    row("CG 0 m + hub 0.6 m", airframe(x_cg_m=0.0, x_hub_m=0.6))
    row("delta-3 -10 + CG 0.25 m", airframe(x_cg_m=0.25), pitch_flap_coupling=delta3_m10)
    row("delta-3 -10 + CG 0 m", airframe(x_cg_m=0.0), pitch_flap_coupling=delta3_m10)
    row("delta-3 -10 + CG 0 m + hub 0.6 m", airframe(x_cg_m=0.0, x_hub_m=0.6), pitch_flap_coupling=delta3_m10)
