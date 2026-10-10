"""Halo whirl flutter (plan 040): the XV-15-calibrated reduced-order model on the sized wing.

The airframe is the cantilever wing with a rigid tip nacelle (`cantilever_wing_nacelle_airframe`), built from the
sized AFDD wing stiffnesses, tip mass and pylon inertia in `output/structure/reference.json` (written by
`python -m examples.halo_structure_reference`). The rotor carries the XV-15 blade dynamics scaled to the Halo rotor:
same Lock number, gimbal frequency, lag and coning frequencies per rev, delta-3 and pitch-lag coupling. The flutter
speed is multiplied by the XV-15 knockdown (0.84, plan 040) and compared with 1.15 V_D (V_D = 1.25 V_max assumed).

First the same cantilever builder is run on the XV-15 (published stiffness, nacelle and rotor masses) and compared
with the XV-15 NASTRAN-mode result, to check the concept airframe against the stick model.

Assumed (no Halo data): nacelle CG 0.5 m ahead of the spindle and the spindle on the wing elastic axis; nacelle roll
radius of gyration 0.3 m; wing torsional radius of gyration 0.25 chord; drive free on 100 kg m2 (electric drive at
each tip, no cross-shaft); sea level; thrust zero (the boundary is insensitive to it). Run from the repo root:
`python -m examples.halo_whirl_flutter`.
"""
import json
from dataclasses import replace
from pathlib import Path

import numpy as onp

from aircraft_closure.aeroelasticity.whirl_flutter import cantilever_wing_nacelle_airframe, flutter_speed
from examples import whirl_flutter_xv15 as xv15

knot_m_s, density_kg_m3 = xv15.knot_m_s, 1.225
knockdown = 0.84                                       # plan 040, XV-15 thick wing against CAMRAD II
velocities_m_s = onp.arange(150, 601, 5) * knot_m_s


def nacelle_airframe(length_m, stiffness_beam_Nm2, stiffness_chord_Nm2, stiffness_torsion_Nm2, mass_wing_kg_m,
                     chord_m, mass_tip_kg, radius_gyration_spindle_m, x_hub_m, x_cg_m=0.5,
                     radius_gyration_roll_m=0.3):
    """Cantilever airframe; pylon pitch/yaw inertia from the gyration radius about the spindle, moved to the CG."""
    inertia_cg_kg_m2 = mass_tip_kg * (radius_gyration_spindle_m**2 - x_cg_m**2)
    return cantilever_wing_nacelle_airframe(
        length_m, stiffness_beam_Nm2, stiffness_chord_Nm2, stiffness_torsion_Nm2, mass_wing_kg_m,
        mass_wing_kg_m * (0.25 * chord_m)**2, mass_tip_kg, x_cg_m,
        (mass_tip_kg * radius_gyration_roll_m**2, inertia_cg_kg_m2, inertia_cg_kg_m2), x_hub_m,
        damping_structural=(0.03, 0.03, 0.03))


def rotor_for(radius_m, count_blades, solidity, speed_rotor_rad_s):
    """XV-15 blade dynamics scaled to another rotor: same Lock number, per-rev frequencies and couplings."""
    chord_m = solidity * onp.pi * radius_m / count_blades
    lock = xv15.rotor.lock_number(density_kg_m3)
    inertia_kg_m2 = density_kg_m3 * xv15.rotor.lift_slope * chord_m * radius_m**4 / lock
    nu2_gimbal = float(xv15.rotor.frequency_flap_per_rev(xv15.speed_rotor_rad_s))**2
    ratio_moment = xv15.moment_first_blade_kg_m / xv15.inertia_flap_blade_kg_m2 * xv15.radius_m / radius_m
    return replace(xv15.rotor, radius_m=radius_m, count_blades=count_blades, chord_m=chord_m,
                   inertia_flap_blade_kg_m2=inertia_kg_m2, moment_first_blade_kg_m=ratio_moment * inertia_kg_m2,
                   stiffness_gimbal_Nm_rad=(nu2_gimbal - 1) * count_blades / 2 * inertia_kg_m2 * speed_rotor_rad_s**2,
                   drive="free", inertia_drive_kg_m2=100.0)


def calibrated_kt(airframe, rotor, speed_rotor_rad_s):
    v, label = flutter_speed(airframe, rotor, speed_rotor_rad_s, density_kg_m3, velocities_m_s)
    return (knockdown * v / knot_m_s if v is not None else onp.inf), label


def xv15_cantilever():
    """XV-15 through the concept airframe: Acree 1999 stiffness, stick-model masses, 47.3 in hub offset."""
    lb_in2 = xv15.lbf_N * 0.0254**2
    length_m = (32.17 * xv15.foot_m - 56 * 0.0254) / 2
    airframe = nacelle_airframe(length_m, 3.70e9 * lb_in2, 1.12e10 * lb_in2, 2.80e9 * lb_in2,
                                2534 * 0.4536 / (32.4 * xv15.foot_m), 169 / 32.17 * xv15.foot_m,
                                (3166 + 1118) / 2 * 0.4536, 0.222 * xv15.radius_m, 47.3 * 0.0254, x_cg_m=0.3)
    rotor = replace(xv15.rotor, drive="free")
    return calibrated_kt(airframe, rotor, xv15.speed_rotor_rad_s)


def halo(reference, scale_beam=1.0, scale_chord=1.0, scale_torsion=1.0):
    """Lowest calibrated flutter speed over the airplane-mode rotor speeds: (kt, mode, rotor speed)."""
    wing = reference["wing_afdd"]
    length_m = (reference["span_wing_m"] - reference["width_fuselage_m"]) / 2
    airframe = nacelle_airframe(length_m, scale_beam * wing["stiffness_beam_Nm2"],
                                scale_chord * wing["stiffness_chord_Nm2"],
                                scale_torsion * wing["stiffness_torsion_Nm2"],
                                reference["mass_wing_kg"] / reference["span_wing_m"], reference["chord_wing_m"],
                                reference["mass_tip_kg"], reference["radius_gyration_pylon_m"],
                                reference["length_mast_m"])
    speeds = sorted({round(p["speed_rotor_rad_s"], 3) for p in reference["whirl_points"]})
    results = []
    for speed in speeds:
        rotor = rotor_for(reference["radius_rotor_m"], reference["count_blades"], reference["solidity"], speed)
        v_kt, label = calibrated_kt(airframe, rotor, speed)
        results.append((v_kt, label, speed))
    return min(results, key=lambda r: r[0])


if __name__ == "__main__":
    v_kt, label = xv15_cantilever()
    print(f"XV-15 through the cantilever airframe: {v_kt:.0f} kt calibrated ({label}); "
          f"NASTRAN hub modes: 335 kt (by construction of the knockdown)")
    reference = json.loads(Path("output/structure/reference.json").read_text())
    v_max_kt = reference["velocity_max_m_s"] / knot_m_s
    required_kt = 1.15 * 1.25 * v_max_kt
    v_kt, label, speed = halo(reference)
    print(f"\nHalo: calibrated flutter speed {v_kt:.0f} kt ({label}, rotor {speed:.1f} rad/s); required "
          f"1.15 x V_D = {required_kt:.0f} kt (V_D = 1.25 x {v_max_kt:.0f} kt); margin {v_kt / required_kt - 1:+.0%}")
    print("Stiffness sensitivity (calibrated kt):")
    for name, kwargs in (("beam", "scale_beam"), ("chord", "scale_chord"), ("torsion", "scale_torsion")):
        row = [halo(reference, **{kwargs: s})[0] for s in (0.5, 0.75, 1.25, 1.5)]
        print(f"  {name:8s} x0.5 {row[0]:.0f}  x0.75 {row[1]:.0f}  x1.25 {row[2]:.0f}  x1.5 {row[3]:.0f}")
