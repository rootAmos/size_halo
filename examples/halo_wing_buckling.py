"""Plate-buckling estimate for the Halo wing-box covers and spar webs at ultimate load (plan 043, open item).

Not a sizing model and not fixed by plan 043: a hand check stating how far the unstiffened AFDD gauges are from
being stable, so the strength margin is not over-read. Classical flat-plate buckling, all edges simply supported
(Bruhn C5; conservative against the clamped edges a bonded rib and spar give, which raise k by up to about 1.7x):

- Compression cover panel between two ribs (`pitch_rib_wing_m`, the OpenVSP layout's 0.5 m) and the two spars
  (box width b): sigma_cr = k_c pi^2 E / (12 (1 - nu^2)) (t / b)^2, k_c = (b / a + a / b)^2 for one half-wave.
- Spar web between two ribs, depth h: tau_cr = k_s pi^2 E / (12 (1 - nu^2)) (t / h)^2, k_s = 5.35 + 4 (h / a)^2
  for a rib pitch a longer than h (the case here).
- Applied at ultimate, at the critical station of the strain model: cover stress E_box x the beam-theory cap strain
  (the panel middle; the peak factor applies at the front-spar corner); web shear the jump-take-off rotor force
  at one tip over the two webs (bending shear only, torsion left out).
- The gauge is the AFDD wall area over the box perimeter, times the load-carrying factor of the calibration.
- What it would take: the sandwich core depth (faces of the same total gauge, isotropic) whose bending stiffness
  makes sigma_cr equal the applied cover stress, and its core mass at an assumed 48 kg/m3 core plus 0.3 kg/m2
  adhesive per panel.

Run from the repo root: `python -m examples.halo_wing_buckling --reference <reference.json> [--share 0.5]`.
"""
import argparse
import json
import math
from pathlib import Path

from aircraft_closure.weights import afdd

pitch_rib_wing_m = 0.5            # OpenVSP structure layout (plan 031)
distance_station_m = 0.3
factor_peak = 1.3324
density_core_kg_m3, mass_adhesive_kg_m2 = 48.0, 0.3     # assumed Nomex-class core and film adhesive


def buckling(reference, share):
    r, w, material = reference, reference["wing_afdd"], reference["material_wing"]
    factor = 1 + share * (r["factor_wing"] - 1)
    chord_m = r["area_wing_m2"] / r["span_wing_m"]
    depth_m = r["ratio_depth_spar_cap"] * r["thickness_to_chord_wing"] * chord_m
    width_m = r["fraction_chord_torque_box"] * chord_m
    thickness_m = factor * w["area_torque_box_m2"] / (2 * (width_m + depth_m))
    modulus_Pa = material["modulus_torque_box_Pa"]
    poisson = min(max(modulus_Pa / (2 * material["modulus_shear_torque_box_Pa"]) - 1, 0.0), 0.45)
    plate_Pa = math.pi**2 * modulus_Pa / (12 * (1 - poisson**2))
    moment_Nm = afdd.moment_jump_station_ultimate_Nm(r["mass_takeoff_kg"], r["mass_tip_kg"], r["span_wing_m"],
                                                     r["width_fuselage_m"], distance_station_m,
                                                     load_factor_jump=r["load_factor_jump"])
    strain_beam, _ = afdd.strain_cap_jump_ultimate(chord_m, r["thickness_to_chord_wing"],
                                                   r["fraction_chord_torque_box"], r["ratio_depth_spar_cap"],
                                                   w["area_torque_box_m2"], w["area_spar_m2"], modulus_Pa,
                                                   material["modulus_spar_Pa"], moment_Nm, factor)
    a_m = pitch_rib_wing_m
    k_c = (width_m / a_m + a_m / width_m)**2
    stress_cr_cover_Pa = k_c * plate_Pa * (thickness_m / width_m)**2
    stress_cover_Pa = modulus_Pa * strain_beam
    k_s = 5.35 + 4 * (depth_m / a_m)**2 if a_m >= depth_m else 5.35 * (depth_m / a_m)**2 + 4
    stress_cr_web_Pa = k_s * plate_Pa * (thickness_m / min(depth_m, a_m))**2
    force_tip_N = moment_Nm / (r["span_wing_m"] / 2 - r["width_fuselage_m"] / 2 - distance_station_m)
    stress_web_Pa = force_tip_N / (2 * depth_m * thickness_m)
    # Sandwich: D = E t h^2 / (4 (1 - nu^2)) with faces t/2 each a core depth h apart; solid D = E t^3 / (12 (1 - nu^2)).
    depth_core_m = thickness_m * math.sqrt(stress_cover_Pa / stress_cr_cover_Pa / 3)
    length_m = r["span_wing_m"] - r["width_fuselage_m"]
    area_covers_m2 = 2 * width_m * length_m
    mass_core_kg = area_covers_m2 * (density_core_kg_m3 * depth_core_m + mass_adhesive_kg_m2)
    return dict(thickness_wall_m=thickness_m, width_box_m=width_m, depth_box_m=depth_m,
                stress_cover_Pa=stress_cover_Pa, stress_cr_cover_Pa=stress_cr_cover_Pa,
                stress_web_Pa=stress_web_Pa, stress_cr_web_Pa=stress_cr_web_Pa,
                depth_core_cover_m=depth_core_m, mass_core_covers_kg=mass_core_kg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", default="output/structure/reference.json")
    parser.add_argument("--share", type=float, default=0.5)
    args = parser.parse_args()
    b = buckling(json.loads(Path(args.reference).read_text()), args.share)
    print(f"Box {1e3 * b['width_box_m']:.0f} x {1e3 * b['depth_box_m']:.0f} mm, walls {1e3 * b['thickness_wall_m']:.2f}"
          f" mm, ribs every {pitch_rib_wing_m} m")
    print(f"Cover (compression): ultimate {b['stress_cover_Pa'] / 1e6:.0f} MPa, buckles at "
          f"{b['stress_cr_cover_Pa'] / 1e6:.2f} MPa ({100 * b['stress_cr_cover_Pa'] / b['stress_cover_Pa']:.2f} % of "
          f"ultimate)")
    print(f"Web (shear): ultimate {b['stress_web_Pa'] / 1e6:.0f} MPa, buckles at {b['stress_cr_web_Pa'] / 1e6:.2f} MPa "
          f"({100 * b['stress_cr_web_Pa'] / b['stress_web_Pa']:.1f} % of ultimate)")
    print(f"Sandwich covers stable at ultimate: core {1e3 * b['depth_core_cover_m']:.0f} mm, core and adhesive "
          f"{b['mass_core_covers_kg']:.0f} kg for both covers (before the wing factor)")
