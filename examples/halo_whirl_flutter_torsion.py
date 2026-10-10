"""Plan 040: AFDD torsion frequency the Halo wing needs to clear 1.15 V_D in whirl flutter, and its wing weight.

Re-sizes the AFDD wing from `output/structure/reference.json` with a scaled torsion-frequency target (min gauge,
smoothing and spar settings of the Halo reference), feeds the realized stiffnesses to `halo_whirl_flutter.halo`
and secant-iterates to the required speed. Run from the repo root: `python -m examples.halo_whirl_flutter_torsion`.
"""
import copy
import json
from pathlib import Path

import numpy as onp

from aircraft_closure.weights.afdd import wing_tiltrotor_afdd_masses
from examples import halo_whirl_flutter as hw
from examples.xv15_reference import Xv15Reference

ref = json.loads(Path("output/structure/reference.json").read_text())
x = Xv15Reference()
m = ref["material_wing"]
speed_design = ref["wing_afdd"]["frequency_torsion_rad_s"] / x.frequency_torsion_wing_per_rev  # guess; checked below


def afdd(scale_torsion_frequency=1.0, speed=None):
    return wing_tiltrotor_afdd_masses(
        span_m=ref["span_wing_m"], chord_m=ref["chord_wing_m"], thickness_to_chord=ref["thickness_to_chord_wing"],
        fraction_chord_torque_box=ref["fraction_chord_torque_box"], mass_design_kg=ref["mass_takeoff_kg"],
        mass_tip_kg=ref["mass_tip_kg"], radius_gyration_pylon_m=ref["radius_gyration_pylon_m"],
        speed_rotor_design_rad_s=speed or speed_design,
        frequency_torsion_per_rev=scale_torsion_frequency * x.frequency_torsion_wing_per_rev,
        frequency_beam_per_rev=x.frequency_beam_wing_per_rev, frequency_chord_per_rev=x.frequency_chord_wing_per_rev,
        density_torque_box_kg_m3=m["density_torque_box_kg_m3"], density_spar_kg_m3=m["density_spar_kg_m3"],
        modulus_shear_torque_box_Pa=m["modulus_shear_torque_box_Pa"], modulus_torque_box_Pa=m["modulus_torque_box_Pa"],
        modulus_spar_Pa=m["modulus_spar_Pa"], strain_ultimate=m["strain_ultimate"],
        area_control_surfaces_m2=0.185 * ref["area_wing_m2"], unit_mass_fairing_kg_m2=10.92,
        unit_mass_control_surfaces_kg_m2=15.18, width_fuselage_m=ref["width_fuselage_m"],
        width_attachment_m=ref["width_fuselage_m"], load_factor_jump=ref["load_factor_jump"], fraction_fittings=0.129,
        efficiency_torque_box=0.583, correction_spar_stiffness=0.526, smoothing=0.01,
        ratio_depth_spar_cap=ref["ratio_depth_spar_cap"], thickness_min_torque_box_m=0.001)


# Design rotor speed: the one whose AFDD output reproduces the reference wing (min gauge may be active).
from scipy.optimize import brentq
speed_design = brentq(lambda s: afdd(speed=s).stiffness_torsion_Nm2 - ref["wing_afdd"]["stiffness_torsion_Nm2"], 50, 61.6)
base = afdd()
for k in ("mass_torque_box_kg", "mass_spar_stiffness_kg", "mass_spar_jump_kg", "stiffness_torsion_Nm2",
          "stiffness_beam_Nm2", "stiffness_chord_Nm2"):
    print(f"{k:24s} model {float(getattr(base, k)):.4g}  reference {ref['wing_afdd'][k]:.4g}")
print(f"design rotor speed {speed_design:.2f} rad/s")
v_max_kt = ref["velocity_max_m_s"] / hw.knot_m_s
required_kt = 1.15 * 1.25 * v_max_kt


def flutter(scale):
    w = afdd(scale)
    r = copy.deepcopy(ref)
    r["wing_afdd"].update(stiffness_torsion_Nm2=float(w.stiffness_torsion_Nm2),
                          stiffness_beam_Nm2=float(w.stiffness_beam_Nm2),
                          stiffness_chord_Nm2=float(w.stiffness_chord_Nm2))
    v_kt, label, speed = hw.halo(r)
    dm = (float(w.total()) - float(base.total())) * ref["factor_wing"]
    print(f"torsion freq x{scale:.3f}: GJ x{float(w.stiffness_torsion_Nm2) / ref['wing_afdd']['stiffness_torsion_Nm2']:.3f}"
          f" EIbeam x{float(w.stiffness_beam_Nm2) / ref['wing_afdd']['stiffness_beam_Nm2']:.3f}: {v_kt:.0f} kt"
          f" ({label}, {speed:.1f} rad/s), wing +{dm:.1f} kg", flush=True)
    return v_kt


lo, hi = 1.0, 1.4
f_lo, f_hi = flutter(lo), flutter(hi)
for _ in range(5):
    mid = lo + (hi - lo) * (required_kt - f_lo) / (f_hi - f_lo)
    f_mid = flutter(mid)
    if abs(f_mid - required_kt) < 1.0:
        break
    lo, f_lo = (mid, f_mid) if f_mid < required_kt else (lo, f_lo)
    hi, f_hi = (hi, f_hi) if f_mid < required_kt else (mid, f_mid)
print(f"required {required_kt:.0f} kt")
