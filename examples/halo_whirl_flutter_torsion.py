"""Plan 040: AFDD torsion frequency the Halo wing needs to clear 1.15 V_D in whirl flutter, and its wing weight.

Re-sizes the AFDD wing from `output/structure/reference.json` with a scaled torsion-frequency target (min gauge,
smoothing and spar settings of the Halo reference), feeds the realized stiffnesses to `halo_whirl_flutter.halo`
and secant-iterates to the required speed. Run from the repo root: `python -m examples.halo_whirl_flutter_torsion`.
"""
import copy
import json
from pathlib import Path

from scipy.optimize import brentq

from aircraft_closure.weights.afdd import wing_tiltrotor_afdd_masses
from examples import halo_whirl_flutter as hw
from examples.xv15_reference import Xv15Reference

reference = json.loads((Path(__file__).resolve().parents[1] / "output/structure/reference.json").read_text())
xv15 = Xv15Reference()


def afdd_wing(scale_torsion_frequency=1.0, speed_rotor_design_rad_s=None):
    """The Halo AFDD wing (reference settings) with its torsion-frequency target scaled."""
    m, r = reference["material_wing"], reference
    return wing_tiltrotor_afdd_masses(
        span_m=r["span_wing_m"], chord_m=r["chord_wing_m"], thickness_to_chord=r["thickness_to_chord_wing"],
        fraction_chord_torque_box=r["fraction_chord_torque_box"], mass_design_kg=r["mass_takeoff_kg"],
        mass_tip_kg=r["mass_tip_kg"], radius_gyration_pylon_m=r["radius_gyration_pylon_m"],
        speed_rotor_design_rad_s=speed_rotor_design_rad_s or speed_rotor_design_rad_s_reference,
        frequency_torsion_per_rev=scale_torsion_frequency * xv15.frequency_torsion_wing_per_rev,
        frequency_beam_per_rev=xv15.frequency_beam_wing_per_rev,
        frequency_chord_per_rev=xv15.frequency_chord_wing_per_rev,
        density_torque_box_kg_m3=m["density_torque_box_kg_m3"], density_spar_kg_m3=m["density_spar_kg_m3"],
        modulus_shear_torque_box_Pa=m["modulus_shear_torque_box_Pa"], modulus_torque_box_Pa=m["modulus_torque_box_Pa"],
        modulus_spar_Pa=m["modulus_spar_Pa"], strain_ultimate=m["strain_ultimate"],
        area_control_surfaces_m2=0.185 * r["area_wing_m2"], unit_mass_fairing_kg_m2=10.92,
        unit_mass_control_surfaces_kg_m2=15.18, width_fuselage_m=r["width_fuselage_m"],
        width_attachment_m=r["width_fuselage_m"], load_factor_jump=r["load_factor_jump"], fraction_fittings=0.129,
        efficiency_torque_box=0.583, correction_spar_stiffness=0.526, smoothing=0.01,
        ratio_depth_spar_cap=r["ratio_depth_spar_cap"], thickness_min_torque_box_m=0.001)


# The wing design rotor speed is not exported; recover it as the one that reproduces the reference torsion stiffness.
speed_rotor_design_rad_s_reference = brentq(
    lambda s: afdd_wing(speed_rotor_design_rad_s=s).stiffness_torsion_Nm2
    - reference["wing_afdd"]["stiffness_torsion_Nm2"], 50, 61.6)
wing_reference = afdd_wing()


def flutter_with_torsion(scale_torsion_frequency):
    """(calibrated kt, mode, rotor speed, GJ ratio, EI beam ratio, wing mass increment kg incl. factor_wing)."""
    w = afdd_wing(scale_torsion_frequency)
    r = copy.deepcopy(reference)
    r["wing_afdd"].update(stiffness_torsion_Nm2=float(w.stiffness_torsion_Nm2),
                          stiffness_beam_Nm2=float(w.stiffness_beam_Nm2),
                          stiffness_chord_Nm2=float(w.stiffness_chord_Nm2))
    v_kt, label, speed = hw.halo(r)
    return (v_kt, label, speed, float(w.stiffness_torsion_Nm2) / reference["wing_afdd"]["stiffness_torsion_Nm2"],
            float(w.stiffness_beam_Nm2) / reference["wing_afdd"]["stiffness_beam_Nm2"],
            (float(w.total()) - float(wing_reference.total())) * reference["factor_wing"])


if __name__ == "__main__":
    for k in ("mass_torque_box_kg", "mass_spar_stiffness_kg", "mass_spar_jump_kg", "stiffness_torsion_Nm2",
              "stiffness_beam_Nm2", "stiffness_chord_Nm2"):
        print(f"{k:24s} model {float(getattr(wing_reference, k)):.4g}  reference {reference['wing_afdd'][k]:.4g}")
    print(f"design rotor speed {speed_rotor_design_rad_s_reference:.2f} rad/s")
    required_kt = 1.15 * 1.25 * reference["velocity_max_m_s"] / hw.knot_m_s

    def flutter(scale):
        v_kt, label, speed, gj, ei, dm = flutter_with_torsion(scale)
        print(f"torsion freq x{scale:.3f}: GJ x{gj:.3f} EIbeam x{ei:.3f}: {v_kt:.0f} kt ({label}, {speed:.1f} rad/s), "
              f"wing +{dm:.1f} kg", flush=True)
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
