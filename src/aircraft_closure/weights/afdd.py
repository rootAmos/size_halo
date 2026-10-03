"""AFDD parametric rotorcraft weight equations, SI in and out.

AeroSandbox's weight library covers fixed-wing groups (Raymer, Torenbeek) but
no rotor, rotorcraft drive-system or tilting-nacelle groups; these fill that
gap in the same functional style. Source: W. Johnson, NDARC Theory,
NASA/TP-2009-215402, ch. 19 (AFDD weight models, regressions on turbine
helicopters and tiltrotors). The published equations take lb, ft, ft/s, hp and
rpm and return lb; each wrapper converts at its boundary and is otherwise the
published equation with a unit technology factor. Powers are plain products,
so every function accepts CasADi symbols. The tiltrotor wing (sec. 19-1.1) is a stiffness-based
structural model in consistent SI rather than a regression.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np
import aerosandbox.tools.units as u


def mass_blades_afdd82_kg(count_rotors, count_blades, radius_m, chord_m, speed_tip_m_s, frequency_flap_per_rev):
    """All blades of all rotors (NDARC eq. 19-2, AFDD82; 7.7 % mean error on 37 aircraft).

    `frequency_flap_per_rev` is the flap natural frequency; NDARC uses the
    coning frequency for teetering and gimballed rotors.
    """
    return 0.02606 * count_rotors * count_blades**0.6592 * (radius_m / u.foot)**1.3371 \
        * (chord_m / u.foot)**0.9959 * (speed_tip_m_s / u.foot)**0.6682 * frequency_flap_per_rev**2.5279 * u.lbm


def mass_hub_afdd82_kg(count_rotors, count_blades, radius_m, speed_tip_m_s, frequency_flap_per_rev, mass_blades_kg):
    """Hubs and hinges of all rotors (AFDD82; 10.2 % mean error on 35 aircraft).

    `mass_blades_kg` is the all-rotor blade mass, as in `mass_blades_afdd82_kg`.
    """
    return 0.003722 * count_rotors * count_blades**0.2807 * (radius_m / u.foot)**1.5377 \
        * (speed_tip_m_s / u.foot)**0.4290 * frequency_flap_per_rev**2.1414 \
        * (mass_blades_kg / count_rotors / u.lbm)**0.5505 * u.lbm


def mass_gearbox_rotor_shaft_afdd83_kg(power_drive_limit_W, speed_rotor_rad_s, speed_engine_rad_s, count_gearboxes,
                                       fraction_torque_second_rotor):
    """Gearboxes plus rotor shafts of the whole drive system (AFDD83; 7.7 % mean error on 30 aircraft).

    `fraction_torque_second_rotor` is the second main (or tail) rotor's share
    of rated torque; NDARC's typical value is 0.6 for twin main rotors.
    """
    return 57.72 * (power_drive_limit_W / u.hp)**0.8195 * (100 * fraction_torque_second_rotor)**0.0680 \
        * count_gearboxes**0.0663 * (speed_engine_rad_s / u.rpm / 1000)**0.0369 \
        / (speed_rotor_rad_s / u.rpm)**0.6379 * u.lbm


def mass_gearbox_rotor_shaft_afdd00_kg(count_rotors, power_drive_limit_W, speed_engine_rad_s, speed_rotor_rad_s):
    """Gearboxes plus rotor shafts (AFDD00; 8.6 % mean error on 52 aircraft). The input-speed exponent (0.099)
    gives a mild penalty for higher reduction ratios, unlike AFDD83's 0.0369."""
    return 95.7634 * count_rotors**0.38553 * (power_drive_limit_W / u.hp)**0.78137         * (speed_engine_rad_s / u.rpm)**0.09899 / (speed_rotor_rad_s / u.rpm)**0.80686 * u.lbm


def mass_drive_shaft_afdd82_kg(power_drive_limit_W, speed_rotor_rad_s, length_drive_shaft_m, count_drive_shafts,
                               fraction_power_second_rotor):
    """Intermediate drive shafts, e.g. a tiltrotor interconnect (AFDD82; 16 % mean error on 28 aircraft)."""
    torque_limit_hp_per_rpm = (power_drive_limit_W / u.hp) / (speed_rotor_rad_s / u.rpm)
    return 1.166 * torque_limit_hp_per_rpm**0.3828 * (length_drive_shaft_m / u.foot)**1.0455 \
        * count_drive_shafts**0.3909 * fraction_power_second_rotor**0.2693 * u.lbm


def mass_engine_support_afdd82_kg(mass_engines_kg, count_engines, fraction_air_induction=0.3):
    """Engine support structure (AFDD82; 11 % mean error on 12 aircraft); `mass_engines_kg` is all engines."""
    return 0.0412 * (1 - fraction_air_induction) * (mass_engines_kg / count_engines / u.lbm)**1.1433 \
        * count_engines**1.3762 * u.lbm


def mass_air_induction_afdd82_kg(mass_engines_kg, count_engines, fraction_air_induction=0.3):
    """Air induction group (AFDD82; 11 % mean error); NDARC's typical fraction is 0.3 (range 0.1-0.6)."""
    return 0.0412 * fraction_air_induction * (mass_engines_kg / count_engines / u.lbm)**1.1433 \
        * count_engines**1.3762 * u.lbm


def mass_engine_cowling_afdd82_kg(area_wetted_nacelles_m2):
    """Engine cowling (AFDD82; 17.9 % mean error on 12 aircraft); wetted area of all nacelles, less spinner."""
    return 0.2315 * (area_wetted_nacelles_m2 / u.foot**2)**1.3476 * u.lbm


def mass_rotor_group_afdd82_kg(count_rotors, count_blades, radius_m, chord_m, speed_tip_m_s, frequency_flap_per_rev):
    """Blades plus hubs of all rotors."""
    mass_blades_kg = mass_blades_afdd82_kg(count_rotors, count_blades, radius_m, chord_m, speed_tip_m_s,
                                           frequency_flap_per_rev)
    return mass_blades_kg + mass_hub_afdd82_kg(count_rotors, count_blades, radius_m, speed_tip_m_s,
                                               frequency_flap_per_rev, mass_blades_kg)


def _positive_part(value, smoothing_scale=0.0):
    """max(0, value); a nonzero `smoothing_scale` (same units) gives a smooth hyperbola for gradient-based solves."""
    if isinstance(smoothing_scale, (int, float)) and smoothing_scale == 0:
        return np.fmax(value, 0)
    return 0.5 * (value + np.sqrt(value**2 + smoothing_scale**2))


def section_form_factors_tiltrotor_wing(thickness_to_chord, fraction_chord_torque_box):
    """NDARC tiltrotor-wing section form factors (F_B, F_C, F_T, F_VH): beam bending, chord bending, torsion and
    spar-cap vertical/horizontal bending, relating airfoil and torque-box geometry to ideal shapes."""
    tau, w = thickness_to_chord, fraction_chord_torque_box
    factor_beam = (0.073 * np.sin(2 * np.pi * (tau - 0.151) / 0.1365) + 0.14598 * tau
                   + 0.610 * np.sin(2 * np.pi * (w + 0.080) / 2.1560) - (0.4126 - 1.6309 * tau) * (w - 0.131) + 0.0081)
    factor_chord = 0.640424 * w**2 - 0.89717 * w + 0.4615 * tau + 0.655317
    factor_torsion = ((0.27 - tau) / 0.12 * 0.12739 * (-0.96 + np.sqrt(3.32 + 94.6788 * w - (w / 0.08344)**2))
                      - 2.7545 * w**2 + 5.1799 * w - 0.2683)
    factor_spar_cap = 0.25 * np.sin(5.236 * w) + 0.325
    return factor_beam, factor_chord, factor_torsion, factor_spar_cap


@dataclass(frozen=True)
class TiltrotorWingMasses:
    """Result of `wing_tiltrotor_afdd_masses`. Masses in kg; stiffnesses in N m2 as realized (after the jump
    take-off increment); mode frequencies in rad/s from NDARC's single-mode relations. Fields may be symbolic."""
    mass_torque_box_kg: Any
    mass_spar_stiffness_kg: Any       # spar caps for the chord and beam bending-frequency requirements
    mass_spar_jump_kg: Any            # extra spar caps for the jump take-off bending moment
    mass_fairing_kg: Any
    mass_control_surfaces_kg: Any
    mass_fittings_kg: Any
    mass_fold_kg: Any
    stiffness_torsion_Nm2: Any
    stiffness_beam_Nm2: Any
    stiffness_chord_Nm2: Any
    frequency_torsion_rad_s: Any
    frequency_beam_rad_s: Any
    frequency_chord_rad_s: Any
    moment_jump_ultimate_Nm: Any
    area_torque_box_m2: Any
    area_spar_m2: Any

    def mass_primary_kg(self):
        return self.mass_torque_box_kg + self.mass_spar_stiffness_kg + self.mass_spar_jump_kg

    def total(self):
        """Wing group: primary structure, fairings, control surfaces, fittings and fold/tilt."""
        return (self.mass_primary_kg() + self.mass_fairing_kg + self.mass_control_surfaces_kg + self.mass_fittings_kg
                + self.mass_fold_kg)


def wing_tiltrotor_afdd_masses(span_m, chord_m, thickness_to_chord, fraction_chord_torque_box, mass_design_kg,
                               mass_tip_kg, radius_gyration_pylon_m, speed_rotor_design_rad_s,
                               frequency_torsion_per_rev, frequency_beam_per_rev, frequency_chord_per_rev,
                               density_torque_box_kg_m3, density_spar_kg_m3, modulus_shear_torque_box_Pa,
                               modulus_torque_box_Pa, modulus_spar_Pa, strain_ultimate,
                               area_control_surfaces_m2, unit_mass_fairing_kg_m2, unit_mass_control_surfaces_kg_m2,
                               width_fuselage_m, width_attachment_m, count_rotors=2, count_tips=2,
                               load_factor_jump=2.0, thrust_max_rotor_N=None, fraction_fittings=0.0,
                               fraction_fold=0.0, efficiency_torque_box=1.0, efficiency_spar=1.0,
                               correction_spar_stiffness=1.0, correction_spar_strength=1.0,
                               correction_moment_spar=1.0, smoothing=0.0):
    """AFDD tiltrotor wing (NDARC Theory, NASA/TP-2009-215402, sec. 19-1.1; Chappell and Peyran, SAWE 2107, 1992).

    Consistent SI, so NDARC's unit factor is 1. The torque box is sized by the torsion frequency, spar caps by the
    chord and beam bending frequencies, then by the jump take-off root moment. Frequencies are per rev of
    `speed_rotor_design_rad_s` (the wing weight design condition). The sequence is NDARC's single explicit pass
    (the jump moment uses the wing mass before the jump increment); nothing iterates. `mass_tip_kg` is the mass on
    one wing tip; its pitch inertia is `mass_tip_kg * radius_gyration_pylon_m**2`; f_tip counts `count_tips` tips.
    NDARC's two "replace by zero if negative" steps are max(0, .); `smoothing` > 0 rounds them with a scale of
    `smoothing` times the required stiffness or moment (for gradient-based sizing).
    """
    g_m_s2 = 9.80665
    thickness_m = thickness_to_chord * chord_m
    chord_torque_box_m = fraction_chord_torque_box * chord_m
    factor_beam, factor_chord, factor_torsion, factor_spar_cap = section_form_factors_tiltrotor_wing(
        thickness_to_chord, fraction_chord_torque_box)
    fraction_tip = count_tips * mass_tip_kg / mass_design_kg
    factor_mode = 1 - fraction_tip
    frequency_torsion_rad_s = frequency_torsion_per_rev * speed_rotor_design_rad_s
    frequency_beam_rad_s = frequency_beam_per_rev * speed_rotor_design_rad_s
    frequency_chord_rad_s = frequency_chord_per_rev * speed_rotor_design_rad_s
    bending_mass_kg_m3 = span_m**3 / 24 * 0.5 * mass_tip_kg * factor_mode
    torsion_inertia_kg_m3 = 0.5 * span_m * 0.5 * mass_tip_kg * radius_gyration_pylon_m**2

    # Torque box from the torsion frequency (ideal shape: a tube of radius t, J = F_T A t^2 / 4).
    stiffness_torsion_Nm2 = frequency_torsion_rad_s**2 * torsion_inertia_kg_m3
    area_torque_box_m2 = 4 * stiffness_torsion_Nm2 / (modulus_shear_torque_box_Pa * factor_torsion * thickness_m**2)

    # Spar caps (beyond the torque box) for the chord and beam bending frequencies.
    stiffness_chord_req_Nm2 = frequency_chord_rad_s**2 * bending_mass_kg_m3
    stiffness_beam_req_Nm2 = frequency_beam_rad_s**2 * bending_mass_kg_m3
    stiffness_chord_box_Nm2 = modulus_torque_box_Pa * factor_chord * area_torque_box_m2 * chord_torque_box_m**2 / 4
    stiffness_chord_spar_Nm2 = _positive_part(stiffness_chord_req_Nm2 - stiffness_chord_box_Nm2,
                                              smoothing * stiffness_chord_req_Nm2)
    area_spar_chord_m2 = stiffness_chord_spar_Nm2 / (modulus_spar_Pa * chord_torque_box_m**2 / 4)
    stiffness_beam_box_Nm2 = modulus_torque_box_Pa * factor_beam * area_torque_box_m2 * thickness_m**2 / 4
    stiffness_spar_cap_beam_Nm2 = modulus_spar_Pa * factor_spar_cap * area_spar_chord_m2 * thickness_m**2 / 4
    stiffness_beam_spar_Nm2 = _positive_part(
        stiffness_beam_req_Nm2 - stiffness_beam_box_Nm2 - stiffness_spar_cap_beam_Nm2,
        smoothing * stiffness_beam_req_Nm2)
    area_spar_beam_m2 = stiffness_beam_spar_Nm2 / (modulus_spar_Pa * thickness_m**2 / 4)
    stiffness_spar_Nm2 = stiffness_spar_cap_beam_Nm2 + stiffness_beam_spar_Nm2
    area_spar_m2 = area_spar_chord_m2 + area_spar_beam_m2

    mass_torque_box_kg = area_torque_box_m2 * density_torque_box_kg_m3 * span_m / efficiency_torque_box
    mass_spar_stiffness_kg = (correction_spar_stiffness * area_spar_m2 * density_spar_kg_m3 * span_m
                              / efficiency_spar)
    area_fairing_m2 = ((span_m - width_attachment_m) * chord_m * (1 - fraction_chord_torque_box)
                       - area_control_surfaces_m2)
    mass_fairing_kg = area_fairing_m2 * unit_mass_fairing_kg_m2
    mass_control_surfaces_kg = area_control_surfaces_m2 * unit_mass_control_surfaces_kg_m2
    ratio_fittings = fraction_fittings / (1 - fraction_fittings)
    mass_wing_kg = (1 + ratio_fittings) * (mass_torque_box_kg + mass_spar_stiffness_kg + mass_fairing_kg
                                           + mass_control_surfaces_kg)

    # Jump take-off: ultimate root bending moment against the torque-box and spar-cap capacity.
    length_wing_m = span_m - width_fuselage_m
    thrust_capability_N = load_factor_jump * mass_design_kg * g_m_s2 / count_rotors
    if thrust_max_rotor_N is not None:
        thrust_capability_N = np.fmax(thrust_capability_N, thrust_max_rotor_N)
    moment_jump_ultimate_Nm = thrust_capability_N * length_wing_m * (
        0.75 * (1 - fraction_tip) - 0.375 * (length_wing_m / span_m) * (mass_wing_kg / mass_design_kg))
    moment_box_Nm = 2 * stiffness_beam_box_Nm2 * strain_ultimate / thickness_m
    moment_spar_Nm = 2 * correction_moment_spar * stiffness_spar_Nm2 * strain_ultimate / thickness_m
    moment_deficit_Nm = _positive_part(moment_jump_ultimate_Nm - moment_box_Nm - moment_spar_Nm,
                                       smoothing * moment_jump_ultimate_Nm)
    area_spar_jump_m2 = 2 * moment_deficit_Nm / (strain_ultimate * modulus_spar_Pa * thickness_m)
    mass_spar_jump_kg = correction_spar_strength * area_spar_jump_m2 * density_spar_kg_m3 * span_m / efficiency_spar

    mass_fittings_kg = ratio_fittings * (mass_torque_box_kg + mass_spar_stiffness_kg + mass_spar_jump_kg
                                         + mass_fairing_kg + mass_control_surfaces_kg)
    mass_fold_kg = fraction_fold * (mass_torque_box_kg + mass_spar_stiffness_kg + mass_spar_jump_kg + mass_fairing_kg
                                    + mass_control_surfaces_kg + mass_fittings_kg + count_tips * mass_tip_kg)

    # Realized stiffness (jump caps add beam stiffness) and the frequencies they give.
    stiffness_chord_Nm2 = stiffness_chord_box_Nm2 + modulus_spar_Pa * area_spar_chord_m2 * chord_torque_box_m**2 / 4
    stiffness_beam_Nm2 = (stiffness_beam_box_Nm2 + stiffness_spar_Nm2
                          + modulus_spar_Pa * area_spar_jump_m2 * thickness_m**2 / 4)
    return TiltrotorWingMasses(
        mass_torque_box_kg=mass_torque_box_kg, mass_spar_stiffness_kg=mass_spar_stiffness_kg,
        mass_spar_jump_kg=mass_spar_jump_kg, mass_fairing_kg=mass_fairing_kg,
        mass_control_surfaces_kg=mass_control_surfaces_kg, mass_fittings_kg=mass_fittings_kg, mass_fold_kg=mass_fold_kg,
        stiffness_torsion_Nm2=stiffness_torsion_Nm2, stiffness_beam_Nm2=stiffness_beam_Nm2,
        stiffness_chord_Nm2=stiffness_chord_Nm2,
        frequency_torsion_rad_s=np.sqrt(stiffness_torsion_Nm2 / torsion_inertia_kg_m3),
        frequency_beam_rad_s=np.sqrt(stiffness_beam_Nm2 / bending_mass_kg_m3),
        frequency_chord_rad_s=np.sqrt(stiffness_chord_Nm2 / bending_mass_kg_m3),
        moment_jump_ultimate_Nm=moment_jump_ultimate_Nm, area_torque_box_m2=area_torque_box_m2,
        area_spar_m2=area_spar_m2 + area_spar_jump_m2)


if __name__ == "__main__":
    # XV-15 rotor group at its published geometry (two 25 ft, 3-blade rotors, 14 in chord, 740 ft/s).
    print(mass_rotor_group_afdd82_kg(2, 3, 12.5 * u.foot, 14 * u.inch, 740 * u.foot, np.array([1.0, 1.55])) / u.lbm)
