"""AFDD parametric rotorcraft weight equations, SI in and out.

AeroSandbox's weight library covers fixed-wing groups (Raymer, Torenbeek) but
no rotor, rotorcraft drive-system or tilting-nacelle groups; these fill that
gap in the same functional style. Source: W. Johnson, NDARC Theory,
NASA/TP-2009-215402, ch. 19 (AFDD weight models, regressions on turbine
helicopters and tiltrotors). The published equations take lb, ft, ft/s, hp and
rpm and return lb; each wrapper converts at its boundary and is otherwise the
published equation with a unit technology factor. Powers are plain products,
so every function accepts CasADi symbols.
"""
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


if __name__ == "__main__":
    # XV-15 rotor group at its published geometry (two 25 ft, 3-blade rotors, 14 in chord, 740 ft/s).
    print(mass_rotor_group_afdd82_kg(2, 3, 12.5 * u.foot, 14 * u.inch, 740 * u.foot, np.array([1.0, 1.55])) / u.lbm)
