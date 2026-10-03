"""Blown-wing correction factors for rotor slipstream in airplane mode (Tier 21, plan 025).

Per rotor (momentum theory, axial flight at V, thrust T, disk area A, radius R):
    v_i   = -V/2 + sqrt(V^2/4 + T / (2 rho A))                    induced velocity at the disk
    v_w   = v_i (1 + x / sqrt(x^2 + R^2))                         axial increment at the wing, a distance x
                                                                   behind the disk (McCormick)
    R_s   = R sqrt((V + v_i) / (V + v_w))                         slipstream radius there (continuity)
    q_s/q = ((V + v_w) / V)^2                                     dynamic-pressure ratio on the immersed span
    v_t   = k_swirl T / (rho A Omega (2/3) R)                     swirl: ideal torque T (V + v_i) / Omega carried by
                                                                   the mass flux rho A (V + v_i) at radius 2R/3
    delta = s atan(v_t / (V + v_w))                               swirl angle at the wing; s = +1 inboard-up rotation
                                                                   for tip-mounted rotors (raises the local angle on
                                                                   the immersed, inboard half)
Immersed area S_b = n f 2 R_s c, f = 1/2 for tip-mounted rotors (half of the slipstream is outboard of the tip).
With the unblown wing's lift and profile-drag coefficients cl_w, cd_w and lift slope a (per rad), on S_ref:
    delta_cl       = S_b/S [ (q_s/q)(cl_w + a delta) - cl_w ]
    delta_cd_prof  = S_b/S (q_s/q - 1) cd_w                       profile drag at the higher dynamic pressure
    delta_cd_swirl = -eta n (1/2) mdot v_t^2 / (V q S),  mdot = rho A (V + v_i)
                                                                   swirl recovery: the wing, acting as a stator, turns
                                                                   back a fraction eta of the slipstream's swirl
                                                                   kinetic-energy flux into thrust (Veldhuis, TU Delft
                                                                   thesis 2005; inboard-up tip rotors: Snyder and
                                                                   Zumwalt, J. Aircraft 6(5), 1969). Bounding it by the
                                                                   swirl energy keeps it below the rotor's own swirl
                                                                   loss; with outboard-up rotation (s = -1) the wing
                                                                   adds swirl instead, the same magnitude as a loss.
Every increment is zero at zero thrust. The induced drag of the extra lift is carried by the caller's
induced-drag term on the total lift.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class RotorState:
    """Per-rotor operating state handed to an aerodynamics model (airplane mode)."""
    thrust_per_rotor_N: Any
    speed_rotor_rad_s: Any


@dataclass(frozen=True)
class BlownWingIncrement:
    velocity_induced_m_s: Any
    velocity_increment_wing_m_s: Any
    ratio_dynamic_pressure: Any
    angle_swirl_deg: Any
    area_immersed_m2: Any
    delta_cl: Any
    delta_cd_profile: Any
    delta_cd_swirl: Any


@dataclass(frozen=True)
class BlownWing:
    fraction_slipstream_on_wing: Any = 0.5         # tip-mounted rotors
    ratio_distance_disk_to_radius: Any = 0.4       # disk ahead of the wing quarter chord, x / R (assumed)
    sign_swirl: Any = 1.0                          # +1 inboard-up, -1 outboard-up
    factor_swirl: Any = 1.0                        # 1: ideal (induced) torque only
    efficiency_swirl_recovery: Any = 0.5           # fraction of the swirl energy flux recovered (assumed)

    def evaluate(self, velocity_m_s, density_kg_m3, rotor_state, area_disk_m2, count_rotors, chord_wing_m,
                 area_wing_m2, cl_wing, cd_profile_wing, cl_alpha_wing_per_rad):
        thrust_N = rotor_state.thrust_per_rotor_N
        radius_m = np.sqrt(area_disk_m2 / np.pi)
        velocity_induced_m_s = -velocity_m_s / 2 + np.sqrt(velocity_m_s**2 / 4
                                                           + thrust_N / (2 * density_kg_m3 * area_disk_m2))
        x = self.ratio_distance_disk_to_radius
        velocity_increment_wing_m_s = velocity_induced_m_s * (1 + x / np.sqrt(x**2 + 1))
        velocity_wing_m_s = velocity_m_s + velocity_increment_wing_m_s
        radius_slipstream_m = radius_m * np.sqrt((velocity_m_s + velocity_induced_m_s) / velocity_wing_m_s)
        ratio_dynamic_pressure = (velocity_wing_m_s / velocity_m_s)**2
        velocity_swirl_m_s = self.factor_swirl * thrust_N / (density_kg_m3 * area_disk_m2
                                                             * rotor_state.speed_rotor_rad_s * (2 / 3) * radius_m)
        angle_swirl_rad = self.sign_swirl * np.arctan(velocity_swirl_m_s / velocity_wing_m_s)
        area_immersed_m2 = (count_rotors * self.fraction_slipstream_on_wing * 2 * radius_slipstream_m * chord_wing_m)
        fraction_area = area_immersed_m2 / area_wing_m2
        cl_blown = cl_wing + cl_alpha_wing_per_rad * angle_swirl_rad
        return BlownWingIncrement(
            velocity_induced_m_s=velocity_induced_m_s, velocity_increment_wing_m_s=velocity_increment_wing_m_s,
            ratio_dynamic_pressure=ratio_dynamic_pressure, angle_swirl_deg=np.degrees(angle_swirl_rad),
            area_immersed_m2=area_immersed_m2,
            delta_cl=fraction_area * (ratio_dynamic_pressure * cl_blown - cl_wing),
            delta_cd_profile=fraction_area * (ratio_dynamic_pressure - 1) * cd_profile_wing,
            delta_cd_swirl=-self.sign_swirl * self.efficiency_swirl_recovery * count_rotors * 0.5 * density_kg_m3
            * area_disk_m2 * (velocity_m_s + velocity_induced_m_s) * velocity_swirl_m_s**2
            / (velocity_m_s * 0.5 * density_kg_m3 * velocity_m_s**2 * area_wing_m2))
