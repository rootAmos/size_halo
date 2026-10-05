"""Pitch trim from the tail load and the drag it costs (plan 036).

The aerodynamics model gives the whole-aircraft pitching moment about the CG (AeroBuildup with the CG as its
moment reference: wing, fuselage, nacelles and tails at zero incidence). Trim adds the horizontal-tail lift
increment that brings it to zero (Scholz, Aircraft Design ch. 11, eqs. 11.12-11.27):

    0 = C_M,CG - eta_H (S_H / S) cos(Gamma_H) (l_H / c) dC_L,H

with the tail dynamic-pressure ratio eta_H (0.85-0.95, typically 0.9) and the tail dihedral Gamma_H (0 for a
conventional tail; the cosine takes the vertical component of a V-tail's normal force). The wing then carries
the rest of the lift, so trim drag is the tail's induced drag for its increment plus the change of the wing's
induced drag:

    dC_D = eta_H (S_H / S) dC_L,H^2 / (pi A_H e_H) + ((C_L - dC_L,t)^2 - C_L^2) / (pi A e),
    dC_L,t = eta_H (S_H / S) cos(Gamma_H) dC_L,H.

AeroBuildup's tail sees the free-stream angle of attack (it has no wing downwash at the tail), so its tail lift and
nose-down moment are too large; the moment is corrected by the downwash the tail would see (Scholz eq. 11.29, the
bracket taken as 1.75 for typical geometries): eps = 1.75 C_L,W / (pi A), dC_M = + eta_H (S_H / S) cos(Gamma_H)
(l_H / c) C_L,alpha,H eps. The CG is taken at the wing's quarter root chord, where the Halo's hover pitch trim places it.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class TailTrim:
    efficiency_tail: Any = 0.9               # eta_H, Scholz ch. 11 (0.85-0.95; typical 0.9)
    angle_dihedral_tail_deg: Any = 0.0       # Gamma_H: 0 conventional tail; a V-tail's dihedral otherwise
    oswald_tail: Any = 0.8                   # assumed
    factor_downwash: Any = 1.75              # Scholz eq. 11.29 bracket for typical geometries

    @staticmethod
    def x_cg_m(aircraft):
        """CG at the wing's quarter root chord (the Halo hover pitch trim)."""
        return aircraft.wing.x_le_root_m + 0.25 * aircraft.wing.chord_root_m()

    def volume_tail(self, aircraft, chord_ref_m):
        """eta_H (S_H / S) cos(Gamma_H) l_H / c: tail moment per unit tail lift coefficient."""
        tail = aircraft.horizontal_tail
        arm_tail_m = tail.x_le_root_m + 0.25 * tail.chord_root_m() - self.x_cg_m(aircraft)
        return (self.efficiency_tail * tail.area_m2 / aircraft.wing.area_m2 * np.cosd(self.angle_dihedral_tail_deg)
                * arm_tail_m / chord_ref_m)

    def moment_downwash_correction(self, aircraft, cl, chord_ref_m, aspect_ratio_wing, cl_alpha_tail_per_rad):
        """+ dC_M from the wing downwash AeroBuildup leaves out at the tail."""
        downwash_rad = self.factor_downwash * cl / (np.pi * aspect_ratio_wing)
        return self.volume_tail(aircraft, chord_ref_m) * cl_alpha_tail_per_rad * downwash_rad

    def lift_tail_increment(self, aircraft, cm_cg, chord_ref_m):
        """dC_L,H (on the tail's own area) that trims `cm_cg`."""
        return cm_cg / self.volume_tail(aircraft, chord_ref_m)

    def drag_coefficient(self, aircraft, cl, cm_cg, chord_ref_m, oswald_wing, aspect_ratio_wing,
                         cl_alpha_tail_per_rad=0.0):
        """Trim drag coefficient on the wing reference area (`cm_cg` before the downwash correction)."""
        tail = aircraft.horizontal_tail
        area_ratio = self.efficiency_tail * tail.area_m2 / aircraft.wing.area_m2
        cm_cg = cm_cg + self.moment_downwash_correction(aircraft, cl, chord_ref_m, aspect_ratio_wing,
                                                        cl_alpha_tail_per_rad)
        lift_tail = self.lift_tail_increment(aircraft, cm_cg, chord_ref_m)
        cl_tail_ref = area_ratio * np.cosd(self.angle_dihedral_tail_deg) * lift_tail
        cd_tail = area_ratio * lift_tail**2 / (np.pi * tail.aspect_ratio * self.oswald_tail)
        cd_wing = ((cl - cl_tail_ref)**2 - cl**2) / (np.pi * aspect_ratio_wing * oswald_wing)
        return cd_tail + cd_wing
