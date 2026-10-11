"""Parametric wing section: a CST (Kulfan) airfoil family in thickness ratio and camber (plan 046).

The family is one fitted base section (default NACA 2423, the Halo and XV-15 wing) whose CST weights are split into
camber and thickness parts and scaled (Kulfan 2008: the class/shape functions are linear in the weights, so
scaling the half-thickness weights scales the thickness distribution, the leading-edge radius with its square, as in
the NACA 4-digit family):

    upper = k_c * (u + l) / 2 + k_t * (u - l) / 2,   lower = k_c * (u + l) / 2 - k_t * (u - l) / 2,
    k_t = thickness_to_chord / t_base,   k_c = camber / camber_base.

The leading-edge weight is a camber mode (it adds to both surfaces) and scales with k_c; the trailing-edge
thickness scales with k_t. `thickness_to_chord` and `camber` may be CasADi expressions: `get_aero_from_neuralfoil`
then gives NeuralFoil section polars (with AeroSandbox's Mach corrections) that are differentiable in them, and
AeroBuildup uses them at every flight point at that point's Reynolds and Mach numbers. At the base thickness and
camber the section is the base airfoil's own CST fit, the one `asb.Airfoil.get_aero_from_neuralfoil` builds from its
coordinates (with the same chord-rotation offset of the angle of attack, scaled with camber), so the polars match
the coordinate airfoil.

`section_cl_max_ratio` is a smooth surrogate (a quadratic fitted to NeuralFoil section maximum lift over the family)
for the change in section maximum lift with thickness, used to scale the aircraft maximum lift coefficient of the
stall requirement; a maximum over angle of attack inside the optimizer would be a non-smooth max.
"""
from dataclasses import dataclass
from functools import lru_cache
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp


@lru_cache(maxsize=None)
def _base_parameters(name):
    """CST parameters of `name` exactly as `asb.Airfoil.get_aero_from_neuralfoil` builds them (normalized
    coordinates, 8 weights a side, leading-edge modification), the normalization's chord rotation in degrees (added
    to the angle of attack there) and the fit's measured maximum thickness."""
    normalized = asb.Airfoil(name).normalize(return_dict=True)
    base = normalized["airfoil"].to_kulfan_airfoil(n_weights_per_side=8, normalize_coordinates=False)
    x = onp.linspace(0, 1, 401)
    return (onp.asarray(base.upper_weights, dtype=float), onp.asarray(base.lower_weights, dtype=float),
            float(base.leading_edge_weight), float(base.TE_thickness), float(onp.max(base.local_thickness(x))),
            float(onp.ravel(normalized["rotation_angle"])[0]))


class SectionAirfoil(asb.KulfanAirfoil):
    """A member of `WingSectionFamily`; carries its (possibly symbolic) thickness and camber and the boundary-layer
    transition settings that AeroBuildup's NeuralFoil calls use (as `buildup.TransitionAirfoil`)."""

    def __init__(self, name, upper_weights, lower_weights, leading_edge_weight, TE_thickness, thickness_to_chord,
                 camber, alpha_offset_deg=0.0, xtr_upper=1.0, xtr_lower=1.0, n_crit=9.0):
        super().__init__(name=name, lower_weights=lower_weights, upper_weights=upper_weights,
                         leading_edge_weight=leading_edge_weight, TE_thickness=TE_thickness)
        self.thickness_to_chord, self.camber, self.alpha_offset_deg = thickness_to_chord, camber, alpha_offset_deg
        self.xtr_upper, self.xtr_lower, self.n_crit = xtr_upper, xtr_lower, n_crit

    def with_transition(self, xtr_upper, xtr_lower, n_crit):
        return SectionAirfoil(self.name, self.upper_weights, self.lower_weights, self.leading_edge_weight,
                              self.TE_thickness, self.thickness_to_chord, self.camber, self.alpha_offset_deg,
                              xtr_upper, xtr_lower, n_crit)

    def get_aero_from_neuralfoil(self, alpha, *args, **kwargs):
        """NeuralFoil polar; `alpha` is measured from the base airfoil's geometric chord (as for `asb.Airfoil`)."""
        kwargs.setdefault("xtr_upper", self.xtr_upper)
        kwargs.setdefault("xtr_lower", self.xtr_lower)
        kwargs.setdefault("n_crit", self.n_crit)
        return super().get_aero_from_neuralfoil(alpha + self.alpha_offset_deg, *args, **kwargs)

    def max_thickness(self, *args, **kwargs):
        """The family parameter (exact for the scaled base; smooth in a symbolic thickness)."""
        return self.thickness_to_chord

    def max_camber(self, *args, **kwargs):
        return self.camber


@dataclass(frozen=True)
class WingSectionFamily:
    """CST family around `base_name`; `camber_base` is its nominal maximum camber (2 % for NACA 2423)."""
    base_name: str = "naca2423"
    thickness_base: float = 0.23
    camber_base: float = 0.02

    def airfoil(self, thickness_to_chord=None, camber=None):
        """Section at `thickness_to_chord` and `camber` (numbers or CasADi expressions; None: the base values)."""
        upper, lower, leading_edge_weight, te_thickness, thickness_fit, rotation_deg = _base_parameters(
            self.base_name)
        thickness_to_chord = self.thickness_base if thickness_to_chord is None else thickness_to_chord
        camber = self.camber_base if camber is None else camber
        # Scale on the fit's own measured thickness so `max_thickness()` is exactly the parameter.
        k_t = thickness_to_chord / thickness_fit
        k_c = camber / self.camber_base
        mean, half = (upper + lower) / 2, (upper - lower) / 2
        return SectionAirfoil(f"{self.base_name} family", upper_weights=k_c * mean + k_t * half,
                              lower_weights=k_c * mean - k_t * half, leading_edge_weight=k_c * leading_edge_weight,
                              TE_thickness=k_t * te_thickness, thickness_to_chord=thickness_to_chord, camber=camber,
                              alpha_offset_deg=k_c * rotation_deg)


@dataclass(frozen=True)
class SectionClMaxSurrogate:
    """Section maximum lift relative to the base section, quadratic in thickness and linear in camber:
    ratio = 1 + a1 dt + a2 dt^2 + b1 dc with dt = t - t_base, dc = c - c_base. Coefficients from
    `fit_section_cl_max_ratio` (NeuralFoil, Re 8e6, Mach 0.18: the 120 kt stall point on the Halo chord)."""
    thickness_base: float = 0.23
    camber_base: float = 0.02
    coefficient_thickness: float = -0.9353          # fitted 2026-10-10 (NACA 2423 family, t 0.15-0.28, c 0-0.04)
    coefficient_thickness_sq: float = -0.4011
    coefficient_camber: float = 2.9716

    def ratio(self, thickness_to_chord, camber=None):
        dt = thickness_to_chord - self.thickness_base
        dc = 0.0 if camber is None else camber - self.camber_base
        return 1 + self.coefficient_thickness * dt + self.coefficient_thickness_sq * dt**2 \
            + self.coefficient_camber * dc


def section_cl_max(airfoil, reynolds, mach, alphas_deg=onp.linspace(0.0, 24.0, 97), model_size="large"):
    """Numerical section maximum lift from NeuralFoil over an angle-of-attack sweep (checks and fits only)."""
    aero = airfoil.get_aero_from_neuralfoil(alpha=alphas_deg, Re=reynolds, mach=mach, model_size=model_size)
    return float(onp.max(aero["CL"]))


def fit_section_cl_max_ratio(family=WingSectionFamily(), thicknesses=(0.15, 0.18, 0.21, 0.23, 0.25, 0.28),
                             cambers=(0.0, 0.02, 0.04), reynolds=8e6, mach=0.18):
    """Least-squares `SectionClMaxSurrogate` over the family (numerical; run once, coefficients stored)."""
    rows, values = [], []
    reference = section_cl_max(family.airfoil(), reynolds, mach)
    for t in thicknesses:
        for c in cambers:
            rows.append([t - family.thickness_base, (t - family.thickness_base)**2, c - family.camber_base])
            values.append(section_cl_max(family.airfoil(t, c), reynolds, mach) / reference - 1)
    coefficients, *_ = onp.linalg.lstsq(onp.array(rows), onp.array(values), rcond=None)
    return SectionClMaxSurrogate(family.thickness_base, family.camber_base, *map(float, coefficients))


if __name__ == "__main__":
    family = WingSectionFamily()
    for t in (0.15, 0.18, 0.21, 0.23, 0.25):
        aero = family.airfoil(t).get_aero_from_neuralfoil(alpha=2.0, Re=1.2e7, mach=0.3, xtr_upper=0.1,
                                                          xtr_lower=0.1, model_size="small")
        print(f"t/c {t:.2f}: Cl {onp.ravel(aero['CL'])[0]:.3f}  Cd {onp.ravel(aero['CD'])[0]:.5f}")
    print(fit_section_cl_max_ratio())
