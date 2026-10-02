"""Unswept trapezoidal lifting surfaces with Raymer general-aviation masses.

Geometry is parameterized by planform area, aspect ratio and taper ratio and
handed to `asb.Wing`, which owns span, MAC and area calculations. Masses call
AeroSandbox's Raymer correlations (Aircraft Design: A Conceptual Approach,
5th ed., sec. 15.3.3; general aviation scope) on that geometry; `mass_factor`
is a dimensionless calibration multiplier. CG sits at 40 % of the MAC.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer
import aerosandbox.numpy as np


def _trapezoid_chords_m(area_m2, aspect_ratio, taper_ratio):
    """Span (one panel pair or one fin) and root/tip chords of a trapezoid."""
    span_m = np.sqrt(area_m2 * aspect_ratio)
    chord_root_m = 2 * area_m2 / (span_m * (1 + taper_ratio))
    return span_m, chord_root_m, chord_root_m * taper_ratio


def _cg_x_m(asb_wing, x_le_root_m):
    # Unswept: the MAC leading edge lies at the root leading-edge x.
    return x_le_root_m + 0.4 * asb_wing.mean_aerodynamic_chord()


class _SymmetricSurface:
    """Shared planform geometry of the symmetric surfaces (wing, horizontal tail)."""
    asb_name = ""

    def span_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[0]

    def chord_root_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[1]

    def to_asb(self):
        span_m, chord_root_m, chord_tip_m = _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)
        return asb.Wing(name=self.asb_name, symmetric=True, xsecs=[
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_m], chord=chord_root_m, airfoil=self.airfoil),
            asb.WingXSec(xyz_le=[self.x_le_root_m, span_m / 2, self.z_m], chord=chord_tip_m, airfoil=self.airfoil),
        ])


@dataclass(frozen=True)
class Wing(_SymmetricSurface):
    asb_name = "wing"
    area_m2: Any = 12.0
    aspect_ratio: Any = 9.0
    taper_ratio: Any = 1.0
    x_le_root_m: Any = 2.5
    z_m: Any = 0.6
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca4418"))
    mass_factor: Any = 1.0

    def get_mass_properties(self, condition):
        wing = self.to_asb()
        mass_kg = self.mass_factor * raymer.mass_wing(
            wing, design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate,
            mass_fuel_in_wing=0, cruise_op_point=condition.operating_point())
        return asb.MassProperties(mass=mass_kg, x_cg=_cg_x_m(wing, self.x_le_root_m), z_cg=self.z_m)


@dataclass(frozen=True)
class HorizontalTail(_SymmetricSurface):
    asb_name = "horizontal_tail"
    area_m2: Any = 2.4
    aspect_ratio: Any = 4.5
    taper_ratio: Any = 0.7
    x_le_root_m: Any = 6.2
    z_m: Any = 0.3
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca0012"))
    mass_factor: Any = 1.0

    def get_mass_properties(self, condition):
        tail = self.to_asb()
        mass_kg = self.mass_factor * raymer.mass_hstab(
            tail, design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, cruise_op_point=condition.operating_point())
        return asb.MassProperties(mass=mass_kg, x_cg=_cg_x_m(tail, self.x_le_root_m), z_cg=self.z_m)


@dataclass(frozen=True)
class VerticalTail:
    """Single fin; aspect ratio is height^2 / area."""
    area_m2: Any = 1.6
    aspect_ratio: Any = 1.6
    taper_ratio: Any = 0.6
    x_le_root_m: Any = 6.0
    z_root_m: Any = 0.5
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca0012"))
    mass_factor: Any = 1.0

    def height_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[0]

    def chord_root_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[1]

    def to_asb(self):
        height_m, chord_root_m, chord_tip_m = _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)
        return asb.Wing(name="vertical_tail", symmetric=False, xsecs=[
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_root_m], chord=chord_root_m, airfoil=self.airfoil),
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_root_m + height_m], chord=chord_tip_m,
                         airfoil=self.airfoil),
        ])

    def get_mass_properties(self, condition):
        fin = self.to_asb()
        mass_kg = self.mass_factor * raymer.mass_vstab(
            fin, design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, cruise_op_point=condition.operating_point())
        return asb.MassProperties(mass=mass_kg, x_cg=_cg_x_m(fin, self.x_le_root_m),
                                  z_cg=self.z_root_m + 0.4 * self.height_m())
