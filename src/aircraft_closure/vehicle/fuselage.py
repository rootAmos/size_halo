"""Constant-section fuselage with conical nose and tail; Raymer GA mass (unpressurized).

The section is round (`diameter_m`) unless `height_m` is given; then it is a
super-ellipse `diameter_m` wide and `height_m` deep with exponent `shape`
(2: ellipse; larger is boxier), as an unpressurized cargo fuselage. Models
that need the width keep using `diameter_m`; fineness ratios use
`diameter_equivalent_m()`.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer
import aerosandbox.numpy as np


@dataclass(frozen=True)
class Fuselage:
    length_m: Any = 7.0
    diameter_m: Any = 1.2
    nose_length_fraction: Any = 0.2
    tail_length_fraction: Any = 0.35
    x_nose_m: Any = 0.0
    z_m: Any = 0.0
    mass_factor: Any = 1.0
    height_m: Any = None          # None: round section of `diameter_m`
    shape: Any = 2.0              # super-ellipse exponent of a non-round section

    def height_section_m(self):
        return self.diameter_m if self.height_m is None else self.height_m

    def diameter_equivalent_m(self):
        """Geometric-mean diameter (the diameter itself for a round section)."""
        return self.diameter_m if self.height_m is None else np.sqrt(self.diameter_m * self.height_m)

    def to_asb(self):
        # Small end sections keep AeroSandbox's wetted-area and volume formulas regular.
        stations = [(0.0, 0.02), (self.nose_length_fraction, 1.0),
                    (1 - self.tail_length_fraction, 1.0), (1.0, 0.1)]
        if self.height_m is None:
            sections = [dict(radius=fraction_size * self.diameter_m / 2) for _, fraction_size in stations]
        else:
            sections = [dict(width=fraction_size * self.diameter_m, height=fraction_size * self.height_m,
                             shape=self.shape) for _, fraction_size in stations]
        return asb.Fuselage(name="fuselage", xsecs=[
            asb.FuselageXSec(xyz_c=[self.x_nose_m + fraction * self.length_m, 0, self.z_m], **section)
            for (fraction, _), section in zip(stations, sections)
        ])

    def get_mass_properties(self, condition, distance_wing_to_tail_m):
        """`distance_wing_to_tail_m`: wing to horizontal-tail root quarter-chord points."""
        mass_kg = self.mass_factor * raymer.mass_fuselage(
            self.to_asb(), design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, L_over_D=condition.lift_to_drag_cruise,
            cruise_op_point=condition.operating_point(), wing_to_tail_distance=distance_wing_to_tail_m)
        return asb.MassProperties(mass=mass_kg, x_cg=self.x_nose_m + 0.45 * self.length_m, z_cg=self.z_m)
