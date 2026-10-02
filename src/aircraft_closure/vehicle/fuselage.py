"""Cylindrical fuselage with conical nose and tail; Raymer GA mass (unpressurized)."""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer


@dataclass(frozen=True)
class Fuselage:
    length_m: Any = 7.0
    diameter_m: Any = 1.2
    nose_length_fraction: Any = 0.2
    tail_length_fraction: Any = 0.35
    x_nose_m: Any = 0.0
    z_m: Any = 0.0
    mass_factor: Any = 1.0

    def to_asb(self):
        radius_m = self.diameter_m / 2
        # Small end radii keep AeroSandbox's wetted-area and volume formulas regular.
        stations = [(0.0, 0.02), (self.nose_length_fraction, 1.0),
                    (1 - self.tail_length_fraction, 1.0), (1.0, 0.1)]
        return asb.Fuselage(name="fuselage", xsecs=[
            asb.FuselageXSec(xyz_c=[self.x_nose_m + fraction * self.length_m, 0, self.z_m],
                             radius=radius_fraction * radius_m)
            for fraction, radius_fraction in stations
        ])

    def get_mass_properties(self, condition, distance_wing_to_tail_m):
        """`distance_wing_to_tail_m`: wing to horizontal-tail root quarter-chord points."""
        mass_kg = self.mass_factor * raymer.mass_fuselage(
            self.to_asb(), design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, L_over_D=condition.lift_to_drag_cruise,
            cruise_op_point=condition.operating_point(), wing_to_tail_distance=distance_wing_to_tail_m)
        return asb.MassProperties(mass=mass_kg, x_cg=self.x_nose_m + 0.45 * self.length_m, z_cg=self.z_m)
