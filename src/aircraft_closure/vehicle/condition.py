"""Structural design condition shared by the empirical mass correlations."""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb


@dataclass(frozen=True)
class StructuralDesignCondition:
    """Design gross mass and the flight condition the correlations are sized at.

    `mass_design_kg` is normally the caller's take-off mass variable, which is
    how the mass closure couples back into every correlation. Lift-to-drag is
    an assumption until aerodynamics exists (Tier 5); Raymer's fuselage
    correlation depends on it only weakly (exponent -0.072).
    """
    mass_design_kg: Any
    load_factor_ultimate: Any = 5.7
    velocity_cruise_m_s: Any = 60.0
    altitude_cruise_m: Any = 1000.0
    lift_to_drag_cruise: Any = 12.0

    def operating_point(self):
        return asb.OperatingPoint(atmosphere=asb.Atmosphere(altitude=self.altitude_cruise_m),
                                  velocity=self.velocity_cruise_m_s)
