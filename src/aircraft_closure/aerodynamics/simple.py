"""Simple analytic aerodynamics: linear lift, parasite buildup, induced drag.

Geometry is read from the physical components (`wing`, `horizontal_tail`,
`vertical_tail`, `fuselage`, each with `to_asb()`); this discipline imports no
vehicle module. AeroSandbox supplies skin friction, the 3D lift-slope ratio,
Oswald efficiency and the fuselage form factor. Wing lift only: tail lift and
trim belong to Tier 6. Valid for attached flow below the stated CLmax and
cruise Mach below about 0.3.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.aerodynamics.aero_3D.aero_buildup_submodels.fuselage_aerodynamics_utilities import (
    fuselage_form_factor)
from aerosandbox.library.aerodynamics.inviscid import CL_over_Cl, oswalds_efficiency
from aerosandbox.library.aerodynamics.viscous import Cf_flat_plate


@dataclass(frozen=True)
class DragIncrement:
    """Additive drag coefficient on the wing reference area (extension hook)."""
    label: str
    cd: Any


@dataclass(frozen=True)
class ParasiteDragItem:
    label: str
    cd0: Any


@dataclass(frozen=True)
class AeroResult:
    alpha_deg: Any
    cl: Any
    cd: Any
    cd0: Any
    cdi: Any
    cl_alpha_per_rad: Any
    oswald_efficiency: Any
    dynamic_pressure_Pa: Any
    mach: Any
    lift_N: Any
    drag_N: Any
    lift_to_drag: Any


def _surface_form_factor(thickness_to_chord, thickness_location_chordwise, mach):
    """Raymer eq. 12.30 for an unswept surface."""
    return ((1 + 0.6 / thickness_location_chordwise * thickness_to_chord + 100 * thickness_to_chord**4)
            * 1.34 * mach**0.18)


@dataclass(frozen=True)
class SimpleAerodynamics:
    alpha_zero_lift_deg: Any = -4.0
    cl_max: Any = 1.5
    thickness_location_chordwise: Any = 0.3
    interference_wing: Any = 1.0
    interference_tail: Any = 1.05
    interference_fuselage: Any = 1.0
    drag_area_misc_m2: Any = 0.25
    # Fraction of hover rotor thrust lost to airframe download (wing under the
    # rotor wake); XV-15: 0.07 with flaps deflected (NASA TM X-62407 sec. 5.1).
    download_fraction_hover: Any = 0.0

    def hover_download_fraction(self, aircraft):
        """A constant fraction (Tier 10b); `BuildupAerodynamics` and `ScholzAerodynamics` can use geometry."""
        return self.download_fraction_hover

    # `temperature_offset_K` (Tier 16): ambient minus ISA temperature at the pressure altitude; 0 = standard day.
    def _flow(self, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        atmosphere = asb.Atmosphere(altitude=altitude_m, temperature_deviation=temperature_offset_K)
        density_kg_m3 = atmosphere.density()
        return (density_kg_m3, atmosphere.dynamic_viscosity(), velocity_m_s / atmosphere.speed_of_sound(),
                0.5 * density_kg_m3 * velocity_m_s**2)

    def parasite_drag_breakdown(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        density_kg_m3, viscosity_Pa_s, mach, _ = self._flow(velocity_m_s, altitude_m, temperature_offset_K)
        wing = aircraft.wing.to_asb()
        area_ref_m2 = wing.area()

        def surface(label, asb_wing, interference):
            chord_m = asb_wing.mean_aerodynamic_chord()
            reynolds = density_kg_m3 * velocity_m_s * chord_m / viscosity_Pa_s
            thickness = np.min(np.array([xsec.airfoil.max_thickness() for xsec in asb_wing.xsecs]))
            form_factor = _surface_form_factor(thickness, self.thickness_location_chordwise, mach)
            return ParasiteDragItem(label, Cf_flat_plate(reynolds) * form_factor * interference
                                    * asb_wing.area("wetted") / area_ref_m2)

        fuselage = aircraft.fuselage.to_asb()
        reynolds_fuselage = density_kg_m3 * velocity_m_s * fuselage.length() / viscosity_Pa_s
        fineness_ratio = aircraft.fuselage.length_m / aircraft.fuselage.diameter_m
        return (
            surface("wing", wing, self.interference_wing),
            surface("horizontal_tail", aircraft.horizontal_tail.to_asb(), self.interference_tail),
            surface("vertical_tail", aircraft.vertical_tail.to_asb(), self.interference_tail),
            ParasiteDragItem("fuselage", Cf_flat_plate(reynolds_fuselage) * fuselage_form_factor(fineness_ratio)
                             * self.interference_fuselage * fuselage.area_wetted() / area_ref_m2),
            ParasiteDragItem("miscellaneous", self.drag_area_misc_m2 / area_ref_m2),
        )

    def surface_lift_curve_slope_per_rad(self, aspect_ratio, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        """Finite-surface slope 2 pi CL_over_Cl(AR, M); also used for the tails (Tier 6)."""
        _, _, mach, _ = self._flow(velocity_m_s, altitude_m, temperature_offset_K)
        return 2 * np.pi * CL_over_Cl(aspect_ratio, mach=mach)

    def lift_curve_slope_per_rad(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        return self.surface_lift_curve_slope_per_rad(aircraft.wing.aspect_ratio, velocity_m_s, altitude_m,
                                                     temperature_offset_K)

    def oswald_efficiency(self, aircraft):
        wing = aircraft.wing
        return oswalds_efficiency(wing.taper_ratio, wing.aspect_ratio,
                                  fuselage_diameter_to_span_ratio=aircraft.fuselage.diameter_m / wing.span_m())

    def alpha_stall_deg(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0, aero=None):
        """Linear-lift angle at CLmax; a caller-side upper bound on alpha. `aero` (the point's `AeroResult`) is
        accepted for interchangeability with `BuildupAerodynamics`; linear lift does not need it."""
        return self.alpha_zero_lift_deg + np.degrees(
            self.cl_max / self.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m, temperature_offset_K))

    def evaluate(self, aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(), temperature_offset_K=0.0,
                 rotor_state=None):
        """`rotor_state` is accepted for interchangeability with `BuildupAerodynamics`; it is not used."""
        _, _, mach, dynamic_pressure_Pa = self._flow(velocity_m_s, altitude_m, temperature_offset_K)
        cl_alpha_per_rad = self.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m, temperature_offset_K)
        cl = cl_alpha_per_rad * np.radians(alpha_deg - self.alpha_zero_lift_deg)
        oswald = self.oswald_efficiency(aircraft)
        cd0 = sum(item.cd0 for item in self.parasite_drag_breakdown(aircraft, velocity_m_s, altitude_m,
                                                                    temperature_offset_K))
        cdi = cl**2 / (np.pi * oswald * aircraft.wing.aspect_ratio)
        cd = cd0 + cdi + sum(increment.cd for increment in drag_increments)
        area_ref_m2 = aircraft.wing.area_m2
        return AeroResult(alpha_deg=alpha_deg, cl=cl, cd=cd, cd0=cd0, cdi=cdi, cl_alpha_per_rad=cl_alpha_per_rad,
                          oswald_efficiency=oswald, dynamic_pressure_Pa=dynamic_pressure_Pa, mach=mach,
                          lift_N=dynamic_pressure_Pa * area_ref_m2 * cl, drag_N=dynamic_pressure_Pa * area_ref_m2 * cd,
                          lift_to_drag=cl / cd)
