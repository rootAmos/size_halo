"""Scholz level-0 drag build-up: the independent hand check of the AeroBuildup model (Tier 21, plan 025).

D. Scholz, *Aircraft Design*, ch. 13 "Drag Prediction" (HAW Hamburg lecture notes):

    C_D0 = sum_c C_f,c FF_c Q_c S_wet,c / S_ref + C_D,misc                       (13.15)

* C_f: turbulent flat plate with the Mach term (13.17). AeroSandbox's `Cf_flat_plate` has no Mach term,
  which is the reason this one is written here.
* FF: DATCOM/Raymer surface form factor (13.22, unswept), DATCOM fuselage form factor (13.23), Raymer nacelle
  1 + 0.35 / (l/d) (13.24).
* Q: interference factors of Table 13.4 (`InterferenceFactors`).
* S_wet: Torenbeek, fuselage with a cylindrical mid-section (13.8), wing (13.10) on the exposed area.
* Wave drag: Korn equation with Lock's 20 (M - M_crit)^4 (AeroSandbox `Cd_wave_Korn`, kappa_A 0.87 for
  conventional sections).
* Oswald factor: Nita and Scholz (DLRK 2012). AeroSandbox's `oswalds_efficiency` implements it with the mean
  k_e,D0 of four aircraft classes and k_e,M = 1; here k_e,D0 is the turboprop value 0.804 and the
  compressibility factor k_e,M = -0.00152 (M / 0.3 - 1)^10.82 + 1 above M = 0.3 is applied.

Lift is linear as in `SimpleAerodynamics`, which this class extends.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.library.aerodynamics.inviscid import oswalds_efficiency
from aerosandbox.library.aerodynamics.transonic import Cd_wave_Korn

from aircraft_closure.aerodynamics.download import HoverDownload, hover_download_fraction
from aircraft_closure.aerodynamics.simple import AeroResult, ParasiteDragItem, SimpleAerodynamics

# Mean k_e,D0 used by AeroSandbox's `oswalds_efficiency` (jet, business jet, turboprop, general aviation).
_k_e_d0_aerosandbox = (0.873 + 0.864 + 0.804 + 0.804) / 4


def skin_friction_turbulent(reynolds, mach):
    """Scholz eq. 13.17 (DATCOM 4.1.5.1-26, Raymer 12.27): fully turbulent flat plate with compressibility."""
    return 0.455 / (np.log10(reynolds)**2.58 * (1 + 0.144 * mach**2)**0.65)


def form_factor_surface(thickness_to_chord, thickness_location_chordwise, mach):
    """Scholz eq. 13.22, unswept."""
    return ((1 + 0.6 / thickness_location_chordwise * thickness_to_chord + 100 * thickness_to_chord**4)
            * 1.34 * mach**0.18)


def form_factor_fuselage(fineness_ratio):
    """Scholz eq. 13.23 (DATCOM 4.2.3.1)."""
    return 1 + 60 / fineness_ratio**3 + fineness_ratio / 400


def form_factor_nacelle(fineness_ratio):
    """Scholz eq. 13.24 (Raymer)."""
    return 1 + 0.35 / fineness_ratio


def area_wetted_fuselage_m2(length_m, diameter_m):
    """Torenbeek, Scholz eq. 13.8: fuselage with a cylindrical mid-section (fineness >= 4.5)."""
    fineness_ratio = length_m / diameter_m
    return np.pi * diameter_m * length_m * (1 - 2 / fineness_ratio)**(2 / 3) * (1 + 1 / fineness_ratio**2)


def area_wetted_surface_m2(area_exposed_m2, thickness_to_chord_root, thickness_ratio_tip_to_root=1.0,
                           taper_ratio=1.0):
    """Torenbeek, Scholz eq. 13.10."""
    return 2 * area_exposed_m2 * (1 + 0.25 * thickness_to_chord_root
                                  * (1 + thickness_ratio_tip_to_root * taper_ratio) / (1 + taper_ratio))


def oswald_nita_scholz(taper_ratio, aspect_ratio, fuselage_diameter_to_span_ratio, mach, k_e_d0=0.804):
    """Nita-Scholz e = e_theo k_e,F k_e,D0 k_e,M (unswept), reusing AeroSandbox for e_theo k_e,F."""
    k_e_m = 1 - 0.00152 * np.fmax(mach / 0.3 - 1, 0)**10.82
    return (oswalds_efficiency(taper_ratio, aspect_ratio,
                               fuselage_diameter_to_span_ratio=fuselage_diameter_to_span_ratio)
            * k_e_d0 / _k_e_d0_aerosandbox * k_e_m)


@dataclass(frozen=True)
class InterferenceFactors:
    """Scholz Table 13.4. Nacelle 1.5: engine mounted directly on the wing (tip nacelles); 1.3 closer than one
    nacelle diameter, 1.0 farther. Wing 1.0 with a wing-fuselage fairing (1.1-1.4 low wing without). Tails:
    conventional 1.04, H-tail 1.08, V-tail 1.03. The fuselage has none."""
    wing: Any = 1.0
    horizontal_tail: Any = 1.04
    vertical_tail: Any = 1.04
    fuselage: Any = 1.0
    nacelle: Any = 1.5


@dataclass(frozen=True)
class ScholzAerodynamics(SimpleAerodynamics):
    """Level-0 component build-up with linear lift; interchangeable with `SimpleAerodynamics`.

    `drag_area_misc_m2`: C_D,misc + C_D,L+P as a drag area (fittings, antennas, leakage).
    `drag_area_landing_gear_fixed_m2`: added when the aircraft's gear is not retractable.
    `download_fraction_hover` None: the hover download comes from wing and rotor geometry (`download`).
    `factor_excrescence` (plan 034): (factor - 1) x the component drag as a separate excrescence item, for leakage,
    protuberances and installation; 1.17 matches the XV-15 components to NDARC's 6.25 ft2 (Johnson 2010).
    `fraction_trim_drag` (plan 034): trim drag / (parasite + induced), added to CD.
    """
    interference: Any = InterferenceFactors()
    drag_area_misc_m2: Any = 0.0
    # NDARC UH-60A (fixed gear, 16,500 lb): landing gear D/q 5.63 ft2 (Johnson 2010, Table 1).
    drag_area_landing_gear_fixed_m2: Any = 5.63 * 0.09290304
    k_e_d0: Any = 0.804                      # Nita-Scholz turboprop
    kappa_korn: Any = 0.87                   # Korn technology factor, conventional (NACA 6-series) sections
    download_fraction_hover: Any = None
    download: Any = HoverDownload()
    factor_excrescence: Any = 1.0
    fraction_trim_drag: Any = 0.0

    def hover_download_fraction(self, aircraft):
        if self.download_fraction_hover is not None:
            return self.download_fraction_hover
        return hover_download_fraction(aircraft, self.download)

    def parasite_drag_breakdown(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        density_kg_m3, viscosity_Pa_s, mach, _ = self._flow(velocity_m_s, altitude_m, temperature_offset_K)
        area_ref_m2 = aircraft.wing.area_m2
        q = self.interference

        def friction(length_m):
            return skin_friction_turbulent(density_kg_m3 * velocity_m_s * length_m / viscosity_Pa_s, mach)

        def surface(label, component, area_exposed_m2, interference):
            asb_wing = component.to_asb()
            thickness = component.airfoil.max_thickness()
            area_wetted_m2 = area_wetted_surface_m2(area_exposed_m2, thickness, 1.0, component.taper_ratio)
            return ParasiteDragItem(label, friction(asb_wing.mean_aerodynamic_chord())
                                    * form_factor_surface(thickness, self.thickness_location_chordwise, mach)
                                    * interference * area_wetted_m2 / area_ref_m2)

        wing, fuselage = aircraft.wing, aircraft.fuselage
        area_exposed_wing_m2 = wing.area_m2 - wing.chord_root_m() * fuselage.diameter_m
        items = [
            surface("wing", wing, area_exposed_wing_m2, q.wing),
            surface("horizontal_tail", aircraft.horizontal_tail, aircraft.horizontal_tail.area_m2, q.horizontal_tail),
            surface("vertical_tail", aircraft.vertical_tail, aircraft.vertical_tail.area_m2, q.vertical_tail),
            ParasiteDragItem("fuselage", friction(fuselage.length_m)
                             * form_factor_fuselage(fuselage.length_m / fuselage.diameter_equivalent_m()) * q.fuselage
                             * area_wetted_fuselage_m2(fuselage.length_m, fuselage.diameter_equivalent_m()) / area_ref_m2),
        ]
        nacelles = aircraft.nacelles
        if nacelles is not None and getattr(nacelles, "length_m", None) is not None:
            items.append(ParasiteDragItem("nacelles", friction(nacelles.length_m)
                                          * form_factor_nacelle(nacelles.length_m / nacelles.diameter_m)
                                          * q.nacelle * nacelles.area_wetted_m2 / area_ref_m2))
        if not (isinstance(self.factor_excrescence, (int, float)) and self.factor_excrescence == 1.0):
            items.append(ParasiteDragItem("excrescence", (self.factor_excrescence - 1) * sum(i.cd0 for i in items)))
        items.append(ParasiteDragItem("miscellaneous", self.drag_area_misc_m2 / area_ref_m2))
        if not aircraft.landing_gear.is_retractable:
            items.append(ParasiteDragItem("landing_gear", self.drag_area_landing_gear_fixed_m2 / area_ref_m2))
        return tuple(items)

    def oswald_efficiency(self, aircraft, mach=0.0):
        wing = aircraft.wing
        return oswald_nita_scholz(wing.taper_ratio, wing.aspect_ratio, aircraft.fuselage.diameter_m / wing.span_m(),
                                  mach, self.k_e_d0)

    def wave_drag_coefficient(self, aircraft, cl, mach):
        return Cd_wave_Korn(cl, aircraft.wing.airfoil.max_thickness(), mach, sweep=0, kappa_A=self.kappa_korn)

    def evaluate(self, aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(), temperature_offset_K=0.0,
                 rotor_state=None):
        """`rotor_state` is accepted for interchangeability; this model has no slipstream increments."""
        _, _, mach, dynamic_pressure_Pa = self._flow(velocity_m_s, altitude_m, temperature_offset_K)
        cl_alpha_per_rad = self.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m, temperature_offset_K)
        cl = cl_alpha_per_rad * np.radians(alpha_deg - self.alpha_zero_lift_deg)
        oswald = self.oswald_efficiency(aircraft, mach)
        cd0 = sum(item.cd0 for item in self.parasite_drag_breakdown(aircraft, velocity_m_s, altitude_m,
                                                                    temperature_offset_K))
        cdi = cl**2 / (np.pi * oswald * aircraft.wing.aspect_ratio)
        cd = (cd0 + cdi + self.wave_drag_coefficient(aircraft, cl, mach)
              + sum(increment.cd for increment in drag_increments))
        if not (isinstance(self.fraction_trim_drag, (int, float)) and self.fraction_trim_drag == 0.0):
            cd = cd + self.fraction_trim_drag * (cd0 + cdi)
        area_ref_m2 = aircraft.wing.area_m2
        return AeroResult(alpha_deg=alpha_deg, cl=cl, cd=cd, cd0=cd0, cdi=cdi, cl_alpha_per_rad=cl_alpha_per_rad,
                          oswald_efficiency=oswald, dynamic_pressure_Pa=dynamic_pressure_Pa, mach=mach,
                          lift_N=dynamic_pressure_Pa * area_ref_m2 * cl, drag_N=dynamic_pressure_Pa * area_ref_m2 * cd,
                          lift_to_drag=cl / cd)


if __name__ == "__main__":
    print(skin_friction_turbulent(1e7, 0.3), form_factor_nacelle(9 / 3.3), asb.Atmosphere(0).density())
