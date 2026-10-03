"""AeroSandbox `AeroBuildup` aerodynamics with the items it does not model (Tier 21, plan 025).

`asb.AeroBuildup` runs on `aircraft.to_asb()`: wing and tails with NeuralFoil section polars (Reynolds, Mach,
post-stall), the fuselage and any nacelle bodies with AeroSandbox's form factors, and the induced drag of the
whole-aircraft lift. Interchangeable with `SimpleAerodynamics` (same `evaluate`, `alpha_stall_deg`, slope
methods, `cl_max` and hover download). Added on top, because AeroBuildup does not provide them:

* boundary-layer transition: AeroBuildup does not pass a transition location to NeuralFoil; the airfoils are
  wrapped (`TransitionAirfoil`) so it does. Default 10 % chord on both surfaces (Scholz ch. 13: "laminar flow
  may exist on the front 10 % to 20 % of the wing"); 0 is fully turbulent;
* interference: (Q - 1) x each component's profile drag, Q from Scholz Table 13.4;
* the Nita-Scholz fuselage factor k_e,F = 1 - 2 (d_F / b)^2 and compressibility factor k_e,M on the Oswald
  factor (AeroBuildup takes the fuselage diameter as zero and k_e,M as 1);
* a miscellaneous drag area and fixed landing gear;
* blown-wing increments (`BlownWing`) when the caller gives the rotor state (airplane mode);
* hover download from wing and rotor geometry (`HoverDownload`) unless a constant fraction is given.

Lift is the whole aircraft's at zero tail incidence (AeroBuildup has no downwash); the stability methods
(`lift_curve_slope_per_rad`, `surface_lift_curve_slope_per_rad`) stay the Tier 6 analytic isolated-surface
slopes. `cd0` is the parasite (profile, interference, miscellaneous) drag at the operating point, not at zero lift.
`alpha_stall_deg` is where the AeroBuildup lift curve, linearized through two reference angles, reaches `cl_max`.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.aerodynamics.aero_3D.aero_buildup_submodels.softmax_scalefree import softmax_scalefree
from aerosandbox.library.aerodynamics.inviscid import CL_over_Cl

from aircraft_closure.aerodynamics.download import HoverDownload, hover_download_fraction
from aircraft_closure.aerodynamics.scholz import InterferenceFactors
from aircraft_closure.aerodynamics.simple import AeroResult, ParasiteDragItem


class TransitionAirfoil(asb.Airfoil):
    """An `asb.Airfoil` whose NeuralFoil calls carry a transition location and amplification factor."""

    def __init__(self, airfoil, xtr_upper=1.0, xtr_lower=1.0, n_crit=9.0):
        super().__init__(name=airfoil.name, coordinates=airfoil.coordinates)
        self.xtr_upper, self.xtr_lower, self.n_crit = xtr_upper, xtr_lower, n_crit

    def get_aero_from_neuralfoil(self, *args, **kwargs):
        kwargs.setdefault("xtr_upper", self.xtr_upper)
        kwargs.setdefault("xtr_lower", self.xtr_lower)
        kwargs.setdefault("n_crit", self.n_crit)
        return super().get_aero_from_neuralfoil(*args, **kwargs)


@dataclass(frozen=True)
class BuildupResult:
    """AeroBuildup output and the added items, all coefficients on the wing reference area."""
    aero: AeroResult
    breakdown: tuple        # ParasiteDragItem per component and added item (profile x Q, misc, gear, blown)
    cl_aerobuildup: Any
    oswald_span: Any        # AeroBuildup's span efficiency (s_eff / b)^2
    blown: Any = None       # BlownWingIncrement or None


@dataclass(frozen=True)
class BuildupAerodynamics:
    cl_max: Any = 1.5
    interference: Any = InterferenceFactors()
    drag_area_misc_m2: Any = 0.0
    drag_area_landing_gear_fixed_m2: Any = 5.63 * 0.09290304   # NDARC UH-60A fixed gear (Johnson 2010)
    xtr_upper: Any = 0.1
    xtr_lower: Any = 0.1
    n_crit: Any = 9.0
    model_size: str = "small"
    blown_wing: Any = None                       # BlownWing; None: no slipstream increments
    download_fraction_hover: Any = None          # None: from geometry (`download`)
    download: Any = field(default_factory=HoverDownload)
    alpha_reference_stall_deg: tuple = (0.0, 8.0)

    def hover_download_fraction(self, aircraft):
        if self.download_fraction_hover is not None:
            return self.download_fraction_hover
        return hover_download_fraction(aircraft, self.download)

    def to_asb(self, aircraft):
        airplane = aircraft.to_asb()
        for wing in airplane.wings:
            for xsec in wing.xsecs:
                xsec.airfoil = TransitionAirfoil(xsec.airfoil, self.xtr_upper, self.xtr_lower, self.n_crit)
        return airplane

    @staticmethod
    def _operating_point(velocity_m_s, altitude_m, alpha_deg, temperature_offset_K):
        return asb.OperatingPoint(atmosphere=asb.Atmosphere(altitude=altitude_m,
                                                            temperature_deviation=temperature_offset_K),
                                  velocity=velocity_m_s, alpha=alpha_deg)

    def _run(self, airplane, operating_point):
        return asb.AeroBuildup(airplane=airplane, op_point=operating_point, model_size=self.model_size).run()

    def surface_lift_curve_slope_per_rad(self, aspect_ratio, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        """Analytic isolated-surface slope 2 pi CL_over_Cl(AR, M), as `SimpleAerodynamics` (Tier 6 stability)."""
        atmosphere = asb.Atmosphere(altitude=altitude_m, temperature_deviation=temperature_offset_K)
        return 2 * np.pi * CL_over_Cl(aspect_ratio, mach=velocity_m_s / atmosphere.speed_of_sound())

    def lift_curve_slope_per_rad(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0):
        return self.surface_lift_curve_slope_per_rad(aircraft.wing.aspect_ratio, velocity_m_s, altitude_m,
                                                     temperature_offset_K)

    def alpha_stall_deg(self, aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0, aero=None):
        """With the point's `aero`: alpha + (cl_max - CL) / CL_alpha, so alpha <= alpha_stall is exactly
        CL <= cl_max at the point (no extra AeroBuildup run). Without it: the AeroBuildup lift curve linearized
        through `alpha_reference_stall_deg` (two extra runs)."""
        if aero is not None:
            return aero.alpha_deg + np.degrees((self.cl_max - aero.cl) / aero.cl_alpha_per_rad)
        airplane = self.to_asb(aircraft)
        alpha_a_deg, alpha_b_deg = self.alpha_reference_stall_deg
        cl_a, cl_b = (self._run(airplane, self._operating_point(velocity_m_s, altitude_m, alpha_deg,
                                                                temperature_offset_K))["CL"]
                      for alpha_deg in (alpha_a_deg, alpha_b_deg))
        return alpha_a_deg + (self.cl_max - cl_a) * (alpha_b_deg - alpha_a_deg) / (cl_b - cl_a)

    def evaluate_buildup(self, aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(),
                         temperature_offset_K=0.0, rotor_state=None):
        airplane = self.to_asb(aircraft)
        operating_point = self._operating_point(velocity_m_s, altitude_m, alpha_deg, temperature_offset_K)
        out = self._run(airplane, operating_point)
        dynamic_pressure_Pa = operating_point.dynamic_pressure()
        mach = operating_point.mach()
        area_ref_m2, span_ref_m = airplane.s_ref, airplane.b_ref
        force_scale_N = dynamic_pressure_Pa * area_ref_m2
        q = self.interference

        wings, bodies = out["wing_aero_components"], out["fuselage_aero_components"]
        labels_q = [("wing", q.wing), ("horizontal_tail", q.horizontal_tail), ("vertical_tail", q.vertical_tail),
                    ("fuselage", q.fuselage)] + [("nacelles", q.nacelle)] * (len(bodies) - 1)
        components = wings + bodies
        breakdown = []
        for (label, interference), component in zip(labels_q, components):
            breakdown.append(ParasiteDragItem(label, interference * component.D / force_scale_N))
        breakdown = _merge(breakdown)
        breakdown.append(ParasiteDragItem("miscellaneous", self.drag_area_misc_m2 / area_ref_m2))
        if not aircraft.landing_gear.is_retractable:
            breakdown.append(ParasiteDragItem("landing_gear", self.drag_area_landing_gear_fixed_m2 / area_ref_m2))

        # Induced drag: AeroBuildup's effective span, with the Nita-Scholz fuselage and Mach factors.
        span_effective_sq_m2 = softmax_scalefree([c.span_effective**2 * c.oswalds_efficiency for c in components])
        oswald_span = span_effective_sq_m2 / span_ref_m**2
        k_e_fuselage = 1 - 2 * (aircraft.fuselage.diameter_m / span_ref_m)**2
        k_e_mach = 1 - 0.00152 * np.fmax(mach / 0.3 - 1, 0)**10.82
        oswald = oswald_span * k_e_fuselage * k_e_mach
        aspect_ratio_ref = span_ref_m**2 / area_ref_m2

        cl = out["CL"]
        blown = None
        if self.blown_wing is not None and rotor_state is not None:
            propulsor = aircraft.powertrain.topology.instances["propulsor"]
            wing = wings[0]
            blown = self.blown_wing.evaluate(
                velocity_m_s, operating_point.atmosphere.density(), rotor_state, propulsor.component.area_disk_m2,
                propulsor.count, area_ref_m2 / span_ref_m, area_ref_m2, wing.L / force_scale_N,
                wing.D / force_scale_N, self.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m,
                                                                      temperature_offset_K))
            cl = cl + blown.delta_cl
            breakdown.append(ParasiteDragItem("blown_wing_profile", q.wing * blown.delta_cd_profile))
            breakdown.append(ParasiteDragItem("blown_wing_swirl", blown.delta_cd_swirl))

        cd0 = sum(item.cd0 for item in breakdown)
        cdi = cl**2 / (np.pi * aspect_ratio_ref * oswald)
        cd = cd0 + cdi + sum(increment.cd for increment in drag_increments)
        aero = AeroResult(alpha_deg=alpha_deg, cl=cl, cd=cd, cd0=cd0, cdi=cdi,
                          cl_alpha_per_rad=self.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m,
                                                                         temperature_offset_K),
                          oswald_efficiency=oswald, dynamic_pressure_Pa=dynamic_pressure_Pa, mach=mach,
                          lift_N=force_scale_N * cl, drag_N=force_scale_N * cd, lift_to_drag=cl / cd)
        return BuildupResult(aero=aero, breakdown=tuple(breakdown), cl_aerobuildup=out["CL"],
                             oswald_span=oswald_span, blown=blown)

    def evaluate(self, aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(), temperature_offset_K=0.0,
                 rotor_state=None):
        return self.evaluate_buildup(aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments,
                                     temperature_offset_K, rotor_state).aero


def _merge(items):
    """Sum items sharing a label (the nacelle pair), keeping first-seen order."""
    merged = {}
    for item in items:
        merged[item.label] = merged[item.label] + item.cd0 if item.label in merged else item.cd0
    return [ParasiteDragItem(label, cd0) for label, cd0 in merged.items()]
