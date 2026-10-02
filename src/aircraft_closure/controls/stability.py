"""Static stability, pitch trim and rudder authority for a conventional tail.

Equations only: callers own trim variables (alpha, elevator) and constraints
(Cm = 0, lift = weight, margins). Surfaces are unswept, so aerodynamic centres
sit at the quarter MAC. Angles are degrees at the interface, radians inside.
References: Raymer ch. 16 (fuselage pitch and yaw terms); classical
neutral-point and downwash relations (Etkin and Reid, Nelson); thin-airfoil
flap effectiveness.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.library.aerodynamics.inviscid import oswalds_efficiency

from aircraft_closure.aerodynamics.simple import DragIncrement


def flap_effectiveness(chord_fraction, correction=0.8):
    """Thin-airfoil d(alpha_0)/d(delta) for a plain flap, times a viscous correction.

    tau = 1 - (theta - sin theta) / pi with theta = arccos(2 c_f/c - 1);
    tau(0) = 0 and tau(1) = 1 before correction.
    """
    theta = np.arccos(2 * chord_fraction - 1)
    return correction * (1 - (theta - np.sin(theta)) / np.pi)


def _quarter_mac_x_m(surface):
    return surface.x_le_root_m + 0.25 * surface.to_asb().mean_aerodynamic_chord()


def _dynamic_pressure_Pa(velocity_m_s, altitude_m):
    return 0.5 * asb.Atmosphere(altitude=altitude_m).density() * velocity_m_s**2


@dataclass(frozen=True)
class LongitudinalResult:
    cl_total: Any
    cl_wing: Any
    cl_tail: Any
    cm: Any
    downwash_deg: Any
    lift_N: Any
    trim_drag: Any


@dataclass(frozen=True)
class LongitudinalStability:
    tail_dynamic_pressure_ratio: Any = 0.9
    cm_ac_wing: Any = -0.09
    fuselage_pitch_factor_per_deg: Any = 0.012
    tail_incidence_deg: Any = 0.0
    flap_effectiveness_correction: Any = 0.8

    def _terms(self, aircraft, aerodynamics, velocity_m_s, altitude_m):
        wing, tail, fuselage = aircraft.wing, aircraft.horizontal_tail, aircraft.fuselage
        chord_m = wing.to_asb().mean_aerodynamic_chord()
        slope_wing = aerodynamics.lift_curve_slope_per_rad(aircraft, velocity_m_s, altitude_m)
        slope_tail = aerodynamics.surface_lift_curve_slope_per_rad(tail.aspect_ratio, velocity_m_s, altitude_m)
        downwash_gradient = 2 * slope_wing / (np.pi * wing.aspect_ratio)
        tail_volume_ratio = self.tail_dynamic_pressure_ratio * tail.area_m2 / wing.area_m2
        # Raymer eq. 16.25: destabilizing fuselage moment slope, converted to per radian.
        cm_alpha_fuselage = (self.fuselage_pitch_factor_per_deg * fuselage.diameter_m**2 * fuselage.length_m
                             / (chord_m * wing.area_m2)) * 180 / np.pi
        return chord_m, slope_wing, slope_tail, downwash_gradient, tail_volume_ratio, cm_alpha_fuselage

    def lift_curve_slope_total_per_rad(self, aircraft, aerodynamics, velocity_m_s, altitude_m):
        _, slope_wing, slope_tail, downwash_gradient, tail_volume_ratio, _ = self._terms(
            aircraft, aerodynamics, velocity_m_s, altitude_m)
        return slope_wing + tail_volume_ratio * slope_tail * (1 - downwash_gradient)

    def neutral_point_x_m(self, aircraft, aerodynamics, velocity_m_s, altitude_m):
        chord_m, slope_wing, slope_tail, downwash_gradient, tail_volume_ratio, cm_alpha_fuselage = self._terms(
            aircraft, aerodynamics, velocity_m_s, altitude_m)
        tail_slope_effective = tail_volume_ratio * slope_tail * (1 - downwash_gradient)
        slope_total = slope_wing + tail_slope_effective
        return ((slope_wing * _quarter_mac_x_m(aircraft.wing)
                 + tail_slope_effective * _quarter_mac_x_m(aircraft.horizontal_tail)) / slope_total
                - cm_alpha_fuselage * chord_m / slope_total)

    def static_margin(self, aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m):
        chord_m = aircraft.wing.to_asb().mean_aerodynamic_chord()
        return (self.neutral_point_x_m(aircraft, aerodynamics, velocity_m_s, altitude_m) - x_cg_m) / chord_m

    def evaluate(self, aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m, alpha_deg, elevator_deg):
        chord_m, _, slope_tail, _, tail_volume_ratio, cm_alpha_fuselage = self._terms(
            aircraft, aerodynamics, velocity_m_s, altitude_m)
        tail = aircraft.horizontal_tail
        cl_wing = aerodynamics.evaluate(aircraft, velocity_m_s, altitude_m, alpha_deg).cl
        downwash_rad = 2 * cl_wing / (np.pi * aircraft.wing.aspect_ratio)
        tau_elevator = flap_effectiveness(tail.elevator_chord_fraction, self.flap_effectiveness_correction)
        alpha_tail_rad = (np.radians(alpha_deg + self.tail_incidence_deg) - downwash_rad
                          + tau_elevator * np.radians(elevator_deg))
        cl_tail = slope_tail * alpha_tail_rad
        x_ac_wing_m = _quarter_mac_x_m(aircraft.wing)
        x_ac_tail_m = _quarter_mac_x_m(tail)
        cm = (self.cm_ac_wing + cl_wing * (x_cg_m - x_ac_wing_m) / chord_m
              - tail_volume_ratio * cl_tail * (x_ac_tail_m - x_cg_m) / chord_m
              + cm_alpha_fuselage * np.radians(alpha_deg))
        cl_total = cl_wing + tail_volume_ratio * cl_tail
        tail_oswald = oswalds_efficiency(tail.taper_ratio, tail.aspect_ratio)
        trim_drag = DragIncrement("horizontal_tail_induced",
                                  tail_volume_ratio * cl_tail**2 / (np.pi * tail_oswald * tail.aspect_ratio))
        return LongitudinalResult(cl_total=cl_total, cl_wing=cl_wing, cl_tail=cl_tail, cm=cm,
                                  downwash_deg=np.degrees(downwash_rad),
                                  lift_N=_dynamic_pressure_Pa(velocity_m_s, altitude_m) * aircraft.wing.area_m2 * cl_total,
                                  trim_drag=trim_drag)


@dataclass(frozen=True)
class DirectionalStability:
    vertical_tail_dynamic_pressure_ratio: Any = 0.9
    effective_aspect_ratio_factor: Any = 1.55
    flap_effectiveness_correction: Any = 0.8

    def _tail_power(self, aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m):
        """eta_v CL_alpha_v S_v l_v / (S b), the fin's yaw power per radian of sideslip."""
        fin, wing = aircraft.vertical_tail, aircraft.wing
        # Fuselage and horizontal tail end-plate the fin (Raymer: about 1.55 x geometric AR).
        slope_fin = aerodynamics.surface_lift_curve_slope_per_rad(
            self.effective_aspect_ratio_factor * fin.aspect_ratio, velocity_m_s, altitude_m)
        arm_m = _quarter_mac_x_m(fin) - x_cg_m
        return self.vertical_tail_dynamic_pressure_ratio * slope_fin * fin.area_m2 * arm_m / (wing.area_m2 * wing.span_m())

    def cn_beta_per_rad(self, aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m):
        """Fin contribution plus Raymer eq. 16.47 fuselage term (round body, D/W = 1)."""
        wing = aircraft.wing
        cn_beta_fuselage = -1.3 * aircraft.fuselage.to_asb().volume() / (wing.area_m2 * wing.span_m())
        return self._tail_power(aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m) + cn_beta_fuselage

    def rudder_for_yaw_moment_deg(self, aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m, yaw_moment_Nm):
        wing = aircraft.wing
        tau_rudder = flap_effectiveness(aircraft.vertical_tail.rudder_chord_fraction, self.flap_effectiveness_correction)
        cn_delta_per_rad = tau_rudder * self._tail_power(aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m)
        dynamic_pressure_Pa = _dynamic_pressure_Pa(velocity_m_s, altitude_m)
        return np.degrees(yaw_moment_Nm / (dynamic_pressure_Pa * wing.area_m2 * wing.span_m() * cn_delta_per_rad))


def failed_propulsor_yaw_moment_Nm(thrust_per_propulsor_N, lateral_arm_m, drag_failed_propulsor_N=0.0):
    """Asymmetric yaw from losing one propulsor: its missing thrust plus its drag."""
    return (thrust_per_propulsor_N + drag_failed_propulsor_N) * lateral_arm_m
