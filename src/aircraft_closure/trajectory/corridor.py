"""Computed conversion corridor and trim of a sized tiltrotor in level flight (plan 039).

At a nacelle angle tau and airspeed V in steady level flight, the aircraft is trimmed when the forces along and
normal to the velocity and the pitching moment about the CG vanish:

    F_x = n T (1 - f_dl) cos(alpha + tau + theta) - D                        = 0
    F_z = n T (1 - f_dl) sin(alpha + tau + theta) + L + L_tail(delta) - W    = 0
    M_y = M_aero(alpha, delta) + M_rotor(T, tau, theta)                       = 0

with four unknowns: thrust per rotor T, attitude alpha (= pitch; zero flight-path angle), tail deflection delta
and longitudinal cyclic theta, the tilt of the rotor tip-path plane (and so the thrust) from the shaft. One
freedom is left; the trim takes the least rotor power. The corridor at each nacelle angle is the least and the
greatest V for which a trim exists inside the limits; the limit at its bound is reported as the binding one.

Forces: `TiltrotorPointMass` (rotor, wing lift and drag, download), evaluated with the thrust along the tip-path
plane (nacelle angle + cyclic). The tail deflection adds lift q S eta (S_H / S) a_H tau_e delta; the aerodynamic
pitching moment is `LongitudinalStability`'s (wing, fuselage, tail with the deflection). The rotor moment acts at
the hub, a mast length from the spindle along the shaft:

    M_rotor = n T (x_fwd,hub sin(tau + theta) - z_up,hub cos(tau + theta)),   hub relative to the CG.

Limits (`CorridorLimits`):
* attitude (pitch) band, which with the force balance sets most of the low-speed side: at low speed and a small
  nacelle angle the forward thrust can only be cancelled by drag or by pitching nose-up;
* wing stall at the free-stream angle of attack (the unblown outboard wing; the blown inboard part's local angle
  of attack is reported, not limited: near hover it sees the rotor downwash, which the download model carries);
* tail deflection (+/- 25 deg ruddervator) and cyclic travel, which washes out as sin(tau) toward airplane mode;
* the high-speed side: edgewise advance ratio mu = V |sin(alpha + tau)| / (Omega R), the first-order measure of
  flapping and hub and pylon loads that bound the XV-15 corridor at high nacelle angles; rotor shaft power (the
  drive rating); blade loading C_T / sigma; an airplane-mode placard speed.
* momentum-theory validity, V cos(alpha + tau + theta) >= 0 (no flow up through the disk).

Approximations: the rotor's edgewise flow does not change its power (as `TiltrotorPointMass`); the tail sees the
free stream (no rotor wake or wing downwash change in conversion); the CG does not move with the nacelles; a
gimballed rotor carries no hub moment, so cyclic acts only by tilting the thrust; the rotor's in-plane (H) force
in edgewise flow is not modelled, which makes the low-speed side at mid nacelle angles conservative (that drag
would help cancel the forward thrust).
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.controls.stability import flap_effectiveness
from aircraft_closure.performance.flight_point import acceleration_gravity_m_s2


@dataclass(frozen=True)
class CorridorLimits:
    pitch_min_deg: float = -5.0
    pitch_max_deg: float = 12.0
    deflection_max_tail_deg: float = 25.0            # ruddervator travel (user, 2026-10-05)
    cyclic_max_deg: float = 10.0                     # longitudinal cyclic (assumed, XV-15 class)
    advance_ratio_edgewise_max: float = 0.28         # flapping and hub-load proxy (assumed)
    velocity_placard_m_s: Any = None                 # airplane-mode limit speed; None: no placard
    blade_loading_max: Any = None                    # None: the rotor's own limit
    power_shaft_max_rotor_W: Any = None              # None: the rotor's drive rating


@dataclass(frozen=True)
class TrimGeometry:
    """Longitudinal positions (x aft, z up, m) the moment balance needs."""
    x_cg_m: float
    z_cg_m: float
    x_spindle_m: float
    z_spindle_m: float
    length_mast_m: float


def moment_rotor_Nm(thrust_total_N, angle_shaft_deg, angle_tip_path_deg, geometry):
    """Nose-up pitching moment about the CG of the rotors' thrust acting at the hubs.

    `angle_shaft_deg`: nacelle angle from the fuselage x-axis (90 deg hover); `angle_tip_path_deg`: the thrust's
    angle (shaft + cyclic). Thrust up ahead of the CG or forward below it pitches nose-up.
    """
    forward_hub_m = (geometry.x_cg_m - geometry.x_spindle_m) + geometry.length_mast_m * np.cosd(angle_shaft_deg)
    up_hub_m = (geometry.z_spindle_m - geometry.z_cg_m) + geometry.length_mast_m * np.sind(angle_shaft_deg)
    return thrust_total_N * (forward_hub_m * np.sind(angle_tip_path_deg) - up_hub_m * np.cosd(angle_tip_path_deg))


@dataclass(frozen=True)
class TrimPoint:
    """A level-flight trim (numbers once solved; expressions while building)."""
    velocity_m_s: Any
    tilt_deg: Any
    pitch_deg: Any
    thrust_per_rotor_N: Any
    deflection_tail_deg: Any
    cyclic_deg: Any
    power_shaft_rotor_W: Any
    blade_loading: Any
    advance_ratio_edgewise: Any
    alpha_stall_deg: Any
    alpha_local_blown_deg: Any
    download_fraction: Any
    lift_wing_N: Any
    fraction_cyclic: Any            # cyclic as a fraction of its travel at this nacelle angle
    margins: dict                   # limit name -> margin (>= 0 inside, normalized by the limit)

    def binding(self, tolerance=1e-3):
        """Names of the limits at their bound."""
        return tuple(name for name, margin in self.margins.items() if margin < tolerance)


def build_trim(opti, model, stability, geometry, limits, *, mass_kg, velocity_m_s, tilt_deg, altitude_m,
               speed_rotor_rad_s, thrust_guess_N=None):
    """Add one level-flight trim to `opti` (variables: pitch, thrust, tail deflection, cyclic).

    `velocity_m_s` and `tilt_deg` may themselves be Opti variables (the corridor bounds). Returns the `TrimPoint`
    of expressions; every limit is constrained.
    """
    aircraft = model.aircraft
    rotor = model.instance("propulsor")
    count_rotors = model.count("propulsor")
    weight_N = mass_kg * acceleration_gravity_m_s2
    thrust_guess_N = weight_N / count_rotors if thrust_guess_N is None else thrust_guess_N
    pitch_deg = opti.variable(init_guess=2.0, scale=5.0, lower_bound=limits.pitch_min_deg,
                              upper_bound=limits.pitch_max_deg)
    thrust_per_rotor_N = opti.variable(init_guess=thrust_guess_N, scale=weight_N / count_rotors, lower_bound=0.0)
    deflection_tail_deg = opti.variable(init_guess=0.0, scale=10.0, lower_bound=-limits.deflection_max_tail_deg,
                                        upper_bound=limits.deflection_max_tail_deg)
    # Cyclic travel washes out toward airplane mode (XV-15 practice): theta = f theta_max sin(tau), |f| <= 1.
    fraction_cyclic = opti.variable(init_guess=0.0, scale=0.5, lower_bound=-1.0, upper_bound=1.0)
    cyclic_deg = fraction_cyclic * limits.cyclic_max_deg * np.sind(tilt_deg)

    angle_tip_path_deg = tilt_deg + cyclic_deg
    forces = model.evaluate(velocity_m_s, altitude_m, pitch_deg, angle_tip_path_deg,
                            thrust_per_rotor_N=thrust_per_rotor_N, speed_rotor_rad_s=speed_rotor_rad_s)

    # Tail and pitching moment: coefficients at the model's floored speed, forces at the true dynamic pressure.
    velocity_coefficient_m_s = np.fmax(velocity_m_s, model.velocity_coefficient_min_m_s)
    density_kg_m3 = asb.Atmosphere(altitude=altitude_m).density()
    pressure_dynamic_Pa = 0.5 * density_kg_m3 * velocity_m_s ** 2
    wing, tail = aircraft.wing, aircraft.horizontal_tail
    chord_m = wing.to_asb().mean_aerodynamic_chord()
    longitudinal = stability.evaluate(aircraft, model.aerodynamics, geometry.x_cg_m, velocity_coefficient_m_s,
                                      altitude_m, pitch_deg, deflection_tail_deg)
    slope_tail_per_rad = model.aerodynamics.surface_lift_curve_slope_per_rad(tail.aspect_ratio,
                                                                            velocity_coefficient_m_s, altitude_m)
    lift_tail_deflection_N = (pressure_dynamic_Pa * stability.tail_dynamic_pressure_ratio * tail.area_m2
                              * slope_tail_per_rad * flap_effectiveness(tail.elevator_chord_fraction,
                                                                        stability.flap_effectiveness_correction)
                              * np.radians(deflection_tail_deg))
    moment_aero_Nm = pressure_dynamic_Pa * wing.area_m2 * chord_m * longitudinal.cm
    thrust_net_N = count_rotors * thrust_per_rotor_N * (1 - forces.download_fraction)
    moment_Nm = moment_aero_Nm + moment_rotor_Nm(thrust_net_N, tilt_deg, angle_tip_path_deg, geometry)

    opti.subject_to([
        forces.force_x_wind_N / weight_N == 0,
        (-forces.force_z_wind_N + lift_tail_deflection_N - weight_N) / weight_N == 0,
        moment_Nm / (weight_N * chord_m) == 0,
    ])

    # Limits, each as a normalized margin >= 0.
    speed_tip_m_s = speed_rotor_rad_s * rotor.radius_m()
    angle_shaft_velocity_deg = pitch_deg + tilt_deg
    advance_ratio_edgewise = velocity_m_s * np.sind(angle_shaft_velocity_deg) / speed_tip_m_s
    blade_loading_max = rotor.blade_loading_max if limits.blade_loading_max is None else limits.blade_loading_max
    power_max_W = rotor.max_shaft_power_W if limits.power_shaft_max_rotor_W is None else limits.power_shaft_max_rotor_W
    mu_max = limits.advance_ratio_edgewise_max
    margins = {
        "pitch_max": (limits.pitch_max_deg - pitch_deg) / 10.0,
        "pitch_min": (pitch_deg - limits.pitch_min_deg) / 10.0,
        "tail_deflection": (limits.deflection_max_tail_deg ** 2 - deflection_tail_deg ** 2)
        / limits.deflection_max_tail_deg ** 2,
        "cyclic": 1 - fraction_cyclic ** 2,
        "wing_stall": (forces.alpha_stall_deg - pitch_deg) / 10.0,
        "edgewise_advance_ratio": (mu_max ** 2 - advance_ratio_edgewise ** 2) / mu_max ** 2,
        "rotor_power": 1 - forces.rotor.shaft_power_W / power_max_W,
        "blade_loading": 1 - forces.rotor.blade_loading / blade_loading_max,
        "inflow_through_disk": forces.velocity_axial_m_s / speed_tip_m_s,
    }
    if limits.velocity_placard_m_s is not None:
        margins["placard_speed"] = 1 - velocity_m_s / limits.velocity_placard_m_s
    # The wing stall limit is meaningless without dynamic pressure; it applies above the coefficient floor speed.
    margins["wing_stall"] = margins["wing_stall"] + np.fmax(model.velocity_coefficient_min_m_s - velocity_m_s, 0.0)
    for name, margin in margins.items():
        if name in ("pitch_max", "pitch_min", "cyclic"):
            continue                       # variable bounds
        opti.subject_to(margin >= 0)

    # Local angle of attack of the blown (inboard) wing: free stream plus the slipstream along the thrust axis.
    velocity_slipstream_m_s = forces.rotor.induced_velocity_m_s * 2          # far-wake value (diagnostic only)
    flow_aft_m_s = velocity_m_s * np.cosd(pitch_deg) + velocity_slipstream_m_s * np.cosd(angle_tip_path_deg)
    flow_up_m_s = velocity_m_s * np.sind(pitch_deg) - velocity_slipstream_m_s * np.sind(angle_tip_path_deg)
    alpha_local_blown_deg = np.arctan2d(flow_up_m_s, flow_aft_m_s)

    return TrimPoint(velocity_m_s=velocity_m_s, tilt_deg=tilt_deg, pitch_deg=pitch_deg,
                     thrust_per_rotor_N=thrust_per_rotor_N, deflection_tail_deg=deflection_tail_deg,
                     cyclic_deg=cyclic_deg, power_shaft_rotor_W=forces.rotor.shaft_power_W,
                     blade_loading=forces.rotor.blade_loading, advance_ratio_edgewise=advance_ratio_edgewise,
                     alpha_stall_deg=forces.alpha_stall_deg, alpha_local_blown_deg=alpha_local_blown_deg,
                     download_fraction=forces.download_fraction, lift_wing_N=forces.lift_N,
                     fraction_cyclic=fraction_cyclic, margins=margins)


def _solved(sol, point):
    value = lambda x: float(sol(x))  # noqa: E731
    return TrimPoint(**{name: value(getattr(point, name)) for name in TrimPoint.__dataclass_fields__
                        if name != "margins"}, margins={k: value(m) for k, m in point.margins.items()})


def _power_objective(point, model):
    rotor = model.instance("propulsor")
    return point.power_shaft_rotor_W / rotor.max_shaft_power_W + 1e-3 * point.fraction_cyclic ** 2


def solve_trim(model, stability, geometry, limits, *, mass_kg, velocity_m_s, tilt_deg, altitude_m,
               speed_rotor_rad_s, verbose=False):
    """Least-power level-flight trim at a given airspeed and nacelle angle; raises RuntimeError if none exists."""
    opti = asb.Opti()
    point = build_trim(opti, model, stability, geometry, limits, mass_kg=mass_kg, velocity_m_s=velocity_m_s,
                       tilt_deg=tilt_deg, altitude_m=altitude_m, speed_rotor_rad_s=speed_rotor_rad_s)
    opti.minimize(_power_objective(point, model))
    sol = opti.solve(verbose=verbose, max_iter=500, behavior_on_failure="raise")
    return _solved(sol, point)


@dataclass(frozen=True)
class CorridorBound:
    tilt_deg: float
    side: str                 # "low" or "high"
    trim: Any                 # TrimPoint at the bound, or None when no trim exists at this nacelle angle
    binding: tuple

    @property
    def velocity_m_s(self):
        return None if self.trim is None else self.trim.velocity_m_s


def solve_corridor_bound(model, stability, geometry, limits, *, mass_kg, tilt_deg, altitude_m, speed_rotor_rad_s,
                         side, velocity_guess_m_s=40.0, velocity_max_m_s=150.0, verbose=False):
    """The least ("low") or greatest ("high") level-flight trim airspeed at a nacelle angle."""
    if side not in ("low", "high"):
        raise ValueError(f"Unknown side '{side}'.")
    opti = asb.Opti()
    velocity_m_s = opti.variable(init_guess=velocity_guess_m_s, scale=50.0, lower_bound=0.0,
                                 upper_bound=velocity_max_m_s)
    point = build_trim(opti, model, stability, geometry, limits, mass_kg=mass_kg, velocity_m_s=velocity_m_s,
                       tilt_deg=tilt_deg, altitude_m=altitude_m, speed_rotor_rad_s=speed_rotor_rad_s)
    sign = 1.0 if side == "low" else -1.0
    opti.minimize(sign * velocity_m_s / 50.0 + 1e-3 * _power_objective(point, model))
    try:
        sol = opti.solve(verbose=verbose, max_iter=1000, behavior_on_failure="raise")
    except RuntimeError:
        return CorridorBound(tilt_deg, side, None, ())
    trim = _solved(sol, point)
    binding = trim.binding(tolerance=2e-3)
    if trim.velocity_m_s < 1e-2:                 # hover: zero inflow is the state itself, not a limit
        binding = ("hover",) + tuple(name for name in binding if name != "inflow_through_disk")
    if trim.velocity_m_s > velocity_max_m_s - 1e-2:
        binding = ("search_bound",) + binding
    return CorridorBound(tilt_deg, side, trim, binding)


def solve_corridor(model, stability, geometry, limits, *, mass_kg, altitude_m, speed_rotor_rad_s,
                   tilts_deg=(0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0), velocity_max_m_s=150.0, verbose=False):
    """Low and high corridor bounds at each nacelle angle: tuple of (low, high) `CorridorBound` pairs."""
    bounds = []
    for tilt_deg in tilts_deg:
        pair = tuple(solve_corridor_bound(model, stability, geometry, limits, mass_kg=mass_kg, tilt_deg=tilt_deg,
                                          altitude_m=altitude_m, speed_rotor_rad_s=speed_rotor_rad_s, side=side,
                                          velocity_max_m_s=velocity_max_m_s, verbose=verbose)
                     for side in ("low", "high"))
        bounds.append(pair)
    return tuple(bounds)


@dataclass(frozen=True)
class ComputedCorridor:
    """The computed corridor as airspeed limits against nacelle angle, for the trajectory layer.

    Same interface as `ConversionCorridor` (the assumed XV-15 shape): `velocity_min_m_s(tilt_deg)` and
    `velocity_max_m_s(tilt_deg)`, piecewise linear between the computed nacelle angles and symbolic in the
    nacelle angle. `velocity_stall_m_s` is the airplane-mode stall speed, which sets the transition end speed.
    The corridor is computed at one altitude and mass; using it elsewhere is an approximation.
    """
    tilts_deg: tuple
    velocity_low_m_s: tuple
    velocity_high_m_s: tuple
    velocity_stall_m_s: float

    @classmethod
    def from_bounds(cls, corridor, velocity_stall_m_s):
        """From `solve_corridor` pairs; nacelle angles without a trim on both sides are left out."""
        pairs = sorted(((low, high) for low, high in corridor if low.trim is not None and high.trim is not None),
                       key=lambda pair: pair[0].tilt_deg)
        return cls(tuple(low.tilt_deg for low, _ in pairs), tuple(low.velocity_m_s for low, _ in pairs),
                   tuple(high.velocity_m_s for _, high in pairs), velocity_stall_m_s)

    def velocity_min_m_s(self, tilt_deg):
        return _piecewise_linear(tilt_deg, self.tilts_deg, self.velocity_low_m_s)

    def velocity_max_m_s(self, tilt_deg):
        return _piecewise_linear(tilt_deg, self.tilts_deg, self.velocity_high_m_s)


def _piecewise_linear(x, xp, fp):
    """Linear interpolation through (xp, fp), constant outside, as a sum of hinges: elementwise on symbolic vectors
    (`np.interp` takes one symbolic point at a time)."""
    x = np.fmin(np.fmax(x, xp[0]), xp[-1])
    slopes = [(fp[i + 1] - fp[i]) / (xp[i + 1] - xp[i]) for i in range(len(xp) - 1)]
    value = fp[0] + slopes[0] * (x - xp[0])
    for i in range(1, len(slopes)):
        value = value + (slopes[i] - slopes[i - 1]) * np.fmax(x - xp[i], 0.0)
    return value
