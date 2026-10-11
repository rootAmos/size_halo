"""Proprotor by blade-element momentum theory with Ning's single residual (plan 047).

Each blade station carries one unknown, the inflow angle phi, which the caller makes an `asb.Opti` variable.
The model returns the residual for the caller to constrain to zero (no hidden iteration). With local solidity
sigma' = B c / (2 pi r), lambda_r = V / (Omega r) and Prandtl's F = F_tip F_hub, Ning's residual
(Struct. Multidisc. Optim. 64(2), 2021) multiplied by sin(phi) is

    R(phi) = sin phi |sin phi| - sigma' cn / (4 F) - lambda_r (sin phi cos phi + sigma' ct / (4 F)) = 0,

    cn = cl cos phi - cd sin phi,  ct = cl sin phi + cd cos phi,

with |sin phi| smoothed (sqrt(sin^2 phi + 0.003^2)) wherever it appears (here, in F and in k'). For phi > 0 this is
Ning's residual; the smoothing lets a lightly loaded tip carry upwash (phi < 0) without a branch. It holds in
hover (lambda_r = 0: sin^2 phi = sigma' cn / 4F). Section lift and
drag come from NeuralFoil with Reynolds and Mach on the undisturbed speed sqrt(V^2 + (Omega r)^2), so the
residual stays a function of phi alone. Loads: dT/dr = B q c cn, dQ/dr = B q c ct r, with
W = Omega r / ((1 + k') cos phi), k' = sigma' ct / (4 F |sin phi| cos phi).

The blade is chord and twist at a few spanwise control points, interpolated by a fixed (C2 cubic-spline or
linear) basis, so station values are linear in the control values and the control values can be Opti
variables. Twist is indexed to zero at 0.75 R; pitch = collective + twist. Sections are a fixed thickness
schedule on one camber line (`SectionSchedule`).

Rotational stall delay (`stall_delay`, after Snel): cl -> cl + K (cl_attached - cl), K = tanh(3 (c/r)^2), with
cl_attached = 2 pi sin(alpha - alpha_0) / sqrt(1 - M^2). Without it, the inboard sections of a highly twisted
proprotor stall early in hover and the model loses the high-thrust JVX points (Acree, TM-2008-214581, found the
same with CAMRAD II).

Validity and calibration (plan 047): see `examples/jvx_rotor_bemt.py`; `drag_factor` multiplies NeuralFoil
section drag and is the one calibration constant.
"""
from dataclasses import dataclass, fields, replace
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp


def station_fractions(fraction_root, count_stations):
    """Panel midpoints and widths (fractions of radius) of a cosine split of [root, 1], clustered at both ends."""
    edges = fraction_root + (1 - fraction_root) * (1 - onp.cos(onp.pi * onp.linspace(0, 1, count_stations + 1))) / 2
    return (edges[1:] + edges[:-1]) / 2, onp.diff(edges)


def interpolation_basis(fraction_control, fraction_eval, kind="cubic"):
    """Matrix M (n_eval x n_control) with values(eval) = M @ values(control): natural cubic spline or linear."""
    fraction_control = onp.asarray(fraction_control, dtype=float)
    identity = onp.eye(len(fraction_control))
    if kind == "linear":
        return onp.stack([onp.interp(fraction_eval, fraction_control, column) for column in identity], axis=1)
    from scipy.interpolate import CubicSpline
    return CubicSpline(fraction_control, identity, bc_type="natural")(onp.asarray(fraction_eval, dtype=float))


def _as_vector(values):
    """A control-value sequence (floats or CasADi scalars) as one array/vector for matrix products."""
    if isinstance(values, (list, tuple)):
        return np.array(list(values))
    return values


@dataclass(frozen=True)
class SectionSchedule:
    """Fixed airfoil family along the blade: a base camber line with thickness scaled to the local t/c."""
    fraction_radius: tuple = (0.10, 0.40, 0.75, 1.00)
    thickness_to_chord: tuple = (0.28, 0.18, 0.12, 0.09)    # XN-28/18/12/09 stand-in (assumed)
    airfoil_base: str = "naca64a210"
    model_size: str = "medium"
    angle_zero_lift_deg: float = -1.66       # NACA 64A210 camber line, NeuralFoil at Re 5e6
    fraction_chord_transition: float = 0.05  # forced transition x/c, both surfaces: full-scale blades run turbulent

    def airfoils(self, fractions):
        base = asb.Airfoil(self.airfoil_base).to_kulfan_airfoil()
        thickness_base = base.max_thickness()
        camber = (base.upper_weights + base.lower_weights) / 2
        half = (base.upper_weights - base.lower_weights) / 2
        result = []
        for fraction in fractions:
            scale = onp.interp(fraction, self.fraction_radius, self.thickness_to_chord) / thickness_base
            result.append(asb.KulfanAirfoil(name=f"section_{fraction:.3f}", upper_weights=camber + scale * half,
                                            lower_weights=camber - scale * half,
                                            leading_edge_weight=base.leading_edge_weight * scale,
                                            TE_thickness=base.TE_thickness))
        return result


@dataclass(frozen=True)
class BladeGeometry:
    """Chord and twist at control fractions of radius; `interpolation` "cubic" (C2 spline) or "linear".
    Chord may use its own control fractions and interpolation (`fraction_control_chord`, `interpolation_chord`),
    e.g. a linear taper through root and tip."""
    fraction_control: tuple = (0.10, 0.40, 0.75, 1.00)
    chord_control_m: Any = (0.30, 0.30, 0.30, 0.30)
    twist_control_deg: Any = (20.0, 9.0, 0.0, -10.0)
    interpolation: str = "cubic"
    fraction_control_chord: tuple = None
    interpolation_chord: str = None

    def chord_m(self, fractions):
        control = self.fraction_control if self.fraction_control_chord is None else self.fraction_control_chord
        kind = self.interpolation if self.interpolation_chord is None else self.interpolation_chord
        return interpolation_basis(control, fractions, kind) @ _as_vector(self.chord_control_m)

    def twist_deg(self, fractions):
        """Twist relative to 0.75 R."""
        basis = interpolation_basis(self.fraction_control, fractions, self.interpolation)
        reference = interpolation_basis(self.fraction_control, [0.75], self.interpolation)[0]
        return (basis - reference[None, :]) @ _as_vector(self.twist_control_deg)


@dataclass(frozen=True)
class BemtResult:
    thrust_N: Any
    torque_Nm: Any
    shaft_power_W: Any
    residual: Any                  # one per station, constrain to zero
    figure_of_merit: Any           # hover definition, T^1.5 / sqrt(2 rho A) / P
    efficiency_propulsive: Any     # T V / P (zero in hover)
    blade_loading: Any             # C_T / sigma, thrust-weighted solidity
    mach_tip_helical: Any
    angle_attack_deg: Any          # per station
    cl: Any
    cd: Any
    fraction_radius: Any
    angle_inflow_rad: Any


@dataclass(frozen=True)
class BladeElementRotor:
    radius_m: Any = 4.83
    count_blades: int = 3
    fraction_root: float = 0.10
    blade: BladeGeometry = BladeGeometry()
    sections: SectionSchedule = SectionSchedule()
    count_stations: int = 16
    drag_factor: Any = 0.964               # JVX fit with turbulent sections (plan 047)
    tip_loss: bool = True
    hub_loss: bool = True
    stall_delay: bool = True
    sin_inflow_smoothing: float = 3e-3     # |sin phi| ~ sqrt(sin^2 phi + s^2): smooth through phi = 0

    def stations(self):
        return station_fractions(self.fraction_root, self.count_stations)

    def area_disk_m2(self):
        return np.pi * self.radius_m**2

    def solidity_thrust_weighted(self):
        """sigma_tw = 3 integral_0^1 sigma(x) x^2 dx over the blade, sigma(x) = B c / (pi R)."""
        fractions, widths = self.stations()
        chord_m = self.blade.chord_m(fractions)
        return 3 * np.sum(self.count_blades * chord_m / (np.pi * self.radius_m) * fractions**2 * widths)

    def with_blade(self, blade):
        return replace(self, blade=blade)

    def initial_inflow_angle(self, axial_velocity_m_s, speed_rad_s, thrust_N, density_kg_m3):
        """Numerical starting guess for phi: uniform momentum inflow (not used inside the equations)."""
        fractions, _ = self.stations()
        area_m2 = float(self.area_disk_m2())
        induced_m_s = -axial_velocity_m_s / 2 + onp.sqrt(axial_velocity_m_s**2 / 4 + thrust_N / (2 * density_kg_m3 * area_m2))
        return onp.arctan2(axial_velocity_m_s + induced_m_s, speed_rad_s * float(self.radius_m) * fractions)

    def evaluate(self, axial_velocity_m_s, atmosphere, *, speed_rad_s, collective_deg, angle_inflow_rad):
        fractions, widths = self.stations()
        radius_m = self.radius_m
        r_m = fractions * radius_m
        density_kg_m3 = atmosphere.density()
        speed_sound_m_s = atmosphere.speed_of_sound()
        viscosity_Pa_s = atmosphere.dynamic_viscosity()
        chord_m = self.blade.chord_m(fractions)
        pitch_deg = collective_deg + self.blade.twist_deg(fractions)
        phi = angle_inflow_rad
        alpha_deg = pitch_deg - np.degrees(phi)
        speed_section_m_s = np.sqrt(axial_velocity_m_s**2 + (speed_rad_s * r_m)**2)
        mach = speed_section_m_s / speed_sound_m_s
        reynolds = density_kg_m3 * speed_section_m_s * chord_m / viscosity_Pa_s

        cl_list, cd_list = [], []
        for i, airfoil in enumerate(self.sections.airfoils(fractions)):
            aero = airfoil.get_aero_from_neuralfoil(alpha=alpha_deg[i], Re=reynolds[i], mach=mach[i],
                                                    xtr_upper=self.sections.fraction_chord_transition,
                                                    xtr_lower=self.sections.fraction_chord_transition,
                                                    model_size=self.sections.model_size)
            cl_list.append(aero["CL"])
            cd_list.append(aero["CD"])
        cl = np.array(cl_list) if not isinstance(cl_list[0], onp.ndarray) else onp.concatenate(cl_list)
        if self.stall_delay:
            # Rotational stall delay (Snel's 3 (c/r)^2, saturated by tanh): lift moves toward the attached-flow
            # line 2 pi sin(alpha - alpha_0) / beta in proportion to the factor.
            factor = np.tanh(3 * (chord_m / r_m)**2)
            lift_attached = 2 * np.pi * np.sind(alpha_deg - self.sections.angle_zero_lift_deg) / np.sqrt(1 - mach**2)
            cl = cl + factor * (lift_attached - cl)
        cd = (np.array(cd_list) if not isinstance(cd_list[0], onp.ndarray) else onp.concatenate(cd_list)) * self.drag_factor

        sin_phi, cos_phi = np.sin(phi), np.cos(phi)
        # |sin phi| smoothed, so a lightly loaded tip can carry a small negative inflow angle (upwash).
        sin_phi_abs = np.sqrt(sin_phi**2 + self.sin_inflow_smoothing**2)
        loss = 1.0
        if self.tip_loss:
            loss = loss * 2 / np.pi * np.arccos(np.exp(-self.count_blades / 2 * (1 - fractions) / (fractions * sin_phi_abs)))
        if self.hub_loss:
            loss = loss * 2 / np.pi * np.arccos(np.exp(-self.count_blades / 2 * (fractions - self.fraction_root)
                                                       / (self.fraction_root * sin_phi_abs)))
        solidity_local = self.count_blades * chord_m / (2 * np.pi * r_m)
        force_normal = cl * cos_phi - cd * sin_phi
        force_tangential = cl * sin_phi + cd * cos_phi
        inflow_local = axial_velocity_m_s / (speed_rad_s * r_m)
        residual = (sin_phi * sin_phi_abs - solidity_local * force_normal / (4 * loss)
                    - inflow_local * (sin_phi * cos_phi + solidity_local * force_tangential / (4 * loss)))
        swirl = solidity_local * force_tangential / (4 * loss * sin_phi_abs * cos_phi)
        speed_relative_m_s = speed_rad_s * r_m / ((1 + swirl) * cos_phi)
        pressure_Pa = density_kg_m3 * speed_relative_m_s**2 / 2
        thrust_N = np.sum(self.count_blades * pressure_Pa * chord_m * force_normal * widths * radius_m)
        torque_Nm = np.sum(self.count_blades * pressure_Pa * chord_m * force_tangential * r_m * widths * radius_m)
        shaft_power_W = torque_Nm * speed_rad_s
        area_m2 = self.area_disk_m2()
        speed_tip_m_s = speed_rad_s * radius_m
        blade_loading = thrust_N / (density_kg_m3 * area_m2 * speed_tip_m_s**2) / self.solidity_thrust_weighted()
        figure_of_merit = thrust_N**1.5 / np.sqrt(2 * density_kg_m3 * area_m2) / shaft_power_W
        mach_tip_helical = np.sqrt(speed_tip_m_s**2 + axial_velocity_m_s**2) / speed_sound_m_s
        return BemtResult(thrust_N, torque_Nm, shaft_power_W, residual, figure_of_merit,
                          thrust_N * axial_velocity_m_s / shaft_power_W, blade_loading, mach_tip_helical,
                          alpha_deg, cl, cd, fractions, phi)


@dataclass(frozen=True)
class AirState:
    """Measured test-section air (density and speed of sound) where no standard altitude applies."""
    density_kg_m3: float
    speed_sound_m_s: float
    viscosity_Pa_s: float = 1.79e-5

    def density(self):
        return self.density_kg_m3

    def speed_of_sound(self):
        return self.speed_sound_m_s

    def dynamic_viscosity(self):
        return self.viscosity_Pa_s


@dataclass(frozen=True)
class OperatingPoint:
    label: str
    axial_velocity_m_s: float
    altitude_m: float
    speed_rad_s: float
    thrust_N: float
    temperature_offset_K: float = 0.0
    air: Any = None                # an `AirState` overrides the standard atmosphere

    def atmosphere(self):
        if self.air is not None:
            return self.air
        return asb.Atmosphere(altitude=self.altitude_m, temperature_deviation=self.temperature_offset_K)


def add_operating_point(opti, rotor, point, *, collective_guess_deg=10.0, thrust_N=None, angle_inflow_guess_rad=None):
    """Opti variables (inflow angle per station, collective) and the residual and thrust constraints for one
    operating point; returns (result, collective variable). `thrust_N` overrides the point's (symbolic allowed)."""
    atmosphere = point.atmosphere()
    target_N = point.thrust_N if thrust_N is None else thrust_N
    phi_guess = angle_inflow_guess_rad
    if phi_guess is None:
        phi_guess = rotor.initial_inflow_angle(point.axial_velocity_m_s, point.speed_rad_s, point.thrust_N,
                                               float(atmosphere.density()))
    phi = opti.variable(init_guess=phi_guess, lower_bound=-0.3, upper_bound=np.pi / 2 - 1e-3)
    collective_deg = opti.variable(init_guess=collective_guess_deg, lower_bound=collective_guess_deg - 15.0,
                                   upper_bound=collective_guess_deg + 15.0)
    result = rotor.evaluate(point.axial_velocity_m_s, atmosphere, speed_rad_s=point.speed_rad_s,
                            collective_deg=collective_deg, angle_inflow_rad=phi)
    opti.subject_to([result.residual == 0, result.thrust_N / target_N == 1])
    return result, collective_deg


def collective_guess_deg(rotor, point):
    """Collective from the momentum inflow angle at 0.75 R plus the blade-loading angle of attack
    (6 C_T / (sigma a), a = 2 pi); a starting guess only."""
    density_kg_m3 = float(point.atmosphere().density())
    phi_75 = rotor.initial_inflow_angle(point.axial_velocity_m_s, point.speed_rad_s, point.thrust_N, density_kg_m3)
    fractions, _ = rotor.stations()
    speed_tip_m_s = point.speed_rad_s * float(rotor.radius_m)
    blade_loading = point.thrust_N / (density_kg_m3 * float(rotor.area_disk_m2()) * speed_tip_m_s**2)         / float(rotor.solidity_thrust_weighted())
    return float(onp.degrees(onp.interp(0.75, fractions, phi_75) + 6 * blade_loading / (2 * onp.pi)))


def numeric_result(solution, result):
    """A `BemtResult` with every field evaluated at an Opti solution."""
    return BemtResult(**{f.name: onp.squeeze(onp.asarray(solution(getattr(result, f.name)), dtype=float))
                         for f in fields(result)})


def solve_point(rotor, point, *, verbose=False, guess=None):
    """Trim one operating point to its thrust (a numerical analysis, not part of a larger Opti). `guess`, a
    (BemtResult, collective_deg) pair from a nearby point, sets the starting inflow angles and collective."""
    opti = asb.Opti()
    if guess is None:
        result, collective = add_operating_point(opti, rotor, point,
                                                 collective_guess_deg=collective_guess_deg(rotor, point))
    else:
        result, collective = add_operating_point(opti, rotor, point, collective_guess_deg=float(guess[1]),
                                                 angle_inflow_guess_rad=guess[0].angle_inflow_rad)
    solution = opti.solve(verbose=verbose)
    return numeric_result(solution, result), float(solution(collective))


def solve_point_stepped(rotor, point, fractions_thrust=(0.4, 0.7, 1.0), *, verbose=False):
    """`solve_point` reached through a fixed, finite list of thrust fractions, each solve starting from the last
    (a continuation of starting points, as the multistart in plan 029; nothing is iterated to a fixed point)."""
    guess = None
    for fraction in fractions_thrust:
        guess = solve_point(rotor, replace(point, thrust_N=point.thrust_N * fraction), verbose=verbose, guess=guess)
    return guess


if __name__ == "__main__":
    rotor = BladeElementRotor()
    point = OperatingPoint("hover", 0.0, 0.0, 49.0, 35000.0)
    result, collective = solve_point(rotor, point)
    print(f"collective {collective:.2f} deg, power {result.shaft_power_W / 1e3:.1f} kW, "
          f"FM {result.figure_of_merit:.3f}, CT/sigma {result.blade_loading:.3f}")
