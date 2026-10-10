"""Proprotor whirl flutter in airplane mode: a gimballed rotor coupled to a flexible wing through hub motion.

The formulation follows Johnson's proprotor analysis (NASA TN D-7677, 1974; Rotorcraft Aeromechanics, 2013) at
the reduced order of a conceptual design tool:

- **Rotor:** a gimbal (tip-path-plane tilt beta_1c, beta_1s in the non-rotating frame) with a gimbal spring,
  blade lag and coning bending along the section principal axes pitched by the collective, quasi-static pitch-gimbal
  (delta-3, growing with collective), pitch-lag and pitch-cone couplings, and the drive train. Blade-element
  aerodynamics linearized about the steady high-inflow state (the large inflow angle is kept, not small-angle),
  uniform steady angle of attack set by the rotor thrust, optional Prandtl-Glauert lift slope with lift divergence,
  constant profile drag, no tip loss, quasi-steady (no dynamic inflow), no blade torsion dynamics.
- **Hub loads** from hub motion: thrust, in-plane H and Y forces (blade drag/inflow terms, thrust tilt with the
  blade slope, blade inertia), the gimbal-spring moment, the shaft torque, and the follower turn of the steady thrust
  and torque with the shaft. The reaction of a frictionless gimbal is its spring only.
- **Airframe:** any linear model seen from the hub (`AirframeDynamics`): either published hub modes (frequency,
  modal mass, structural damping, hub displacement and rotation per mode, the CAMRAD input form) or the cantilever
  wing with a rigid nacelle built from the AFDD wing stiffnesses (`cantilever_wing_nacelle_airframe`). No wing
  aerodynamics (CAMRAD practice in the published XV-15 studies: conservative).

Aircraft axes for the airframe: x forward, y to the right (outboard for the right rotor), z down. The right rotor
spins about +x (counter-clockwise seen from ahead, `rotation_sense=+1`, the CAMRAD XV-15 right rotor); mirror for
the other sense. Rotor axes: z_r along the shaft (thrust, forward), x_r up, y_r right; azimuth from x_r.

`whirl_flutter_matrices` builds M, C, K with `aerosandbox.numpy` so that they stay symbolic in the airframe and
rotor inputs (for a later smooth eigenpair constraint inside `asb.Opti`). The eigenvalue sweep and the flutter speed
(`whirl_modes`, `flutter_speed`) are numerical checks outside the optimization.

Against CAMRAD II on the XV-15 (examples/whirl_flutter_xv15.py) the model reproduces the 60 kt thick-to-thin wing
drop but sits about 64 kt high (non-conservative); a calibrated airspeed knockdown of 0.84 is needed.
"""
from dataclasses import dataclass, field
from typing import Any, Callable

import aerosandbox.numpy as np
import numpy as onp


@dataclass(frozen=True)
class AirframeDynamics:
    """Linear airframe seen from the right rotor hub: M q'' + C q' + K q = Q (generalized hub loads).

    Matrices are n x n nested lists (entries may be symbolic). `displacement_hub` and `rotation_hub` are three rows
    (x, y, z aircraft axes) of n: hub displacement in m and rotation in rad per unit generalized coordinate."""
    mass: Any
    damping: Any
    stiffness: Any
    displacement_hub: Any
    rotation_hub: Any
    labels: tuple

    @property
    def count(self):
        return len(self.labels)


def airframe_from_hub_modes(frequency_rad_s, mass_modal_kg, damping_structural, displacement_hub, rotation_hub,
                            labels):
    """Airframe from normal modes given at the hub (CAMRAD's QFREQ, QMASS, QDAMP, LSHAPE, ASHAPE form).

    Per mode: frequency, modal mass (consistent with the shapes), structural damping g (viscous equivalent at the
    mode frequency, damping ratio g / 2), hub displacement (x, y, z) and rotation (x, y, z) per unit modal
    coordinate. Shapes are lists per mode; they are stored transposed (three rows of n)."""
    n = len(labels)
    mass = [[mass_modal_kg[i] if i == j else 0.0 for j in range(n)] for i in range(n)]
    stiffness = [[mass_modal_kg[i] * frequency_rad_s[i]**2 if i == j else 0.0 for j in range(n)] for i in range(n)]
    damping = [[damping_structural[i] * mass_modal_kg[i] * frequency_rad_s[i] if i == j else 0.0 for j in range(n)]
               for i in range(n)]
    displacement = [[displacement_hub[j][k] for j in range(n)] for k in range(3)]
    rotation = [[rotation_hub[j][k] for j in range(n)] for k in range(3)]
    return AirframeDynamics(mass, damping, stiffness, displacement, rotation, tuple(labels))


def cantilever_wing_nacelle_airframe(length_m, stiffness_beam_Nm2, stiffness_chord_Nm2, stiffness_torsion_Nm2,
                                     mass_wing_per_length_kg_m, inertia_torsion_wing_per_length_kg_m,
                                     mass_nacelle_kg, x_cg_nacelle_m, inertia_nacelle_kg_m2, x_hub_m,
                                     z_cg_nacelle_m=0.0, z_hub_m=0.0, damping_structural=(0.03, 0.03, 0.03)):
    """Cantilever wing semispan with a rigid nacelle at the tip, three assumed modes (Rayleigh-Ritz).

    Coordinates: beam bending (tip displacement, + down), chord bending (tip displacement, + forward) and torsion
    (tip rotation, + nose up). Bending uses the static tip-load shape (3 eta^2 - eta^3) / 2 (tip slope 1.5 / L,
    generalized stiffness 3 EI / L^3); torsion is linear in span (GJ / L). Wing mass and torsional inertia per
    length are uniform with the section CG on the elastic axis. Nacelle offsets are from the tip elastic axis: CG
    and hub `x_*` forward and `z_*` down; `inertia_nacelle_kg_m2` is (roll, pitch, yaw) about the nacelle CG with
    the rotor's non-flapping parts but not the blades' flap inertia (the gimbal carries that). The nacelle mass
    includes the rotor mass. Structural damping g per coordinate is viscous on the uncoupled frequency.

    The cantilever is the fuselage-fixed limit; whole-aircraft symmetric and antisymmetric modes differ (the
    XV-15 antisymmetric beam mode involves fuselage roll)."""
    L = length_m
    # Tip rotation rows (x roll, y pitch, z yaw) and tip displacement rows (x, y, z) per (beam, chord, torsion).
    rotation_tip = [[1.5 / L, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.5 / L, 0.0]]
    displacement_tip = [[0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]

    def point_rows(dx_m, dz_m):
        rx, ry, rz = rotation_tip
        return [[displacement_tip[0][j] + dz_m * ry[j] for j in range(3)],
                [dx_m * rz[j] - dz_m * rx[j] for j in range(3)],
                [displacement_tip[2][j] - dx_m * ry[j] for j in range(3)]]

    cg_rows = point_rows(x_cg_nacelle_m, z_cg_nacelle_m)
    mass_bending_kg = mass_wing_per_length_kg_m * L * 33 / 140
    mass = [[0.0] * 3 for _ in range(3)]
    mass[0][0] = mass_bending_kg
    mass[1][1] = mass_bending_kg
    mass[2][2] = inertia_torsion_wing_per_length_kg_m * L / 3
    for i in range(3):
        for j in range(3):
            mass[i][j] = mass[i][j] + mass_nacelle_kg * sum(row[i] * row[j] for row in cg_rows) + sum(
                inertia_nacelle_kg_m2[k] * rotation_tip[k][i] * rotation_tip[k][j] for k in range(3))
    stiffness_diagonal = [3 * stiffness_beam_Nm2 / L**3, 3 * stiffness_chord_Nm2 / L**3, stiffness_torsion_Nm2 / L]
    stiffness = [[stiffness_diagonal[i] if i == j else 0.0 for j in range(3)] for i in range(3)]
    damping = [[damping_structural[i] * np.sqrt(stiffness_diagonal[i] * mass[i][i]) if i == j else 0.0
                for j in range(3)] for i in range(3)]
    return AirframeDynamics(mass, damping, stiffness, point_rows(x_hub_m, z_hub_m), rotation_tip,
                            ("wing beam", "wing chord", "wing torsion"))


@dataclass(frozen=True)
class GimballedProprotor:
    """Gimballed proprotor (three or more blades): gimbal tilt plus optional elastic blade bending.

    `pitch_flap_coupling` is K_P = tan(delta_3): blade pitch changes by -K_P x gimbal flap (positive K_P: flap up,
    pitch down); the XV-15 and V-22 have delta_3 = -15 deg. `inertia_flap_blade_kg_m2` is one blade's flap inertia
    about the hub centre (also its lag inertia); `moment_first_blade_kg_m` its first mass moment (None: uniform
    blade, 1.5 I_b / R). The gimbal spring acts on tip-path-plane tilt. `inertia_polar_hub_kg_m2` is the spinning
    non-flapping hub (gyroscopic moment on the pylon, and polar inertia).

    Blade bending: each blade carries up to two elastic rotations about the hub centre along its section principal
    axes, which are pitched by the collective at 0.75 R (theta_0.75 = inflow angle + steady angle of attack, about
    45 deg in cruise). Chordwise bending (`frequency_lag_per_rev`, rotating frequency at zero pitch; cyclic and
    collective) moves the blade cos(theta) in plane and -sin(theta) out of plane; flapwise bending moves it
    cos(theta) out of plane and sin(theta) in plane. Flapwise bending is the collective coning mode
    (`frequency_coning_per_rev`, the yoke/blade coning of a gimballed hub) and optionally a cyclic mode
    (`frequency_flap_bending_per_rev`; with the gimbal free, the cyclic elastic flap mode is the higher one). None
    omits a mode; with every one omitted the blades are rigid. This pitched-axis coupling of lag with flap is what
    makes a stiff-inplane rotor's lag modes matter in high inflow (Johnson, NASA TN D-7677). Each elastic mode has
    viscous damping ratio `damping_blade_ratio`. Flap-lag Coriolis coupling through steady coning and blade
    torsion are not modelled.

    Compressibility (`speed_sound_m_s`; None: incompressible): the section lift slope is `lift_slope` /
    sqrt(1 - M^2) on the helical section Mach number, and above `mach_lift_divergence` it falls linearly to zero at
    M = 1 (Johnson's lift-divergence approximation, NASA TM X-73213). The slope is taken at the steady state.

    Pitch couplings (quasi-static; blade pitch dynamics on a flexible control system reduce to these, Johnson,
    NASA TM X-73213): pitch-gimbal K_P (above; with `collective_pitch_horn_level_rad`, the collective at 0.75 R at
    which the pitch horn is level, it grows with collective as K_P / cos(theta_0.75 - that collective)),
    `pitch_lag_coupling` K_Pz (positive lag back / pitch down, per rad of chordwise bending) and
    `pitch_cone_coupling` (positive cone up / pitch down). A gimballed stiff-inplane rotor without a lag hinge has a
    large elastic pitch-lag coupling from lag moments about the pitch axis through the control-system flexibility;
    Johnson found it the major influence on the XV-15 stability boundary.

    Drive train (rotor torque path): the hub and the drive (`inertia_drive_kg_m2`, engine and gearbox inertia
    reflected to rotor speed) turn with the nacelle. `drive="free"` adds the drive azimuth relative to the nacelle
    as a degree of freedom (rigid drive train, optional governor spring and damper on it): the symmetric modes of a
    cross-shafted tiltrotor. `drive="locked"` holds it to the nacelle: the antisymmetric modes, where a stiff
    interconnect shaft prevents opposite rotor-speed changes. In both, the aerodynamic torque from hub axial velocity
    and blade speed change loads the nacelle about the shaft (the wing beam slope), and the rotor's polar inertia
    rolls with it."""
    radius_m: Any
    count_blades: int
    chord_m: Any
    inertia_flap_blade_kg_m2: Any
    stiffness_gimbal_Nm_rad: Any = 0.0
    pitch_flap_coupling: Any = 0.0
    lift_slope: Any = 5.7
    drag_coefficient: Any = 0.01
    speed_sound_m_s: Any = None
    mach_lift_divergence: Any = None
    fraction_root_cutout: Any = 0.12
    inertia_polar_hub_kg_m2: Any = 0.0
    collective_pitch_horn_level_rad: Any = None
    pitch_lag_coupling: Any = 0.0
    pitch_cone_coupling: Any = 0.0
    frequency_lag_per_rev: Any = None
    frequency_coning_per_rev: Any = None
    frequency_flap_bending_per_rev: Any = None
    damping_blade_ratio: Any = 0.0
    moment_first_blade_kg_m: Any = None
    inertia_drive_kg_m2: Any = 0.0
    drive: str = "locked"
    stiffness_governor_Nm_rad: Any = 0.0
    damping_governor_Nms_rad: Any = 0.0
    rotation_sense: int = 1
    count_quadrature: int = 16

    def lock_number(self, density_kg_m3):
        return density_kg_m3 * self.lift_slope * self.chord_m * self.radius_m**4 / self.inertia_flap_blade_kg_m2

    def frequency_flap_per_rev(self, speed_rotor_rad_s):
        """Gimbal (cyclic flap) frequency in per rev: nu^2 = 1 + K_gimbal / ((N / 2) I_b Omega^2)."""
        return np.sqrt(1 + self.stiffness_gimbal_Nm_rad
                       / (self.count_blades / 2 * self.inertia_flap_blade_kg_m2 * speed_rotor_rad_s**2))

    def moment_first_kg_m(self):
        if self.moment_first_blade_kg_m is not None:
            return self.moment_first_blade_kg_m
        return 1.5 * self.inertia_flap_blade_kg_m2 / self.radius_m


@dataclass(frozen=True)
class RotorCoefficients:
    """Blade-element integrals (per blade, non-dimensional, lift slope included) about the steady state.

    F_z is the section force along the thrust, F_x the in-plane force opposing rotation; U_P the inflow and U_T the
    tangential velocity; theta the blade pitch."""
    inflow_ratio: Any
    angle_of_attack_rad: Any
    flap_inflow: Any          # int r^2 dFz/dUP: aerodynamic flap damping (negative)
    flap_tangential: Any      # int r dFz/dUT
    flap_lag: Any             # int r^2 dFz/dUT
    flap_axial: Any           # int r dFz/dUP (flap moment from axial hub velocity; thrust from coning rate)
    flap_pitch: Any           # int r dFz/dtheta
    inplane_inflow: Any       # int r dFx/dUP (also the torque from inflow)
    inplane_tangential: Any   # int dFx/dUT
    inplane_pitch: Any        # int dFx/dtheta
    lag_inflow: Any           # int r^2 dFx/dUP
    lag_tangential: Any       # int r dFx/dUT
    lag_pitch: Any            # int r dFx/dtheta
    torque_tangential: Any    # int r^2 dFx/dUT (torque and lag damping from blade speed change)
    thrust_inflow: Any        # int dFz/dUP
    thrust_pitch: Any         # int dFz/dtheta
    thrust_steady: Any        # int Fz0
    torque_steady: Any        # int r Fx0


def rotor_coefficients(rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N=0.0):
    """Linearized blade-element coefficients at airplane-mode axial flight.

    Velocities are in tip-speed units: U_T = r, U_P = lambda = (V + v_i) / (Omega R) with momentum-theory induced
    velocity in axial climb. Section forces per 1/2 rho c (Omega R)^2: normal (thrust direction)
    Fz = a alpha U U_T - c_d U U_P and in-plane (opposing rotation) Fx = a alpha U U_P + c_d U U_T. The steady angle
    of attack alpha is uniform and gives the requested rotor thrust (no trim iteration). With compressibility the lift
    slope `a` varies along the span (see `GimballedProprotor`)."""
    radius_m, a, cd = rotor.radius_m, rotor.lift_slope, rotor.drag_coefficient
    area_disk_m2 = np.pi * radius_m**2
    velocity_induced_m_s = -velocity_m_s / 2 + np.sqrt(velocity_m_s**2 / 4
                                                       + np.fmax(thrust_N, 0.0) / (2 * density_kg_m3 * area_disk_m2))
    lam = (velocity_m_s + velocity_induced_m_s) / (speed_rotor_rad_s * radius_m)
    nodes, weights = onp.polynomial.legendre.leggauss(rotor.count_quadrature)
    r0 = rotor.fraction_root_cutout
    radii = [r0 + (1 - r0) * (x + 1) / 2 for x in nodes]
    weights = [(1 - r0) / 2 * w for w in weights]
    force_scale_N = 0.5 * density_kg_m3 * rotor.chord_m * speed_rotor_rad_s**2 * radius_m**3
    slopes = [a for _ in radii]
    if rotor.speed_sound_m_s is not None:
        speed_tip_m_s = speed_rotor_rad_s * radius_m
        slopes = []
        for r in radii:
            mach = np.sqrt(r**2 + lam**2) * speed_tip_m_s / rotor.speed_sound_m_s
            if rotor.mach_lift_divergence is None:
                slopes.append(a / np.sqrt(1 - mach**2))
            else:
                m_div = rotor.mach_lift_divergence
                mach_capped = np.fmin(mach, m_div)
                slopes.append(a / np.sqrt(1 - mach_capped**2) * np.fmin(1.0, (1 - mach) / (1 - m_div)))
    sum_u_ut = sum(w * s * np.sqrt(r**2 + lam**2) * r for r, w, s in zip(radii, weights, slopes))
    sum_u_up = sum(w * np.sqrt(r**2 + lam**2) * lam for r, w in zip(radii, weights))
    alpha = (thrust_N / (rotor.count_blades * force_scale_N) + cd * sum_u_up) / sum_u_ut
    names = ("flap_inflow", "flap_tangential", "flap_lag", "flap_axial", "flap_pitch", "inplane_inflow", "inplane_tangential",
             "inplane_pitch", "lag_inflow", "lag_tangential", "lag_pitch", "torque_tangential", "thrust_inflow",
             "thrust_pitch", "thrust_steady", "torque_steady")
    totals = dict.fromkeys(names, 0.0)
    for r, w, a in zip(radii, weights, slopes):
        ut, up = r, lam
        u = np.sqrt(ut**2 + up**2)
        fz_p = -a * ut**2 / u + a * alpha * up * ut / u - cd * (up**2 / u + u)
        fz_t = a * up * ut / u + a * alpha * (ut**2 / u + u) - cd * up * ut / u
        fz_theta = a * u * ut
        fx_p = -a * ut * up / u + a * alpha * (up**2 / u + u) + cd * ut * up / u
        fx_t = a * up**2 / u + a * alpha * up * ut / u + cd * (ut**2 / u + u)
        fx_theta = a * u * up
        for name, value in (("flap_inflow", r**2 * fz_p), ("flap_tangential", r * fz_t), ("flap_lag", r**2 * fz_t),
                            ("flap_axial", r * fz_p),
                            ("flap_pitch", r * fz_theta), ("inplane_inflow", r * fx_p), ("inplane_tangential", fx_t),
                            ("inplane_pitch", fx_theta), ("lag_inflow", r**2 * fx_p), ("lag_tangential", r * fx_t),
                            ("lag_pitch", r * fx_theta), ("torque_tangential", r**2 * fx_t),
                            ("thrust_inflow", fz_p), ("thrust_pitch", fz_theta), ("thrust_steady", a * alpha * u * ut - cd * u * up),
                            ("torque_steady", r * (a * alpha * u * up + cd * u * ut))):
            totals[name] = totals[name] + w * value
    return RotorCoefficients(inflow_ratio=lam, angle_of_attack_rad=alpha, **totals)


@dataclass(frozen=True)
class _Linear:
    """A quantity linear in the states: damping row (on rates) and stiffness row (on displacements)."""
    c: list
    k: list

    def __add__(self, other):
        return _Linear([a + b for a, b in zip(self.c, other.c)], [a + b for a, b in zip(self.k, other.k)])

    def __rmul__(self, factor):
        return _Linear([factor * a for a in self.c], [factor * a for a in self.k])


def _row_combination(*terms):
    """Sum of coefficient x row for (coefficient, row) pairs; rows are equal-length lists."""
    length = len(terms[0][1])
    return [sum(c * row[i] for c, row in terms) for i in range(length)]


def state_labels(airframe, rotor):
    """Names of the states of `whirl_flutter_matrices`, in order."""
    labels = list(airframe.labels) + ["gimbal cos", "gimbal sin"]
    if rotor.frequency_flap_bending_per_rev is not None:
        labels += ["flap bending cos", "flap bending sin"]
    if rotor.frequency_coning_per_rev is not None:
        labels += ["coning"]
    if rotor.frequency_lag_per_rev is not None:
        labels += ["lag cos", "lag sin", "lag collective"]
    if rotor.drive == "free":
        labels += ["drive"]
    return tuple(labels)


def collective_pitch_rad(rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N=0.0):
    """Blade pitch at 0.75 R in the steady state: inflow angle there plus the uniform angle of attack."""
    co = rotor_coefficients(rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N)
    return np.arctan2(co.inflow_ratio, 0.75) + co.angle_of_attack_rad


def whirl_flutter_matrices(airframe, rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N=0.0):
    """M, C, K of the coupled rotor-airframe system; states in `state_labels` order.

    States: the airframe coordinates; gimbal tilt beta_1c, beta_1s; blade bending in multiblade coordinates (cyclic
    flapwise, coning, cyclic and collective chordwise), each an elastic rotation of the blade about the hub centre
    along its pitched principal axes; with a free drive, the drive azimuth relative to the nacelle.

    Per blade, the out-of-plane slope is phi_w = beta + cos(theta) q_flap - sin(theta) q_chord and the in-plane
    (lag, opposite to rotation) angle phi_v = sin(theta) q_flap + cos(theta) q_chord. Their rotating-frame
    equations (per blade inertia I_b, rad/s^2) are Johnson's flap equation with shaft motion,
    phi_w'' + phi_w = gamma M_w - (alpha_x'' + 2 alpha_y') sin psi - (2 alpha_x' - alpha_y'') cos psi
    - (S_b / I_b) z_hub'' / Omega^2, and the lag equation phi_v'' = (S_b / I_b) a_hub . e_t / Omega^2
    + Omega_hub' / Omega^2 + gamma M_v. Each blade coordinate's equation is the projection of these on its
    displacement plus its elastic stiffness; the multiblade transform is exact for an axisymmetric rotor in axial
    flow. The airframe and drive equations take the hub loads: thrust (aerodynamic and blade inertia), in-plane
    forces (aerodynamic, thrust tilt with the blade slope, follower turn with the shaft, blade inertia), gimbal-spring
    and gyroscopic moments, the steady drag torque turning with the shaft, and the shaft torque perturbation
    (aerodynamic and blade inertia). M is not symmetric."""
    if rotor.drive not in ("free", "locked"):
        raise ValueError(f"Unknown drive '{rotor.drive}'.")
    labels = state_labels(airframe, rotor)
    size, n = len(labels), airframe.count

    def unit(label):
        return [1.0 if name == label else 0.0 for name in labels]

    zero = [0.0] * size

    def pad(row):
        return list(row) + [0.0] * (size - n)

    omega, radius_m, s = speed_rotor_rad_s, rotor.radius_m, rotor.rotation_sense
    co = rotor_coefficients(rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N)
    lam, kp, count = co.inflow_ratio, rotor.pitch_flap_coupling, rotor.count_blades
    pitch = np.arctan2(lam, 0.75) + co.angle_of_attack_rad
    cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
    g_lock = rotor.lock_number(density_kg_m3) / (2 * rotor.lift_slope)
    force_scale_N = 0.5 * density_kg_m3 * rotor.chord_m * omega**2 * radius_m**3
    half_N = count / 2 * force_scale_N
    thrust_total_N = count * force_scale_N * co.thrust_steady
    inertia_blade = rotor.inertia_flap_blade_kg_m2
    ratio_moment = rotor.moment_first_kg_m() / inertia_blade
    polar = rotor.inertia_polar_hub_kg_m2

    d, rot = airframe.displacement_hub, airframe.rotation_hub
    # Hub motion in rotor axes (x_r up = -z, y_r = y, z_r = x; tilt about x_r = -rotation z, about y_r = rotation y).
    hub_x, hub_y, hub_z = pad([-v for v in d[2]]), pad([s * v for v in d[1]]), pad(d[0])
    tilt_x, tilt_y = pad([-s * v for v in rot[2]]), pad(rot[1])
    e_c, e_s = unit("gimbal cos"), unit("gimbal sin")
    f_c, f_s, f_0 = unit("flap bending cos"), unit("flap bending sin"), unit("coning")
    l_c, l_s, l_0 = unit("lag cos"), unit("lag sin"), unit("lag collective")
    # Hub (shaft) rotation rate perturbation in the rotor's sense: drive azimuth plus nacelle roll about the shaft.
    rate_hub = _row_combination((1.0, pad([s * v for v in rot[0]])), (1.0, unit("drive")))
    # Blade out-of-plane slope and in-plane (lag) angle, by harmonic.
    w_c = _row_combination((1.0, e_c), (cos_pitch, f_c), (-sin_pitch, l_c))
    w_s = _row_combination((1.0, e_s), (cos_pitch, f_s), (-sin_pitch, l_s))
    w_0 = _row_combination((cos_pitch, f_0), (-sin_pitch, l_0))
    v_c = _row_combination((sin_pitch, f_c), (cos_pitch, l_c))
    v_s = _row_combination((sin_pitch, f_s), (cos_pitch, l_s))
    v_0 = _row_combination((sin_pitch, f_0), (cos_pitch, l_0))

    # Velocity perturbations seen by the sections (non-dimensional; rates / Omega), by harmonic:
    flap_cos = _Linear(_row_combination((1 / omega, w_c), (-1 / omega, tilt_y)), w_s)        # U_P / r
    flap_sin = _Linear(_row_combination((1 / omega, w_s), (1 / omega, tilt_x)), [-v for v in w_c])
    flap_0 = _Linear(_row_combination((1 / omega, w_0)), zero)
    axial = _Linear(_row_combination((1 / (omega * radius_m), hub_z)), zero)                   # U_P, uniform
    tangential_cos = _Linear(_row_combination((1 / (omega * radius_m), hub_y)), _row_combination((lam, tilt_x)))
    tangential_sin = _Linear(_row_combination((-1 / (omega * radius_m), hub_x)), _row_combination((lam, tilt_y)))
    lag_cos = _Linear(_row_combination((1 / omega, v_c)), v_s)                                # -U_T / r
    lag_sin = _Linear(_row_combination((1 / omega, v_s)), [-v for v in v_c])
    speed_0 = _Linear(_row_combination((1 / omega, rate_hub), (-1 / omega, v_0)), zero)       # U_T / r
    if rotor.collective_pitch_horn_level_rad is not None:
        kp = kp / np.cos(pitch - rotor.collective_pitch_horn_level_rad)
    kpz, kpc = rotor.pitch_lag_coupling, rotor.pitch_cone_coupling
    pitch_cos = _Linear(zero, _row_combination((-kp, e_c), (-kpz, l_c)))
    pitch_sin = _Linear(zero, _row_combination((-kp, e_s), (-kpz, l_s)))
    pitch_0 = _Linear(zero, _row_combination((-kpc, f_0), (-kpz, l_0)))

    # Aerodynamic out-of-plane and in-plane blade moments by harmonic, per gamma / (2 a).
    moment_w = {
        "cos": (co.flap_inflow * flap_cos + co.flap_tangential * tangential_cos + (-co.flap_lag) * lag_cos
                + co.flap_pitch * pitch_cos),
        "sin": (co.flap_inflow * flap_sin + co.flap_tangential * tangential_sin + (-co.flap_lag) * lag_sin
                + co.flap_pitch * pitch_sin),
        "0": co.flap_inflow * flap_0 + co.flap_axial * axial + co.flap_lag * speed_0 + co.flap_pitch * pitch_0}
    moment_v = {
        "cos": (co.lag_inflow * flap_cos + co.lag_tangential * tangential_cos + (-co.torque_tangential) * lag_cos
                + co.lag_pitch * pitch_cos),
        "sin": (co.lag_inflow * flap_sin + co.lag_tangential * tangential_sin + (-co.torque_tangential) * lag_sin
                + co.lag_pitch * pitch_sin),
        "0": (co.lag_inflow * flap_0 + co.inplane_inflow * axial + co.torque_tangential * speed_0
              + co.lag_pitch * pitch_0)}

    def rotating(inertial_m, inertial_c, inertial_k, aero):
        """(M, C, K) rows of 'inertial terms - Omega^2 gamma / (2 a) x aero'."""
        return (inertial_m, _row_combination((1.0, inertial_c), (-omega**2 * g_lock, aero.c)),
                _row_combination((1.0, inertial_k), (-omega**2 * g_lock, aero.k)))

    # Out-of-plane (centrifugal stiffness 1/rev^2) and in-plane (none) equations, multiblade.
    eq_w = {"cos": rotating(_row_combination((1.0, w_c), (-1.0, tilt_y)),
                            _row_combination((2 * omega, w_s), (2 * omega, tilt_x)), zero, moment_w["cos"]),
            "sin": rotating(_row_combination((1.0, w_s), (1.0, tilt_x)),
                            _row_combination((-2 * omega, w_c), (2 * omega, tilt_y)), zero, moment_w["sin"]),
            "0": rotating(_row_combination((1.0, w_0), (ratio_moment, hub_z)), zero,
                          _row_combination((omega**2, w_0)), moment_w["0"])}
    eq_v = {"cos": rotating(_row_combination((1.0, v_c), (-ratio_moment, hub_y)),
                            _row_combination((2 * omega, v_s)), _row_combination((-omega**2, v_c)), moment_v["cos"]),
            "sin": rotating(_row_combination((1.0, v_s), (ratio_moment, hub_x)),
                            _row_combination((-2 * omega, v_c)), _row_combination((-omega**2, v_s)), moment_v["sin"]),
            "0": rotating(_row_combination((1.0, v_0), (-1.0, rate_hub)), zero, zero, moment_v["0"])}

    def project(weight_w, weight_v, harmonic, unit_row, stiffness_per_inertia, frequency_per_rev):
        """Blade coordinate equation: weight_w x out-of-plane + weight_v x in-plane equation + elastic terms."""
        a, b = eq_w[harmonic], eq_v[harmonic]
        damper = 2 * rotor.damping_blade_ratio * frequency_per_rev * omega
        return (_row_combination((weight_w, a[0]), (weight_v, b[0])),
                _row_combination((weight_w, a[1]), (weight_v, b[1]), (damper, unit_row)),
                _row_combination((weight_w, a[2]), (weight_v, b[2]), (stiffness_per_inertia, unit_row)))

    nu2_gimbal = rotor.frequency_flap_per_rev(omega)**2
    rows = {"gimbal cos": project(1.0, 0.0, "cos", e_c, (nu2_gimbal - 1) * omega**2, 0.0),
            "gimbal sin": project(1.0, 0.0, "sin", e_s, (nu2_gimbal - 1) * omega**2, 0.0)}
    if rotor.frequency_flap_bending_per_rev is not None:
        nu = rotor.frequency_flap_bending_per_rev
        for harmonic, row in (("cos", f_c), ("sin", f_s)):
            rows[f"flap bending {harmonic}"] = project(cos_pitch, sin_pitch, harmonic, row, (nu**2 - 1) * omega**2, nu)
    if rotor.frequency_coning_per_rev is not None:
        nu = rotor.frequency_coning_per_rev
        rows["coning"] = project(cos_pitch, sin_pitch, "0", f_0, (nu**2 - 1) * omega**2, nu)
    if rotor.frequency_lag_per_rev is not None:
        nu = rotor.frequency_lag_per_rev
        for harmonic, row in (("cos", l_c), ("sin", l_s), ("0", l_0)):
            label = "lag collective" if harmonic == "0" else f"lag {harmonic}"
            rows[label] = project(-sin_pitch, cos_pitch, harmonic, row, nu**2 * omega**2, nu)

    # Hub loads in rotor axes on the airframe: inertia row (on accelerations) and _Linear (rates, displacements).
    inplane_cos = (co.inplane_inflow * flap_cos + co.inplane_tangential * tangential_cos
                   + (-co.lag_tangential) * lag_cos + co.inplane_pitch * pitch_cos)
    inplane_sin = (co.inplane_inflow * flap_sin + co.inplane_tangential * tangential_sin
                   + (-co.lag_tangential) * lag_sin + co.inplane_pitch * pitch_sin)
    moment_first_kg_m = rotor.moment_first_kg_m()
    force_x = (_row_combination((-count / 2 * moment_first_kg_m, v_s)),
               half_N * inplane_sin + _Linear(zero, _row_combination((-half_N * co.thrust_steady, w_c),
                                                                     (thrust_total_N, tilt_y))))
    force_y = (_row_combination((count / 2 * moment_first_kg_m, v_c)),
               (-half_N) * inplane_cos + _Linear(zero, _row_combination((-half_N * co.thrust_steady, w_s),
                                                                        (-thrust_total_N, tilt_x))))
    force_z = (_row_combination((-count * moment_first_kg_m, w_0)),
               (count * force_scale_N) * (co.thrust_inflow * axial + co.flap_axial * flap_0
                                          + co.flap_tangential * speed_0 + co.thrust_pitch * pitch_0))
    stiffness_gimbal = count / 2 * (nu2_gimbal - 1) * inertia_blade * omega**2
    # Gimbal spring, hub gyroscopic moment, and the steady aerodynamic drag torque turning with the shaft (a follower
    # moment: the drive reaction stays inside the nacelle, so only the rotor's aerodynamic torque acts outside).
    torque_steady_Nm = count * force_scale_N * radius_m * co.torque_steady
    moment_x = (zero, _Linear(_row_combination((-polar * omega, tilt_y)),
                              _row_combination((stiffness_gimbal, e_s), (-torque_steady_Nm, tilt_y))))
    moment_y = (zero, _Linear(_row_combination((polar * omega, tilt_x)),
                              _row_combination((-stiffness_gimbal, e_c), (torque_steady_Nm, tilt_x))))
    # Aerodynamic drag torque on the blades (opposing rotation), all blades.
    torque = (count * force_scale_N * radius_m) * moment_v["0"]

    mass = [[0.0] * size for _ in range(size)]
    damping = [[0.0] * size for _ in range(size)]
    stiffness = [[0.0] * size for _ in range(size)]
    for i in range(n):
        for j in range(n):
            mass[i][j] = airframe.mass[i][j]
            damping[i][j] = airframe.damping[i][j]
            stiffness[i][j] = airframe.stiffness[i][j]
        for kinematic, (load_m, load) in ((hub_x, force_x), (hub_y, force_y), (hub_z, force_z), (tilt_x, moment_x),
                                          (tilt_y, moment_y)):
            for j in range(size):
                mass[i][j] = mass[i][j] - kinematic[i] * load_m[j]
                damping[i][j] = damping[i][j] - kinematic[i] * load.c[j]
                stiffness[i][j] = stiffness[i][j] - kinematic[i] * load.k[j]
    for label, (row_m, row_c, row_k) in rows.items():
        index = labels.index(label)
        mass[index], damping[index], stiffness[index] = row_m, row_c, row_k
    # Shaft torque on the airframe and drive (the rows where the hub rate has a component): hub polar inertia, blade
    # inertia at the blade rate (hub rate less the in-plane collective rate), aerodynamic torque; drive inertia and
    # governor on the drive azimuth.
    drive = unit("drive")
    for i in range(size):
        for j in range(size):
            mass[i][j] = mass[i][j] + rate_hub[i] * (polar * rate_hub[j]
                                                     + count * inertia_blade * (rate_hub[j] - v_0[j])) \
                + rotor.inertia_drive_kg_m2 * drive[i] * drive[j]
            damping[i][j] = damping[i][j] + rate_hub[i] * torque.c[j] \
                + rotor.damping_governor_Nms_rad * drive[i] * drive[j]
            stiffness[i][j] = stiffness[i][j] + rotor.stiffness_governor_Nm_rad * drive[i] * drive[j]
    return np.array(mass), np.array(damping), np.array(stiffness)


@dataclass(frozen=True)
class WhirlModes:
    """Eigen-solution at one flight condition (numerical). One entry per oscillatory mode (and per real root)."""
    velocity_m_s: float
    frequency_rad_s: tuple
    damping_ratio: tuple
    labels: tuple
    eigenvalues: tuple = field(repr=False)

    def least_damped(self):
        i = int(onp.argmin(self.damping_ratio))
        return self.labels[i], self.damping_ratio[i], self.frequency_rad_s[i]


def _numeric(matrix):
    return onp.asarray(matrix, dtype=float)


def whirl_modes(airframe, rotor, velocity_m_s, speed_rotor_rad_s, density_kg_m3, thrust_N=0.0):
    """Eigenvalues of the coupled system; each mode labelled by its dominant coordinate (kinetic-energy share)."""
    mass, damping, stiffness = (_numeric(m) for m in whirl_flutter_matrices(airframe, rotor, velocity_m_s,
                                                                           speed_rotor_rad_s, density_kg_m3, thrust_N))
    size = mass.shape[0]
    inverse = onp.linalg.inv(mass)
    state = onp.block([[onp.zeros((size, size)), onp.eye(size)], [-inverse @ stiffness, -inverse @ damping]])
    values, vectors = onp.linalg.eig(state)
    # Energy weights: airframe diagonal masses; gimbal coordinates weighted by N/2 I_b (consistent kinetic energy).
    names, weights = [], []
    blade = float(rotor.inertia_flap_blade_kg_m2)
    for i, label in enumerate(state_labels(airframe, rotor)):
        if i < airframe.count:
            names.append(label)
            weights.append(abs(float(_numeric(airframe.mass)[i, i])))
        else:
            names.append(dict(drive="rotor speed").get(label, label.replace(" cos", "").replace(" sin", "")))
            weights.append(dict(drive=float(rotor.inertia_drive_kg_m2) + float(rotor.inertia_polar_hub_kg_m2))
                           .get(label, (rotor.count_blades if label in ("lag collective", "coning") else rotor.count_blades / 2)
                                * blade))
    weights = onp.array(weights)
    frequency, damping_ratio, labels, kept = [], [], [], []
    for value, vector in zip(values, vectors.T):
        if value.imag < -1e-9:
            continue                                   # one of each conjugate pair
        energy = weights * onp.abs(vector[:size])**2
        frequency.append(abs(value.imag))
        damping_ratio.append(-value.real / max(abs(value), 1e-12))
        labels.append(names[int(onp.argmax(energy))])
        kept.append(value)
    return WhirlModes(float(velocity_m_s), tuple(frequency), tuple(damping_ratio), tuple(labels), tuple(kept))


def flutter_speed(airframe, rotor, speed_rotor_rad_s, density_kg_m3, velocities_m_s,
                  thrust_N: Callable[[float], float] | float = 0.0, tolerance_m_s=0.05):
    """Lowest airspeed where any mode's damping crosses zero (scan, then bisection), and that mode's label.

    `thrust_N` is a constant or a function of airspeed (the trim criterion, e.g. limited power). Returns
    (None, None) when every mode stays stable over `velocities_m_s`."""
    def thrust_at(v):
        return thrust_N(v) if callable(thrust_N) else thrust_N

    def least(v):
        return whirl_modes(airframe, rotor, v, speed_rotor_rad_s, density_kg_m3, thrust_at(v)).least_damped()

    previous = None
    for v in velocities_m_s:
        label, zeta, _ = least(v)
        if zeta < 0:
            if previous is None:
                return float(v), label
            low, high = previous, float(v)
            while high - low > tolerance_m_s:
                mid = 0.5 * (low + high)
                if least(mid)[1] < 0:
                    high = mid
                else:
                    low = mid
            return high, least(high)[0]
        previous = float(v)
    return None, None


if __name__ == "__main__":
    # Sanity case: a stiff pylon with a lightly loaded rotor stays stable; its gimbal modes sit near (nu +- 1)/rev.
    stiff = cantilever_wing_nacelle_airframe(4.0, 1e9, 1e9, 1e9, 30.0, 5.0, 900.0, 0.3, (100.0, 400.0, 400.0), 1.2)
    rotor = GimballedProprotor(radius_m=3.81, count_blades=3, chord_m=0.356, inertia_flap_blade_kg_m2=140.0,
                               stiffness_gimbal_Nm_rad=17000.0, pitch_flap_coupling=np.tan(np.radians(-15)))
    modes = whirl_modes(stiff, rotor, 150.0, 48.0, 1.225, 5000.0)
    for f, z, label in zip(modes.frequency_rad_s, modes.damping_ratio, modes.labels):
        print(f"{label:14s} {f / 48.0:6.3f} /rev  damping {z:7.4f}")
