"""Classic bending-torsion flutter of a cantilever wing with a rigid tip body (plan 056).

The plan 040 whirl model has no wing aerodynamics and three assumed wing shapes. For a heavy concentrated mass the
classical literature finds assumed first bending and torsion modes unreliable (Runyan and Watkins, NACA TN 1848 /
Report 966, 1949: a weight of 92 % of the wing mass needed the exact differential-equation solution), so this module
discretizes the wing with beam finite elements and uses Theodorsen's unsteady strip theory:

- **Structure:** uniform cantilever, Hermite cubic bending (w down, slope w'), linear torsion (theta, nose up) and
  optional chord bending (v forward, slope v'); the section CG may sit off the elastic axis. A rigid tip body
  (nacelle) has its own mass, CG offset ahead of the elastic axis and inertias.
- **Aerodynamics:** Theodorsen strip theory (NACA Report 496) per Gauss point: non-circulatory lift and moment, and
  the circulatory part through R. T. Jones' two-lag approximation of C(k) (A = 0.165, 0.335; b = 0.0455, 0.3), so the
  flutter problem is an exact eigenvalue problem in airspeed (no k-iteration). Optional Yates-type finite-span
  correction: the circulatory lift scaled by a / (2 pi). No aerodynamic loads on chord bending.
- **Symbolic data flow:** the mesh integrals are numerical (they depend on the span and element count only); the mass,
  stiffness and aerodynamic matrices are combinations of them with the wing, tip, airspeed and density parameters,
  so they stay CasADi-symbolic. The eigenvalue scan and flutter-speed bisection are numerical checks.
- **Verification:** the Goland wing (Goland 1945; 137.2 m/s, 70.7 rad/s at 1.225 kg/m^3 with strip theory).

Sign conventions follow Theodorsen: h (here w) positive down, theta positive leading edge up, `a` the elastic axis
behind mid-chord in semichords. Offsets in this module are positive AHEAD of the elastic axis (plan 040 nacelle
convention).
"""
from dataclasses import dataclass, replace

import aerosandbox.numpy as np
import casadi as cas
import numpy as onp

_gauss_x, _gauss_w = onp.polynomial.legendre.leggauss(4)
_gauss_x, _gauss_w = 0.5 * (_gauss_x + 1), 0.5 * _gauss_w


@dataclass(frozen=True)
class UniformWing:
    """Uniform cantilever semispan. `inertia_torsion_kg_m` is per length about the elastic axis (includes m d^2).

    `length_m` sets the mesh and must be numerical; the other fields may be CasADi symbols."""
    length_m: float
    chord_m: float
    stiffness_beam_Nm2: float
    stiffness_torsion_Nm2: float
    mass_kg_m: float
    inertia_torsion_kg_m: float
    fraction_elastic_axis: float            # chord fraction of the elastic axis from the leading edge
    offset_cg_m: float = 0.0                # section CG ahead of the elastic axis (negative: behind)
    stiffness_chord_Nm2: float = None       # None: no chord bending coordinates

    @property
    def semichord_m(self):
        return self.chord_m / 2

    @property
    def a(self):
        """Theodorsen's elastic-axis position behind mid-chord, in semichords."""
        return 2 * self.fraction_elastic_axis - 1


@dataclass(frozen=True)
class TipBody:
    """Rigid body at the wing tip: mass, CG ahead of the elastic axis, inertias about its CG (pitch, roll, yaw)."""
    mass_kg: float
    offset_cg_m: float = 0.0
    inertia_pitch_kg_m2: float = 0.0
    inertia_roll_kg_m2: float = 0.0
    inertia_yaw_kg_m2: float = 0.0


def _layout(wing, count_elements):
    """DOFs per node: w, w', theta (+ v, v'); node 0 clamped (removed)."""
    per_node = 5 if wing.stiffness_chord_Nm2 is not None else 3
    return per_node, per_node * count_elements


def _element_rows(wing, count_elements, element, xi):
    """Rows (over the free DOFs) giving w, w', theta, v, v' at local coordinate xi of an element."""
    per_node, size = _layout(wing, count_elements)
    length_element_m = wing.length_m / count_elements
    h = (1 - 3 * xi**2 + 2 * xi**3, length_element_m * (xi - 2 * xi**2 + xi**3), 3 * xi**2 - 2 * xi**3,
         length_element_m * (-xi**2 + xi**3))
    dh = ((-6 * xi + 6 * xi**2) / length_element_m, 1 - 4 * xi + 3 * xi**2, (6 * xi - 6 * xi**2) / length_element_m,
          -2 * xi + 3 * xi**2)
    rows = {key: onp.zeros(size) for key in ("w", "dw", "theta", "v", "dv")}

    def put(row, node, local, value):
        if node == 0:
            return                                  # clamped root
        row[(node - 1) * per_node + local] += value

    for side, node in ((0, element), (1, element + 1)):
        put(rows["w"], node, 0, h[2 * side]); put(rows["w"], node, 1, h[2 * side + 1])
        put(rows["dw"], node, 0, dh[2 * side]); put(rows["dw"], node, 1, dh[2 * side + 1])
        put(rows["theta"], node, 2, 1 - xi if side == 0 else xi)
        if per_node == 5:
            put(rows["v"], node, 3, h[2 * side]); put(rows["v"], node, 4, h[2 * side + 1])
            put(rows["dv"], node, 3, dh[2 * side]); put(rows["dv"], node, 4, dh[2 * side + 1])
    return rows


def _second_derivative_rows(wing, count_elements, element, xi):
    per_node, size = _layout(wing, count_elements)
    length_element_m = wing.length_m / count_elements
    d2 = ((-6 + 12 * xi) / length_element_m**2, (-4 + 6 * xi) / length_element_m,
          (6 - 12 * xi) / length_element_m**2, (-2 + 6 * xi) / length_element_m)
    w, v, dtheta = onp.zeros(size), onp.zeros(size), onp.zeros(size)
    for side, node in ((0, element), (1, element + 1)):
        if node == 0:
            continue
        base = (node - 1) * per_node
        w[base] += d2[2 * side]; w[base + 1] += d2[2 * side + 1]
        dtheta[base + 2] += (-1 if side == 0 else 1) / length_element_m
        if per_node == 5:
            v[base + 3] += d2[2 * side]; v[base + 4] += d2[2 * side + 1]
    return w, v, dtheta


def tip_rows(wing, count_elements):
    """Rows of the tip: w, w', theta (pitch), v, v' (minus yaw)."""
    return _element_rows(wing, count_elements, count_elements - 1, 1.0)


@dataclass(frozen=True)
class _MeshIntegrals:
    """Spanwise integrals of shape-function products (numerical: they depend on the mesh only) and the rows at each
    Gauss point (point-major) for the aerodynamic lag states."""
    ww: onp.ndarray
    wt: onp.ndarray
    tt: onp.ndarray
    vv: onp.ndarray
    bending: onp.ndarray
    torsion: onp.ndarray
    chord: onp.ndarray
    w_points: onp.ndarray                   # w at each Gauss point times its span weight
    t_points: onp.ndarray
    w_rows: onp.ndarray                     # w at each Gauss point
    t_rows: onp.ndarray


def _mesh_integrals(wing, count_elements):
    _, size = _layout(wing, count_elements)
    length_element_m = wing.length_m / count_elements
    totals = {key: onp.zeros((size, size)) for key in ("ww", "wt", "tt", "vv", "bending", "torsion", "chord")}
    points = {key: [] for key in ("w_points", "t_points", "w_rows", "t_rows")}
    for e in range(count_elements):
        for xi, weight in zip(_gauss_x, _gauss_w):
            r = _element_rows(wing, count_elements, e, xi)
            w2, v2, dtheta = _second_derivative_rows(wing, count_elements, e, xi)
            length_weight_m = weight * length_element_m
            pairs = dict(ww=(r["w"], r["w"]), wt=(r["w"], r["theta"]), tt=(r["theta"], r["theta"]),
                         vv=(r["v"], r["v"]), bending=(w2, w2), torsion=(dtheta, dtheta), chord=(v2, v2))
            for key, (left, right) in pairs.items():
                totals[key] += length_weight_m * onp.outer(left, right)
            points["w_points"].append(length_weight_m * r["w"]); points["t_points"].append(length_weight_m * r["theta"])
            points["w_rows"].append(r["w"]); points["t_rows"].append(r["theta"])
    return _MeshIntegrals(**totals, **{key: onp.array(rows) for key, rows in points.items()})


def _combination(*terms):
    """Sum of coefficient x numerical matrix over (coefficient, matrix) pairs: a NumPy array for numerical
    coefficients, a CasADi matrix if any coefficient is symbolic."""
    if any(np.is_casadi_type(coefficient, recursive=False) for coefficient, _ in terms):
        return sum(cas.DM(matrix) * coefficient for coefficient, matrix in terms)
    return sum(coefficient * matrix for coefficient, matrix in terms)


def structural_matrices(wing, tip=None, count_elements=20):
    """(M, K) of the cantilever wing and the tip body; symbolic in the stiffnesses, masses, inertias and offsets."""
    mesh = _mesh_integrals(wing, count_elements)
    offset_m = wing.offset_cg_m
    # m (w - d theta)^2 + (I_theta - m d^2) theta^2, expanded into parameter x mesh-integral terms.
    mass_terms = [(wing.mass_kg_m, mesh.ww), (-wing.mass_kg_m * offset_m, mesh.wt + mesh.wt.T),
                  (wing.inertia_torsion_kg_m, mesh.tt)]
    stiffness_terms = [(wing.stiffness_beam_Nm2, mesh.bending), (wing.stiffness_torsion_Nm2, mesh.torsion)]
    if wing.stiffness_chord_Nm2 is not None:
        mass_terms.append((wing.mass_kg_m, mesh.vv))
        stiffness_terms.append((wing.stiffness_chord_Nm2, mesh.chord))
    if tip is not None:
        r = tip_rows(wing, count_elements)
        offset_m = tip.offset_cg_m
        mass_terms += [(tip.mass_kg, onp.outer(r["w"], r["w"])),
                       (-tip.mass_kg * offset_m, onp.outer(r["w"], r["theta"]) + onp.outer(r["theta"], r["w"])),
                       (tip.mass_kg * offset_m**2 + tip.inertia_pitch_kg_m2, onp.outer(r["theta"], r["theta"])),
                       (tip.inertia_roll_kg_m2, onp.outer(r["dw"], r["dw"]))]
        if wing.stiffness_chord_Nm2 is not None:
            mass_terms += [(tip.mass_kg, onp.outer(r["v"], r["v"])),
                           (tip.inertia_yaw_kg_m2, onp.outer(r["dv"], r["dv"]))]
    return _combination(*mass_terms), _combination(*stiffness_terms)


def aerodynamic_matrices(wing, count_elements, velocity_m_s, density_kg_m3, factor_lift_slope=1.0,
                         lags_jones=((0.165, 0.0455), (0.335, 0.3))):
    """Theodorsen strip theory with Jones' lags: (M_a, C_a, K_a, D_a, E_q, E_qdot, F) with
    M_a q'' + C_a q' + K_a q + D_a z = -Q_aero (moved to the left) and z' = E_q q + E_qdot q' + F z.

    `lags_jones`: the (A, b) pairs of R. T. Jones' C(k) ~ 1 - sum A_i i k / (i k + b_i). Symbolic in the chord,
    elastic-axis position, airspeed, density and lift-slope factor."""
    mesh = _mesh_integrals(wing, count_elements)
    semichord_m, a = wing.semichord_m, wing.a
    noncirculatory = onp.pi * density_kg_m3 * semichord_m**2
    circulatory = 2 * onp.pi * density_kg_m3 * velocity_m_s * semichord_m * factor_lift_slope
    tw = mesh.wt.T
    M_a = _combination((noncirculatory, mesh.ww), (-noncirculatory * semichord_m * a, mesh.wt + tw),
                       (noncirculatory * semichord_m**2 * (1 / 8 + a**2), mesh.tt))
    # Circulatory lift at 3/4 chord: generalized force per unit lift g = -w + b (a + 1/2) theta, downwash rate
    # w' + b (1/2 - a) theta'; C_a gets -1/2 f g (downwash rate), K_a -1/2 f V g theta.
    C_a = _combination((noncirculatory * velocity_m_s, mesh.wt),
                       (noncirculatory * velocity_m_s * semichord_m * (0.5 - a), mesh.tt),
                       (0.5 * circulatory, mesh.ww), (0.5 * circulatory * semichord_m * (0.5 - a), mesh.wt),
                       (-0.5 * circulatory * semichord_m * (a + 0.5), tw),
                       (-0.5 * circulatory * semichord_m**2 * (a + 0.5) * (0.5 - a), mesh.tt))
    K_a = _combination((0.5 * circulatory * velocity_m_s, mesh.wt),
                       (-0.5 * circulatory * velocity_m_s * semichord_m * (a + 0.5), mesh.tt))
    # One lag state per Gauss point and Jones term (point-major): D_a column -f A b V / b_semi g at that point.
    count_points = len(mesh.w_rows)
    amplitude_rate = onp.tile([amplitude * rate for amplitude, rate in lags_jones], count_points)
    rates = onp.tile([rate for _, rate in lags_jones], count_points)
    repeat = lambda rows: onp.repeat(rows, len(lags_jones), axis=0)
    columns = lambda rows: (repeat(rows) * amplitude_rate[:, None]).T
    D_a = _combination((circulatory * velocity_m_s / semichord_m, columns(mesh.w_points)),
                       (-circulatory * velocity_m_s * (a + 0.5), columns(mesh.t_points)))
    E_q = _combination((velocity_m_s, repeat(mesh.t_rows)))
    E_qdot = _combination((1.0, repeat(mesh.w_rows)), (semichord_m * (0.5 - a), repeat(mesh.t_rows)))
    F = _combination((velocity_m_s / semichord_m, onp.diag(-rates)))
    return M_a, C_a, K_a, D_a, E_q, E_qdot, F


def flutter_eigenvalues(wing, tip, velocity_m_s, density_kg_m3, count_elements=20, factor_lift_slope=1.0,
                        damping_structural=0.0, basis=None):
    """Eigenvalues (one of each conjugate pair, plus real ones) of the aeroelastic system and each mode's label.

    Numerical check on a sized wing (dense eigensolver, outside the optimization): the inputs must be numerical.
    `basis` (columns over the FE DOFs): solve in those modes only (a Rayleigh-type reduced analysis)."""
    M, K = structural_matrices(wing, tip, count_elements)
    M_full = M
    C = onp.zeros_like(M)
    if damping_structural:
        values, vectors = _modes(M, K)
        modal = vectors.T @ M @ vectors
        C = M @ vectors @ onp.diag(damping_structural * onp.sqrt(onp.maximum(values, 0)) / onp.diag(modal)) \
            @ vectors.T @ M
    M_a, C_a, K_a, D_a, E_q, E_qdot, F = aerodynamic_matrices(wing, count_elements, velocity_m_s, density_kg_m3,
                                                              factor_lift_slope)
    if basis is not None:
        P = onp.asarray(basis)
        M, K, C, M_a, C_a, K_a = (P.T @ X @ P for X in (M, K, C, M_a, C_a, K_a))
        D_a, E_q, E_qdot = P.T @ D_a, E_q @ P, E_qdot @ P
    n, m = M.shape[0], F.shape[0]
    inverse = onp.linalg.inv(M + M_a)
    A = onp.zeros((2 * n + m, 2 * n + m))
    A[:n, n:2 * n] = onp.eye(n)
    A[n:2 * n, :n] = -inverse @ (K + K_a)
    A[n:2 * n, n:2 * n] = -inverse @ (C + C_a)
    A[n:2 * n, 2 * n:] = -inverse @ D_a
    A[2 * n:, :n], A[2 * n:, n:2 * n], A[2 * n:, 2 * n:] = E_q, E_qdot, F
    values, vectors = onp.linalg.eig(A)
    per_node, size = _layout(wing, count_elements)
    groups = {"bending": [0, 1], "torsion": [2], "chord": [3, 4]}
    out = []
    for value, vector in zip(values, vectors.T):
        if value.imag < -1e-9:
            continue
        q = vector[:n] if basis is None else onp.asarray(basis) @ vector[:n]
        energy = {}
        for name, locals_ in groups.items():
            index = [node * per_node + j for node in range(size // per_node) for j in locals_ if j < per_node]
            if index:
                part = q[index]
                energy[name] = abs(onp.conj(part) @ M_full[onp.ix_(index, index)] @ part)
        out.append((value, max(energy, key=energy.get)))
    return out


def _modes(M, K):
    values, vectors = onp.linalg.eig(onp.linalg.solve(M, K))
    order = onp.argsort(values.real)
    return values.real[order], vectors.real[:, order]


def natural_frequencies_rad_s(wing, tip=None, count_elements=20):
    M, K = structural_matrices(wing, tip, count_elements)
    return onp.sqrt(onp.maximum(_modes(M, K)[0], 0))


def normal_modes(wing, tip=None, count_elements=20):
    """(frequencies rad/s, mass-normalized mode shapes as columns over the FE DOFs)."""
    M, K = structural_matrices(wing, tip, count_elements)
    values, vectors = _modes(M, K)
    vectors = vectors / onp.sqrt(onp.einsum("ij,ik,kj->j", vectors, M, vectors))
    return onp.sqrt(onp.maximum(values, 0)), vectors


def uncoupled_basis(wing, count_elements=20):
    """First bending and first torsion modes of the bare wing with the section CG on the elastic axis: the classic
    two-mode Rayleigh basis (Runyan and Watkins found it fails with a heavy concentrated mass)."""
    bare = replace(wing, offset_cg_m=0.0, inertia_torsion_kg_m=wing.inertia_torsion_kg_m
                   - wing.mass_kg_m * wing.offset_cg_m**2)
    _, vectors = normal_modes(bare, None, count_elements)
    per_node, _ = _layout(wing, count_elements)
    bending = next(v for v in vectors.T if onp.abs(v[0::per_node]).sum() > 1e-9 and onp.abs(v[2::per_node]).sum() < 1e-6)
    torsion = next(v for v in vectors.T if onp.abs(v[2::per_node]).sum() > 1e-9 and onp.abs(v[0::per_node]).sum() < 1e-6)
    return onp.column_stack([bending, torsion])


def flutter_speed(wing, tip, density_kg_m3, velocities_m_s, count_elements=20, factor_lift_slope=1.0,
                  damping_structural=0.0, tolerance_m_s=0.05, frequency_min_rad_s=0.5, basis=None):
    """Lowest airspeed where an oscillatory mode loses damping (flutter) or a real root turns positive
    (divergence): (speed m/s, frequency rad/s, label) or (None, None, None).

    Numerical check on a sized wing, run outside the optimization: it scans `velocities_m_s` and bisects the first
    unstable interval to `tolerance_m_s`, solving the eigenproblem at each airspeed. The matrices it evaluates
    (`structural_matrices`, `aerodynamic_matrices`) stay symbolic."""
    lag_floor = 1e-6

    def critical(velocity_m_s):
        worst = None
        for value, label in flutter_eigenvalues(wing, tip, velocity_m_s, density_kg_m3, count_elements,
                                                factor_lift_slope, damping_structural, basis):
            if value.real > lag_floor * max(abs(value), 1.0):
                kind = label if abs(value.imag) > frequency_min_rad_s else "divergence"
                if worst is None or value.real > worst[0].real:
                    worst = (value, kind)
        return worst

    previous_m_s = None
    for velocity_m_s in velocities_m_s:
        hit = critical(velocity_m_s)
        if hit is not None:
            if previous_m_s is None:
                return float(velocity_m_s), abs(hit[0].imag), hit[1]
            low_m_s, high_m_s = previous_m_s, float(velocity_m_s)
            while high_m_s - low_m_s > tolerance_m_s:
                mid_m_s = 0.5 * (low_m_s + high_m_s)
                if critical(mid_m_s) is not None:
                    high_m_s = mid_m_s
                else:
                    low_m_s = mid_m_s
            value, kind = critical(high_m_s)
            return high_m_s, abs(value.imag), kind
        previous_m_s = float(velocity_m_s)
    return None, None, None


def lift_slope_factor_helmbold(aspect_ratio):
    """Finite-span lift slope over 2 pi (Helmbold), for a Yates-type strip correction."""
    return aspect_ratio / (2 + np.sqrt(aspect_ratio**2 + 4))


def goland_wing(**changes):
    """Goland (1945) wing as tabulated by Cal (City University, 1992, table 2.4): 20 ft semispan, 6 ft chord,
    elastic axis 33 % chord, CG 0.183 m behind it."""
    data = dict(length_m=6.096, chord_m=1.8288, stiffness_beam_Nm2=9.773e6, stiffness_torsion_Nm2=9.876e5,
                mass_kg_m=35.717, inertia_torsion_kg_m=8.642, fraction_elastic_axis=0.33, offset_cg_m=-0.183)
    data.update(changes)
    return UniformWing(**data)
