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
- **Verification:** the Goland wing (Goland 1945; 137.2 m/s, 70.7 rad/s at 1.225 kg/m^3 with strip theory).

Sign conventions follow Theodorsen: h (here w) positive down, theta positive leading edge up, `a` the elastic axis
behind mid-chord in semichords. Offsets in this module are positive AHEAD of the elastic axis (plan 040 nacelle
convention).
"""
from dataclasses import dataclass

import numpy as onp

jones_lag_terms = ((0.165, 0.0455), (0.335, 0.3))
_gauss_x, _gauss_w = onp.polynomial.legendre.leggauss(4)
_gauss_x, _gauss_w = 0.5 * (_gauss_x + 1), 0.5 * _gauss_w


@dataclass(frozen=True)
class UniformWing:
    """Uniform cantilever semispan. `inertia_torsion_kg_m` is per length about the elastic axis (includes m d^2)."""
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
    le = wing.length_m / count_elements
    h = (1 - 3 * xi**2 + 2 * xi**3, le * (xi - 2 * xi**2 + xi**3), 3 * xi**2 - 2 * xi**3, le * (-xi**2 + xi**3))
    dh = ((-6 * xi + 6 * xi**2) / le, 1 - 4 * xi + 3 * xi**2, (6 * xi - 6 * xi**2) / le, -2 * xi + 3 * xi**2)
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
    le = wing.length_m / count_elements
    d2 = ((-6 + 12 * xi) / le**2, (-4 + 6 * xi) / le, (6 - 12 * xi) / le**2, (-2 + 6 * xi) / le)
    w, v, dtheta = onp.zeros(size), onp.zeros(size), onp.zeros(size)
    for side, node in ((0, element), (1, element + 1)):
        if node == 0:
            continue
        base = (node - 1) * per_node
        w[base] += d2[2 * side]; w[base + 1] += d2[2 * side + 1]
        dtheta[base + 2] += (-1 if side == 0 else 1) / le
        if per_node == 5:
            v[base + 3] += d2[2 * side]; v[base + 4] += d2[2 * side + 1]
    return w, v, dtheta


def tip_rows(wing, count_elements):
    """Rows of the tip: w, w' (roll), theta (pitch), v, v' (minus yaw)."""
    return _element_rows(wing, count_elements, count_elements - 1, 1.0)


def structural_matrices(wing, tip=None, count_elements=20):
    """(M, K) of the cantilever wing and the tip body."""
    _, size = _layout(wing, count_elements)
    mass, stiffness = onp.zeros((size, size)), onp.zeros((size, size))
    le = wing.length_m / count_elements
    for e in range(count_elements):
        for xi, weight in zip(_gauss_x, _gauss_w):
            r = _element_rows(wing, count_elements, e, xi)
            d = wing.offset_cg_m
            wl = weight * le
            mass += wl * (wing.mass_kg_m * onp.outer(r["w"] - d * r["theta"], r["w"] - d * r["theta"])
                          + (wing.inertia_torsion_kg_m - wing.mass_kg_m * d**2) * onp.outer(r["theta"], r["theta"]))
            if wing.stiffness_chord_Nm2 is not None:
                mass += wl * wing.mass_kg_m * onp.outer(r["v"], r["v"])
            w2, v2, dtheta = _second_derivative_rows(wing, count_elements, e, xi)
            stiffness += wl * (wing.stiffness_beam_Nm2 * onp.outer(w2, w2)
                               + wing.stiffness_torsion_Nm2 * onp.outer(dtheta, dtheta))
            if wing.stiffness_chord_Nm2 is not None:
                stiffness += wl * wing.stiffness_chord_Nm2 * onp.outer(v2, v2)
    if tip is not None:
        r = tip_rows(wing, count_elements)
        d = tip.offset_cg_m
        mass += tip.mass_kg * onp.outer(r["w"] - d * r["theta"], r["w"] - d * r["theta"])
        mass += tip.inertia_pitch_kg_m2 * onp.outer(r["theta"], r["theta"])
        mass += tip.inertia_roll_kg_m2 * onp.outer(r["dw"], r["dw"])
        if wing.stiffness_chord_Nm2 is not None:
            mass += tip.mass_kg * onp.outer(r["v"], r["v"]) + tip.inertia_yaw_kg_m2 * onp.outer(r["dv"], r["dv"])
    return mass, stiffness


def aerodynamic_matrices(wing, count_elements, velocity_m_s, density_kg_m3, factor_lift_slope=1.0):
    """Theodorsen strip theory with Jones' lags: (M_a, C_a, K_a, D_a, E_q, E_qdot, F) with
    M_a q'' + C_a q' + K_a q + D_a z = -Q_aero (moved to the left) and z' = E_q q + E_qdot q' + F z."""
    _, size = _layout(wing, count_elements)
    b, a, rho, V = wing.semichord_m, wing.a, density_kg_m3, velocity_m_s
    le = wing.length_m / count_elements
    points = [(e, xi, w) for e in range(count_elements) for xi, w in zip(_gauss_x, _gauss_w)]
    lags = len(jones_lag_terms) * len(points)
    M_a, C_a, K_a = onp.zeros((size, size)), onp.zeros((size, size)), onp.zeros((size, size))
    D_a, E_q, E_qdot, F = onp.zeros((size, lags)), onp.zeros((lags, size)), onp.zeros((lags, size)), onp.zeros(lags)
    f = 2 * onp.pi * rho * V * b * factor_lift_slope
    for p, (e, xi, weight) in enumerate(points):
        r = _element_rows(wing, count_elements, e, xi)
        rw, rt, wl = r["w"], r["theta"], weight * le
        nc = onp.pi * rho * b**2 * wl
        M_a += nc * (onp.outer(rw, rw) - b * a * (onp.outer(rw, rt) + onp.outer(rt, rw))
                     + b**2 * (1 / 8 + a**2) * onp.outer(rt, rt))
        C_a += nc * (V * onp.outer(rw, rt) + V * b * (0.5 - a) * onp.outer(rt, rt))
        g = -rw + b * (a + 0.5) * rt                                  # generalized force per unit lift
        downwash_rate = rw + b * (0.5 - a) * rt                       # 3/4-chord downwash: rate part
        C_a += -f * wl * 0.5 * onp.outer(g, downwash_rate)
        K_a += -f * wl * 0.5 * V * onp.outer(g, rt)
        for i, (A, beta) in enumerate(jones_lag_terms):
            k = len(jones_lag_terms) * p + i
            D_a[:, k] = -f * wl * A * beta * V / b * g
            E_q[k], E_qdot[k], F[k] = V * rt, downwash_rate, -beta * V / b
    return M_a, C_a, K_a, D_a, E_q, E_qdot, onp.diag(F)


def flutter_eigenvalues(wing, tip, velocity_m_s, density_kg_m3, count_elements=20, factor_lift_slope=1.0,
                        damping_structural=0.0, basis=None):
    """Eigenvalues (one of each conjugate pair, plus real ones) of the aeroelastic system and each mode's label.

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
    from dataclasses import replace
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
    (divergence): (speed m/s, frequency rad/s, label) or (None, None, None)."""
    lag_floor = 1e-6

    def critical(v):
        worst = None
        for value, label in flutter_eigenvalues(wing, tip, v, density_kg_m3, count_elements, factor_lift_slope,
                                                damping_structural, basis):
            if value.real > lag_floor * max(abs(value), 1.0):
                kind = label if abs(value.imag) > frequency_min_rad_s else "divergence"
                if worst is None or value.real > worst[0].real:
                    worst = (value, kind)
        return worst

    previous = None
    for v in velocities_m_s:
        hit = critical(v)
        if hit is not None:
            if previous is None:
                return float(v), abs(hit[0].imag), hit[1]
            low, high = previous, float(v)
            while high - low > tolerance_m_s:
                mid = 0.5 * (low + high)
                if critical(mid) is not None:
                    high = mid
                else:
                    low = mid
            value, kind = critical(high)
            return high, abs(value.imag), kind
        previous = float(v)
    return None, None, None


def lift_slope_factor_helmbold(aspect_ratio):
    """Finite-span lift slope over 2 pi (Helmbold), for a Yates-type strip correction."""
    return aspect_ratio / (2 + onp.sqrt(aspect_ratio**2 + 4))


def goland_wing(**changes):
    """Goland (1945) wing as tabulated by Cal (City University, 1992, table 2.4): 20 ft semispan, 6 ft chord,
    elastic axis 33 % chord, CG 0.183 m behind it."""
    data = dict(length_m=6.096, chord_m=1.8288, stiffness_beam_Nm2=9.773e6, stiffness_torsion_Nm2=9.876e5,
                mass_kg_m=35.717, inertia_torsion_kg_m=8.642, fraction_elastic_axis=0.33, offset_cg_m=-0.183)
    data.update(changes)
    return UniformWing(**data)
