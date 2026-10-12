"""Derivatives of the optimal objective with respect to fixed parameters, from one solve (envelope theorem).

At a local optimum, dJ*/dp = dL/dp, the partial derivative of the Lagrangian L = f + lam' g at fixed design
variables and multipliers. CasADi moves terms that depend only on parameters into the constraint bounds
(`x >= p` becomes g = x with lbg = p), so the bound side is differentiated too: for each constraint the active
bound is the upper one when its multiplier is positive and the lower one otherwise.

The result is first order: it is exact for small changes and local to the optimum (an active set that changes
under a large step is not seen). Callers check large-step claims by re-solving.
"""
import casadi as cas
import numpy as np


def _dense(value):
    if hasattr(value, "toarray"):
        return value.toarray()
    return np.atleast_2d(np.array(value, dtype=float))


def objective_sensitivities(opti, solution, parameters):
    """dJ*/dp for each scalar in `parameters` (an Opti parameter or a list of them), as a 1-D array."""
    p = cas.vertcat(*parameters) if isinstance(parameters, (list, tuple)) else parameters
    lam = np.array(solution.value(opti.lam_g), dtype=float).ravel()
    gradient = _dense(solution.value(cas.gradient(opti.f + cas.dot(cas.DM(lam), opti.g), p))).ravel()
    if lam.size == 0:
        return gradient
    jacobian_lower = _dense(solution.value(cas.jacobian(opti.lbg, p)))
    jacobian_upper = _dense(solution.value(cas.jacobian(opti.ubg, p)))
    bound = np.where(lam[:, None] > 0, jacobian_upper, jacobian_lower)
    return gradient - lam @ bound


def _sparse(value):
    import scipy.sparse as sp
    if hasattr(value, "sparse"):
        return sp.csc_matrix(value.sparse())
    return sp.csc_matrix(_dense(value))


def active_constraints(opti, solution, tolerance=1e-8):
    """Indices of the constraint rows held at a bound at the optimum (equalities, and inequalities whose multiplier is
    above `tolerance` relative to the largest), and the bound side of each: True for the upper bound.

    Weakly active rows (at a bound with a zero multiplier) are left out: the derivative is then one-sided there.
    """
    lam = np.array(solution.value(opti.lam_g), dtype=float).ravel()
    lower = np.array(solution.value(opti.lbg), dtype=float).ravel()
    upper = np.array(solution.value(opti.ubg), dtype=float).ravel()
    equality = np.isclose(lower, upper)
    threshold = tolerance * max(1.0, np.max(np.abs(lam), initial=0.0))
    active = equality | (np.abs(lam) > threshold)
    return np.flatnonzero(active), lam > 0


def parametric_jacobian(opti, solution, outputs, parameters, tolerance=1e-8):
    """d(output*)/dp at the optimum, as an array (outputs x parameters): how each output moves when a parameter
    changes and the optimizer re-solves (post-optimal sensitivity, implicit function theorem on the KKT system).

    `outputs` is a list of scalar expressions in the Opti variables and parameters; `parameters` a list of scalar
    Opti parameters. With the active set A held, differentiating grad_x L = 0 and g_A(x, p) = bound_A(p) gives

        [ H    J_A' ] [ dx/dp   ]     [ d(grad_x L)/dp       ]
        [ J_A  0    ] [ dlam/dp ] = - [ dg_A/dp - dbound_A/dp ]

    with H the Hessian of the Lagrangian in x. First order and local, like `objective_sensitivities`: a step large
    enough to change the active set is not seen, and callers check large steps by re-solving.
    """
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla

    p = cas.vertcat(*parameters)
    y = cas.vertcat(*outputs)
    x = opti.x
    lam_values = np.array(solution.value(opti.lam_g), dtype=float).ravel()
    lagrangian = opti.f + cas.dot(cas.DM(lam_values), opti.g) if lam_values.size else opti.f
    gradient_x = cas.gradient(lagrangian, x)
    hessian = _sparse(solution.value(cas.jacobian(gradient_x, x)))
    hessian_xp = _dense(solution.value(cas.jacobian(gradient_x, p)))
    n = hessian.shape[0]

    rows, upper_side = active_constraints(opti, solution, tolerance)
    if rows.size:
        jacobian_gx = _sparse(solution.value(cas.jacobian(opti.g, x)))[rows, :]
        jacobian_gp = _dense(solution.value(cas.jacobian(opti.g, p)))[rows, :]
        bound_lower = _dense(solution.value(cas.jacobian(opti.lbg, p)))[rows, :]
        bound_upper = _dense(solution.value(cas.jacobian(opti.ubg, p)))[rows, :]
        bound = np.where(upper_side[rows][:, None], bound_upper, bound_lower)
        kkt = sp.bmat([[hessian, jacobian_gx.T], [jacobian_gx, None]], format="csc")
        right = -np.vstack([hessian_xp, jacobian_gp - bound])
    else:
        kkt, right = hessian, -hessian_xp
    try:
        step = spla.splu(kkt).solve(right)
    except RuntimeError:                         # singular KKT (dependent active constraints): least squares
        step = np.linalg.lstsq(kkt.toarray(), right, rcond=None)[0]
    dx_dp = step[:n, :]
    dy_dx = _sparse(solution.value(cas.jacobian(y, x)))
    dy_dp = _dense(solution.value(cas.jacobian(y, p)))
    return np.asarray(dy_dx @ dx_dp) + dy_dp


def elasticities(jacobian, outputs_value, parameters_value):
    """% change of each output per % change of each parameter: J_ij p_j / y_i (NaN where y_i or p_j is zero)."""
    y = np.asarray(outputs_value, dtype=float)[:, None]
    p = np.asarray(parameters_value, dtype=float)[None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        result = np.asarray(jacobian, dtype=float) * p / y
    result[~np.isfinite(result)] = np.nan
    return result
