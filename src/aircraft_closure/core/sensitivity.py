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
