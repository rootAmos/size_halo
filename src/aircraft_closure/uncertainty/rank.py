"""Priority from impact x uncertainty (plan 059).

For output i, the first-order spread owed to input j is |dy_i/dx_j| sigma_j; with independent inputs the output
variance is the sum of their squares, and the variance share S_ij = (dy_i/dx_j sigma_j)^2 / sum_k (...)^2 says
which inputs own the spread of the output (the shares sum to 1). Design inputs (levers) are ranked by elasticity
alone. Inputs without a spread or without a derivative are blind spots: listed, never ranked as zero.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Driver:
    input_key: str
    output_key: str
    elasticity: float            # % output per % input (NaN when undefined)
    spread: float                # one-sigma output change from this input, output units (NaN for levers)
    share: float                 # variance share of the output (NaN for levers)


def output_spreads(jacobian, sigmas):
    """Per-input one-sigma output changes |J_ij| sigma_j (NaN where sigma is None) and the total per output."""
    j = np.asarray(jacobian, dtype=float)
    s = np.array([np.nan if v is None else float(v) for v in sigmas])
    contribution = np.abs(j) * s[None, :]
    total = np.sqrt(np.nansum(contribution ** 2, axis=1))
    return contribution, total


def variance_shares(jacobian, sigmas):
    contribution, total = output_spreads(jacobian, sigmas)
    with np.errstate(divide="ignore", invalid="ignore"):
        return contribution ** 2 / (total[:, None] ** 2)


def rank_drivers(output_keys, inputs, jacobian, elasticity):
    """{output key: (risks by variance share, levers by |elasticity|, blind-spot input keys)}.

    `inputs` are `UncertainInput`s in the column order of `jacobian` and `elasticity` (outputs x inputs); a NaN
    derivative marks an input whose impact is not computed yet.
    """
    jacobian, elasticity = np.asarray(jacobian, dtype=float), np.asarray(elasticity, dtype=float)
    sigmas = [None if i.kind == "design" or not i.ranked else i.spread() for i in inputs]
    contribution, _ = output_spreads(jacobian, sigmas)
    shares = variance_shares(jacobian, sigmas)
    result = {}
    for row, output_key in enumerate(output_keys):
        risks, levers, blind = [], [], []
        for k, item in enumerate(inputs):
            if not item.ranked:
                continue
            if not math.isfinite(jacobian[row, k]):
                blind.append(item.key)
            elif item.kind == "design":
                levers.append(Driver(item.key, output_key, float(elasticity[row, k]), math.nan, math.nan))
            elif sigmas[k] is None:
                blind.append(item.key)
            else:
                risks.append(Driver(item.key, output_key, float(elasticity[row, k]), float(contribution[row, k]),
                                    float(shares[row, k])))
        risks.sort(key=lambda d: -d.share if math.isfinite(d.share) else 0.0)
        levers.sort(key=lambda d: -abs(d.elasticity) if math.isfinite(d.elasticity) else 0.0)
        result[output_key] = (risks, levers, blind)
    return result
