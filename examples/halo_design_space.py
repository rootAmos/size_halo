"""Tier 22 (plan 029): design-space practice around the Halo sizing solve.

* `enumerate_architectures`: a full-factorial enumeration over discrete choices that already exist as
  `HaloAssumptions` flags. Each combination is a separate coupled solve through the starting-point strategy;
  a discrete trade, not a loop.
* `sensitivity_study`: one-at-a-time sensitivity of a sized design to uncertain inputs (assumption fields or
  XV-15 calibration factors) scaled by 1 +/- a fraction, as tornado data. Each case runs the starting-point
  strategy with the base design as its first start.

Both take the aerodynamics model from the assumptions they are given; with AeroBuildup a solve takes
2-3 minutes, so the notebook runs the larger sweeps with `aerodynamics_model="scholz"` (about 1 % in mass from
the build-up, plan 025) and says so.
"""
import itertools
import time
from dataclasses import dataclass, replace
from typing import Any

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, objective_value, solve_halo_sizing,
                                  solve_halo_sizing_multistart)
from examples.xv15_reference import calibration_factors


@dataclass(frozen=True)
class ArchitectureCase:
    """One enumerated combination: `settings` as ((field, value), ...); `result` None when no start converged."""
    settings: tuple
    result: Any
    time_s: float
    message: str = ""

    @property
    def converged(self):
        return self.result is not None


def enumerate_architectures(options, requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                            objective="mass_takeoff", max_iter=3000):
    """Solve every combination of `options` ({HaloAssumptions field: (values, ...)}) on top of `assumptions`.

    Each case uses `solve_halo_sizing_multistart` with one shared precursor cache; failures are recorded, not
    raised. Returns a tuple of `ArchitectureCase` in itertools.product order.
    """
    names = tuple(options)
    cache, cases = {}, []
    for values in itertools.product(*(options[name] for name in names)):
        settings = tuple(zip(names, values))
        time_start_s = time.perf_counter()
        try:
            result = solve_halo_sizing_multistart(requirements, replace(assumptions, **dict(settings)), factors,
                                                  max_iter=max_iter, objective=objective, cache=cache)
            cases.append(ArchitectureCase(settings, result, time.perf_counter() - time_start_s))
        except RuntimeError as error:
            cases.append(ArchitectureCase(settings, None, time.perf_counter() - time_start_s, str(error)[:300]))
    return tuple(cases)


@dataclass(frozen=True)
class UncertainInput:
    """An uncertain input: a `HaloAssumptions` field (`target` "assumptions") or an `Xv15MassFactors` field
    (`target` "factors")."""
    label: str
    target: str
    name: str


default_uncertain_inputs = (
    UncertainInput("fittings drag area", "assumptions", "drag_area_misc_buildup_m2"),
    UncertainInput("cell power-density factor", "assumptions", "factor_power_density_battery"),
    UncertainInput("battery capacity at end of life", "assumptions", "factor_capacity_ageing_battery"),
    UncertainInput("battery resistance at end of life", "assumptions", "factor_resistance_ageing_battery"),
    UncertainInput("machine torque density", "assumptions", "torque_density_Nm_kg"),
    UncertainInput("fixed equipment mass", "assumptions", "mass_equipment_kg"),
    UncertainInput("wing calibration factor", "factors", "wing_tiltrotor"),
    UncertainInput("rotor calibration factor", "factors", "rotor"),
    UncertainInput("fuselage calibration factor", "factors", "fuselage"),
    UncertainInput("powerplant calibration factor", "factors", "powerplant"),
)


@dataclass(frozen=True)
class SensitivityCase:
    """One perturbed solve: the input scaled by `scale`; `result` None when it did not converge."""
    label: str
    scale: float
    result: Any
    start: str = ""


@dataclass(frozen=True)
class SensitivityStudy:
    base: Any
    cases: tuple

    def tornado(self, objective="mass_takeoff"):
        """Rows (label, delta at the low scale, delta at the high scale) of the objective value against the base,
        sorted by the largest absolute delta (None where a case failed)."""
        base_value = objective_value(self.base, objective)
        rows = {}
        for case in self.cases:
            delta = (objective_value(case.result, objective) - base_value) if case.result is not None else None
            low, high = rows.get(case.label, (None, None))
            rows[case.label] = (delta, high) if case.scale < 1 else (low, delta)
        return tuple(sorted(((label, low, high) for label, (low, high) in rows.items()),
                            key=lambda row: -max(abs(row[1] or 0.0), abs(row[2] or 0.0))))


def sensitivity_study(requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                      inputs=default_uncertain_inputs, fraction=0.15, base=None, objective="mass_takeoff",
                      max_iter=3000):
    """Scale each input by 1 - fraction and 1 + fraction, one at a time, and re-size.

    `base`: the sized design at the nominal inputs (solved here when None). Each case runs the starting-point
    strategy with the base design as the first ("caller") start.
    """
    factors = factors if factors is not None else calibration_factors()
    if base is None:
        base = solve_halo_sizing(requirements, assumptions, factors, max_iter=max_iter, objective=objective)
    cases = []
    for item in inputs:
        for scale in (1 - fraction, 1 + fraction):
            a, f = assumptions, factors
            if item.target == "assumptions":
                a = replace(assumptions, **{item.name: getattr(assumptions, item.name) * scale})
            elif item.target == "factors":
                f = replace(factors, **{item.name: getattr(factors, item.name) * scale})
            else:
                raise ValueError(f"Unknown target '{item.target}'.")
            try:
                result = solve_halo_sizing_multistart(requirements, a, f, max_iter=max_iter, objective=objective,
                                                      initial=base)
                start = result.start.label
            except RuntimeError:
                result, start = None, "none converged"
            cases.append(SensitivityCase(item.label, scale, result, start))
    return SensitivityStudy(base, tuple(cases))
