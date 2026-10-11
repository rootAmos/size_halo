"""Inputs with their certainty, and the outputs they drive (plan 042).

An input is anything a solve reads: requirement, assumption, technology level, calibration factor, design choice.
Values are in SI, as the dataclass that holds them. `sigma` is one standard deviation in the same units; a range
`low`/`high` is read as a uniform distribution (sigma = (high - low) / sqrt(12)). An input with no spread set is
**unquantified**: listed as a blind spot, never treated as certain.

Kinds:
- "design": we choose it. Its value is not uncertain; it is ranked by impact alone, as a lever.
- "technology", "model", "requirement", "operation": uncertain; ranked by impact x uncertainty, as a risk.
- "calibration": a factor that tunes a model to reference aircraft (wing, fuselage weight, excrescence drag). Always
  a large lever because it stands in for low design confidence, so it is recorded, not ranked.
- "setting": a modelling switch or count (flags, solver options); recorded, not ranked.
"""
import math
from dataclasses import dataclass
from typing import Optional

kinds = ("design", "technology", "model", "requirement", "operation", "calibration", "setting")


@dataclass
class UncertainInput:
    key: str                          # "assumptions.solidity", "calibration.wing_tiltrotor"
    value: object                     # the value the solve used (float, int, bool or str)
    label: str = ""                   # plain name for reports and the dashboard
    kind: str = "setting"
    sigma: Optional[float] = None     # one sigma, units of `value`
    low: Optional[float] = None
    high: Optional[float] = None
    unit: str = ""
    source: str = ""                  # where the value and its spread come from
    fidelity: int = 0                 # ledger fidelity tier (0 = placeholder prior)
    owner: str = ""
    lock_date: Optional[str] = None   # ISO date by which it must be frozen
    note: str = ""

    def __post_init__(self):
        if self.kind not in kinds:
            raise ValueError(f"{self.key}: kind '{self.kind}' not in {kinds}")

    @property
    def numeric(self):
        return isinstance(self.value, (int, float)) and not isinstance(self.value, bool)

    def spread(self):
        """One sigma, from `sigma` or the `low`/`high` range; None when unquantified."""
        if self.sigma is not None:
            return float(self.sigma)
        if self.low is not None and self.high is not None:
            return (float(self.high) - float(self.low)) / math.sqrt(12.0)
        return None

    @property
    def ranked(self):
        """Lever or risk: numeric, and neither a setting nor a calibration factor."""
        return self.numeric and self.kind not in ("setting", "calibration")


@dataclass
class Output:
    key: str                          # "range_m", "mass_takeoff_kg", "margin.whirl_flutter"
    label: str
    unit: str = ""
    limit: Optional[float] = None     # requirement or allowable, when the output has one
    above: bool = True                # True: the output must stay above `limit`
