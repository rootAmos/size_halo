"""The decision ledger: every uncertain quantity, binding limit and open trade, with its evidence and its plan.

Units: masses in kg, take-off mass effects in kg. A quantity's `sensitivity` is the take-off mass change per unit
of the quantity (kg per kg for masses, the growth factor), from the sizing (envelope theorem) or a re-solve.

Evidence rule: a re-run of the same source replaces its earlier entry (re-running a handbook model is not new
information); independent sources at the highest fidelity tier present are fused by inverse variance; lower tiers
are then history. Fidelity tiers are listed in `fidelity_tiers`.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

fidelity_tiers = {
    0: "placeholder prior",
    1: "handbook or parametric model",
    2: "analysis (layout, FE, CFD)",
    3: "vendor data or component test",
    4: "weighed or flight-measured hardware",
}


@dataclass
class Evidence:
    date: str                         # ISO date
    value: float
    sigma: float                      # one sigma, same units as value
    source: str                       # what produced it: "sizing v3.6", "CalculiX wing box", "vendor quote"
    fidelity: int = 1                 # key of `fidelity_tiers`
    note: str = ""


@dataclass
class Activity:
    """Planned burn-down: the work that should cut a quantity's uncertainty, and by when."""
    label: str
    due: str                          # ISO date
    sigma_after: float                # expected one sigma once done
    owner: str = ""
    done: bool = False


@dataclass
class Quantity:
    key: str
    label: str
    kind: str                         # "mass" (an empty-weight item), "gap" (unmodelled item), "input" (model input)
    unit: str
    owner: str
    lock_gate: str = ""
    lock_date: Optional[str] = None
    evidence: list = field(default_factory=list)
    sensitivity: Optional[float] = None   # take-off kg per unit; None = not yet computed (a blind spot)
    sensitivity_source: str = ""
    allowance: float = 0.0            # expected growth (maturity), same units; shifts the prediction, not the spread
    plan: list = field(default_factory=list)
    sizing_keys: tuple = ()           # result keys that sum to this item ("pt:" = powertrain instance)
    note: str = ""
    category: str = "Increase maturity"  # decision type: how far to firm up the estimate before committing

    def belief(self):
        """(mean, sigma) from the evidence rule above; (nan, nan) with no evidence."""
        if not self.evidence:
            return float("nan"), float("nan")
        latest = {}
        for e in sorted(self.evidence, key=lambda e: e.date):
            latest[e.source] = e
        top = max(e.fidelity for e in latest.values())
        fused = [e for e in latest.values() if e.fidelity == top]
        weights = np.array([1 / max(e.sigma, 1e-12) ** 2 for e in fused])
        mean = float(np.dot(weights, [e.value for e in fused]) / weights.sum())
        return mean, float(1 / np.sqrt(weights.sum()))

    def fidelity(self):
        return max((e.fidelity for e in self.evidence), default=0)

    def next_activity(self):
        pending = [a for a in self.plan if not a.done]
        return min(pending, key=lambda a: a.due) if pending else None


@dataclass
class Limit:
    """A binding margin of the sizing, priced in take-off kg per unit relaxation (0.01 = 1 % of the limit)."""
    key: str                          # the margin label in the sizing
    label: str
    owner: str = "Chief engineer"
    price: float = 0.0
    margin: float = 0.0
    relaxation_low: float = 0.02      # plausible relaxation range, fraction of the limit (assumed until set)
    relaxation_high: float = 0.05
    lock_date: Optional[str] = None
    binding: bool = True
    note: str = ""
    category: str = "Requirement"
    lock_gate: str = ""
    action: str = ""                  # what to do about it (default: argue the limit or change the design)


@dataclass
class TradeOption:
    label: str
    quantity: str                     # key of the quantity the option changes ("" = take-off kg directly)
    delta_mean: float                 # change in that quantity's units against the baseline option
    delta_sigma: float
    source: str = ""


@dataclass
class Trade:
    key: str
    label: str
    owner: str
    options: list = field(default_factory=list)   # the first option is the baseline (delta 0)
    lock_gate: str = ""
    lock_date: Optional[str] = None
    status: str = "open"              # "open", "decided" or "retired" (below the line; kept for the record)
    decision: str = ""
    note: str = ""
    discipline: str = ""
    affects: list = field(default_factory=list)   # ledger keys (quantities, limits) the choice moves
    model_hook: str = ""              # how the sizing can price the options ("" = owner estimate needed)
    decided_date: str = ""            # ISO date of the decision, when decided
    category: str = ""                # decision type: requirement, architecture, technology and material, margin
                                      # policy, increase maturity or verification
    precedent: str = ""               # what comparable programmes chose and what it cost them
    block: str = ""                   # "Block 0" (gates the first flight article) or "Block 1" (final design)
    block0_choice: str = ""           # the simpler choice flown first
    sources: list = field(default_factory=list)   # [label, url] pairs behind the precedent


@dataclass
class Ledger:
    name: str
    growth_factor: Optional[float] = None         # take-off kg per kg of empty mass, from the latest sizing
    baseline: dict = field(default_factory=dict)  # latest sizing: take-off, empty, payload (kg)
    gates: dict = field(default_factory=dict)     # gate name -> ISO date
    quantities: dict = field(default_factory=dict)
    limits: dict = field(default_factory=dict)
    trades: dict = field(default_factory=dict)
    runs: list = field(default_factory=list)      # every ingest: date, source, summary
    history: list = field(default_factory=list)   # ranking snapshots, for trends and burn-down
    schedule: list = field(default_factory=list)  # plan tasks: lane, task, start, end (start == end: milestone)
