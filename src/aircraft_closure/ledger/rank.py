"""Rank what the team should work on, in take-off kg, with a range and a schedule flag.

Four kinds of priority, one currency (take-off mass at the required payload; payload kg at a fixed take-off mass
is the same kg divided by the growth factor):

- **Reduce uncertainty** (masses and model inputs): the aircraft is sized to its 99 % weight, so a spread costs
  z99 x sigma of take-off mass. Narrowing one quantity frees margin:
  freed = z99 (sigma_total - sqrt(sigma_total^2 - e^2 + e_after^2)), with e = |S| sigma the quantity's take-off
  spread and e_after the spread its next planned activity should leave. The range runs from that planned value
  (halving the spread when no work is planned) to full resolution (sigma -> 0).
- **Model gap**: the same for an unmodelled item carried as a prior; its mean is weight the baseline does not
  book, shown as exposure (take-off kg) beside the saving.
- **Relax a limit**: price x plausible relaxation (low to high).
- **Decide a trade**: the best option's expected change against the baseline option, with a 90 % range from the
  options' spreads and the probability that it is really better. A probability above 99 % means the data has
  decided it.

Anything without a sensitivity, or a trade without option data, is a **blind spot**: listed, never valued at zero.
Schedule: a priority whose lock date passes before its planned work lands, or that has no plan, is flagged.
"""
import math
from dataclasses import dataclass
from datetime import date
from typing import Optional

from aircraft_closure.ledger.model import fidelity_tiers

z_90, z_99 = 1.6449, 2.3263


@dataclass(frozen=True)
class Priority:
    key: str
    label: str
    kind: str                         # "reduce uncertainty", "model gap", "relax limit", "decide trade", "blind spot"
    owner: str
    value_kg: Optional[float]         # expected take-off saving from acting (None: unknown)
    value_low_kg: Optional[float]
    value_high_kg: Optional[float]
    share: Optional[float]            # share of the take-off variance (uncertainty kinds)
    exposure_kg: Optional[float]      # take-off kg at risk: unbooked mean (gaps) plus a 2-sigma overrun
    lock_date: Optional[str]
    days_to_lock: Optional[int]
    schedule: str                     # "", "no plan", "plan lands after lock", "past lock"
    action: str                       # the next planned activity or what is missing
    detail: str
    category: str = ""                # decision type
    phase: str = ""                   # programme phase in which it locks


def _days(iso, today):
    return None if not iso else (date.fromisoformat(iso) - today).days


def _schedule(lock_date, activity, today):
    if lock_date and date.fromisoformat(lock_date) < today:
        return "past lock"
    if activity is None:
        return "no plan"
    if lock_date and activity.due > lock_date:
        return "plan lands after lock"
    return ""


def spread_kg(quantity):
    """Take-off mass spread (one sigma, kg) of one quantity, or None without a sensitivity."""
    if quantity.sensitivity is None:
        return None
    mean, sigma = quantity.belief()
    return abs(quantity.sensitivity) * sigma if math.isfinite(sigma) else None


def portfolio(ledger):
    """Take-off mass spread (one sigma, kg) over all valued quantities, and the summed allowance (empty kg)."""
    spreads = [spread_kg(q) for q in ledger.quantities.values()]
    sigma_total = math.sqrt(sum(e ** 2 for e in spreads if e is not None))
    allowance = sum(q.allowance for q in ledger.quantities.values() if q.kind in ("mass", "gap"))
    gaps = sum(q.belief()[0] for q in ledger.quantities.values() if q.kind == "gap")
    takeoff = ledger.baseline.get("mass_takeoff_kg")
    predicted = None if takeoff is None or ledger.growth_factor is None else (
        takeoff + ledger.growth_factor * (allowance + gaps))
    return dict(sigma_takeoff_kg=sigma_total, margin_99_kg=z_99 * sigma_total, allowance_empty_kg=allowance,
                gaps_empty_kg=gaps, takeoff_predicted_kg=predicted,
                takeoff_99_kg=None if predicted is None else predicted + z_99 * sigma_total)


def _freed(sigma_total, spread, spread_after):
    rest = max(sigma_total ** 2 - spread ** 2 + spread_after ** 2, 0.0)
    return z_99 * (sigma_total - math.sqrt(rest))


def priorities(ledger, today=None):
    today = today or date.today()
    sigma_total = portfolio(ledger)["sigma_takeoff_kg"]
    out = []
    for q in ledger.quantities.values():
        activity = q.next_activity()
        schedule = _schedule(q.lock_date, activity, today)
        mean, sigma = q.belief()
        if q.sensitivity is None:
            out.append(Priority(q.key, q.label, "blind spot", q.owner, None, None, None, None, None, q.lock_date,
                                _days(q.lock_date, today), schedule, "compute its sensitivity",
                                f"{mean:.4g} ± {sigma:.2g} {q.unit}; no take-off sensitivity yet",
                                q.category, q.lock_gate))
            continue
        spread = abs(q.sensitivity) * sigma
        after = abs(q.sensitivity) * (activity.sigma_after if activity else 0.5 * sigma)
        planned, full = _freed(sigma_total, spread, after), _freed(sigma_total, spread, 0.0)
        unbooked = abs(q.sensitivity) * mean if q.kind == "gap" else 0.0
        out.append(Priority(
            q.key, q.label, "model gap" if q.kind == "gap" else "reduce uncertainty", q.owner,
            planned, planned, full, spread ** 2 / sigma_total ** 2 if sigma_total > 0 else 0.0,
            unbooked + 2 * spread, q.lock_date, _days(q.lock_date, today),
            schedule, activity.label if activity else "plan the work that narrows it",
            f"{mean:.4g} ± {sigma:.2g} {q.unit} ({fidelity_tiers[q.fidelity()]}); take-off spread ±{spread:.0f} kg",
            q.category, q.lock_gate))
    for limit in ledger.limits.values():
        if not limit.binding:
            continue
        price = abs(limit.price)
        out.append(Priority(
            limit.key, limit.label, "relax limit", limit.owner, price * limit.relaxation_low,
            price * limit.relaxation_low, price * limit.relaxation_high, None, None, limit.lock_date,
            _days(limit.lock_date, today), "", "argue the requirement or change the design that meets it",
            f"{price / 100:.1f} kg take-off per 1 % of the limit", limit.category, limit.lock_gate))
    for trade in ledger.trades.values():
        if trade.status != "open":
            continue
        out.append(_trade_priority(ledger, trade, today))
    return sorted(out, key=lambda p: (p.value_kg is None, -(p.value_kg or 0.0)))


def _trade_priority(ledger, trade, today):
    def impact(option):
        if not option.quantity:
            return option.delta_mean, option.delta_sigma
        q = ledger.quantities.get(option.quantity)
        if q is None or q.sensitivity is None:
            return None
        return q.sensitivity * option.delta_mean, abs(q.sensitivity) * option.delta_sigma

    impacts = [impact(o) for o in trade.options]
    days = _days(trade.lock_date, today)
    schedule = "past lock" if days is not None and days < 0 else ""
    if len(trade.options) < 2 or any(i is None for i in impacts):
        return Priority(trade.key, trade.label, "blind spot", trade.owner, None, None, None, None, None,
                        trade.lock_date,
                        days, schedule,
                        f"price it with the sizing: {trade.model_hook}" if trade.model_hook
                        else "owner to estimate each option against a ledger quantity",
                        f"{trade.category or 'Trade'} without option data"
                        + (f"; moves {', '.join(trade.affects)}" if trade.affects else ""),
                        trade.category, trade.lock_gate)
    (base_mean, base_sigma), best = impacts[0], min(range(1, len(impacts)), key=lambda i: impacts[i][0])
    gain = base_mean - impacts[best][0]
    sigma = math.hypot(base_sigma, impacts[best][1])
    probability = 0.5 * (1 + math.erf(gain / (sigma * math.sqrt(2)))) if sigma > 0 else float(gain > 0)
    ready = "the data has decided it" if probability > 0.99 or probability < 0.01 else "still open"
    return Priority(
        trade.key, trade.label, "decide trade", trade.owner, max(gain, 0.0), gain - z_90 * sigma,
        gain + z_90 * sigma, None, z_90 * sigma, trade.lock_date, days, schedule,
        f"best option: {trade.options[best].label} ({ready})",
        f"{trade.category + ': ' if trade.category else ''}P(better than {trade.options[0].label}) = "
        f"{100 * probability:.0f} %", trade.category, trade.lock_gate)
