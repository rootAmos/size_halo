"""Read and write the ledger as JSON (kept in git, so its history is the decision history), and the view the
dashboard reads."""
import json
from dataclasses import asdict
from datetime import date

from aircraft_closure.ledger.model import Activity, Evidence, Ledger, Limit, Quantity, Trade, TradeOption
from aircraft_closure.ledger.rank import portfolio, priorities, z_99


def to_dict(ledger):
    return asdict(ledger)


def from_dict(data):
    quantities = {}
    for key, q in data.get("quantities", {}).items():
        q = dict(q, evidence=[Evidence(**e) for e in q.get("evidence", [])],
                 plan=[Activity(**a) for a in q.get("plan", [])], sizing_keys=tuple(q.get("sizing_keys", ())))
        quantities[key] = Quantity(**q)
    trades = {key: Trade(**dict(t, options=[TradeOption(**o) for o in t.get("options", [])]))
              for key, t in data.get("trades", {}).items()}
    limits = {key: Limit(**l) for key, l in data.get("limits", {}).items()}
    rest = {k: v for k, v in data.items() if k not in ("quantities", "trades", "limits")}
    return Ledger(**rest, quantities=quantities, limits=limits, trades=trades)


def load(path):
    with open(path, encoding="utf-8") as file:
        return from_dict(json.load(file))


def save(ledger, path):
    with open(path, "w", encoding="utf-8") as file:
        json.dump(to_dict(ledger), file, indent=1, ensure_ascii=False)
        file.write("\n")


def snapshot(ledger, source, today=None, top=15):
    """Append the current ranking to the history (one entry per ingest), for trends and burn-down."""
    today = today or date.today()
    p = portfolio(ledger)
    ranked = priorities(ledger, today)
    ledger.history.append(dict(
        date=today.isoformat(), source=source, sigma_takeoff_kg=round(p["sigma_takeoff_kg"], 2),
        margin_99_kg=round(p["margin_99_kg"], 2),
        top=[dict(key=r.key, value_kg=None if r.value_kg is None else round(r.value_kg, 2)) for r in ranked[:top]]))


def view(ledger, today=None):
    """Everything the dashboard shows, computed here so the page only draws it."""
    today = today or date.today()
    p = portfolio(ledger)
    quantities = []
    for q in ledger.quantities.values():
        mean, sigma = q.belief()
        activity = q.next_activity()
        quantities.append(dict(
            key=q.key, label=q.label, kind=q.kind, unit=q.unit, owner=q.owner, mean=mean, sigma=sigma,
            fidelity=q.fidelity(), sensitivity=q.sensitivity, allowance=q.allowance, lock_date=q.lock_date,
            next_activity=None if activity is None else asdict(activity),
            evidence=[asdict(e) for e in sorted(q.evidence, key=lambda e: e.date)], note=q.note))
    return dict(
        name=ledger.name, as_of=today.isoformat(), growth_factor=ledger.growth_factor, baseline=ledger.baseline,
        gates=ledger.gates, portfolio=p, z_99=z_99,
        priorities=[asdict(r) for r in priorities(ledger, today)], quantities=quantities,
        runs=ledger.runs[-30:], history=ledger.history,
        trades=[dict(key=t.key, label=t.label, owner=t.owner, category=t.category, discipline=t.discipline, lock_gate=t.lock_gate,
                     lock_date=t.lock_date, status=t.status, decision=t.decision, decided_date=t.decided_date,
                     model_hook=t.model_hook, affects=t.affects, options=len(t.options), note=t.note,
                     precedent=t.precedent, sources=t.sources)
                for t in ledger.trades.values()])
