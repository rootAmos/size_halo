"""Feed the ledger: a new sizing (sensitivities, limits, sized masses) or a new piece of evidence (an analysis,
a vendor weight, a test). Each ingest logs a run and snapshots the ranking, so the dashboard shows what moved."""
from datetime import date

from aircraft_closure.ledger.model import Evidence, Limit
from aircraft_closure.ledger.store import snapshot

price_tolerance_kg = 1e-3             # a margin whose price is below this (take-off kg per unit) is not binding


def sized_mass_kg(result, keys):
    components, powertrain = dict(result.component_masses_kg), dict(result.powertrain_masses_kg)
    return sum(powertrain.get(k[3:], 0.0) if k.startswith("pt:") else components.get(k, 0.0) for k in keys)


def joint_limits(margins):
    """Binding margins, with those of one condition that carry the same price merged (they bind together)."""
    groups = []
    for label, margin, price in margins:
        if abs(price) < price_tolerance_kg:
            continue
        condition = label.split(":")[0]
        for group in groups:
            if group[0].split(":")[0] == condition and abs(group[2] - price) <= 1e-6 * max(abs(price), 1.0):
                group.append(label)
                break
        else:
            groups.append([label, margin, price])
    return [((g[0], *g[3:]), g[1], g[2]) for g in groups]


def ingest_sizing(ledger, result, source, today=None, sigma_fraction=None, names=None, limit_owners=None):
    """Update sensitivities, limits and sized masses from a `HaloSizingResult` with `sensitivity` set.

    Mass quantities with `sizing_keys` get the sized mass as evidence from `source` (replacing that source's
    earlier entry), with sigma = `sigma_fraction[key]` x mass, or the previous entry's relative sigma.
    `names` and `limit_owners` map margin labels to a plain name and an owner for new limits.
    """
    today = today or date.today()
    s = result.sensitivity
    if s is None or s.objective != "mass_takeoff":
        raise ValueError("ingest_sizing needs a take-off-mass sizing with sensitivities")
    ledger.growth_factor = s.growth_factor
    ledger.baseline = dict(mass_takeoff_kg=result.mass_takeoff_kg, mass_empty_kg=result.mass_empty_kg,
                           mass_payload_kg=result.mass_payload_kg)
    for q in ledger.quantities.values():
        if q.kind in ("mass", "gap"):
            q.sensitivity, q.sensitivity_source = s.growth_factor, f"growth factor, {source}"
        if not q.sizing_keys:
            continue
        mass = sized_mass_kg(result, q.sizing_keys)
        previous = [e for e in q.evidence if e.source == source]
        fraction = (sigma_fraction or {}).get(q.key)
        if fraction is None:
            fraction = previous[-1].sigma / previous[-1].value if previous and previous[-1].value else 0.1
        q.evidence = [e for e in q.evidence if e.source != source]
        q.evidence.append(Evidence(today.isoformat(), mass, fraction * mass, source, 1, "sized mass"))
    seen = set()
    for labels, margin, price in joint_limits(s.margins):
        key = labels[0]
        seen.add(key)
        limit = ledger.limits.get(key) or Limit(key=key, label=(names or {}).get(key, " + ".join(labels)),
                                                owner=(limit_owners or {}).get(key, "Chief engineer"))
        limit.price, limit.margin, limit.binding = price, margin, True
        if len(labels) > 1:
            limit.note = "Binds together with " + ", ".join(labels[1:]) + "; relaxing one alone may save nothing"
        ledger.limits[key] = limit
    for label, limit in ledger.limits.items():
        if label not in seen:
            limit.binding = False
    ledger.runs.append(dict(date=today.isoformat(), source=source, kind="sizing",
                            summary=f"take-off {result.mass_takeoff_kg:,.0f} kg, growth factor "
                                    f"{s.growth_factor:.2f}, {len(seen)} binding limits"))
    snapshot(ledger, source, today)


def ingest_evidence(ledger, key, value, sigma, source, fidelity, today=None, note=""):
    """Add one result (an analysis, a vendor weight, a test) to a quantity and re-rank."""
    today = today or date.today()
    q = ledger.quantities[key]
    q.evidence = [e for e in q.evidence if e.source != source]
    q.evidence.append(Evidence(today.isoformat(), value, sigma, source, fidelity, note))
    mean, spread = q.belief()
    ledger.runs.append(dict(date=today.isoformat(), source=source, kind="evidence",
                            summary=f"{q.label}: {value:.4g} ± {sigma:.2g} {q.unit} → belief {mean:.4g} ± "
                                    f"{spread:.2g}"))
    snapshot(ledger, source, today)


def set_sensitivity(ledger, key, sensitivity, source, today=None):
    """Record a take-off sensitivity found by a re-solve (finite difference) for a model input."""
    today = today or date.today()
    q = ledger.quantities[key]
    q.sensitivity, q.sensitivity_source = sensitivity, source
    ledger.runs.append(dict(date=today.isoformat(), source=source, kind="sensitivity",
                            summary=f"{q.label}: {sensitivity:.4g} take-off kg per {q.unit}"))
    snapshot(ledger, source, today)
