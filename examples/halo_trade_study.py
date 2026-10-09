"""Price trades with the sizing and write each result into the master ledger, with a report tied to a commit.

    python -m examples.halo_trade_study trade.wing_material trade.hv_voltage --cache output/ledger/baseline.pkl

Each option is one coupled sizing, warm-started from the baseline at its machine unit counts. An option's effect is
its take-off mass against the trade's reference option (the first). Its uncertainty is the model error of the items
that change: sigma = growth factor x sqrt(sum (sigma fraction x item mass change)^2) over the empty-weight items,
since items estimated by the same method share their bias between options. Options that fail to solve are reported,
never dropped silently.

The run refuses to start with uncommitted changes under src/ or examples/, so the commit it records reproduces it.
"""
import argparse
import math
import os
import subprocess
from dataclasses import replace
from datetime import date

from aircraft_closure.ledger.model import TradeOption
from aircraft_closure.ledger.rank import priorities
from aircraft_closure.ledger.store import load, snapshot
from examples.halo_ledger import baseline_result, fixed_units, path_ledger, write
from examples.halo_oew_uncertainty import OewItem, item_definitions

lb = 2.20462


def items_of(result):
    """Empty-weight items by label; whatever the 16 items do not cover (the electrical layer's cables, inverters and
    protection when it is on) is one more item with a 30 % spread."""
    components, powertrain = dict(result.component_masses_kg), dict(result.powertrain_masses_kg)
    items = {}
    for label, group, keys, maturity, sigma, basis in item_definitions:
        mass = sum(powertrain.get(k[3:], 0.0) if k.startswith("pt:") else components.get(k, 0.0) for k in keys)
        items[label] = OewItem(label, group, float(mass), maturity, sigma, basis)
    rest = result.mass_empty_kg - sum(i.mass_kg for i in items.values())
    items["Electrical layer (cables, inverters, protection)"] = OewItem(
        "Electrical layer (cables, inverters, protection)", "Powertrain", max(rest, 0.0), "Estimated", 0.30,
        "remainder of the empty weight")
    return items


def options_for(key, base):
    """(label, assumptions) per option; the first is the reference."""
    from examples.halo_sizing import assumptions_for_bus_voltage
    if key == "trade.wing_material":
        return [("Graphite-epoxy (baseline)", base), ("Aluminium (the XV-15's)", replace(base, wing_material="aluminium"))]
    if key == "trade.hv_voltage":
        return [(f"{v:,.0f} V floating bus", assumptions_for_bus_voltage(base, v)) for v in (756.0, 540.0, 800.0, 1000.0)]
    if key == "trade.blade_count":
        return [("3 blades (baseline)", base), ("4 blades", replace(base, count_blades=4))]
    raise KeyError(f"No sizing hook for {key}")


def commit():
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "src", "examples"], capture_output=True,
                           text=True).stdout.strip()
    if dirty:
        raise SystemExit(f"Commit the model code first; uncommitted changes:\n{dirty}")
    return subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()


def solve(assumptions, start):
    from examples.halo_sizing import solve_halo_sizing
    try:
        return solve_halo_sizing(assumptions=assumptions, initial=start)
    except RuntimeError:
        return None


def study(key, baseline):
    from examples.halo_sizing import HaloAssumptions
    base = fixed_units(HaloAssumptions(), baseline)
    rows, previous = [], None
    for label, assumptions in options_for(key, base):
        result = solve(assumptions, baseline) or (previous and solve(assumptions, previous))
        previous = result or previous
        rows.append((label, result))
    return rows


def priced_options(rows, growth_factor):
    reference = rows[0][1]
    if reference is None:
        raise RuntimeError("The reference option did not solve")
    items_ref = items_of(reference)
    options = []
    for label, result in rows:
        if result is None:
            continue
        items = items_of(result)
        sigma = growth_factor * math.sqrt(sum((items_ref[k].sigma_fraction * (items[k].mass_kg - items_ref[k].mass_kg))
                                              ** 2 for k in items))
        options.append((label, result, result.mass_takeoff_kg - reference.mass_takeoff_kg, sigma, items, items_ref))
    return options


def report(key, trade, rows, options, baseline, sha, today, ranked):
    sha_short = sha[:7]
    lines = [f"# Trade study: {trade.label}", "",
             f"Run {today.isoformat()} at commit [`{sha_short}`](https://github.com/rootAmos/size_halo/commit/{sha}). "
             f"Ledger key `{key}`, owner {trade.owner}, locks at {trade.lock_gate} ({trade.lock_date}).", ""]
    entry = next(p for p in ranked if p.key == key)
    best = min(options[1:], key=lambda o: o[2]) if len(options) > 1 else None
    if best is not None:
        verdict = ("the data has decided it" if "decided" in entry.action else "still open")
        lines += [f"**Result.** {best[0]} changes take-off mass by {best[2] * lb:+,.0f} lb "
                  f"(±{best[3] * lb:,.0f} lb, 1σ) against {options[0][0]}. {entry.detail}; {verdict}. "
                  f"It ranks #{[p.key for p in ranked].index(key) + 1} of {len(ranked)} in the ledger.", ""]
    lines += ["| Option | Take-off (lb) | Change (lb) | 1σ (lb) | Empty (lb) | Battery (lb) | Binding limits |",
              "|---|---|---|---|---|---|---|"]
    for label, result in rows:
        if result is None:
            lines.append(f"| {label} | did not solve | | | | | |")
            continue
        o = next(o for o in options if o[0] == label)
        battery = o[4]["Battery"].mass_kg
        lines.append(f"| {label} | {result.mass_takeoff_kg * lb:,.0f} | {o[2] * lb:+,.0f} | {o[3] * lb:,.0f} | "
                     f"{result.mass_empty_kg * lb:,.0f} | {battery * lb:,.0f} | {len(result.binding)} |")
    lines += ["", "**What changes, item by item** (empty weight, lb, against the reference):", "",
              "| Item | " + " | ".join(o[0] for o in options[1:]) + " |", "|---|" + "---|" * (len(options) - 1)]
    for item in options[0][4]:
        deltas = [(o[4][item].mass_kg - o[5][item].mass_kg) * lb for o in options[1:]]
        if any(abs(d) >= 1 for d in deltas):
            lines.append(f"| {item} | " + " | ".join(f"{d:+,.0f}" for d in deltas) + " |")
    lines += ["", "**Method.** Each option is one coupled sizing, warm-started from the baseline "
              f"({baseline.mass_takeoff_kg * lb:,.0f} lb) at its machine unit counts. The 1σ is the model error of the "
              "items that change (their maturity spreads, scaled by the growth factor); errors common to all options "
              "cancel.", "", "**Reproduce.**", "", "```",
              f"git checkout {sha}",
              f"python -m examples.halo_trade_study {key} --cache output/ledger/baseline.pkl", "```", ""]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m examples.halo_trade_study")
    parser.add_argument("keys", nargs="+")
    parser.add_argument("--cache", default=None)
    args = parser.parse_args(argv)
    sha, today = commit(), date.today()
    baseline = baseline_result(args.cache)
    ledger = load(path_ledger)
    os.makedirs("docs/trades", exist_ok=True)
    for key in args.keys:
        trade = ledger.trades[key]
        rows = study(key, baseline)
        options = priced_options(rows, ledger.growth_factor)
        trade.options = [TradeOption(label, "", delta, sigma, f"Halo sizing at {sha[:7]}")
                         for label, result, delta, sigma, _, _ in options]
        failed = [label for label, result in rows if result is None]
        ledger.runs.append(dict(date=today.isoformat(), source=f"trade study {key} at {sha[:7]}", kind="trade",
                                summary=f"{trade.label}: {len(options)} options priced"
                                        + (f"; did not solve: {', '.join(failed)}" if failed else "")))
        snapshot(ledger, f"trade study {key}", today)
        ranked = priorities(ledger, today)
        path = os.path.join("docs/trades", f"{key.split('.', 1)[1]}.md")
        with open(path, "w", encoding="utf-8") as file:
            file.write(report(key, trade, rows, options, baseline, sha, today, ranked))
        print(f"{key}: wrote {path}")
    write(ledger, today)


if __name__ == "__main__":
    main()
