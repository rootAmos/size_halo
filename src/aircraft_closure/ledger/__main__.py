"""Command line for the decision ledger.

    python -m aircraft_closure.ledger rank ledger/halo.json
    python -m aircraft_closure.ledger evidence ledger/halo.json mass.wing 452 25 --source "CalculiX wing box" --fidelity 2
    python -m aircraft_closure.ledger view ledger/halo.json ledger/view.json

`evidence` also rewrites view.json beside the ledger, the file the dashboard reads.
"""
import argparse
import json
import os

from aircraft_closure.ledger.ingest import ingest_evidence
from aircraft_closure.ledger.rank import portfolio, priorities
from aircraft_closure.ledger.store import load, save, view

lb_per_kg = 2.20462


def print_ranking(ledger, top):
    p = portfolio(ledger)
    print(f"{ledger.name}: take-off spread ±{p['sigma_takeoff_kg'] * lb_per_kg:,.0f} lb (1σ), "
          f"99 % margin {p['margin_99_kg'] * lb_per_kg:,.0f} lb")
    for i, r in enumerate(priorities(ledger)[:top], 1):
        value = "unknown" if r.value_kg is None else (f"{r.value_kg * lb_per_kg:,.0f} lb "
                                                      f"({r.value_low_kg * lb_per_kg:,.0f} to "
                                                      f"{r.value_high_kg * lb_per_kg:,.0f})")
        flag = f"  [{r.schedule}]" if r.schedule else ""
        print(f"{i:3d}. {r.label} | {r.kind} | {value} | {r.owner}{flag}\n      next: {r.action}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m aircraft_closure.ledger")
    sub = parser.add_subparsers(dest="command", required=True)
    rank = sub.add_parser("rank")
    rank.add_argument("path")
    rank.add_argument("--top", type=int, default=20)
    evidence = sub.add_parser("evidence")
    evidence.add_argument("path")
    evidence.add_argument("key")
    evidence.add_argument("value", type=float)
    evidence.add_argument("sigma", type=float)
    evidence.add_argument("--source", required=True)
    evidence.add_argument("--fidelity", type=int, required=True)
    evidence.add_argument("--note", default="")
    out = sub.add_parser("view")
    out.add_argument("path")
    out.add_argument("output")
    args = parser.parse_args(argv)
    ledger = load(args.path)
    if args.command == "rank":
        print_ranking(ledger, args.top)
    elif args.command == "evidence":
        ingest_evidence(ledger, args.key, args.value, args.sigma, args.source, args.fidelity, note=args.note)
        save(ledger, args.path)
        write_view(ledger, os.path.join(os.path.dirname(args.path), "view.json"))
        print_ranking(ledger, 10)
    else:
        write_view(ledger, args.output)


def write_view(ledger, path):
    with open(path, "w", encoding="utf-8") as file:
        json.dump(view(ledger), file, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
