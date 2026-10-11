"""War-room feed: model results into the master ledger and the dashboard view, without re-solving.

- Plan 055 (whirl flutter in the sizing loop, PR #37 at bf74c0f): the stiffness the reference lacks to clear
  1.15 V_D (gap.whirl_flutter) and the priced delta-3 and wing-planform trades, from its results table.
- Plan 042 (impact x uncertainty): what drives range, cruise L/D and take-off mass of the fixed aircraft (80 % of the
  design payload), from a saved parametric table (`examples.halo_parametric`); calibration factors are not ranked.

    python -m examples.halo_war_room [--parametric output/fixed_parametric_family_080.pkl]
"""
import argparse
import pickle
from datetime import date

from aircraft_closure.ledger.ingest import ingest_drivers, ingest_evidence, price_trade
from aircraft_closure.ledger.model import Quantity, TradeOption
from aircraft_closure.ledger.store import load
from aircraft_closure.uncertainty.registry import load_overrides
from examples.halo_ledger import apply_paths, apply_schedule, classify, gaps, gates, path_ledger, write
from examples.halo_parametric import driver_records, path_inputs, registry
from examples.halo_sizing import HaloAssumptions, HaloRequirements
from examples.halo_trades import register
from examples.xv15_reference import calibration_factors

lb = 2.20462
source_055 = "plan 055 sizing (bf74c0f)"
# Plan 055 results (docs/decisions/055-whirl-flutter-in-the-loop.md): take-off lb, one solve each from the same start.
takeoff_055_lb = dict(proxy=13_834, loop=14_227, delta3=13_834, wing=13_923, all=13_484)
# Option uncertainty: the flutter calibration rests on one aircraft (XV-15); 25 % of each change, assumed.
fraction_sigma_055 = 0.25


# Delta-3's cost (gimbal flapping, limit 9 deg) is not modelled yet: its whole gain is uncertain until it is.
fraction_sigma_delta3 = 1.0


def option(label, case, reference="loop", fraction_sigma=fraction_sigma_055):
    delta_kg = (takeoff_055_lb[case] - takeoff_055_lb[reference]) / lb
    return TradeOption(label, "", delta_kg, fraction_sigma * abs(delta_kg),
                       f"{source_055}; sigma {fraction_sigma:.0%} of the change, assumed")


def feed_flutter(ledger, today):
    key, label, owner, mass, sigma, fidelity, gate, source, note = next(g for g in gaps if g[0] == "gap.whirl_flutter")
    if key not in ledger.quantities:
        ledger.quantities[key] = Quantity(key, label, "gap", "kg", owner, gate, gates[gate], note=note)
    q = ledger.quantities[key]
    q.sensitivity, q.sensitivity_source = ledger.growth_factor, "growth factor, latest sizing"
    ingest_evidence(ledger, key, mass, sigma, source, fidelity, today, note="take-off +393 lb over the proxy")
    price_trade(ledger, "trade.delta3", [
        option("-18 deg (the XV-15's)", "loop"),
        option("-11 deg (flapping cost not modelled)", "delta3", fraction_sigma=fraction_sigma_delta3)],
        source_055, today)
    # The wing alone (delta-3 at -18 deg), so the two trades do not count the delta-3 gain twice; together they reach
    # 13,484 lb (AR 7.2, 27 %, delta-3 -10 deg), 743 lb under the compliant reference.
    price_trade(ledger, "trade.wing_planform", [
        option("AR 6.12, 23 % (today)", "loop"), option("AR 7.0, 28 % (thickness at its bound)", "wing")],
        source_055, today)


def feed_drivers(ledger, path, today):
    with open(path, "rb") as file:
        parametric = pickle.load(file)
    inputs = registry(HaloRequirements(), HaloAssumptions(), calibration_factors(), load_overrides(path_inputs))
    ingest_drivers(ledger, driver_records(parametric, inputs),
                   "plan 042 fixed aircraft, 80 % payload (36f08aa)", today)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m examples.halo_war_room")
    parser.add_argument("--parametric", default="output/fixed_parametric_family_080.pkl",
                        help="pickled `parametric` table of a fixed-aircraft solve")
    args = parser.parse_args(argv)
    today = date.today()
    ledger = load(path_ledger)
    added, retired = register(ledger, gates, today.isoformat())
    feed_flutter(ledger, today)
    feed_drivers(ledger, args.parametric, today)
    classify(ledger)
    apply_paths(ledger)
    apply_schedule(ledger)
    write(ledger, today)
    print(f"register: {added} added, {retired} retired")
    for key, record in ledger.drivers.items():
        print(f"{record['label']} = {record['value']:,.4g} {record['unit']}: "
              + "; ".join(f"{r['label']} {r['share']:.0%}" for r in record["risks"][:4]))


if __name__ == "__main__":
    main()
