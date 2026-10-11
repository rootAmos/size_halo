"""War-room feed: model results into the master ledger and the dashboard view, without re-solving.

- Plan 055 (whirl flutter in the sizing loop, PR #37 at bf74c0f): the stiffness the reference lacks to clear
  1.15 V_D (gap.whirl_flutter) and the priced delta-3 and wing-planform trades, from its results table.
- Plan 042 (impact x uncertainty): what drives range, cruise L/D and take-off mass of the fixed aircraft (80 % of the
  design payload), from a saved parametric table (`examples.halo_parametric`); calibration factors are not ranked.
- Plan 042 --all: every continuous input with its elasticity on every output (the coverage table), and the trades
  whose options are moves of continuous inputs priced from it to first order (take-off kg = sum dTOGW/dx dx, the
  re-sized KKT derivatives; confirm large moves by a re-solve).

    python -m examples.halo_war_room [--parametric output/fixed_parametric_family_080.pkl]
                                     [--all output/parametric_all.pkl]
"""
import argparse
import pickle
from datetime import date

from aircraft_closure.ledger.ingest import ingest_coverage, ingest_drivers, ingest_evidence, price_trade
from aircraft_closure.ledger.model import Quantity, TradeOption
from aircraft_closure.ledger.store import load
from aircraft_closure.uncertainty.registry import load_overrides
from examples.halo_ledger import apply_paths, apply_schedule, classify, gaps, gates, path_ledger, write
from examples.halo_parametric import coverage_records, driver_records, path_inputs, registry
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


# Discipline of each input, by the first keyword its key contains (order matters).
groups = (
    ("factors.", "Calibration factors"), ("requirements.", "Mission and requirements"),
    ("heat_exchanger", "Thermal"), ("coolant", "Thermal"), ("specific_heat", "Thermal"), ("temperature_max", "Thermal"),
    ("battery", "Battery"), ("cell", "Battery"), ("resistance_energy", "Battery"),
    ("motor", "Electrical machines and power"), ("generator", "Electrical machines and power"),
    ("machine", "Electrical machines and power"), ("inverter", "Electrical machines and power"),
    ("voltage", "Electrical machines and power"), ("cable", "Electrical machines and power"),
    ("feeder", "Electrical machines and power"), ("dcdc", "Electrical machines and power"),
    ("torque_density", "Electrical machines and power"), ("database", "Electrical machines and power"),
    ("turboshaft", "Turbogenerators and drive"), ("turbogenerator", "Turbogenerators and drive"),
    ("reduction_ratio", "Turbogenerators and drive"),
    ("solidity", "Rotor"), ("tip", "Rotor"), ("coning", "Rotor"), ("figure_of_merit", "Rotor"),
    ("download", "Rotor"), ("coefficient_airplane", "Rotor"),
    ("drag", "Aerodynamics"), ("excrescence", "Aerodynamics"), ("efficiency_tail", "Aerodynamics"),
    ("wetted", "Aerodynamics"), ("camber", "Aerodynamics"),
    ("fuselage", "Fuselage and tail"), ("tail", "Fuselage and tail"),
    ("wing", "Wing, nacelle and structure"), ("load_factor", "Wing, nacelle and structure"),
    ("mast", "Wing, nacelle and structure"), ("pylon", "Wing, nacelle and structure"),
    ("cowling", "Wing, nacelle and structure"), ("nacelle", "Wing, nacelle and structure"),
    ("equipment", "Systems and equipment"),
)


def group_of(key):
    return next((g for word, g in groups if word in key), "Other")


# First-order trade pricing uncertainty: the KKT derivative is exact at the reference; moves of 10-20 % carry the
# curvature and any limit that starts or stops binding (assumed 20 % of the change).
fraction_sigma_first_order = 0.20
same = {}
continuous_trades = {
    "trade.load_factors": [
        ("Ultimate 4.5, jump 2.0 (today)", same),
        ("Ultimate 4.0", {"assumptions.load_factor_ultimate": lambda x: 4.0}),
        ("Ultimate 5.0", {"assumptions.load_factor_ultimate": lambda x: 5.0}),
        ("Jump take-off 1.5", {"assumptions.load_factor_jump": lambda x: 1.5})],
    "trade.hover_cases": [
        ("Hot day 4,000 ft, 95 F; engine-out 60 s (today)", same),
        ("Hot day at 2,000 ft", {"requirements.altitude_hover_hot_m": lambda x: x / 2}),
        ("Hot day at 86 F", {"requirements.temperature_hover_hot_K": lambda x: x - 5.0}),
        ("Engine-out hover 30 s", {"requirements.duration_engine_out_hover_s": lambda x: x / 2})],
    "trade.payload_range": [
        ("900 kg over 445 nm (today)", same),
        ("800 kg payload", {"requirements.mass_payload_kg": lambda x: x - 100.0}),
        ("1,000 kg payload", {"requirements.mass_payload_kg": lambda x: x + 100.0}),
        ("400 nm range", {"requirements.range_m": lambda x: x * 400 / 445}),
        ("500 nm range", {"requirements.range_m": lambda x: x * 500 / 445})],
    "trade.battery_reserve": [
        ("Reserve loiter 20 min (today)", same),
        ("Reserve loiter 15 min", {"requirements.duration_loiter_s": lambda x: x * 0.75}),
        ("Reserve loiter 30 min", {"requirements.duration_loiter_s": lambda x: x * 1.5})],
    "trade.battery_life": [
        ("Size for 80 % capacity at end of life (today)", same),
        ("Replace at 90 % capacity", {"assumptions.factor_capacity_ageing_battery": lambda x: 0.9}),
        ("Run to 70 % capacity", {"assumptions.factor_capacity_ageing_battery": lambda x: 0.7})],
    "trade.rotor_span": [
        ("Rotor-fuselage clearance today", same),
        ("Clearance 0.1 m less", {"assumptions.clearance_rotor_fuselage_m": lambda x: x - 0.1})],
}


def feed_coverage(ledger, path, today):
    """The every-input table, and the continuous-input trades priced from it."""
    with open(path, "rb") as file:
        saved = pickle.load(file)
    tables = saved["tables"]
    inputs = registry(HaloRequirements(), HaloAssumptions(), calibration_factors(), load_overrides(path_inputs))
    records = coverage_records(tables, inputs, saved.get("rejected", ()))
    for r in records:
        r["group"] = group_of(r["key"])
    ingest_coverage(ledger, dict(outputs={m: t["outputs"] for m, t in tables.items()}, inputs=records),
                    "plan 042 every-input table", today)
    resized = tables["resized"]
    row = resized["jacobian"][resized["outputs"].index("mass_takeoff_kg")]
    derivative = dict(zip(resized["inputs"], row))
    value = dict(zip(resized["inputs"], resized["input_values"]))
    for key, options in continuous_trades.items():
        priced = []
        for label, moves in options:
            delta = float(sum(derivative[k] * (to(value[k]) - value[k]) for k, to in moves.items()))
            priced.append(TradeOption(label, "", delta, fraction_sigma_first_order * abs(delta),
                                      "plan 042 every-input table, first order (KKT); re-solve to confirm"))
        price_trade(ledger, key, priced, "plan 042 every-input table", today)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m examples.halo_war_room")
    parser.add_argument("--parametric", default="output/fixed_parametric_family_080.pkl",
                        help="pickled `parametric` table of a fixed-aircraft solve")
    parser.add_argument("--all", default="output/parametric_all.pkl",
                        help="pickled every-input tables (`examples.halo_parametric --all`)")
    args = parser.parse_args(argv)
    today = date.today()
    ledger = load(path_ledger)
    added, retired = register(ledger, gates, today.isoformat())
    feed_flutter(ledger, today)
    feed_drivers(ledger, args.parametric, today)
    feed_coverage(ledger, args.all, today)
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
