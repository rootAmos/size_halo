"""Halo decision ledger: seed it from the sized baseline, then keep it current from every new sizing or result.

    python -m examples.halo_ledger seed                 # sizes the baseline, writes ledger/halo.json + view
    python -m examples.halo_ledger seed --sensitivities # also re-solves for the model-input sensitivities
    python -m examples.halo_ledger resize               # re-sizes and ingests (sensitivities, limits, masses)
    python -m examples.halo_ledger trades               # adds new trade-register entries, no solve
    python -m aircraft_closure.ledger evidence ledger/halo.json mass.wing 452 25 --source "CalculiX" --fidelity 2

What is seeded and where it comes from:
- **Empty-weight items:** the 16 items of the empty-weight prediction (`halo_oew_uncertainty`), with their sized
  mass, maturity allowance and one-sigma spread.
- **Model gaps:** items the baseline does not carry. The HV installation is the model's own electrical layer (off
  by default); the others are placeholder priors (fidelity 0) for their owners to replace.
- **Model inputs:** drag excrescence factor, cell-to-pack mass fraction and end-of-life capacity, with spreads
  stated as assumptions; their take-off sensitivities come from re-solves (`--sensitivities`), else they are
  blind spots.
- **Limits:** every binding margin of the sizing, priced from the same solve.
- **Trades:** the register in `examples/halo_trades.py`, from concept to first article, plus decisions taken.
Gate dates are a placeholder programme calendar. Illustrative study, not Archer data.
"""
import argparse
import json
import os
import pickle
from dataclasses import replace
from datetime import date

from aircraft_closure.ledger.ingest import ingest_sizing, set_sensitivity
from aircraft_closure.ledger.model import Activity, Evidence, Ledger, Quantity
from aircraft_closure.ledger.store import load, save, snapshot, view
from examples.halo_oew_uncertainty import item_definitions, maturity_levels
from examples.halo_trades import register

directory = "ledger"
path_ledger = os.path.join(directory, "halo.json")
path_view = os.path.join(directory, "view.json")
source_sizing = "Halo sizing (baseline)"

# Placeholder programme calendar: gate -> lock date.
gates = {"Concept freeze": "2026-12-15", "Preliminary design review": "2027-06-30",
         "Critical design review": "2028-03-31", "First article": "2028-12-31"}

owners = {"Wing": "Structures", "Tails": "Structures", "Fuselage": "Structures", "Nacelles": "Structures",
          "Landing gear": "Structures", "Systems": "Flight controls and systems", "Fixed equipment": "Systems",
          "Rotors": "Rotor and drive", "Rotor gearboxes": "Rotor and drive", "Generator gearboxes": "Rotor and drive",
          "Motors (with inverters)": "Electrical", "Generators (with inverters)": "Electrical",
          "Turboshafts": "Propulsion", "Battery": "Electrical", "Heat exchanger": "Thermal",
          "Protection and bus tie": "Electrical"}

# Planned burn-down work already identified (docs/NEXT_STEPS.md); dates and target spreads are proposals.
plans = {
    "Wing": [Activity("FE: calibrated structure, root fittings and a buckling step", "2026-10-30", 0.06,
                      "Structures")],
    "Systems": [Activity("Second calibration aircraft (V-22 or AW609 group weights)", "2026-11-20", 0.09,
                         "Weights")],
    "Fuselage": [Activity("Second calibration aircraft (V-22 or AW609 group weights)", "2026-11-20", 0.07,
                          "Weights")],
    "Rotors": [Activity("Blade and hub layout weights", "2027-01-31", 0.05, "Rotor and drive")],
    "Battery": [Activity("Pack layout: cell-to-pack fraction from a module design", "2026-12-01", 0.04,
                         "Electrical")],
}

# (key, label, owner, mass kg, sigma kg, fidelity, gate, source, note)
gaps = (
    ("gap.hv_installation", "HV installation: inverters, cables, protection, partial-discharge insulation",
     "Electrical", 330.0, 100.0, 1, "Concept freeze", "electrical layer (off in the baseline)",
     "Modelled but off by default; also adds about 3 % losses"),
    ("gap.tilt_fittings", "Tilt spindle, tip ribs and fittings", "Structures", 70.0, 50.0, 0,
     "Preliminary design review", "placeholder prior", "Owner to replace with a layout estimate"),
    ("gap.conversion_actuators", "Conversion actuators and backup structure", "Flight controls and systems", 45.0,
     30.0, 0, "Preliminary design review", "placeholder prior", "Owner to replace"),
    ("gap.coolant_loop", "Coolant loop: plumbing, pumps, fluid", "Thermal", 40.0, 30.0, 0, "Concept freeze",
     "placeholder prior", "Owner to replace"),
    ("gap.tilt_joint_crossings", "HV and coolant crossings at the tilt joint", "Electrical", 20.0, 15.0, 0,
     "Preliminary design review", "placeholder prior", "Flex loops or slip rings; swivel joints"),
    ("gap.wing_strength", "Wing structure added for ultimate strength", "Structures", 40.0, 45.0, 0,
     "Concept freeze", "placeholder prior",
     "FE peak cap strain 10 % over the ultimate allowable on raw gauges; 0.83 of it if the 1.33 calibration "
     "material is load-carrying"),
)

# (key, label, owner, assumptions field, value, sigma, step, gate, note)
inputs = (
    ("input.drag_excrescence", "Drag excrescence factor on AeroBuildup", "Aerodynamics", "factor_excrescence_buildup",
     1.27, 0.10, 0.02, "Concept freeze", "Calibrated to NDARC XV-15 components, not flight"),
    ("input.cell_to_pack", "Cell-to-pack mass fraction", "Electrical", "fraction_mass_cells_battery", 0.70, 0.025,
     0.01, "Concept freeze", "Cylindrical-cell packs about 0.65-0.75"),
    ("input.capacity_end_of_life", "Battery capacity at end of life", "Electrical", "factor_capacity_ageing_battery",
     0.80, 0.05, 0.01, "Preliminary design review", "Assumed 80 % of rated"),
)

# Margin label patterns -> (plain name, owner) for the binding limits. "{c}" is the flight condition.
limit_patterns = (
    ("rotor_radius_m", "Rotor radius capped by the span (rotor diameter against span)", "Configuration"),
    ("battery end voltage", "{c}: battery end voltage (2.5 V per cell cutoff)", "Electrical"),
    ("heat_exchanger power_heat", "{c}: heat rejection at the exchanger rating", "Thermal"),
    ("turboshaft power_shaft", "{c}: turboshaft power at the fixed 1,120 hp deck", "Propulsion"),
    ("generator_gearbox power_input", "{c}: turboshaft power at the fixed 1,120 hp deck", "Propulsion"),
    ("propulsor power_shaft", "{c}: rotor and gearbox power rating", "Rotor and drive"),
    ("gearbox power_input", "{c}: rotor and gearbox power rating", "Rotor and drive"),
    ("motor torque", "{c}: motor torque rating", "Electrical"),
    ("generator torque", "{c}: generator torque rating", "Electrical"),
    ("static_margin", "Static margin", "Flight controls and systems"),
    ("cn_beta", "Directional stability (Cn beta)", "Flight controls and systems"),
)


def limit_names(margins):
    names, limit_owners = {}, {}
    for label, _, _ in margins:
        condition = label.split(":")[0] if ":" in label else ""
        for pattern, name, owner in limit_patterns:
            if pattern in label:
                names[label], limit_owners[label] = name.format(c=condition), owner
                break
    return names, limit_owners


def slug(label):
    return "mass." + "".join(c if c.isalnum() else "_" for c in label.lower().split(" (")[0]).strip("_")


def seed(result, today):
    ledger = Ledger("Halo decision ledger", gates=gates)
    sigma_fraction = {}
    for label, group, keys, maturity, sigma, basis in item_definitions:
        key = slug(label)
        sigma_fraction[key] = sigma
        gate = "Preliminary design review" if group == "Airframe and systems" else "Concept freeze"
        ledger.quantities[key] = Quantity(
            key, label, "mass", "kg", owners[label], gate, gates[gate], sizing_keys=tuple(keys),
            plan=[replace(a) for a in plans.get(label, [])], note=f"{maturity}; {basis}")
    for key, label, owner, mass, sigma, fidelity, gate, source, note in gaps:
        ledger.quantities[key] = Quantity(key, label, "gap", "kg", owner, gate, gates[gate],
                                          evidence=[Evidence(today.isoformat(), mass, sigma, source, fidelity)],
                                          note=note)
    for key, label, owner, name, value, sigma, step, gate, note in inputs:
        ledger.quantities[key] = Quantity(key, label, "input", "-", owner, gate, gates[gate],
                                          evidence=[Evidence(today.isoformat(), value, sigma, "assumption", 1)],
                                          note=note)
    register(ledger, gates, today.isoformat())
    ingest_sizing(ledger, result, source_sizing, today, sigma_fraction, *limit_names(result.sensitivity.margins))
    for label, group, keys, maturity, sigma, basis in item_definitions:
        q = ledger.quantities[slug(label)]
        mean = q.belief()[0]
        q.allowance = maturity_levels[maturity] * mean
        q.plan = [replace(a, sigma_after=a.sigma_after * mean) for a in q.plan]   # fractions -> kg
    ledger.history.clear()                     # the ingest's snapshot predates the allowances and plans
    snapshot(ledger, source_sizing, today)
    return ledger


def fixed_units(assumptions, result):
    counts = {name: fixed for name, (relaxed, fixed) in (result.machine_units or {}).items()}
    if not counts:
        return assumptions
    return replace(assumptions, count_units_motor=counts["motor"], count_units_generator=counts["generator"])


def input_sensitivities(ledger, result, today):
    """Central differences on take-off mass, each a warm-started re-solve at the baseline's unit counts."""
    from examples.halo_sizing import HaloAssumptions, solve_halo_sizing
    base = fixed_units(HaloAssumptions(), result)
    for key, label, owner, name, value, sigma, step, gate, note in inputs:
        masses = [solve_halo_sizing(assumptions=replace(base, **{name: value + sign * step}), initial=result)
                  .mass_takeoff_kg for sign in (-1, 1)]
        set_sensitivity(ledger, key, (masses[1] - masses[0]) / (2 * step),
                        f"central difference ±{step:g}, Halo sizing", today)


def baseline_result(cache):
    if cache and os.path.exists(cache):
        with open(cache, "rb") as file:
            return pickle.load(file)
    from examples.halo_sizing import solve_halo_sizing
    return solve_halo_sizing()


def write(ledger, today):
    os.makedirs(directory, exist_ok=True)
    save(ledger, path_ledger)
    with open(path_view, "w", encoding="utf-8") as file:
        json.dump(view(ledger, today), file, indent=1, ensure_ascii=False)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m examples.halo_ledger")
    parser.add_argument("command", choices=("seed", "resize", "trades"))
    parser.add_argument("--cache", default=None, help="pickled HaloSizingResult to use instead of solving")
    parser.add_argument("--sensitivities", action="store_true", help="re-solve for the model-input sensitivities")
    args = parser.parse_args(argv)
    today = date.today()
    if args.command == "trades":                 # add new register entries to the master ledger, no solve
        ledger = load(path_ledger)
        added, retired = register(ledger, gates, today.isoformat())
        ledger.runs.append(dict(date=today.isoformat(), source="trade register (examples/halo_trades.py)",
                                kind="register", summary=f"{added} added, {retired} retired below the line"))
        snapshot(ledger, "trade register", today)
        write(ledger, today)
        counts = {s: sum(t.status == s for t in ledger.trades.values()) for s in ("open", "decided", "retired")}
        print(f"{added} added, {retired} retired; {counts}")
        return
    result = baseline_result(args.cache)
    if args.command == "seed":
        ledger = seed(result, today)
    else:
        ledger = load(path_ledger)
        ingest_sizing(ledger, result, source_sizing, today, None, *limit_names(result.sensitivity.margins))
    if args.sensitivities:
        input_sensitivities(ledger, result, today)
    write(ledger, today)
    from aircraft_closure.ledger.__main__ import print_ranking
    print_ranking(ledger, 20)


if __name__ == "__main__":
    main()
