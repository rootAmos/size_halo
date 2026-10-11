"""Plan 048: the v3.6 reference with machine units allowed a small uprating, one coupled solve per case.

Usage: python -m examples.halo_unit_uprating <case> <start.pkl> <output directory>
Cases: "reference" (factor 1.0), "power_1.05", "power_1.10" (the author's assumption), "power_1.10_ntu" (with the
plan 042 cooler).
Each run starts from `start.pkl` (a HaloSizingResult), solves the relaxed unit counts, rounds them up to whole units
of the (uprated) unit and re-solves with the counts fixed (`solve_halo_sizing`'s plan 035 path), and writes
<case>.json with take-off mass, unit counts, battery, cooler and the binding constraints.
"""
import json
import pickle
import sys
import time
from dataclasses import replace
from pathlib import Path

from examples.halo_sizing import HaloAssumptions, HaloRequirements, solve_halo_sizing

cases = {
    "reference": dict(factor_uprating_power_units=1.0),
    "power_1.05": dict(factor_uprating_power_units=1.05),
    "power_1.10": dict(factor_uprating_power_units=1.10),
    "power_1.10_ntu": dict(factor_uprating_power_units=1.10, heat_exchanger_model="ntu"),
    "v3_6": dict(factor_uprating_power_units=1.0),    # the catalogue rating: must reproduce 15,179 lb
    "v3_7": dict(),                                   # the defaults (10 % uprating, inverter at catalogue mass)
    "v3_7_ntu": dict(heat_exchanger_model="ntu"),
}

if __name__ == "__main__":
    case, path_start, directory = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    directory.mkdir(parents=True, exist_ok=True)
    start = pickle.loads(path_start.read_bytes())
    assumptions = replace(HaloAssumptions(), **cases[case])
    time_start_s = time.time()
    result = solve_halo_sizing(HaloRequirements(), assumptions, initial=start)
    lb = 2.20462
    summary = dict(
        case=case, seconds=round(time.time() - time_start_s), mass_takeoff_kg=result.mass_takeoff_kg,
        mass_takeoff_lb=result.mass_takeoff_kg * lb, mass_empty_kg=result.mass_empty_kg,
        machine_units={name: list(counts) for name, counts in (result.machine_units or {}).items()},
        energy_battery_kWh=result.energy_battery_kWh,
        mass_heat_exchanger_kg=(result.heat_exchanger or {}).get("mass_kg"),
        binding=list(result.binding))
    (directory / f"{case}.json").write_text(json.dumps(summary, indent=1, default=float))
    (directory / f"{case}.pkl").write_bytes(pickle.dumps(result))
    print(json.dumps({k: summary[k] for k in ("case", "seconds", "mass_takeoff_lb", "machine_units")}, default=float))
