"""Plan 042: every Halo performance output against every uncertain input, from one sizing solve.

Lifts the inputs in `ledger/inputs.json` to Opti parameters, sizes the reference (re-sized mode: the mission held,
the aircraft re-closes), and reports the elasticities (% output per % input), the drivers of each output by
impact x uncertainty, and the blind spots. Writes a run manifest to `ledger/runs/`.

    python -m examples.halo_parametric            # one solve
    python -m examples.halo_parametric --check    # plus central-difference re-solves of a few entries

Spreads in `ledger/inputs.json` marked "placeholder prior" are for owners to replace.
"""
import argparse
from dataclasses import fields, replace
from pathlib import Path

import numpy as np

from aircraft_closure.core.sensitivity import elasticities
from aircraft_closure.uncertainty.manifest import build_manifest, write_manifest
from aircraft_closure.uncertainty.rank import rank_drivers
from aircraft_closure.uncertainty.registry import apply_overrides, inputs_from_dataclass, load_overrides
from examples.halo_payload_range import sized_reference
from examples.halo_sizing import HaloAssumptions, HaloRequirements, solve_halo_off_design, solve_halo_sizing_once
from examples.xv15_reference import calibration_factors

path_inputs = Path("ledger/inputs.json")
path_runs = Path("ledger/runs")
# Fixed-aircraft mode flies this share of the design payload for maximum range: the full-payload corner of the
# payload-range diagram (take-off mass, tanks and payload all at their limits) is degenerate for IPOPT.
fraction_payload_fixed = 0.9
# Entries re-solved by central difference under --check: (mode, output, input, relative step).
checks = (("resized", "mass_takeoff_kg", "assumptions.aspect_ratio_wing", 0.02),
          ("resized", "mass_takeoff_kg", "assumptions.fraction_mass_cells_battery", 0.02),
          ("fixed", "range_m", "assumptions.aspect_ratio_wing", 0.02))


def registry(requirements, assumptions, factors, overrides):
    inputs = (inputs_from_dataclass(requirements, "requirements") + inputs_from_dataclass(assumptions, "assumptions")
              + inputs_from_dataclass(factors, "factors"))
    return apply_overrides(inputs, overrides)


def solve(mode, requirements, assumptions, factors, sized, keys):
    """Re-sized: the mission held, the aircraft re-closes. Fixed: the sized aircraft flies its design payload as far
    as its tanks allow with `fraction_payload_fixed` of the design payload (plan 045 off-design, maximum range)."""
    if mode == "resized":
        return solve_halo_sizing_once(requirements, assumptions, factors, initial=sized, parameters=keys)
    return solve_halo_off_design(sized, mass_payload_kg=fraction_payload_fixed * requirements.mass_payload_kg,
                                 requirements=requirements,
                                 assumptions=assumptions, factors=factors, parameters=keys).result


def with_input(requirements, assumptions, factors, key, value):
    holder, _, name = key.partition(".")
    if holder == "requirements":
        return replace(requirements, **{name: value}), assumptions, factors
    if holder == "assumptions":
        return requirements, replace(assumptions, **{name: value}), factors
    return requirements, assumptions, replace(factors, **{name: value})


def report(parametric, inputs):
    by_key = {i.key: i for i in inputs}
    unused = set(parametric.get("inputs_unused", ()))
    jacobian = np.asarray(parametric["jacobian"], dtype=float)
    elasticity = elasticities(jacobian, parametric["output_values"], parametric["input_values"])
    used = [k for k, key in enumerate(parametric["inputs"]) if key not in unused]
    columns = [by_key[parametric["inputs"][k]] for k in used]
    print(f"\nElasticity, % output per % input ({parametric['mode']}, objective {parametric['objective']}):")
    width = max(len(k) for k in parametric["inputs"])
    print(" " * width + "".join(f"{o[:12]:>13}" for o in parametric["outputs"]))
    for k, key in enumerate(parametric["inputs"]):
        print(f"{key:<{width}}" + "".join(f"{elasticity[i, k]:>13.3f}" for i in range(len(parametric["outputs"]))))
    if unused:
        print(f"\nNot read by the active models (zero by construction): {', '.join(sorted(unused))}")
    drivers = rank_drivers(parametric["outputs"], columns, jacobian[:, used], elasticity[:, used])
    for output, value in zip(parametric["outputs"], parametric["output_values"]):
        risks, levers, blind = drivers[output]
        print(f"\n{output} = {value:.6g}")
        spread = float(np.sqrt(sum(d.spread ** 2 for d in risks)))
        if spread <= 1e-9 * max(abs(value), 1.0):
            print("  held at a limit or fixed by the mission: no spread from these inputs")
            continue
        for d in [d for d in risks if d.share >= 1e-3][:5]:
            print(f"  risk  {d.input_key:<48} share {d.share:6.1%}  1-sigma {d.spread:+.4g}  elasticity {d.elasticity:+.3f}")
        for d in levers[:3]:
            print(f"  lever {d.input_key:<48} elasticity {d.elasticity:+.3f}")
        if blind:
            print(f"  blind spots: {', '.join(blind)}")
    return elasticity


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="re-solve a few entries by central difference")
    args = parser.parse_args()
    requirements, assumptions, factors = HaloRequirements(), HaloAssumptions(), calibration_factors()
    overrides = load_overrides(path_inputs)
    inputs = registry(requirements, assumptions, factors, overrides)
    keys = list(overrides)
    sized = sized_reference()
    tables = {}
    for mode in ("resized", "fixed"):
        tables[mode] = solve(mode, requirements, assumptions, factors, sized, keys).parametric
        report(tables[mode], inputs)

    checked = []
    if args.check:
        print("\nChecks (central-difference re-solves):")
        for mode, output, key, step in checks:
            table = tables[mode]
            i, k = table["outputs"].index(output), table["inputs"].index(key)
            x = table["input_values"][k]
            values = []
            for sign in (-1, 1):
                case = with_input(requirements, assumptions, factors, key, x * (1 + sign * step))
                values.append(getattr(solve(mode, *case, sized, ()), output))
            finite = (values[1] - values[0]) / (2 * step * x)
            checked.append(dict(mode=mode, output=output, input=key, step=step,
                                jacobian=float(table["jacobian"][i][k]), central_difference=finite))
            print(f"  {mode}: d {output} / d {key}: KKT {table['jacobian'][i][k]:.5g}, re-solve {finite:.5g}")

    resized = tables["resized"]
    manifest = build_manifest("halo sizing, parametric (plan 042)", inputs,
                              dict(zip(resized["outputs"], resized["output_values"])),
                              sensitivity=dict(tables=tables, checks=checked))
    print(f"\nManifest: {write_manifest(path_runs, manifest)}")


if __name__ == "__main__":
    main()
