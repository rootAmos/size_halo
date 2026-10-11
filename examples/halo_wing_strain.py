"""Halo sized with the ultimate spar-cap strain margin (plan 043).

One coupled solve from a cached reference result (`--initial`, a pickled `HaloSizingResult`) with
`HaloAssumptions(wing_strain_margin=True)` and the given load-carrying share of the wing calibration increment.
The machine unit counts are held at the start's whole counts (one solve, no relaxed pre-solve). Writes the result
pickle, a summary JSON and the structure reference for the CalculiX re-check (`examples/halo_wing_fe.py
--reference`) into `--directory`. Run from the repo root:
`python -m examples.halo_wing_strain --initial <baseline.pkl> --share 0.5 --directory output/wing_strain/share050`.
"""
import argparse
import json
import pickle
import time
from dataclasses import replace
from pathlib import Path

import aerosandbox.tools.units as u

from examples.halo_sizing import HaloAssumptions, HaloRequirements, solve_halo_sizing
from examples.halo_structure_reference import structure_reference


class _Unknown:
    """Stands in for classes a pickle names that this checkout does not define (fields of other branches)."""

    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        if isinstance(state, dict):
            self.__dict__.update(state)


class _TolerantUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        try:
            return super().find_class(module, name)
        except AttributeError:
            return _Unknown


def load_result(path):
    with open(path, "rb") as file:
        return _TolerantUnpickler(file).load()


def wing_summary(result):
    parts = dict(result.wing_masses_kg)
    return dict(mass_takeoff_kg=result.mass_takeoff_kg, mass_takeoff_lb=result.mass_takeoff_kg / u.lbm,
                wing_afdd_kg=parts, wing_strain=result.wing_strain, binding=list(result.binding),
                area_wing_m2=result.design.area_wing_m2, span_m=result.span_m,
                component_masses_kg=dict(result.component_masses_kg))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial", required=True)
    parser.add_argument("--share", type=float, default=0.5)
    parser.add_argument("--directory", default="output/wing_strain")
    args = parser.parse_args()
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    initial = load_result(args.initial)
    units = initial.machine_units
    requirements = HaloRequirements()
    assumptions = HaloAssumptions(wing_strain_margin=True, fraction_load_carrying_calibration_wing=args.share,
                                  count_units_motor=units["motor"][1], count_units_generator=units["generator"][1])
    start_s = time.time()
    result = solve_halo_sizing(requirements, assumptions, initial=initial)
    print(f"Solved in {time.time() - start_s:.0f} s")
    with open(directory / "result.pkl", "wb") as file:
        pickle.dump(result, file)
    summary = dict(share=args.share, sized=wing_summary(result), initial=dict(
        mass_takeoff_kg=initial.mass_takeoff_kg, mass_takeoff_lb=initial.mass_takeoff_kg / u.lbm,
        wing_afdd_kg=dict(initial.wing_masses_kg), component_masses_kg=dict(initial.component_masses_kg)))
    (directory / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
    reference = structure_reference(result, requirements, replace(assumptions))
    (directory / "reference.json").write_text(json.dumps(reference, indent=1))
    s = result.wing_strain
    print(f"Take-off {result.mass_takeoff_kg:.1f} kg ({result.mass_takeoff_kg / u.lbm:.0f} lb), start "
          f"{initial.mass_takeoff_kg / u.lbm:.0f} lb; wing {dict(result.component_masses_kg)['wing']:.1f} kg "
          f"(start {dict(initial.component_masses_kg)['wing']:.1f} kg); added caps {1e6 * s['area_spar_added_m2']:.0f}"
          f" mm2; peak strain {s['strain_peak']:.5f} / {s['strain_allowable']:.4f} (margin {100 * s['margin']:+.1f} %)")
    print("Binding:", ", ".join(result.binding))
