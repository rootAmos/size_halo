"""Halo sizing with the solidity floor the blade-element design supports (plan 047).

One solve from the cached 15,179 lb baseline with `HaloAssumptions(solidity_min=0.067)`: the least thrust-weighted
solidity at which the blade-element blade hovers at the sizing's figure of merit with a 2 deg section stall margin
(examples/halo_blade_design.py). Everything else is the reference assumption set.

Usage: python -m examples.halo_blade_solidity_sizing <baseline.pkl> <output.pkl>
"""
import pickle
import sys
from dataclasses import replace


class _Placeholder:
    """Stands in for classes of the baseline pickle that this checkout does not define (not used by the solve)."""
    def __init__(self, *args, **kwargs):
        pass

    def __setstate__(self, state):
        self.__dict__.update(state if isinstance(state, dict) else {})


class _Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        try:
            return super().find_class(module, name)
        except AttributeError:
            return _Placeholder


def main(path_baseline, path_output, solidity_min=0.067):
    from examples.halo_sizing import HaloAssumptions, solve_halo_sizing
    baseline = _Unpickler(open(path_baseline, "rb")).load()
    result = solve_halo_sizing(assumptions=replace(HaloAssumptions(), solidity_min=solidity_min), initial=baseline)
    pickle.dump(result, open(path_output, "wb"))
    print(f"solidity_min {solidity_min}: take-off mass {result.mass_takeoff_kg:.1f} kg "
          f"({result.mass_takeoff_kg / 0.45359237:.0f} lb), baseline {baseline.mass_takeoff_kg:.1f} kg; "
          f"solidity {result.design.solidity:.4f}, fuel {result.mass_fuel_kg:.1f} kg (baseline {baseline.mass_fuel_kg:.1f})")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
