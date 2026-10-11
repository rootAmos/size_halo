"""Plan 049 check: is the cooling fan's use in the minimum-time climb cooling or thrust?

Flies the minimum-time climb of the sized NTU aircraft twice: as sized, and with the cooling fan's rating cut to
100 W (practically off). If the capped climb solves at nearly the same time and energy with machine temperatures
inside their limits, the fan was buying thrust; if it is infeasible, slower or hotter, the fan was cooling.

Usage: python -m examples.halo_climb_fan_check <sizing.pkl>   (a HaloSizingResult solved with the defaults)
"""
import pickle
import sys
from dataclasses import replace

from examples.halo_sizing import HaloAssumptions, HaloRequirements
from examples.halo_trajectory_computed_corridor import computed_corridor_case
from examples.trajectory_optimization import describe, solve_min_time_climb

if __name__ == "__main__":
    sizing = pickle.loads(open(sys.argv[1], "rb").read())
    r, a = HaloRequirements(), HaloAssumptions()
    for label, rating_W in (("as sized", None), ("fan capped at 100 W", 100.0)):
        sized = sizing if rating_W is None else replace(
            sizing, design=replace(sizing.design, power_rated_fan_heat_exchanger_W=rating_W))
        try:
            result = solve_min_time_climb(computed_corridor_case(sized, requirements=r, assumptions=a))
            print(f"== {label}\n{describe(result)}")
        except RuntimeError as error:
            print(f"== {label}: did not solve ({str(error).splitlines()[-1][:160]})")
