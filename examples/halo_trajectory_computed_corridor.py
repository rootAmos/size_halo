"""Tier 14 trajectories flown inside the computed conversion corridor (plan 039) instead of the assumed one.

The aircraft is sized (`solve_halo_sizing()`), its corridor is computed from level-flight trim at sea level
(`examples/halo_conversion_corridor.py`), and the minimum-energy transition and the minimum time to climb are flown
with that corridor as their airspeed limits. The corridor is computed at sea level and the take-off mass; the climb
applies it up to 10,000 ft, an approximation.

Finding: the minimum-energy transition rides the corridor's low-speed side from about 15 to 100 kt and descends
(flight-path angle down to about -10 deg, inside the +/-30 m altitude band) through 15-90 kt. No level,
constant-acceleration conversion exists inside the computed corridor: with the nacelles high enough for the
corridor, accelerating needs a nose-down attitude beyond the trajectory model's -10 deg angle-of-attack limit (the
trajectory model has no rotor disc tilt, unlike the trim that computes the corridor). So the naive
prescribed transition of `examples/trajectory_optimization.py` is not flown here.

    python examples/halo_trajectory_computed_corridor.py   # writes output/trajectory/computed_corridor.png
"""
import os

import aerosandbox.tools.units as u

from aircraft_closure.trajectory.corridor import ComputedCorridor
from examples.halo_conversion_corridor import run as run_corridor
from examples.halo_sizing import solve_halo_sizing
from examples.trajectory_optimization import (describe, halo_trajectory_case, solve_min_energy_transition,
                                              solve_min_time_climb)


def computed_corridor_case(sizing, assumed=None):
    """The trajectory case of the sized aircraft with its computed corridor as the airspeed limits."""
    assumed = assumed if assumed is not None else halo_trajectory_case(sizing)
    bounds, _ = run_corridor(sizing, plot=False)
    corridor = ComputedCorridor.from_bounds(bounds, velocity_stall_m_s=assumed.corridor.velocity_stall_m_s)
    return halo_trajectory_case(sizing, corridor=corridor)


def run(sizing=None, path="output/trajectory/computed_corridor.png"):
    sizing = sizing if sizing is not None else solve_halo_sizing()
    assumed = halo_trajectory_case(sizing)
    case = computed_corridor_case(sizing, assumed)
    results = [solve_min_energy_transition(case), solve_min_time_climb(case)]
    for result in results:
        print(describe(result))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    plot(case, results, path)
    return case, results


def plot(case, results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    corridor = case.corridor
    tilts = list(corridor.tilts_deg)
    low = [v / u.knot for v in corridor.velocity_low_m_s]
    high = [v / u.knot for v in corridor.velocity_high_m_s]
    fig, ax = plt.subplots(figsize=(8, 4.6))
    ax.fill_betweenx(tilts, low, high, color="#cfe3f3", label="computed corridor (plan 039, sea level)")
    ax.plot(low, tilts, color="#1d5c8f", lw=1.2)
    ax.plot(high, tilts, color="#1d5c8f", lw=1.2)
    colors = {"minimum-energy transition": "#2a78d6", "prescribed transition": "#eb6834",
              "minimum time to climb": "#1baf7a"}
    # Each leg's end points, from the problem definitions in examples/trajectory_optimization.py.
    legs = {"minimum-energy transition": "hover at 500 ft to 1.3 x stall in airplane mode, level",
            "minimum time to climb": "hover at sea level to 10,000 ft at 165 kt"}
    for result in results:
        leg = f": {legs[result.label]}" if result.label in legs else ""
        label = f"{result.label}{leg} ({result.energy_bus_J[-1] / 3.6e6:.1f} kWh, {result.duration_s:.0f} s)"
        ax.plot(result.velocity_m_s / u.knot, result.tilt_deg, color=colors.get(result.label, "#333333"), lw=2,
                label=label)
    ax.set(xlabel="true airspeed [kt]", ylabel="nacelle tilt [deg]", xlim=(0, 240), ylim=(0, 92),
           title="Trajectories inside the computed conversion corridor")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=1, frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    run()
