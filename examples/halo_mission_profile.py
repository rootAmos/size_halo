"""Mission profiles of the sized Halo baseline: airspeed, battery and turbine power, altitude against time.

Two figures:

- the design mission as sized (take-off hover, climb, cruise, reserve loiter, descent, landing hover), each segment
  at the operating point the sizing solved for it;
- the same aircraft and mission with an optimized take-off: the take-off hover and the prescribed climb are replaced
  by the minimum-time climb from hover at sea level to cruise speed at 10,000 ft (`solve_min_time_climb`), flown
  inside the computed conversion corridor. The aircraft is not re-sized, and the cruise is lengthened or shortened
  so that the mission covers the same distance.

Powers: turbine shaft power summed over the turboshafts, and battery power (discharge positive, recharge negative).

    python -m examples.halo_mission_profile    # writes output/mission/mission_profile*.png
"""
import os

import aerosandbox.numpy as np
import numpy
import aerosandbox.tools.units as u

from examples.halo_sizing import HaloAssumptions, solve_halo_sizing

watts_per_hp = 745.69987
blue, orange, grey = "#2a78d6", "#eb6834", "#5e5d58"
duration_zoom_s = 15 * 60.0     # left-hand panels: the first 15 min


def segment_series(segments, time_start_s=0.0):
    """Step series (time, airspeed, turbine power, battery power, altitude) from the sizing's segment summaries.

    Airspeed and powers are constant over a segment (its solved operating point); altitude is linear between the
    segment's start and end.
    """
    rows, t = [], time_start_s
    for s in segments:
        t_end = t + s["duration_s"]
        for time_s, altitude_m in ((t, s["altitude_start_m"]), (t_end, s["altitude_end_m"])):
            rows.append((time_s, s["velocity_m_s"], s["power_turboshafts_W"], s["power_battery_W"], altitude_m))
        t = t_end
    return np.array(rows, dtype=float)


def regular_mission(sizing):
    return segment_series(sizing.segments), [(s["label"], s["duration_s"]) for s in sizing.segments]


def optimized_takeoff_mission(sizing, climb=None):
    """The mission with the minimum-time climb in place of the take-off hover and prescribed climb.

    `climb`: a `TrajectoryResult` of `solve_min_time_climb`; solved here (computed corridor) when not given.
    """
    if climb is None:
        from examples.halo_trajectory_computed_corridor import computed_corridor_case
        from examples.trajectory_optimization import solve_min_time_climb
        climb = solve_min_time_climb(computed_corridor_case(sizing))
    segments = [dict(s) for s in sizing.segments]
    hover, climb_prescribed, cruise = segments[0], segments[1], segments[2]
    # Same mission distance: the cruise absorbs the difference in ground distance covered by the climb.
    distance_prescribed_m = climb_prescribed["velocity_m_s"] * climb_prescribed["duration_s"]
    cruise["duration_s"] += (distance_prescribed_m - climb.x_m[-1]) / cruise["velocity_m_s"]
    # `power_shaft_generator_W` is per turbogenerator (turboshaft output, with the step-up gearbox loss).
    power_turbines_W = HaloAssumptions().count_turbogenerators * climb.power_shaft_generator_W
    rows = np.column_stack([climb.time_s, climb.velocity_m_s, power_turbines_W, climb.power_battery_W,
                            climb.altitude_m])
    rest = segment_series(segments[2:], time_start_s=climb.time_s[-1])
    labels = [("optimized take-off and climb", climb.duration_s)] + [(s["label"], s["duration_s"])
                                                                     for s in segments[2:]]
    summary = dict(duration_prescribed_s=hover["duration_s"] + climb_prescribed["duration_s"],
                   duration_optimized_s=climb.duration_s,
                   energy_battery_prescribed_kWh=hover["battery_kWh"] + climb_prescribed["battery_kWh"],
                   energy_battery_optimized_kWh=float(numpy.trapezoid(climb.power_battery_W, climb.time_s)) / 3.6e6,
                   fuel_prescribed_kg=hover["fuel_kg"] + climb_prescribed["fuel_kg"],
                   fuel_optimized_kg=climb.mass_fuel_burnt_kg)
    return np.vstack([rows, rest]), labels, summary


def plot_profile(series, labels, title, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    time_s, velocity, turbines, battery, altitude = series.T
    fig, axes = plt.subplots(3, 2, figsize=(11, 7.5), sharex="col", gridspec_kw=dict(width_ratios=(1, 1.6)))
    for column, (scale, unit, xmax) in enumerate(((60.0, "min", duration_zoom_s), (3600.0, "h", time_s[-1]))):
        x = time_s / scale
        ax_v, ax_p, ax_h = axes[:, column]
        ax_v.plot(x, velocity / u.knot, color=blue, lw=1.8)
        ax_p.plot(x, turbines / watts_per_hp, color=orange, lw=1.8, label="turbine shaft (both engines)")
        ax_p.plot(x, battery / watts_per_hp, color=blue, lw=1.8, label="battery (+ discharge, - recharge)")
        ax_p.axhline(0, color=grey, lw=0.6)
        ax_h.plot(x, altitude / u.foot, color=blue, lw=1.8)
        ax_h.set_xlabel(f"time ({unit})")
        ax_h.set_xlim(0, xmax / scale)
        # Segment boundaries and labels (zoom: those that start inside it).
        t = 0.0
        for label, duration_s in labels:
            if t <= xmax:
                for ax in (ax_v, ax_p, ax_h):
                    ax.axvline(t / scale, color="#c9c8c2", lw=0.6, zorder=0)
                # Whole mission: label only segments long enough to read; the zoom labels the short ones.
                if (column == 0 and t < xmax * 0.9) or (column == 1 and duration_s >= 0.02 * time_s[-1]):
                    ax_v.annotate(label, (t / scale, 1.0), xycoords=("data", "axes fraction"), xytext=(2, -2),
                                  textcoords="offset points", fontsize=6.5, color=grey, va="top", rotation=90)
            t += duration_s
        for ax in (ax_v, ax_p, ax_h):
            ax.grid(alpha=0.25)
            ax.spines[["top", "right"]].set_visible(False)
        ax_v.set_ylim(0, None)
        ax_h.set_ylim(-300, None)
    axes[0, 0].set_title("first 15 min", fontsize=9, color=grey)
    axes[0, 1].set_title("whole mission", fontsize=9, color=grey)
    axes[0, 0].set_ylabel("airspeed (kt)")
    axes[1, 0].set_ylabel("power (hp)\n1,000 hp = 746 kW")
    axes[2, 0].set_ylabel("altitude (ft)")
    axes[1, 1].legend(loc="upper right", fontsize=7.5, frameon=False)
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run(sizing=None, climb=None, directory="output/mission"):
    sizing = sizing if sizing is not None else solve_halo_sizing()
    mass_lb = sizing.mass_takeoff_kg / u.lbm
    series, labels = regular_mission(sizing)
    plot_profile(series, labels, f"Design mission of the baseline design ({mass_lb:,.0f} lb)",
                 os.path.join(directory, "mission_profile.png"))
    series, labels, summary = optimized_takeoff_mission(sizing, climb)
    plot_profile(series, labels, f"Same mission with an optimized take-off and climb ({mass_lb:,.0f} lb)",
                 os.path.join(directory, "mission_profile_optimized_takeoff.png"))
    print({k: round(v, 2) for k, v in summary.items()})
    return summary


if __name__ == "__main__":
    run()
