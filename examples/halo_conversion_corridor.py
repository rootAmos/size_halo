"""Computed conversion corridor and trim schedule of the sized Halo reference (plan 039).

The aircraft is sized first (`solve_halo_sizing()`, the default reference); the corridor flies that numeric
aircraft at take-off mass, sea level, hover rotor speed. Each nacelle angle gets the least and greatest
level-flight trim airspeed inside `CorridorLimits`, with the limit that sets it, and a least-power trim schedule
at the corridor's mid speed.

    python examples/halo_conversion_corridor.py            # writes output/corridor/corridor.png and .json
"""
import json
import os
from dataclasses import asdict

import aerosandbox.tools.units as u

from aircraft_closure.controls.stability import LongitudinalStability
from aircraft_closure.trajectory.corridor import CorridorLimits, TrimGeometry, solve_corridor, solve_trim
from aircraft_closure.trajectory.tiltrotor import TiltrotorPointMass
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, build_halo_aerodynamics, build_halo_aircraft,
                                  solve_halo_sizing)

ratio_placard_speed = 1.1           # airplane-mode limit speed over the required maximum speed (assumed)
tilts_deg = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0)


def halo_corridor_case(sizing, requirements=HaloRequirements(), assumptions=HaloAssumptions()):
    """Model, trim geometry, limits and rotor speed for the sized aircraft."""
    r, a = requirements, assumptions
    aircraft = build_halo_aircraft(sizing.design, r, a)
    model = TiltrotorPointMass(aircraft, build_halo_aerodynamics(r, a))
    condition = StructuralDesignCondition(mass_design_kg=sizing.mass_takeoff_kg,
                                          load_factor_ultimate=a.load_factor_ultimate,
                                          velocity_cruise_m_s=sizing.velocity_cruise_m_s,
                                          altitude_cruise_m=(sizing.design.altitude_cruise_m
                                                             if sizing.design.altitude_cruise_m is not None
                                                             else r.altitude_cruise_m),
                                          lift_to_drag_cruise=sizing.lift_to_drag_cruise)
    total = aircraft.get_mass_breakdown(condition).total()
    wing = aircraft.wing
    geometry = TrimGeometry(x_cg_m=float(total.x_cg), z_cg_m=float(total.z_cg),
                            x_spindle_m=wing.x_le_root_m + 0.25 * wing.chord_root_m(), z_spindle_m=wing.z_m,
                            length_mast_m=a.length_mast_m)
    limits = CorridorLimits(velocity_placard_m_s=ratio_placard_speed * r.velocity_max_m_s)
    rotor = model.instance("propulsor")
    speed_rotor_rad_s = rotor.speed_tip_max_m_s / rotor.radius_m()           # hover rotor speed throughout
    return model, geometry, limits, speed_rotor_rad_s


def run(sizing=None, directory="output/corridor", plot=True):
    sizing = sizing if sizing is not None else solve_halo_sizing()
    model, geometry, limits, speed_rotor_rad_s = halo_corridor_case(sizing)
    stability = LongitudinalStability()
    common = dict(mass_kg=sizing.mass_takeoff_kg, altitude_m=0.0, speed_rotor_rad_s=speed_rotor_rad_s)
    corridor = solve_corridor(model, stability, geometry, limits, tilts_deg=tilts_deg,
                              velocity_max_m_s=1.05 * limits.velocity_placard_m_s, **common)
    schedule = []
    for low, high in corridor:
        if low.trim is None or high.trim is None:
            continue
        velocity_m_s = 0.5 * (low.velocity_m_s + high.velocity_m_s)
        schedule.append(solve_trim(model, stability, geometry, limits, velocity_m_s=velocity_m_s,
                                   tilt_deg=low.tilt_deg, **common))

    print(f"Halo conversion corridor, {sizing.mass_takeoff_kg:.0f} kg, sea level, "
          f"tip speed {speed_rotor_rad_s * model.instance('propulsor').radius_m():.0f} m/s")
    print(f"{'tilt':>5} {'low kt':>7}  {'set by':<34} {'high kt':>7}  set by")
    for low, high in corridor:
        text = lambda b: "-" if b.trim is None else f"{b.velocity_m_s / u.knot:7.1f}"  # noqa: E731
        print(f"{low.tilt_deg:5.0f} {text(low)}  {', '.join(low.binding):<34} {text(high)}  {', '.join(high.binding)}")
    print(f"\nMid-corridor trim schedule\n{'tilt':>5} {'kt':>6} {'pitch':>6} {'tail':>6} {'disc':>7} "
          f"{'CT/s':>6} {'mu_e':>5} {'kW/rotor':>9}")
    for t in schedule:
        print(f"{t.tilt_deg:5.0f} {t.velocity_m_s / u.knot:6.1f} {t.pitch_deg:6.1f} {t.deflection_tail_deg:6.1f} "
              f"{t.tilt_disc_deg:7.1f} {t.blade_loading:6.3f} {t.advance_ratio_edgewise:5.2f} "
              f"{t.power_shaft_rotor_W / 1e3:9.0f}")

    os.makedirs(directory, exist_ok=True)
    record = dict(mass_kg=sizing.mass_takeoff_kg, speed_rotor_rad_s=speed_rotor_rad_s, limits=asdict(limits),
                  geometry=asdict(geometry),
                  corridor=[dict(tilt_deg=b.tilt_deg, side=b.side, binding=list(b.binding),
                                 trim=None if b.trim is None else asdict(b.trim)) for pair in corridor for b in pair],
                  schedule=[asdict(t) for t in schedule])
    with open(os.path.join(directory, "corridor.json"), "w") as file:
        json.dump(record, file, indent=1)
    if plot:
        plot_corridor(corridor, sizing, model, os.path.join(directory, "corridor.png"))
    return corridor, schedule


def plot_corridor(corridor, sizing, model, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pairs = [(lo, hi) for lo, hi in corridor if lo.trim is not None and hi.trim is not None]
    tilt = [lo.tilt_deg for lo, _ in pairs]
    low = [lo.velocity_m_s / u.knot for lo, _ in pairs]
    high = [hi.velocity_m_s / u.knot for _, hi in pairs]
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.fill_betweenx(tilt, low, high, color="#9ecae1", alpha=0.6, label="Computed corridor (trim exists)")
    ax.plot(low, tilt, "o-", color="#08519c")
    ax.plot(high, tilt, "o-", color="#08519c")
    for lo, hi in pairs:
        for bound, align in ((lo, "right"), (hi, "left")):
            names = [n for n in bound.binding if n not in ("hover",)]
            if names:
                offset = -3 if align == "right" else 3
                ax.annotate(", ".join(n.replace("_", " ") for n in names), (bound.velocity_m_s / u.knot, bound.tilt_deg),
                            xytext=(offset, 4), textcoords="offset points", ha=align, fontsize=7, color="#333333")
    ax.set_xlabel("True airspeed (kt)")
    ax.set_ylabel("Nacelle angle (deg, 90 = hover)")
    ax.set_title(f"Halo conversion corridor: level flight, {sizing.mass_takeoff_kg:.0f} kg, sea level")
    ax.set_xlim(left=0)
    ax.set_ylim(-3, 95)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    run()
