"""Computed conversion corridor and trim schedule of the sized Halo reference (plan 039).

The aircraft is sized first (`solve_halo_sizing()`, the default reference); the corridor flies that numeric
aircraft at take-off mass, sea level, hover rotor speed. Each nacelle angle gets the least and greatest
level-flight trim airspeed inside `CorridorLimits`, with the limit that sets it, and a least-power trim schedule
at the corridor's mid speed.

    python examples/halo_conversion_corridor.py            # writes output/corridor/corridor.png, .json and
                                                           # corridor_power.png (power required across the corridor)
"""
import json
import os
from dataclasses import asdict

import aerosandbox.tools.units as u

from aircraft_closure.controls.stability import LongitudinalStability
from aircraft_closure.trajectory.corridor import (CorridorLimits, TrimGeometry, solve_corridor, solve_power_map,
                                                  solve_trim)
from aircraft_closure.trajectory.tiltrotor import TiltrotorPointMass
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, build_halo_aerodynamics, build_halo_aircraft,
                                  solve_halo_sizing)

ratio_placard_speed = 1.1           # airplane-mode limit speed over the required maximum speed (assumed)
tilts_deg = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0)
tilts_map_deg = tuple(7.5 * k for k in range(13))   # finer nacelle-angle grid for the power map
# Plot labels for the limit names in `CorridorLimits` margins; others print with underscores as spaces.
labels_limit = {"disc_tilt": "rotor pitch limit", "pitch_max": "attitude max", "pitch_min": "attitude min"}


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


def run(sizing=None, directory="output/corridor", plot=True, requirements=HaloRequirements(),
        assumptions=HaloAssumptions(), power_map=True):
    sizing = sizing if sizing is not None else solve_halo_sizing(requirements, assumptions)
    model, geometry, limits, speed_rotor_rad_s = halo_corridor_case(sizing, requirements, assumptions)
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
    if power_map:
        fine = solve_corridor(model, stability, geometry, limits, tilts_deg=tilts_map_deg,
                              velocity_max_m_s=1.05 * limits.velocity_placard_m_s, **common)
        powers = solve_power_map(model, stability, geometry, limits, fine, **common)
        if plot:
            plot_power_map(powers, fine, sizing, model, os.path.join(directory, "corridor_power.png"))
    return corridor, schedule


def plot_power_map(power_map, corridor, sizing, model, path, ax=None):
    """Total rotor shaft power (both rotors) of the least-power trim across the corridor, as filled contours with
    labelled lines, the corridor bounds and the limit that sets each bound. Returns the axes."""
    import numpy as onp
    import matplotlib
    import matplotlib.pyplot as plt
    count_rotors = model.count("propulsor")
    velocity_kt = onp.array(power_map.velocities_m_s) / u.knot
    tilt = onp.repeat(onp.array(power_map.tilts_deg)[:, None], velocity_kt.shape[1], axis=1)
    power_kW = count_rotors * onp.array(power_map.values("power_shaft_rotor_W")) / 1e3
    rating_kW = count_rotors * float(model.instance("propulsor").max_shaft_power_W) / 1e3
    own = ax is None
    if own:
        matplotlib.use("Agg")
        fig, ax = plt.subplots(figsize=(9, 6))
    step_kW = 50.0
    levels = onp.arange(onp.floor(onp.nanmin(power_kW) / step_kW) * step_kW,
                        onp.nanmax(power_kW) + step_kW, step_kW)
    filled = ax.contourf(velocity_kt, tilt, onp.ma.masked_invalid(power_kW), levels=levels, cmap="viridis")
    lines = ax.contour(velocity_kt, tilt, onp.ma.masked_invalid(power_kW),
                       levels=[level for level in levels if level % (2 * step_kW) == 0], colors="white",
                       linewidths=0.7)
    ax.clabel(lines, fmt="%.0f", fontsize=7)
    ax.figure.colorbar(filled, ax=ax, label=f"Rotor shaft power, both rotors (kW); drive rating {rating_kW:.0f} kW")
    pairs = [(lo, hi) for lo, hi in corridor if lo.trim is not None and hi.trim is not None]
    for side in (0, 1):
        ax.plot([p[side].velocity_m_s / u.knot for p in pairs], [p[side].tilt_deg for p in pairs], "-",
                color="black", lw=1.5)
    labels_map = {"disc_tilt": "disc tilt", "pitch_max": "attitude max", "pitch_min": "attitude min",
                  "edgewise_advance_ratio": "edgewise mu", "rotor_power": "rotor power"}
    for lo, hi in pairs[::2]:
        for bound, align in ((lo, "right"), (hi, "left")):
            names = [n for n in bound.binding if n not in ("hover",)]
            if names:
                ax.annotate(", ".join(labels_map.get(n, n.replace("_", " ")) for n in names),
                            (bound.velocity_m_s / u.knot, bound.tilt_deg), xytext=(-3 if align == "right" else 3, 3),
                            textcoords="offset points", ha=align, fontsize=7, color="#333333")
    index = onp.unravel_index(onp.nanargmin(power_kW), power_kW.shape)
    ax.plot(velocity_kt[index], tilt[index], "w*", ms=10)
    ax.annotate(f"least power {power_kW[index]:.0f} kW", (velocity_kt[index], tilt[index]), xytext=(5, -10),
                textcoords="offset points", color="white", fontsize=8)
    ax.set_xlabel("True airspeed (kt)")
    ax.set_ylabel("Nacelle angle (deg, 90 = hover)")
    ax.set_title(f"Power required across the conversion corridor: level flight, {sizing.mass_takeoff_kg:.0f} kg, "
                 f"sea level")
    ax.set_xlim(0, onp.nanmax(velocity_kt) + 45)
    ax.set_ylim(-3, 95)
    if own:
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
    return ax


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
                ax.annotate(", ".join(labels_limit.get(n, n.replace("_", " ")) for n in names),
                            (bound.velocity_m_s / u.knot, bound.tilt_deg),
                            xytext=(offset, 4), textcoords="offset points", ha=align, fontsize=7, color="#333333")
    ax.set_xlabel("True airspeed (kt)")
    ax.set_ylabel("Nacelle angle (deg, 90 = hover)")
    ax.set_title(f"Halo conversion corridor: level flight, {sizing.mass_takeoff_kg:.0f} kg, sea level")
    ax.set_xlim(left=0)
    ax.set_ylim(-3, 95)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_trim_free_body(ax=None, path=None, pitch_deg=8.0, tilt_deg=60.0, tilt_disc_deg=7.0):
    """Side-view free-body diagram of the level-flight trim that defines the corridor (illustrative angles).

    Angles: pitch alpha (body x-axis from the velocity; level flight, so attitude = angle of attack), nacelle angle
    tau (shaft from the body x-axis, 90 = hover), disc tilt theta (tip-path-plane normal, i.e. the thrust, from the
    shaft). Forces: net thrust n T (1 - f_dl) at the hub, wing lift L and drag D, tail lift from the ruddervator
    L_tail(delta), weight W at the CG; the moment balance is about the CG. Returns the axes."""
    import numpy as onp
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, FancyArrowPatch, Polygon

    own = ax is None
    if own:
        matplotlib.use("Agg")
        fig, ax = plt.subplots(figsize=(11, 8))
    a = onp.radians(pitch_deg)

    def world(x, z):                                   # body (forward, up) -> world (flight direction, up)
        return onp.array([x * onp.cos(a) - z * onp.sin(a), x * onp.sin(a) + z * onp.cos(a)])

    def unit(angle_deg):                               # world direction at angle_deg above the velocity
        return onp.array([onp.cos(onp.radians(angle_deg)), onp.sin(onp.radians(angle_deg))])

    def arrow(start, vector, color, label, offset=(0, 0), lw=2.2, ha="center"):
        ax.add_patch(FancyArrowPatch(start, start + vector, arrowstyle="-|>", mutation_scale=16, color=color, lw=lw))
        tip = start + vector
        ax.text(tip[0] + offset[0], tip[1] + offset[1], label, color=color, fontsize=11, ha=ha, va="center")

    def angle_arc(center, radius, start_deg, end_deg, label, color, label_radius=1.25):
        ax.add_patch(Arc(center, 2 * radius, 2 * radius, theta1=min(start_deg, end_deg),
                         theta2=max(start_deg, end_deg), color=color, lw=1.4))
        mid = onp.radians(0.5 * (start_deg + end_deg))
        ax.text(center[0] + label_radius * radius * onp.cos(mid), center[1] + label_radius * radius * onp.sin(mid),
                label, color=color, fontsize=12, ha="center", va="center")

    ink, blue, red, green, orange, grey = "#222222", "#1f6fb4", "#c0392b", "#2e8b57", "#e67e22", "#888888"
    # Airframe (body axes): fuselage, wing section at the spindle, V-tail with the ruddervator deflected.
    fuselage = [world(x, z) for x, z in ((4.2, 0.0), (3.4, 0.75), (-4.5, 0.6), (-6.6, 0.9), (-6.9, 0.9), (-6.4, -0.1),
                                         (-4.5, -0.55), (3.4, -0.75))]
    ax.add_patch(Polygon(fuselage, closed=True, facecolor="#e8eef5", edgecolor=ink, lw=1.2))
    spindle, cg = world(0.0, 1.0), world(-0.35, 0.0)
    wing = [world(x, z) for x, z in ((0.9, 1.0), (0.0, 1.12), (-1.1, 1.0), (0.0, 0.94))]
    ax.add_patch(Polygon(wing, closed=True, facecolor="#c9d6e3", edgecolor=ink, lw=1.0))
    hinge, deflection_deg = world(-6.75, 0.95), -12.0
    elevator_end = hinge + 0.9 * unit(pitch_deg + 180 + deflection_deg)
    ax.plot([hinge[0], elevator_end[0]], [hinge[1], elevator_end[1]], color=orange, lw=3)
    # Body x-axis and the velocity (level flight: gamma = 0).
    ax.plot(*zip(world(-8.2, 0.0), world(6.0, 0.0)), color=grey, lw=1, ls="--")
    ax.text(*world(6.1, 0.0), "body x", color=grey, fontsize=9, va="center")
    origin = onp.array([5.0, -2.8])
    arrow(origin, onp.array([2.4, 0.0]), ink, "V  (level: gamma = 0)", offset=(0.15, 0), ha="left")
    ax.plot([cg[0], cg[0] + 5.3], [cg[1], cg[1]], color=ink, lw=0.8, ls=":")
    angle_arc(cg, 5.0, 0.0, pitch_deg, "", blue)
    ax.text(cg[0] + 5.25, cg[1] + 0.35, "alpha = pitch", color=blue, fontsize=12, va="center")
    # Nacelle: shaft from the spindle at tau above the body x-axis, hub a mast length out; tip-path plane at theta.
    mast = 2.4
    shaft_deg, thrust_deg = pitch_deg + tilt_deg, pitch_deg + tilt_deg + tilt_disc_deg
    hub = spindle + mast * unit(shaft_deg)
    ax.plot([spindle[0], hub[0]], [spindle[1], hub[1]], color=ink, lw=5, solid_capstyle="round")
    shaft_end = spindle + (mast + 2.8) * unit(shaft_deg)
    ax.plot([spindle[0], shaft_end[0]], [spindle[1], shaft_end[1]], color=grey, lw=1, ls="--")
    ax.text(*(shaft_end + 0.3 * unit(shaft_deg - 90)), "shaft", color=grey, fontsize=9, va="center")
    ax.plot(*zip(spindle, spindle + 3.2 * unit(pitch_deg)), color=grey, lw=0.8, ls=":")
    angle_arc(spindle, 1.35, pitch_deg, shaft_deg, "tau", green)
    disc = [hub + 3.0 * unit(thrust_deg + 90), hub - 3.0 * unit(thrust_deg + 90)]
    ax.plot(*zip(*disc), color=ink, lw=2.5)
    ax.text(*(disc[1] + onp.array([0.2, -0.1])), "tip-path plane", fontsize=9, ha="left", va="top")
    angle_arc(hub, 3.0, shaft_deg, thrust_deg, "", red)
    ax.text(*(hub + 3.0 * unit(thrust_deg) + 0.5 * unit(thrust_deg + 90)), "theta", color=red, fontsize=12,
            ha="right", va="center")
    # Forces.
    arrow(hub, 4.4 * unit(thrust_deg), red, "n T (1 - f_dl)", offset=(0.3, 0.25), ha="left")
    arrow(spindle + onp.array([-2.6, 0.3]), onp.array([0.0, 3.0]), blue, "L  (normal to V)", offset=(0.0, 0.35))
    arrow(spindle - onp.array([1.3, -0.1]), onp.array([-2.4, 0.0]), blue, "D", offset=(-0.35, 0))
    arrow(cg, onp.array([0.0, -3.4]), ink, "W", offset=(0.0, -0.35))
    tail_point = world(-6.75, 0.95)
    arrow(tail_point, onp.array([0.0, -1.9]), orange, "L_tail(delta)", offset=(0.0, -0.35))
    ax.text(*(elevator_end + onp.array([-0.15, 0.35])), "delta", color=orange, fontsize=12, ha="right")
    # Moment about the CG and the reference points.
    ax.add_patch(Arc(cg, 1.6, 1.6, theta1=40, theta2=320, color=ink, lw=1.4))
    ax.add_patch(FancyArrowPatch(cg + 0.8 * unit(305), cg + 0.8 * unit(325), arrowstyle="-|>", mutation_scale=12,
                                 color=ink, lw=1.4))
    ax.text(*(cg + onp.array([0.6, -1.6])), "M_y about CG", fontsize=10, ha="left")
    for point, name, dx in ((cg, "CG", -0.35), (spindle, "spindle", 0.3), (hub, "hub", 0.3)):
        ax.plot(*point, "o", color="white", mec=ink, ms=7, zorder=5)
        ax.text(point[0] + dx, point[1] - 0.35, name, fontsize=9, ha="right" if dx < 0 else "left")
    # Equations and limits.
    text = ("Trim: 3 equations, 4 unknowns (T, alpha, delta, theta); the spare one is spent on least rotor power\n"
            "  F_x:  n T (1 - f_dl) cos(alpha + tau + theta) - D = 0\n"
            "  F_z:  n T (1 - f_dl) sin(alpha + tau + theta) + L + L_tail(delta) - W = 0\n"
            "  M_y:  M_aero(alpha, delta) + n T (1 - f_dl) (x_hub sin(tau + theta) - z_hub cos(tau + theta)) = 0\n"
            "        (nose-up positive; x_hub, z_hub: hub forward of and above the CG, a mast length out along the shaft)\n"
            "Limits that bound the corridor:\n"
            "  -5 <= alpha <= 12 deg      |delta| <= 25 deg      |theta| <= 10 deg x sin(tau)\n"
            "  edgewise mu = V |sin(alpha + tau)| / (Omega R) <= 0.28      rotor power <= drive rating      C_T / sigma\n"
            "  alpha <= alpha_stall (unblown wing)      V cos(alpha + tau + theta) >= 0      placard speed")
    ax.text(-8.8, -4.6, text, fontsize=9, family="monospace", va="top")
    ax.set_title(f"Level-flight trim that defines the conversion corridor (illustrative: alpha {pitch_deg:.0f} deg, "
                 f"tau {tilt_deg:.0f} deg, theta {tilt_disc_deg:.0f} deg)", fontsize=12)
    ax.set_xlim(-9, 11)
    ax.set_ylim(-9.0, 9.0)
    ax.set_aspect("equal")
    ax.axis("off")
    if own:
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
    return ax


if __name__ == "__main__":
    run()
