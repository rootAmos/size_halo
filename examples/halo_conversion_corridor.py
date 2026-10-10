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


def plot_trim_free_body(ax=None, path=None, alpha_deg=6.0, gamma_deg=6.0, tilt_deg=60.0, tilt_disc_deg=7.0,
                        deflection_tail_deg=-10.0):
    """Side-view free-body diagram of a tiltrotor in conversion (illustrative angles; returns the axes).

    Drawn for a general flight path so the three angles are distinct: flight-path angle gamma (horizon to the
    velocity), angle of attack alpha (velocity to the body x-axis) and pitch attitude Theta = alpha + gamma (horizon to
    the body x-axis). The corridor trims level flight, gamma = 0, so there pitch = angle of attack (`pitch_deg` in
    `TrimPoint`). Nacelle angle tau (body x-axis to the shaft, 90 = hover), disc tilt theta_d (shaft to the thrust,
    the tip-path-plane normal), ruddervator delta. Forces in wind axes: net thrust at the hub, wing lift (normal to V)
    and drag (along -V), tail lift, weight; the moment balance is about the CG. At the hub the velocity splits into
    the axial component V cos(alpha + tau) (the rotor model's inflow) and the edgewise one V sin(alpha + tau)
    (the edgewise advance-ratio limit)."""
    import numpy as onp
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, FancyArrowPatch, Polygon

    own = ax is None
    if own:
        matplotlib.use("Agg")
        fig, ax = plt.subplots(figsize=(12, 9.5))
    pitch_deg = alpha_deg + gamma_deg
    a = onp.radians(pitch_deg)
    ink, blue, red, green, orange, grey, purple = ("#222222", "#1f6fb4", "#c0392b", "#2e8b57", "#d35400", "#8a8a8a",
                                                   "#7d3c98")

    def world(x, z):                                   # body (forward, up) -> world (horizon, up)
        return onp.array([x * onp.cos(a) - z * onp.sin(a), x * onp.sin(a) + z * onp.cos(a)])

    def unit(angle_deg):                               # world direction at angle_deg above the horizon
        return onp.array([onp.cos(onp.radians(angle_deg)), onp.sin(onp.radians(angle_deg))])

    def arrow(start, vector, color, label=None, offset=(0.0, 0.0), lw=2.4, ha="center", size=13, style="-|>"):
        ax.add_patch(FancyArrowPatch(start, start + vector, arrowstyle=style, mutation_scale=18, color=color, lw=lw,
                                     zorder=6))
        if label:
            tip = start + vector
            ax.text(tip[0] + offset[0], tip[1] + offset[1], label, color=color, fontsize=size, ha=ha, va="center",
                    zorder=7)

    def angle_arc(center, radius, start_deg, end_deg, color, label=None, label_at=None, size=14, lw=1.5):
        ax.add_patch(Arc(center, 2 * radius, 2 * radius, theta1=min(start_deg, end_deg),
                         theta2=max(start_deg, end_deg), color=color, lw=lw, zorder=4))
        if label:
            mid = 0.5 * (start_deg + end_deg)
            position = center + (label_at if label_at is not None else 1.22 * radius) * unit(mid)
            ax.text(*position, label, color=color, fontsize=size, ha="center", va="center", zorder=7)

    def line(p, q, color=grey, lw=1.0, ls="--", zorder=2):
        ax.plot([p[0], q[0]], [p[1], q[1]], color=color, lw=lw, ls=ls, zorder=zorder)

    # Airframe in body axes: fuselage with tail boom, wing section at the spindle, V-tail with a deflected ruddervator.
    outline = ((5.3, 0.05), (5.0, 0.5), (4.3, 0.85), (2.6, 1.05), (-2.4, 1.0), (-4.6, 0.8), (-6.6, 1.0),
               (-7.3, 1.05), (-7.2, 0.7), (-4.6, -0.15), (-2.4, -0.8), (2.6, -0.88), (4.4, -0.62), (5.1, -0.25))
    ax.add_patch(Polygon([world(*p) for p in outline], closed=True, facecolor="#e9eff6", edgecolor=ink, lw=1.3,
                         zorder=3))
    ax.add_patch(Polygon([world(*p) for p in ((4.1, 0.55), (3.3, 0.9), (2.3, 0.95), (2.3, 0.45))], closed=True,
                         facecolor="#cfd9e4", edgecolor=ink, lw=0.8, zorder=3))           # canopy
    spindle, cg = world(0.2, 1.1), world(-0.45, 0.1)
    airfoil = [world(0.2 + 1.2 * x, 1.1 + z) for x, z in ((1.0, 0.0), (0.75, 0.12), (0.2, 0.2), (-0.4, 0.18),
                                                           (-1.0, 0.04), (-1.0, -0.02), (-0.4, -0.06), (0.4, -0.06))]
    ax.add_patch(Polygon(airfoil, closed=True, facecolor="#b9c8d8", edgecolor=ink, lw=1.0, zorder=5))
    tail = [world(*p) for p in ((-6.1, 1.0), (-6.5, 2.2), (-7.0, 2.25), (-7.15, 1.05))]
    ax.add_patch(Polygon(tail, closed=True, facecolor="#cfd9e4", edgecolor=ink, lw=1.0, zorder=3))
    hinge = world(-7.1, 1.1)
    line(hinge, hinge + 1.0 * unit(pitch_deg + 180), grey, lw=1.0, ls=":")
    tab_end = hinge + 1.0 * unit(pitch_deg + 180 - deflection_tail_deg)
    line(hinge, tab_end, orange, lw=3.5, ls="-", zorder=5)

    # Reference directions at the CG: horizon, velocity (gamma), body x (alpha, pitch).
    line(cg + onp.array([-9.0, 0.0]), cg + onp.array([7.5, 0.0]), grey, lw=1.0, ls="--")
    ax.text(*(cg + onp.array([7.6, 0.0])), "horizon", color=grey, fontsize=10, va="center")
    line(cg, cg + 7.0 * unit(gamma_deg), ink, lw=1.0, ls=":")
    line(world(-8.0, 0.1) - world(0, 0.1) + cg, cg + 7.6 * unit(pitch_deg), grey, lw=1.0, ls="-.")
    ax.text(*(cg + 7.7 * unit(pitch_deg)), "body $x$", color=grey, fontsize=10, va="center")
    arrow(cg + 7.0 * unit(gamma_deg), 1.6 * unit(gamma_deg), ink, r"$V$", offset=(0.35, 0.05), size=15)
    angle_arc(cg, 5.4, 0.0, gamma_deg, purple, r"$\gamma$", label_at=5.75)
    angle_arc(cg, 6.4, gamma_deg, pitch_deg, blue, r"$\alpha$", label_at=6.8)
    angle_arc(cg, 4.4, 0.0, pitch_deg, ink, lw=1.0)
    ax.text(*(cg + 4.15 * unit(-6.0)), r"pitch $\Theta=\alpha+\gamma$", color=ink, fontsize=12, va="center")

    # Nacelle about the spindle, hub a mast length out along the shaft; tip-path plane normal to the thrust.
    mast = 2.6
    shaft_deg, thrust_deg = pitch_deg + tilt_deg, pitch_deg + tilt_deg + tilt_disc_deg
    s, n = unit(shaft_deg), unit(shaft_deg + 90)
    hub = spindle + mast * s
    nacelle = [spindle - 0.6 * s + 0.42 * n, hub - 0.25 * s + 0.42 * n, hub + 0.35 * s + 0.15 * n,
               hub + 0.35 * s - 0.15 * n, hub - 0.25 * s - 0.42 * n, spindle - 0.6 * s - 0.42 * n]
    ax.add_patch(Polygon(nacelle, closed=True, facecolor="#dfe6ee", edgecolor=ink, lw=1.2, zorder=5))
    line(spindle, spindle + (mast + 3.4) * s, grey, lw=1.0, ls="-.")
    ax.text(*(spindle + (mast + 3.5) * s + 0.25 * unit(shaft_deg - 90)), "shaft", color=grey, fontsize=10)
    line(spindle, spindle + 2.6 * unit(pitch_deg), grey, lw=0.8, ls=":")
    angle_arc(spindle, 1.6, pitch_deg, shaft_deg, green, r"$\tau$", label_at=1.95)
    disc_dir = unit(thrust_deg + 90)
    line(hub + 3.3 * disc_dir, hub - 3.3 * disc_dir, ink, lw=3.0, ls="-", zorder=5)
    line(hub + 3.3 * n, hub - 3.3 * n, grey, lw=0.8, ls=":")
    ax.text(*(hub - 3.4 * disc_dir + onp.array([0.15, -0.25])), "tip-path plane", fontsize=10, ha="left")
    angle_arc(hub, 3.3, shaft_deg, thrust_deg, red, r"$\theta_d$", label_at=3.75)

    # Inset: the velocity at the rotor, split into the axial component (the rotor model's inflow) and the edgewise
    # component in the disc plane (the edgewise advance-ratio limit).
    origin = onp.array([7.0, 3.6])
    scale = 2.8
    v_axial = onp.cos(onp.radians(alpha_deg + tilt_deg))
    v_edge = onp.sin(onp.radians(alpha_deg + tilt_deg))
    ax.text(*(origin + onp.array([-1.0, 4.0])), "the same V at the hub, drawn in the rotor's axes\n(shaft horizontal)",
            fontsize=11, color=purple, va="bottom")
    # The flight velocity copied to the hub (dotted), with a leader to the enlarged inset.
    ax.add_patch(FancyArrowPatch(hub, hub + 1.6 * unit(gamma_deg), arrowstyle="-|>", mutation_scale=14, color=purple,
                                 lw=1.4, ls=(0, (2, 2)), zorder=6))
    ax.annotate("", xy=origin + onp.array([-0.5, 0.0]), xytext=hub + 1.8 * unit(gamma_deg),
                arrowprops=dict(arrowstyle="->", color=purple, lw=0.8, ls="--", connectionstyle="arc3,rad=-0.2"))
    # Drawn in the rotor's own axes: the shaft is the horizontal reference axis, the disc plane the vertical one, and
    # V sits above the shaft axis at alpha + tau, measured from the axis (the edgewise sense is shown as positive).
    axis_x, axis_y = onp.array([1.0, 0.0]), onp.array([0.0, 1.0])
    line(origin - 0.4 * axis_x, origin + 3.6 * axis_x, grey, lw=1.2, ls="-")
    ax.text(*(origin + 3.7 * axis_x), "shaft\n(axial)", color=grey, fontsize=9, va="center")
    line(origin - 0.4 * axis_y, origin + 3.4 * axis_y, grey, lw=1.2, ls="-")
    ax.text(*(origin + 3.5 * axis_y), "disc plane (edgewise)", color=grey, fontsize=9, ha="center")
    v_end = origin + scale * (v_axial * axis_x + v_edge * axis_y)
    arrow(origin, v_end - origin, ink, r"$V$", offset=(0.25, 0.2), lw=2.0, size=14)
    arrow(origin, scale * v_axial * axis_x, purple, r"$V\cos(\alpha+\tau)$", offset=(0.0, -0.4), size=11, lw=1.8)
    arrow(origin, scale * v_edge * axis_y, purple, r"$V\sin(\alpha+\tau)$", offset=(-0.15, 0.0), ha="right",
          size=11, lw=1.8)
    line(origin + scale * v_axial * axis_x, v_end, purple, lw=0.8, ls=":")
    line(origin + scale * v_edge * axis_y, v_end, purple, lw=0.8, ls=":")
    angle_v_deg = alpha_deg + tilt_deg
    angle_arc(origin, 1.0, 0.0, angle_v_deg, ink, r"$\alpha+\tau$", label_at=1.45, size=11, lw=1.0)

    # Forces (wind axes for L, D; vertical W; thrust along the tip-path normal).
    arrow(hub, 4.8 * unit(thrust_deg), red, r"$nT(1-f_{dl})$", offset=(0.25, 0.3), ha="left", size=14)
    line(hub, hub + 2.0 * unit(gamma_deg), ink, lw=0.8, ls=":")
    angle_arc(hub, 1.7, gamma_deg, thrust_deg, red, lw=1.0)
    ax.text(*(hub + 1.9 * unit(thrust_deg) + onp.array([-0.2, 0.0])), r"$\alpha+\tau+\theta_d$", color=red,
            fontsize=11, ha="right", va="center")
    wing_ac = world(-0.75, 1.3)
    arrow(wing_ac, 3.4 * unit(gamma_deg + 90), blue, r"$L$  ($\perp V$)", offset=(0.0, 0.35))
    arrow(wing_ac, 2.8 * unit(gamma_deg + 180), blue, r"$D$  ($\parallel -V$)", offset=(-0.2, 0.35), ha="right")
    arrow(cg, onp.array([0.0, -3.6]), ink, r"$W$", offset=(0.0, -0.4), size=15)
    tail_ac = world(-6.6, 1.6)
    arrow(tail_ac, 1.9 * unit(gamma_deg - 90), orange, r"$L_{tail}(\delta)$", offset=(0.0, -0.45))
    ax.text(*(tab_end + onp.array([-0.15, 0.3])), r"$\delta$", color=orange, fontsize=15, ha="right")
    ax.add_patch(Arc(cg, 1.7, 1.7, theta1=60, theta2=330, color=ink, lw=1.5, zorder=6))
    ax.add_patch(FancyArrowPatch(cg + 0.85 * unit(52), cg + 0.85 * unit(75), arrowstyle="-|>", mutation_scale=13,
                                 color=ink, lw=1.5, zorder=6))
    ax.text(*(cg + onp.array([-0.6, -2.1])), r"$M_y$ about CG (nose-up +)", fontsize=11, ha="right")
    for point, name, dx, dy in ((cg, "CG", -0.35, -0.45), (spindle, "spindle", 0.35, -0.4), (hub, "hub", 0.35, -0.5)):
        ax.plot(*point, "o", color="white", mec=ink, ms=7, zorder=8)
        ax.text(point[0] + dx, point[1] + dy, name, fontsize=10, ha="right" if dx < 0 else "left", zorder=8)

    # Equations and limits.
    lines = [
        (r"Trim: three equations, four unknowns ($T$, $\alpha$, $\delta$, $\theta_d$); the spare freedom is spent on least rotor power", ink),
        (r"$\parallel V$:  $nT(1-f_{dl})\cos(\alpha+\tau+\theta_d) - D - W\sin\gamma = 0$", ink),
        (r"$\perp V$:  $nT(1-f_{dl})\sin(\alpha+\tau+\theta_d) + L + L_{tail}(\delta) - W\cos\gamma = 0$", ink),
        (r"$M_y$:  $M_{aero}(\alpha,\delta) + nT(1-f_{dl})\,[x_{hub}\sin(\tau+\theta_d) - z_{hub}\cos(\tau+\theta_d)] = 0$"
         r"   ($x_{hub}, z_{hub}$: hub ahead of / above the CG)", ink),
        (r"Corridor: level flight, $\gamma = 0$, so pitch $\Theta$ = angle of attack $\alpha$", purple),
        (r"Limits:  $-5^\circ \leq \alpha \leq 12^\circ$,   $|\delta| \leq 25^\circ$,   $|\theta_d| \leq 10^\circ\,\sin\tau$,   "
         r"$\mu_e = V|\sin(\alpha+\tau)|/\Omega R \leq 0.28$,   $P_{rotor} \leq$ drive rating,", ink),
        (r"            $C_T/\sigma$ max,   $\alpha \leq \alpha_{stall}$ (unblown wing),   $V\cos(\alpha+\tau+\theta_d) \geq 0$,   placard speed", ink),
    ]
    for k, (text, color) in enumerate(lines):
        ax.text(-9.3, -5.0 - 0.62 * k, text, fontsize=11.5, color=color, va="top")
    ax.set_title("Tiltrotor in conversion: angles, forces and moments of the corridor trim (illustrative angles)",
                 fontsize=14)
    ax.set_xlim(-9.5, 12.5)
    ax.set_ylim(-9.6, 10.0)
    ax.set_aspect("equal")
    ax.axis("off")
    if own:
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
    return ax


if __name__ == "__main__":
    run()
