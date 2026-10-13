"""Empty-weight breakdown of the sized Halo baseline as a donut: powertrain groups in blue, airframe in orange.

    python -m examples.halo_oew_breakdown    # sizes the baseline; writes output/oew/oew_breakdown.png
"""
import os

import aerosandbox.tools.units as u

# (label, airframe component names, powertrain instance names); powertrain groups first.
groups = (
    ("Rotors and gearboxes", (), ("propulsor", "gearbox", "generator_gearbox")),
    ("Motors and generators", (), ("motor", "generator")),
    ("Battery", (), ("battery",)),
    ("Turboshafts, cooling,\nprotection", (), ("turboshaft", "heat_exchanger", "protection_string",
                                              "protection_bus_tie", "cable_bus_tie")),
    ("Systems and equipment", ("systems", "equipment"), ()),
    ("Wing, nacelles, tails", ("wing", "nacelles", "horizontal_tail", "vertical_tail", "drive_shaft"), ()),
    ("Fuselage", ("fuselage",), ()),
    ("Landing gear", ("landing_gear",), ()),
)
colors = ("#1d4f91", "#2a78d6", "#71a3e3", "#b3cdf0", "#f2b28f", "#eb6834", "#b84a1f", "#7a3315")


def group_masses_kg(sizing):
    """(label, mass) per group; every empty-weight item lands in exactly one group."""
    components, powertrain = dict(sizing.component_masses_kg), dict(sizing.powertrain_masses_kg)
    rows = [(label, sum(components.get(n, 0.0) for n in names_c) + sum(powertrain.get(n, 0.0) for n in names_p))
            for label, names_c, names_p in groups]
    named = {n for _, c, p in groups for n in c + p}
    missing = [n for n in list(powertrain) if n not in named]
    if missing:
        raise ValueError(f"powertrain items not grouped: {missing}")
    return rows


def plot_breakdown(sizing, path, version=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = group_masses_kg(sizing)
    total_lb = sum(m for _, m in rows) / u.lbm
    fig, ax = plt.subplots(figsize=(9.3, 6.3), facecolor="#FFFFFF")
    wedges, _ = ax.pie([m for _, m in rows], colors=colors, startangle=90, counterclock=False,
                       wedgeprops=dict(width=0.42, edgecolor="#FFFFFF", linewidth=2))
    for wedge, (label, mass_kg) in zip(wedges, rows):
        angle = numpy_deg_mid(wedge)
        x, y = cos_sin(angle)
        ax.annotate(f"{label}\n{mass_kg / u.lbm:,.0f} lb ({mass_kg / u.lbm / total_lb:.0%})".replace("%", " %"),
                    xy=(0.8 * x, 0.8 * y), xytext=(1.28 * x, 1.22 * y), fontsize=9.5, color="#222222",
                    ha="left" if x > 0 else "right", va="center",
                    arrowprops=dict(arrowstyle="-", color="#555555", lw=0.8))
    ax.text(0, 0.08, f"{total_lb:,.0f} lb", ha="center", va="center", fontsize=16, fontweight="bold")
    ax.text(0, -0.12, f"empty weight\n({total_lb * u.lbm:,.0f} kg)", ha="center", va="center", fontsize=9.5,
            color="#666666")
    name = f"{version}, " if version else ""
    ax.set_title(f"Empty weight breakdown, {name}{sizing.mass_takeoff_kg / u.lbm:,.0f} lb take-off "
                 f"(powertrain blue, airframe orange)", fontsize=11.5, pad=30)
    ax.set_aspect("equal")
    fig.tight_layout()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=110, facecolor="#FFFFFF")
    plt.close(fig)
    return rows


def numpy_deg_mid(wedge):
    return 0.5 * (wedge.theta1 + wedge.theta2)


def cos_sin(angle_deg):
    import math
    return math.cos(math.radians(angle_deg)), math.sin(math.radians(angle_deg))


def run(sizing=None, directory="output/oew", version=None):
    if sizing is None:
        from examples.halo_sizing import solve_halo_sizing
        sizing = solve_halo_sizing()
    return plot_breakdown(sizing, os.path.join(directory, "oew_breakdown.png"), version)


if __name__ == "__main__":
    run()
