"""Plan 042 slide figure: the ε-NTU cooler against the mass-per-watt cooler on the Halo reference.

    python -m examples.ntu_heat_exchanger_figure output/ntu_default.pkl output/ntu_ntu.pkl

Reads two `HaloSizingResult` pickles (the default model and `heat_exchanger_model="ntu"`) and writes
docs/figures/ntu_heat_exchanger.png: cooler mass by item, hover fan power and airplane-mode cooling drag by point.
"""
import pickle
import sys

import matplotlib.pyplot as plt

blue, orange = "#2a78d6", "#eb6834"
orange_steps = ("#f4a582", "#eb6834", "#b8461a", "#7a2c0e")     # one hue, light to dark (mass items)
ink, ink_muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
pound_kg = 0.45359237


def points(result, mode):
    """Thermal-trace rows of one mode, one per label (a repeated label gets " (requirement)"), skipping the
    sub-intervals after the first of a subdivided segment and the failure cases other than engine-out."""
    rows, seen = [], set()
    for t in result.thermal_trace:
        label = t["label"]
        if t["mode"] != mode or label.startswith(("bus-out", "string-out")) or label.endswith(("2/3", "2/4", "3/4",
                                                                                                "4/4", "1/3")):
            continue
        label = label.replace(" 3/3", "").replace(" 1/4", "")
        if label in seen:
            label = f"{label} (requirement)"
        seen.add(label)
        rows.append(dict(t, label=label))
    return rows


def style(ax, title):
    ax.set_title(title, loc="left", color=ink, fontsize=13, fontweight="bold")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(ink_muted)
    ax.tick_params(colors=ink_muted, labelsize=10)
    ax.set_axisbelow(True)


def paired_bars(ax, labels, old, new, unit, scale):
    y = range(len(labels))
    ax.barh([i + 0.2 for i in y], [v / scale for v in old], height=0.38, color=blue, label="mass per watt")
    ax.barh([i - 0.2 for i in y], [v / scale for v in new], height=0.38, color=orange, label="ε-NTU core + duct")
    ax.set_yticks(list(y), labels, color=ink)
    ax.invert_yaxis()
    ax.axvline(0, color=ink_muted, linewidth=0.8)
    ax.grid(axis="x", color=grid, linewidth=0.8)
    ax.set_xlabel(unit, color=ink_muted)


def main(path_old, path_new, path_out="docs/figures/ntu_heat_exchanger.png", note=""):
    old, new = (pickle.load(open(p, "rb")) for p in (path_old, path_new))
    fig, axes = plt.subplots(1, 3, figsize=(16, 10), gridspec_kw=dict(width_ratios=(1.0, 1.25, 1.25)))
    fig.patch.set_facecolor("white")
    delta_lb = (new.mass_takeoff_kg - old.mass_takeoff_kg) / pound_kg
    fig.suptitle(f"ε-NTU cooler: take-off weight {new.mass_takeoff_kg / pound_kg:,.0f} lb "
                 f"({delta_lb:+,.0f} lb against {old.mass_takeoff_kg / pound_kg:,.0f} lb)",
                 x=0.02, y=0.985, ha="left", fontsize=17, fontweight="bold", color=ink)
    if note:
        fig.text(0.02, 0.925, note, ha="left", fontsize=12, color=ink_muted)

    ax = axes[0]
    style(ax, "Cooler mass (kg)")
    ax.bar(0, old.heat_exchanger["mass_kg"], width=0.6, color=blue)
    ax.text(0, old.heat_exchanger["mass_kg"] + 2, f"{old.heat_exchanger['mass_kg']:.0f}", ha="center", color=ink)
    bottom = 0.0
    names = dict(mass_core_kg="core", mass_ducts_kg="ducts", mass_fans_kg="fans", mass_pump_pipes_kg="pump, pipes")
    for color, (key, mass_kg) in zip(orange_steps, new.heat_exchanger["masses_kg"].items()):
        ax.bar(1, mass_kg, bottom=bottom, width=0.6, color=color, edgecolor="white", linewidth=2)
        ax.text(1.36, bottom + mass_kg / 2, f"{names[key]} {mass_kg:.0f}", va="center", color=ink_muted, fontsize=10)
        bottom += mass_kg
    ax.text(1, bottom + 2, f"{bottom:.0f}", ha="center", color=ink)
    hx = new.heat_exchanger
    ax.set_xticks([0, 1], ["mass per\nwatt", "ε-NTU"], color=ink)
    ax.set_xlim(-0.5, 2.3)
    ax.grid(axis="y", color=grid, linewidth=0.8)
    ax.text(0.0, -0.16, f"core {hx['width_m']:.2f} x {hx['height_m']:.2f} x {hx['length_m']:.3f} m\n"
            f"nozzle {hx['area_nozzle_m2']:.3f} m², fan rating {hx['power_rated_fan_W'] / 1e3:.1f} kW",
            transform=ax.transAxes, color=ink_muted, fontsize=10, va="top")

    hover_old, hover_new = points(old, "hover"), {t["label"]: t for t in points(new, "hover")}
    labels = [t["label"] for t in hover_old if t["label"] in hover_new]
    style(axes[1], "Hover fan power (kW)")
    paired_bars(axes[1], labels, [t["power_fan_W"] for t in hover_old if t["label"] in hover_new],
                [hover_new[label]["power_fan_W"] for label in labels], "electrical power, kW", 1e3)
    axes[1].legend(frameon=False, loc="upper right", fontsize=11)

    air_old, air_new = points(old, "airplane"), {t["label"]: t for t in points(new, "airplane")}
    labels = [t["label"] for t in air_old if t["label"] in air_new]
    style(axes[2], "Airplane-mode cooling drag (N)")
    paired_bars(axes[2], labels, [t["drag_cooling_N"] for t in air_old if t["label"] in air_new],
                [air_new[label]["drag_cooling_N"] for label in labels], "net duct drag, N (heat recovery included)", 1.0)
    fig.tight_layout(rect=(0, 0.03, 1, 0.9))
    fig.savefig(path_out, dpi=150)
    print(f"wrote {path_out}")


if __name__ == "__main__":
    # Optional: the relaxed-unit pair (two more pickles) splits the change into the cooler's continuous effect and
    # the generator's whole-unit step.
    note = ""
    if len(sys.argv) == 5:
        old_relaxed, new_relaxed = (pickle.load(open(p, "rb")) for p in sys.argv[3:5])
        units_old, units_new = (r.machine_units["generator"][0] for r in (old_relaxed, new_relaxed))
        note = (f"Continuous (relaxed machine units): {(new_relaxed.mass_takeoff_kg - old_relaxed.mass_takeoff_kg) / pound_kg:+,.0f}"
                f" lb. The rest is the generator's whole-unit step: {units_old:.3f} units rounds up to 3, "
                f"{units_new:.3f} to 2.")
    main(sys.argv[1], sys.argv[2], note=note)
