"""Plan 044 figure: take-off weight and cost per mission against battery energy (engine-out wall marked), and
the battery share of power by mission segment for each objective. Reads the study summaries written by the
plan 044 runner (output/battery/<case>.json); run from the repository root."""
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

folder = sys.argv[1] if len(sys.argv) > 1 else os.path.join("output", "battery")
figure_path = os.path.join("docs", "figures", "battery_energy_objectives.png")
kg_per_lb = 0.45359237
colors = dict(mass_takeoff="#2a78d6", cost="#eb6834", fuel="#1baf7a", payload="#eda100", range="#6250d6")
names = dict(mass_takeoff="Min. take-off weight", cost="Min. cost per mission", fuel="Min. fuel",
             payload="Max. payload", range="Max. range")
ink, muted, grid = "#0b0b0b", "#52514e", "#e4e3df"
# The 4,000 ft OGE hover requirement point, then the mission segments.
segment_labels = ("hover", "take-off hover", "climb", "cruise", "reserve loiter", "descent", "landing hover")
segment_short = ("4,000 ft hover\n(requirement)", "Take-off\nhover", "Climb", "Cruise", "Loiter", "Descent",
                 "Landing\nhover")


def load(name):
    path = os.path.join(folder, f"{name}.json")
    return json.load(open(path)) if os.path.exists(path) else None


def sweep(prefix):
    points = [load(f"{prefix}_free")] + [load(f"{prefix}_np{k:.2f}") for k in (1.25, 1.5, 1.75, 2.0)]
    return sorted((p for p in points if p is not None), key=lambda p: p["energy_battery_kWh"])


def style(ax):
    ax.grid(True, color=grid, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(muted)
    ax.tick_params(colors=muted, labelsize=10)


def battery_shares(result):
    """Battery share of motor power (%) at the requirement hover and over each mission segment."""
    shares = {s["label"]: s["hybridization_mean"] for s in result["segments"]}
    shares["hover"] = next(h["hybridization"] for h in result["hovers"] if h["label"] == "hover")
    return [100 * shares[label] for label in segment_labels]


def main():
    plt.rcParams.update({"font.size": 11, "axes.labelcolor": ink, "axes.titlesize": 12})
    fig = plt.figure(figsize=(12.8, 8.0), facecolor="white")
    grid_spec = fig.add_gridspec(2, 2, height_ratios=(1.15, 1.0), hspace=0.45, wspace=0.22,
                                 left=0.07, right=0.98, top=0.89, bottom=0.08)
    ax_mass, ax_cost = fig.add_subplot(grid_spec[0, 0]), fig.add_subplot(grid_spec[0, 1])
    ax_share = fig.add_subplot(grid_spec[1, :])
    free = {objective: load(f"{'mass' if objective == 'mass_takeoff' else objective}_free") for objective in colors}
    # Each free optimum sits on its own engine-out wall: the wall moves with the weight the objective chooses.
    energy_wall_kWh = free["mass_takeoff"]["energy_battery_kWh"]
    for ax, key, scale, label in ((ax_mass, "mass_takeoff_kg", 1 / kg_per_lb, "Take-off weight (lb)"),
                                  (ax_cost, "cost_per_mission_usd", 1.0, "Cost per mission (USD)")):
        style(ax)
        # Cost along the mass sweep is left out: the mass objective is nearly indifferent to cruise speed, so the
        # cost of its points scatters with the speed it happens to pick.
        for objective, prefix in (("mass_takeoff", "mass"), ("cost", "cost"))[(1 if ax is ax_cost else 0):]:
            points = sweep(prefix)
            ax.plot([p["energy_battery_kWh"] for p in points], [p[key] * scale for p in points],
                    color=colors[objective], linewidth=2, marker="o", markersize=6,
                    label=f"{names[objective]}, pack fixed")
        for objective in ("mass_takeoff", "cost", "fuel"):
            p = free[objective]
            ax.plot(p["energy_battery_kWh"], p[key] * scale, marker="D", markersize=10, color=colors[objective],
                    markeredgecolor="white", markeredgewidth=1.5, linestyle="none", zorder=5,
                    label=f"{names[objective]}, pack free")
        low, high = 55.0, 145.0
        ax.axvspan(low, energy_wall_kWh, color="#f0efec", zorder=0)
        ax.axvline(energy_wall_kWh, color=muted, linewidth=1.2, linestyle="--")
        ax.set_xlim(low, high)
        bottom, top = ax.get_ylim()
        ax.text(energy_wall_kWh - 1.0, bottom + 0.03 * (top - bottom), "infeasible:\nengine-out\nhover, cells\nhit 2.5 V",
                ha="right", va="bottom", color=muted, fontsize=9)
        ax.set_xlabel("Battery energy (kWh)")
        ax.set_ylabel(label)
    ax_mass.set_title("Take-off weight: about +23 lb per kWh above the wall", loc="left", color=ink)
    ax_cost.set_title("Cost per mission: about +USD 2 per kWh above the wall", loc="left", color=ink)
    handles, labels = ax_mass.get_legend_handles_labels()
    handles, labels = [handles[i] for i in (0, 2, 1, 3, 4)], [labels[i] for i in (0, 2, 1, 3, 4)]
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, fontsize=9.5, bbox_to_anchor=(0.5, 0.995))

    style(ax_share)
    width = 0.8 / len(colors)
    for i, objective in enumerate(colors):
        x = [j + (i - (len(colors) - 1) / 2) * width for j in range(len(segment_labels))]
        ax_share.bar(x, battery_shares(free[objective]), width=width * 0.9, color=colors[objective],
                     label=f"{names[objective]}: {free[objective]['energy_battery_kWh']:.0f} kWh, "
                           f"{free[objective]['mass_takeoff_kg'] / kg_per_lb:,.0f} lb")
    ax_share.axhline(0, color=muted, linewidth=1)
    ax_share.set_xticks(range(len(segment_labels)), segment_short)
    ax_share.set_ylabel("Battery share of\nmotor power (%)")
    ax_share.set_ylim(-30, 140)
    ax_share.set_yticks(range(-25, 101, 25))
    ax_share.set_title("Where each optimum uses the battery (pack free; negative: the generators recharge it)",
                       loc="left", color=ink)
    ax_share.legend(frameon=False, fontsize=9, ncol=2, loc="upper left")
    os.makedirs(os.path.dirname(figure_path), exist_ok=True)
    fig.savefig(figure_path, dpi=150, facecolor="white")
    print(figure_path)


if __name__ == "__main__":
    main()
