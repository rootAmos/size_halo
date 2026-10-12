"""docs/figures/ntu_validation.png: the epsilon-NTU cooler validation (plans 050 and 051) in three panels.

Left: our core against OpenConcept's stored outputs (tests/powertrain/data/compact_heat_exchanger_openconcept.json),
heat parity over the 11 cases. Middle: our model over Kellermann et al. 2021 at their 96 kW design point (plan 051
results). Right: cooling-system heat per mass against the published systems (plan 051), Halo v3.7 at its peak heat
(112 kW) and cooler mass (58 kg).

Usage: python -m examples.ntu_validation_figure [output.png]
"""
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from examples.ntu_openconcept_check import ours, stored_path

blue, orange, gray, ink, muted = "#2A6CB0", "#C8651B", "#8A929E", "#15233B", "#46505E"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.edgecolor": "#B8BCC4",
                     "axes.labelcolor": muted, "xtick.color": muted, "ytick.color": muted})

data = json.loads(Path(stored_path).read_text())
heat_oc, heat_ours, dp_oc, dp_ours = [], [], [], []
for entry in data["cases"]:
    mine = ours(entry["inputs"])
    reference = entry["openconcept"]
    heat_oc.append(reference["heat_transfer"] / 1e3)
    heat_ours.append(mine["power_heat_W"] / 1e3)
    dp_oc.append(abs(reference["delta_p_hot"]) / 1e3)
    dp_ours.append(abs(mine["pressure_drop_coolant_Pa"]) / 1e3)
heat_oc, heat_ours = np.array(heat_oc), np.array(heat_ours)
worst_heat = np.max(np.abs(heat_ours / heat_oc - 1)) * 100

fig, axes = plt.subplots(1, 3, figsize=(18, 7.2), dpi=150, gridspec_kw=dict(width_ratios=[1, 1.15, 1.1]))
fig.patch.set_facecolor("#FFFFFF")

# A: parity against OpenConcept.
ax = axes[0]
top = max(heat_oc.max(), heat_ours.max()) * 1.08
ax.plot([0, top], [0, top], color=gray, lw=1.5, ls="--", zorder=1)
ax.scatter(heat_oc, heat_ours, s=90, color=blue, edgecolor="#FFFFFF", linewidth=1.5, zorder=3)
halo = int(np.argmin(np.abs(heat_oc - 113.75)))
ax.annotate("Halo core, hot day\n113.78 vs 113.75 kW", (heat_oc[halo], heat_ours[halo]), xytext=(70, 45),
            fontsize=12, color=ink, arrowprops=dict(arrowstyle="-", color=ink, lw=1))
ax.set_xlim(0, top), ax.set_ylim(0, top)
ax.set_xlabel("Heat rejected, OpenConcept (kW)")
ax.set_ylabel("Heat rejected, our model (kW)")
ax.set_title("Code to code: same answer as OpenConcept", loc="left", fontsize=15, color=ink, fontweight="bold")
ax.text(0.03, 0.97, f"11 cases, identical inputs\nworst heat difference {worst_heat:.2f} %\n"
        "coolant pressure drop within 2.2 %\n(one documented d_h definition)",
        transform=ax.transAxes, va="top", fontsize=12, color=muted)

# B: Kellermann 2021 design point, ratio ours / published.
ax = axes[1]
rows = [("Heat rejected", 96.3 / 96.3, "96.3 kW both, imposed"),
        ("Frontal area", 0.164 / 0.131, "0.164 vs 0.131 m²"),
        ("Core volume", 19 / 63, "19 vs 63 L: offset strip fins vs plain channels"),
        ("Cooling drag", 25.5 / 5, "25.5 vs ≈5 N: our inlet model"),
        ("  ideal inlet", 14.0 / 5, "14 N")]
y = np.arange(len(rows))[::-1]
for yi, (label, ratio, note) in zip(y, rows):
    color = blue if 0.67 <= ratio <= 1.5 else orange
    ax.plot([1, ratio], [yi, yi], color=color, lw=3, zorder=2)
    ax.scatter([ratio], [yi], s=110, color=color, edgecolor="#FFFFFF", linewidth=1.5, zorder=3)
    ax.text(0.205, yi + 0.27, note, fontsize=11.5, color=muted)
ax.axvline(1, color=ink, lw=1.5)
ax.axvspan(0.67, 1.5, color=blue, alpha=0.07, lw=0)
ax.set_xscale("log")
ax.xaxis.set_minor_formatter(plt.NullFormatter())
ax.xaxis.set_minor_locator(plt.NullLocator())
ax.set_xlim(0.2, 8)
ax.set_xticks([0.25, 0.5, 1, 2, 4, 8])
ax.set_xticklabels(["¼×", "½×", "same", "2×", "4×", "8×"])
ax.set_yticks(y)
ax.set_yticklabels([r[0] for r in rows], fontsize=13, color=ink)
ax.set_ylim(-0.6, len(rows) - 0.3)
ax.set_xlabel("Our model ÷ Kellermann et al. 2021 (96 kW design point)")
ax.set_title("Published design point: same face, smaller core", loc="left", fontsize=15, color=ink,
             fontweight="bold")

# C: published TMS specific power.
ax = axes[2]
published = [("Kellermann 2021, one TMS", 1.75), ("NASA STARC-ABL", 3.1), ("NASA tiltwing, coolant loops", 3.8),
             ("NASA 15-pax tiltwing", 4.2), ("NASA RVLT tiltwing", 6.7)]
halo_kw_kg = 111.9 / 57.95
ax.axvspan(min(v for _, v in published), max(v for _, v in published), color=gray, alpha=0.12, lw=0)
for i, (label, value) in enumerate(published):
    ax.scatter([value], [i], s=90, color=gray, edgecolor="#FFFFFF", linewidth=1.5, zorder=3)
    ax.text(value + 0.12, i, f"{label}  {value:g}", va="center", fontsize=12, color=muted)
yh = len(published) + 0.4
ax.scatter([halo_kw_kg], [yh], s=160, color=orange, edgecolor="#FFFFFF", linewidth=2, zorder=4, marker="D")
ax.text(halo_kw_kg + 0.3, yh, f"Halo v3.7  {halo_kw_kg:.1f}  (112 kW, 58 kg)", va="center", fontsize=13,
        color=ink, fontweight="bold")
ax.set_xlim(0, 9.5)
ax.set_ylim(-0.8, len(published) + 1.2)
ax.set_yticks([])
ax.set_xlabel("Cooling system heat per mass (kW/kg)")
ax.set_title("Mass: inside the published range", loc="left", fontsize=15, color=ink, fontweight="bold")
ax.text(0.02, 0.02, "Kellermann et al., Aerospace 2021\nChapman et al., NASA TMs 2020",
        transform=ax.transAxes, fontsize=10.5, color=muted)

for ax in axes:
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
axes[2].spines["left"].set_visible(False)
fig.tight_layout(w_pad=3)
out = sys.argv[1] if len(sys.argv) > 1 else "docs/figures/ntu_validation.png"
fig.savefig(out, facecolor=fig.get_facecolor())
print("saved", out, f"worst heat diff {worst_heat:.3f} %, Halo {halo_kw_kg:.2f} kW/kg")
