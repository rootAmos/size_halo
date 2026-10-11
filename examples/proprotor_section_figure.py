"""Figure for plan 052: docs/figures/proprotor_sections.png, the 2-D blade sections the BEM uses.

Left: the XV-15's own NACA 64-series sections (a = 0.3 mean line at each station's design lift coefficient, t/c from
TM-86833 fig. 6). Right: the plan 047 Halo blade schedule (one NACA 64A210 camber line, thickness scaled).
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import SectionSchedule
from examples.proprotor_bemt_curves import airfoil_xv15_fraction, lift_design_xv15, palette, xv15_rotor

root = Path(__file__).resolve().parents[1]
ink, ink_muted = "#0b0b0b", "#52514e"
names_xv15 = ("NACA 64-935", "NACA 64-528", "NACA 64-118", "NACA 64-(1.5)12", "NACA 64-208")


def _draw(ax, airfoils, labels, title):
    for k, (airfoil, label) in enumerate(zip(airfoils, labels)):
        coordinates = airfoil.to_airfoil().coordinates
        offset = -0.42 * k
        ax.fill(coordinates[:, 0], coordinates[:, 1] + offset, color=palette[k], alpha=0.18, lw=0)
        ax.plot(coordinates[:, 0], coordinates[:, 1] + offset, color=palette[k], lw=2)
        ax.text(1.04, offset, label, va="center", fontsize=12, color=ink)
    ax.set_aspect("equal")
    ax.set_xlim(-0.03, 1.75)
    ax.axis("off")
    ax.set_title(title, fontsize=14, color=ink, loc="left")


def make_figure(path=root / "docs" / "figures" / "proprotor_sections.png"):
    plt.rcParams.update({"font.size": 12})
    figure, axes = plt.subplots(1, 2, figsize=(16, 9), facecolor="white")
    sections = xv15_rotor().sections
    fractions = airfoil_xv15_fraction
    t_c = onp.interp(fractions, sections.fraction_radius, sections.thickness_to_chord)
    labels = [f"{n}\n{f:.2f} R: t/c {t:.2f}, design $c_l$ {cl:.2f}" for n, f, t, cl in
              zip(names_xv15, fractions, t_c, lift_design_xv15)]
    _draw(axes[0], sections.airfoils(fractions), labels, "XV-15 blade (model of TM-86833 / CR-114363)")
    halo = SectionSchedule()
    fractions_halo = (0.10, 0.40, 0.75, 1.00)
    labels_halo = [f"{f:.2f} R: t/c {t:.2f}\n64A210 camber line" for f, t in zip(fractions_halo, halo.thickness_to_chord)]
    _draw(axes[1], halo.airfoils(fractions_halo), labels_halo, "Halo blade (plan 047 schedule, JVX-like)")
    figure.savefig(path, dpi=110, bbox_inches="tight")
    return path


if __name__ == "__main__":
    print(make_figure())
