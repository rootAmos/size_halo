"""Empty-weight (OEW) uncertainty of the sized Halo baseline: item-by-item mass build-up and its distribution.

Each item of the empty-weight build-up gets a one-sigma uncertainty that reflects how its mass is estimated:

- catalogue hardware (fixed turboshafts, whole machine units) is tight;
- calibrated handbook groups (AFDD, Raymer with XV-15 factors) carry +/-15 % at 95 % confidence, the spread the
  design-space sensitivity study uses (sigma 7.5 %), widened where the calibration is weakest;
- lightly modelled items (heat exchanger, protection and bus tie) are wide.

Items are taken as independent and normally distributed, so OEW is normal with mean = the sized OEW and
sigma = sqrt(sum sigma_i^2). Correlated errors (for example every Raymer group biased the same way) would widen it.
This is a bottom-up spread at fixed take-off weight: the aircraft is not re-sized, so the growth a heavier empty
weight causes through fuel, battery and structure is not included.

The OEW target is the sized baseline OEW; the 95 % and 99 % values are one-sided: the OEW not exceeded with that
probability.

    python -m examples.halo_oew_uncertainty    # sizes the baseline; writes output/oew/oew_distribution.png
"""
import math
import os
from dataclasses import dataclass

import numpy
import aerosandbox.tools.units as u

blue, orange, grey, ink = "#2a78d6", "#eb6834", "#5e5d58", "#1a1a19"
z_95, z_99 = 1.6449, 2.3263              # one-sided standard-normal quantiles


@dataclass(frozen=True)
class OewItem:
    label: str
    group: str                           # "Powertrain" or "Airframe and systems"
    mass_kg: float
    sigma_fraction: float                # one-sigma uncertainty as a fraction of the item mass
    basis: str


# (label, group, keys in the sizing result, sigma, basis). Keys starting "pt:" are powertrain instances.
item_definitions = (
    ("Wing", "Airframe and systems", ("wing",), 0.10,
     "AFDD tiltrotor wing x 1.33 (XV-15); strength at ultimate not demonstrated"),
    ("Tails", "Airframe and systems", ("horizontal_tail", "vertical_tail"), 0.10, "Raymer x XV-15 factor"),
    ("Fuselage", "Airframe and systems", ("fuselage",), 0.10, "Raymer x 1.70, anchored to the drawn layout"),
    ("Nacelles", "Airframe and systems", ("nacelles",), 0.10, "AFDD-class estimate"),
    ("Landing gear", "Airframe and systems", ("landing_gear",), 0.075, "Raymer"),
    ("Systems", "Airframe and systems", ("systems",), 0.15, "Raymer; flight controls carry an XV-15 factor of about 4"),
    ("Fixed equipment", "Airframe and systems", ("equipment",), 0.10, "Assumed allowance"),
    ("Rotors", "Powertrain", ("pt:propulsor",), 0.075, "AFDD blades and hubs, XV-15 calibrated"),
    ("Rotor gearboxes", "Powertrain", ("pt:gearbox",), 0.10, "AFDD drive system"),
    ("Generator gearboxes", "Powertrain", ("pt:generator_gearbox",), 0.10, "AFDD drive system"),
    ("Motors (with inverters)", "Powertrain", ("pt:motor",), 0.05, "Whole catalogue units plus inverter allowance"),
    ("Generators (with inverters)", "Powertrain", ("pt:generator",), 0.05,
     "Whole catalogue units plus inverter allowance"),
    ("Turboshafts", "Powertrain", ("pt:turboshaft",), 0.025, "Fixed off-the-shelf engines plus installation"),
    ("Battery", "Powertrain", ("pt:battery",), 0.075, "50G cell data; 70 % cell-to-pack mass assumed"),
    ("Heat exchanger", "Powertrain", ("pt:heat_exchanger",), 0.15, "Thermal model, mass per watt assumed"),
    ("Protection and bus tie", "Powertrain", ("pt:protection_string", "pt:protection_bus_tie", "pt:cable_bus_tie"),
     0.20, "Simple ratings-based estimate"),
)


def oew_items(sizing):
    components = dict(sizing.component_masses_kg)
    powertrain = dict(sizing.powertrain_masses_kg)
    items = []
    for label, group, keys, sigma, basis in item_definitions:
        mass_kg = sum(powertrain.get(k[3:], 0.0) if k.startswith("pt:") else components.get(k, 0.0) for k in keys)
        items.append(OewItem(label, group, float(mass_kg), sigma, basis))
    total_kg = sum(item.mass_kg for item in items)
    if abs(total_kg - sizing.mass_empty_kg) > 1.0:
        raise ValueError(f"items sum to {total_kg:.1f} kg, OEW is {sizing.mass_empty_kg:.1f} kg")
    return tuple(items)


@dataclass(frozen=True)
class OewDistribution:
    mean_lb: float
    sigma_lb: float

    @property
    def value_95_lb(self):
        return self.mean_lb + z_95 * self.sigma_lb

    @property
    def value_99_lb(self):
        return self.mean_lb + z_99 * self.sigma_lb


def oew_distribution(items):
    mean_lb = sum(item.mass_kg for item in items) / u.lbm
    sigma_lb = float(numpy.sqrt(sum((item.sigma_fraction * item.mass_kg / u.lbm) ** 2 for item in items)))
    return OewDistribution(mean_lb, sigma_lb)


def markdown_table(items, distribution):
    rows = ["| Item | Weight (lb) | 1σ (%) | 1σ (lb) | Share of OEW variance | Basis |", "|---|---|---|---|---|---|"]
    variance = distribution.sigma_lb ** 2
    for group in ("Powertrain", "Airframe and systems"):
        for item in sorted((i for i in items if i.group == group), key=lambda i: -i.mass_kg):
            sigma_lb = item.sigma_fraction * item.mass_kg / u.lbm
            rows.append(f"| {item.label} | {item.mass_kg / u.lbm:,.0f} | {100 * item.sigma_fraction:.1f} | "
                        f"{sigma_lb:,.0f} | {100 * sigma_lb ** 2 / variance:.0f} % | {item.basis} |")
    rows.append(f"| **OEW** | **{distribution.mean_lb:,.0f}** | **{100 * distribution.sigma_lb / distribution.mean_lb:.1f}**"
                f" | **{distribution.sigma_lb:,.0f}** | 100 % | Root sum of squares, items independent |")
    return "\n".join(rows)


def probability_at_most(distribution, weight_lb):
    """Probability that the OEW comes in at or below `weight_lb`."""
    z = (weight_lb - distribution.mean_lb) / distribution.sigma_lb
    return 0.5 * (1 + math.erf(z / math.sqrt(2)))


def plot_distribution(distribution, path, title, target_lb=None):
    """OEW density with the target and the 95 % and 99 % values; shades the chance of meeting the target."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = distribution
    target_lb = d.mean_lb if target_lb is None else target_lb
    weight_lb = numpy.linspace(d.mean_lb - 4 * d.sigma_lb, d.mean_lb + 4 * d.sigma_lb, 600)
    pdf = numpy.exp(-0.5 * ((weight_lb - d.mean_lb) / d.sigma_lb) ** 2) / (d.sigma_lb * numpy.sqrt(2 * numpy.pi))
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    ax.plot(weight_lb, pdf, color=blue, lw=2.2)
    ax.fill_between(weight_lb, pdf, where=weight_lb <= target_lb, color=blue, alpha=0.18, lw=0,
                    label=f"meets the target: {100 * probability_at_most(d, target_lb):.0f} %")
    peak = pdf.max()
    # The design rule: the 99 % value must sit at or below the not-to-exceed target.
    if d.value_99_lb > target_lb:
        ax.annotate("", xy=(target_lb, peak * 0.30), xytext=(d.value_99_lb, peak * 0.30),
                    arrowprops=dict(arrowstyle="<->", color=orange, lw=1.3))
        ax.text(0.5 * (target_lb + d.value_99_lb), peak * 0.33,
                f"{d.value_99_lb - target_lb:,.0f} lb over:\n99 % value must be\nat or below the target",
                ha="center", va="bottom", fontsize=8.5, color=orange,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.85, pad=1.5))
    for value, label, color, style, side in ((target_lb, "target (not to exceed)", ink, "-", -1),
                                             (d.value_95_lb, "95 %", orange, "--", -1),
                                             (d.value_99_lb, "99 %", orange, ":", 1)):
        ax.axvline(value, color=color, ls=style, lw=1.6)
        ax.annotate(f"{label}\n{value:,.0f} lb", (value, peak * 1.02), xytext=(4 * side, 0),
                    textcoords="offset points", fontsize=9, color=color, va="bottom",
                    ha="left" if side > 0 else "right")
    ax.set_xlabel("Empty weight, OEW (lb)")
    ax.set_ylabel("Probability density")
    ax.set_title(title, fontsize=11)
    ax.set_ylim(0, peak * 1.25)
    ax.set_yticks([])
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.grid(axis="x", alpha=0.3)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    ax.text(0.01, 0.80, f"σ = {d.sigma_lb:,.0f} lb ({100 * d.sigma_lb / d.mean_lb:.1f} %)\n"
                        f"estimate {d.mean_lb:,.0f} lb\n"
                        f"chance of meeting the target: {100 * probability_at_most(d, target_lb):.0f} %",
            transform=ax.transAxes, ha="left", va="top", fontsize=9, color=grey)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run(sizing=None, directory="output/oew"):
    if sizing is None:
        from examples.halo_sizing import solve_halo_sizing
        sizing = solve_halo_sizing()
    items = oew_items(sizing)
    distribution = oew_distribution(items)
    os.makedirs(directory, exist_ok=True)
    plot_distribution(distribution, os.path.join(directory, "oew_distribution.png"),
                      f"Empty-weight uncertainty of the baseline design ({sizing.mass_takeoff_kg / u.lbm:,.0f} lb take-off)")
    table = markdown_table(items, distribution)
    with open(os.path.join(directory, "oew_items.md"), "w", encoding="utf-8") as file:
        file.write(table + "\n")
    print(table)
    print(f"\nOEW {distribution.mean_lb:,.0f} lb, sigma {distribution.sigma_lb:,.0f} lb; 95 % {distribution.value_95_lb:,.0f} lb, "
          f"99 % {distribution.value_99_lb:,.0f} lb")
    return items, distribution


if __name__ == "__main__":
    run()
