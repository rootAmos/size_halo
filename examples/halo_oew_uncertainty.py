"""Empty-weight (OEW) prediction of the sized Halo baseline: growth allowance and uncertainty, item by item.

Each item of the empty-weight build-up carries two separate quantities:

- **Growth allowance** (`maturity_levels`): the growth expected as the design matures, set by how mature the item's
  weight is, following the structure of AIAA S-120A / SAWE mass-growth practice (the percentages are program
  defaults, to be set by the weights team). It shifts the prediction: predicted OEW = basic OEW + sum of the
  allowances.
- **Uncertainty**: a one-sigma spread that reflects how the item's mass is estimated. Catalogue hardware is tight;
  calibrated handbook groups (AFDD, Raymer with XV-15 factors) carry about +/-15 % at 95 % confidence, widened where
  the calibration is weakest; lightly modelled items are wide.

Items are independent and normally distributed, so OEW is normal with mean = the predicted OEW and
sigma = sqrt(sum sigma_i^2). Correlated errors (every Raymer group biased the same way) would widen it. Weight is
held at a fixed take-off weight: the aircraft is not re-sized, so the growth a heavier empty weight causes through
fuel, battery and structure is not included.

The target is a not-to-exceed OEW, by default the basic OEW of the sized baseline. The 95 % and 99 % values are
one-sided: the OEW not exceeded with that probability.

    python -m examples.halo_oew_uncertainty    # sizes the baseline; writes output/oew/oew_distribution.png
"""
import math
import os
from dataclasses import dataclass

import numpy
import aerosandbox.tools.units as u

blue, orange, grey, ink = "#2a78d6", "#eb6834", "#5e5d58", "#1a1a19"
z_95, z_99 = 1.6449, 2.3263              # one-sided standard-normal quantiles

# Design maturity -> growth allowance (fraction of the basic weight).
maturity_levels = {
    "Vendor hardware": 0.02,             # catalogue item, quoted weight
    "Vendor data": 0.05,                 # catalogue units plus an integration estimate
    "Calculated / layout": 0.08,         # from component data or a drawn layout
    "Calibrated parametric": 0.10,       # handbook method calibrated on a reference aircraft
    "Estimated": 0.15,                   # uncalibrated method or an assumed allowance
}


@dataclass(frozen=True)
class OewItem:
    label: str
    group: str                           # "Powertrain" or "Airframe and systems"
    mass_kg: float                       # basic weight from the sizing
    maturity: str                        # key of `maturity_levels`
    sigma_fraction: float                # one-sigma uncertainty as a fraction of the basic weight
    basis: str

    @property
    def growth_fraction(self):
        return maturity_levels[self.maturity]


# (label, group, keys in the sizing result, maturity, sigma, basis). Keys starting "pt:" are powertrain instances.
item_definitions = (
    ("Wing", "Airframe and systems", ("wing",), "Calibrated parametric", 0.10,
     "AFDD tiltrotor wing x 1.33 (XV-15); strength at ultimate not demonstrated"),
    ("Tails", "Airframe and systems", ("horizontal_tail", "vertical_tail"), "Calibrated parametric", 0.10,
     "Raymer x XV-15 factor"),
    ("Fuselage", "Airframe and systems", ("fuselage",), "Calculated / layout", 0.10,
     "Raymer x 1.70, anchored to the drawn layout"),
    ("Nacelles", "Airframe and systems", ("nacelles",), "Calibrated parametric", 0.10, "AFDD-class estimate"),
    ("Landing gear", "Airframe and systems", ("landing_gear",), "Calibrated parametric", 0.075,
     "Raymer x XV-15 factor"),
    ("Systems", "Airframe and systems", ("systems",), "Calibrated parametric", 0.15,
     "Raymer; flight controls carry an XV-15 factor of about 4"),
    ("Fixed equipment", "Airframe and systems", ("equipment",), "Estimated", 0.10, "Assumed allowance"),
    ("Rotors", "Powertrain", ("pt:propulsor",), "Calibrated parametric", 0.075,
     "AFDD blades and hubs, XV-15 calibrated"),
    ("Rotor gearboxes", "Powertrain", ("pt:gearbox",), "Calibrated parametric", 0.10, "AFDD drive system"),
    ("Generator gearboxes", "Powertrain", ("pt:generator_gearbox",), "Calibrated parametric", 0.10,
     "AFDD drive system"),
    ("Motors (with inverters)", "Powertrain", ("pt:motor",), "Vendor data", 0.05,
     "Whole catalogue units plus inverter allowance"),
    ("Generators (with inverters)", "Powertrain", ("pt:generator",), "Vendor data", 0.05,
     "Whole catalogue units plus inverter allowance"),
    ("Turboshafts", "Powertrain", ("pt:turboshaft",), "Vendor hardware", 0.025,
     "Fixed off-the-shelf engines plus installation"),
    ("Battery", "Powertrain", ("pt:battery",), "Calculated / layout", 0.075,
     "50G cell data; 70 % cell-to-pack mass assumed"),
    ("Heat exchanger", "Powertrain", ("pt:heat_exchanger",), "Estimated", 0.15,
     "Thermal model, mass per watt assumed"),
    ("Protection and bus tie", "Powertrain", ("pt:protection_string", "pt:protection_bus_tie", "pt:cable_bus_tie"),
     "Estimated", 0.20, "Simple ratings-based estimate"),
)


def oew_items(sizing):
    components = dict(sizing.component_masses_kg)
    powertrain = dict(sizing.powertrain_masses_kg)
    items = []
    for label, group, keys, maturity, sigma, basis in item_definitions:
        mass_kg = sum(powertrain.get(k[3:], 0.0) if k.startswith("pt:") else components.get(k, 0.0) for k in keys)
        items.append(OewItem(label, group, float(mass_kg), maturity, sigma, basis))
    total_kg = sum(item.mass_kg for item in items)
    if abs(total_kg - sizing.mass_empty_kg) > 1.0:
        raise ValueError(f"items sum to {total_kg:.1f} kg, OEW is {sizing.mass_empty_kg:.1f} kg")
    return tuple(items)


@dataclass(frozen=True)
class OewDistribution:
    basic_lb: float                      # sum of the basic item weights (the sized OEW)
    growth_lb: float                     # sum of the growth allowances
    sigma_lb: float                      # root sum of squares of the item uncertainties

    @property
    def mean_lb(self):
        """Predicted OEW: basic weight plus growth allowance."""
        return self.basic_lb + self.growth_lb

    @property
    def value_95_lb(self):
        return self.mean_lb + z_95 * self.sigma_lb

    @property
    def value_99_lb(self):
        return self.mean_lb + z_99 * self.sigma_lb


def oew_distribution(items):
    basic_lb = sum(item.mass_kg for item in items) / u.lbm
    growth_lb = sum(item.growth_fraction * item.mass_kg for item in items) / u.lbm
    sigma_lb = float(numpy.sqrt(sum((item.sigma_fraction * item.mass_kg / u.lbm) ** 2 for item in items)))
    return OewDistribution(basic_lb, growth_lb, sigma_lb)


def markdown_table(items, distribution):
    """Item build-up: basic weight, maturity and growth allowance, uncertainty (one sigma), predicted weight."""
    d = distribution
    rows = ["| Item | Basic weight (lb) | Maturity | Growth allowance (%) | Growth allowance (lb) "
            "| Uncertainty, 1σ (%) | Uncertainty, 1σ (lb) | Predicted weight (lb) | Basis |",
            "|---|---|---|---|---|---|---|---|---|"]
    for group in ("Powertrain", "Airframe and systems"):
        for item in sorted((i for i in items if i.group == group), key=lambda i: -i.mass_kg):
            basic_lb = item.mass_kg / u.lbm
            rows.append(f"| {item.label} | {basic_lb:,.0f} | {item.maturity} | {100 * item.growth_fraction:.0f} | "
                        f"{item.growth_fraction * basic_lb:,.0f} | {100 * item.sigma_fraction:.1f} | "
                        f"{item.sigma_fraction * basic_lb:,.0f} | {(1 + item.growth_fraction) * basic_lb:,.0f} | "
                        f"{item.basis} |")
    rows.append(f"| **OEW** | **{d.basic_lb:,.0f}** | | **{100 * d.growth_lb / d.basic_lb:.1f}** | **{d.growth_lb:,.0f}** "
                f"| **{100 * d.sigma_lb / d.basic_lb:.1f}** | **{d.sigma_lb:,.0f}** | **{d.mean_lb:,.0f}** "
                f"| Growth summed; uncertainty root sum of squares, items independent |")
    return "\n".join(rows)


def probability_at_most(distribution, weight_lb):
    """Probability that the OEW comes in at or below `weight_lb`."""
    z = (weight_lb - distribution.mean_lb) / distribution.sigma_lb
    return 0.5 * (1 + math.erf(z / math.sqrt(2)))


def plot_distribution(distribution, path, title, target_lb=None):
    """Predicted OEW density with the target and the 95 % and 99 % values; shades the chance of meeting the target."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = distribution
    target_lb = d.basic_lb if target_lb is None else target_lb
    low_lb, high_lb = min(target_lb, d.mean_lb - 4 * d.sigma_lb), d.mean_lb + 4 * d.sigma_lb
    weight_lb = numpy.linspace(low_lb - 0.5 * d.sigma_lb, high_lb, 700)
    pdf = numpy.exp(-0.5 * ((weight_lb - d.mean_lb) / d.sigma_lb) ** 2) / (d.sigma_lb * numpy.sqrt(2 * numpy.pi))
    chance = probability_at_most(d, target_lb)
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    ax.plot(weight_lb, pdf, color=blue, lw=2.2)
    ax.fill_between(weight_lb, pdf, where=weight_lb <= target_lb, color=blue, alpha=0.25, lw=0,
                    label=f"meets the target: {100 * chance:.1f} %")
    peak = pdf.max()
    # The design rule: the 99 % value must sit at or below the not-to-exceed target.
    if d.value_99_lb > target_lb:
        ax.annotate("", xy=(target_lb, peak * 0.30), xytext=(d.value_99_lb, peak * 0.30),
                    arrowprops=dict(arrowstyle="<->", color=orange, lw=1.3))
        ax.text(0.5 * (target_lb + d.value_99_lb), peak * 0.33,
                f"{d.value_99_lb - target_lb:,.0f} lb over:\n99 % value must be\nat or below the target",
                ha="center", va="bottom", fontsize=8.5, color=orange,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.85, pad=1.5))
    for value, label, color, style, side, height in (
            (target_lb, "target (not to exceed)\n= basic OEW", ink, "-", 1, 1.02),
            (d.mean_lb, "predicted\n(basic + growth)", blue, "-.", -1, 1.02),
            (d.value_95_lb, "95 %", orange, "--", 1, 1.22),
            (d.value_99_lb, "99 %", orange, ":", 1, 1.02)):
        ax.axvline(value, color=color, ls=style, lw=1.6)
        ax.annotate(f"{label}\n{value:,.0f} lb", (value, peak * height), xytext=(4 * side, 0),
                    textcoords="offset points", fontsize=8.5, color=color, va="bottom",
                    ha="left" if side > 0 else "right")
    ax.set_xlabel("Empty weight, OEW (lb)")
    ax.set_ylabel("Probability density")
    ax.set_title(f"{title}\nbasic {d.basic_lb:,.0f} lb + growth allowance {d.growth_lb:,.0f} lb "
                 f"({100 * d.growth_lb / d.basic_lb:.1f} %); uncertainty σ = {d.sigma_lb:,.0f} lb "
                 f"({100 * d.sigma_lb / d.basic_lb:.1f} %)", fontsize=10.5)
    ax.set_ylim(0, peak * 1.45)
    ax.set_yticks([])
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.grid(axis="x", alpha=0.3)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run(sizing=None, directory="output/oew", target_lb=None):
    if sizing is None:
        from examples.halo_sizing import solve_halo_sizing
        sizing = solve_halo_sizing()
    items = oew_items(sizing)
    distribution = oew_distribution(items)
    os.makedirs(directory, exist_ok=True)
    plot_distribution(distribution, os.path.join(directory, "oew_distribution.png"),
                      f"Empty-weight prediction of the baseline design ({sizing.mass_takeoff_kg / u.lbm:,.0f} lb take-off)",
                      target_lb)
    table = markdown_table(items, distribution)
    with open(os.path.join(directory, "oew_items.md"), "w", encoding="utf-8") as file:
        file.write(table + "\n")
    print(table)
    d = distribution
    target = d.basic_lb if target_lb is None else target_lb
    print(f"\nbasic {d.basic_lb:,.0f} lb, growth {d.growth_lb:,.0f} lb, predicted {d.mean_lb:,.0f} lb, "
          f"sigma {d.sigma_lb:,.0f} lb; 95 % {d.value_95_lb:,.0f} lb, 99 % {d.value_99_lb:,.0f} lb; "
          f"P(OEW <= {target:,.0f} lb) = {100 * probability_at_most(d, target):.1f} %")
    return items, distribution


if __name__ == "__main__":
    run()
