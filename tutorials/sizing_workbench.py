"""A small workbench for sizing Halo variants quickly and comparing them (used by the workbook notebooks).

    from sizing_workbench import *
    base = baseline_variant()                                   # the cached near-reference (Scholz aerodynamics)
    heavy = size("1,100 kg payload", requirements=replace(base.requirements, mass_payload_kg=1100.0))
    compare(base, heavy)                                        # key numbers side by side, with deltas
    binding_changes(base, heavy)                                # what started or stopped sizing the aircraft
    weight_changes(base, heavy)                                 # where the mass went

Every `size` call is one `solve_halo_sizing` warm-started from the baseline (about 15-40 s). If that fails it falls back to the
full starting-point strategy (minutes) and says so. Nothing here changes the model.
"""
import re
import time
from dataclasses import dataclass, replace
from typing import Any

from tutorial_helpers import *                                 # noqa: F401,F403 (paths, asb, np, u, plt, cached, lb, quiet_solver)
from tutorial_helpers import cached, lb, quiet_solver, plt, u, blue, orange, aqua, yellow, red, ink_muted
from examples.halo_sizing import HaloAssumptions, HaloRequirements, solve_halo_sizing


@dataclass(frozen=True)
class Variant:
    label: str
    requirements: Any
    assumptions: Any
    result: Any
    time_s: float = 0.0
    started_from: str = "baseline"


def baseline_variant():
    """The near-reference: every model at the reference setting except Scholz aerodynamics (tutorial 19), cached."""
    requirements = HaloRequirements()
    assumptions = replace(HaloAssumptions(), aerodynamics_model="scholz")
    result = cached("reference_scholz", lambda: solve_halo_sizing(requirements, assumptions))
    return Variant("baseline", requirements, assumptions, result, 0.0, "cache")


def size(label, requirements=None, assumptions=None, start=None, objective="mass_takeoff", max_iter=1000):
    """Size one variant. Unset `requirements` / `assumptions` default to the baseline's; `start` is a Variant to warm-start from.

    `max_iter` caps IPOPT on the warm start (a good warm start converges in a few hundred iterations), so a bad one fails fast and
    the full strategy takes over."""
    base = baseline_variant()
    requirements = requirements if requirements is not None else base.requirements
    assumptions = assumptions if assumptions is not None else base.assumptions
    start = start if start is not None else base
    time_start_s = time.perf_counter()
    try:
        with quiet_solver():
            result = solve_halo_sizing(requirements, assumptions, initial=start.result, objective=objective, max_iter=max_iter)
        started_from = start.label
    except RuntimeError:
        print(f"[{label}: the warm start from '{start.label}' failed; trying the full starting-point strategy (can take minutes)]")
        with quiet_solver():
            result = solve_halo_sizing(requirements, assumptions, objective=objective)
        started_from = "multistart"
    variant = Variant(label, requirements, assumptions, result, time.perf_counter() - time_start_s, started_from)
    print(f"[{label}: sized in {variant.time_s:.0f} s from '{started_from}': {lb(result.mass_takeoff_kg):,.0f} lb]")
    return variant


_rows = (
    ("take-off weight [lb]", lambda r: lb(r.mass_takeoff_kg), "{:,.0f}"),
    ("empty weight [lb]", lambda r: lb(r.mass_empty_kg), "{:,.0f}"),
    ("payload [lb]", lambda r: lb(r.mass_payload_kg), "{:,.0f}"),
    ("fuel [lb]", lambda r: lb(r.mass_fuel_kg), "{:,.0f}"),
    ("battery [kWh]", lambda r: r.energy_battery_kWh, "{:.1f}"),
    ("wing area [m2]", lambda r: r.design.area_wing_m2, "{:.2f}"),
    ("span [m]", lambda r: r.span_m, "{:.2f}"),
    ("rotor radius [m]", lambda r: r.radius_rotor_m, "{:.3f}"),
    ("disk loading [kg/m2]", lambda r: r.disk_loading_kg_m2, "{:.1f}"),
    ("cruise speed [kt]", lambda r: r.velocity_cruise_m_s / u.knot, "{:.0f}"),
    ("cruise L/D", lambda r: r.lift_to_drag_cruise, "{:.2f}"),
    ("motor rating per lane [kW]", lambda r: r.power_rated_motor_W / 1e3, "{:.0f}"),
    ("generator rating [kW]", lambda r: r.power_rated_generator_W / 1e3, "{:.0f}"),
    ("cost per mission [USD]", lambda r: r.cost.per_mission_usd, "{:,.0f}"),
)


def compare(*variants):
    """Key numbers of each variant, with the change from the first one."""
    head = f"{'':<28}" + "".join(f"{v.label[:22]:>24}" for v in variants)
    print(head)
    for name, f, fmt in _rows:
        values = [f(v.result) for v in variants]
        cells = []
        for i, value in enumerate(values):
            text = fmt.format(value)
            if i > 0 and values[0]:
                text += f" ({(value / values[0] - 1):+.1%})"
            cells.append(f"{text:>24}")
        print(f"{name:<28}" + "".join(cells))
    units = [v.result.machine_units for v in variants]
    if all(units):
        print(f"{'machine units (motor, gen)':<28}" + "".join(f"{str((x['motor'][1] or round(x['motor'][0], 2), x['generator'][1] or round(x['generator'][0], 2))):>24}"
                                                         for x in units))


def _family(label):
    """'engine-out hover 2/3: battery end voltage_V' -> 'engine-out hover: battery end voltage_V' (sub-points merged)."""
    return re.sub(r" \d+/\d+:", ":", label)


def binding_changes(base, other):
    """Which binding constraints (sub-points merged) started or stopped sizing the aircraft, and which stayed."""
    a = {_family(x) for x in base.result.binding}
    b = {_family(x) for x in other.result.binding}
    for title, items in (("now binding (new)", sorted(b - a)), ("no longer binding", sorted(a - b)), ("still binding", sorted(a & b))):
        print(f"{title}:")
        for item in items:
            print(f"    {item}")
        if not items:
            print("    -")


def weight_changes(base, other, threshold_kg=5.0):
    """Mass changes by group (airframe groups and powertrain instances), largest first."""
    def groups(result):
        masses = dict(result.component_masses_kg)
        masses.pop("powertrain", None)
        masses.update({f"powertrain: {k}": v for k, v in result.powertrain_masses_kg})
        return masses
    ga, gb = groups(base.result), groups(other.result)
    rows = sorted(((gb.get(k, 0.0) - ga.get(k, 0.0), k) for k in set(ga) | set(gb)), key=lambda row: -abs(row[0]))
    print(f"{'group':<34}{base.label[:14]:>16}{other.label[:14]:>16}{'change':>12}")
    for delta, k in rows:
        if abs(delta) >= threshold_kg:
            print(f"{k:<34}{lb(ga.get(k, 0.0)):14,.0f} lb{lb(gb.get(k, 0.0)):14,.0f} lb{lb(delta):+10,.0f} lb")
    print(f"{'take-off weight':<34}{lb(base.result.mass_takeoff_kg):14,.0f} lb{lb(other.result.mass_takeoff_kg):14,.0f} lb"
          f"{lb(other.result.mass_takeoff_kg - base.result.mass_takeoff_kg):+10,.0f} lb")


def plot_trade(variants, x_values, x_label, title=None, quantities=("take-off weight [lb]", "battery [kWh]", "fuel [lb]")):
    """Plot chosen `_rows` quantities against a swept input."""
    lookup = {name: f for name, f, _ in _rows}
    fig, axes = plt.subplots(1, len(quantities), figsize=(4 * len(quantities), 3.2))
    for ax, name, color in zip(axes, quantities, (blue, orange, aqua, yellow, red)):
        ax.plot(x_values, [lookup[name](v.result) for v in variants], "o-", color=color, lw=2)
        ax.set(xlabel=x_label, ylabel=name)
    if title:
        fig.suptitle(title)
    plt.tight_layout(); plt.show()
