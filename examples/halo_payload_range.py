"""Payload-range diagram of the sized Halo (plan 045): the reference aircraft frozen and flown off-design.

    python -m examples.halo_payload_range                 # solve every point (one coupled solve each), then plot
    python -m examples.halo_payload_range design ferry    # only these points
    python -m examples.halo_payload_range --plot          # plot the saved points

The sized aircraft is read from `--sized <pickle>` (a HaloSizingResult; default `output/payload_range/sized.pkl`,
solved and saved there if missing). Points are saved to `output/payload_range/points.json` as they finish.
"""
import json
import pickle
import sys
from dataclasses import asdict
from pathlib import Path

import aerosandbox.tools.units as u
import numpy as np

from examples.halo_sizing import solve_halo_off_design, solve_halo_sizing

path_output = Path("output/payload_range")
path_sized = path_output / "sized.pkl"
path_points = path_output / "points.json"
path_figure = Path("docs/figures/payload_range.png")


def payload_range_points(sized):
    """(name, keyword arguments of `solve_halo_off_design`) for the corner and intermediate points."""
    payload_kg = sized.mass_payload_kg
    range_design_nm = (sized.range_m or 445 * u.naut_mile) / u.naut_mile
    points = [("design", dict(range_nm=range_design_nm)),           # MTOW, full tanks, design payload
              ("short range", dict(range_nm=0.6 * range_design_nm)),
              ("payload-limited 1.1 x range", dict(range_nm=1.1 * range_design_nm))]
    for fraction in (0.75, 0.5, 0.25, 0.0):
        name = "ferry" if fraction == 0.0 else f"tanks full, {fraction:.0%} payload"
        points.append((name, dict(mass_payload_kg=fraction * payload_kg)))
        points.append((f"larger tanks, {fraction:.0%} payload", dict(mass_payload_kg=fraction * payload_kg,
                                                                      larger_tanks=True)))
    # What an operator gains by flying the ferry at its own best speeds instead of the sized ones.
    points.append(("ferry, free speeds", dict(mass_payload_kg=0.0, hold_speeds=False)))
    return points


class _TolerantUnpickler(pickle.Unpickler):
    """Reads a HaloSizingResult pickled by a branch with extra classes (e.g. plan 041's SizingSensitivity)."""

    def find_class(self, module, name):
        try:
            return super().find_class(module, name)
        except AttributeError:
            return type(name, (), {})


def load_sized(path):
    with open(path, "rb") as file:
        return _TolerantUnpickler(file).load()


def sized_reference(path=path_sized):
    """The sized reference (v3.6 defaults), from `path` or solved and saved there."""
    if Path(path).exists():
        return load_sized(path)
    sized = solve_halo_sizing()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as file:
        pickle.dump(sized, file)
    return sized


def point_record(name, point):
    result = point.result
    record = {k: v for k, v in asdict(point).items() if k != "result"}
    record.update(name=name, binding=list(result.binding), min_margin=result.min_margin, soc_end=result.soc_end,
                  offset_cg_trim_m=result.offset_cg_trim_m, closure_residual_kg=result.closure_residual_kg,
                  segments=[dict(label=s["label"], duration_s=s["duration_s"], fuel_kg=s["fuel_kg"])
                            for s in result.segments])
    return record


def solve_points(sized, names=None):
    records = json.loads(path_points.read_text()) if path_points.exists() else {}
    previous = {}     # the last point solved on each line, the start of the next (larger tanks: long range steps)
    for name, kwargs in payload_range_points(sized):
        if names and name not in names:
            continue
        line = "larger" if kwargs.get("larger_tanks") else "tanks"
        try:
            point = solve_halo_off_design(sized, start=previous.get(line) if "range_nm" not in kwargs else None,
                                          **kwargs)
            if "range_nm" not in kwargs:
                previous[line] = point
            records[name] = point_record(name, point)
            print(f"{name}: {point.mass_payload_kg:7.1f} kg over {point.range_nm:6.1f} nm, TOW "
                  f"{point.mass_takeoff_kg:7.1f} kg, fuel {point.mass_fuel_kg:6.1f} kg, min margin "
                  f"{point.result.min_margin:.2e}", flush=True)
        except RuntimeError as error:
            records[name] = dict(name=name, failed=str(error)[:200], **{k: v for k, v in kwargs.items()})
            print(f"{name}: failed ({error})", flush=True)
        path_points.write_text(json.dumps(records, indent=1))
    return records


def range_breguet_nm(sized, mass_payload_kg, mass_fuel_kg):
    """First-order range for comparison: Breguet cruise, range = k ln(Wi / Wf), with k calibrated on the sized
    cruise (constant speed, L/D and powertrain efficiency); hover, climb, loiter and descent fuel scaled with take-off
    mass; the same 10 % fuel reserve; climb and descent distances fixed; the battery as in the design mission."""
    segments = {s["label"]: s for s in sized.segments}
    reserve = sized.mass_fuel_kg / sized.mass_fuel_burnt_kg
    distance_climb_descent_nm = (segments["climb"]["duration_s"] + segments["descent"]["duration_s"]) * 80.0 / 1852.0
    fuel_before_kg = segments["take-off hover"]["fuel_kg"] + segments["climb"]["fuel_kg"]
    fuel_after_kg = sum(segments[k]["fuel_kg"] for k in ("reserve loiter", "descent", "landing hover"))
    mass_start_kg = sized.mass_takeoff_kg - fuel_before_kg
    distance_cruise_nm = segments["cruise"]["duration_s"] * segments["cruise"]["velocity_m_s"] / 1852.0
    k_nm = distance_cruise_nm / np.log(mass_start_kg / (mass_start_kg - segments["cruise"]["fuel_kg"]))
    mass_takeoff_kg = sized.mass_empty_kg + mass_payload_kg + mass_fuel_kg
    scale = mass_takeoff_kg / sized.mass_takeoff_kg
    fuel_cruise_kg = mass_fuel_kg / reserve - (fuel_before_kg + fuel_after_kg) * scale
    mass_cruise_kg = mass_takeoff_kg - fuel_before_kg * scale
    return distance_climb_descent_nm + k_nm * np.log(mass_cruise_kg / (mass_cruise_kg - fuel_cruise_kg))


def plot(records, sized, path=path_figure):
    import matplotlib.pyplot as plt

    lb = 1 / u.lbm
    ok = {k: v for k, v in records.items() if "failed" not in v}
    design = ok["design"]
    tank = sorted([ok[k] for k in ok if k.startswith("tanks full") or k == "ferry"] + [design],
                  key=lambda p: -p["mass_payload_kg"])
    larger = sorted([ok[k] for k in ok if k.startswith("larger tanks")] + [design], key=lambda p: -p["mass_payload_kg"])
    fig, ax = plt.subplots(figsize=(11.2, 7.0), dpi=150)
    fig.patch.set_facecolor("#FFFFFF")
    blue, orange, ink, grey = "#2A6CB0", "#C8651B", "#15233B", "#3A3F47"
    payload_lb = design["mass_payload_kg"] * lb
    ax.plot([0, design["range_nm"]], [payload_lb] * 2, color=blue, lw=2.6)
    ax.plot([p["range_nm"] for p in tank], [p["mass_payload_kg"] * lb for p in tank], color=blue, lw=2.6,
            marker="o", ms=5, label=f"Tanks sized to the design mission ({sized.mass_fuel_kg / u.lbm:,.0f} lb fuel)")
    ax.plot([p["range_nm"] for p in larger], [p["mass_payload_kg"] * lb for p in larger], color=orange, lw=2.2,
            ls="--", marker="o", ms=5, label=f"Larger tanks: fuel up to the {sized.mass_takeoff_kg / u.lbm:,.0f} lb MTOW")
    payloads_kg = np.linspace(sized.mass_payload_kg, 0.0, 30)
    ax.plot([range_breguet_nm(sized, p, sized.mass_fuel_kg) for p in payloads_kg], payloads_kg * lb, color="#9AA0A8",
            lw=1.4, ls=":", label="First-order Breguet estimate, tanks full")
    free = ok.get("ferry, free speeds")
    if free is not None:
        ax.plot([free["range_nm"]], [0], "D", ms=8, color=blue, mfc="#FFFFFF", mew=1.8, clip_on=False, zorder=5)
        ax.annotate(f"Ferry at its own best speeds\n{free['range_nm']:,.0f} nm", (free["range_nm"], 0),
                    xytext=(free["range_nm"] + 25, 880), fontsize=11.5, color=blue,
                    arrowprops=dict(arrowstyle="-", color=blue, lw=1.0))
    checks = [ok[k] for k in ("short range", "payload-limited 1.1 x range") if k in ok]
    ax.plot([p["range_nm"] for p in checks], [p["mass_payload_kg"] * lb for p in checks], "s", ms=6, ls="none",
            color=blue, mfc="#FFFFFF", label="Maximum payload at a set range (check points)")
    ax.plot([design["range_nm"]], [payload_lb], "o", ms=13, color=ink, mec="#FFFFFF", mew=2.5, zorder=5)
    ax.annotate(f"Design point\n{payload_lb:,.0f} lb over {design['range_nm']:.0f} nm", (design["range_nm"], payload_lb),
                xytext=(design["range_nm"] - 230, payload_lb - 560), fontsize=12.5, color=ink,
                arrowprops=dict(arrowstyle="-", color=ink, lw=1.2))
    ferry = ok.get("ferry")
    if ferry is not None:
        ax.annotate(f"Ferry, tank-limited\n{ferry['range_nm']:,.0f} nm", (ferry["range_nm"], 0),
                    xytext=(ferry["range_nm"] + 30, 300), fontsize=12.5, color=blue,
                    arrowprops=dict(arrowstyle="-", color=blue, lw=1.2))
    zero = ok.get("larger tanks, 0% payload")
    if zero is not None:
        ax.annotate(f"{zero['range_nm']:,.0f} nm\n({zero['mass_fuel_kg'] / u.lbm:,.0f} lb fuel)", (zero["range_nm"], 0),
                    xytext=(zero["range_nm"] - 250, 160), fontsize=12.5, color=orange,
                    arrowprops=dict(arrowstyle="-", color=orange, lw=1.2))
    x_max = max(p["range_nm"] for p in ok.values())
    ax.set_xlim(0, x_max * 1.06)
    ax.set_ylim(0, payload_lb * 1.2)
    ax.set_xlabel("Range (nm), with 20 min reserve loiter and 10 % fuel reserve", fontsize=14, color=grey)
    ax.set_ylabel("Payload (lb)", fontsize=14, color=grey)
    ax.set_title("Halo v3.6 payload-range: the sized aircraft flown off-design (coupled solve per point)",
                 fontsize=14, color=ink, loc="left")
    ax.tick_params(labelsize=12, colors=grey)
    for side in ("right", "top"):
        ax.spines[side].set_visible(False)
    ax.grid(color="#E4E2DA", lw=0.8)
    ax.legend(loc="upper right", fontsize=12, frameon=True, facecolor="#FFFFFF", edgecolor="#E4E2DA")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, facecolor=fig.get_facecolor())
    print("saved", path)


if __name__ == "__main__":
    args = sys.argv[1:]
    path = path_sized
    if "--sized" in args:
        i = args.index("--sized")
        path = Path(args[i + 1])
        del args[i:i + 2]
    path_output.mkdir(parents=True, exist_ok=True)
    sized = sized_reference(path)
    only_plot = "--plot" in args
    names = [a for a in args if a != "--plot"]
    records = json.loads(path_points.read_text()) if only_plot else solve_points(sized, names or None)
    if "design" in records and "failed" not in records["design"]:
        plot(records, sized)
