"""Plan 044: battery energy by objective. One coupled solve per case from the cached v3.6 baseline.

Cases: each objective with the pack free; a fixed-pack sweep (1.25-2 x the reference parallel count) for the mass
and cost objectives; battery price x 2 (cost) and +25 % cell capacity (mass). Results go to output/battery/<case>.pkl
and .json (summaries read by halo_battery_objectives_figure.py). Run from the repository root with
PYTHONPATH=src;. and OMP_NUM_THREADS=1; arguments name the cases to run (none: all). Restartable: done cases are
skipped. output/battery/baseline.json (the v3.6 design, the start) is exported from the cached baseline pickle.
"""
import ctypes, json, os, pickle, shutil, sys, time, traceback
from dataclasses import replace, asdict, is_dataclass
from types import SimpleNamespace

import examples.halo_sizing as hs

out = os.path.join("output", "battery")
# Optional shared solve slots (a folder in which each running solve holds a slotN directory); unset: none.
slots = os.environ.get("HALO_SOLVE_SLOTS")
slot = None


def free_kB():
    class Status(ctypes.Structure):
        _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong), ("ullTotalPhys", ctypes.c_ulonglong),
                    ("ullAvailPhys", ctypes.c_ulonglong), ("ullTotalPageFile", ctypes.c_ulonglong),
                    ("ullAvailPageFile", ctypes.c_ulonglong), ("ullTotalVirtual", ctypes.c_ulonglong),
                    ("ullAvailVirtual", ctypes.c_ulonglong), ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]
    s = Status(); s.dwLength = ctypes.sizeof(Status)
    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(s))
    return s.ullAvailPhys / 1024


def acquire():
    global slot
    if slots is None:
        return
    while True:
        if free_kB() >= 1.8e6:
            for name in ("slot1", "slot2", "slot3"):
                try:
                    os.makedirs(os.path.join(slots, name), exist_ok=False)
                except FileExistsError:
                    continue
                slot = os.path.join(slots, name)
                open(os.path.join(slot, "owner.txt"), "w").write("battery-energy-sweep (plan 044)")
                log(f"  took {name}")
                return
        log(f"  waiting for slot / memory (free {free_kB() / 1e6:.2f} GB)")
        time.sleep(60)


def release():
    global slot
    if slot is not None:
        shutil.rmtree(slot, ignore_errors=True)
        slot = None


def log(text):
    print(time.strftime("%H:%M:%S"), text, flush=True)


def baseline_start():
    d = json.load(open(os.path.join(out, "baseline.json")))
    return SimpleNamespace(design=hs.HaloDesign(**d["design"]), mass_takeoff_kg=d["mass_takeoff_kg"],
                           velocity_cruise_m_s=d["velocity_cruise_m_s"], velocity_loiter_m_s=d["velocity_loiter_m_s"],
                           mass_payload_kg=d["mass_payload_kg"], count_lanes_motor=d["count_lanes_motor"],
                           power_rated_motor_W=d["power_rated_motor_W"], range_m=None)


def load(name):
    path = os.path.join(out, f"{name}.pkl")
    return pickle.load(open(path, "rb")) if os.path.exists(path) else None


def summary(result, a):
    hov = [h if isinstance(h, dict) else (asdict(h) if is_dataclass(h) else str(h)) for h in result.hovers]
    return dict(mass_takeoff_kg=result.mass_takeoff_kg, mass_payload_kg=result.mass_payload_kg,
                mass_fuel_burnt_kg=result.mass_fuel_burnt_kg, mass_empty_kg=result.mass_empty_kg,
                energy_battery_kWh=result.energy_battery_kWh, count_parallel=result.design.count_parallel_battery,
                range_nm=result.range_m / 1852.0, cost=asdict(result.cost), cost_per_mission_usd=result.cost.per_mission_usd,
                velocity_cruise_kt=result.velocity_cruise_m_s / 0.514444, soc_end=result.soc_end,
                soc_after_engine_out=result.soc_after_engine_out, binding=list(result.binding),
                machine_units={k: list(v) for k, v in result.machine_units.items()},
                segments=[{k: v for k, v in s.items() if isinstance(v, (int, float, str))} for s in result.segments],
                hovers=hov, powertrain_masses_kg=dict(result.powertrain_masses_kg),
                component_masses_kg=dict(result.component_masses_kg))


# The reference machines: whole units fixed at the v3.6 counts (2 motor units per lane, 3 generator units).
base = replace(hs.HaloAssumptions(), count_units_motor=2, count_units_generator=3)
factors_sweep = (1.25, 1.5, 1.75, 2.0)


def jobs():
    """(name, objective, assumptions, start) in order; later jobs read earlier results."""
    yield "mass_free", "mass_takeoff", base, lambda: baseline_start()
    for k in factors_sweep:
        yield f"mass_np{k:.2f}", "mass_takeoff", lambda k=k: replace(
            base, count_parallel_battery_fixed=k * load("mass_free").design.count_parallel_battery), lambda: load("mass_free")
    yield "cost_free", "cost", base, lambda: load("mass_free")
    for k in factors_sweep:
        yield f"cost_np{k:.2f}", "cost", lambda k=k: replace(
            base, count_parallel_battery_fixed=k * load("mass_free").design.count_parallel_battery), \
            lambda k=k: load(f"mass_np{k:.2f}") or load("cost_free") or load("mass_free")
    yield "fuel_free", "fuel", base, lambda: load("mass_free")
    yield "payload_free", "payload", base, lambda: load("mass_free")
    yield "range_free", "range", base, lambda: load("mass_free")
    yield "cost_battery_price_x2", "cost", lambda: replace(base, cost_model=replace(
        base.cost_model, price_battery_usd_J=2 * base.cost_model.price_battery_usd_J)), lambda: load("cost_free")
    yield "mass_capacity_x1.25", "mass_takeoff", lambda: replace(base, factor_capacity_ageing_battery=1.0), \
        lambda: load("mass_free")


only = set(sys.argv[1:])
for name, objective, assumptions, start in jobs():
    if only and name not in only:
        continue
    if os.path.exists(os.path.join(out, f"{name}.pkl")) or os.path.exists(os.path.join(out, f"{name}.failed")):
        continue
    assumptions = assumptions() if callable(assumptions) else assumptions
    initial = start()
    if initial is None:
        log(f"{name}: no start available, skipped")
        continue
    acquire()
    t = time.time()
    log(f"{name}: solving ({objective}, N_p fixed {assumptions.count_parallel_battery_fixed})")
    try:
        result = hs.solve_halo_sizing(hs.HaloRequirements(), assumptions, initial=initial, objective=objective)
    except Exception as error:
        release()
        log(f"{name}: FAILED after {time.time() - t:.0f} s: {error}")
        traceback.print_exc()
        open(os.path.join(out, f"{name}.failed"), "w").write(str(error))
        continue
    release()
    pickle.dump(result, open(os.path.join(out, f"{name}.pkl"), "wb"))
    s = summary(result, assumptions)
    json.dump(s, open(os.path.join(out, f"{name}.json"), "w"), indent=1, default=str)
    log(f"{name}: {time.time() - t:.0f} s; TOGW {s['mass_takeoff_kg'] / 0.45359237:.0f} lb, battery "
        f"{s['energy_battery_kWh']:.1f} kWh, fuel {s['mass_fuel_burnt_kg']:.1f} kg, payload {s['mass_payload_kg']:.1f} kg, "
        f"range {s['range_nm']:.1f} nm, cost {s['cost_per_mission_usd']:.1f} USD")
log("done")
