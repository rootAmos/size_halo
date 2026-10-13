"""Prescribed and semi-free missions as chains of quasi-steady segments (M2-M3).

Orchestration: builds one flight point per segment (start mass, start SOC) and
chains mass and state of charge explicitly. Fuel burnt = fuel flow x duration;
battery chemical energy = OCV x current x duration, so SOC falls by that energy
over the pack capacity (the Tier 1 battery identity). Mission constraints
(fuel sufficiency, SOC floor, margins) are left to the caller.

Tier 17: `subsegments` splits each segment into equal-duration points so OCV
and resistance can vary within it (1, the default, is the earlier one point
per segment). With an `EquivalentCircuitBattery` the battery owns the SOC
update (coulomb counting, OCV at mid-interval SOC) and the RC voltages are
propagated from point to point, starting from rest (`polarization_start`
"rest") or fully developed ("steady"); each point also reports its
end-of-interval terminal voltage against the pack's minimum voltage.

Plan 040: a per-bus engine-out point (`FlightCondition.bus_tie`, see
`performance.bus_split`) has one string group per bus, each with its own SOC
and RC voltages; the chain then carries them as dicts keyed "A" and "B",
bounds each SOC and end voltage, and the mission's `soc_end` is their mean
(the pack SOC, the groups being equal halves), with `soc_end_by_bus` per bus.

Tier 19: component temperatures (machines and packs with a thermal model) propagate
from point to point like the RC voltages. `thermal_start` "steady" (default)
gives every point its steady state (no history, i.e. continuous ratings);
"coolant" starts the mission at each component's coolant temperature; a dict
of start temperatures by instance name continues an earlier history. Every
point then holds its temperature margins at the end of its interval.

Plan 060: `battery_dynamics` "quasi_steady" (default) holds the end-of-interval
voltage with OCV and R0 at the mid-interval SOC; "constant_current" evaluates it at
the end SOC, the true minimum of the interval under constant discharge current
(the RC update is already exact), so the limit holds at every instant as the
subsegments converge (first order: the current is constant within a point).

"piecewise" (plan 060 part 2, equivalent-circuit battery) is direct collocation:
a segment of n subsegments has n + 1 nodes, each an instantaneous flight point
(zero duration) whose current is solved from its power required, P = V(I) I with
V = OCV(s) - I R0(s) - sum v_k (the low-current root). SOC, mass, the RC
branch voltages and the Tier 19 temperatures are node values linked by the
trapezoidal rule (SOC: -I/Q; mass: -fuel flow; v_k: (I R_k - v_k)/tau_k;
T: (loss - (T - T_coolant)/R)/C), and the voltage, SOC, temperature and every
point limit hold at every node. Current is piecewise linear in time within a
segment (second order) and steps only where the flight condition does, at
segment boundaries, where the states carry over. Each node is reported as a
subsegment whose `duration_s` is its trapezoidal weight (dt/2 at the ends, dt
inside), so the sums of fuel, energy and duration are the trapezoidal integrals.
"""
import math
from dataclasses import dataclass, replace
from typing import Any

from aircraft_closure.core.margins import margin_above, margin_below
from aircraft_closure.performance.flight_point import build_flight_point
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.redundancy import battery_for_condition
from aircraft_closure.thermal.heat import coolant_temperatures_C, thermal_parameters


@dataclass(frozen=True)
class Mission:
    segments: tuple


@dataclass(frozen=True)
class SegmentResult:
    segment: Any
    point: Any                          # the (first) flight point of the segment
    duration_s: Any
    mass_start_kg: Any
    mass_fuel_burnt_kg: Any
    energy_battery_chemical_J: Any
    energy_battery_terminal_J: Any
    soc_start: Any
    soc_end: Any
    subsegments: tuple = ()             # per-point SegmentResults when the segment is split


@dataclass(frozen=True)
class MissionResult:
    segments: tuple
    mass_fuel_burnt_kg: Any
    energy_battery_chemical_J: Any
    soc_end: Any
    mass_end_kg: Any
    duration_s: Any
    margins: tuple
    temperatures_end_C: Any = None      # Tier 19: by instance name (None: no thermal history)
    soc_end_by_bus: Any = None          # plan 040: {"A": ..., "B": ...} after per-bus points (soc_end is their mean)


def build_mission(opti, aircraft, aerodynamics, mission, mass_start_kg, soc_start, hybridization_electric_min=0.0,
                  soc_floor=None, subsegments=1, polarization_start="rest", thermal_start="steady",
                  battery_dynamics="quasi_steady", step_battery_max_s=30.0, count_nodes_battery_min=9):
    """`hybridization_electric_min` < 0 allows in-flight recharging from the generators on free-split segments.

    SOC stays inside the battery's own window (min_soc..max_soc) at every segment end, so a mission cannot dip
    below the floor and recover by recharging later. `soc_floor` raises that lower bound (e.g. to hold an
    emergency reserve throughout the flight); None uses the battery's min_soc.

    `subsegments`: points per segment, an int for every segment or a tuple with one count per segment.

    "piecewise" (plan 060 part 3) integrates the battery between the flight-point nodes in sub-steps of at most
    `step_battery_max_s`, with at least `count_nodes_battery_min` battery nodes per segment.
    """
    battery = aircraft.powertrain.topology.instances["battery"].component
    is_ecm = isinstance(battery, EquivalentCircuitBattery)
    counts = subsegments if isinstance(subsegments, tuple) else (subsegments,) * len(mission.segments)
    if len(counts) != len(mission.segments) or any(n < 1 for n in counts):
        raise ValueError("subsegments needs one count >= 1 per segment.")
    if polarization_start not in ("rest", "steady"):
        raise ValueError(f"Unknown polarization_start '{polarization_start}'.")
    if battery_dynamics not in ("quasi_steady", "constant_current", "piecewise"):
        raise ValueError(f"Unknown battery_dynamics '{battery_dynamics}'.")
    if battery_dynamics == "piecewise":
        if not is_ecm:
            raise ValueError("battery_dynamics 'piecewise' needs the equivalent-circuit battery.")
        return _build_mission_nodes(opti, aircraft, aerodynamics, mission, mass_start_kg, soc_start,
                                    hybridization_electric_min, soc_floor, counts, polarization_start, thermal_start,
                                    step_battery_max_s, count_nodes_battery_min)
    if isinstance(thermal_start, dict):
        temperatures_C = dict(thermal_start)
    elif thermal_start == "coolant":
        temperatures_C = coolant_temperatures_C(aircraft.powertrain)
    elif thermal_start == "steady":
        temperatures_C = None
    else:
        raise ValueError(f"Unknown thermal_start '{thermal_start}'.")
    rest_V = tuple(0.0 for _ in battery.cell.resistance_model.time_constants_s()) if is_ecm else None
    voltage_rc_V = rest_V if polarization_start == "rest" else None
    soc_lower = battery.min_soc if soc_floor is None else soc_floor
    results, margins = [], []
    mass_kg, soc = mass_start_kg, soc_start
    for segment, count in zip(mission.segments, counts):
        duration_s = segment.duration_s() if count == 1 else segment.duration_s() / count
        points = []
        for index in range(count):
            condition = segment.flight_condition(soc)
            if count > 1:
                condition = replace(condition, label=f"{condition.label} {index + 1}/{count}")
            if is_ecm:
                point = build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg,
                                           hybridization_electric_min=hybridization_electric_min,
                                           duration_s=duration_s, voltage_rc_start_V=voltage_rc_V,
                                           temperature_start_C=temperatures_C)
            else:
                point = build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg,
                                           hybridization_electric_min=hybridization_electric_min,
                                           duration_s=duration_s, temperature_start_C=temperatures_C)
            if temperatures_C is not None:
                temperatures_C = point.thermal.temperatures_end_C
            # The actuator disk is valid for non-negative thrust (relevant in descent).
            opti.subject_to(point.thrust_per_rotor_N >= 0)
            fuel_kg = point.fuel_flow_kg_s * duration_s
            energy_chemical_J = point.battery.power_chemical_W * duration_s
            if point.split is not None:
                # Plan 040: per-bus string groups, each with its own SOC, RC history and end voltage.
                states = point.split.buses
                energy_chemical_J = sum(states[b].battery.power_chemical_W for b in states) * duration_s
                soc_end = dict(point.split.soc_next)
                voltage_rc_V = dict(point.split.voltage_rc_end_V)
                for b in states:
                    margins.append(margin_above(f"{condition.label}: battery {b} end voltage_V",
                                                point.split.voltage_end_V[b], battery.get_limits().min_voltage_V))
                    opti.subject_to([soc_end[b] <= battery.max_soc, soc_end[b] >= soc_lower])
            elif is_ecm:
                soc_end = point.battery.soc_next
                voltage_rc_V = point.battery.voltage_rc_end_V
                voltage_end_V = (point.battery.voltage_end_V if battery_dynamics == "quasi_steady" else
                                 battery_for_condition(aircraft.powertrain.topology, condition).voltage_terminal_V(
                                     point.battery.current_A, soc_end, voltage_rc_V))
                margins.append(margin_above(f"{condition.label}: battery end voltage_V", voltage_end_V,
                                            battery.get_limits().min_voltage_V))
            else:
                # Tier 18: a degraded point (a string isolated) depletes only the strings still connected.
                capacity_J = battery_for_condition(aircraft.powertrain.topology, condition).energy_capacity_J
                soc_end = soc - energy_chemical_J / capacity_J
            if point.split is None:
                if isinstance(soc, dict) or isinstance(voltage_rc_V, dict):
                    raise ValueError("A lumped point cannot follow per-bus points in one mission.")
                opti.subject_to([soc_end <= battery.max_soc, soc_end >= soc_lower])
            margins.extend(point.margins)
            points.append(SegmentResult(segment=segment, point=point, duration_s=duration_s, mass_start_kg=mass_kg,
                                        mass_fuel_burnt_kg=fuel_kg, energy_battery_chemical_J=energy_chemical_J,
                                        energy_battery_terminal_J=point.power_battery_W * duration_s,
                                        soc_start=soc, soc_end=soc_end))
            mass_kg, soc = mass_kg - fuel_kg, soc_end
        if count == 1:
            results.append(points[0])
        else:
            results.append(SegmentResult(
                segment=segment, point=points[0].point, duration_s=sum(p.duration_s for p in points),
                mass_start_kg=points[0].mass_start_kg, mass_fuel_burnt_kg=sum(p.mass_fuel_burnt_kg for p in points),
                energy_battery_chemical_J=sum(p.energy_battery_chemical_J for p in points),
                energy_battery_terminal_J=sum(p.energy_battery_terminal_J for p in points),
                soc_start=points[0].soc_start, soc_end=points[-1].soc_end, subsegments=tuple(points)))
    soc_by_bus = soc if isinstance(soc, dict) else None
    return MissionResult(
        segments=tuple(results),
        mass_fuel_burnt_kg=sum(r.mass_fuel_burnt_kg for r in results),
        energy_battery_chemical_J=sum(r.energy_battery_chemical_J for r in results),
        soc_end=sum(soc.values()) / len(soc) if soc_by_bus else soc, mass_end_kg=mass_kg,
        duration_s=sum(r.duration_s for r in results), margins=tuple(margins), temperatures_end_C=temperatures_C,
        soc_end_by_bus=soc_by_bus,
    )


def _guess(opti, expression, default):
    """Numeric initial guess of an expression from the Opti's initial values (a number passes through)."""
    if isinstance(expression, (int, float)):
        return float(expression)
    try:
        return float(opti.value(expression, opti.initial()))
    except Exception:  # noqa: BLE001 (an expression of parameters without values)
        return default


def _rates_temperature_C_s(aircraft, point, battery):
    """dT/dt = (loss - (T - T_coolant) / R) / C per thermal-modelled instance at an instantaneous node."""
    instances = aircraft.powertrain.topology.instances
    rates = {}
    for load in point.thermal.heat_loads:
        if load.source not in point.thermal.temperatures_end_C:
            continue
        component = battery if load.source == "battery" else instances[load.source].component
        p = thermal_parameters(component)
        rates[load.source] = (load.power_W - point.thermal.power_to_coolant_W[load.source]) / p.capacity_J_K
    return rates


def _build_mission_nodes(opti, aircraft, aerodynamics, mission, mass_start_kg, soc_start, hybridization_electric_min,
                         soc_floor, counts, polarization_start, thermal_start, step_battery_max_s=30.0,
                         count_nodes_battery_min=9):
    """`build_mission` with `battery_dynamics="piecewise"` (plan 060): flight points on nodes; mass and temperatures
    by the trapezoidal rule between them; SOC and RC voltages sub-stepped through the equivalent circuit at a
    terminal power linear between the nodes (part 3), with the voltage limits at every sub-step."""
    topology = aircraft.powertrain.topology
    battery_design = topology.instances["battery"].component
    soc_lower = battery_design.min_soc if soc_floor is None else soc_floor
    count_rc = len(battery_design.cell.resistance_model.time_constants_s())
    if isinstance(thermal_start, dict):
        temperatures_C = dict(thermal_start)
    elif thermal_start == "coolant":
        temperatures_C = coolant_temperatures_C(aircraft.powertrain)
    elif thermal_start == "steady":
        temperatures_C = None
    else:
        raise ValueError(f"Unknown thermal_start '{thermal_start}'.")
    voltage_rc_V = (0.0,) * count_rc if polarization_start == "rest" else None
    mass_kg, soc = mass_start_kg, soc_start
    results, margins = [], []
    for segment, count in zip(mission.segments, counts):
        dt_s = segment.duration_s() / count
        count_steps = max(math.ceil(float(_guess(opti, dt_s, 60.0)) / step_battery_max_s),
                          math.ceil((count_nodes_battery_min - 1) / count))
        nodes = []
        for index in range(count + 1):
            steady_start = False
            if index > 0:
                # The node's states are unknowns, tied to the previous node by the trapezoidal rule below.
                mass_guess_kg = _guess(opti, mass_kg, 1000.0)
                mass_kg = opti.variable(init_guess=mass_guess_kg, scale=mass_guess_kg)
                soc = opti.variable(init_guess=_guess(opti, soc, 0.5), scale=0.1)
                voltage_rc_V = tuple(opti.variable(init_guess=_guess(opti, v, 0.0), scale=10.0) for v in voltage_rc_V)
                if temperatures_C is not None:
                    temperatures_C = {name: opti.variable(init_guess=_guess(opti, t, 60.0), scale=50.0)
                                      for name, t in temperatures_C.items()}
            elif voltage_rc_V is None:
                # "steady" start: fully developed at the first node's current (tied once the point is built).
                voltage_rc_V = tuple(opti.variable(init_guess=0.0, scale=10.0) for _ in range(count_rc))
                steady_start = True
            condition = segment.flight_condition(soc)
            condition = replace(condition, label=f"{condition.label} {index}/{count}")
            battery = battery_for_condition(topology, condition)
            point = build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg,
                                       hybridization_electric_min=hybridization_electric_min,
                                       duration_s=0.0, voltage_rc_start_V=voltage_rc_V,
                                       temperature_start_C=temperatures_C)
            current_A = point.battery.current_A
            if steady_start:
                opti.subject_to([(v - current_A * r) / 10.0 == 0
                                 for v, r in zip(voltage_rc_V, battery.resistances_ohm(soc)[1:])])
            # The actuator disk is valid for non-negative thrust (relevant in descent).
            opti.subject_to(point.thrust_per_rotor_N >= 0)
            opti.subject_to([soc <= battery_design.max_soc, soc >= soc_lower])
            margins.append(margin_above(f"{condition.label}: battery voltage_V", point.battery.voltage_V,
                                        battery.get_limits().min_voltage_V))
            margins.extend(point.margins)
            node = dict(point=point, mass_kg=mass_kg, soc=soc, voltage_rc_V=voltage_rc_V,
                        temperatures_C=temperatures_C, battery=battery, power_battery_W=point.battery.power_electric_W,
                        rates_temperature_C_s=(_rates_temperature_C_s(aircraft, point, battery)
                                               if temperatures_C is not None else {}))
            if nodes:
                a, b = nodes[-1], node
                scale_mass_kg = _guess(opti, a["mass_kg"], 1000.0)
                # Part 3: the battery between the nodes, sub-stepped at a terminal power linear from a's to b's.
                soc_end, voltage_rc_end_V, voltages_V, _ = a["battery"].integrate_power(
                    a["power_battery_W"], b["power_battery_W"], a["soc"], a["voltage_rc_V"], dt_s, count_steps)
                limits = a["battery"].get_limits()
                label = condition.label.rsplit(" ", 1)[0]
                for step, voltage_V in enumerate(voltages_V[:-1], start=1):
                    where = f"{label} {index - 1}/{count} step {step}/{count_steps}: battery"
                    margins.append(margin_above(f"{where} voltage_V", voltage_V, limits.min_voltage_V))
                    margins.append(margin_below(f"{where} max_voltage_V", voltage_V, limits.max_voltage_V))
                opti.subject_to(
                    [(b["soc"] - soc_end) / 0.1 == 0,
                     (b["mass_kg"] - a["mass_kg"] + dt_s / 2 * (a["point"].fuel_flow_kg_s + b["point"].fuel_flow_kg_s))
                     / scale_mass_kg == 0]
                    + [(vb - ve) / 10.0 == 0 for vb, ve in zip(b["voltage_rc_V"], voltage_rc_end_V)]
                    + [(b["temperatures_C"][name] - a["temperatures_C"][name]
                        - dt_s / 2 * (a["rates_temperature_C_s"][name] + b["rates_temperature_C_s"][name])) / 50.0 == 0
                       for name in a["rates_temperature_C_s"]])
            nodes.append(node)
        weights_s = [dt_s / 2 if index in (0, count) else dt_s for index in range(count + 1)]
        points = tuple(SegmentResult(
            segment=segment, point=n["point"], duration_s=w, mass_start_kg=n["mass_kg"],
            mass_fuel_burnt_kg=n["point"].fuel_flow_kg_s * w,
            energy_battery_chemical_J=n["point"].battery.power_chemical_W * w,
            energy_battery_terminal_J=n["point"].power_battery_W * w, soc_start=n["soc"], soc_end=n["soc"])
            for n, w in zip(nodes, weights_s))
        results.append(SegmentResult(
            segment=segment, point=points[0].point, duration_s=segment.duration_s(),
            mass_start_kg=points[0].mass_start_kg, mass_fuel_burnt_kg=points[0].mass_start_kg - nodes[-1]["mass_kg"],
            energy_battery_chemical_J=sum(p.energy_battery_chemical_J for p in points),
            energy_battery_terminal_J=sum(p.energy_battery_terminal_J for p in points),
            soc_start=points[0].soc_start, soc_end=nodes[-1]["soc"], subsegments=points))
        mass_kg, soc = nodes[-1]["mass_kg"], nodes[-1]["soc"]
        voltage_rc_V, temperatures_C = nodes[-1]["voltage_rc_V"], nodes[-1]["temperatures_C"]
    return MissionResult(
        segments=tuple(results),
        mass_fuel_burnt_kg=sum(r.mass_fuel_burnt_kg for r in results),
        energy_battery_chemical_J=sum(r.energy_battery_chemical_J for r in results),
        soc_end=soc, mass_end_kg=mass_kg, duration_s=sum(r.duration_s for r in results),
        margins=tuple(margins), temperatures_end_C=temperatures_C,
    )
