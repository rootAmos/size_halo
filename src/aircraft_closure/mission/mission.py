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

Tier 19: component temperatures (machines and packs with a thermal model) propagate
from point to point like the RC voltages. `thermal_start` "steady" (default)
gives every point its steady state (no history, i.e. continuous ratings);
"coolant" starts the mission at each component's coolant temperature; a dict
of start temperatures by instance name continues an earlier history. Every
point then holds its temperature margins at the end of its interval.
"""
from dataclasses import dataclass, replace
from typing import Any

from aircraft_closure.core.margins import margin_above
from aircraft_closure.performance.flight_point import build_flight_point
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.thermal.heat import coolant_temperatures_C


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


def build_mission(opti, aircraft, aerodynamics, mission, mass_start_kg, soc_start, hybridization_electric_min=0.0,
                  soc_floor=None, subsegments=1, polarization_start="rest", thermal_start="steady"):
    """`hybridization_electric_min` < 0 allows in-flight recharging from the generators on free-split segments.

    SOC stays inside the battery's own window (min_soc..max_soc) at every segment end, so a mission cannot dip
    below the floor and recover by recharging later. `soc_floor` raises that lower bound (e.g. to hold an
    emergency reserve throughout the flight); None uses the battery's min_soc.

    `subsegments`: points per segment, an int for every segment or a tuple with one count per segment.
    """
    battery = aircraft.powertrain.topology.instances["battery"].component
    is_ecm = isinstance(battery, EquivalentCircuitBattery)
    counts = subsegments if isinstance(subsegments, tuple) else (subsegments,) * len(mission.segments)
    if len(counts) != len(mission.segments) or any(n < 1 for n in counts):
        raise ValueError("subsegments needs one count >= 1 per segment.")
    if polarization_start not in ("rest", "steady"):
        raise ValueError(f"Unknown polarization_start '{polarization_start}'.")
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
            if is_ecm:
                soc_end = point.battery.soc_next
                voltage_rc_V = point.battery.voltage_rc_end_V
                margins.append(margin_above(f"{condition.label}: battery end voltage_V", point.battery.voltage_end_V,
                                            battery.get_limits().min_voltage_V))
            else:
                soc_end = soc - energy_chemical_J / battery.energy_capacity_J
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
    return MissionResult(
        segments=tuple(results),
        mass_fuel_burnt_kg=sum(r.mass_fuel_burnt_kg for r in results),
        energy_battery_chemical_J=sum(r.energy_battery_chemical_J for r in results),
        soc_end=soc, mass_end_kg=mass_kg, duration_s=sum(r.duration_s for r in results),
        margins=tuple(margins), temperatures_end_C=temperatures_C,
    )
