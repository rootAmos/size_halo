"""Prescribed and semi-free missions as chains of quasi-steady segments (M2-M3).

Orchestration: builds one flight point per segment (start mass, start SOC) and
chains mass and state of charge explicitly. Fuel burnt = fuel flow x duration;
battery chemical energy = OCV x current x duration, so SOC falls by that energy
over the pack capacity (the Tier 1 battery identity). Mission constraints
(fuel sufficiency, SOC floor, margins) are left to the caller.
"""
from dataclasses import dataclass
from typing import Any

from aircraft_closure.performance.flight_point import build_flight_point


@dataclass(frozen=True)
class Mission:
    segments: tuple


@dataclass(frozen=True)
class SegmentResult:
    segment: Any
    point: Any
    duration_s: Any
    mass_start_kg: Any
    mass_fuel_burnt_kg: Any
    energy_battery_chemical_J: Any
    energy_battery_terminal_J: Any
    soc_start: Any
    soc_end: Any


@dataclass(frozen=True)
class MissionResult:
    segments: tuple
    mass_fuel_burnt_kg: Any
    energy_battery_chemical_J: Any
    soc_end: Any
    mass_end_kg: Any
    duration_s: Any
    margins: tuple


def build_mission(opti, aircraft, aerodynamics, mission, mass_start_kg, soc_start):
    battery = aircraft.powertrain.topology.instances["battery"].component
    results = []
    mass_kg, soc = mass_start_kg, soc_start
    for segment in mission.segments:
        point = build_flight_point(opti, aircraft, aerodynamics, segment.flight_condition(soc), mass_kg)
        # The actuator disk is valid for non-negative thrust (relevant in descent).
        opti.subject_to(point.thrust_per_rotor_N >= 0)
        duration_s = segment.duration_s()
        fuel_kg = point.fuel_flow_kg_s * duration_s
        energy_chemical_J = point.battery.power_chemical_W * duration_s
        soc_end = soc - energy_chemical_J / battery.energy_capacity_J
        results.append(SegmentResult(segment=segment, point=point, duration_s=duration_s, mass_start_kg=mass_kg,
                                     mass_fuel_burnt_kg=fuel_kg, energy_battery_chemical_J=energy_chemical_J,
                                     energy_battery_terminal_J=point.power_battery_W * duration_s,
                                     soc_start=soc, soc_end=soc_end))
        mass_kg, soc = mass_kg - fuel_kg, soc_end
    return MissionResult(
        segments=tuple(results),
        mass_fuel_burnt_kg=sum(r.mass_fuel_burnt_kg for r in results),
        energy_battery_chemical_J=sum(r.energy_battery_chemical_J for r in results),
        soc_end=soc, mass_end_kg=mass_kg, duration_s=sum(r.duration_s for r in results),
        margins=tuple(m for r in results for m in r.point.margins),
    )
