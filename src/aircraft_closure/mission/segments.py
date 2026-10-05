"""Quasi-steady mission segments (architecture level M1).

Each segment maps to one Tier 7 flight condition plus a duration. Speeds,
altitudes, distances and electric power fractions may be Opti variables, which
is how missions become (semi-)free. Hover segments stand for vertical take-off
and landing phases at T/W = 1; climb and descent are evaluated at their mean
altitude.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np

from aircraft_closure.performance.flight_point import FlightCondition


@dataclass(frozen=True)
class HoverSegment:
    duration_s_given: Any = 60.0
    altitude_m: Any = 0.0
    hybridization_electric: Any = None
    label: str = "hover"
    temperature_offset_K: Any = 0.0            # Tier 16: ISA + offset at the altitude
    active_generator_count: Any = None    # None: all; fewer models a turbogenerator out (Tier 17 reserve)
    # Tier 18 failure states (see FlightCondition): lanes per rotor, battery strings, failed buses.
    active_lane_count: Any = None
    active_battery_string_count: Any = None
    count_buses_failed: int = 0

    def duration_s(self):
        return self.duration_s_given

    def flight_condition(self, soc):
        return FlightCondition(mode="hover", velocity_m_s=0.0, altitude_m=self.altitude_m, thrust_to_weight=1.0,
                               active_generator_count=self.active_generator_count,
                               active_lane_count=self.active_lane_count,
                               active_battery_string_count=self.active_battery_string_count,
                               count_buses_failed=self.count_buses_failed,
                               soc=soc, hybridization_electric=self.hybridization_electric, label=self.label,
                               temperature_offset_K=self.temperature_offset_K)


@dataclass(frozen=True)
class ClimbSegment:
    altitude_start_m: Any = 0.0
    altitude_end_m: Any = 1000.0
    climb_rate_m_s: Any = 4.0
    velocity_m_s: Any = 50.0
    hybridization_electric: Any = None
    label: str = "climb"
    temperature_offset_K: Any = 0.0            # Tier 16: ISA + offset at the altitude

    def duration_s(self):
        return (self.altitude_end_m - self.altitude_start_m) / self.climb_rate_m_s

    def flight_condition(self, soc):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=(self.altitude_start_m + self.altitude_end_m) / 2,
                               climb_rate_m_s=self.climb_rate_m_s, soc=soc,
                               hybridization_electric=self.hybridization_electric, label=self.label,
                               temperature_offset_K=self.temperature_offset_K)


@dataclass(frozen=True)
class CruiseSegment:
    distance_m: Any = 100000.0
    altitude_m: Any = 1000.0
    velocity_m_s: Any = 60.0
    hybridization_electric: Any = None
    label: str = "cruise"
    temperature_offset_K: Any = 0.0            # Tier 16: ISA + offset at the altitude

    def duration_s(self):
        return self.distance_m / self.velocity_m_s

    def flight_condition(self, soc):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=self.altitude_m, soc=soc,
                               hybridization_electric=self.hybridization_electric, label=self.label,
                               temperature_offset_K=self.temperature_offset_K)


@dataclass(frozen=True)
class LoiterSegment:
    duration_s_given: Any = 1200.0
    altitude_m: Any = 1000.0
    velocity_m_s: Any = 45.0
    hybridization_electric: Any = None
    label: str = "loiter"
    temperature_offset_K: Any = 0.0            # Tier 16: ISA + offset at the altitude

    def duration_s(self):
        return self.duration_s_given

    def flight_condition(self, soc):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=self.altitude_m, soc=soc,
                               hybridization_electric=self.hybridization_electric, label=self.label,
                               temperature_offset_K=self.temperature_offset_K)


@dataclass(frozen=True)
class DescentSegment:
    altitude_start_m: Any = 1000.0
    altitude_end_m: Any = 0.0
    descent_rate_m_s: Any = 3.0
    velocity_m_s: Any = 50.0
    hybridization_electric: Any = None
    label: str = "descent"
    temperature_offset_K: Any = 0.0            # Tier 16: ISA + offset at the altitude

    def duration_s(self):
        return (self.altitude_start_m - self.altitude_end_m) / self.descent_rate_m_s

    def flight_condition(self, soc):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=(self.altitude_start_m + self.altitude_end_m) / 2,
                               climb_rate_m_s=-self.descent_rate_m_s, soc=soc,
                               hybridization_electric=self.hybridization_electric, label=self.label,
                               temperature_offset_K=self.temperature_offset_K)


def ground_distance_m(segment):
    """Horizontal distance covered (zero in hover)."""
    if isinstance(segment, HoverSegment):
        return 0.0
    if isinstance(segment, CruiseSegment):
        return segment.distance_m
    flight_path_rate_m_s = (segment.climb_rate_m_s if isinstance(segment, ClimbSegment)
                            else segment.descent_rate_m_s if isinstance(segment, DescentSegment) else 0.0)
    return np.sqrt(segment.velocity_m_s**2 - flight_path_rate_m_s**2) * segment.duration_s()
