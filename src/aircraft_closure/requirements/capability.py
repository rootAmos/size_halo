"""Capability requirements as quasi-steady flight conditions.

A requirement is met when every Tier 3 operating margin at its flight point is
non-negative. Requirements describe capability; missions (Tier 8) describe
what is flown. Sizing rule for the series hybrid: sustained requirements
(climb, speed, ceiling) default to turbogenerator power alone
(`hybridization_electric = 0`, the battery may be depleted); hover may draw on
the battery (`None` = free split). Point requirements carry no energy limit.
"""
from dataclasses import dataclass
from typing import Any

from aircraft_closure.performance.flight_point import FlightCondition


@dataclass(frozen=True)
class HoverRequirement:
    altitude_m: Any = 1000.0
    thrust_to_weight: Any = 1.1
    active_rotor_count: Any = None
    hybridization_electric: Any = None

    def flight_condition(self):
        return FlightCondition(mode="hover", velocity_m_s=0.0, altitude_m=self.altitude_m,
                               thrust_to_weight=self.thrust_to_weight, active_rotor_count=self.active_rotor_count,
                               hybridization_electric=self.hybridization_electric, label="hover")


@dataclass(frozen=True)
class ClimbRequirement:
    climb_rate_m_s: Any = 5.0
    velocity_m_s: Any = 50.0
    altitude_m: Any = 1000.0
    hybridization_electric: Any = 0.0

    def flight_condition(self):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=self.altitude_m,
                               climb_rate_m_s=self.climb_rate_m_s,
                               hybridization_electric=self.hybridization_electric, label="climb")


@dataclass(frozen=True)
class SpeedRequirement:
    velocity_m_s: Any = 80.0
    altitude_m: Any = 1000.0
    hybridization_electric: Any = 0.0

    def flight_condition(self):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=self.altitude_m,
                               hybridization_electric=self.hybridization_electric, label="max_speed")


@dataclass(frozen=True)
class CeilingRequirement:
    """Service ceiling: a residual climb rate at the ceiling altitude."""
    altitude_m: Any = 4000.0
    climb_rate_m_s: Any = 0.5
    velocity_m_s: Any = 55.0
    hybridization_electric: Any = 0.0

    def flight_condition(self):
        return FlightCondition(velocity_m_s=self.velocity_m_s, altitude_m=self.altitude_m,
                               climb_rate_m_s=self.climb_rate_m_s,
                               hybridization_electric=self.hybridization_electric, label="ceiling")


@dataclass(frozen=True)
class RequirementSet:
    mass_payload_kg: Any = 300.0
    hover: Any = HoverRequirement()
    climb: Any = ClimbRequirement()
    speed: Any = SpeedRequirement()
    ceiling: Any = CeilingRequirement()

    def flight_conditions(self):
        return tuple(r.flight_condition() for r in (self.hover, self.climb, self.speed, self.ceiling) if r is not None)
