"""Typed physical ports; flow is positive in the port's nominal direction."""
from dataclasses import dataclass
from enum import Enum
from typing import Any


class Domain(Enum):
    MECHANICAL = "mechanical"
    ELECTRICAL = "electrical"
    FUEL = "fuel"


class Direction(Enum):
    """Nominal power or fuel flow direction relative to the owning component."""
    IN = "in"
    OUT = "out"


@dataclass(frozen=True)
class PortSpec:
    name: str
    domain: Domain
    direction: Direction


@dataclass(frozen=True)
class MechanicalPortValue:
    """Shaft state; torque is positive when power flows in the nominal direction."""
    speed_rad_s: Any
    torque_Nm: Any


@dataclass(frozen=True)
class ElectricalPortValue:
    """Terminal state; current is positive when power flows in the nominal direction."""
    voltage_V: Any
    current_A: Any


@dataclass(frozen=True)
class FuelPortValue:
    fuel_flow_kg_s: Any


port_value_types = {
    Domain.MECHANICAL: MechanicalPortValue,
    Domain.ELECTRICAL: ElectricalPortValue,
    Domain.FUEL: FuelPortValue,
}
