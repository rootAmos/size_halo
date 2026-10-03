"""Feeder protection (Tier 15): line contactors and fuses, one set per feeder.

Per pole: mass = mass_fixed + mass_per_current x max_current (TE Kilovac EV200-class contactor, about 500 A
continuous and 0.43 kg, plus a fuse and bus bar: 0.2 kg + 1.3 g/A, assumed). Losses are the contact and fuse
drop, `voltage_drop_rated_V` per pole at the rated current (resistive, so loss = R I^2 for either sign).
"""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ProtectionResult:
    current_A: Any
    power_loss_W: Any
    voltage_drop_V: Any


@dataclass(frozen=True)
class ProtectionLimits:
    max_current_A: Any
    max_voltage_V: Any


@dataclass(frozen=True)
class ProtectionUnit:
    max_current_A: Any = 500.0
    max_voltage_V: Any = 900.0
    count_poles: int = 2
    mass_fixed_kg: Any = 0.2
    mass_per_current_kg_A: Any = 1.3e-3
    voltage_drop_rated_V: Any = 0.15

    def resistance_ohm(self):
        return self.count_poles * self.voltage_drop_rated_V / self.max_current_A

    def get_mass(self):
        return self.count_poles * (self.mass_fixed_kg + self.mass_per_current_kg_A * self.max_current_A)

    def get_limits(self):
        return ProtectionLimits(self.max_current_A, self.max_voltage_V)

    def evaluate(self, current_A):
        resistance_ohm = self.resistance_ohm()
        return ProtectionResult(current_A, resistance_ohm * current_A**2, resistance_ohm * current_A)
