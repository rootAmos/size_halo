"""Constant-efficiency engine, shaft demand >= 0; no altitude lapse or idle."""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TurboshaftResult:
    power_shaft_W: Any
    fuel_flow_kg_s: Any
    power_loss_W: Any


@dataclass(frozen=True)
class TurboshaftLimits:
    power_rated_W: Any


@dataclass(frozen=True)
class SimpleTurboshaft:
    power_rated_W: Any = 150000.0
    specific_power_W_kg: float = 2000.0
    thermal_efficiency: float = 0.3
    fuel_lower_heating_value_J_kg: float = 43000000.0

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return TurboshaftLimits(self.power_rated_W)

    def evaluate(self, shaft_power_W):
        # Fuel LHV defines chemical input. Rated power is an independent limit
        # for the caller to enforce; this equation does not impose altitude lapse.
        power_fuel_W = shaft_power_W / self.thermal_efficiency
        return TurboshaftResult(shaft_power_W, power_fuel_W / self.fuel_lower_heating_value_J_kg,
                               power_fuel_W - shaft_power_W)


if __name__ == "__main__":
    print(SimpleTurboshaft().evaluate(100000))
