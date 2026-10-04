"""Constant OCV/resistance battery; positive current discharges the pack."""
from dataclasses import dataclass
from typing import Any
import aerosandbox.numpy as np


@dataclass(frozen=True)
class BatteryResult:
    voltage_V: Any
    current_A: Any
    power_electric_W: Any
    power_loss_W: Any
    power_chemical_W: Any
    soc_next: Any


@dataclass(frozen=True)
class BatteryLimits:
    max_discharge_power_W: Any
    max_charge_power_W: Any
    min_soc: float
    max_soc: float


@dataclass(frozen=True)
class Battery:
    energy_capacity_J: Any = 36000000.0
    voltage_open_circuit_V: Any = 800.0
    resistance_ohm: Any = 0.05
    specific_energy_J_kg: float = 900000.0
    specific_power_W_kg: float = 3000.0
    max_discharge_power_W: Any = 100000.0
    max_charge_power_W: Any = 50000.0
    min_soc: float = 0.2
    max_soc: float = 0.95
    mass_smoothing_kg: Any = None
    thermal_model: Any = None       # Tier 19: LumpedThermalModel or None

    def get_mass(self):
        # Energy capacity alone can undersize a hover pack: both charge and
        # discharge ratings must be supported by the installed pack mass.
        mass_energy_kg = self.energy_capacity_J / self.specific_energy_J_kg
        mass_power_kg = np.maximum(self.max_discharge_power_W, self.max_charge_power_W) / self.specific_power_W_kg
        if self.mass_smoothing_kg is None:
            return np.maximum(mass_energy_kg, mass_power_kg)
        # Optional smooth maximum for optimizers that size energy and power
        # together; it overestimates the exact maximum by at most smoothing x ln 2.
        return np.softmax(mass_energy_kg, mass_power_kg, softness=self.mass_smoothing_kg)

    def get_limits(self):
        return BatteryLimits(self.max_discharge_power_W, self.max_charge_power_W, self.min_soc, self.max_soc)

    def evaluate(self, current_A, soc, duration_s=0.0):
        # Chemical depletion includes resistive heating; terminal power alone
        # would underestimate SOC depletion. Negative current reverses depletion
        # while Joule heating remains positive. Never clamp SOC inside the model.
        voltage_V = self.voltage_open_circuit_V - current_A * self.resistance_ohm
        power_chemical_W = self.voltage_open_circuit_V * current_A
        return BatteryResult(voltage_V, current_A, voltage_V * current_A,
                             current_A**2 * self.resistance_ohm, power_chemical_W,
                             soc - power_chemical_W * duration_s / self.energy_capacity_J)


if __name__ == "__main__":
    print(Battery().evaluate(100, 0.9, 60))
