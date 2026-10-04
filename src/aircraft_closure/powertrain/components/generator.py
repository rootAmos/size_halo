"""Generating quadrant; positive shaft input and electrical output."""
from dataclasses import dataclass, field
from typing import Any
from .motor import McDonaldMotorLossModel, MotorLimits


@dataclass(frozen=True)
class GeneratorResult:
    power_shaft_W: Any
    power_electric_W: Any
    current_A: Any
    power_loss_W: Any


@dataclass(frozen=True)
class Generator:
    power_rated_W: Any = 100000.0
    specific_power_W_kg: float = 4000.0
    max_speed_rad_s: Any = 1000.0
    max_torque_Nm: Any = 500.0
    min_voltage_V: Any = 400.0
    max_voltage_V: Any = 900.0
    loss_model: Any = field(default_factory=McDonaldMotorLossModel)
    mass_model: Any = None          # e.g. TorqueDensityMassModel(); None: power / specific power
    thermal_model: Any = None       # Tier 19: LumpedThermalModel or None (see Motor)

    def get_mass(self):
        if self.mass_model is not None:
            return self.mass_model.mass_kg(self)
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return MotorLimits(self.power_rated_W, self.max_speed_rad_s, self.max_torque_Nm,
                           self.min_voltage_V, self.max_voltage_V)

    def evaluate(self, speed_rad_s, torque_Nm, voltage_V):
        # Generator current is positive out of the machine, opposite to motor
        # consumption. The caller requires shaft input >= the modeled losses.
        power_shaft_W = speed_rad_s * torque_Nm
        power_loss_W = self.loss_model.evaluate(speed_rad_s, torque_Nm, voltage_V)
        power_electric_W = power_shaft_W - power_loss_W
        return GeneratorResult(power_shaft_W, power_electric_W, power_electric_W / voltage_V, power_loss_W)


if __name__ == "__main__":
    print(Generator().evaluate(400, 200, 800))
