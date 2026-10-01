"""Motoring quadrant only: speed and torque >= 0, voltage > 0."""
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class SimpleMotorLossModel:
    """Quadratic torque/speed losses; coefficients require calibration.

    Voltage is accepted so later loss maps can retain this operating interface.
    This approximation has no voltage-dependent losses or inverter model.
    """
    torque_loss_W_Nm2: float = 0.02
    speed_loss_W_s2_rad2: float = 0.001

    def evaluate(self, speed_rad_s, torque_Nm, voltage_V):
        return self.torque_loss_W_Nm2 * torque_Nm**2 + self.speed_loss_W_s2_rad2 * speed_rad_s**2


@dataclass(frozen=True)
class MotorResult:
    power_shaft_W: Any
    power_electric_W: Any
    current_A: Any
    power_loss_W: Any


@dataclass(frozen=True)
class MotorLimits:
    power_rated_W: Any
    max_speed_rad_s: Any
    max_torque_Nm: Any
    min_voltage_V: Any
    max_voltage_V: Any


@dataclass(frozen=True)
class Motor:
    power_rated_W: Any = 100000.0
    specific_power_W_kg: float = 5000.0
    max_speed_rad_s: Any = 1000.0
    max_torque_Nm: Any = 500.0
    min_voltage_V: Any = 400.0
    max_voltage_V: Any = 900.0
    loss_model: SimpleMotorLossModel = field(default_factory=SimpleMotorLossModel)

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return MotorLimits(self.power_rated_W, self.max_speed_rad_s, self.max_torque_Nm,
                           self.min_voltage_V, self.max_voltage_V)

    def evaluate(self, speed_rad_s, torque_Nm, voltage_V):
        power_shaft_W = speed_rad_s * torque_Nm
        # Electrical input supplies both useful shaft output and internal losses.
        power_loss_W = self.loss_model.evaluate(speed_rad_s, torque_Nm, voltage_V)
        power_electric_W = power_shaft_W + power_loss_W
        return MotorResult(power_shaft_W, power_electric_W, power_electric_W / voltage_V, power_loss_W)


if __name__ == "__main__":
    print(Motor().evaluate(400, 200, 800))
