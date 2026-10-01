"""Forward power transfer; reduction_ratio = input speed / output speed."""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class GearboxResult:
    speed_output_rad_s: Any
    torque_output_Nm: Any
    power_input_W: Any
    power_output_W: Any
    power_loss_W: Any


@dataclass(frozen=True)
class GearboxLimits:
    power_rated_W: Any


@dataclass(frozen=True)
class Gearbox:
    reduction_ratio: Any = 4.0
    efficiency: float = 0.97
    power_rated_W: Any = 100000.0
    specific_power_W_kg: float = 10000.0

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return GearboxLimits(self.power_rated_W)

    def evaluate(self, speed_input_rad_s, torque_input_Nm):
        # Reduction increases torque and decreases speed, with loss deducted
        # from torque so the output speed*torque identity remains exact.
        power_input_W = speed_input_rad_s * torque_input_Nm
        return GearboxResult(speed_input_rad_s / self.reduction_ratio,
                             torque_input_Nm * self.reduction_ratio * self.efficiency,
                             power_input_W, power_input_W * self.efficiency,
                             power_input_W * (1 - self.efficiency))


if __name__ == "__main__":
    print(Gearbox().evaluate(400, 200))
