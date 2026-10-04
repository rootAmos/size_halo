"""Forward power transfer; reduction_ratio = input speed / output speed."""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class GearStageModel:
    """Stage count, efficiency and relative mass of a gear train from its overall ratio (plan 033).

    The ratio is fast / slow, at least 1, for reductions and step-ups alike. With x = ln(ratio) / ln(ratio_max_stage),
    the stage count is a smooth ceil(x), at least 1:
    - staircase: 1 + sum_k sigmoid((x - k) / width - 3), k = 1..count_stages_max. Each step is centred 3 widths
      past the integer, so count_stages(ratio_max_stage) = 1 + sigmoid(-3) = 1.047;
    - relaxed (`staircase` False): 1 + width softplus((x - 1) / width), a smooth max(1, x).

    Efficiency is efficiency_fixed (1 - loss_stage)^n:
    - loss_stage = 1 %, about 0.5 % per mesh for a mesh pair. Measured values: NASA TP-2795 measured a planetary
      stage at 99.44-99.75 %; NASA TM-81426 found spur gears above 98 %;
    - efficiency_fixed covers bearings, seals and churning.
    This gives 0.980 / 0.970 / 0.961 for 1 / 2 / 3 stages; the Tier 13 constant was 0.97.

    Mass factor 1 + fraction_mass_stage (n - 1) is relative to a one-stage drive. An upstream stage carries 1/r of the
    torque, (1/5)^0.78 ~ 0.28 of the output stage with AFDD's power exponent, plus its own bearings, housing and
    lubrication; 0.3 is used.
    Ratio per stage: simple planetary 3:1-10:1, the OH-58 planetary 4.67:1, H3X's single epicyclic 6.7:1.
    5:1 is taken for the mixed bevel, spur and planetary trains of a tiltrotor drive.
    """
    ratio_max_stage: Any = 5.0
    loss_stage: Any = 0.01
    efficiency_fixed: Any = 0.99
    fraction_mass_stage: Any = 0.3
    staircase: bool = True
    width_step: Any = 0.02
    width_relaxed: Any = 0.05
    count_stages_max: int = 6

    def count_stages(self, ratio):
        x = np.log(ratio) / np.log(self.ratio_max_stage)
        if self.staircase:
            # sigmoid(z) = (1 + tanh(z / 2)) / 2, which does not overflow far from the step.
            return 1 + sum(0.5 * (1 + np.tanh(0.5 * ((x - k) / self.width_step - 3)))
                           for k in range(1, self.count_stages_max + 1))
        return 1 + self.width_relaxed * np.softplus((x - 1) / self.width_relaxed)

    def efficiency(self, ratio):
        return self.efficiency_fixed * (1 - self.loss_stage) ** self.count_stages(ratio)

    def mass_factor(self, ratio):
        return 1 + self.fraction_mass_stage * (self.count_stages(ratio) - 1)


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
