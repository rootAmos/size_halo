"""Lumped-capacitance thermal submodel for a component (Tier 19, plan 028).

One temperature per component, one thermal capacitance C [J/K] and one
thermal resistance R [K/W] to a coolant held at `temperature_coolant_C`:

    C dT/dt = Q - (T - T_c) / R.

For a constant heat Q over a duration dt from T_0 the exact solution is

    T(dt) = T_ss + (T_0 - T_ss) e^(-dt / tau),  T_ss = T_c + Q R,  tau = R C

(Incropera and DeWitt, ch. 5). The temperature moves monotonically from T_0
towards T_ss, so the largest temperature in an interval is at one of its ends:
constraining every end constrains the whole history.

The owning component supplies C and R (the discipline layer derives them from
its mass and continuous rating); this submodel holds the material and limit
data only. A start temperature of None means no history: the steady state.

The heat the component passes to its coolant is (T - T_c) / R, not its loss:
the thermal mass absorbs the difference. Over an interval its mean is
(T_mean - T_c) / R with the interval-mean temperature from
`temperature_mean_C`; at the end of the interval it is (T_end - T_c) / R.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class LumpedThermalModel:
    specific_heat_J_kg_K: Any = 500.0      # effective, whole machine (copper ~385, steel ~460, aluminium ~900)
    temperature_max_C: Any = 150.0         # mean winding limit (class H insulation: 180 C hot spot)
    temperature_coolant_C: Any = 60.0      # coolant supplied by the heat exchanger

    def temperature_steady_C(self, power_loss_W, resistance_K_W):
        return self.temperature_coolant_C + power_loss_W * resistance_K_W

    @staticmethod
    def time_constant_s(capacity_J_K, resistance_K_W):
        return capacity_J_K * resistance_K_W

    def temperature_end_C(self, power_loss_W, duration_s, capacity_J_K, resistance_K_W, temperature_start_C=None):
        """Temperature after holding `power_loss_W` for `duration_s` from `temperature_start_C` (None: steady)."""
        temperature_steady_C = self.temperature_steady_C(power_loss_W, resistance_K_W)
        if temperature_start_C is None:
            return temperature_steady_C
        if isinstance(duration_s, (int, float)) and duration_s == 0:
            return temperature_start_C + 0 * power_loss_W
        decay = np.exp(-duration_s / self.time_constant_s(capacity_J_K, resistance_K_W))
        return temperature_steady_C + (temperature_start_C - temperature_steady_C) * decay

    def temperature_mean_C(self, power_loss_W, duration_s, capacity_J_K, resistance_K_W, temperature_start_C=None):
        """Interval-mean temperature: T_ss + (T_0 - T_ss) g, g = (tau/dt)(1 - e^(-dt/tau)) (1 at zero duration).

        The mean heat flow to the coolant is (T_mean - T_c) / R = loss - C (T_end - T_0) / dt.
        """
        temperature_steady_C = self.temperature_steady_C(power_loss_W, resistance_K_W)
        if temperature_start_C is None:
            return temperature_steady_C
        if isinstance(duration_s, (int, float)) and duration_s == 0:
            return temperature_start_C + 0 * power_loss_W
        tau_s = self.time_constant_s(capacity_J_K, resistance_K_W)
        fraction = tau_s / duration_s * (1 - np.exp(-duration_s / tau_s))
        return temperature_steady_C + (temperature_start_C - temperature_steady_C) * fraction


if __name__ == "__main__":
    model = LumpedThermalModel()
    print(model.temperature_end_C(30000.0, 60.0, 1e5, 0.003, 60.0))
