"""Power converters (Tier 15): the motor/generator inverter and an optional DC/DC converter.

Loss model (`ConverterLossModel`): at the rated point (power P_r, DC voltage V_r) the loss is
L_r = P_r (1 - eta_r) / eta_r, split into a fixed part (gate drive, control, capacitive), a switching part
proportional to |P| (switching energy ~ V I per event, with I = P / V) and a conduction part proportional
to I^2 = (P / V)^2:

    P_loss = L_r [f_0 + f_s |P| / P_r + f_c (P / P_r)^2 (V_r / V)^2],   f_0 = 1 - f_s - f_c.

So efficiency is exactly eta_r at (P_r, V_r), peaks at |P| / P_r = sqrt(f_0 / f_c), and falls as the DC
voltage sags (more current for the same power). |P| is smoothed (sqrt(P^2 + (eps P_r)^2)) so the model is
differentiable through zero power. Defaults: 98.5 % at rated (SiC; NASA EAP converter goal 99 %, Jansen et
al., AIAA 2017-4701), split 0.5 conduction / 0.4 switching / 0.1 fixed (assumed).
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class ConverterLossModel:
    efficiency_rated: Any = 0.985
    fraction_loss_conduction: Any = 0.5
    fraction_loss_switching: Any = 0.4
    voltage_rated_V: Any = 800.0
    fraction_power_smoothing: float = 1e-3

    def evaluate(self, power_W, voltage_V, power_rated_W):
        loss_rated_W = power_rated_W * (1 - self.efficiency_rated) / self.efficiency_rated
        fraction_fixed = 1 - self.fraction_loss_conduction - self.fraction_loss_switching
        power_abs_W = np.sqrt(power_W**2 + (self.fraction_power_smoothing * power_rated_W)**2)
        return loss_rated_W * (fraction_fixed + self.fraction_loss_switching * power_abs_W / power_rated_W
                               + self.fraction_loss_conduction * (power_W / power_rated_W)**2
                               * (self.voltage_rated_V / voltage_V)**2)


@dataclass(frozen=True)
class InverterResult:
    power_ac_W: Any          # positive: DC -> AC (motoring); negative: AC -> DC (active rectifier)
    power_dc_W: Any          # = power_ac_W + power_loss_W
    power_loss_W: Any
    current_dc_A: Any        # positive into the DC terminals


@dataclass(frozen=True)
class InverterLimits:
    power_rated_W: Any
    min_voltage_V: Any
    max_voltage_V: Any       # derated semiconductor blocking voltage


@dataclass(frozen=True)
class Inverter:
    """Three-phase two-level DC/AC converter, also used as the generator's active rectifier.

    Mass = rated AC power / specific power (default 20 kW/kg, assumed between the NASA 19 kW/kg goal and
    recent SiC demonstrators). The DC-link limit is the blocking voltage x a derating factor (0.75 for
    cosmic-ray single-event-burnout margin at altitude; 1,200 V devices -> 900 V).
    """
    power_rated_W: Any = 500000.0
    specific_power_W_kg: Any = 20000.0
    voltage_blocking_V: Any = 1200.0
    factor_derating_voltage: Any = 0.75
    min_voltage_V: Any = 400.0
    loss_model: Any = field(default_factory=ConverterLossModel)

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return InverterLimits(self.power_rated_W, self.min_voltage_V,
                              self.voltage_blocking_V * self.factor_derating_voltage)

    def evaluate(self, power_ac_W, voltage_dc_V):
        power_loss_W = self.loss_model.evaluate(power_ac_W, voltage_dc_V, self.power_rated_W)
        power_dc_W = power_ac_W + power_loss_W
        return InverterResult(power_ac_W, power_dc_W, power_loss_W, power_dc_W / voltage_dc_V)


@dataclass(frozen=True)
class DcDcConverterResult:
    power_input_W: Any       # positive: battery side -> bus side
    power_output_W: Any      # = power_input_W - power_loss_W
    power_loss_W: Any
    current_input_A: Any
    current_output_A: Any


@dataclass(frozen=True)
class DcDcConverterLimits:
    power_rated_W: Any
    min_voltage_input_V: Any
    max_voltage_input_V: Any
    voltage_output_V: Any


@dataclass(frozen=True)
class DcDcConverter:
    """Bidirectional DC/DC converter that regulates the bus at `voltage_output_V` (optional, off by default).

    Mass = rated power / specific power (12 kW/kg, assumed); losses as `ConverterLossModel` with the input
    (battery) voltage as the conduction voltage, 98 % at rated (assumed).
    """
    power_rated_W: Any = 500000.0
    specific_power_W_kg: Any = 12000.0
    voltage_output_V: Any = 800.0
    min_voltage_input_V: Any = 400.0
    max_voltage_input_V: Any = 900.0
    loss_model: Any = field(default_factory=lambda: ConverterLossModel(efficiency_rated=0.98))

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return DcDcConverterLimits(self.power_rated_W, self.min_voltage_input_V, self.max_voltage_input_V,
                                   self.voltage_output_V)

    def evaluate(self, power_input_W, voltage_input_V):
        power_loss_W = self.loss_model.evaluate(power_input_W, voltage_input_V, self.power_rated_W)
        power_output_W = power_input_W - power_loss_W
        return DcDcConverterResult(power_input_W, power_output_W, power_loss_W, power_input_W / voltage_input_V,
                                   power_output_W / self.voltage_output_V)
