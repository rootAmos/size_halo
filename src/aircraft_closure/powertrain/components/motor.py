"""Motoring quadrant only: speed and torque >= 0, voltage > 0.

Default losses: McDonald parametric model (AIAA 2015-1676); the quadratic
`SimpleMotorLossModel` remains available as the simplest model.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox.numpy as np


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
class McDonaldLossCoefficients:
    constant_W: Any
    linear_W_s_rad: Any
    cubic_W_s3_rad3: Any
    torque_W_Nm2: Any


@dataclass(frozen=True)
class McDonaldMotorLossModel:
    """Parametric loss model of McDonald, AIAA 2015-1676, eqs. 1-2.

    P_L = C0 + C1 w + C2 w^3 + C3 Q^2, with coefficients fixed by the speed,
    torque and value of peak efficiency and the parasite loss ratio k0 in
    [0, 1]. C0 is a constant (standstill) loss. Voltage is accepted to keep
    the machine interface stable; losses do not depend on it.
    """
    speed_peak_efficiency_rad_s: Any = 400.0
    torque_peak_efficiency_Nm: Any = 200.0
    efficiency_peak: Any = 0.96
    parasite_loss_ratio: Any = 0.5

    def coefficients(self):
        speed_rad_s = self.speed_peak_efficiency_rad_s
        torque_Nm = self.torque_peak_efficiency_Nm
        loss_ratio = (1 - self.efficiency_peak) / self.efficiency_peak
        constant_W = self.parasite_loss_ratio * speed_rad_s * torque_Nm * loss_ratio / 6
        return McDonaldLossCoefficients(
            constant_W=constant_W,
            linear_W_s_rad=-3 * constant_W / (2 * speed_rad_s) + torque_Nm * loss_ratio / 4,
            cubic_W_s3_rad3=constant_W / (2 * speed_rad_s**3) + torque_Nm * loss_ratio / (4 * speed_rad_s**2),
            torque_W_Nm2=speed_rad_s * loss_ratio / (2 * torque_Nm),
        )

    def evaluate(self, speed_rad_s, torque_Nm, voltage_V):
        c = self.coefficients()
        return (c.constant_W + c.linear_W_s_rad * speed_rad_s + c.cubic_W_s3_rad3 * speed_rad_s**3
                + c.torque_W_Nm2 * torque_Nm**2)


@dataclass(frozen=True)
class TorqueDensityMassModel:
    """Machine mass from peak torque, with a specific-power cap at high speed (Tier 13, plan 018).

    mass = softmax(max_torque / torque_density, power_rated / specific_power_max): electromagnetic
    machine mass scales with rotor volume, i.e. torque, until high speed makes a power-density limit bind.
    Defaults: about 15 N.m/kg (magniX magni650: 3,216 N.m peak, 206 kg; magni350: 1,608 N.m, 128 kg; both
    including inverters and HV cables), and 10 kW/kg as a conventional high-speed cap (NASA's partially
    superconducting HEMM targets 16 kW/kg electromagnetic). The smooth maximum overestimates the exact one by
    at most smoothing_kg x ln 2.
    """
    torque_density_Nm_kg: Any = 15.0
    specific_power_max_W_kg: Any = 10000.0
    smoothing_kg: Any = 2.0

    def mass_kg(self, machine):
        return np.softmax(machine.max_torque_Nm / self.torque_density_Nm_kg,
                          machine.power_rated_W / self.specific_power_max_W_kg, softness=self.smoothing_kg)


@dataclass(frozen=True)
class DatabaseMassModel:
    """Machine mass from continuous torque, with torque density falling with base speed (plan 033).

    The machine's continuous torque is T_cont = ratio_torque_continuous_peak x max_torque_Nm. Its base speed is
    w = power_rated_W / T_cont. Bare-machine mass is

        softmax(T_cont / tau(w), power_rated_W / specific_power_max_W_kg),
        tau(w) = torque_density_ref_Nm_kg (w / speed_ref_rad_s)^(-exponent_speed),

    so mass falls as w^(exponent_speed - 1) at fixed power until the specific-power cap binds.

    Defaults:
    - tau_ref and the exponent are the least-squares fit to the 15 bare (or unknown-inverter) machines with
      published continuous ratings in data/machines/aerospace_motors.csv (`machine_database.fit_torque_density`);
      the RMS log residual is 0.37 (factor 1.45); the residuals are in plan 033;
    - ratio 0.5 matches the Halo `rubber_machine` ratings (P_rated = 1.25 w_hat Q_hat, T_max = 2.5 Q_hat, so
      w = w_hat);
    - the 20 kW/kg cap is the Tier 15 bare-machine assumption (the data do not constrain it).

    `specific_power_inverter_W_kg` = None gives a bare machine (the inverter is a separate component, Tier 15);
    a value adds power_rated_W / specific_power_inverter_W_kg (integrated machine).

    Stacking: a machine of identical axial stacks (Evolito D250/D500 units), each carrying at most
    `torque_continuous_max_stack_Nm`. The stack count is relaxed (continuous): count = T_cont / T_stack_max. Fitted
    mass is linear in torque at fixed speed, so n stacks weigh n times one stack. `mass_overhead_stack_kg` adds a
    housing per stack; 0 by default, because the fit is to whole machines whose housings are inside tau.
    """
    torque_density_ref_Nm_kg: Any = 11.79
    speed_ref_rad_s: Any = 500.0
    exponent_speed: Any = 0.271
    specific_power_max_W_kg: Any = 20000.0
    ratio_torque_continuous_peak: Any = 0.5
    specific_power_inverter_W_kg: Any = None
    torque_continuous_max_stack_Nm: Any = None
    mass_overhead_stack_kg: Any = 0.0
    smoothing_kg: Any = 2.0

    def torque_density_Nm_kg(self, speed_base_rad_s):
        return self.torque_density_ref_Nm_kg * (speed_base_rad_s / self.speed_ref_rad_s) ** (-self.exponent_speed)

    def torque_continuous_Nm(self, machine):
        return self.ratio_torque_continuous_peak * machine.max_torque_Nm

    def speed_base_rad_s(self, machine):
        return machine.power_rated_W / self.torque_continuous_Nm(machine)

    def count_stacks(self, machine):
        """Relaxed stack count, T_cont / T_stack_max; 1 without a per-stack torque limit."""
        if self.torque_continuous_max_stack_Nm is None:
            return 1.0
        return self.torque_continuous_Nm(machine) / self.torque_continuous_max_stack_Nm

    def mass_bare_kg(self, machine):
        torque_Nm = self.torque_continuous_Nm(machine)
        mass_torque_kg = torque_Nm / self.torque_density_Nm_kg(self.speed_base_rad_s(machine))
        mass_kg = np.softmax(mass_torque_kg, machine.power_rated_W / self.specific_power_max_W_kg,
                             softness=self.smoothing_kg)
        return mass_kg + self.mass_overhead_stack_kg * self.count_stacks(machine)

    def mass_inverter_kg(self, machine):
        if self.specific_power_inverter_W_kg is None:
            return 0.0
        return machine.power_rated_W / self.specific_power_inverter_W_kg

    def mass_kg(self, machine):
        return self.mass_bare_kg(machine) + self.mass_inverter_kg(machine)


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
    loss_model: Any = field(default_factory=McDonaldMotorLossModel)
    mass_model: Any = None
    # Tier 19: LumpedThermalModel or None. With one, the power rating is a continuous (thermal) rating that
    # short peaks may exceed; callers then constrain temperature instead of rated power (plan 028).
    thermal_model: Any = None

    def get_mass(self):
        if self.mass_model is not None:
            return self.mass_model.mass_kg(self)
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


def rubber_machine(machine_type, speed_peak_efficiency_rad_s, torque_peak_efficiency_Nm, efficiency_peak=0.96,
                   parasite_loss_ratio=0.5, torque_ratio=2.0, power_ratio=2.0, speed_ratio=2.0, **machine_kwargs):
    """Motor or Generator from the seven McDonald parameters (eqs. 2 and 4).

    Ratings follow the peak-efficiency point: Q_rated = kQ Q_hat,
    P_rated = kP w_hat Q_hat, w_limit = kw w_hat.
    """
    loss_model = McDonaldMotorLossModel(speed_peak_efficiency_rad_s, torque_peak_efficiency_Nm,
                                        efficiency_peak, parasite_loss_ratio)
    return machine_type(power_rated_W=power_ratio * speed_peak_efficiency_rad_s * torque_peak_efficiency_Nm,
                        max_torque_Nm=torque_ratio * torque_peak_efficiency_Nm,
                        max_speed_rad_s=speed_ratio * speed_peak_efficiency_rad_s,
                        loss_model=loss_model, **machine_kwargs)


if __name__ == "__main__":
    print(Motor().evaluate(400, 200, 800))
