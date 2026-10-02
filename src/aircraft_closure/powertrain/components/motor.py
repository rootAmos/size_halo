"""Motoring quadrant only: speed and torque >= 0, voltage > 0.

Default losses: McDonald parametric model (AIAA 2015-1676); the quadratic
`SimpleMotorLossModel` remains available as the simplest model.
"""
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
