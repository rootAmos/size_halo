"""Axial actuator disk, normal working state only; no RPM or hidden solve."""
from dataclasses import dataclass, replace
from typing import Any
import aerosandbox as asb
import aerosandbox.numpy as np


@dataclass(frozen=True)
class PropulsorResult:
    thrust_N: Any
    shaft_power_W: Any
    induced_velocity_m_s: Any
    power_residual_W: Any


@dataclass(frozen=True)
class PropulsorLimits:
    max_shaft_power_W: Any


@dataclass(frozen=True)
class ActuatorDiskPropulsor:
    """`coefficient_of_performance` is the hover figure of merit. Optional:
    `coefficient_of_performance_airplane` for propeller-mode flight (None = the
    same value) and `speed_tip_max_m_s`, a tip-speed bound callers enforce."""
    area_disk_m2: Any = 10.0
    mass_kg: Any = 30.0
    coefficient_of_performance: float = 0.8
    max_shaft_power_W: Any = 100000.0
    coefficient_of_performance_airplane: Any = None
    speed_tip_max_m_s: Any = None

    def radius_m(self):
        return np.sqrt(self.area_disk_m2 / np.pi)

    def in_airplane_mode(self):
        """The same rotor with its propeller-mode coefficient."""
        if self.coefficient_of_performance_airplane is None:
            return self
        return replace(self, coefficient_of_performance=self.coefficient_of_performance_airplane)

    def get_mass(self):
        return self.mass_kg

    def get_limits(self):
        return PropulsorLimits(self.max_shaft_power_W)

    def evaluate(self, axial_velocity_m_s, atmosphere, *, shaft_power_W=None,
                 induced_velocity_m_s=None, thrust_N=None):
        """Supply thrust, or power AND externally solved induced velocity.

        In power mode caller enforces power_residual_W == 0. Valid domain:
        axial_velocity >= 0, thrust/power/induced_velocity >= 0, density > 0.
        """
        rho_kg_m3 = atmosphere.density()
        if thrust_N is not None:
            if shaft_power_W is not None or induced_velocity_m_s is not None:
                raise ValueError("Supply thrust alone or power with induced velocity.")
            # Rationalized positive root of T = 2*rho*A*vi*(V + vi).
            # It avoids subtracting nearly equal numbers at high axial speeds.
            velocity_root_m_s = np.sqrt(axial_velocity_m_s**2 +
                                        2 * thrust_N / (rho_kg_m3 * self.area_disk_m2))
            denominator_m_s = velocity_root_m_s + axial_velocity_m_s
            # The exact zero-thrust/zero-airspeed point has vi=0. Protect only
            # that removable 0/0 singularity; do not regularize nonzero loads.
            denominator_safe_m_s = np.where(denominator_m_s == 0, 1, denominator_m_s)
            induced_velocity_m_s = thrust_N / (rho_kg_m3 * self.area_disk_m2 * denominator_safe_m_s)
            shaft_power_W = thrust_N * (axial_velocity_m_s + induced_velocity_m_s) / self.coefficient_of_performance
            return PropulsorResult(thrust_N, shaft_power_W, induced_velocity_m_s, 0.0)
        if shaft_power_W is None or induced_velocity_m_s is None:
            raise ValueError("Power mode requires an explicit induced velocity coupling variable.")
        thrust_N = 2 * rho_kg_m3 * self.area_disk_m2 * induced_velocity_m_s * (axial_velocity_m_s + induced_velocity_m_s)
        # An equality residual leaves inverse momentum theory in the aircraft's
        # Opti system rather than running a private root finder in this component.
        power_required_W = thrust_N * (axial_velocity_m_s + induced_velocity_m_s) / self.coefficient_of_performance
        return PropulsorResult(thrust_N, shaft_power_W, induced_velocity_m_s, shaft_power_W - power_required_W)


if __name__ == "__main__":
    print(ActuatorDiskPropulsor().evaluate(0, asb.Atmosphere(altitude=0), thrust_N=5000))
