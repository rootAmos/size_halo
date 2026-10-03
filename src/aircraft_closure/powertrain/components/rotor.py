"""Proprotor with rotor-speed physics: momentum induced power plus blade profile power.

Coefficients on the rotor disk and tip speed: CT = T / (rho A (Omega R)^2),
CP = P / (rho A (Omega R)^3), lambda = V / (Omega R), sigma = solidity.

    CP = CT lambda + kappa CT lambda_i + (sigma / 2) c_d I(lambda)
    lambda_i = -lambda / 2 + sqrt(lambda^2 / 4 + CT / 2)          (axial momentum)
    I(lambda) = integral_0^1 x^2 sqrt(x^2 + lambda^2) dx           (profile power of sections
                                                                     at sqrt((Omega r)^2 + V^2))
    c_d = d0 + d1 x + d2 x^2 + [airplane mode] (a + b lambda^2),  x = (CT / sigma) / (1 + 3 lambda^2)

x is blade loading referred to the mean section dynamic pressure, so one
drag polar serves hover and airplane mode; the airplane-mode increment
carries blade-twist off-design and hub/spinner effects. Default constants are
the least-squares calibration on full-scale JVX proprotor data,
NASA/TM-2016-219070 (Acree, 2016), plan 016:
  hover (Table D-1, Mtip 0.67-0.68, 58 points): kappa 1.15 (not identifiable
  from the data, standard hover value), polar d0, d1, d2; FM RMS error 0.012;
  airplane mode (Phase II, Tables D-7a/c, lambda 0.26-0.56, 42 points):
  increment a, b; propulsive efficiency RMS error 0.013, max 0.036.
Validity: tip Mach 0.58-0.73, CT/sigma 0.01-0.16, lambda <= 0.56. `advance_ratio_max` (0.60, a slight
extrapolation) is a validity bound for callers. The model has no blade-stall physics, and at large lambda
the (1 + 3 lambda^2) loading reference makes heavily loaded blades look light: cruise power keeps falling
as the rotor slows, so without the bound an optimizer drives lambda to ~0.8+ (plan 016). The bounds
(lambda, CT/sigma), not the physics, set cruise rotor speed; BEM with stall is the deferred remedy.

Pure equations: limits (blade loading, helical tip Mach, tip speed, shaft
power) are returned for the caller to enforce.
"""
from dataclasses import dataclass, replace
from typing import Any

import aerosandbox.numpy as np


def profile_integral(advance_ratio):
    """I(lambda) for lambda > 0 in closed form; I(0) = 1/4 (use `hover=True` paths for exactly zero)."""
    root = np.sqrt(1 + advance_ratio**2)
    return (2 + advance_ratio**2) * root / 8 - advance_ratio**4 / 8 * np.log((1 + root) / advance_ratio)


@dataclass(frozen=True)
class RotorResult:
    thrust_N: Any
    shaft_power_W: Any
    induced_velocity_m_s: Any
    power_residual_W: Any
    blade_loading: Any
    mach_tip_helical: Any
    advance_ratio: Any


@dataclass(frozen=True)
class RotorLimits:
    max_shaft_power_W: Any
    blade_loading_max: Any
    mach_tip_helical_max: Any
    speed_tip_max_m_s: Any
    advance_ratio_max: Any


@dataclass(frozen=True)
class MomentumProfileRotor:
    area_disk_m2: Any = 45.6
    solidity: Any = 0.1138
    mass_kg: Any = 300.0
    max_shaft_power_W: Any = 1.0e6
    kappa_hover: Any = 1.15
    kappa_airplane: Any = 1.15
    drag_polar: tuple = (0.018497991193502324, -0.22163949587830145, 1.066254638364128)
    drag_increment_airplane: tuple = (0.0016871232864198954, 0.018759922413934375)
    blade_loading_max: Any = 0.14
    mach_tip_helical_max: Any = 0.80
    advance_ratio_max: Any = 0.60
    speed_tip_max_m_s: Any = None
    airplane_mode: bool = False

    def radius_m(self):
        return np.sqrt(self.area_disk_m2 / np.pi)

    def in_airplane_mode(self):
        return replace(self, airplane_mode=True)

    def get_mass(self):
        return self.mass_kg

    def get_limits(self):
        return RotorLimits(self.max_shaft_power_W, self.blade_loading_max, self.mach_tip_helical_max,
                           self.speed_tip_max_m_s, self.advance_ratio_max)

    def section_drag(self, blade_loading, advance_ratio):
        d0, d1, d2 = self.drag_polar
        x = blade_loading / (1 + 3 * advance_ratio**2)
        drag = d0 + d1 * x + d2 * x**2
        if self.airplane_mode:
            a, b = self.drag_increment_airplane
            drag = drag + a + b * advance_ratio**2
        return drag

    def evaluate(self, axial_velocity_m_s, atmosphere, *, thrust_N, speed_rad_s):
        density_kg_m3 = atmosphere.density()
        speed_tip_m_s = speed_rad_s * self.radius_m()
        scale_thrust_N = density_kg_m3 * self.area_disk_m2 * speed_tip_m_s**2
        thrust_coefficient = thrust_N / scale_thrust_N
        blade_loading = thrust_coefficient / self.solidity
        if self.airplane_mode:
            advance_ratio = axial_velocity_m_s / speed_tip_m_s
            inflow_induced = -advance_ratio / 2 + np.sqrt(advance_ratio**2 / 4 + thrust_coefficient / 2)
            power_coefficient = (thrust_coefficient * advance_ratio + self.kappa_airplane * thrust_coefficient * inflow_induced
                                 + self.solidity / 2 * self.section_drag(blade_loading, advance_ratio)
                                 * profile_integral(advance_ratio))
        else:
            # Hover (zero axial velocity): I(0) = 1/4 exactly, no 0 x log(1/0) evaluation.
            advance_ratio = 0.0
            inflow_induced = np.sqrt(thrust_coefficient / 2)
            power_coefficient = (self.kappa_hover * thrust_coefficient * inflow_induced
                                 + self.solidity / 8 * self.section_drag(blade_loading, 0.0))
        shaft_power_W = power_coefficient * scale_thrust_N * speed_tip_m_s
        mach_tip_helical = np.sqrt(speed_tip_m_s**2 + axial_velocity_m_s**2) / atmosphere.speed_of_sound()
        return RotorResult(thrust_N, shaft_power_W, inflow_induced * speed_tip_m_s, 0.0, blade_loading,
                           mach_tip_helical, advance_ratio)


if __name__ == "__main__":
    import aerosandbox as asb
    rotor = MomentumProfileRotor()
    hover = rotor.evaluate(0.0, asb.Atmosphere(altitude=0), thrust_N=60000.0, speed_rad_s=60.0)
    print(hover)
