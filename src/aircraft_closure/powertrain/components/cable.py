"""DC feeder cable (Tier 15): conductor sized by current density, insulation by partial discharge.

A feeder is `count_conductors` round conductors (2 for a +/- DC pair) of length `length_m`.

- Conductor area A = max_current / J (design current density of the material).
- Loop resistance R = resistivity x length x count / A; loss R I^2 (>= 0 for either current sign);
  voltage drop R I.
- Insulation wall t = smooth max(t_min, t_PD), where t_PD makes the partial-discharge inception voltage at
  the design altitude equal to factor_safety x the peak voltage (`PartialDischargeModel`). Insulation mass
  is the annulus pi (2 r t + t^2) x length x count x density, so it grows with voltage (about V^2.2).
- Accessories (terminations, shield, clamps) add a fraction of the conductor plus insulation mass.

Partial discharge (`PartialDischargeModel`): Dakin's empirical inception voltage for an insulated
conductor in air, PDIV = 163 (t / eps_r)^0.46 V peak with t in micrometres at sea level, times
(p / p0)^0.5 for the fall of air breakdown with pressure (Paschen; exponent assumed). Inverting it gives
the closed-form wall thickness, so no iteration is needed.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np


@dataclass(frozen=True)
class ConductorMaterial:
    resistivity_ohm_m: float
    density_kg_m3: float
    current_density_A_m2: float


def aluminium_conductor():
    """Aluminium at about 80 C (3.4e-8 ohm m), 2,700 kg/m3, 3 A/mm2 design current density (assumed)."""
    return ConductorMaterial(resistivity_ohm_m=3.4e-8, density_kg_m3=2700.0, current_density_A_m2=3.0e6)


def copper_conductor():
    """Copper at about 80 C (2.1e-8 ohm m), 8,960 kg/m3, 4 A/mm2 design current density (assumed)."""
    return ConductorMaterial(resistivity_ohm_m=2.1e-8, density_kg_m3=8960.0, current_density_A_m2=4.0e6)


@dataclass(frozen=True)
class InsulationMaterial:
    """Default: fluoropolymer (ETFE-like) wall, eps_r 2.5, 1,700 kg/m3, 0.25 mm minimum (mechanical)."""
    permittivity_relative: float = 2.5
    density_kg_m3: float = 1700.0
    thickness_min_m: float = 0.25e-3


@dataclass(frozen=True)
class PartialDischargeModel:
    coefficient_dakin_V: float = 163.0
    exponent_dakin: float = 0.46
    thickness_reference_m: float = 1e-6     # Dakin's formula takes t in micrometres
    exponent_pressure: float = 0.5
    factor_safety: float = 1.5

    def voltage_inception_V(self, thickness_m, permittivity_relative, pressure_ratio=1.0):
        return (self.coefficient_dakin_V * (thickness_m / (permittivity_relative * self.thickness_reference_m))
                ** self.exponent_dakin * pressure_ratio**self.exponent_pressure)

    def thickness_required_m(self, voltage_peak_V, permittivity_relative, pressure_ratio=1.0):
        """Wall at which PDIV(pressure) = factor_safety x voltage_peak_V."""
        voltage_sea_level_V = self.factor_safety * voltage_peak_V / pressure_ratio**self.exponent_pressure
        return (permittivity_relative * self.thickness_reference_m
                * (voltage_sea_level_V / self.coefficient_dakin_V) ** (1 / self.exponent_dakin))


def pressure_ratio(altitude_m, temperature_offset_K=0.0):
    return asb.Atmosphere(altitude=altitude_m, temperature_deviation=temperature_offset_K).pressure() / 101325.0


@dataclass(frozen=True)
class CableResult:
    current_A: Any
    power_loss_W: Any
    voltage_drop_V: Any


@dataclass(frozen=True)
class CableLimits:
    max_current_A: Any
    max_voltage_V: Any
    voltage_inception_V: Any      # PDIV at the design altitude


@dataclass(frozen=True)
class Cable:
    length_m: Any = 10.0
    max_current_A: Any = 1000.0             # design (thermal) current; sets the conductor area
    max_voltage_V: Any = 900.0              # peak operating voltage the insulation is designed for
    altitude_design_m: Any = 4000.0         # partial-discharge design altitude (the ceiling)
    count_conductors: int = 2
    conductor: Any = field(default_factory=aluminium_conductor)
    insulation: Any = field(default_factory=InsulationMaterial)
    partial_discharge: Any = field(default_factory=PartialDischargeModel)
    fraction_mass_accessories: Any = 0.2
    smoothing_thickness_m: Any = 1e-5

    def area_conductor_m2(self):
        return self.max_current_A / self.conductor.current_density_A_m2

    def thickness_insulation_m(self):
        required_m = self.partial_discharge.thickness_required_m(
            self.max_voltage_V, self.insulation.permittivity_relative, pressure_ratio(self.altitude_design_m))
        return np.softmax(self.insulation.thickness_min_m, required_m, softness=self.smoothing_thickness_m)

    def resistance_ohm(self):
        return self.conductor.resistivity_ohm_m * self.length_m * self.count_conductors / self.area_conductor_m2()

    def voltage_inception_V(self, pressure_ratio_operating=None):
        ratio = pressure_ratio(self.altitude_design_m) if pressure_ratio_operating is None else pressure_ratio_operating
        return self.partial_discharge.voltage_inception_V(self.thickness_insulation_m(),
                                                          self.insulation.permittivity_relative, ratio)

    def get_mass(self):
        area_m2 = self.area_conductor_m2()
        radius_m = np.sqrt(area_m2 / np.pi)
        thickness_m = self.thickness_insulation_m()
        mass_per_length_kg = (area_m2 * self.conductor.density_kg_m3
                              + np.pi * (2 * radius_m * thickness_m + thickness_m**2) * self.insulation.density_kg_m3)
        return (1 + self.fraction_mass_accessories) * self.count_conductors * self.length_m * mass_per_length_kg

    def get_limits(self):
        return CableLimits(self.max_current_A, self.max_voltage_V, self.voltage_inception_V())

    def evaluate(self, current_A):
        resistance_ohm = self.resistance_ohm()
        return CableResult(current_A, resistance_ohm * current_A**2, resistance_ohm * current_A)
