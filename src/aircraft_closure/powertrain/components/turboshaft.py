"""Turboshaft: fuel-to-shaft power with optional altitude lapse and part-power knockdown.

Defaults are the Tier 1 model: constant thermal efficiency, rated power at
every altitude, mass from specific power. Each refinement is a submodel that
keeps the operating input (shaft power) and adds only the point's atmosphere:

* `lapse_exponent` n: power available = rated x (rho / rho_SL)^n.
* `part_power_knockdown`: efficiency x knockdown(throttle), throttle = shaft
  power / power available. The knockdown is AeroSandbox's (Geiss 2020, via
  `thermal_efficiency_turboshaft`), taken as a ratio so `thermal_efficiency`
  remains the full-power value.
* `mass_kg`: explicit mass (e.g. a caller-side AeroSandbox regression),
  overriding `power_rated_W / specific_power_W_kg`.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
from aerosandbox.library.power_turboshaft import thermal_efficiency_turboshaft

density_sea_level_kg_m3 = asb.Atmosphere(altitude=0).density()


@dataclass(frozen=True)
class TurboshaftResult:
    power_shaft_W: Any
    fuel_flow_kg_s: Any
    power_loss_W: Any


@dataclass(frozen=True)
class TurboshaftLimits:
    power_rated_W: Any


@dataclass(frozen=True)
class SimpleTurboshaft:
    power_rated_W: Any = 150000.0
    specific_power_W_kg: float = 2000.0
    thermal_efficiency: float = 0.3
    fuel_lower_heating_value_J_kg: float = 43000000.0
    lapse_exponent: Any = 0.0
    part_power_knockdown: bool = False
    mass_kg: Any = None

    def get_mass(self):
        return self.mass_kg if self.mass_kg is not None else self.power_rated_W / self.specific_power_W_kg

    def power_available_W(self, atmosphere=None):
        """Maximum shaft power at the point; sea level (rated) when no atmosphere is given."""
        if atmosphere is None:
            return self.power_rated_W
        return self.power_rated_W * (atmosphere.density() / density_sea_level_kg_m3)**self.lapse_exponent

    def get_limits(self, atmosphere=None):
        return TurboshaftLimits(self.power_available_W(atmosphere))

    def thermal_efficiency_at(self, shaft_power_W, atmosphere=None):
        if not self.part_power_knockdown:
            return self.thermal_efficiency
        throttle = shaft_power_W / self.power_available_W(atmosphere)
        # Ratio form: the mass argument cancels, leaving Geiss's cubic in throttle.
        knockdown = (thermal_efficiency_turboshaft(100.0, throttle_setting=throttle)
                     / thermal_efficiency_turboshaft(100.0, throttle_setting=1.0))
        return self.thermal_efficiency * knockdown

    def evaluate(self, shaft_power_W, atmosphere=None):
        # Fuel LHV defines chemical input. Power available is an independent
        # limit for the caller to enforce (operating margins).
        power_fuel_W = shaft_power_W / self.thermal_efficiency_at(shaft_power_W, atmosphere)
        return TurboshaftResult(shaft_power_W, power_fuel_W / self.fuel_lower_heating_value_J_kg,
                                power_fuel_W - shaft_power_W)


if __name__ == "__main__":
    print(SimpleTurboshaft().evaluate(100000))
