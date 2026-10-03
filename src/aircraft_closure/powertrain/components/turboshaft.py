"""Turboshaft: fuel-to-shaft power with optional altitude lapse and part-power submodel.

Defaults are the Tier 1 model: constant thermal efficiency, rated power at
every altitude, mass from specific power. Each refinement is a submodel that
keeps the operating input (shaft power) and adds only the point's atmosphere:

* `lapse_exponent` n: power available = rated x (rho / rho_SL)^n.
* `part_power_model`: efficiency x ratio(throttle), throttle = shaft power /
  power available. None keeps efficiency constant; `GeissPartPowerModel` is
  AeroSandbox's knockdown (Geiss 2020); `CubicPartPowerModel` is the same
  cubic form with fitted coefficients, e.g. `deck_1120hp_part_power_model()`
  fitted to an engine deck; `TabulatedPartPowerModel` interpolates a table.
  `thermal_efficiency` remains the full-power value.
* `mass_kg`: explicit mass (e.g. a caller-side AeroSandbox regression),
  overriding `power_rated_W / specific_power_W_kg`.
"""
from dataclasses import dataclass
from functools import cached_property
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.library.power_turboshaft import thermal_efficiency_turboshaft

density_sea_level_kg_m3 = asb.Atmosphere(altitude=0).density()


@dataclass(frozen=True)
class GeissPartPowerModel:
    """AeroSandbox / Geiss (2020) knockdown, as a ratio so the mass argument cancels."""

    def efficiency_ratio(self, throttle):
        return (thermal_efficiency_turboshaft(100.0, throttle_setting=throttle)
                / thermal_efficiency_turboshaft(100.0, throttle_setting=1.0))


@dataclass(frozen=True)
class TabulatedPartPowerModel:
    """Efficiency ratio = 1 / (sfc / sfc_max) on a power-fraction table.

    Smooth B-spline through the nodes (AeroSandbox `InterpolatedModel`);
    outside the table the end values are held (constant extrapolation).
    """
    power_fraction: tuple
    sfc_ratio: tuple
    source: str = ""

    @cached_property
    def _table(self):
        return asb.InterpolatedModel(x_data_coordinates=np.array(self.power_fraction),
                                     y_data_structured=1 / np.array(self.sfc_ratio), method="bspline",
                                     fill_value=None)  # None: hold end values

    def efficiency_ratio(self, throttle):
        return self._table(throttle)


@dataclass(frozen=True)
class CubicPartPowerModel:
    """ratio = 1 + a (t - 1) + b (t - 1)^2 + c (t - 1)^3: exactly 1 at full power, smooth everywhere."""
    coefficients: tuple
    source: str = ""

    def efficiency_ratio(self, throttle):
        a, b, c = self.coefficients
        x = throttle - 1
        return 1 + a * x + b * x**2 + c * x**3


# Median normalised sfc of the user-supplied GASP_TS 1,120 hp deck (`MAPS_1120hp.eng`) over all 130 Mach x
# altitude rows, derived 2026-10-02 with `decks.part_power_curve`; row spread about +/-10-15 % (plan 014).
deck_1120hp_power_fraction = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
deck_1120hp_sfc_ratio = (3.0374, 1.945, 1.5388, 1.3355, 1.2206, 1.1432, 1.0871, 1.0495, 1.0211, 1.0)


def deck_1120hp_part_power_model():
    """Cubic least-squares fit (`decks.fit_cubic_part_power`) to the 1,120 hp deck table; within 1.1 % of the
    table from 20 % to 100 % power and 1.4 % at 10 %. A cubic, not a spline, because the coupled sizing needs a
    smooth, cheap expression (a B-spline through the steep low-power end stalled IPOPT, plan 014)."""
    return CubicPartPowerModel(coefficients=(0.28925719746444895, 0.35363369887773566, 0.9495496745484245),
                               source="cubic fit to GASP_TS turboshaft_1120hp deck median")


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
    part_power_model: Any = None
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
        if self.part_power_model is None:
            return self.thermal_efficiency
        throttle = shaft_power_W / self.power_available_W(atmosphere)
        return self.thermal_efficiency * self.part_power_model.efficiency_ratio(throttle)

    def evaluate(self, shaft_power_W, atmosphere=None):
        # Fuel LHV defines chemical input. Power available is an independent
        # limit for the caller to enforce (operating margins).
        power_fuel_W = shaft_power_W / self.thermal_efficiency_at(shaft_power_W, atmosphere)
        return TurboshaftResult(shaft_power_W, power_fuel_W / self.fuel_lower_heating_value_J_kg,
                                power_fuel_W - shaft_power_W)


if __name__ == "__main__":
    print(SimpleTurboshaft().evaluate(100000))
