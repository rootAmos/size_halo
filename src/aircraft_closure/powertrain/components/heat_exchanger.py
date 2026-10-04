"""Ram-air liquid-to-air heat exchanger with ducts and hover fans (Tier 19, plan 028).

Rating convention: `power_rated_W` is the heat rejected at the reference
coolant-to-air temperature difference `delta_temperature_ref_C` (a
difference, so kelvin and Celsius agree) with the rated air flow. Mass is
rating / `specific_power_W_kg` and covers core, ducts, fans, pump and coolant.

At an operating point with heat Q and ambient temperature T_a
(dT = T_coolant - T_a, cp of air, air-side effectiveness eps):

* air mass flow  m = Q / (eps cp dT);  rated flow m_ref = Q_rated / (eps cp dT_ref);
* total-pressure loss dp = dp_ref (m / m_ref)^2 (rho_ref / rho), the
  turbulent quadratic law at the sea-level ISA reference density;
* pumping power P = m dp / rho;
* airplane mode (ram air): cooling drag D = P / V. Meredith (ARC R&M 1683):
  a ducted radiator's drag power is the flow work lost in it; the heat-addition
  thrust recovery is not credited here (conservative);
* hover (`fan=True`): no ram pressure, fans supply P: fan power P / eta_fan
  (electrical), no drag;
* required rating Q dT_ref / dT (the core conductance needed at this
  temperature difference); callers constrain it below `power_rated_W`.

With zero heat every output is zero. P grows as Q^3 / (dT^3 Q_rated^2): a
larger exchanger is heavier but has less drag. The model clips nothing; dT
must stay positive (ambient below the coolant), which callers ensure by
their choice of coolant temperature.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb

specific_heat_air_J_kg_K = 1005.0
kelvin_offset_K = 273.15


@dataclass(frozen=True)
class HeatExchangerResult:
    mass_flow_air_kg_s: Any
    pressure_drop_Pa: Any
    power_pumping_W: Any
    drag_N: Any
    power_fan_W: Any
    power_heat_equivalent_W: Any       # heat at the reference temperature difference (rating demand)
    delta_temperature_C: Any           # coolant minus ambient


@dataclass(frozen=True)
class HeatExchangerLimits:
    power_rated_W: Any


@dataclass(frozen=True)
class RamAirHeatExchanger:
    power_rated_W: Any = 300000.0
    specific_power_W_kg: Any = 1000.0      # at delta_temperature_ref_C; Kellermann 2021 ~0.6, Potamiti 2024 ~1.6 kW/kg
    temperature_coolant_C: Any = 60.0
    delta_temperature_ref_C: Any = 40.0
    effectiveness: Any = 0.8
    pressure_drop_ref_Pa: Any = 1000.0
    efficiency_fan: Any = 0.6

    def get_mass(self):
        return self.power_rated_W / self.specific_power_W_kg

    def get_limits(self):
        return HeatExchangerLimits(self.power_rated_W)

    def evaluate(self, power_heat_W, atmosphere, velocity_m_s=0.0, fan=False):
        delta_temperature_C = self.temperature_coolant_C - (atmosphere.temperature() - kelvin_offset_K)
        capacity_air_W_K = self.effectiveness * specific_heat_air_J_kg_K
        mass_flow_air_kg_s = power_heat_W / (capacity_air_W_K * delta_temperature_C)
        mass_flow_ref_kg_s = self.power_rated_W / (capacity_air_W_K * self.delta_temperature_ref_C)
        density_kg_m3 = atmosphere.density()
        density_ref_kg_m3 = asb.Atmosphere(altitude=0).density()
        pressure_drop_Pa = (self.pressure_drop_ref_Pa * (mass_flow_air_kg_s / mass_flow_ref_kg_s)**2
                            * density_ref_kg_m3 / density_kg_m3)
        power_pumping_W = mass_flow_air_kg_s * pressure_drop_Pa / density_kg_m3
        if fan:
            drag_N, power_fan_W = 0 * power_pumping_W, power_pumping_W / self.efficiency_fan
        else:
            drag_N, power_fan_W = power_pumping_W / velocity_m_s, 0 * power_pumping_W
        return HeatExchangerResult(
            mass_flow_air_kg_s=mass_flow_air_kg_s, pressure_drop_Pa=pressure_drop_Pa,
            power_pumping_W=power_pumping_W, drag_N=drag_N, power_fan_W=power_fan_W,
            power_heat_equivalent_W=power_heat_W * self.delta_temperature_ref_C / delta_temperature_C,
            delta_temperature_C=delta_temperature_C)


if __name__ == "__main__":
    print(RamAirHeatExchanger().evaluate(150000.0, asb.Atmosphere(altitude=3048.0), 100.0))
