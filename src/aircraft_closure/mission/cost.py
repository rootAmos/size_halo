"""Acquisition-plus-operating cost per mission (Tier 22, plan 029).

The simplest useful direct-operating-cost form for an uncrewed hybrid-electric
aircraft, written so every term is an AeroSandbox/CasADi expression of the
sized design and the flown mission:

    cost per mission = capital + fuel + ground electricity + battery + time

* capital: (airframe x price per kg + electric machines x price per W +
  turboshafts x price per W) / missions in the airframe life (straight-line
  depreciation, no interest, no residual value);
* fuel: fuel burnt x fuel price;
* ground electricity: the energy that restores the take-off SOC, at the
  charger efficiency, x the electricity price;
* battery: pack price x (discharge throughput / cycle life + capacity /
  missions in the calendar life). Cycle ageing and calendar ageing add (a
  first-order approximation of the battery-replacement rate);
* time: mission duration x cost per flight hour (maintenance, remote crew and
  ground operations, lumped).

The battery's pack price is not in `capital`: the battery term replaces the
pack at the rate it is consumed. Prices are US dollars (a currency, not a
physical unit) per SI unit. Default values are assumptions (see each field);
none is Halo or Archer data. Cost is an evaluator: it builds expressions and
owns no variables or constraints.
"""
from dataclasses import dataclass
from typing import Any

joule_per_kWh = 3.6e6


@dataclass(frozen=True)
class CostBreakdown:
    """Cost per mission by item (US dollars) and the acquisition price of one aircraft."""
    acquisition_usd: Any            # one aircraft, battery included
    capital_usd: Any                # per mission, battery excluded
    fuel_usd: Any
    electricity_usd: Any
    battery_usd: Any
    time_usd: Any

    @property
    def per_mission_usd(self):
        return self.capital_usd + self.fuel_usd + self.electricity_usd + self.battery_usd + self.time_usd


@dataclass(frozen=True)
class CostModel:
    # Airframe (structure, rotors, gearboxes, systems, equipment): assumed. Light-rotorcraft list prices are of the
    # order of USD 1,000-3,000 per kg of empty mass including engines; the engines and machines are priced apart.
    price_airframe_usd_kg: float = 1500.0
    # Electric motors and generators with their inverters, per W rated: assumed (USD 150 per kW, aerospace grade).
    price_machine_usd_W: float = 150.0 / 1000.0
    # Turboshafts, per W rated: assumed (USD 500 per kW). Constant while the engines are fixed (plan 017).
    price_turboshaft_usd_W: float = 500.0 / 1000.0
    # Battery pack, per J of capacity: assumed USD 500 per kWh for an aviation-grade pack, about four times the
    # BloombergNEF 2024 automotive pack average (USD 115 per kWh).
    price_battery_usd_J: float = 500.0 / joule_per_kWh
    # Jet-A, per kg: assumed USD 0.90 per kg (about USD 2.7 per US gallon delivered; EIA US Gulf Coast jet
    # fuel spot prices were about USD 2.0-2.5 per gallon in 2024, before distribution and into-plane fees).
    price_fuel_usd_kg: float = 0.90
    # Grid electricity, per J: assumed USD 0.15 per kWh (EIA US commercial average about USD 0.13 per kWh, 2024).
    price_electricity_usd_J: float = 0.15 / joule_per_kWh
    efficiency_charger: float = 0.90                 # assumed ground charger and pack round-trip share
    # Equivalent full discharge cycles to end of life within the sizing SOC window: assumed.
    count_cycles_life_battery: float = 1000.0
    # Missions in the battery's calendar life: assumed (8 years at about 470 missions per year).
    count_missions_calendar_battery: float = 3750.0
    # Missions in the airframe life: assumed (15 years at about 470 missions per year, about 1,500 flight hours
    # per year at the Halo's ~3.2 h mission).
    count_missions_life: float = 7000.0
    # Maintenance, remote crew and ground operations, per second of mission: assumed USD 500 per flight hour.
    cost_time_usd_s: float = 500.0 / 3600.0

    def evaluate(self, mass_airframe_kg, power_rated_machines_W, power_rated_turboshafts_W,
                 energy_capacity_battery_J, mass_fuel_burnt_kg, energy_discharge_battery_J,
                 energy_recharge_ground_J, duration_mission_s):
        """Cost per mission. Masses and powers are aircraft totals; energies are battery chemical energies
        (discharge throughput over the mission, and the energy returned on the ground to the take-off SOC)."""
        price_battery_usd = self.price_battery_usd_J * energy_capacity_battery_J
        acquisition_usd = (self.price_airframe_usd_kg * mass_airframe_kg
                           + self.price_machine_usd_W * power_rated_machines_W
                           + self.price_turboshaft_usd_W * power_rated_turboshafts_W)
        return CostBreakdown(
            acquisition_usd=acquisition_usd + price_battery_usd,
            capital_usd=acquisition_usd / self.count_missions_life,
            fuel_usd=self.price_fuel_usd_kg * mass_fuel_burnt_kg,
            electricity_usd=self.price_electricity_usd_J * energy_recharge_ground_J / self.efficiency_charger,
            battery_usd=(self.price_battery_usd_J * energy_discharge_battery_J / self.count_cycles_life_battery
                         + price_battery_usd / self.count_missions_calendar_battery),
            time_usd=self.cost_time_usd_s * duration_mission_s,
        )
