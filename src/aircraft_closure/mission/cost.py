"""Acquisition-plus-operating cost per mission (Tier 22, plan 029).

The simplest useful direct-operating-cost form for an uncrewed hybrid-electric
aircraft, written so every term is an AeroSandbox/CasADi expression of the
sized design and the flown mission:

    cost per mission = capital + fuel + ground electricity + battery + time
                       (+ airframe maintenance + powerplant maintenance, `maintenance_model="scaled"`)

* capital: (airframe x price per kg + electric machines x price per W +
  turboshafts x price per W) / missions in the airframe life (straight-line
  depreciation, no interest, no residual value);
* fuel: fuel burnt x fuel price;
* ground electricity: the energy that restores the take-off SOC, at the
  charger efficiency, x the electricity price;
* battery: pack price x (discharge throughput / cycle life + capacity /
  missions in the calendar life). Cycle ageing and calendar ageing add (a
  first-order approximation of the battery-replacement rate);
* time: mission duration x cost per flight hour. With `maintenance_model`
  "lumped" (the default) the hour covers maintenance, remote crew and ground
  operations (USD 500); with "scaled" it covers crew and ground operations only
  (USD 200) and maintenance is priced by what is maintained:
* airframe maintenance ("scaled"): duration x airframe mass x rate per kg-hour;
* powerplant maintenance ("scaled"): duration x (turboshaft rated power x its
  rate, including the overhaul reserve, + electric machine rated power x its
  rate), so a heavier or more powerful design pays for it on every flight (the
  user's request, 2026-10-11; the war room's cost axis uses it).

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
    time_usd: Any                   # operations: remote crew and ground handling
    maintenance_airframe_usd: Any = 0.0
    maintenance_powerplant_usd: Any = 0.0

    @property
    def per_mission_usd(self):
        return (self.capital_usd + self.fuel_usd + self.electricity_usd + self.battery_usd + self.time_usd
                + self.maintenance_airframe_usd + self.maintenance_powerplant_usd)


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
    # Jet-A, per kg: USD 3.20 per US gallon delivered (set by the user, 2026-10-11) at 0.804 kg/L: USD 1.05 per kg.
    price_fuel_usd_kg: float = 3.20 / (3.785411784 * 0.804)
    # Grid electricity, per J: USD 0.20 per kWh (set by the user, 2026-10-11).
    price_electricity_usd_J: float = 0.20 / joule_per_kWh
    efficiency_charger: float = 0.90                 # assumed ground charger and pack round-trip share
    # Equivalent full discharge cycles to end of life within the sizing SOC window: assumed.
    count_cycles_life_battery: float = 1000.0
    # Missions in the battery's calendar life: assumed (8 years at about 470 missions per year).
    count_missions_calendar_battery: float = 3750.0
    # Missions in the airframe life: assumed (15 years at about 470 missions per year, about 1,500 flight hours
    # per year at the Halo's ~3.2 h mission).
    count_missions_life: float = 7000.0
    # "lumped": `cost_time_usd_s` covers maintenance, crew and ground operations (the reference). "scaled": crew and
    # ground operations at `cost_operations_usd_s`, maintenance by the rates below.
    maintenance_model: str = "lumped"
    # Maintenance, remote crew and ground operations, per second of mission: assumed USD 500 per flight hour.
    cost_time_usd_s: float = 500.0 / 3600.0
    # "scaled": remote crew and ground operations, per second of mission: assumed USD 200 per flight hour.
    cost_operations_usd_s: float = 200.0 / 3600.0
    # Airframe maintenance (labour and parts; structure, rotors, gearboxes, systems), per kg of airframe per second:
    # assumed USD 0.10 per kg per flight hour, about USD 470 per flight hour for the Halo. Conklin & de Decker put
    # medium twin helicopters (4-6 t empty) at roughly USD 600-1,300 per flight hour with engines included.
    cost_maintenance_airframe_usd_kg_s: float = 0.10 / 3600.0
    # Turboshaft maintenance with the overhaul reserve, per W rated per second: assumed USD 0.12 per kW per flight
    # hour (about USD 100 per flight hour per 1,120 hp engine, PT6C class).
    cost_maintenance_turboshaft_usd_W_s: float = 0.12 / 1000.0 / 3600.0
    # Electric machines and inverters (no hot section; bearings, inspections), per W rated per second: assumed USD
    # 0.02 per kW per flight hour.
    cost_maintenance_machine_usd_W_s: float = 0.02 / 1000.0 / 3600.0

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
            **self._time_and_maintenance(mass_airframe_kg, power_rated_machines_W, power_rated_turboshafts_W,
                                         duration_mission_s),
        )

    def _time_and_maintenance(self, mass_airframe_kg, power_rated_machines_W, power_rated_turboshafts_W,
                              duration_mission_s):
        if self.maintenance_model == "lumped":
            return dict(time_usd=self.cost_time_usd_s * duration_mission_s)
        if self.maintenance_model != "scaled":
            raise ValueError(f"Unknown maintenance model '{self.maintenance_model}'.")
        return dict(
            time_usd=self.cost_operations_usd_s * duration_mission_s,
            maintenance_airframe_usd=self.cost_maintenance_airframe_usd_kg_s * mass_airframe_kg * duration_mission_s,
            maintenance_powerplant_usd=(self.cost_maintenance_turboshaft_usd_W_s * power_rated_turboshafts_W
                                        + self.cost_maintenance_machine_usd_W_s * power_rated_machines_W)
            * duration_mission_s)
