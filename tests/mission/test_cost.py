"""Tier 22 (plan 029): acquisition-plus-operating cost per mission."""
import math
import unittest
from dataclasses import fields, replace

import aerosandbox as asb

from aircraft_closure.mission.cost import CostBreakdown, CostModel, joule_per_kWh

zero = CostModel(price_airframe_usd_kg=0.0, price_machine_usd_W=0.0, price_turboshaft_usd_W=0.0,
                 price_battery_usd_J=0.0, price_fuel_usd_kg=0.0, price_electricity_usd_J=0.0, cost_time_usd_s=0.0)
inputs = dict(mass_airframe_kg=3000.0, power_rated_machines_W=2.8e6, power_rated_turboshafts_W=1.67e6,
              energy_capacity_battery_J=56.0 * joule_per_kWh, mass_fuel_burnt_kg=780.0,
              energy_discharge_battery_J=30.0 * joule_per_kWh, energy_recharge_ground_J=25.0 * joule_per_kWh,
              duration_mission_s=3.2 * 3600.0)


class CostModelTests(unittest.TestCase):
    def test_zero_prices_give_zero_cost(self):
        cost = zero.evaluate(**inputs)
        self.assertEqual(cost.per_mission_usd, 0.0)
        self.assertEqual(cost.acquisition_usd, 0.0)

    def test_closed_form_items(self):
        m = CostModel()
        cost = m.evaluate(**inputs)
        self.assertAlmostEqual(cost.fuel_usd, m.price_fuel_usd_kg * 780.0)
        self.assertAlmostEqual(cost.time_usd, m.cost_time_usd_s * 3.2 * 3600.0)
        self.assertAlmostEqual(cost.electricity_usd, m.price_electricity_usd_J * 25.0 * joule_per_kWh
                               / m.efficiency_charger)
        acquisition_ex_battery_usd = (m.price_airframe_usd_kg * 3000.0 + m.price_machine_usd_W * 2.8e6
                                      + m.price_turboshaft_usd_W * 1.67e6)
        self.assertAlmostEqual(cost.capital_usd, acquisition_ex_battery_usd / m.count_missions_life)
        price_battery_usd = m.price_battery_usd_J * 56.0 * joule_per_kWh
        self.assertAlmostEqual(cost.acquisition_usd, acquisition_ex_battery_usd + price_battery_usd)
        self.assertAlmostEqual(cost.battery_usd, price_battery_usd * (30.0 / 56.0) / m.count_cycles_life_battery
                               + price_battery_usd / m.count_missions_calendar_battery)
        self.assertAlmostEqual(cost.per_mission_usd, sum(getattr(cost, f.name) for f in fields(CostBreakdown)
                                                         if f.name != "acquisition_usd"))

    def test_battery_wear_limits(self):
        """Unlimited cycle and calendar life: no battery cost per mission; the cycle term depends on throughput
        only (one equivalent full cycle costs one pack over the cycle life, whatever the pack size)."""
        endless = replace(CostModel(), count_cycles_life_battery=math.inf, count_missions_calendar_battery=math.inf)
        self.assertEqual(endless.evaluate(**inputs).battery_usd, 0.0)
        cycles_only = replace(CostModel(), count_missions_calendar_battery=math.inf)
        small = cycles_only.evaluate(**inputs).battery_usd
        large = cycles_only.evaluate(**dict(inputs, energy_capacity_battery_J=2 * inputs["energy_capacity_battery_J"])
                                     ).battery_usd
        self.assertAlmostEqual(small, large)

    def test_every_item_is_non_negative_and_rises_with_its_input(self):
        base = CostModel().evaluate(**inputs)
        for f in fields(CostBreakdown):
            self.assertGreaterEqual(getattr(base, f.name), 0.0)
        for name in inputs:
            more = CostModel().evaluate(**dict(inputs, **{name: 1.1 * inputs[name]}))
            self.assertGreater(more.per_mission_usd, base.per_mission_usd, name)

    def test_symbolic(self):
        opti = asb.Opti()
        fuel_kg = opti.variable(init_guess=500.0)
        cost = CostModel().evaluate(**dict(inputs, mass_fuel_burnt_kg=fuel_kg))
        opti.subject_to(fuel_kg == 780.0)
        opti.minimize(cost.per_mission_usd)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(cost.per_mission_usd)),
                               CostModel().evaluate(**inputs).per_mission_usd, places=4)


if __name__ == "__main__":
    unittest.main()
