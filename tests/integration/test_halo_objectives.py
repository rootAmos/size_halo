"""Plan 044: the sizing objectives (fuel and range added) on a small asb.Opti problem, and their numeric values."""
import unittest
from types import SimpleNamespace

import aerosandbox as asb

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, default_starts, halo_objective,
                                  hybridization_mean, objective_value, objectives, scale_objective_cost_usd)


def solve_toy_hybrid(objective):
    """A toy series hybrid sized like the Halo: fixed engine power, a battery that adds hover power and mission
    energy but weighs more per kWh than fuel, an engine-out floor on the battery and a pack-size cap.

    Mass closure: m = m_empty + payload + battery / e_battery + fuel. Mission energy (shaft-equivalent):
    c m R = e_fuel fuel + battery. Hover: w m <= P_engine + p_battery battery. Cost: airframe per kg + fuel
    price + battery price (per mission)."""
    opti = asb.Opti()
    energy_battery_kWh = opti.variable(init_guess=100.0, scale=100.0, lower_bound=70.0, upper_bound=200.0)
    mass_fuel_kg = opti.variable(init_guess=50.0, scale=100.0, lower_bound=0.0)
    mass_takeoff_kg = opti.variable(init_guess=4500.0, scale=1000.0)
    mass_payload_kg = (opti.variable(init_guess=900.0, scale=100.0, lower_bound=0.0) if objective == "payload"
                       else 900.0)
    range_m = opti.variable(init_guess=100e3, scale=1e5, lower_bound=50e3) if objective == "range" else 100e3
    opti.subject_to([
        mass_takeoff_kg == 3000.0 + mass_payload_kg + 5.0 * energy_battery_kWh + mass_fuel_kg,
        6.25e-4 * mass_takeoff_kg * range_m / 1e3 == 4.0 * mass_fuel_kg + energy_battery_kWh,
        0.15 * mass_takeoff_kg <= 900.0 + 3.0 * energy_battery_kWh,
    ])
    cost_per_mission_usd = 0.2 * mass_takeoff_kg + 0.9 * mass_fuel_kg + 0.15 * energy_battery_kWh
    opti.minimize(halo_objective(objective, mass_takeoff_kg, mass_payload_kg, mass_fuel_kg, range_m,
                                 cost_per_mission_usd))
    solution = opti.solve(verbose=False)
    return {name: float(solution.value(x)) for name, x in dict(
        energy_battery_kWh=energy_battery_kWh, mass_fuel_kg=mass_fuel_kg, mass_takeoff_kg=mass_takeoff_kg,
        mass_payload_kg=mass_payload_kg, range_m=range_m, cost_per_mission_usd=cost_per_mission_usd).items()}


class ObjectiveInOptiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.optima = {objective: solve_toy_hybrid(objective) for objective in objectives}

    def test_mass_optimum_sits_on_the_battery_floor(self):
        self.assertAlmostEqual(self.optima["mass_takeoff"]["energy_battery_kWh"], 70.0, places=3)

    def test_fuel_optimum_buys_battery_to_displace_fuel(self):
        mass, fuel = self.optima["mass_takeoff"], self.optima["fuel"]
        self.assertAlmostEqual(fuel["energy_battery_kWh"], 200.0, places=3)   # up to the pack-size cap
        self.assertLess(fuel["mass_fuel_kg"], mass["mass_fuel_kg"] - 1.0)
        self.assertGreater(fuel["mass_takeoff_kg"], mass["mass_takeoff_kg"] + 1.0)

    def test_payload_and_range_are_maximised_at_the_required_other(self):
        mass, payload, range_ = self.optima["mass_takeoff"], self.optima["payload"], self.optima["range"]
        self.assertGreater(payload["mass_payload_kg"], 900.0 + 1.0)
        self.assertGreater(range_["range_m"], 100e3 + 1.0)
        self.assertEqual(range_["mass_payload_kg"], 900.0)
        self.assertEqual(payload["range_m"], 100e3)
        # Both end on the hover power wall (fixed engine plus battery).
        for optimum in (payload, range_):
            self.assertAlmostEqual(0.15 * optimum["mass_takeoff_kg"],
                                   900.0 + 3.0 * optimum["energy_battery_kWh"], delta=1e-3)

    def test_each_optimum_wins_its_own_objective(self):
        keys = dict(mass_takeoff=("mass_takeoff_kg", 1), fuel=("mass_fuel_kg", 1), cost=("cost_per_mission_usd", 1),
                    payload=("mass_payload_kg", -1), range=("range_m", -1))
        for objective, (key, sign) in keys.items():
            own = sign * self.optima[objective][key]
            for other in ("mass_takeoff", "fuel", "cost"):     # the fixed-payload, fixed-range optima
                self.assertLessEqual(own, sign * self.optima[other][key] + 1e-6 * abs(own), (objective, other))


class ObjectiveValueTests(unittest.TestCase):
    def test_numeric_values(self):
        result = SimpleNamespace(mass_takeoff_kg=6885.0, mass_payload_kg=900.0, mass_fuel_burnt_kg=850.0,
                                 range_m=824e3, cost=SimpleNamespace(per_mission_usd=3300.0))
        self.assertEqual(objective_value(result, "mass_takeoff"), 6885.0)
        self.assertEqual(objective_value(result, "payload"), -900.0)
        self.assertEqual(objective_value(result, "cost"), 3300.0)
        self.assertEqual(objective_value(result, "fuel"), 850.0)
        self.assertEqual(objective_value(result, "range"), -824e3)
        # The scaled expression orders results the same way as objective_value.
        self.assertEqual(halo_objective("cost", 0, 0, 0, 0, 3300.0), 3300.0 / scale_objective_cost_usd)
        self.assertEqual(halo_objective("range", 0, 0, 0, 824e3, 0), -8.24)

    def test_unknown_objective_raises(self):
        with self.assertRaises(ValueError):
            objective_value(SimpleNamespace(), "speed")
        with self.assertRaises(ValueError):
            halo_objective("speed", 0, 0, 0, 0, 0)

    def test_fuel_and_range_start_from_the_mass_optimum(self):
        for objective in ("fuel", "range"):
            self.assertIn("mass_objective", default_starts(HaloRequirements(), HaloAssumptions(), objective))
        self.assertNotIn("mass_objective", default_starts(HaloRequirements(), HaloAssumptions(), "mass_takeoff"))

    def test_fixed_pack_is_off_by_default(self):
        self.assertIsNone(HaloAssumptions().count_parallel_battery_fixed)


class HybridizationMeanTests(unittest.TestCase):
    def test_weighted_by_motor_energy(self):
        def point(share, power_W, duration_s):
            return SimpleNamespace(point=SimpleNamespace(hybridization_electric=share, power_electric_motors_W=power_W),
                                   duration_s=duration_s)
        segment = SimpleNamespace(subsegments=(point(-0.1, 1e6, 100.0), point(0.3, 3e6, 100.0)))
        self.assertAlmostEqual(hybridization_mean(segment), (-0.1 * 1 + 0.3 * 3) / 4)
        single = SimpleNamespace(subsegments=(), point=SimpleNamespace(hybridization_electric=0.2,
                                                                       power_electric_motors_W=1e6), duration_s=60.0)
        self.assertAlmostEqual(hybridization_mean(single), 0.2)


if __name__ == "__main__":
    unittest.main()
