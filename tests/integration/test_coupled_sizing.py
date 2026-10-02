import unittest

from examples.coupled_sizing import soc_emergency_floor, solve_coupled_sizing
from examples.mission_analysis import fuel_factor, soc_minimum


class CoupledSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.light = solve_coupled_sizing("mass_takeoff")
        cls.frugal = solve_coupled_sizing("fuel")

    def test_all_constraints_satisfied(self):
        for result in (self.light, self.frugal):
            self.assertLess(abs(result.closure_residual_kg), 1e-5)
            self.assertGreater(result.min_margin, -1e-6)
            self.assertGreaterEqual(result.soc_end, soc_minimum - 1e-7)
            self.assertAlmostEqual(result.mass_fuel_kg, fuel_factor * result.mass_fuel_burnt_kg, places=5)
            self.assertGreaterEqual(result.static_margin, 0.10 - 1e-7)
            self.assertGreaterEqual(result.cn_beta_per_rad, 0.06 - 1e-7)
            self.assertLessEqual(result.rudder_failed_rotor_deg, 20.0 + 1e-6)

    def test_masses_sum_to_mtom(self):
        for result in (self.light, self.frugal):
            self.assertAlmostEqual(sum(m for _, m in result.component_masses_kg), result.mass_takeoff_kg, places=4)

    def test_engine_out_reserve_sizes_the_battery_at_min_mtom(self):
        self.assertIn("soc_after_engine_out_hover", self.light.binding)
        self.assertIn("engine-out hover: battery discharge_power_W", self.light.binding)

    def test_requirements_size_the_powertrain_at_min_mtom(self):
        self.assertIn("max_speed: turboshaft power_shaft_W", self.light.binding)
        self.assertIn("hover: motor power_shaft_W", self.light.binding)
        self.assertIn("static_margin", self.light.binding)
        self.assertIn("cn_beta_per_rad", self.light.binding)

    def test_energy_allocation_uses_the_carried_battery(self):
        battery_energy_J = sum(segment[3] for segment in self.light.segments)
        self.assertGreater(battery_energy_J, 0.0)
        self.assertTrue(all(-1e-9 <= segment[5] <= 1 + 1e-9 for segment in self.light.segments))

    def test_objective_trade(self):
        self.assertLess(self.frugal.mass_fuel_kg, self.light.mass_fuel_kg)
        self.assertGreater(self.frugal.energy_battery_kWh, self.light.energy_battery_kWh)
        self.assertGreater(self.frugal.mass_takeoff_kg, self.light.mass_takeoff_kg)


if __name__ == "__main__":
    unittest.main()
