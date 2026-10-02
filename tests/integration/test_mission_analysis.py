import unittest

from examples.mission_analysis import fuel_factor, soc_minimum, solve_prescribed_mission, solve_semi_free_mission


class MissionAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.prescribed = solve_prescribed_mission()
        cls.free = solve_semi_free_mission()

    def test_fuel_load_covers_mission_with_reserve(self):
        for result in (self.prescribed, self.free):
            self.assertAlmostEqual(result.mass_fuel_kg, fuel_factor * result.mass_fuel_burnt_kg, places=6)
            self.assertGreaterEqual(result.soc_end, soc_minimum - 1e-7)  # IPOPT constraint tolerance
            self.assertGreater(result.min_margin, -1e-6)

    def test_prescribed_uses_battery_only_in_hover(self):
        for label, _, _, energy_J, _, _, _ in self.prescribed.segments:
            if "hover" not in label:
                self.assertAlmostEqual(energy_J, 0.0, places=3)

    def test_semi_free_mission_saves_fuel(self):
        self.assertLess(self.free.mass_fuel_kg, self.prescribed.mass_fuel_kg)
        self.assertAlmostEqual(self.free.soc_end, soc_minimum, places=6)
        self.assertLess(self.free.mass_takeoff_kg, self.prescribed.mass_takeoff_kg)

    def test_mission_time(self):
        self.assertAlmostEqual(self.prescribed.duration_s, 60 + 250 + 100000 / 60 + 1200 + 1000 / 3 + 60, places=6)


if __name__ == "__main__":
    unittest.main()
