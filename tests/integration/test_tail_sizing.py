import unittest

from examples.tail_sizing import TailSizingRequirements, solve_tail_sizing


class TailSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = solve_tail_sizing()
        cls.stricter = solve_tail_sizing(TailSizingRequirements(static_margin_min=0.15, cn_beta_min_per_rad=0.08))

    def test_all_requirements_met(self):
        self.assertTrue(all(margin > -1e-6 for _, margin in self.reference.margins))

    def test_stability_requirements_size_the_tails(self):
        binding = {label for label, margin in self.reference.margins if abs(margin) < 1e-6}
        self.assertEqual(binding, {"static_margin", "cn_beta_per_rad"})
        self.assertAlmostEqual(self.reference.static_margin, 0.10, places=6)
        self.assertAlmostEqual(self.reference.cn_beta_per_rad, 0.06, places=6)

    def test_stricter_requirements_grow_tails_and_mass(self):
        self.assertGreater(self.stricter.area_horizontal_tail_m2, self.reference.area_horizontal_tail_m2)
        self.assertGreater(self.stricter.area_vertical_tail_m2, self.reference.area_vertical_tail_m2)
        self.assertGreater(self.stricter.mass_takeoff_kg, self.reference.mass_takeoff_kg)

    def test_trim_is_nose_down_with_small_trim_drag(self):
        self.assertLess(self.reference.elevator_cruise_deg, 0)
        self.assertLess(self.reference.trim_drag_cd, 0.002)


if __name__ == "__main__":
    unittest.main()
