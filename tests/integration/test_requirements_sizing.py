import unittest
from dataclasses import replace

from aircraft_closure.requirements.capability import RequirementSet, SpeedRequirement
from examples.requirements_sizing import solve_requirements_sizing


class RequirementsSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = solve_requirements_sizing()
        cls.faster = solve_requirements_sizing(replace(RequirementSet(), speed=SpeedRequirement(velocity_m_s=90.0)))

    def test_every_requirement_met(self):
        self.assertTrue(all(margin > -1e-6 for _, margin in self.reference.margins))

    def test_binding_requirements(self):
        binding = {label for label, margin in self.reference.margins if abs(margin) < 1e-6}
        self.assertIn("max_speed: turboshaft power_shaft_W", binding)
        self.assertIn("hover: motor power_shaft_W", binding)

    def test_sustained_points_use_no_battery(self):
        for label, _, power_battery_W, _ in self.reference.point_powers_W:
            if label != "hover":
                self.assertAlmostEqual(power_battery_W, 0.0, places=6)

    def test_faster_requirement_grows_turboshaft_and_mass(self):
        self.assertGreater(self.faster.power_rated_turboshaft_W, self.reference.power_rated_turboshaft_W)
        self.assertGreater(self.faster.mass_takeoff_kg, self.reference.mass_takeoff_kg)

    def test_masses_sum_to_mtom(self):
        self.assertAlmostEqual(sum(m for _, m in self.reference.component_masses_kg), self.reference.mass_takeoff_kg,
                               places=5)


if __name__ == "__main__":
    unittest.main()
