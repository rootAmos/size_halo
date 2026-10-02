import unittest

from examples.aircraft_mass_closure import acceleration_gravity_m_s2, solve_mass_closure
from examples.cruise_closure import solve_cruise_closure


class CruiseClosureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cruise = solve_cruise_closure()
        cls.assumed = solve_mass_closure()

    def test_residuals_vanish(self):
        self.assertLess(abs(self.cruise.closure_residual_kg), 1e-6)
        self.assertLess(abs(self.cruise.lift_residual_N), 1e-6)

    def test_equilibrium_and_stall_margin(self):
        self.assertLess(self.cruise.alpha_cruise_deg, self.cruise.alpha_stall_deg)
        self.assertAlmostEqual(self.cruise.cl_cruise / self.cruise.cd_cruise, self.cruise.lift_to_drag_cruise, places=9)
        weight_N = self.cruise.mass_takeoff_kg * acceleration_gravity_m_s2
        self.assertAlmostEqual(self.cruise.drag_cruise_N, weight_N / self.cruise.lift_to_drag_cruise, places=6)

    def test_weak_coupling_through_fuselage_correlation(self):
        # Raymer's fuselage mass depends on L/D^-0.072, so replacing the assumed 12
        # by the solved cruise value moves MTOM by well under 1 %.
        self.assertLess(abs(self.cruise.mass_takeoff_kg - self.assumed.mass_takeoff_kg) / self.assumed.mass_takeoff_kg,
                        0.01)


if __name__ == "__main__":
    unittest.main()
