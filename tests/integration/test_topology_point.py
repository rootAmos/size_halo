import unittest

from examples.series_hybrid_point import solve_topology_point
from examples.series_hybrid_point_explicit import solve_reference_point


class TopologyPointTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.explicit = solve_reference_point()
        cls.single = solve_topology_point(count_rotors=1)
        cls.quad = solve_topology_point(count_rotors=4)

    def assertRelative(self, actual, expected, rel=1e-6):
        self.assertLessEqual(abs(actual - expected), rel * abs(expected))

    def test_single_rotor_reproduces_tier1_explicit_coupling(self):
        for field in ("thrust_N", "shaft_power_W", "battery_power_W", "fuel_flow_kg_s"):
            self.assertRelative(getattr(self.single, field), getattr(self.explicit, field))
        self.assertRelative(self.single.shaft_power_W, 89285.745, rel=1e-5)
        self.assertRelative(self.single.battery_power_W, 18653.249, rel=1e-5)
        self.assertRelative(self.single.fuel_flow_kg_s, 0.0058516, rel=1e-4)

    def test_connection_and_physics_residuals_vanish(self):
        for result in (self.single, self.quad):
            self.assertLess(result.max_abs_connection_residual, 1e-6)
            self.assertLess(abs(result.bus_power_residual_W), 1e-4)
            self.assertLess(abs(result.rotor_power_residual_W), 1e-4)

    def test_four_symmetric_rotor_strings(self):
        self.assertRelative(self.quad.thrust_N, 4 * 5000)
        # Per-rotor and per-motor state is unchanged; shared-bus quantities scale.
        self.assertRelative(self.quad.shaft_power_W, self.single.shaft_power_W)
        self.assertRelative(self.quad.current_motor_A, self.single.current_motor_A)
        self.assertRelative(self.quad.voltage_bus_V, self.single.voltage_bus_V)
        self.assertRelative(self.quad.current_motors_total_A, 4 * self.single.current_motors_total_A)
        self.assertRelative(self.quad.battery_power_W, 4 * self.single.battery_power_W)
        # One generator carries 4x load with unscaled loss coefficients, so
        # fuel flow rises by more than 4x.
        self.assertGreater(self.quad.fuel_flow_kg_s, 4 * self.single.fuel_flow_kg_s)


if __name__ == "__main__":
    unittest.main()
