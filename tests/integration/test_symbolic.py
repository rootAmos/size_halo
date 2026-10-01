import unittest
import aerosandbox as asb
import casadi as cas
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from examples.series_hybrid_point_explicit import solve_reference_point


class SymbolicTests(unittest.TestCase):
    def test_all_components_with_opti_variables(self):
        opti = asb.Opti()
        speed_rad_s = opti.variable(init_guess=400, lower_bound=1)
        torque_Nm = opti.variable(init_guess=200, lower_bound=0)
        voltage_V = opti.variable(init_guess=800, lower_bound=1)
        current_A = opti.variable(init_guess=100)
        soc = opti.variable(init_guess=0.9)
        thrust_N = opti.variable(init_guess=5000, lower_bound=1)
        altitude_m = opti.variable(init_guess=0)
        opti.subject_to([speed_rad_s == 400, torque_Nm == 200, voltage_V == 800,
                         current_A == 100, soc == 0.9, thrust_N == 5000, altitude_m == 0])
        expressions = [Motor().evaluate(speed_rad_s, torque_Nm, voltage_V).power_electric_W,
                       Generator().evaluate(speed_rad_s, torque_Nm, voltage_V).power_electric_W,
                       Battery().evaluate(current_A, soc, 60).soc_next,
                       SimpleTurboshaft().evaluate(speed_rad_s * torque_Nm).fuel_flow_kg_s,
                       Gearbox().evaluate(speed_rad_s, torque_Nm).power_output_W,
                       ActuatorDiskPropulsor().evaluate(0, asb.Atmosphere(altitude=altitude_m),
                                                       thrust_N=thrust_N).shaft_power_W]
        self.assertTrue(all(isinstance(expression, cas.MX) for expression in expressions))
        solution = opti.solve(verbose=False)
        expected = [80960, 79040, 0.9 - 80000 * 60 / 36000000,
                    80000 / (0.3 * 43000000), 77600,
                    ActuatorDiskPropulsor().evaluate(0, asb.Atmosphere(altitude=0), thrust_N=5000).shaft_power_W]
        for expression, numeric in zip(expressions, expected):
            self.assertAlmostEqual(float(solution.value(expression)), numeric, places=5)

    def test_symbolic_sizes_and_mass(self):
        opti = asb.Opti()
        power_W = opti.variable(init_guess=100000, lower_bound=1)
        energy_J = opti.variable(init_guess=36000000, lower_bound=1)
        opti.subject_to([power_W == 100000, energy_J == 36000000])
        expressions = [Motor(power_rated_W=power_W).get_mass(),
                       Generator(power_rated_W=power_W).get_mass(),
                       SimpleTurboshaft(power_rated_W=power_W).get_mass(),
                       Gearbox(power_rated_W=power_W).get_mass(),
                       Battery(energy_capacity_J=energy_J, max_discharge_power_W=power_W).get_mass(),
                       ActuatorDiskPropulsor(mass_kg=power_W / 10000).get_mass()]
        solution = opti.solve(verbose=False)
        for expression, expected in zip(expressions, [20, 25, 50, 10, 40, 10]):
            self.assertAlmostEqual(float(solution.value(expression)), expected)

    def test_coupled_series_hybrid_point(self):
        result = solve_reference_point()
        self.assertAlmostEqual(result.thrust_N, 5000, places=4)
        self.assertGreater(result.shaft_power_W, 0)
        self.assertLess(result.shaft_power_W, 100000)
        self.assertGreater(result.battery_power_W, 0)
        self.assertGreater(result.fuel_flow_kg_s, 0)
        self.assertAlmostEqual(result.bus_power_residual_W, 0, places=4)
        self.assertAlmostEqual(result.rotor_power_residual_W, 0, places=4)


if __name__ == "__main__":
    unittest.main()
