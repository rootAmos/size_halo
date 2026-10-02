import unittest
import aerosandbox as asb
import aerosandbox.numpy as np
from aerosandbox.library.propulsion_propeller import propeller_shaft_power_from_thrust
from aircraft_closure.powertrain.components.motor import Motor, SimpleMotorLossModel
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor


class MachineTests(unittest.TestCase):
    def test_motor_power_and_current(self):
        result = Motor().evaluate(400, 200, 800)
        self.assertEqual(result.power_shaft_W, 80000)
        self.assertEqual(result.power_electric_W, result.power_shaft_W + result.power_loss_W)
        self.assertEqual(result.current_A * 800, result.power_electric_W)
        self.assertGreater(result.power_loss_W, 0)

    def test_lossless_machines(self):
        for machine_type in (Motor, Generator):
            with self.subTest(machine=machine_type):
                result = machine_type(loss_model=SimpleMotorLossModel(0, 0)).evaluate(400, 200, 800)
                self.assertEqual(result.power_electric_W, result.power_shaft_W)

    def test_zero_and_increasing_demand(self):
        # The simple model has no standstill loss; McDonald's C0 is a constant loss.
        self.assertEqual(Motor(loss_model=SimpleMotorLossModel()).evaluate(0, 0, 800).power_electric_W, 0)
        self.assertAlmostEqual(Motor().evaluate(0, 0, 800).power_electric_W,
                               Motor().loss_model.coefficients().constant_W)
        self.assertGreater(Motor().evaluate(400, 200, 800).power_electric_W,
                           Motor().evaluate(400, 100, 800).power_electric_W)

    def test_generator_conservation(self):
        result = Generator().evaluate(400, 200, 800)
        self.assertEqual(result.power_shaft_W, result.power_electric_W + result.power_loss_W)
        self.assertGreater(result.current_A, 0)
        self.assertEqual(result.current_A * 800, result.power_electric_W)
        self.assertEqual(Generator(loss_model=SimpleMotorLossModel()).evaluate(0, 0, 800).power_electric_W, 0)
        self.assertAlmostEqual(Generator().evaluate(0, 0, 800).power_electric_W,
                               -Generator().loss_model.coefficients().constant_W)

    def test_generator_output_trend(self):
        self.assertGreater(Generator().evaluate(400, 200, 800).power_electric_W,
                           Generator().evaluate(400, 100, 800).power_electric_W)

    def test_voltage_affects_current(self):
        self.assertEqual(Motor().evaluate(400, 200, 400).current_A,
                         2 * Motor().evaluate(400, 200, 800).current_A)

    def test_mass_and_limits(self):
        for component in (Motor(), Generator()):
            self.assertEqual(component.get_mass(), component.power_rated_W / component.specific_power_W_kg)
            self.assertEqual(component.get_limits().max_torque_Nm, component.max_torque_Nm)

    def test_numeric_arrays(self):
        speed_rad_s = np.array([100., 200., 300.])
        result = Motor(loss_model=SimpleMotorLossModel(0, 0)).evaluate(speed_rad_s, 10, 800)
        self.assertTrue(np.allclose(result.power_electric_W, np.array([1000., 2000., 3000.])))


class BatteryTests(unittest.TestCase):
    def test_discharge_energy_conservation(self):
        battery = Battery()
        result = battery.evaluate(100, 0.9, 60)
        self.assertEqual(result.voltage_V, 795)
        self.assertEqual(result.power_chemical_W, result.power_electric_W + result.power_loss_W)
        self.assertAlmostEqual((0.9 - result.soc_next) * battery.energy_capacity_J,
                               result.power_chemical_W * 60)

    def test_charge_sign(self):
        result = Battery().evaluate(-100, 0.5, 60)
        self.assertLess(result.power_electric_W, 0)
        self.assertGreater(result.soc_next, 0.5)
        self.assertGreater(result.power_loss_W, 0)
        self.assertEqual(result.power_chemical_W, result.power_electric_W + result.power_loss_W)

    def test_zero_current(self):
        result = Battery().evaluate(0, 0.5, 60)
        self.assertEqual(result.soc_next, 0.5)
        self.assertEqual(result.power_loss_W, 0)
        self.assertEqual(result.voltage_V, 800)

    def test_constant_power_ideal_case(self):
        battery = Battery(resistance_ohm=0)
        result = battery.evaluate(10000 / 800, 0.9, 360)
        self.assertEqual(result.power_electric_W, 10000)
        self.assertAlmostEqual(result.soc_next, 0.8)

    def test_resistance_loss_trend(self):
        self.assertGreater(Battery(resistance_ohm=0.1).evaluate(100, 0.5).power_loss_W,
                           Battery(resistance_ohm=0.05).evaluate(100, 0.5).power_loss_W)

    def test_energy_and_power_mass_limits(self):
        self.assertEqual(Battery().get_mass(), 40)
        self.assertEqual(Battery(max_discharge_power_W=300000).get_mass(), 100)
        self.assertEqual(Battery(max_charge_power_W=600000).get_mass(), 200)
        self.assertEqual(Battery().get_limits().min_soc, 0.2)

    def test_no_hidden_soc_clamping(self):
        self.assertLess(Battery().evaluate(100, 0.2, 1000).soc_next, 0.2)


class EngineGearTests(unittest.TestCase):
    def test_engine_fuel_identity(self):
        engine = SimpleTurboshaft()
        result = engine.evaluate(100000)
        self.assertAlmostEqual(result.fuel_flow_kg_s * engine.fuel_lower_heating_value_J_kg * engine.thermal_efficiency,
                               100000)
        self.assertAlmostEqual(result.power_loss_W + result.power_shaft_W,
                               result.fuel_flow_kg_s * engine.fuel_lower_heating_value_J_kg)

    def test_engine_zero_and_trends(self):
        self.assertEqual(SimpleTurboshaft().evaluate(0).fuel_flow_kg_s, 0)
        self.assertGreater(SimpleTurboshaft().evaluate(100000).fuel_flow_kg_s,
                           SimpleTurboshaft().evaluate(50000).fuel_flow_kg_s)
        self.assertLess(SimpleTurboshaft(thermal_efficiency=0.4).evaluate(100000).fuel_flow_kg_s,
                        SimpleTurboshaft(thermal_efficiency=0.3).evaluate(100000).fuel_flow_kg_s)

    def test_gearbox_transformations(self):
        result = Gearbox().evaluate(400, 200)
        self.assertEqual(result.speed_output_rad_s, 100)
        self.assertEqual(result.torque_output_Nm, 776)
        self.assertEqual(result.power_output_W, result.speed_output_rad_s * result.torque_output_Nm)
        self.assertAlmostEqual(result.power_input_W, result.power_output_W + result.power_loss_W)

    def test_gearbox_transparent_limit(self):
        result = Gearbox(reduction_ratio=1, efficiency=1).evaluate(400, 200)
        self.assertEqual(result.speed_output_rad_s, 400)
        self.assertEqual(result.torque_output_Nm, 200)
        self.assertEqual(result.power_loss_W, 0)

    def test_gearbox_zero_and_loss_trend(self):
        self.assertEqual(Gearbox().evaluate(0, 0).power_output_W, 0)
        self.assertGreater(Gearbox(efficiency=0.9).evaluate(400, 200).power_loss_W,
                           Gearbox(efficiency=0.99).evaluate(400, 200).power_loss_W)

    def test_mass_ratings(self):
        for component in (SimpleTurboshaft(), Gearbox()):
            self.assertEqual(component.get_mass(), component.power_rated_W / component.specific_power_W_kg)
            self.assertEqual(component.get_limits().power_rated_W, component.power_rated_W)


class RotorTests(unittest.TestCase):
    def setUp(self):
        self.atmosphere = asb.Atmosphere(altitude=0)

    def test_ideal_hover_identity(self):
        rotor = ActuatorDiskPropulsor(coefficient_of_performance=1)
        result = rotor.evaluate(0, self.atmosphere, thrust_N=5000)
        expected_power_W = 5000**1.5 / np.sqrt(2 * self.atmosphere.density() * rotor.area_disk_m2)
        self.assertAlmostEqual(result.shaft_power_W, expected_power_W)

    def test_native_forward_flight_agreement(self):
        rotor = ActuatorDiskPropulsor()
        result = rotor.evaluate(50, self.atmosphere, thrust_N=5000)
        expected_power_W = propeller_shaft_power_from_thrust(5000, rotor.area_disk_m2, 50,
                                                           self.atmosphere.density(), 0.8)
        self.assertAlmostEqual(result.shaft_power_W, expected_power_W)

    def test_disk_area_density_performance_trends(self):
        baseline = ActuatorDiskPropulsor().evaluate(0, self.atmosphere, thrust_N=5000)
        self.assertLess(ActuatorDiskPropulsor(area_disk_m2=20).evaluate(0, self.atmosphere, thrust_N=5000).shaft_power_W,
                        baseline.shaft_power_W)
        self.assertGreater(ActuatorDiskPropulsor().evaluate(0, asb.Atmosphere(altitude=3000), thrust_N=5000).shaft_power_W,
                           baseline.shaft_power_W)
        self.assertLess(ActuatorDiskPropulsor(coefficient_of_performance=1).evaluate(0, self.atmosphere, thrust_N=5000).shaft_power_W,
                        baseline.shaft_power_W)

    def test_zero_thrust_and_power(self):
        for axial_velocity_m_s in (0, 50):
            result = ActuatorDiskPropulsor().evaluate(axial_velocity_m_s, self.atmosphere, thrust_N=0)
            self.assertEqual(result.shaft_power_W, 0)
            self.assertEqual(result.induced_velocity_m_s, 0)
            result = ActuatorDiskPropulsor().evaluate(axial_velocity_m_s, self.atmosphere,
                                                    shaft_power_W=0, induced_velocity_m_s=0)
            self.assertEqual(result.thrust_N, 0)
            self.assertEqual(result.power_residual_W, 0)

    def test_power_mode_residual(self):
        rotor = ActuatorDiskPropulsor()
        forward = rotor.evaluate(20, self.atmosphere, thrust_N=5000)
        inverse = rotor.evaluate(20, self.atmosphere, shaft_power_W=forward.shaft_power_W,
                                 induced_velocity_m_s=forward.induced_velocity_m_s)
        self.assertAlmostEqual(inverse.thrust_N, 5000)
        self.assertAlmostEqual(inverse.power_residual_W, 0)

    def test_interface_errors_and_limits(self):
        rotor = ActuatorDiskPropulsor()
        with self.assertRaises(ValueError):
            rotor.evaluate(0, self.atmosphere, shaft_power_W=100000)
        with self.assertRaises(ValueError):
            rotor.evaluate(0, self.atmosphere, thrust_N=5000, induced_velocity_m_s=10)
        self.assertEqual(rotor.get_mass(), 30)
        self.assertEqual(rotor.get_limits().max_shaft_power_W, 100000)


if __name__ == "__main__":
    unittest.main()


class RotorModeTests(unittest.TestCase):
    def test_airplane_mode_coefficient(self):
        rotor = ActuatorDiskPropulsor(coefficient_of_performance=0.67, coefficient_of_performance_airplane=0.87)
        self.assertEqual(rotor.in_airplane_mode().coefficient_of_performance, 0.87)
        self.assertEqual(ActuatorDiskPropulsor().in_airplane_mode().coefficient_of_performance, 0.8)
        atmosphere = asb.Atmosphere(altitude=0)
        hover = rotor.evaluate(60.0, atmosphere, thrust_N=2000.0).shaft_power_W
        cruise = rotor.in_airplane_mode().evaluate(60.0, atmosphere, thrust_N=2000.0).shaft_power_W
        self.assertAlmostEqual(float(cruise / hover), 0.67 / 0.87, places=12)

    def test_radius(self):
        self.assertAlmostEqual(float(ActuatorDiskPropulsor(area_disk_m2=np.pi * 4.0).radius_m()), 2.0, places=12)


class BatterySmoothingTests(unittest.TestCase):
    def test_default_is_exact_maximum(self):
        self.assertEqual(Battery().get_mass(), max(36e6 / 9e5, 1e5 / 3000))

    def test_smoothing_bounds(self):
        for power_W in (5e4, 1.2e5, 3e5):
            exact = Battery(max_discharge_power_W=power_W).get_mass()
            smooth = Battery(max_discharge_power_W=power_W, mass_smoothing_kg=2.0).get_mass()
            self.assertGreaterEqual(float(smooth), float(exact) - 1e-12)
            self.assertLessEqual(float(smooth), float(exact) + 2.0 * np.log(2) + 1e-12)


class TurboshaftSubmodelTests(unittest.TestCase):
    def test_defaults_are_the_tier1_engine(self):
        engine = SimpleTurboshaft()
        high = asb.Atmosphere(altitude=4000)
        self.assertEqual(engine.power_available_W(high), engine.power_rated_W)
        self.assertEqual(engine.evaluate(1e5, high).fuel_flow_kg_s, engine.evaluate(1e5).fuel_flow_kg_s)

    def test_lapse_is_density_ratio_power(self):
        engine = SimpleTurboshaft(power_rated_W=1e6, lapse_exponent=0.8)
        atmosphere = asb.Atmosphere(altitude=4000)
        sigma = atmosphere.density() / asb.Atmosphere(altitude=0).density()
        self.assertAlmostEqual(float(engine.power_available_W(atmosphere)), 1e6 * float(sigma)**0.8, places=6)
        self.assertAlmostEqual(float(engine.power_available_W(asb.Atmosphere(altitude=0))), 1e6, places=6)
        self.assertEqual(engine.power_available_W(), 1e6)
        self.assertLess(float(engine.get_limits(asb.Atmosphere(altitude=6000)).power_rated_W),
                        float(engine.get_limits(atmosphere).power_rated_W))

    def test_knockdown_is_aerosandbox_ratio(self):
        from aerosandbox.library.power_turboshaft import thermal_efficiency_turboshaft
        engine = SimpleTurboshaft(power_rated_W=1e6, part_power_knockdown=True)
        for throttle in (0.3, 0.7, 1.0):
            expected = (thermal_efficiency_turboshaft(250.0, throttle_setting=throttle)
                        / thermal_efficiency_turboshaft(250.0, throttle_setting=1.0))
            self.assertAlmostEqual(float(engine.thermal_efficiency_at(throttle * 1e6) / engine.thermal_efficiency),
                                   float(expected), places=12)
        self.assertAlmostEqual(float(engine.thermal_efficiency_at(1e6)), engine.thermal_efficiency, places=12)

    def test_part_power_costs_fuel_per_watt(self):
        engine = SimpleTurboshaft(power_rated_W=1e6, part_power_knockdown=True)
        specific = lambda power_W: float(engine.evaluate(power_W).fuel_flow_kg_s) / power_W
        self.assertGreater(specific(4e5), specific(8e5))

    def test_throttle_uses_power_available_at_altitude(self):
        engine = SimpleTurboshaft(power_rated_W=1e6, lapse_exponent=1.0, part_power_knockdown=True)
        high = asb.Atmosphere(altitude=4000)
        self.assertAlmostEqual(float(engine.thermal_efficiency_at(engine.power_available_W(high), high)),
                               engine.thermal_efficiency, places=12)

    def test_explicit_mass_overrides_specific_power(self):
        self.assertEqual(SimpleTurboshaft(mass_kg=250.0).get_mass(), 250.0)

    def test_symbolic_altitude_and_power(self):
        opti = asb.Opti()
        altitude_m = opti.variable(init_guess=1000.0)
        power_W = opti.variable(init_guess=5e5)
        engine = SimpleTurboshaft(power_rated_W=1e6, lapse_exponent=0.8, part_power_knockdown=True)
        fuel = engine.evaluate(power_W, asb.Atmosphere(altitude=altitude_m)).fuel_flow_kg_s
        opti.subject_to([altitude_m == 3000.0, power_W == 5e5])
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(fuel)),
                               float(engine.evaluate(5e5, asb.Atmosphere(altitude=3000.0)).fuel_flow_kg_s), places=10)
