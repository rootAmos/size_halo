import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
import casadi as cas

from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor, TorqueDensityMassModel, rubber_machine
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.topologies import build_series_hybrid
from aircraft_closure.weights import afdd

ratios = dict(torque_ratio=2.5, power_ratio=1.25, speed_ratio=2.5)


class TorqueDensityMassTests(unittest.TestCase):
    def test_default_machines_unchanged(self):
        self.assertEqual(Motor().get_mass(), 100000.0 / 5000.0)
        self.assertEqual(Generator().get_mass(), 100000.0 / 4000.0)

    def test_torque_limited_machine(self):
        model = TorqueDensityMassModel(torque_density_Nm_kg=15.0, specific_power_max_W_kg=10000.0, smoothing_kg=1e-3)
        motor = rubber_machine(Motor, 50.0, 4000.0, mass_model=model, **ratios)   # slow, high torque
        self.assertAlmostEqual(float(motor.get_mass()), motor.max_torque_Nm / 15.0, places=3)

    def test_power_limited_machine(self):
        model = TorqueDensityMassModel(torque_density_Nm_kg=15.0, specific_power_max_W_kg=10000.0, smoothing_kg=1e-3)
        motor = rubber_machine(Motor, 2000.0, 100.0, mass_model=model, **ratios)   # fast, low torque
        self.assertAlmostEqual(float(motor.get_mass()), motor.power_rated_W / 10000.0, places=3)

    def test_smooth_maximum_bounds(self):
        model = TorqueDensityMassModel(smoothing_kg=2.0)
        motor = rubber_machine(Motor, 400.0, 1000.0, mass_model=model, **ratios)
        exact = max(motor.max_torque_Nm / 15.0, motor.power_rated_W / 10000.0)
        self.assertGreaterEqual(float(motor.get_mass()), exact)
        self.assertLessEqual(float(motor.get_mass()) - exact, 2.0 * np.log(2) + 1e-9)

    def test_faster_machine_is_lighter_at_equal_power(self):
        model = TorqueDensityMassModel()
        slow = rubber_machine(Motor, 200.0, 2000.0, mass_model=model, **ratios)
        fast = rubber_machine(Motor, 800.0, 500.0, mass_model=model, **ratios)
        self.assertAlmostEqual(slow.power_rated_W, fast.power_rated_W)
        self.assertLess(float(fast.get_mass()), float(slow.get_mass()))

    def test_magnix_anchor(self):
        """magni650: 3,216 N.m peak torque, 206 kg with inverters and cables -> ~15.6 N.m/kg."""
        self.assertAlmostEqual(3216 / 206, 15.6, delta=0.1)

    def test_symbolic(self):
        opti = asb.Opti()
        torque = opti.variable(init_guess=1000.0)
        motor = rubber_machine(Motor, 400.0, torque, mass_model=TorqueDensityMassModel(), **ratios)
        self.assertIsInstance(motor.get_mass(), cas.MX)


class GearboxMassTests(unittest.TestCase):
    def test_afdd00_published_equation(self):
        # 95.7634 * 2^0.38553 * 3100^0.78137 * 20000^0.09899 / 565^0.80686 [lb]
        value = afdd.mass_gearbox_rotor_shaft_afdd00_kg(2, 3100 * u.hp, 20000 * u.rpm, 565 * u.rpm) / u.lbm
        expected = 95.7634 * 2**0.38553 * 3100**0.78137 * 20000**0.09899 / 565**0.80686
        self.assertAlmostEqual(value, expected, places=6)

    def test_higher_ratio_costs_mass(self):
        low = afdd.mass_gearbox_rotor_shaft_afdd00_kg(2, 1e6, 400.0, 60.0)
        high = afdd.mass_gearbox_rotor_shaft_afdd00_kg(2, 1e6, 800.0, 60.0)
        self.assertAlmostEqual(high / low, 2**0.09899, places=10)


class GeneratorGearboxTopologyTests(unittest.TestCase):
    def test_step_up_gearbox_between_engine_and_generator(self):
        topology = build_series_hybrid(Motor(), Generator(), Battery(), SimpleTurboshaft(), Gearbox(),
                                       ActuatorDiskPropulsor(), count_rotors=2, count_turbogenerators=2,
                                       generator_gearbox=Gearbox(reduction_ratio=0.1))
        links = {(c.source, c.target) for c in topology.connections}
        self.assertIn(("turboshaft.shaft", "generator_gearbox.shaft_in"), links)
        self.assertIn(("generator_gearbox.shaft_out", "generator.shaft"), links)
        self.assertNotIn(("turboshaft.shaft", "generator.shaft"), links)
        self.assertEqual(topology.instances["generator_gearbox"].count, 2)


if __name__ == "__main__":
    unittest.main()
