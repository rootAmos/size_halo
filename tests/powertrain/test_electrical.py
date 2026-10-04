"""Tier 15 electrical layer: inverter, DC/DC, cable, protection, ports and topology (plan 023)."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.core.ports import Direction
from aircraft_closure.powertrain.compatibility import design_margins, port_envelope
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.cable import (Cable, InsulationMaterial, PartialDischargeModel,
                                                         aluminium_conductor, copper_conductor, pressure_ratio)
from aircraft_closure.powertrain.components.converters import ConverterLossModel, DcDcConverter, Inverter
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.ports import port_specs_for
from aircraft_closure.powertrain.topologies import ElectricalLayer, build_series_hybrid


class ConverterLossTests(unittest.TestCase):
    def setUp(self):
        self.model = ConverterLossModel(efficiency_rated=0.985, voltage_rated_V=800.0)

    def test_rated_efficiency_identity(self):
        inverter = Inverter(power_rated_W=500e3, loss_model=self.model)
        result = inverter.evaluate(500e3, 800.0)
        self.assertAlmostEqual(result.power_ac_W / result.power_dc_W, 0.985, delta=2e-6)
        self.assertAlmostEqual(result.current_dc_A, result.power_dc_W / 800.0)

    def test_loss_split_at_rated(self):
        loss_rated_W = 500e3 * 0.015 / 0.985
        only_conduction = ConverterLossModel(0.985, 1.0, 0.0, 800.0, fraction_power_smoothing=0.0)
        self.assertAlmostEqual(only_conduction.evaluate(500e3, 800.0, 500e3), loss_rated_W, places=6)
        self.assertAlmostEqual(only_conduction.evaluate(250e3, 800.0, 500e3), loss_rated_W / 4, places=6)
        self.assertAlmostEqual(only_conduction.evaluate(500e3, 400.0, 500e3), 4 * loss_rated_W, places=6)

    def test_peak_efficiency_load(self):
        loads = np.linspace(0.05, 1.0, 400)
        efficiency = loads / (loads + self.model.evaluate(loads * 500e3, 800.0, 500e3) / 500e3)
        self.assertAlmostEqual(loads[np.argmax(efficiency)], np.sqrt(0.1 / 0.5), delta=0.01)

    def test_limiting_cases_and_signs(self):
        inverter = Inverter(power_rated_W=500e3, loss_model=self.model)
        idle = inverter.evaluate(0.0, 800.0)
        self.assertGreater(idle.power_loss_W, 0)                       # fixed loss at zero power
        motoring, generating = inverter.evaluate(300e3, 800.0), inverter.evaluate(-300e3, 800.0)
        self.assertAlmostEqual(motoring.power_loss_W, generating.power_loss_W, places=6)
        self.assertGreater(motoring.power_dc_W, 300e3)                 # draws more than it delivers
        self.assertGreater(-generating.power_dc_W, 0)
        self.assertLess(-generating.power_dc_W, 300e3)                 # delivers less than it receives

    def test_trend_lower_voltage_more_loss(self):
        self.assertGreater(self.model.evaluate(400e3, 600.0, 500e3), self.model.evaluate(400e3, 800.0, 500e3))

    def test_mass_and_limits(self):
        inverter = Inverter(power_rated_W=600e3, specific_power_W_kg=20e3, voltage_blocking_V=1200.0,
                            factor_derating_voltage=0.75)
        self.assertAlmostEqual(inverter.get_mass(), 30.0)
        self.assertAlmostEqual(inverter.get_limits().max_voltage_V, 900.0)

    def test_dcdc_identities(self):
        dcdc = DcDcConverter(power_rated_W=400e3, voltage_output_V=1000.0,
                             loss_model=ConverterLossModel(efficiency_rated=0.98, voltage_rated_V=756.0))
        result = dcdc.evaluate(400e3 / 0.98, 756.0)
        self.assertAlmostEqual(result.power_output_W, result.power_input_W - result.power_loss_W)
        self.assertAlmostEqual(result.current_output_A, result.power_output_W / 1000.0)
        charging = dcdc.evaluate(-100e3, 756.0)
        self.assertLess(charging.power_output_W, -100e3)               # the bus supplies the loss too
        self.assertAlmostEqual(dcdc.get_mass(), 400e3 / 12e3)


class CableTests(unittest.TestCase):
    def test_area_resistance_identities(self):
        cable = Cable(length_m=10.0, max_current_A=600.0, max_voltage_V=900.0, conductor=aluminium_conductor())
        self.assertAlmostEqual(cable.area_conductor_m2(), 600.0 / 3e6)
        self.assertAlmostEqual(cable.resistance_ohm(), 3.4e-8 * 10.0 * 2 / 2e-4)
        result = cable.evaluate(500.0)
        self.assertAlmostEqual(result.power_loss_W, cable.resistance_ohm() * 500.0**2)
        self.assertAlmostEqual(result.voltage_drop_V, cable.resistance_ohm() * 500.0)

    def test_loss_sign_and_zero(self):
        cable = Cable()
        self.assertEqual(cable.evaluate(0.0).power_loss_W, 0.0)
        self.assertAlmostEqual(cable.evaluate(-300.0).power_loss_W, cable.evaluate(300.0).power_loss_W)
        self.assertLess(cable.evaluate(-300.0).voltage_drop_V, 0)

    def test_rated_loss_fraction_closed_form(self):
        # At the design current the loss fraction is resistivity x J x total conductor length / V.
        cable = Cable(length_m=8.0, max_current_A=1000.0, max_voltage_V=800.0)
        fraction = cable.evaluate(1000.0).power_loss_W / (800.0 * 1000.0)
        self.assertAlmostEqual(fraction, 3.4e-8 * 3e6 * 16.0 / 800.0, places=12)

    def test_partial_discharge_round_trip(self):
        model = PartialDischargeModel()
        for ratio in (1.0, 0.6):
            thickness_m = model.thickness_required_m(1000.0, 2.5, ratio)
            self.assertAlmostEqual(model.voltage_inception_V(thickness_m, 2.5, ratio), 1.5 * 1000.0, places=6)
        # Dakin's form: 163 (t / eps)^0.46 with t in micrometres.
        self.assertAlmostEqual(model.voltage_inception_V(250e-6, 2.5), 163 * 100**0.46, places=6)

    def test_insulation_grows_with_voltage_and_altitude(self):
        low, high = Cable(max_voltage_V=600.0, altitude_design_m=0.0), Cable(max_voltage_V=1200.0, altitude_design_m=0.0)
        self.assertGreater(high.thickness_insulation_m(), low.thickness_insulation_m())
        self.assertGreater(high.get_mass(), low.get_mass())
        ground, ceiling = Cable(max_voltage_V=1200.0, altitude_design_m=0.0), Cable(max_voltage_V=1200.0,
                                                                                    altitude_design_m=4000.0)
        self.assertGreater(ceiling.thickness_insulation_m(), ground.thickness_insulation_m())
        self.assertLess(pressure_ratio(4000.0), 1.0)

    def test_minimum_wall_limit(self):
        cable = Cable(max_voltage_V=50.0)
        self.assertAlmostEqual(cable.thickness_insulation_m(), InsulationMaterial().thickness_min_m, delta=2e-5)
        self.assertGreaterEqual(cable.voltage_inception_V(), 1.5 * 50.0)

    def test_copper_heavier_but_lower_resistance_per_area(self):
        aluminium = Cable(conductor=aluminium_conductor())
        copper = Cable(conductor=copper_conductor())
        self.assertLess(copper.resistance_ohm() * copper.area_conductor_m2(),
                        aluminium.resistance_ohm() * aluminium.area_conductor_m2())
        self.assertGreater(copper.get_mass(), aluminium.get_mass())

    def test_mass_closed_form(self):
        cable = Cable(length_m=5.0, max_current_A=300.0, max_voltage_V=50.0, fraction_mass_accessories=0.0)
        area_m2, t_m = 1e-4, cable.thickness_insulation_m()
        radius_m = np.sqrt(area_m2 / np.pi)
        expected = 2 * 5.0 * (area_m2 * 2700.0 + np.pi * (2 * radius_m * t_m + t_m**2) * 1700.0)
        self.assertAlmostEqual(cable.get_mass(), expected, places=9)


class ProtectionTests(unittest.TestCase):
    def test_identities(self):
        unit = ProtectionUnit(max_current_A=1000.0)
        self.assertAlmostEqual(unit.get_mass(), 2 * (0.2 + 1.3))
        self.assertAlmostEqual(unit.evaluate(1000.0).voltage_drop_V, 0.3)
        self.assertAlmostEqual(unit.evaluate(500.0).power_loss_W, unit.resistance_ohm() * 500.0**2)
        self.assertAlmostEqual(unit.evaluate(-500.0).power_loss_W, unit.evaluate(500.0).power_loss_W)
        self.assertEqual(unit.get_limits().max_current_A, 1000.0)


def layer(dcdc=None):
    return ElectricalLayer(inverter_motor=Inverter(), cable_motor=Cable(), protection_motor=ProtectionUnit(),
                           inverter_generator=Inverter(), cable_generator=Cable(),
                           protection_generator=ProtectionUnit(), cable_battery=Cable(length_m=2.0),
                           protection_battery=ProtectionUnit(), dcdc=dcdc)


class TopologyTests(unittest.TestCase):
    def build(self, electrical):
        return build_series_hybrid(Motor(), Generator(), Battery(), SimpleTurboshaft(), Gearbox(),
                                   ActuatorDiskPropulsor(), count_rotors=2, count_turbogenerators=2,
                                   electrical=electrical)

    def test_ports(self):
        self.assertEqual([p.direction for p in port_specs_for(Inverter())], [Direction.IN, Direction.OUT])
        self.assertEqual([p.name for p in port_specs_for(Cable())], ["input", "output"])

    def test_insertion_and_counts(self):
        topology = self.build(layer())
        instances = topology.instances
        self.assertEqual(instances["inverter_motor"].count, 2)
        self.assertEqual(instances["cable_generator"].count, 2)
        self.assertEqual(instances["cable_battery"].count, 1)
        self.assertEqual(topology.get_port("inverter_generator.ac").direction, Direction.IN)
        bus_ports = sorted(c.source for c in topology.connections if c.target == "bus")
        self.assertEqual(bus_ports, ["cable_battery.output", "protection_generator.output", "protection_motor.input"])
        self.assertNotIn("dcdc", instances)

    def test_dcdc_sets_bus_voltage(self):
        topology = self.build(layer(DcDcConverter()))
        self.assertIn("dcdc.output", [c.source for c in topology.connections if c.target == "bus"])
        self.assertTrue(port_envelope(topology.instances["dcdc"].component, "output").sets_voltage)

    def test_design_margins_cover_feeders(self):
        labels = [m.label for m in design_margins(self.build(layer()))]
        self.assertIn("cable_motor.output->inverter_motor.dc max_power_W", labels)

    def test_without_layer_unchanged(self):
        self.assertNotIn("inverter_motor", self.build(None).instances)


class SymbolicTests(unittest.TestCase):
    def test_sizes_cable_area_against_loss(self):
        # Choose the design current (conductor area) to minimize mass + a loss penalty: exercises MX through
        # every Cable expression and checks the closed-form optimum dm/dA = penalty d(loss)/dA.
        opti = asb.Opti()
        current_design_A = opti.variable(init_guess=500.0, lower_bound=50.0)
        cable = Cable(length_m=10.0, max_current_A=current_design_A, max_voltage_V=50.0,
                      fraction_mass_accessories=0.0)
        penalty_kg_W = 0.01
        opti.minimize(cable.get_mass() + penalty_kg_W * cable.evaluate(400.0).power_loss_W)
        solution = opti.solve(verbose=False)
        area_m2 = solution.value(cable.area_conductor_m2())
        # Bare-conductor optimum (minimum wall adds a small, area-dependent term): A ~ I sqrt(rho p / d).
        expected_m2 = 400.0 * np.sqrt(3.4e-8 * penalty_kg_W / 2700.0)
        self.assertAlmostEqual(area_m2 / expected_m2, 1.0, delta=0.05)

    def test_inverter_rating_from_operating_point(self):
        opti = asb.Opti()
        power_rated_W = opti.variable(init_guess=300e3, scale=1e5, lower_bound=1e4)
        inverter = Inverter(power_rated_W=power_rated_W)
        result = inverter.evaluate(250e3, 700.0)
        opti.subject_to(result.power_ac_W <= inverter.get_limits().power_rated_W)
        opti.minimize(inverter.get_mass() + result.power_loss_W / 1e3)
        solution = opti.solve(verbose=False)
        self.assertGreaterEqual(solution.value(power_rated_W), 250e3 - 1e-3)
        self.assertGreater(solution.value(result.power_loss_W), 0)


if __name__ == "__main__":
    unittest.main()
