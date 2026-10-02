import unittest

import aerosandbox as asb
import casadi as cas

from aircraft_closure.core.margins import margin_report
from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.powertrain.compatibility import (ElectricalEnvelope, MechanicalEnvelope,
                                                       battery_voltage_range_V, design_margins,
                                                       operating_margins, port_envelope)
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.topologies import build_series_hybrid


def series_hybrid(count_rotors=1, generator=None, battery=None):
    return build_series_hybrid(Motor(), generator or Generator(), battery or Battery(), SimpleTurboshaft(),
                               Gearbox(), ActuatorDiskPropulsor(), count_rotors=count_rotors)


def reference_port_values(torque_motor_Nm=200.0, voltage_bus_V=800.0, current_battery_A=20.0):
    gear = Gearbox().evaluate(400, torque_motor_Nm)
    return {
        "turboshaft.shaft": MechanicalPortValue(400, 250),
        "generator.shaft": MechanicalPortValue(400, 250),
        "generator.electrical": ElectricalPortValue(voltage_bus_V, 100),
        "battery.electrical": ElectricalPortValue(voltage_bus_V, current_battery_A),
        "motor.electrical": ElectricalPortValue(voltage_bus_V, 120),
        "motor.shaft": MechanicalPortValue(400, torque_motor_Nm),
        "gearbox.shaft_in": MechanicalPortValue(400, torque_motor_Nm),
        "gearbox.shaft_out": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
        "propulsor.shaft": MechanicalPortValue(gear.speed_output_rad_s, gear.torque_output_Nm),
    }


class EnvelopeTests(unittest.TestCase):
    def test_battery_voltage_range_satisfies_power_and_ohm(self):
        battery = Battery()
        min_voltage_V, max_voltage_V = battery_voltage_range_V(battery)
        current_discharge_A = (battery.voltage_open_circuit_V - min_voltage_V) / battery.resistance_ohm
        current_charge_A = (max_voltage_V - battery.voltage_open_circuit_V) / battery.resistance_ohm
        self.assertAlmostEqual(min_voltage_V * current_discharge_A, battery.max_discharge_power_W, places=4)
        self.assertAlmostEqual(max_voltage_V * current_charge_A, battery.max_charge_power_W, places=4)
        self.assertLess(min_voltage_V, battery.voltage_open_circuit_V)
        self.assertGreater(max_voltage_V, battery.voltage_open_circuit_V)

    def test_ideal_battery_range_collapses_to_ocv(self):
        for voltage_V in battery_voltage_range_V(Battery(resistance_ohm=1e-12)):
            self.assertAlmostEqual(voltage_V, 800.0, places=6)

    def test_component_envelopes(self):
        motor = Motor()
        self.assertEqual(port_envelope(motor, "shaft"), MechanicalEnvelope(1000.0, 500.0, 100000.0))
        self.assertEqual(port_envelope(motor, "electrical"), ElectricalEnvelope(400.0, 900.0, 100000.0))
        self.assertTrue(port_envelope(Battery(), "electrical").sets_voltage)
        self.assertEqual(port_envelope(SimpleTurboshaft(), "shaft"), MechanicalEnvelope(max_power_W=150000.0))
        self.assertIsNone(port_envelope(SimpleTurboshaft(), "fuel"))
        self.assertAlmostEqual(port_envelope(Gearbox(), "shaft_out").max_power_W, 97000.0)
        self.assertEqual(port_envelope(ActuatorDiskPropulsor(), "shaft"), MechanicalEnvelope(max_power_W=100000.0))
        with self.assertRaises(KeyError):
            port_envelope(motor, "fuel")


class OperatingMarginTests(unittest.TestCase):
    def test_reference_values(self):
        margins = {m.label: m.value for m in operating_margins(series_hybrid(), reference_port_values())}
        self.assertAlmostEqual(margins["motor power_shaft_W"], (100000 - 80000) / 100000)
        self.assertAlmostEqual(margins["motor torque_Nm"], (500 - 200) / 500)
        self.assertAlmostEqual(margins["motor speed_rad_s"], (1000 - 400) / 1000)
        self.assertAlmostEqual(margins["motor min_voltage_V"], (800 - 400) / 400)
        self.assertAlmostEqual(margins["motor max_voltage_V"], (900 - 800) / 900)
        self.assertAlmostEqual(margins["generator power_shaft_W"], 0.0)
        self.assertAlmostEqual(margins["turboshaft power_shaft_W"], (150000 - 100000) / 150000)
        self.assertAlmostEqual(margins["gearbox power_input_W"], 0.2)
        self.assertAlmostEqual(margins["propulsor power_shaft_W"], (100000 - 77600) / 100000)
        self.assertAlmostEqual(margins["battery discharge_power_W"], (100000 - 16000) / 100000)

    def test_turboshaft_margin_uses_power_available_at_altitude(self):
        engine = SimpleTurboshaft(lapse_exponent=1.0)
        atmosphere = asb.Atmosphere(altitude=3000)
        sigma = float(atmosphere.density() / asb.Atmosphere(altitude=0).density())
        topology = build_series_hybrid(Motor(), Generator(), Battery(), engine, Gearbox(), ActuatorDiskPropulsor())
        sea_level = {m.label: m.value for m in operating_margins(topology, reference_port_values())}
        high = {m.label: m.value for m in operating_margins(topology, reference_port_values(), atmosphere)}
        self.assertAlmostEqual(sea_level["turboshaft power_shaft_W"], (150000 - 100000) / 150000)
        self.assertAlmostEqual(float(high["turboshaft power_shaft_W"]), (150000 * sigma - 100000) / (150000 * sigma))

    def test_battery_charge_sign(self):
        margins = {m.label: m.value for m in operating_margins(
            series_hybrid(), reference_port_values(current_battery_A=-75))}
        self.assertAlmostEqual(margins["battery charge_power_W"], (50000 - 60000) / 50000)
        self.assertGreater(margins["battery discharge_power_W"], 1)

    def test_violations_are_negative_not_clipped(self):
        report = margin_report(operating_margins(series_hybrid(), reference_port_values(torque_motor_Nm=600)))
        self.assertEqual(report[0].label, "motor power_shaft_W")
        self.assertAlmostEqual(report[0].value, (100000 - 240000) / 100000)
        self.assertIn("motor torque_Nm", [e.label for e in report if not e.compatible])

    def test_missing_port_value_rejected(self):
        values = reference_port_values()
        del values["motor.shaft"]
        with self.assertRaisesRegex(KeyError, "motor.shaft"):
            operating_margins(series_hybrid(), values)

    def test_symbolic_port_values(self):
        opti = asb.Opti()
        torque_Nm = opti.variable(init_guess=100)
        values = reference_port_values()
        values["motor.shaft"] = MechanicalPortValue(400, torque_Nm)
        margins = operating_margins(series_hybrid(), values)
        self.assertIsInstance(next(m for m in margins if m.label == "motor torque_Nm").value, cas.MX)


class DesignMarginTests(unittest.TestCase):
    def test_default_series_hybrid(self):
        margins = {m.label: m.value for m in design_margins(series_hybrid())}
        self.assertAlmostEqual(margins["turboshaft.shaft->generator.shaft max_power_W"], -0.5)
        self.assertAlmostEqual(margins["motor.shaft->gearbox.shaft_in max_power_W"], 0.0)
        self.assertAlmostEqual(margins["gearbox.shaft_out->propulsor.shaft max_power_W"], 0.03)
        self.assertAlmostEqual(margins["bus power_W (loss-free)"], 1.0)
        min_voltage_V, max_voltage_V = battery_voltage_range_V(Battery())
        self.assertAlmostEqual(margins["bus battery.electrical->motor.electrical min_voltage_V"],
                               (min_voltage_V - 400) / 400)
        self.assertAlmostEqual(margins["bus battery.electrical->generator.electrical max_voltage_V"],
                               (900 - max_voltage_V) / 900)
        self.assertNotIn("motor.shaft->gearbox.shaft_in max_speed_rad_s", margins)

    def test_bus_power_scales_with_count(self):
        margins = {m.label: m.value for m in design_margins(series_hybrid(count_rotors=4))}
        self.assertAlmostEqual(margins["bus power_W (loss-free)"], (200000 - 400000) / 400000)

    def test_low_voltage_battery_flagged(self):
        margins = {m.label: m.value for m in design_margins(series_hybrid(
            battery=Battery(voltage_open_circuit_V=410.0)))}
        self.assertLess(margins["bus battery.electrical->motor.electrical min_voltage_V"], 0)

    def test_symbolic_rating_sizing(self):
        opti = asb.Opti()
        power_rated_generator_W = opti.variable(init_guess=100000, lower_bound=1)
        margins = design_margins(series_hybrid(generator=Generator(power_rated_W=power_rated_generator_W)))
        opti.minimize(power_rated_generator_W / 1e5)
        opti.subject_to([m.value >= 0 for m in margins])
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(power_rated_generator_W)), 150000, delta=1e-2)


if __name__ == "__main__":
    unittest.main()
