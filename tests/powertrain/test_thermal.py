"""Tier 19 (plan 028): lumped thermal model, ram-air heat exchanger and heat loads."""
import math
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.powertrain.compatibility import operating_margins
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.heat_exchanger import (RamAirHeatExchanger, kelvin_offset_K,
                                                                   specific_heat_air_J_kg_K)
from aircraft_closure.powertrain.components.motor import Motor, rubber_machine
from aircraft_closure.powertrain.components.thermal import LumpedThermalModel
from aircraft_closure.thermal.heat import HeatLoad, thermal_parameters, total_heat_W
from examples.series_hybrid_point import build_reference_topology

thermal = LumpedThermalModel(specific_heat_J_kg_K=500.0, temperature_max_C=150.0, temperature_coolant_C=60.0)
sea_level = asb.Atmosphere(altitude=0.0)


class LumpedThermalModelTests(unittest.TestCase):
    def test_exponential_response_closed_form(self):
        capacity_J_K, resistance_K_W, loss_W, duration_s = 1.0e5, 0.003, 20000.0, 120.0
        tau_s = capacity_J_K * resistance_K_W
        steady_C = 60.0 + loss_W * resistance_K_W
        expected_C = steady_C + (70.0 - steady_C) * math.exp(-duration_s / tau_s)
        self.assertAlmostEqual(thermal.temperature_end_C(loss_W, duration_s, capacity_J_K, resistance_K_W, 70.0),
                               expected_C, places=10)
        self.assertEqual(LumpedThermalModel.time_constant_s(capacity_J_K, resistance_K_W), tau_s)

    def test_one_time_constant_covers_63_percent(self):
        end_C = thermal.temperature_end_C(10000.0, 300.0, 1e5, 0.003, 60.0)
        self.assertAlmostEqual((end_C - 60.0) / 30.0, 1 - math.exp(-1), places=12)

    def test_steady_state_limits(self):
        # No history (None) and a very long hold both give T_c + Q R; zero duration keeps the start.
        self.assertAlmostEqual(thermal.temperature_end_C(10000.0, 1.0, 1e5, 0.003), 90.0)
        self.assertAlmostEqual(thermal.temperature_end_C(10000.0, 1e6, 1e5, 0.003, 60.0), 90.0)
        self.assertAlmostEqual(thermal.temperature_end_C(10000.0, 0.0, 1e5, 0.003, 75.0), 75.0)
        self.assertAlmostEqual(thermal.temperature_end_C(10000.0, 0, 1e5, 0.003, 75.0), 75.0)

    def test_zero_loss_relaxes_to_coolant(self):
        self.assertAlmostEqual(thermal.temperature_end_C(0.0, 1e6, 1e5, 0.003, 140.0), 60.0)
        self.assertAlmostEqual(thermal.temperature_end_C(0.0, 50.0, 1e5, 0.003, 60.0), 60.0)
        self.assertAlmostEqual(thermal.temperature_steady_C(0.0, 0.003), 60.0)

    def test_trends_and_signs(self):
        # Hotter with more loss and longer holds; cools from above towards a lower steady state.
        a = thermal.temperature_end_C(10000.0, 60.0, 1e5, 0.003, 60.0)
        self.assertGreater(thermal.temperature_end_C(20000.0, 60.0, 1e5, 0.003, 60.0), a)
        self.assertGreater(thermal.temperature_end_C(10000.0, 120.0, 1e5, 0.003, 60.0), a)
        self.assertGreater(a, thermal.temperature_end_C(10000.0, 60.0, 2e5, 0.003, 60.0))  # more thermal mass
        cooling = thermal.temperature_end_C(10000.0, 60.0, 1e5, 0.003, 140.0)
        self.assertLess(cooling, 140.0)
        self.assertGreater(cooling, 90.0)

    def test_chaining_equals_one_interval(self):
        one = thermal.temperature_end_C(15000.0, 200.0, 1e5, 0.003, 60.0)
        half = thermal.temperature_end_C(15000.0, 100.0, 1e5, 0.003, 60.0)
        self.assertAlmostEqual(thermal.temperature_end_C(15000.0, 100.0, 1e5, 0.003, half), one, places=10)

    def test_mean_temperature_energy_balance(self):
        """Mean heat to the coolant = loss - C (T_end - T_0) / dt (what the thermal mass does not store)."""
        capacity_J_K, resistance_K_W, loss_W, duration_s = 1.0e5, 0.003, 25000.0, 90.0
        end_C = thermal.temperature_end_C(loss_W, duration_s, capacity_J_K, resistance_K_W, 70.0)
        mean_C = thermal.temperature_mean_C(loss_W, duration_s, capacity_J_K, resistance_K_W, 70.0)
        self.assertAlmostEqual((mean_C - 60.0) / resistance_K_W,
                               loss_W - capacity_J_K * (end_C - 70.0) / duration_s, places=6)
        self.assertGreater(end_C, mean_C)
        self.assertGreater(mean_C, 70.0)
        self.assertAlmostEqual(thermal.temperature_mean_C(loss_W, 0.0, capacity_J_K, resistance_K_W, 70.0), 70.0)
        self.assertAlmostEqual(thermal.temperature_mean_C(loss_W, 1.0, capacity_J_K, resistance_K_W), 135.0)

    def test_numeric_arrays(self):
        end_C = thermal.temperature_end_C(np.array([0.0, 1e4, 2e4]), 60.0, 1e5, 0.003, 60.0)
        self.assertEqual(end_C.shape, (3,))
        self.assertTrue(np.all(np.diff(end_C) > 0))

    def test_symbolic_through_opti(self):
        # Largest loss a 1e5 J/K, 0.003 K/W machine holds for 60 s from the coolant without passing 150 C.
        opti = asb.Opti()
        loss_W = opti.variable(init_guess=1e4, scale=1e4)
        duration_s = opti.variable(init_guess=60.0)
        opti.subject_to([duration_s == 60.0,
                         thermal.temperature_end_C(loss_W, duration_s, 1e5, 0.003, 60.0) <= 150.0])
        opti.maximize(loss_W / 1e4)
        s = opti.solve(verbose=False)
        expected_W = 90.0 / 0.003 / (1 - math.exp(-60.0 / 300.0))
        self.assertAlmostEqual(float(s.value(loss_W)) / expected_W, 1.0, places=6)
        self.assertGreater(expected_W, 90.0 / 0.003 * 5)     # a 60 s rating is > 5x the continuous loss


class MachineThermalParameterTests(unittest.TestCase):
    def test_continuous_rating_sits_on_the_limit(self):
        motor = rubber_machine(Motor, 400.0, 500.0, thermal_model=thermal)
        p = thermal_parameters(motor)
        speed_rad_s = motor.loss_model.speed_peak_efficiency_rad_s
        loss_W = motor.loss_model.evaluate(speed_rad_s, motor.power_rated_W / speed_rad_s, 800.0)
        self.assertAlmostEqual(p.power_loss_continuous_W, loss_W)
        self.assertAlmostEqual(thermal.temperature_steady_C(loss_W, p.resistance_K_W), 150.0)
        self.assertAlmostEqual(p.capacity_J_K, 500.0 * motor.get_mass())

    def test_battery_continuous_rating_sits_on_the_limit(self):
        battery_thermal = LumpedThermalModel(1000.0, 60.0, 25.0)
        pack = EquivalentCircuitBattery(count_parallel=20.0, factor_power_density=5.0, thermal_model=battery_thermal)
        p = thermal_parameters(pack)
        current_A = pack.get_limits().max_discharge_current_A
        loss_W = current_A**2 * sum(pack.resistances_ohm(0.5))
        self.assertAlmostEqual(p.power_loss_continuous_W, loss_W)
        self.assertAlmostEqual(battery_thermal.temperature_steady_C(loss_W, p.resistance_K_W), 60.0)
        self.assertAlmostEqual(p.capacity_J_K, 1000.0 * pack.get_mass())
        simple = Battery(thermal_model=battery_thermal)
        q = thermal_parameters(simple)
        self.assertAlmostEqual(q.power_loss_continuous_W, (100000.0 / 800.0)**2 * 0.05)

    def test_generator_and_symbolic_machine(self):
        opti = asb.Opti()
        torque_Nm = opti.variable(init_guess=500.0, scale=100.0)
        generator = rubber_machine(Generator, 400.0, torque_Nm, thermal_model=thermal)
        p = thermal_parameters(generator)
        opti.subject_to(torque_Nm == 500.0)
        s = opti.solve(verbose=False)
        numeric = thermal_parameters(rubber_machine(Generator, 400.0, 500.0, thermal_model=thermal))
        self.assertAlmostEqual(float(s.value(p.resistance_K_W)), numeric.resistance_K_W)

    def test_power_margin_replaced_by_thermal(self):
        """A machine with a thermal model has no power-rating margin (its rating is continuous)."""
        def labels(motor_thermal):
            topology = build_reference_topology(1, motor=Motor(thermal_model=motor_thermal))
            shaft = MechanicalPortValue(400.0, 200.0)
            electrical = ElectricalPortValue(800.0, 100.0)
            values = {"turboshaft.shaft": shaft, "generator.shaft": shaft, "generator.electrical": electrical,
                      "battery.electrical": electrical, "motor.electrical": electrical, "motor.shaft": shaft,
                      "gearbox.shaft_in": shaft, "gearbox.shaft_out": shaft, "propulsor.shaft": shaft}
            return [m.label for m in operating_margins(topology, values)]
        self.assertIn("motor power_shaft_W", labels(None))
        self.assertNotIn("motor power_shaft_W", labels(thermal))
        self.assertIn("motor torque_Nm", labels(thermal))
        self.assertIn("generator power_shaft_W", labels(thermal))


class HeatExchangerTests(unittest.TestCase):
    exchanger = RamAirHeatExchanger(power_rated_W=200000.0, specific_power_W_kg=1000.0, temperature_coolant_C=60.0,
                                    delta_temperature_ref_C=40.0, effectiveness=0.8, pressure_drop_ref_Pa=1000.0,
                                    efficiency_fan=0.6)

    def test_mass_and_limits(self):
        self.assertAlmostEqual(self.exchanger.get_mass(), 200.0)
        self.assertEqual(self.exchanger.get_limits().power_rated_W, 200000.0)

    def test_zero_heat_gives_zero_flow_drag_and_fan(self):
        for fan in (False, True):
            r = self.exchanger.evaluate(0.0, sea_level, 80.0, fan=fan)
            for value in (r.mass_flow_air_kg_s, r.pressure_drop_Pa, r.power_pumping_W, r.drag_N, r.power_fan_W,
                          r.power_heat_equivalent_W):
                self.assertEqual(value, 0.0)

    def test_rated_point_identities(self):
        """At the reference temperature difference and sea level the rated heat needs the rated flow and loss."""
        temperature_ambient_K = (60.0 - 40.0) + kelvin_offset_K
        atmosphere = asb.Atmosphere(altitude=0.0, temperature_deviation=temperature_ambient_K - sea_level.temperature())
        r = self.exchanger.evaluate(200000.0, atmosphere, 100.0)
        mass_flow_kg_s = 200000.0 / (0.8 * specific_heat_air_J_kg_K * 40.0)
        self.assertAlmostEqual(r.delta_temperature_C, 40.0, places=9)
        self.assertAlmostEqual(r.mass_flow_air_kg_s, mass_flow_kg_s, places=9)
        self.assertAlmostEqual(r.power_heat_equivalent_W, 200000.0, places=6)
        density_kg_m3 = atmosphere.density()
        pressure_drop_Pa = 1000.0 * sea_level.density() / density_kg_m3
        self.assertAlmostEqual(r.pressure_drop_Pa, pressure_drop_Pa, places=9)
        self.assertAlmostEqual(r.power_pumping_W, mass_flow_kg_s * pressure_drop_Pa / density_kg_m3, places=6)
        self.assertAlmostEqual(r.drag_N * 100.0, r.power_pumping_W, places=6)      # ram: D V = pumping power
        hover = self.exchanger.evaluate(200000.0, atmosphere, 0.0, fan=True)
        self.assertEqual(hover.drag_N, 0.0)
        self.assertAlmostEqual(hover.power_fan_W, r.power_pumping_W / 0.6, places=6)

    def test_trends(self):
        cruise = asb.Atmosphere(altitude=3048.0)
        hot = asb.Atmosphere(altitude=1219.0, temperature_deviation=27.7)
        base = self.exchanger.evaluate(100000.0, cruise, 100.0)
        # Pumping power ~ Q^3: doubling heat is 8x the drag.
        self.assertAlmostEqual(self.exchanger.evaluate(200000.0, cruise, 100.0).drag_N / base.drag_N, 8.0, places=9)
        # Faster flight: less drag for the same pumping power.
        self.assertAlmostEqual(self.exchanger.evaluate(100000.0, cruise, 200.0).drag_N / base.drag_N, 0.5, places=9)
        # A larger exchanger (more rated flow) has less pressure loss: drag ~ 1 / rating^2.
        bigger = replace(self.exchanger, power_rated_W=400000.0).evaluate(100000.0, cruise, 100.0)
        self.assertAlmostEqual(bigger.drag_N / base.drag_N, 0.25, places=9)
        # A hot day shrinks the temperature difference: more air, more fan power, a larger required rating.
        hot_hover = self.exchanger.evaluate(100000.0, hot, 0.0, fan=True)
        cold_hover = self.exchanger.evaluate(100000.0, sea_level, 0.0, fan=True)
        self.assertLess(hot_hover.delta_temperature_C, cold_hover.delta_temperature_C)
        self.assertGreater(hot_hover.mass_flow_air_kg_s, cold_hover.mass_flow_air_kg_s)
        self.assertGreater(hot_hover.power_fan_W, cold_hover.power_fan_W)
        self.assertGreater(hot_hover.power_heat_equivalent_W, cold_hover.power_heat_equivalent_W)
        self.assertGreater(base.drag_N, 0.0)

    def test_symbolic_rating_through_opti(self):
        """Smallest rating that covers two points, as in sizing (a margin per point, no loop)."""
        opti = asb.Opti()
        rating_W = opti.variable(init_guess=1e5, scale=1e5, lower_bound=1e3)
        exchanger = replace(self.exchanger, power_rated_W=rating_W)
        hot = asb.Atmosphere(altitude=1219.0, temperature_deviation=27.7)
        points = [exchanger.evaluate(150000.0, sea_level, 0.0, fan=True), exchanger.evaluate(120000.0, hot, 0.0, fan=True)]
        opti.subject_to([p.power_heat_equivalent_W <= rating_W for p in points])
        opti.minimize(exchanger.get_mass() / 100 + points[1].power_fan_W / 1e4)
        s = opti.solve(verbose=False)
        required_W = max(float(s.value(p.power_heat_equivalent_W)) for p in points)
        self.assertGreaterEqual(float(s.value(rating_W)), required_W * (1 - 1e-6))
        self.assertGreater(float(s.value(rating_W)), 150000.0)


class HeatLoadTests(unittest.TestCase):
    loads = (HeatLoad("motor", 30000.0, 2), HeatLoad("gearbox", 20000.0, 2), HeatLoad("generator", 25000.0, 1),
             HeatLoad("battery", 5000.0), HeatLoad("inverter", 4000.0, 2))

    def test_sum_with_counts_and_exclusions(self):
        self.assertAlmostEqual(total_heat_W(self.loads), 60000 + 40000 + 25000 + 5000 + 8000)
        self.assertAlmostEqual(total_heat_W(self.loads, ("gearbox",)), 98000.0)
        # A new source (here "inverter", Tier 15) is rejected by default, by name, with no code change.
        self.assertAlmostEqual(total_heat_W(self.loads[:4], ("gearbox",)) + 8000.0, total_heat_W(self.loads, ("gearbox",)))
        self.assertEqual(total_heat_W(()), 0.0)


if __name__ == "__main__":
    unittest.main()
