"""Plan 042: the installed ε-NTU ram-air cooler (core, duct, fan, mass allowances) and its margins."""
import unittest

import aerosandbox as asb

from aircraft_closure.powertrain.components.compact_heat_exchanger import OffsetStripFinCore, air_properties
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger, with_core_counts
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct

hot_day = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
cruise = asb.Atmosphere(altitude=3048.0)
exchanger = NtuRamAirHeatExchanger(core=OffsetStripFinCore(count_wide=600.0, count_long=15.0, count_tall=40.0),
                                   duct=RamAirDuct(area_nozzle_m2=0.15), power_rated_fan_W=8e3)


class MassTests(unittest.TestCase):
    def test_allowances_are_explicit(self):
        b = exchanger.mass_breakdown()
        self.assertAlmostEqual(b.mass_core_kg, exchanger.core.get_mass())
        self.assertAlmostEqual(b.mass_ducts_kg, 12.0 * exchanger.core.area_frontal_m2())
        self.assertAlmostEqual(b.mass_fans_kg, 8e3 / 2000.0)
        self.assertAlmostEqual(b.mass_pump_pipes_kg, 30.0)
        self.assertAlmostEqual(exchanger.get_mass(), b.mass_core_kg + b.mass_ducts_kg + 4.0 + 30.0)

    def test_design_variables_in_place(self):
        e = with_core_counts(exchanger, 300.0, 10.0, 20.0, 0.1, 5e3)
        self.assertEqual((e.core.count_wide, e.core.count_long, e.core.count_tall), (300.0, 10.0, 20.0))
        self.assertEqual(e.duct.area_nozzle_m2, 0.1)
        self.assertEqual(e.get_limits().power_rated_fan_W, 5e3)
        self.assertLess(e.get_mass(), exchanger.get_mass())


class HeatRejectionTests(unittest.TestCase):
    def test_needs_the_air_flow(self):
        with self.assertRaises(ValueError):
            exchanger.evaluate(50e3, hot_day)

    def test_margin_zero_where_core_rejects_the_load(self):
        # At a fixed flow, the heat Q* that zeroes the margin is what the core rejects with the coolant entering at
        # T_c + Q*/C_coolant (the coolant leaves at the loop temperature).
        r = exchanger.evaluate(80e3, hot_day, 0.0, True, 4.0, 800.0)
        k, c = r.conductance_effective_W_K, r.capacity_coolant_W_K
        heat_W = k * r.delta_temperature_C / (1 - k / c)
        _, margins = exchanger.margins(heat_W, r, "hot")
        self.assertAlmostEqual(margins[0].value, 0.0, places=9)
        core = exchanger.core.evaluate(4.0, exchanger.mass_flow_coolant_kg_s, hot_day.temperature(),
                                       273.15 + 60.0 + heat_W / c, air_properties(hot_day), exchanger.coolant)
        self.assertAlmostEqual(core.power_heat_W / heat_W, 1.0, places=6)
        self.assertAlmostEqual(core.temperature_coolant_out_K, 273.15 + 60.0, places=6)
        self.assertAlmostEqual(exchanger.power_heat_capacity_W(heat_W, r) / heat_W, 1.0, places=9)
        # More heat than that: negative margin; and the fan margin against the rating.
        _, margins = exchanger.margins(1.1 * heat_W, r, "hot")
        self.assertLess(margins[0].value, 0.0)
        self.assertAlmostEqual(margins[1].value, 1 - r.power_fan_W / 8e3, places=12)

    def test_duct_pressure_margin(self):
        # Hover without the fan: the core exit is below ambient, so the duct cannot expel the flow (negative margin);
        # with enough fan pressure it can.
        _, without_fan = exchanger.margins(50e3, exchanger.evaluate(50e3, hot_day, 0.0, True, 3.0, 0.0), "h")
        _, with_fan = exchanger.margins(50e3, exchanger.evaluate(50e3, hot_day, 0.0, True, 3.0, 2000.0), "h")
        self.assertEqual([m.label for m in with_fan], ["h: heat_exchanger heat_rejection", "h: heat_exchanger power_fan_W",
                                                       "h: heat_exchanger pressure_exit_Pa",
                                                       "h: heat_exchanger area_nozzle_m2"])
        self.assertLess(without_fan[2].value, 0.0)
        self.assertGreater(with_fan[2].value, 0.0)
        self.assertAlmostEqual(with_fan[3].value, 1 - 0.15 / exchanger.core.area_frontal_m2())

    def test_more_air_rejects_more(self):
        low = exchanger.evaluate(50e3, hot_day, 0.0, True, 2.0, 500.0)
        high = exchanger.evaluate(50e3, hot_day, 0.0, True, 6.0, 500.0)
        self.assertGreater(high.conductance_effective_W_K, low.conductance_effective_W_K)
        self.assertGreater(high.pressure_drop_Pa, low.pressure_drop_Pa)


class SizingInOptiTests(unittest.TestCase):
    def test_hot_day_hover_and_cruise_sizing(self):
        """Core counts, nozzle area and fan rating as variables; a hover and a cruise point each with a flow unknown
        (and a fan pressure rise in hover). Mass plus penalties on fan rating and cruise drag is minimized."""
        opti = asb.Opti()
        design = with_core_counts(
            exchanger, opti.variable(init_guess=600.0, scale=100.0, lower_bound=10.0),
            opti.variable(init_guess=15.0, scale=10.0, lower_bound=1.0),
            opti.variable(init_guess=40.0, scale=10.0, lower_bound=2.0),
            opti.variable(init_guess=0.15, scale=0.1, lower_bound=0.005),
            opti.variable(init_guess=1e4, scale=1e4, lower_bound=100.0))
        flow_hover = opti.variable(init_guess=3.0, scale=3.0, lower_bound=0.05)
        rise_fan = opti.variable(init_guess=500.0, scale=500.0, lower_bound=0.0)
        flow_cruise = opti.variable(init_guess=3.0, scale=3.0, lower_bound=0.05)
        hover = design.evaluate(70e3, hot_day, 0.0, True, flow_hover, rise_fan)
        level = design.evaluate(65e3, cruise, 85.0, False, flow_cruise)
        _, margins_hover = design.margins(80e3, hover, "hover")
        _, margins_cruise = design.margins(65e3, level, "cruise")
        opti.subject_to([hover.residual_mass_flow_kg_s / flow_hover == 0,
                         level.residual_mass_flow_kg_s / flow_cruise == 0]
                        + [m.value >= 0 for m in margins_hover + margins_cruise])
        # About 0.25 kg per kW of fan (battery and generator power) and 0.5 kg per N of cruise drag.
        opti.minimize(design.get_mass() + 0.25e-3 * design.power_rated_fan_W + 0.5 * level.drag_N)
        s = opti.solve(verbose=False)
        self.assertAlmostEqual(s(margins_hover[0].value), 0.0, delta=1e-6)       # hot-day hover sizes the core
        self.assertAlmostEqual(s(margins_hover[1].value), 0.0, delta=1e-6)       # fan rated at the hover need
        self.assertAlmostEqual(s(hover.residual_mass_flow_kg_s), 0.0, delta=1e-6)
        self.assertAlmostEqual(s(level.residual_mass_flow_kg_s), 0.0, delta=1e-6)
        self.assertGreater(s(hover.power_fan_W), 0.0)
        self.assertLess(s(design.get_mass()), 127.0)                             # the mass-per-watt reference
        self.assertLess(abs(s(level.drag_N)), 200.0)
        self.assertGreaterEqual(s(level.pressure_total_exit_Pa), cruise.pressure() - 1e-3)


if __name__ == "__main__":
    unittest.main()
