"""Plan 053: metal-tube coolant lines, one loop per wing, hose only across the nacelle tilt joints."""
import math
import unittest
from dataclasses import replace

from aircraft_closure.powertrain.components.compact_heat_exchanger import OffsetStripFinCore
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger

tube = NtuRamAirHeatExchanger(allowances="tube")


class TubeLineTests(unittest.TestCase):
    def test_loop_bore_keeps_the_velocity(self):
        # Two loops share the flow: each bore is D / sqrt(2), so area per unit flow (and velocity) is unchanged.
        self.assertAlmostEqual(tube.diameter_loop_m(), 0.0254 / math.sqrt(2), places=12)
        flow_area_m2 = tube.count_loops * math.pi * tube.diameter_loop_m()**2 / 4
        self.assertAlmostEqual(flow_area_m2, math.pi * 0.0254**2 / 4, places=12)

    def test_minimum_gauge_governs_at_loop_pressure(self):
        hoop_m = tube.diameter_loop_m() * tube.pressure_design_line_Pa / (2 * tube.stress_allowable_tube_Pa)
        self.assertLess(hoop_m, tube.thickness_min_tube_m)
        d, t = tube.diameter_loop_m(), tube.thickness_min_tube_m
        expected = (d + t) * math.pi * t * 2700.0 + math.pi * d**2 / 4 * tube.coolant.density_kg_m3
        self.assertAlmostEqual(tube.mass_tube_per_length_kg_m(), expected, places=12)

    def test_hand_calculation(self):
        # 26 m of line in two loops, 1.5 m of hose per loop at the tilt joint, the 1.7 kW pump.
        length_tube_m = 26.0 - 2 * 1.5
        expected = (1700.0 / 450.0 + length_tube_m * tube.mass_tube_per_length_kg_m()
                    + 3.0 * tube.mass_lines_per_length_kg_m(tube.diameter_loop_m()))
        self.assertAlmostEqual(tube.mass_pump_pipes_tube_kg(), expected, places=12)
        self.assertAlmostEqual(tube.mass_pump_pipes_tube_kg(), 15.6, delta=0.1)

    def test_lighter_than_hose_heavier_than_nothing(self):
        published = replace(tube, allowances="published")
        self.assertLess(tube.mass_breakdown().mass_pump_pipes_kg, 0.5 * published.mass_breakdown().mass_pump_pipes_kg)
        self.assertGreater(tube.mass_breakdown().mass_pump_pipes_kg, 1700.0 / 450.0)

    def test_system_lands_near_three_kw_per_kg(self):
        # The v3.7 core (plan 048: counts 726 x 11.2 x 21.9, fan 8.8 kW) at its 112 kW peak heat: the whole cooling
        # system lands in NASA's published 3.1-6.7 kW/kg band's neighbourhood (check, not input).
        cooler = replace(tube, core=OffsetStripFinCore(count_wide=726.4, count_long=11.17, count_tall=21.86),
                         power_rated_fan_W=8.8e3)
        specific_kW_kg = 111.9 / cooler.get_mass()
        self.assertTrue(2.8 < specific_kW_kg < 4.0, msg=f"{specific_kW_kg:.2f} kW/kg")

    def test_unknown_allowances_rejected(self):
        with self.assertRaises(ValueError):
            replace(tube, allowances="pipe").get_mass()


if __name__ == "__main__":
    unittest.main()
