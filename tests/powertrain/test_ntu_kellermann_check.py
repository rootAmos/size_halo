"""Plan 051: the ε-NTU cooler at Kellermann et al.'s 2021 design point, published TMS masses, published allowances."""
import math
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.powertrain.components.compact_heat_exchanger import OffsetStripFinCore
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct
from examples.ntu_kellermann_check import (KellermannDesignPoint, KellermannReference, plan042_cooler,
                                           pressure_drop_lines_Pa, published_breakdowns, size_at_kellermann)

point = KellermannDesignPoint()
reference = KellermannReference()
check = size_at_kellermann(point)


class DesignPointTests(unittest.TestCase):
    def test_inputs_from_the_paper(self):
        # T_in = T1 + ε_cp (T_cp - T1) = 275 + 0.4 x 105 = 317 K; C = 96.3 kW / 42 K; C*_R = 1 sets the air flow.
        self.assertAlmostEqual(point.temperature_coolant_inlet_K(), 317.0)
        self.assertAlmostEqual(point.capacity_coolant_W_K(), 96.3e3 / 42.0)
        self.assertAlmostEqual(check.mass_flow_air_kg_s * 1005.0, point.capacity_coolant_W_K())
        self.assertAlmostEqual(point.velocity_m_s() / point.atmosphere().speed_of_sound(), 0.68)
        self.assertAlmostEqual(reference.area_frontal_m2(), 0.73 * 0.18)

    def test_sized_core_meets_the_point(self):
        self.assertAlmostEqual(check.power_heat_W / point.power_heat_W, 1.0, places=4)
        self.assertAlmostEqual(check.ratio_pressure_core, 0.95, places=6)
        self.assertLessEqual(check.pressure_drop_coolant_Pa, point.pressure_drop_coolant_max_Pa * (1 + 1e-6))
        # Effectiveness from the temperatures alone: Q / (C_min (T_hot,in - T_air,in)); C_air = C_coolant here, so
        # the core's smooth min of the two sits 0.07 % below either (softness 1e-3 C_air).
        self.assertAlmostEqual(check.effectiveness, point.power_heat_W / (point.capacity_coolant_W_K()
                                                                          * (317.0 - check.temperature_air_inlet_K)),
                               delta=1e-3)

    def test_system_level_comparison(self):
        # Same heat, inlet state and Π_c: the frontal area is within a factor 1.5 of Kellermann's (set by the air
        # flow and the pressure budget, not the surface), while the offset-strip core is far shorter in the air
        # direction and smaller in volume (higher j per unit length than plain channels of d_h 5-10 mm).
        self.assertLess(abs(math.log(check.core.area_frontal_m2() / reference.area_frontal_m2())), math.log(1.5))
        self.assertLess(check.core.length_m(), 0.5 * reference.length_air_m)
        self.assertLess(check.volume_core_m3(), 0.5 * reference.volume_core_m3())
        # Drag: tens of newtons at most, against a few newtons published (our inlet keeps 95 % of the ram rise).
        self.assertLess(abs(check.drag_N), 50.0)
        self.assertGreater(check.drag_N, reference.drag_tms_N)

    def test_trends(self):
        # A better inlet cuts drag at the same core; a looser Π_c budget (0.98) needs a bigger face and less drag.
        ideal = size_at_kellermann(point, RamAirDuct(recovery_inlet=1.0))
        self.assertLess(ideal.drag_N, check.drag_N)
        looser = size_at_kellermann(replace(point, ratio_pressure_core=0.98))
        self.assertGreater(looser.core.area_frontal_m2(), check.core.area_frontal_m2())
        self.assertLess(looser.drag_N, check.drag_N)


class PublishedBreakdownTests(unittest.TestCase):
    def test_totals_as_printed(self):
        values = [b.total_kg() for b in published_breakdowns()]
        self.assertAlmostEqual(values[1], 176.2, places=1)      # Chapman 2020 Table 4 total
        self.assertAlmostEqual(values[3], 54.04, places=2)      # Chapman 2021 Table 8 baseline
        self.assertAlmostEqual(values[4], 109.33, places=2)     # Chapman 2021 Table 6 corrected baseline

    def test_specific_powers_bracket_the_halo_cooler(self):
        # Published whole-system ratios span about 1.7 to 7 kW/kg; the plan 042 cooler (48 kg for the 121 kW peak)
        # sits inside that range.
        ratios = [b.specific_power_W_kg() for b in published_breakdowns()]
        self.assertTrue(all(1.5e3 < r < 7e3 for r in ratios))
        halo = 121e3 / plan042_cooler().get_mass()
        self.assertTrue(min(ratios) < halo < max(ratios))


class PublishedAllowanceTests(unittest.TestCase):
    def test_default_is_the_estimate(self):
        self.assertEqual(NtuRamAirHeatExchanger().allowances, "estimate")
        self.assertAlmostEqual(plan042_cooler().mass_breakdown().mass_pump_pipes_kg, 30.0)
        self.assertAlmostEqual(plan042_cooler().get_mass(), 47.6, delta=0.1)   # plan 042's reported cooler

    def test_hose_by_hand(self):
        # OpenConcept SimpleHose defaults: t = 0.0254 x 1.03e6 / (2 x 2.07e6) = 6.32 mm.
        cooler = plan042_cooler("published")
        t = 0.0254 * 1.03e6 / (2 * 2.07e6)
        per_m = (0.0254 + t) * math.pi * t * 1356.3 + math.pi * 0.0254**2 / 4 * cooler.coolant.density_kg_m3
        self.assertAlmostEqual(cooler.mass_lines_per_length_kg_m(), per_m)
        self.assertAlmostEqual(per_m, 1.38, delta=0.01)
        b = cooler.mass_breakdown()
        self.assertAlmostEqual(b.mass_pump_pipes_kg, 1700.0 / 450.0 + 26.0 * per_m)
        self.assertAlmostEqual(cooler.get_mass() - plan042_cooler().get_mass(), b.mass_pump_pipes_kg - 30.0)

    def test_pump_rating_supported(self):
        # 2 kg/s through about 150 kPa of core and the line loss, at OpenConcept's pump efficiency 0.35, needs about
        # the 1.7 kW rating.
        cooler = plan042_cooler("published")
        loss_Pa = 150e3 + pressure_drop_lines_Pa(cooler)
        power_W = cooler.mass_flow_coolant_kg_s * loss_Pa / cooler.coolant.density_kg_m3 / 0.35
        self.assertAlmostEqual(power_W / cooler.power_rated_pump_W, 1.0, delta=0.05)

    def test_unknown_allowances_rejected(self):
        with self.assertRaises(ValueError):
            replace(plan042_cooler(), allowances="guess").get_mass()

    def test_symbolic(self):
        opti = asb.Opti()
        log_wide = opti.variable(init_guess=np.log(500.0))
        cooler = replace(plan042_cooler("published"), core=OffsetStripFinCore(count_wide=np.exp(log_wide)))
        opti.subject_to(cooler.core.width_m() == 0.5)
        opti.minimize(cooler.get_mass())
        s = opti.solve(verbose=False)
        self.assertAlmostEqual(s(cooler.core.width_m()), 0.5, places=6)
        self.assertGreater(s(cooler.get_mass()), cooler.mass_pump_pipes_published_kg())


if __name__ == "__main__":
    unittest.main()
