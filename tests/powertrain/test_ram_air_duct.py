"""Plan 042: ram-air duct with core loss, heat addition, fan and nozzle."""
import unittest

import aerosandbox as asb

from aircraft_closure.powertrain.components.compact_heat_exchanger import OffsetStripFinCore, air_properties
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct

cruise = asb.Atmosphere(altitude=3048.0)
hot_day = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
lossless = RamAirDuct(recovery_inlet=1.0, loss_diffuser=0.0, coefficient_velocity_nozzle=1.0)


class RamAirDuctTests(unittest.TestCase):
    def test_lossless_cold_duct_has_no_force(self):
        # No heat, no losses: the exit velocity equals the flight speed, so the net force vanishes.
        r = lossless.evaluate(5.0, 0.0, 0.0, 0.5, cruise, 85.0)
        self.assertAlmostEqual(r.velocity_exit_m_s, 85.0, delta=1e-6)
        self.assertAlmostEqual(r.force_net_N, 0.0, delta=1e-4)

    def test_losses_make_drag(self):
        r = lossless.evaluate(5.0, 0.0, 800.0, 0.5, cruise, 85.0)
        self.assertGreater(r.drag_N, 0.0)
        # Small-loss limit: drag x V is about the flow work lost, ṁ Δp / ρ.
        self.assertAlmostEqual(r.drag_N * 85.0 / (5.0 * 800.0 / cruise.density()), 1.0, delta=0.05)

    def test_meredith_heat_recovers_thrust(self):
        duct = RamAirDuct()
        cold = duct.evaluate(5.0, 0.0, 800.0, 0.5, cruise, 85.0)
        hot = duct.evaluate(5.0, 100e3, 800.0, 0.5, cruise, 85.0)
        self.assertLess(hot.drag_N, cold.drag_N)
        self.assertGreater(hot.temperature_total_exit_K, cold.temperature_total_exit_K)

    def test_hover_needs_the_fan(self):
        duct = RamAirDuct()
        no_fan = duct.evaluate(5.0, 100e3, 800.0, 0.5, hot_day, 0.0)
        with_fan = duct.evaluate(5.0, 100e3, 800.0, 0.5, hot_day, 0.0, pressure_rise_fan_Pa=1500.0)
        self.assertLess(no_fan.pressure_total_core_exit_Pa, hot_day.pressure())   # cannot push air out
        self.assertGreater(with_fan.velocity_exit_m_s, 1.0)
        density_kg_m3 = hot_day.pressure() / (287.05 * hot_day.temperature())
        self.assertAlmostEqual(with_fan.power_fan_W, 5.0 * 1500.0 / (density_kg_m3 * 0.55), delta=1e-6)

    def test_flow_solved_by_opti_with_the_core(self):
        # One flow variable per point: the duct residual and the core's pressure drop at that flow, in cruise.
        core = OffsetStripFinCore(count_wide=600.0, count_tall=40.0, count_long=15.0)
        air = air_properties(cruise)
        opti = asb.Opti()
        mass_flow_kg_s = opti.variable(init_guess=5.0, lower_bound=0.1)
        core_result = core.evaluate(mass_flow_kg_s, 3.0, cruise.temperature(), 273.15 + 60.0, air)
        duct = RamAirDuct(area_nozzle_m2=0.06)
        r = duct.evaluate(mass_flow_kg_s, core_result.power_heat_W, core_result.air.pressure_drop_Pa,
                          core.area_frontal_m2(), cruise, 85.0)
        opti.subject_to(r.residual_mass_flow_kg_s == 0)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution(r.residual_mass_flow_kg_s), 0.0, delta=1e-6)
        self.assertLess(solution(r.mach_exit), 0.5)
        self.assertGreater(solution(core_result.power_heat_W), 50e3)
        self.assertLess(abs(solution(r.drag_N)), 300.0)


if __name__ == "__main__":
    unittest.main()
