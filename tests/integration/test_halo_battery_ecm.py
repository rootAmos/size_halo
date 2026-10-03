"""Tier 17: Halo-class sizing with the equivalent-circuit battery (option; the reference keeps the Tier 1 battery)."""
import unittest
from dataclasses import replace

import aerosandbox.tools.units as u

from examples.halo_sizing import (HaloAssumptions, assumptions_tier12b, assumptions_tier17, solve_halo_sizing,
                                  soc_emergency_floor, soc_minimum)


class HaloEquivalentCircuitBatteryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = solve_halo_sizing()
        cls.ecm = solve_halo_sizing(assumptions=assumptions_tier17, objective="payload", initial=cls.base)
        cls.denser = solve_halo_sizing(assumptions=replace(assumptions_tier17, factor_power_density_battery=8.0),
                                       objective="payload", initial=cls.ecm)

    def test_reference_keeps_the_constant_battery(self):
        self.assertEqual(HaloAssumptions().battery_model, "constant")
        self.assertEqual(self.base.battery_trace, ())
        self.assertAlmostEqual(self.base.mass_takeoff_kg / u.lbm, 14875, delta=5)
        tier12b = solve_halo_sizing(assumptions=assumptions_tier12b, initial=self.base)
        self.assertAlmostEqual(tier12b.mass_takeoff_kg / u.lbm, 14875, delta=5)

    def test_ecm_sizing_solves_and_closes(self):
        r = self.ecm
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-5)
        self.assertGreater(r.design.count_parallel_battery, 1.0)
        self.assertGreaterEqual(r.soc_after_engine_out, soc_emergency_floor - 1e-6)

    def test_payload_falls_short_of_the_requirement_on_fixed_engines(self):
        """Plan 021 finding: with the 50G-shaped pack at end of life, 900 kg at 210 kt does not close."""
        self.assertLess(self.ecm.mass_payload_kg, 900.0)
        self.assertTrue(500.0 < self.ecm.mass_payload_kg < 800.0)
        self.assertGreater(self.denser.mass_payload_kg, self.ecm.mass_payload_kg)

    def test_low_soc_minimum_voltage_case(self):
        """The engine-out hover from SOC 0.30 ends at the pack's cutoff voltage (2.5 V x 210 cells)."""
        last = self.ecm.engine_out_trace[-1]
        self.assertAlmostEqual(last["voltage_end_V"], 2.5 * 210, delta=1.0)
        self.assertIn("engine-out hover 3/3: battery end voltage_V", self.ecm.binding)
        first = self.ecm.engine_out_trace[0]
        self.assertAlmostEqual(first["soc_start"], soc_minimum, places=6)
        self.assertLess(last["voltage_ocv_V"], first["voltage_ocv_V"])
        # Bus voltage sags well below OCV at the engine-out current (real sag, not < 1 % as before).
        self.assertGreater(1 - last["voltage_bus_V"] / last["voltage_ocv_V"], 0.1)

    def test_mission_voltage_follows_the_discharge_curve(self):
        trace = self.ecm.battery_trace
        self.assertEqual(len(trace), sum(HaloAssumptions().subsegments_mission))
        # OCV follows the 50G curve: monotonic in the mid-interval SOC (the optimizer may recharge in cruise).
        by_soc = sorted(trace, key=lambda row: row["soc_start"] + row["soc_end"])
        ocv = [row["voltage_ocv_V"] for row in by_soc]
        self.assertTrue(all(a <= b + 1e-6 for a, b in zip(ocv, ocv[1:])))
        self.assertGreater(ocv[-1] - ocv[0], 80.0)
        self.assertTrue(all(soc_minimum - 1e-6 <= row["soc_end"] <= 0.95 + 1e-6 for row in trace))
        self.assertTrue(all(525.0 - 1e-3 <= row["voltage_end_V"] <= 882.0 + 1e-3 for row in trace))


if __name__ == "__main__":
    unittest.main()
