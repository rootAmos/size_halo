"""Plan 036: trajectory thermal states, cooling and lane motors (fast settings: Scholz aero, single lane)."""
import unittest
from dataclasses import replace

import numpy as np

from examples.halo_sizing import assumptions_plan030, requirements_plan030, solve_halo_sizing
from examples.trajectory_optimization import halo_trajectory_case, solve_min_energy_transition


class TrajectoryThermalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assumptions = replace(assumptions_plan030, aerodynamics_model="scholz")    # thermal on, rubber machines
        sizing = solve_halo_sizing(requirements_plan030, cls.assumptions)
        cls.case = halo_trajectory_case(sizing, requirements_plan030, cls.assumptions)
        cls.transition = solve_min_energy_transition(cls.case)

    def test_temperatures_are_states_within_limits(self):
        r = self.transition
        self.assertEqual(set(r.temperatures_C), {"motor", "generator", "battery"})
        for name, temperature in r.temperatures_C.items():
            thermal = self.case.model.instance(name).thermal_model
            self.assertAlmostEqual(temperature[0], thermal.temperature_coolant_C, places=6)     # cold start
            self.assertTrue(np.all(temperature <= thermal.temperature_max_C + 1e-6))
            self.assertGreater(temperature[-1], temperature[0])                                # losses heat it

    def test_cooling_is_fan_in_hover_and_drag_in_flight(self):
        r = self.transition
        # Cold start: no heat to reject at t = 0. Afterwards the fans' share of the pumping power falls with speed.
        self.assertAlmostEqual(r.power_fan_W[0], 0.0, places=9)
        share_fan = r.power_fan_W / (r.power_fan_W + r.drag_cooling_N * np.fmax(r.velocity_m_s, 1.0) + 1e-12)
        # Ram pressure covers the exchanger's pressure drop once q >> dp; dp grows with heat squared.
        self.assertGreater(share_fan[1], share_fan[-1])
        self.assertLess(share_fan[-1], 0.05)                                    # wing-borne: ram drag
        self.assertTrue(np.all(r.power_fan_W >= -1e-9) and np.all(r.drag_cooling_N >= -1e-9))


if __name__ == "__main__":
    unittest.main()
