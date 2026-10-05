"""Plan 036: trim drag from the tail load (Scholz ch. 11) with the downwash correction."""
import unittest
from dataclasses import replace

import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.trim import TailTrim
from tests.integration.test_halo_thermal import numeric_design
from examples.halo_sizing import assumptions_plan030, build_halo_aircraft

aircraft = build_halo_aircraft(numeric_design(), assumptions=assumptions_plan030)


class TailTrimTests(unittest.TestCase):
    def test_trimmed_moment_needs_no_tail_lift_and_no_drag(self):
        trim = TailTrim()
        self.assertEqual(trim.lift_tail_increment(aircraft, 0.0, 1.5), 0.0)
        self.assertAlmostEqual(trim.drag_coefficient(aircraft, 0.5, 0.0, 1.5, 0.8, 6.0), 0.0, places=12)

    def test_tail_lift_balances_the_moment(self):
        trim = TailTrim()
        lift = trim.lift_tail_increment(aircraft, -0.05, 1.5)
        self.assertAlmostEqual(-0.05 - trim.volume_tail(aircraft, 1.5) * lift, 0.0, places=12)
        self.assertLess(lift, 0.0)                     # nose-down moment: the tail pushes down

    def test_dihedral_and_efficiency_need_more_tail_lift(self):
        base = TailTrim().lift_tail_increment(aircraft, -0.05, 1.5)
        v_tail = TailTrim(angle_dihedral_tail_deg=40.0).lift_tail_increment(aircraft, -0.05, 1.5)
        weak = TailTrim(efficiency_tail=0.8).lift_tail_increment(aircraft, -0.05, 1.5)
        self.assertAlmostEqual(v_tail / base, 1 / np.cosd(40.0), places=12)
        self.assertAlmostEqual(weak / base, 0.9 / 0.8, places=12)

    def test_downwash_relieves_a_tail_heavy_moment(self):
        trim = TailTrim()
        self.assertGreater(trim.moment_downwash_correction(aircraft, 0.6, 1.5, 6.0, 4.0), 0.0)
        with_downwash = trim.drag_coefficient(aircraft, 0.6, -0.1, 1.5, 0.8, 6.0, cl_alpha_tail_per_rad=4.0)
        without = trim.drag_coefficient(aircraft, 0.6, -0.1, 1.5, 0.8, 6.0, cl_alpha_tail_per_rad=0.0)
        self.assertLess(with_downwash, without)


if __name__ == "__main__":
    unittest.main()
