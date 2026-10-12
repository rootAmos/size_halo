"""Plan 064: the per-node mission trace (`examples.halo_sizing.node_trace`) on stand-in segments, no solve."""
import math
import unittest
from types import SimpleNamespace as N

from examples.halo_sizing import node_trace


def point(label, mode="airplane", velocity_m_s=80.0, climb_rate_m_s=0.0, alpha_deg=4.0, engines=None):
    return N(condition=N(label=label, mode=mode, velocity_m_s=velocity_m_s, climb_rate_m_s=climb_rate_m_s,
                         altitude_m=1000.0, active_generator_count=engines),
             alpha_deg=alpha_deg if mode == "airplane" else None,
             aero=N(cl=0.5, cd=0.05) if mode == "airplane" else None, weight_N=6e4, fuel_flow_kg_s=0.05,
             thrust_per_rotor_N=5e3, speed_rotor_rad_s=40.0, power_shaft_rotor_W=2e5, speed_motor_rad_s=100.0,
             torque_motor_Nm=500.0, power_electric_motors_W=4.4e5, engine=N(power_shaft_W=3e5),
             power_battery_W=1e4, hybridization_electric=0.02, battery=N(voltage_V=800.0, current_A=12.5),
             thermal=N(temperatures_end_C={"motor": 80.0}))


def node(p, soc=0.9):
    return N(point=p, mass_start_kg=6000.0, soc_start=soc, duration_s=1.0)


class NodeTraceTests(unittest.TestCase):
    def setUp(self):
        hover = N(segment=N(duration_s=lambda: 60.0), subsegments=(node(point("hover", "hover", 0.0, 0.0)),))
        climb_points = [point(f"climb {i}/2", climb_rate_m_s=6.0) for i in range(3)]
        climb = N(segment=N(duration_s=lambda: 500.0), subsegments=tuple(node(p) for p in climb_points))
        descent = N(segment=N(duration_s=lambda: 600.0),
                    subsegments=tuple(node(point(f"descent {i}/1", climb_rate_m_s=-5.0, engines=1)) for i in range(2)))
        self.rows = node_trace((hover, climb, descent), 2, 2, float)

    def test_node_times(self):
        # A segment's nodes are instants evenly spaced over it; the next segment starts where it ended.
        self.assertEqual([r["time_s"] for r in self.rows], [0.0, 60.0, 310.0, 560.0, 560.0, 1160.0])

    def test_flight_path_and_pitch(self):
        climb = self.rows[1]
        gamma = math.degrees(math.asin(6.0 / 80.0))
        self.assertAlmostEqual(climb["flight_path_deg"], gamma)
        self.assertAlmostEqual(climb["pitch_deg"], 4.0 + gamma)
        self.assertLess(self.rows[-1]["flight_path_deg"], 0.0)
        # Hover: no attitude in the model, level flight path.
        self.assertIsNone(self.rows[0]["alpha_deg"])
        self.assertIsNone(self.rows[0]["pitch_deg"])
        self.assertEqual(self.rows[0]["flight_path_deg"], 0.0)

    def test_powers_count_running_units(self):
        self.assertEqual(self.rows[1]["power_shaft_turboshafts_W"], 6e5)    # both engines
        self.assertEqual(self.rows[-1]["power_shaft_turboshafts_W"], 3e5)   # single-engine descent
        self.assertEqual(self.rows[1]["power_shaft_rotors_W"], 4e5)
        self.assertEqual(self.rows[1]["power_shaft_motors_W"], 1e5)


if __name__ == "__main__":
    unittest.main()
