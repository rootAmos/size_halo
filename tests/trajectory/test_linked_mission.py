"""Plan 069: a mission flown as linked trajectory segments on the notional all-electric aircraft."""
import unittest
from dataclasses import replace

import aerosandbox.numpy as np

from aircraft_closure.trajectory.mission import MissionSegment, SegmentEnd, solve_linked_mission
from aircraft_closure.trajectory.segments import _hover_estimates, default_corridor, solve_transition_climb
from aircraft_closure.trajectory.tiltrotor import TrajectoryGuess, TrajectoryLimits
from examples.all_electric_transition_climb import build_notional_trajectory_model

mass_kg = 1600.0
soc_initial = 0.95
altitude_hover_m = 15.0
altitude_cruise_m = 500.0
velocity_cruise_m_s = 70.0
voltage_guess_V = 700.0


def hover_start():
    return SegmentEnd(altitude_m=altitude_hover_m, velocity_m_s=1.0, gamma_deg=0.0, tilt_deg=90.0, trimmed=True)


def level(altitude_m=altitude_cruise_m, velocity_m_s=velocity_cruise_m_s):
    return SegmentEnd(altitude_m=altitude_m, velocity_m_s=velocity_m_s, gamma_deg=0.0, tilt_deg=0.0, trimmed=True)


def segments(model, count_nodes=5, cruise_split=False):
    """Hover 30 s, conversion and climb to level flight, cruise 21 km (optionally in two linked halves)."""
    thrust_N, speed_rad_s, power_W = _hover_estimates(model, mass_kg, 0.0)
    current_A = power_W / voltage_guess_V
    corridor = default_corridor(model, mass_kg)
    f = np.linspace(0, 1, count_nodes)
    ramp = np.fmin(f / 0.25, 1.0)
    hover_limits = replace(TrajectoryLimits(), velocity_max_m_s=1.0)
    out = [
        MissionSegment("hover", TrajectoryGuess(duration_s=30.0, velocity_m_s=1.0, altitude_m=altitude_hover_m,
                                                tilt_deg=90.0, alpha_deg=0.0, thrust_per_rotor_N=thrust_N,
                                                speed_rotor_rad_s=speed_rad_s, current_battery_A=current_A),
                       count_nodes=count_nodes, duration_bounds_s=(30.0, 30.0), end=hover_start(),
                       limits=hover_limits, corridor=corridor),
        MissionSegment("transition and climb", TrajectoryGuess(
            duration_s=160.0, velocity_m_s=1.0 + (velocity_cruise_m_s - 1.0) * ramp,
            altitude_m=altitude_hover_m + (altitude_cruise_m - altitude_hover_m) * f, tilt_deg=90 * (1 - ramp),
            gamma_deg=5.0 * np.sin(np.pi * f), thrust_per_rotor_N=thrust_N * (1 - 0.85 * ramp),
            speed_rotor_rad_s=speed_rad_s, current_battery_A=current_A * (1 - 0.5 * ramp)),
                       count_nodes=count_nodes, duration_bounds_s=(20.0, 900.0), end=level(),
                       altitude_min_m=altitude_hover_m, corridor=corridor),
    ]
    cruise = TrajectoryGuess(duration_s=300.0, velocity_m_s=velocity_cruise_m_s, altitude_m=altitude_cruise_m,
                             tilt_deg=0.0, thrust_per_rotor_N=0.1 * thrust_N, speed_rotor_rad_s=0.8 * speed_rad_s,
                             current_battery_A=0.25 * current_A)
    # Cruise held at its speed and altitude at every node (a cruise leg), so splitting it changes nothing.
    held = dict(limits=replace(TrajectoryLimits(), velocity_min_m_s=velocity_cruise_m_s,
                               velocity_max_m_s=velocity_cruise_m_s),
                configure=lambda opti, t: opti.subject_to(t.altitude_m == altitude_cruise_m), corridor=corridor)
    if cruise_split:
        half = replace(cruise, duration_s=150.0)
        out += [MissionSegment("cruise 1", half, count_nodes=count_nodes, distance_m=10500.0, end=level(), **held),
                MissionSegment("cruise 2", half, count_nodes=count_nodes, distance_m=10500.0, end=level(), **held)]
    else:
        out.append(MissionSegment("cruise", cruise, count_nodes=count_nodes, distance_m=21000.0, end=level(), **held))
    return out


class LinkedMissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = build_notional_trajectory_model()
        cls.battery = cls.model.instance("battery")
        kwargs = dict(mass_kg=mass_kg, soc_initial=soc_initial, start=hover_start())
        cls.mission = solve_linked_mission(cls.model, segments(cls.model), **kwargs)
        cls.split = solve_linked_mission(cls.model, segments(cls.model, cruise_split=True), **kwargs)
        cls.climb21 = solve_linked_mission(cls.model, segments(cls.model, count_nodes=21)[:2], **kwargs)

    def test_state_continuous_at_every_join(self):
        for result in (self.mission, self.split):
            for a, b in zip(result.segments[:-1], result.segments[1:]):
                for name in ("altitude_m", "velocity_m_s", "gamma_deg", "tilt_deg", "soc", "mass_kg"):
                    first, last = getattr(b, name)[0], getattr(a, name)[-1]
                    self.assertAlmostEqual(first, last, delta=1e-6 * max(1.0, abs(last)), msg=name)

    def test_boundary_conditions(self):
        r = self.mission
        hover, climb, cruise = r.segments
        self.assertAlmostEqual(hover.soc[0], soc_initial, places=9)
        self.assertAlmostEqual(hover.duration_s, 30.0, places=6)
        self.assertAlmostEqual(cruise.distance_m, 21000.0, delta=1e-3)
        self.assertAlmostEqual(climb.tilt_deg[-1], 0.0, delta=1e-6)
        self.assertAlmostEqual(cruise.altitude_m[-1], altitude_cruise_m, delta=1e-4)
        self.assertAlmostEqual(cruise.velocity_m_s[-1], velocity_cruise_m_s, delta=1e-6)
        self.assertLessEqual(np.max(hover.velocity_m_s), 1.0 + 1e-6)

    def test_split_cruise_reproduces_single_segment(self):
        """Two linked halves of a steady level cruise cost the same energy as one segment."""
        self.assertAlmostEqual(self.split.energy_electric_J / self.mission.energy_electric_J, 1.0, delta=1e-3)
        self.assertAlmostEqual(self.split.segment("cruise 1").energy_electric_J
                               + self.split.segment("cruise 2").energy_electric_J,
                               self.mission.segment("cruise").energy_electric_J,
                               delta=1e-3 * self.mission.segment("cruise").energy_electric_J)

    def test_linked_climb_matches_stand_alone_segment(self):
        """Plan 068's segment started at the hover's end SOC: within 2 % (same nodes, same boundary states)."""
        hover = self.climb21.segment("hover")
        alone = solve_transition_climb(self.model, mass_kg=mass_kg, soc_initial=float(hover.soc[-1]),
                                       altitude_initial_m=altitude_hover_m, altitude_final_m=altitude_cruise_m,
                                       velocity_final_m_s=velocity_cruise_m_s, count_nodes=21)
        linked = self.climb21.segment("transition and climb")
        self.assertAlmostEqual(linked.energy_electric_J / alone.energy_electric_J, 1.0, delta=0.02)

    def test_soc_drop_is_the_chemical_energy(self):
        r = self.mission
        drop = r.segments[0].soc[0] - r.segments[-1].soc[-1]
        self.assertAlmostEqual(drop * self.battery.energy_capacity_J / r.energy_battery_chemical_J, 1.0, delta=2e-3)

    def test_totals_and_counted_segments(self):
        r = self.mission
        self.assertAlmostEqual(r.duration_s, sum(s.duration_s for s in r.segments), places=6)
        self.assertAlmostEqual(r.energy_electric_J, sum(s.energy_electric_J for s in r.segments), places=3)
        uncounted = segments(self.model)
        uncounted[0] = replace(uncounted[0], counted=False)
        u = solve_linked_mission(self.model, uncounted, mass_kg=mass_kg, soc_initial=soc_initial, start=hover_start())
        self.assertEqual(u.counted, (False, True, True))
        self.assertAlmostEqual(u.energy_electric_J, u.segments[1].energy_electric_J + u.segments[2].energy_electric_J,
                               places=3)

    def test_mission_level_distance(self):
        """A constraint across segments: the two cruise halves free, their sum 21 km; same energy as one leg."""
        s = segments(self.model, cruise_split=True)
        s[2], s[3] = replace(s[2], distance_m=None), replace(s[3], distance_m=None)
        r = solve_linked_mission(self.model, s, mass_kg=mass_kg, soc_initial=soc_initial, start=hover_start(),
                                 configure=lambda opti, ts: opti.subject_to(ts[2].x_m[-1] + ts[3].x_m[-1] == 21000.0))
        self.assertAlmostEqual(r.segments[2].distance_m + r.segments[3].distance_m, 21000.0, delta=1e-3)
        self.assertAlmostEqual(r.energy_electric_J / self.mission.energy_electric_J, 1.0, delta=1e-3)

    def test_rejects_bad_input(self):
        s = segments(self.model)
        with self.assertRaises(ValueError):
            solve_linked_mission(self.model, s, mass_kg=mass_kg, soc_initial=soc_initial, objective="range")
        with self.assertRaises(ValueError):
            solve_linked_mission(self.model, [s[0], s[0]], mass_kg=mass_kg, soc_initial=soc_initial)


if __name__ == "__main__":
    unittest.main()
