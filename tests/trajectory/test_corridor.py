"""Plan 039: computed conversion corridor and level-flight trim."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.controls.stability import LongitudinalStability
from aircraft_closure.performance.flight_point import acceleration_gravity_m_s2
from aircraft_closure.trajectory.corridor import (ComputedCorridor, CorridorBound, CorridorLimits, TrimGeometry, moment_rotor_Nm, solve_corridor_bound,
                                                  solve_trim)
from aircraft_closure.trajectory.tiltrotor import TiltrotorPointMass
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, build_halo_aerodynamics, build_halo_aircraft,
                                  pre_layout, pre_plan035)
from tests.integration.test_halo_thermal import numeric_design

geometry_level = TrimGeometry(x_cg_m=0.0, z_cg_m=0.0, x_spindle_m=0.0, z_spindle_m=0.0, length_mast_m=1.0)


class RotorMomentTests(unittest.TestCase):
    def test_signs(self):
        ahead = replace(geometry_level, x_spindle_m=-1.0, length_mast_m=0.0)      # hub 1 m ahead of the CG
        self.assertAlmostEqual(moment_rotor_Nm(100.0, 90.0, 90.0, ahead), 100.0)    # lift ahead: nose-up
        below = replace(geometry_level, z_spindle_m=-1.0, length_mast_m=0.0)
        self.assertAlmostEqual(moment_rotor_Nm(100.0, 0.0, 0.0, below), 100.0)      # forward thrust below: nose-up

    def test_hub_on_the_shaft(self):
        """Thrust along the shaft through a spindle at the CG has no moment, whatever the mast length."""
        for tilt_deg in (0.0, 30.0, 75.0, 90.0):
            self.assertAlmostEqual(moment_rotor_Nm(1e4, tilt_deg, tilt_deg, geometry_level), 0.0, places=6)

    def test_disc_tilt_at_the_hub(self):
        """In hover, disc tilt theta at a hub h above the CG: M = T h sin(theta) (thrust tilted forward pitches down)."""
        moment_Nm = moment_rotor_Nm(1e4, 90.0, 85.0, geometry_level)
        self.assertAlmostEqual(moment_Nm, -1e4 * 1.0 * np.sind(5.0), places=6)

    def test_symbolic(self):
        opti = asb.Opti()
        thrust_N = opti.variable(init_guess=1.0)
        opti.subject_to(moment_rotor_Nm(thrust_N, 80.0, 75.0, geometry_level) == -2.0)
        self.assertAlmostEqual(float(opti.solve(verbose=False).value(thrust_N)), 2.0 / np.sind(5.0), places=5)


class ComputedCorridorTests(unittest.TestCase):
    """The computed corridor as trajectory limits: piecewise linear in nacelle angle, symbolic."""

    def setUp(self):
        trim = lambda v: type("Trim", (), {"velocity_m_s": v})()        # noqa: E731
        bounds = [(CorridorBound(90.0, "low", trim(0.0), ()), CorridorBound(90.0, "high", trim(67.0), ())),
                  (CorridorBound(0.0, "low", trim(61.0), ()), CorridorBound(0.0, "high", trim(113.0), ())),
                  (CorridorBound(45.0, "low", trim(55.0), ()), CorridorBound(45.0, "high", None, ())),
                  (CorridorBound(60.0, "low", trim(46.0), ()), CorridorBound(60.0, "high", trim(74.0), ()))]
        self.corridor = ComputedCorridor.from_bounds(bounds, velocity_stall_m_s=50.0)

    def test_nodes_sorted_and_untrimmed_angles_dropped(self):
        self.assertEqual(self.corridor.tilts_deg, (0.0, 60.0, 90.0))

    def test_node_recovery(self):
        for tilt, low, high in zip(self.corridor.tilts_deg, self.corridor.velocity_low_m_s,
                                   self.corridor.velocity_high_m_s):
            self.assertAlmostEqual(float(self.corridor.velocity_min_m_s(tilt)), low)
            self.assertAlmostEqual(float(self.corridor.velocity_max_m_s(tilt)), high)

    def test_linear_between_nodes_and_clamped_outside(self):
        self.assertAlmostEqual(float(self.corridor.velocity_min_m_s(75.0)), 23.0)
        self.assertAlmostEqual(float(self.corridor.velocity_max_m_s(30.0)), 93.5)
        self.assertAlmostEqual(float(self.corridor.velocity_max_m_s(95.0)), 67.0)

    def test_symbolic(self):
        opti = asb.Opti()
        tilt_deg = opti.variable(init_guess=50.0, lower_bound=0.0, upper_bound=90.0)
        opti.subject_to(self.corridor.velocity_min_m_s(tilt_deg) == 23.0)
        self.assertAlmostEqual(float(opti.solve(verbose=False).value(tilt_deg)), 75.0, places=4)

    def test_symbolic_vector(self):
        opti = asb.Opti()
        tilt_deg = opti.variable(init_guess=np.linspace(0.0, 90.0, 7))
        opti.subject_to(tilt_deg == np.linspace(0.0, 90.0, 7))
        values = opti.solve(verbose=False).value(self.corridor.velocity_max_m_s(tilt_deg))
        expected = [float(self.corridor.velocity_max_m_s(t)) for t in np.linspace(0.0, 90.0, 7)]
        for value, target in zip(np.asarray(values).ravel(), expected):
            self.assertAlmostEqual(float(value), target, places=6)


class CorridorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        r, a = HaloRequirements(), HaloAssumptions(**pre_plan035, **pre_layout)
        aircraft = build_halo_aircraft(numeric_design(), r, a)
        cls.model = TiltrotorPointMass(aircraft, build_halo_aerodynamics(r, a))
        wing = aircraft.wing
        x_spindle_m = wing.x_le_root_m + 0.25 * wing.chord_root_m()
        cls.geometry = TrimGeometry(x_cg_m=x_spindle_m, z_cg_m=wing.z_m - 0.6, x_spindle_m=x_spindle_m,
                                    z_spindle_m=wing.z_m, length_mast_m=1.0)
        cls.limits = CorridorLimits(velocity_placard_m_s=120.0)
        rotor = cls.model.instance("propulsor")
        cls.speed_rotor_rad_s = 220.0 / rotor.radius_m()
        cls.common = dict(mass_kg=6000.0, altitude_m=0.0, speed_rotor_rad_s=cls.speed_rotor_rad_s)
        cls.stability = LongitudinalStability()

    def bound(self, tilt_deg, side):
        return solve_corridor_bound(self.model, self.stability, self.geometry, self.limits, tilt_deg=tilt_deg,
                                    side=side, **self.common)

    def test_hover_trim(self):
        """At V = 0, tilt 90 deg with the CG under the spindle: level attitude, no disc tilt, n T (1 - f_dl) = W."""
        trim = solve_trim(self.model, self.stability, self.geometry, self.limits, velocity_m_s=0.0, tilt_deg=90.0,
                          **self.common)
        self.assertAlmostEqual(trim.pitch_deg, 0.0, places=4)
        self.assertAlmostEqual(trim.tilt_disc_deg, 0.0, places=4)
        self.assertAlmostEqual(2 * trim.thrust_per_rotor_N * (1 - trim.download_fraction),
                               6000.0 * acceleration_gravity_m_s2, delta=1.0)

    def test_hover_cg_offset_needs_disc_tilt(self):
        """CG 0.2 m aft of the spindle in hover, the tail powerless: the thrust must pass through the CG, so the disc
        tilts it forward by atan(0.2 / h) at a hub h = 1.6 m above the CG, and the fuselage pitches nose-up by the
        same angle to keep the thrust vertical."""
        aft = replace(self.geometry, x_cg_m=self.geometry.x_cg_m + 0.2)
        trim = solve_trim(self.model, self.stability, aft, self.limits, velocity_m_s=0.0, tilt_deg=90.0, **self.common)
        self.assertAlmostEqual(trim.tilt_disc_deg, -np.degrees(np.arctan(0.2 / 1.6)), places=3)
        self.assertAlmostEqual(trim.pitch_deg, -trim.tilt_disc_deg, places=3)

    def test_helicopter_mode_high_side_is_edgewise(self):
        """At 90 deg the high side is the edgewise advance-ratio limit: V cos(alpha) = mu_max Omega R."""
        high = self.bound(90.0, "high")
        self.assertIn("edgewise_advance_ratio", high.binding)
        tip_m_s = 220.0
        self.assertAlmostEqual(abs(high.trim.advance_ratio_edgewise), self.limits.advance_ratio_edgewise_max,
                               places=3)
        self.assertAlmostEqual(high.velocity_m_s * np.cosd(high.trim.pitch_deg),
                               self.limits.advance_ratio_edgewise_max * tip_m_s, delta=1.0)

    def test_airplane_mode_bounds(self):
        """At 0 deg disc tilt is washed out; the low side is the attitude (wing-borne) limit, the high side the placard."""
        low, high = self.bound(0.0, "low"), self.bound(0.0, "high")
        self.assertIn("pitch_max", low.binding)
        self.assertAlmostEqual(low.trim.tilt_disc_deg, 0.0, places=6)
        self.assertIn("placard_speed", high.binding)
        self.assertAlmostEqual(high.velocity_m_s, 120.0, places=2)

    def test_trends(self):
        """The corridor shifts to higher speed as the nacelles come down; a looser edgewise limit widens it."""
        self.assertGreater(self.bound(30.0, "low").velocity_m_s, self.bound(60.0, "low").velocity_m_s)
        self.assertGreater(self.bound(60.0, "high").velocity_m_s, self.bound(90.0, "high").velocity_m_s)
        looser = replace(self.limits, advance_ratio_edgewise_max=0.35)
        wider = solve_corridor_bound(self.model, self.stability, self.geometry, looser, tilt_deg=90.0, side="high",
                                     **self.common)
        self.assertGreater(wider.velocity_m_s, self.bound(90.0, "high").velocity_m_s)

    def test_tail_sign(self):
        """Wing-borne at the low side the trim needs nose-up tail (negative deflection, trailing edge up)."""
        self.assertLess(self.bound(0.0, "low").trim.deflection_tail_deg, 0.0)


if __name__ == "__main__":
    unittest.main()
