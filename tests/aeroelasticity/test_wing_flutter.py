"""Plan 056: classic wing flutter, beam finite elements with Theodorsen strip theory (Jones' approximation)."""
import unittest
from dataclasses import replace

import numpy as onp

from aircraft_closure.aeroelasticity import wing_flutter as wf

velocities_m_s = onp.arange(50, 400, 2.0)


class TestStructure(unittest.TestCase):
    def test_uncoupled_frequencies_match_beam_theory(self):
        wing = wf.goland_wing(offset_cg_m=0.0)
        omega = wf.natural_frequencies_rad_s(wing, count_elements=16)
        bending = 1.875104**2 * onp.sqrt(wing.stiffness_beam_Nm2 / (wing.mass_kg_m * wing.length_m**4))
        torsion = onp.pi / 2 * onp.sqrt(wing.stiffness_torsion_Nm2 / (wing.inertia_torsion_kg_m * wing.length_m**2))
        self.assertAlmostEqual(omega[0] / bending, 1.0, delta=0.002)
        self.assertAlmostEqual(omega[1] / torsion, 1.0, delta=0.005)

    def test_tip_mass_lowers_bending_frequency(self):
        wing = wf.goland_wing()
        tip = wf.TipBody(100.0)
        self.assertLess(wf.natural_frequencies_rad_s(wing, tip)[0], wf.natural_frequencies_rad_s(wing)[0])


class TestGolandWing(unittest.TestCase):
    def test_flutter_speed_and_frequency(self):
        # Goland (1945), corrected: 137.2 m/s and 70.68 rad/s at 1.225 kg/m^3 with strip theory (Cal 1992, table 2.5).
        speed, frequency, _ = wf.flutter_speed(wf.goland_wing(), None, 1.225, velocities_m_s, count_elements=16)
        self.assertAlmostEqual(speed / 137.2, 1.0, delta=0.01)
        self.assertAlmostEqual(frequency / 70.68, 1.0, delta=0.03)

    def test_mesh_convergence(self):
        coarse = wf.flutter_speed(wf.goland_wing(), None, 1.225, velocities_m_s, count_elements=4)[0]
        fine = wf.flutter_speed(wf.goland_wing(), None, 1.225, velocities_m_s, count_elements=16)[0]
        self.assertLess(abs(coarse / fine - 1), 0.02)

    def test_finite_span_lift_slope_raises_flutter_speed(self):
        two_d = wf.flutter_speed(wf.goland_wing(), None, 1.225, velocities_m_s, count_elements=8)[0]
        finite = wf.flutter_speed(wf.goland_wing(), None, 1.225, velocities_m_s, count_elements=8,
                                  factor_lift_slope=float(wf.lift_slope_factor_helmbold(6.67)))[0]
        self.assertGreater(finite, two_d)


class TestConcentratedMass(unittest.TestCase):
    def test_mass_ahead_of_elastic_axis_raises_flutter_speed(self):
        # Runyan and Watkins (NACA Report 966): masses ahead of the elastic axis tend to raise the flutter speed.
        wing = wf.goland_wing()
        mass_kg = 0.92 * wing.mass_kg_m * wing.length_m
        speeds = [wf.flutter_speed(wing, wf.TipBody(mass_kg, d, inertia_pitch_kg_m2=mass_kg * 0.01), 1.225,
                                   onp.arange(20, 400, 2.0), count_elements=8)[0] for d in (0.3, 0.0, -0.3)]
        self.assertGreater(speeds[0], speeds[1])
        self.assertGreater(speeds[1], speeds[2])

    def test_zero_airspeed_has_no_aerodynamic_stiffness_or_damping(self):
        _, C_a, K_a, D_a, *_ = wf.aerodynamic_matrices(wf.goland_wing(), 4, 0.0, 1.225)
        self.assertEqual(float(onp.abs(C_a).max() + onp.abs(K_a).max() + onp.abs(D_a).max()), 0.0)


if __name__ == "__main__":
    unittest.main()
