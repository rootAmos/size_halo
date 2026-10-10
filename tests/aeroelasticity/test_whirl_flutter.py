"""Proprotor whirl flutter (reduced order): identities, limits, classical whirl, trends, symmetry, symbols."""
import math
import unittest
from dataclasses import replace

import casadi as cas
import numpy as onp

from aircraft_closure.aeroelasticity.whirl_flutter import (AirframeDynamics, GimballedProprotor,
                                                           cantilever_wing_nacelle_airframe, flutter_speed,
                                                           whirl_flutter_matrices, whirl_modes)

speed_rotor_rad_s = 48.0
# XV-15-like gimballed rotor: 12.5 ft, 3 blades, 14 in chord, Lock number about 3.8, delta-3 -15 deg.
rotor_gimbal = GimballedProprotor(radius_m=3.81, count_blades=3, chord_m=0.356, inertia_flap_blade_kg_m2=139.0,
                                  stiffness_gimbal_Nm_rad=17000.0, pitch_flap_coupling=math.tan(math.radians(-15)))
rotor_stiff = replace(rotor_gimbal, stiffness_gimbal_Nm_rad=1e8, pitch_flap_coupling=0.0)


def pylon(inertia_kg_m2, frequency_rad_s, length_m, damping_ratio=0.0, mirror=False):
    """Rigid pylon on pitch (rotation y) and yaw (rotation z) springs, pivot `length_m` behind the hub."""
    k = inertia_kg_m2 * frequency_rad_s**2
    c = 2 * damping_ratio * math.sqrt(k * inertia_kg_m2)
    sign = -1.0 if mirror else 1.0                     # mirror across the x-z plane: y, rotation x and z flip
    return AirframeDynamics([[inertia_kg_m2, 0.0], [0.0, inertia_kg_m2]], [[c, 0.0], [0.0, c]], [[k, 0.0], [0.0, k]],
                            [[0.0, 0.0], [0.0, sign * length_m], [-length_m, 0.0]],
                            [[0.0, 0.0], [1.0, 0.0], [0.0, sign * 1.0]], ("pitch", "yaw"))


def whirl_direction(airframe, rotor, velocity_m_s):
    """(damping ratio, 'forward'/'backward') of each oscillatory pylon mode below 3/rev."""
    mass, damping, stiffness = (onp.asarray(m, float) for m in whirl_flutter_matrices(
        airframe, rotor, velocity_m_s, speed_rotor_rad_s, 1.225))
    n = mass.shape[0]
    inverse = onp.linalg.inv(mass)
    values, vectors = onp.linalg.eig(onp.block([[onp.zeros((n, n)), onp.eye(n)],
                                                [-inverse @ stiffness, -inverse @ damping]]))
    out = []
    for value, vector in zip(values, vectors.T):
        tilt_x, tilt_y = -vector[1], vector[0]         # rotor-axis tilts from yaw and pitch (sense +1)
        if value.imag <= 0 or value.imag > 3 * speed_rotor_rad_s or abs(tilt_x) < 1e-9:
            continue
        # Forward whirl (with the rotation, about +z_r): tilt_y / tilt_x = -i.
        out.append((-value.real / abs(value), "forward" if (tilt_y / tilt_x).imag < 0 else "backward"))
    return out


class TestLimits(unittest.TestCase):
    def test_gimbal_modes_at_nu_plus_minus_one_in_vacuum(self):
        # Rigid support, no air: the gimbal modes sit at (nu - 1) and (nu + 1) per rev in the fixed frame.
        rigid = pylon(1.0, 1e4, 1.0)
        modes = whirl_modes(rigid, rotor_gimbal, 50.0, speed_rotor_rad_s, 1e-9)
        nu = float(rotor_gimbal.frequency_flap_per_rev(speed_rotor_rad_s))
        gimbal = sorted(f / speed_rotor_rad_s for f, label in zip(modes.frequency_rad_s, modes.labels)
                        if label == "gimbal" and f < 3 * speed_rotor_rad_s)
        self.assertAlmostEqual(gimbal[0], nu - 1, places=4)
        self.assertAlmostEqual(gimbal[-1], nu + 1, places=4)

    def test_cantilever_tip_mass_frequencies(self):
        # Massless wing, point nacelle on the elastic axis: sqrt(3 EI / (L^3 m)) and sqrt(GJ / (L I_pitch)).
        airframe = cantilever_wing_nacelle_airframe(4.0, 2e7, 5e7, 6e6, 0.0, 0.0, 1000.0, 0.0, (0.0, 400.0, 0.0), 1.0,
                                                    damping_structural=(0.0, 0.0, 0.0))
        k = onp.diag(onp.asarray(airframe.stiffness, float))
        m = onp.diag(onp.asarray(airframe.mass, float))
        self.assertAlmostEqual(k[0] / m[0], 3 * 2e7 / (4.0**3 * 1000.0))
        self.assertAlmostEqual(k[2] / m[2], 6e6 / (4.0 * 400.0))

    def test_steady_thrust_reproduced(self):
        from aircraft_closure.aeroelasticity.whirl_flutter import rotor_coefficients
        co = rotor_coefficients(rotor_gimbal, 120.0, speed_rotor_rad_s, 1.225, 8000.0)
        force_scale_N = 0.5 * 1.225 * 0.356 * speed_rotor_rad_s**2 * 3.81**3
        self.assertAlmostEqual(3 * force_scale_N * float(co.thrust_steady), 8000.0, places=6)


class TestClassicalWhirl(unittest.TestCase):
    def test_stiff_propeller_backward_whirl_goes_unstable(self):
        # Reed / Houbolt: aerodynamics destabilize the backward whirl mode of a stiff propeller on a soft pylon.
        modes = whirl_direction(pylon(1300.0, 0.6 * speed_rotor_rad_s, 1.3), rotor_stiff, 150.0)
        unstable = [d for z, d in modes if z < 0]
        self.assertEqual(unstable, ["backward"])
        self.assertTrue(all(z > 0 for z, d in modes if d == "forward"))

    def test_flutter_speed_rises_with_pylon_stiffness(self):
        velocities = onp.arange(20.0, 400.0, 5.0)
        speeds = [flutter_speed(pylon(1300.0, f * speed_rotor_rad_s, 1.3, 0.01), rotor_stiff, speed_rotor_rad_s,
                                1.225, velocities)[0] for f in (0.3, 0.6, 1.0)]
        self.assertTrue(all(v is not None for v in speeds))
        self.assertLess(speeds[0], speeds[1])
        self.assertLess(speeds[1], speeds[2])

    def test_gimbal_more_stable_than_stiff_propeller(self):
        airframe = pylon(1300.0, 0.6 * speed_rotor_rad_s, 1.3, 0.01)
        velocities = onp.arange(20.0, 400.0, 5.0)
        stiff = flutter_speed(airframe, rotor_stiff, speed_rotor_rad_s, 1.225, velocities)[0]
        gimbal = flutter_speed(airframe, replace(rotor_gimbal, pitch_flap_coupling=0.0), speed_rotor_rad_s, 1.225,
                               velocities)[0]
        self.assertGreater(gimbal if gimbal is not None else onp.inf, stiff)


class TestSymmetry(unittest.TestCase):
    def test_mirror_image_with_opposite_rotation_has_same_eigenvalues(self):
        rotor = replace(rotor_gimbal, frequency_lag_per_rev=1.6, frequency_coning_per_rev=1.5, drive="free", inertia_drive_kg_m2=200.0)
        airframe = cantilever_wing_nacelle_airframe(4.2, 1.06e7, 3.2e7, 8.0e6, 40.0, 8.0, 970.0, 0.4,
                                                    (300.0, 900.0, 900.0), 1.2, z_cg_nacelle_m=0.2)
        mirrored = AirframeDynamics(airframe.mass, airframe.damping, airframe.stiffness,
                                    [airframe.displacement_hub[0], [-v for v in airframe.displacement_hub[1]],
                                     airframe.displacement_hub[2]],
                                    [[-v for v in airframe.rotation_hub[0]], airframe.rotation_hub[1],
                                     [-v for v in airframe.rotation_hub[2]]], airframe.labels)
        a = whirl_modes(airframe, rotor, 140.0, speed_rotor_rad_s, 1.225, 4000.0)
        b = whirl_modes(mirrored, replace(rotor, rotation_sense=-1), 140.0, speed_rotor_rad_s, 1.225, 4000.0)
        self.assertTrue(onp.allclose(sorted(a.eigenvalues, key=lambda z: (z.imag, z.real)),
                                     sorted(b.eigenvalues, key=lambda z: (z.imag, z.real)), rtol=1e-8, atol=1e-8))


class TestSymbolic(unittest.TestCase):
    def test_matrices_stay_symbolic_in_wing_stiffness_and_rotor_speed(self):
        stiffness_torsion, speed = cas.MX.sym("stiffness_torsion"), cas.MX.sym("speed")
        airframe = cantilever_wing_nacelle_airframe(4.2, 1.06e7, 3.2e7, stiffness_torsion, 40.0, 8.0, 970.0, 0.4,
                                                    (300.0, 900.0, 900.0), 1.2)
        rotor = replace(rotor_gimbal, frequency_lag_per_rev=1.6, frequency_coning_per_rev=1.5, drive="free", inertia_drive_kg_m2=200.0)
        mass, damping, stiffness = whirl_flutter_matrices(airframe, rotor, 140.0, speed, 1.225, 4000.0)
        self.assertEqual(mass.shape, (10, 10))               # numeric here: no symbol enters the inertia
        for matrix in (damping, stiffness):
            self.assertIsInstance(matrix, cas.MX)
            self.assertEqual(matrix.shape, (10, 10))
        evaluated = cas.Function("k", [stiffness_torsion, speed], [stiffness])(8e6, 48.0)
        numeric = whirl_flutter_matrices(
            cantilever_wing_nacelle_airframe(4.2, 1.06e7, 3.2e7, 8e6, 40.0, 8.0, 970.0, 0.4, (300.0, 900.0, 900.0),
                                             1.2), rotor, 140.0, 48.0, 1.225, 4000.0)[2]
        self.assertTrue(onp.allclose(onp.asarray(evaluated), onp.asarray(numeric, float)))


class TestXv15AgainstCamrad(unittest.TestCase):
    """NASA/TP-2004-212262 stick-model hub modes and rotor deck (examples/whirl_flutter_xv15.py)."""

    def test_thick_to_thin_wing_drop_and_calibrated_boundary(self):
        from examples import whirl_flutter_xv15 as xv15
        knockdown, thick_kt, thin_kt, thin_calibrated_kt = xv15.calibration()
        self.assertAlmostEqual(thick_kt - thin_kt, 60.0, delta=10.0)          # CAMRAD: 335 - 275 kt
        self.assertAlmostEqual(thin_calibrated_kt, 275.0, delta=0.06 * 275.0)  # one knockdown, set on the thick wing
        self.assertGreater(knockdown, 0.75)                                   # model is non-conservative, not wild
        self.assertLess(knockdown, 0.95)


if __name__ == "__main__":
    unittest.main()
