"""Tier 21 (plan 025): blown-wing correction factors."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.slipstream import BlownWing, RotorState

density_kg_m3 = 0.905
args = dict(area_disk_m2=70.0, count_rotors=2, chord_wing_m=1.92, area_wing_m2=22.5, cl_wing=0.6,
            cd_profile_wing=0.009, cl_alpha_wing_per_rad=4.5)


def increment(thrust_N, model=BlownWing(), velocity_m_s=100.0, speed_rad_s=30.0):
    return model.evaluate(velocity_m_s, density_kg_m3, RotorState(thrust_N, speed_rad_s), **args)


class BlownWingTests(unittest.TestCase):
    def test_zero_thrust_gives_no_increment(self):
        b = increment(0.0)
        for value in (b.velocity_induced_m_s, b.angle_swirl_deg, b.delta_cl, b.delta_cd_profile, b.delta_cd_swirl):
            self.assertAlmostEqual(float(value), 0.0, places=12)
        self.assertAlmostEqual(float(b.ratio_dynamic_pressure), 1.0, places=12)

    def test_momentum_theory_identities(self):
        thrust_N = 8000.0
        b = increment(thrust_N)
        v_i, velocity_m_s = float(b.velocity_induced_m_s), 100.0
        self.assertAlmostEqual(2 * density_kg_m3 * 70.0 * v_i * (velocity_m_s + v_i), thrust_N, places=6)
        x = BlownWing().ratio_distance_disk_to_radius
        self.assertAlmostEqual(float(b.velocity_increment_wing_m_s), v_i * (1 + x / np.sqrt(1 + x**2)), places=10)
        far = increment(thrust_N, BlownWing(ratio_distance_disk_to_radius=1e6))
        self.assertAlmostEqual(float(far.velocity_increment_wing_m_s), 2 * v_i, places=6)   # fully developed
        self.assertAlmostEqual(float(b.ratio_dynamic_pressure),
                               ((velocity_m_s + float(b.velocity_increment_wing_m_s)) / velocity_m_s)**2, places=12)
        # Slipstream contracts: immersed span below the tip rotors' half disks.
        radius_m = np.sqrt(70.0 / np.pi)
        self.assertLess(float(b.area_immersed_m2), 2 * radius_m * 1.92)

    def test_increments_grow_with_thrust_and_fall_with_speed(self):
        low, high = increment(2000.0), increment(8000.0)
        self.assertGreater(float(high.delta_cl), float(low.delta_cl))
        self.assertGreater(float(high.delta_cd_profile), float(low.delta_cd_profile))
        self.assertLess(float(high.delta_cd_swirl), float(low.delta_cd_swirl))
        fast = increment(8000.0, velocity_m_s=140.0)
        self.assertLess(float(fast.ratio_dynamic_pressure), float(high.ratio_dynamic_pressure))

    def test_swirl_sign_convention(self):
        """Inboard-up (+1) raises the local angle on the immersed span: more lift, swirl recovered (negative
        drag); outboard-up (-1) the reverse."""
        up, down = increment(8000.0), increment(8000.0, BlownWing(sign_swirl=-1.0))
        self.assertGreater(float(up.angle_swirl_deg), 0.0)
        self.assertLess(float(down.angle_swirl_deg), 0.0)
        self.assertGreater(float(up.delta_cl), float(down.delta_cl))
        self.assertLess(float(up.delta_cd_swirl), 0.0)
        self.assertGreater(float(down.delta_cd_swirl), 0.0)
        no_swirl = increment(8000.0, BlownWing(factor_swirl=0.0))
        self.assertAlmostEqual(float(no_swirl.delta_cd_swirl), 0.0, places=12)
        # Dynamic pressure alone: delta_cl = S_b/S (q_s/q - 1) cl.
        self.assertAlmostEqual(float(no_swirl.delta_cl), float(no_swirl.area_immersed_m2 / 22.5
                                                                * (no_swirl.ratio_dynamic_pressure - 1) * 0.6),
                               places=12)

    def test_swirl_recovery_is_bounded_by_the_swirl_energy(self):
        b = increment(8000.0, BlownWing(efficiency_swirl_recovery=1.0))
        v_i = float(b.velocity_induced_m_s)
        swirl_m_s = 8000.0 / (density_kg_m3 * 70.0 * 30.0 * (2 / 3) * np.sqrt(70.0 / np.pi))
        power_swirl_W = 2 * 0.5 * density_kg_m3 * 70.0 * (100.0 + v_i) * swirl_m_s**2
        power_recovered_W = -float(b.delta_cd_swirl) * 0.5 * density_kg_m3 * 100.0**2 * 22.5 * 100.0
        self.assertAlmostEqual(power_recovered_W, power_swirl_W, delta=1e-6 * power_swirl_W)

    def test_symbolic(self):
        opti = asb.Opti()
        thrust_N = opti.variable(init_guess=1000.0, lower_bound=0.0)
        b = increment(thrust_N)
        opti.subject_to(b.delta_cl == 0.05)
        opti.minimize(thrust_N / 1000)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution(b.delta_cl)), 0.05, places=6)
        self.assertGreater(solution(thrust_N), 0.0)


if __name__ == "__main__":
    unittest.main()
