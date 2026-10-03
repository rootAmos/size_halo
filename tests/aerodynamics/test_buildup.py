"""Tier 21 (plan 025): AeroBuildup-based aerodynamics, blown wing and cross-checks."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.scholz import InterferenceFactors, ScholzAerodynamics
from aircraft_closure.aerodynamics.slipstream import BlownWing, RotorState
from examples.halo_sizing import HaloDesign, build_halo_aircraft
from examples.xv15_reference import Xv15Reference, build_xv15_aircraft


def scalar(x):
    return float(np.asarray(x).ravel()[0])


# The plan 022 reference design (780 kg, 14,436 lb), numeric.
halo_design = HaloDesign(
    x_le_wing_m=4.0354, area_wing_m2=22.5451, area_horizontal_tail_m2=3.8451, area_vertical_tail_m2=1.7021,
    torque_peak_motor_Nm=425.34, torque_peak_generator_Nm=461.28, power_rated_turboshaft_W=835183.7,
    mass_turboshaft_bare_kg=184.21, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
    area_disk_m2=69.976, mass_fuel_kg=989.6, solidity=0.06, speed_tip_m_s=238.2, speed_peak_motor_rad_s=1373.0,
    reduction_ratio=35.02, speed_peak_generator_rad_s=1405.0, count_parallel_battery=21.28)
halo = build_halo_aircraft(halo_design)
clean = BuildupAerodynamics(interference=InterferenceFactors(1.0, 1.0, 1.0, 1.0, 1.0), xtr_upper=1.0, xtr_lower=1.0)
altitude_m = 10000 * u.foot


class AeroBuildupIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = clean.evaluate_buildup(halo, 100.0, altitude_m, 4.0)
        airplane = clean.to_asb(halo)
        cls.operating_point = asb.OperatingPoint(atmosphere=asb.Atmosphere(altitude_m), velocity=100.0, alpha=4.0)
        cls.native = asb.AeroBuildup(airplane, cls.operating_point).run()
        cls.force_scale_N = scalar(cls.operating_point.dynamic_pressure()) * halo.wing.area_m2

    def test_lift_and_profile_drag_are_aerobuildups(self):
        """With Q = 1, free transition and nothing added, the build-up is AeroBuildup's own."""
        self.assertAlmostEqual(scalar(self.result.aero.cl), scalar(self.native["CL"]), places=10)
        self.assertAlmostEqual(scalar(self.result.aero.cd0), scalar(self.native["D_profile"]) / self.force_scale_N,
                               places=10)

    def test_induced_drag_is_aerobuildups_with_the_fuselage_factor(self):
        r = self.result
        aspect_ratio = halo.wing.aspect_ratio
        cdi_native = scalar(self.native["D_induced"]) / self.force_scale_N
        self.assertAlmostEqual(cdi_native, scalar(r.aero.cl**2 / (np.pi * aspect_ratio * r.oswald_span)), places=8)
        k_e_fuselage = 1 - 2 * (halo.fuselage.diameter_m / halo.wing.span_m())**2      # M < 0.3: k_e,M = 1
        self.assertAlmostEqual(scalar(r.aero.oswald_efficiency), scalar(r.oswald_span) * k_e_fuselage, places=10)
        self.assertAlmostEqual(scalar(r.aero.cd - r.aero.cd0), scalar(r.aero.cdi), places=12)

    def test_nacelles_are_aerosandbox_bodies(self):
        self.assertEqual(len(halo.to_asb().fuselages), 3)
        self.assertEqual([i.label for i in self.result.breakdown][:5],
                         ["wing", "horizontal_tail", "vertical_tail", "fuselage", "nacelles"])
        bare = replace(halo, nacelles=replace(halo.nacelles, length_m=None))
        self.assertEqual(len(bare.to_asb().fuselages), 1)
        self.assertLess(scalar(clean.evaluate(bare, 100.0, altitude_m, 4.0).cd0), scalar(self.result.aero.cd0))


class AddedItemTests(unittest.TestCase):
    def test_interference_scales_component_profile_drag(self):
        base = clean.evaluate_buildup(halo, 100.0, altitude_m, 2.0)
        q = BuildupAerodynamics(interference=InterferenceFactors(1.2, 1.0, 1.0, 1.0, 1.5), xtr_upper=1.0,
                                xtr_lower=1.0).evaluate_buildup(halo, 100.0, altitude_m, 2.0)
        items = {i.label: scalar(i.cd0) for i in base.breakdown}
        self.assertAlmostEqual(scalar(q.aero.cd0 - base.aero.cd0), 0.2 * items["wing"] + 0.5 * items["nacelles"],
                               places=10)

    def test_earlier_transition_raises_drag(self):
        free = scalar(clean.evaluate(halo, 100.0, altitude_m, 2.0).cd0)
        tripped = scalar(replace(clean, xtr_upper=0.0, xtr_lower=0.0).evaluate(halo, 100.0, altitude_m, 2.0).cd0)
        self.assertGreater(tripped, free)

    def test_misc_and_fixed_gear(self):
        aero = replace(clean, drag_area_misc_m2=0.3)
        delta = scalar(aero.evaluate(halo, 100.0, altitude_m, 2.0).cd0 - clean.evaluate(halo, 100.0, altitude_m, 2.0).cd0)
        self.assertAlmostEqual(delta, 0.3 / halo.wing.area_m2, places=10)
        fixed = replace(halo, landing_gear=replace(halo.landing_gear, is_retractable=False))
        delta = scalar(clean.evaluate(fixed, 100.0, altitude_m, 2.0).cd0 - clean.evaluate(halo, 100.0, altitude_m, 2.0).cd0)
        self.assertAlmostEqual(delta, clean.drag_area_landing_gear_fixed_m2 / halo.wing.area_m2, places=10)

    def test_stall_bound_is_the_lift_coefficient_bound(self):
        aero = BuildupAerodynamics()
        for alpha_deg in (2.0, 8.0, 14.0):
            point = aero.evaluate(halo, 60.0, 0.0, alpha_deg)
            alpha_stall_deg = scalar(aero.alpha_stall_deg(halo, 60.0, 0.0, aero=point))
            self.assertEqual(alpha_deg <= alpha_stall_deg, scalar(point.cl) <= aero.cl_max)
        # The standalone two-point linearization: the lift curve bends below linear, so CL there is a little short.
        alpha_stall_deg = scalar(aero.alpha_stall_deg(halo, 60.0, 0.0))
        self.assertAlmostEqual(scalar(aero.evaluate(halo, 60.0, 0.0, alpha_stall_deg).cl), aero.cl_max, delta=0.1)

    def test_lift_and_drag_rise_with_alpha_and_trends(self):
        aero = BuildupAerodynamics()
        polar = [aero.evaluate(halo, 100.0, altitude_m, a) for a in (0.0, 4.0, 8.0)]
        self.assertTrue(all(scalar(b.cl) > scalar(a.cl) for a, b in zip(polar, polar[1:])))
        self.assertTrue(all(scalar(b.cd) > scalar(a.cd) for a, b in zip(polar, polar[1:])))
        self.assertGreater(scalar(polar[0].cl), 0.0)         # cambered NACA 2423

    def test_vectorized_operating_points(self):
        aero = BuildupAerodynamics()
        result = aero.evaluate(halo, np.array([60.0, 100.0]), altitude_m, np.array([4.0, 4.0]))
        self.assertEqual(np.asarray(result.cl).size, 2)
        single = aero.evaluate(halo, 100.0, altitude_m, 4.0)
        self.assertAlmostEqual(float(np.asarray(result.cd)[1]), scalar(single.cd), places=8)


class BlownWingIntegrationTests(unittest.TestCase):
    aero = BuildupAerodynamics(blown_wing=BlownWing())

    def test_zero_thrust_is_the_unblown_polar(self):
        unblown = self.aero.evaluate(halo, 100.0, altitude_m, 4.0)
        blown = self.aero.evaluate(halo, 100.0, altitude_m, 4.0, rotor_state=RotorState(0.0, 30.0))
        self.assertAlmostEqual(scalar(blown.cl), scalar(unblown.cl), places=12)
        self.assertAlmostEqual(scalar(blown.cd), scalar(unblown.cd), places=12)

    def test_thrust_raises_lift_and_lowers_trimmed_drag(self):
        low, high = (self.aero.evaluate_buildup(halo, 100.0, altitude_m, 4.0, rotor_state=RotorState(t, 30.0))
                     for t in (2000.0, 8000.0))
        self.assertGreater(scalar(high.aero.cl), scalar(low.aero.cl))
        self.assertGreater(scalar(high.blown.ratio_dynamic_pressure), 1.0)
        self.assertLess(scalar(high.blown.delta_cd_swirl), scalar(low.blown.delta_cd_swirl))

    def test_symbolic_cruise_trim_with_thrust_coupling(self):
        """Wing area, angle of attack and rotor thrust as Opti variables; thrust = drag is an equality."""
        opti = asb.Opti()
        area_wing_m2 = opti.variable(init_guess=22.0, lower_bound=12.0, upper_bound=40.0)
        alpha_deg = opti.variable(init_guess=4.0, lower_bound=-4.0, upper_bound=12.0)
        thrust_per_rotor_N = opti.variable(init_guess=5000.0, scale=5000.0, lower_bound=0.0)
        aircraft = build_halo_aircraft(replace(halo_design, area_wing_m2=area_wing_m2))
        result = self.aero.evaluate(aircraft, 100.0, altitude_m, alpha_deg,
                                    rotor_state=RotorState(thrust_per_rotor_N, 30.0))
        weight_N = 6500 * 9.80665
        opti.subject_to([result.lift_N / weight_N == 1, (2 * thrust_per_rotor_N - result.drag_N) / weight_N == 0,
                         alpha_deg <= self.aero.alpha_stall_deg(aircraft, 100.0, altitude_m, aero=result)])
        opti.minimize(thrust_per_rotor_N / 1000)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution(2 * thrust_per_rotor_N), solution(result.drag_N), delta=1e-3)
        self.assertAlmostEqual(solution(result.lift_N), weight_N, delta=1e-2)
        self.assertGreater(solution(thrust_per_rotor_N), 0.0)


class CrossCheckTests(unittest.TestCase):
    def test_aerobuildup_and_scholz_parasite_drag_agree(self):
        """Independent correlations on the Halo reference: CD0 within 15 % at a low-lift cruise point."""
        buildup = BuildupAerodynamics(drag_area_misc_m2=3.0 * u.foot**2)
        scholz = ScholzAerodynamics(drag_area_misc_m2=3.0 * u.foot**2)
        ratio = scalar(buildup.evaluate(halo, 108.0, altitude_m, 2.0).cd0
                       / scholz.evaluate(halo, 108.0, altitude_m, 2.0).cd0)
        self.assertAlmostEqual(ratio, 1.0, delta=0.15)

    def test_xv15_parasite_drag_area_against_ndarc(self):
        """NDARC XV-15 cruise D/q 9.25 ft2 (Johnson 2010, Table 1): fuselage 1.56, pylons 2 x 0.76, tails 0.99,
        wing 2.18, i.e. 6.25 ft2 for these components plus 3.00 ft2 fittings and fixtures (taken as the
        miscellaneous area here). Both build-ups within 25 % on the components."""
        r = Xv15Reference()
        xv15 = build_xv15_aircraft(r, mass_engine_kg=300.0)
        xv15 = replace(xv15, nacelles=replace(xv15.nacelles, length_m=9 * u.foot, diameter_m=3.3 * u.foot,
                                              y_m=xv15.wing.span_m() / 2))
        tails = InterferenceFactors(horizontal_tail=1.08, vertical_tail=1.08)    # H-tail, Table 13.4
        for model in (BuildupAerodynamics(interference=tails), ScholzAerodynamics(interference=tails)):
            result = model.evaluate(xv15, r.velocity_cruise_m_s, r.altitude_cruise_m, 0.0)
            drag_area_ft2 = scalar(result.cd0) * xv15.wing.area_m2 / u.foot**2
            self.assertAlmostEqual(drag_area_ft2 / 6.25, 1.0, delta=0.25, msg=type(model).__name__)


if __name__ == "__main__":
    unittest.main()
