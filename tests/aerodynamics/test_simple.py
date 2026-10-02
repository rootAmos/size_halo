import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas
from aerosandbox.library.aerodynamics.inviscid import CL_over_Cl
from aerosandbox.library.aerodynamics.viscous import Cf_flat_plate

from aircraft_closure.aerodynamics.simple import DragIncrement, SimpleAerodynamics
from examples.aircraft_mass_closure import build_reference_aircraft

aircraft = build_reference_aircraft(3.2)
aero = SimpleAerodynamics()


class LiftTests(unittest.TestCase):
    def test_zero_lift_angle_and_slope(self):
        self.assertAlmostEqual(float(aero.evaluate(aircraft, 60, 1000, -4.0).cl), 0.0, places=12)
        a = aero.evaluate(aircraft, 60, 1000, 2.0)
        b = aero.evaluate(aircraft, 60, 1000, 3.0)
        self.assertAlmostEqual(float(b.cl - a.cl), float(a.cl_alpha_per_rad) * np.pi / 180, places=12)

    def test_lift_slope_is_aerosandbox_ratio_times_two_pi(self):
        mach = float(aero.evaluate(aircraft, 60, 1000, 0.0).mach)
        expected = 2 * np.pi * CL_over_Cl(aircraft.wing.aspect_ratio, mach=mach)
        self.assertAlmostEqual(float(aero.lift_curve_slope_per_rad(aircraft, 60, 1000)), float(expected), places=12)

    def test_slope_below_two_pi_and_rising_with_aspect_ratio(self):
        low = replace(aircraft, wing=replace(aircraft.wing, aspect_ratio=6.0))
        high = replace(aircraft, wing=replace(aircraft.wing, aspect_ratio=12.0))
        slope_low = float(aero.lift_curve_slope_per_rad(low, 60, 1000))
        slope_high = float(aero.lift_curve_slope_per_rad(high, 60, 1000))
        self.assertLess(slope_low, slope_high)
        self.assertLess(slope_high, 2 * np.pi)

    def test_stall_angle_reaches_cl_max(self):
        alpha_stall_deg = float(aero.alpha_stall_deg(aircraft, 60, 1000))
        self.assertAlmostEqual(float(aero.evaluate(aircraft, 60, 1000, alpha_stall_deg).cl), aero.cl_max, places=10)


class DragTests(unittest.TestCase):
    def test_cd0_is_breakdown_sum(self):
        breakdown = aero.parasite_drag_breakdown(aircraft, 60, 1000)
        self.assertEqual([item.label for item in breakdown],
                         ["wing", "horizontal_tail", "vertical_tail", "fuselage", "miscellaneous"])
        result = aero.evaluate(aircraft, 60, 1000, 3.0)
        self.assertAlmostEqual(float(result.cd0), sum(float(i.cd0) for i in breakdown), places=12)
        self.assertAlmostEqual(float(breakdown[-1].cd0), 0.25 / aircraft.wing.area_m2, places=12)

    def test_wing_parasite_uses_aerosandbox_skin_friction(self):
        atmosphere = asb.Atmosphere(altitude=1000)
        wing = aircraft.wing.to_asb()
        reynolds = atmosphere.density() * 60 * wing.mean_aerodynamic_chord() / atmosphere.dynamic_viscosity()
        mach = 60 / atmosphere.speed_of_sound()
        thickness = asb.Airfoil("naca4418").max_thickness()
        form_factor = (1 + 0.6 / 0.3 * thickness + 100 * thickness**4) * 1.34 * mach**0.18
        expected = Cf_flat_plate(reynolds) * form_factor * wing.area("wetted") / wing.area()
        self.assertAlmostEqual(float(aero.parasite_drag_breakdown(aircraft, 60, 1000)[0].cd0), float(expected), places=12)

    def test_drag_polar_identity(self):
        result = aero.evaluate(aircraft, 60, 1000, 5.0)
        expected_cdi = result.cl**2 / (np.pi * result.oswald_efficiency * aircraft.wing.aspect_ratio)
        self.assertAlmostEqual(float(result.cd - result.cd0), float(expected_cdi), places=12)
        self.assertAlmostEqual(float(result.lift_to_drag), float(result.cl / result.cd), places=12)

    def test_max_lift_to_drag_closed_form(self):
        result = aero.evaluate(aircraft, 60, 1000, 0.0)
        cd0, e, aspect_ratio = float(result.cd0), float(result.oswald_efficiency), aircraft.wing.aspect_ratio
        cl_star = np.sqrt(cd0 * np.pi * e * aspect_ratio)
        alpha_star_deg = aero.alpha_zero_lift_deg + np.degrees(cl_star / float(result.cl_alpha_per_rad))
        best = aero.evaluate(aircraft, 60, 1000, alpha_star_deg)
        self.assertAlmostEqual(float(best.lift_to_drag), 0.5 * np.sqrt(np.pi * e * aspect_ratio / cd0), places=9)
        for delta in (-0.5, 0.5):
            self.assertLess(float(aero.evaluate(aircraft, 60, 1000, alpha_star_deg + delta).lift_to_drag),
                            float(best.lift_to_drag))

    def test_increments_add(self):
        base = aero.evaluate(aircraft, 60, 1000, 3.0)
        more = aero.evaluate(aircraft, 60, 1000, 3.0, drag_increments=(DragIncrement("antenna", 0.002),))
        self.assertAlmostEqual(float(more.cd - base.cd), 0.002, places=12)

    def test_trends(self):
        fast = aero.evaluate(aircraft, 80, 1000, 3.0)
        slow = aero.evaluate(aircraft, 40, 1000, 3.0)
        self.assertLess(float(fast.cd0), float(slow.cd0))  # Reynolds number lowers skin friction
        dirty = replace(aero, drag_area_misc_m2=0.5).evaluate(aircraft, 60, 1000, 3.0)
        self.assertGreater(float(dirty.cd0), float(aero.evaluate(aircraft, 60, 1000, 3.0).cd0))


class SymbolicTests(unittest.TestCase):
    def test_symbolic_alpha_and_geometry(self):
        opti = asb.Opti()
        alpha_deg = opti.variable(init_guess=3)
        area_m2 = opti.variable(init_guess=12, lower_bound=1)
        symbolic = replace(aircraft, wing=replace(aircraft.wing, area_m2=area_m2))
        result = aero.evaluate(symbolic, 60, 1000, alpha_deg)
        self.assertIsInstance(result.lift_to_drag, cas.MX)
        opti.subject_to(area_m2 == 12)
        opti.minimize(-result.lift_to_drag)
        solution = opti.solve(verbose=False)
        numeric = aero.evaluate(aircraft, 60, 1000, 0.0)
        expected = 0.5 * np.sqrt(np.pi * float(numeric.oswald_efficiency) * 9 / float(numeric.cd0))
        self.assertAlmostEqual(float(solution.value(result.lift_to_drag)), expected, places=6)


if __name__ == "__main__":
    unittest.main()
