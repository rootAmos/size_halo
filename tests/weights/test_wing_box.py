"""Bottom-up tiltrotor wing box (plan 054): closed forms, limiting cases, trends, the XV-15 with no calibration
factor, and CasADi compatibility."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import numpy as onp

from aircraft_closure.weights import wing_box as wb

lb, ft, inch = 0.45359237, 0.3048, 0.0254
stiffness_lb_in2 = 4.4482216152605 * inch**2


def xv15(**changes):
    """XV-15 wing from its published geometry, tip masses and stiffness (data/weights/tiltrotor_wing_reference.csv)."""
    kwargs = dict(span_m=32.17 * ft, chord_m=169.0 / 32.17 * ft, thickness_to_chord=0.23, mass_design_kg=13000 * lb,
                  mass_tip_kg=(3166 + 1118) / 2 * lb, radius_gyration_pylon_m=1.0,
                  offset_cg_tip_m=47.3 * inch * 1118 / (3166 + 1118), speed_rotor_design_rad_s=48.0,
                  frequency_torsion_per_rev=1.087, frequency_beam_per_rev=0.432, frequency_chord_per_rev=0.825,
                  width_attachment_m=56 * inch, material=wb.aluminium_wing_box_material(),
                  stiffness_required_Nm2=(2.80e9 * stiffness_lb_in2, 3.70e9 * stiffness_lb_in2,
                                          1.12e10 * stiffness_lb_in2))
    kwargs.update(changes)
    return wb.size_wing_box(**kwargs)


class TestEllipticLoads(unittest.TestCase):
    def test_closed_forms_match_quadrature(self):
        eta = onp.linspace(0, 1, 20001)
        load = 4 / onp.pi * onp.sqrt(1 - eta**2)           # unit lift on a unit semispan
        for e in (0.0, 0.2, 0.5, 0.8):
            outboard = eta >= e
            shear = onp.trapezoid(load[outboard], eta[outboard])
            moment = onp.trapezoid(load[outboard] * (eta[outboard] - e), eta[outboard])
            self.assertAlmostEqual(float(wb.shear_elliptic_fraction(e)), shear, places=4)
            self.assertAlmostEqual(float(wb.moment_elliptic_fraction(e)), moment, places=4)

    def test_root_moment_is_lift_at_centre_of_pressure(self):
        self.assertAlmostEqual(float(wb.moment_elliptic_fraction(0.0)), 4 / (3 * onp.pi), places=12)
        self.assertAlmostEqual(float(wb.shear_elliptic_fraction(0.0)), 1.0, places=12)


class TestWingBox(unittest.TestCase):
    def test_xv15_without_calibration(self):
        # Published breakdown 946 lb (Acree et al. 1999); the AFDD wing needs x 1.33 for it.
        error = xv15().total() / (946 * lb) - 1
        self.assertLess(abs(error), 0.15)           # +8 % (plan 054)

    def test_stiffness_floor_is_met(self):
        sizing = xv15()
        self.assertGreaterEqual(sizing.stiffness_torsion_Nm2, 0.99 * 2.80e9 * stiffness_lb_in2)
        self.assertGreaterEqual(sizing.stiffness_beam_Nm2, 0.99 * 3.70e9 * stiffness_lb_in2)
        self.assertGreaterEqual(sizing.stiffness_chord_Nm2, 0.99 * 1.12e10 * stiffness_lb_in2)

    def test_root_strain_within_allowable(self):
        sizing = xv15()
        self.assertLessEqual(sizing.strain_root_lower_cover, wb.aluminium_wing_box_material().strain_tension)

    def test_skin_at_least_minimum_gauge_and_tapers_outboard(self):
        sizing = xv15(stiffness_required_Nm2=(1.0, 1.0, 1.0))
        t_min = wb.aluminium_wing_box_material().thickness_min_m
        self.assertTrue(all(t >= t_min * 0.999 for t in sizing.thickness_skin_upper_m))
        covers = [s + st for s, st in zip(sizing.thickness_skin_upper_m, sizing.thickness_stringers_upper_m)]
        self.assertGreater(covers[0], covers[-1])

    def test_no_torque_without_tip_offset(self):
        with_offset = xv15(stiffness_required_Nm2=(1.0, 1.0, 1.0))
        without = xv15(stiffness_required_Nm2=(1.0, 1.0, 1.0), offset_cg_tip_m=0.0)
        self.assertLessEqual(without.mass_webs_kg, with_offset.mass_webs_kg)

    def test_longer_rib_pitch_needs_thicker_upper_cover(self):
        short = xv15(layout=replace(wb.WingBoxLayout(), pitch_rib_m=0.3), stiffness_required_Nm2=(1.0, 1.0, 1.0))
        long = xv15(layout=replace(wb.WingBoxLayout(), pitch_rib_m=1.5), stiffness_required_Nm2=(1.0, 1.0, 1.0))
        cover = lambda s: s.thickness_skin_upper_m[0] + s.thickness_stringers_upper_m[0]
        self.assertGreaterEqual(cover(long), cover(short))
        self.assertLess(long.mass_ribs_kg, short.mass_ribs_kg)

    def test_sandwich_webs_meet_minimum_and_beat_solid_plate(self):
        composite = dict(material=wb.graphite_epoxy_wing_box_material(), stiffness_required_Nm2=(1.0, 1.0, 1.0))
        sandwich = xv15(**composite)
        plate = xv15(layout=replace(wb.WingBoxLayout(), construction_web="plate"), **composite)
        self.assertGreaterEqual(min(sandwich.thickness_web_m), 1.5e-3 - 1e-9)
        self.assertGreaterEqual(min(sandwich.thickness_core_web_m), 6.0e-3 - 1e-9)
        self.assertEqual(max(plate.thickness_core_web_m), 0.0)
        self.assertLess(sandwich.mass_webs_kg, plate.mass_webs_kg)

    def test_hot_wet_knockdown_thickens_buckling_limited_upper_cover(self):
        dry = replace(wb.graphite_epoxy_wing_box_material(), knockdown_modulus_environment=1.0,
                      knockdown_shear_environment=1.0)
        wet = xv15(material=wb.graphite_epoxy_wing_box_material(), stiffness_required_Nm2=(1.0, 1.0, 1.0))
        self.assertGreater(wet.mass_covers_kg, xv15(material=dry, stiffness_required_Nm2=(1.0, 1.0, 1.0)).mass_covers_kg)

    def test_thicker_section_lightens_covers(self):
        thin = xv15(thickness_to_chord=0.15, stiffness_required_Nm2=(1.0, 1.0, 1.0))
        thick = xv15(thickness_to_chord=0.23, stiffness_required_Nm2=(1.0, 1.0, 1.0))
        self.assertLess(thick.mass_covers_kg, thin.mass_covers_kg)

    def test_frequency_placement_matches_stiffness(self):
        sizing = xv15(stiffness_required_Nm2=None)
        bending, torsion = wb.wing_tiltrotor_mode_inertias(32.17 * ft, (3166 + 1118) / 2 * lb, 1.0, 13000 * lb)
        correction = wb.WingBoxLayout().correction_torsion_frequency
        self.assertAlmostEqual(sizing.frequency_torsion_rad_s,
                               correction * onp.sqrt(sizing.stiffness_torsion_Nm2 / torsion))
        self.assertGreaterEqual(sizing.frequency_torsion_rad_s, 0.99 * 1.087 * 48.0)
        self.assertGreaterEqual(sizing.frequency_beam_rad_s, 0.99 * 0.432 * 48.0)

    def test_layout_outputs_are_physical(self):
        sizing = xv15()
        self.assertEqual(len(sizing.y_stations_m), wb.WingBoxLayout().count_stations)
        self.assertTrue(all(p > 0 for p in sizing.pitch_stringer_upper_m))
        self.assertGreater(sizing.thickness_rib_m, 0)
        self.assertLess(sizing.y_stations_m[-1], 32.17 * ft / 2)


class TestSymbolic(unittest.TestCase):
    def test_casadi_matches_numeric(self):
        opti = asb.Opti()
        mass_tip_kg = opti.variable(init_guess=900.0)
        opti.subject_to(mass_tip_kg == (3166 + 1118) / 2 * lb)
        sizing = xv15(mass_tip_kg=mass_tip_kg, smoothing=1e-3)
        opti.minimize(sizing.total())
        solution = opti.solve(verbose=False)
        numeric = xv15(smoothing=1e-3).total()
        self.assertAlmostEqual(float(solution(sizing.total())), float(numeric), delta=1e-6 * float(numeric))


class TestSectionShape(unittest.TestCase):
    """Plan 062: box depths from the section's own thickness shape."""

    def test_naca2423_shape_reproduces_the_4_digit_formula(self):
        from functools import partial

        from aircraft_closure.aerodynamics.wing_section import section_thickness_shape
        formula, shaped = xv15(), xv15(thickness_shape=partial(section_thickness_shape, "naca2423"))
        self.assertAlmostEqual(float(shaped.depth_box_m), float(formula.depth_box_m), delta=0.01 * float(formula.depth_box_m))
        self.assertAlmostEqual(float(shaped.total()), float(formula.total()), delta=0.01 * float(formula.total()))

    def test_64a223_box_is_shallower_at_the_front_spar(self):
        """The 6A section carries its thickness aft (40 % chord): thinner at the 15 % front spar, so the box between
        the spars is shallower than the 2423's at the same thickness ratio."""
        from functools import partial

        from aircraft_closure.aerodynamics.wing_section import section_thickness_shape
        shape = partial(section_thickness_shape, "naca64a223")
        self.assertLess(shape(0.15), section_thickness_shape("naca2423", 0.15))
        self.assertAlmostEqual(shape(0.40), 1.0, delta=0.01)
        self.assertLess(float(xv15(thickness_shape=shape).depth_box_m), float(xv15().depth_box_m))

    def test_shape_is_smooth_in_a_symbolic_thickness(self):
        from functools import partial

        from aircraft_closure.aerodynamics.wing_section import section_thickness_shape
        shape = partial(section_thickness_shape, "naca64a223")
        opti = asb.Opti()
        thickness = opti.variable(init_guess=0.2, lower_bound=0.15, upper_bound=0.28)
        sizing = xv15(thickness_to_chord=thickness, thickness_shape=shape, smoothing=1e-3)
        opti.minimize(sizing.total())
        solution = opti.solve(verbose=False)
        self.assertGreater(float(solution(thickness)), 0.15)


class TestBeamStiffnessFactor(unittest.TestCase):
    def test_factor_scales_reported_stiffness_and_adds_caps(self):
        """Plan 062: a beam factor below 1 reports factor x the box stiffness and sizes caps so the effective
        stiffness still meets the requirement (the frequency-placement case, no imposed stiffness)."""
        free = dict(stiffness_required_Nm2=None)
        base = xv15(**free)
        factored = xv15(layout=replace(wb.WingBoxLayout(), factor_stiffness_beam=0.886), **free)
        # Strength-sized here (the beam requirement does not set the caps): the effective frequency falls by
        # sqrt(0.886) and no caps are added.
        self.assertAlmostEqual(float(factored.frequency_beam_rad_s) / float(base.frequency_beam_rad_s), 0.886**0.5,
                               places=6)
        self.assertGreaterEqual(float(factored.mass_caps_kg), float(base.mass_caps_kg))
        # Stiffness-sized (an imposed beam requirement above the box): caps grow so the effective stiffness meets it.
        required = (1e7, 3.0 * float(base.stiffness_beam_Nm2), 1e7)
        stiff = xv15(stiffness_required_Nm2=required)
        stiff_factored = xv15(stiffness_required_Nm2=required, layout=replace(wb.WingBoxLayout(), factor_stiffness_beam=0.886))
        self.assertGreater(float(stiff_factored.mass_caps_kg), float(stiff.mass_caps_kg))
        self.assertGreaterEqual(float(stiff_factored.stiffness_beam_Nm2), 0.99 * required[1])


if __name__ == "__main__":
    unittest.main()
