"""Tier 21 (plan 025): Scholz level-0 build-up, Korn wave drag, Nita-Scholz Oswald factor, hover download."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
from aerosandbox.library.aerodynamics.inviscid import oswalds_efficiency

from aircraft_closure.aerodynamics.download import HoverDownload, chord_projected_m, download_fraction
from aircraft_closure.aerodynamics.scholz import (ScholzAerodynamics, area_wetted_fuselage_m2, area_wetted_surface_m2,
                                                  form_factor_fuselage, form_factor_nacelle, form_factor_surface,
                                                  oswald_nita_scholz, skin_friction_turbulent)
from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from examples.aircraft_mass_closure import build_reference_aircraft

aircraft = build_reference_aircraft(3.2)


class ClosedFormTests(unittest.TestCase):
    def test_skin_friction_is_the_datcom_turbulent_law(self):
        for reynolds in (1e6, 1e7, 1e8):
            self.assertAlmostEqual(float(skin_friction_turbulent(reynolds, 0.0)),
                                   0.455 / np.log10(reynolds)**2.58, places=12)
        # Mach lowers the coefficient by (1 + 0.144 M^2)^0.65.
        ratio = float(skin_friction_turbulent(1e7, 0.5) / skin_friction_turbulent(1e7, 0.0))
        self.assertAlmostEqual(ratio, (1 + 0.144 * 0.25)**-0.65, places=12)

    def test_form_factors(self):
        self.assertAlmostEqual(float(form_factor_nacelle(4.0)), 1 + 0.35 / 4, places=12)
        self.assertAlmostEqual(float(form_factor_fuselage(8.0)), 1 + 60 / 512 + 8 / 400, places=12)
        self.assertAlmostEqual(float(form_factor_surface(0.12, 0.3, 0.2)),
                               (1 + 2 * 0.12 + 100 * 0.12**4) * 1.34 * 0.2**0.18, places=12)
        self.assertGreater(float(form_factor_nacelle(2.0)), float(form_factor_nacelle(4.0)))   # blunter: more drag

    def test_torenbeek_wetted_areas(self):
        # Long fuselage tends to the cylinder pi d l; the wing to twice the exposed area plus thickness.
        self.assertAlmostEqual(float(area_wetted_fuselage_m2(1000.0, 1.0)) / (np.pi * 1000.0), 1.0, delta=2e-3)
        self.assertAlmostEqual(float(area_wetted_surface_m2(10.0, 0.0)), 20.0, places=12)
        self.assertAlmostEqual(float(area_wetted_surface_m2(10.0, 0.2)), 20.0 * 1.05, places=12)

    def test_nita_scholz_oswald(self):
        """e_theo k_e,F from AeroSandbox, k_e,D0 0.804, k_e,M = 1 below M 0.3 and < 1 above."""
        mean_k_e_d0 = (0.873 + 0.864 + 0.804 + 0.804) / 4
        expected = float(oswalds_efficiency(1.0, 6.12, fuselage_diameter_to_span_ratio=0.14)) / mean_k_e_d0 * 0.804
        self.assertAlmostEqual(float(oswald_nita_scholz(1.0, 6.12, 0.14, 0.2)), expected, places=12)
        # Hand value: lambda - delta_lambda = 0.907, f = 0.00788, e_theo = 0.954, k_e,F = 0.961.
        self.assertAlmostEqual(float(oswald_nita_scholz(1.0, 6.12, 0.14, 0.2)), 0.954 * 0.961 * 0.804, delta=0.003)
        self.assertLess(float(oswald_nita_scholz(1.0, 6.12, 0.14, 0.6)), float(oswald_nita_scholz(1.0, 6.12, 0.14, 0.3)))

    def test_korn_drag_rise_of_the_thick_section(self):
        """Korn: M_DD = 0.87 - t/c - CL/10; Lock: M_crit = M_DD - 0.108; no wave drag at the Halo cruise Mach."""
        aero = ScholzAerodynamics()
        thickness = aircraft.wing.airfoil.max_thickness()
        mach_crit = 0.87 - thickness - 0.5 / 10 - (0.1 / 80)**(1 / 3)
        self.assertAlmostEqual(float(aero.wave_drag_coefficient(aircraft, 0.5, mach_crit + 0.05)),
                               20 * 0.05**4, delta=1e-6)
        self.assertEqual(float(aero.wave_drag_coefficient(aircraft, 0.5, mach_crit - 0.01)), 0.0)
        thick = replace(aircraft, wing=replace(aircraft.wing, airfoil=asb.Airfoil("naca2423")))
        self.assertEqual(float(aero.wave_drag_coefficient(thick, 0.6, 0.35)), 0.0)   # 210 kt at 10,000 ft
        self.assertGreater(float(aero.wave_drag_coefficient(thick, 0.6, 0.55)), 0.0)


class ScholzModelTests(unittest.TestCase):
    aero = ScholzAerodynamics(drag_area_misc_m2=0.25)

    def test_breakdown_and_polar(self):
        breakdown = self.aero.parasite_drag_breakdown(aircraft, 60, 1000)
        labels = [i.label for i in breakdown]
        self.assertEqual(labels[:4], ["wing", "horizontal_tail", "vertical_tail", "fuselage"])
        self.assertIn("miscellaneous", labels)
        result = self.aero.evaluate(aircraft, 60, 1000, 4.0)
        self.assertAlmostEqual(float(result.cd0), sum(float(i.cd0) for i in breakdown), places=12)
        self.assertAlmostEqual(float(result.cd - result.cd0),
                               float(result.cl**2 / (np.pi * result.oswald_efficiency * aircraft.wing.aspect_ratio)),
                               places=12)

    def test_fixed_gear_adds_drag_and_retracted_does_not(self):
        fixed = replace(aircraft, landing_gear=replace(aircraft.landing_gear, is_retractable=False))
        retracted = replace(aircraft, landing_gear=replace(aircraft.landing_gear, is_retractable=True))
        delta = float(self.aero.evaluate(fixed, 60, 1000, 2.0).cd0 - self.aero.evaluate(retracted, 60, 1000, 2.0).cd0)
        self.assertAlmostEqual(delta, self.aero.drag_area_landing_gear_fixed_m2 / aircraft.wing.area_m2, places=12)

    def test_same_order_as_simple(self):
        """Different correlations, same component method: within 30 % of SimpleAerodynamics' CD0."""
        simple = SimpleAerodynamics(drag_area_misc_m2=0.25)
        retracted = replace(aircraft, landing_gear=replace(aircraft.landing_gear, is_retractable=True))
        ratio = float(self.aero.evaluate(retracted, 60, 1000, 2.0).cd0 / simple.evaluate(retracted, 60, 1000, 2.0).cd0)
        self.assertAlmostEqual(ratio, 1.0, delta=0.1)

    def test_symbolic(self):
        opti = asb.Opti()
        velocity_m_s = opti.variable(init_guess=60.0, lower_bound=30.0, upper_bound=150.0)
        result = self.aero.evaluate(aircraft, velocity_m_s, 1000, 4.0)
        opti.minimize(result.drag_N / 1000)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution(velocity_m_s), 30.0, delta=1e-3)    # drag at fixed alpha rises with q


class DownloadTests(unittest.TestCase):
    def test_xv15_calibration_gives_seven_percent(self):
        """XV-15 (NDARC Table 1 geometry) at the calibration flap setting: TM X-62407's 0.07."""
        chord_m = 168.88 / 32.17 * u.foot
        self.assertAlmostEqual(float(download_fraction(chord_m, 12.5 * u.foot, 2)), 0.07, delta=2e-4)

    def test_formula_and_trends(self):
        d = HoverDownload()
        expected = (d.drag_coefficient_vertical * 2 * (4.0 / np.sqrt(2)) * chord_projected_m(2.0, 0.25, 60.0)
                    / (2 * np.pi * 16.0))
        self.assertAlmostEqual(float(download_fraction(2.0, 4.0, 2)), expected, places=12)
        self.assertAlmostEqual(float(chord_projected_m(2.0, 0.25, 0.0)), 2.0, places=12)    # flap up: full chord
        self.assertGreater(float(download_fraction(2.5, 4.0, 2)), float(download_fraction(2.0, 4.0, 2)))
        self.assertLess(float(download_fraction(2.0, 5.0, 2)), float(download_fraction(2.0, 4.0, 2)))
        flaps_up = HoverDownload(deflection_flap_hover_deg=0.0)
        self.assertGreater(float(download_fraction(2.0, 4.0, 2, flaps_up)), float(download_fraction(2.0, 4.0, 2)))

    def test_models_dispatch(self):
        self.assertEqual(SimpleAerodynamics(download_fraction_hover=0.07).hover_download_fraction(aircraft), 0.07)
        self.assertEqual(ScholzAerodynamics(download_fraction_hover=0.05).hover_download_fraction(aircraft), 0.05)


if __name__ == "__main__":
    unittest.main()
