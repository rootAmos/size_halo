"""Tier 20 (plan 024): XV-15 calibration of the AFDD tiltrotor wing and its V-22 / Bell D266 cross-checks."""
import unittest
from dataclasses import fields

import aerosandbox.tools.units as u

from aircraft_closure.vehicle.surfaces import TiltrotorWingMassModel
from examples.tiltrotor_wing_calibration import (d266_rotor_implied_solidity, d266_wing_check, load_reference_data,
                                                 v22_wing_check, xv15_frequency_stiffness_ratios,
                                                 xv15_section_calibration)
from examples.xv15_reference import (Xv15MassFactors, Xv15Reference, calibration_factors, compare_groups,
                                     mass_turboshaft_from_power_kg, published_groups, solve_xv15_closure)


class Xv15CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mass_engine_kg = mass_turboshaft_from_power_kg(Xv15Reference().power_takeoff_engine_W)
        cls.factors = calibration_factors(mass_engine_kg=cls.mass_engine_kg)

    def test_reference_data_complete(self):
        data = load_reference_data()
        breakdown = sum(data[("xv15", k)] for k in ("wing_torque_box", "wing_spars", "wing_control_surfaces",
                                                     "wing_fairings", "wing_fittings_other"))
        self.assertEqual(breakdown, data[("xv15", "wing_total_breakdown")])     # 946 lb
        for key in (("v22", "wing_overall"), ("d266", "wing_group"), ("d266", "rotor_group")):
            self.assertIn(key, data)

    def test_section_calibration_reproduces_model_defaults(self):
        calibration = xv15_section_calibration()
        defaults = TiltrotorWingMassModel(mass_tip_kg=1.0, radius_gyration_pylon_m=1.0, speed_rotor_design_rad_s=1.0,
                                          width_fuselage_m=1.0)
        for name in ("efficiency_torque_box", "correction_spar_stiffness", "unit_mass_fairing_kg_m2",
                     "unit_mass_control_surfaces_kg_m2", "fraction_fittings", "fraction_area_control_surfaces"):
            self.assertAlmostEqual(getattr(defaults, name) / getattr(calibration, name), 1.0, delta=0.005, msg=name)

    def test_model_stiffness_from_published_modes(self):
        """NDARC's single-mode relations give about 0.6 of the published XV-15 stiffness (model form; absorbed by
        the wing factor). Regression band."""
        for ratio in xv15_frequency_stiffness_ratios():
            self.assertTrue(0.5 < ratio < 0.7, ratio)

    def test_wing_factor_is_separate_and_existing_factors_unchanged(self):
        self.assertAlmostEqual(self.factors.wing_tiltrotor, 1.327, delta=0.01)
        self.assertAlmostEqual(self.factors.wing, 1.930, delta=0.005)          # Raymer factor (Tier 10a)
        self.assertEqual(Xv15MassFactors().wing_tiltrotor, 1.0)

    def test_calibrated_afdd_wing_reproduces_statement(self):
        groups = compare_groups(factors=self.factors, mass_engine_kg=self.mass_engine_kg,
                                wing_weight_model="afdd_tiltrotor")
        for f in fields(groups):
            self.assertAlmostEqual(getattr(groups, f.name) / getattr(published_groups, f.name), 1.0, places=6,
                                   msg=f.name)

    def test_calibrated_closure_with_afdd_wing(self):
        closure = solve_xv15_closure(factors=self.factors, mass_engine_kg=self.mass_engine_kg,
                                     wing_weight_model="afdd_tiltrotor")
        self.assertAlmostEqual(closure.mass_takeoff_kg / u.lbm, 13000, delta=1)

    def test_uncalibrated_closure_moves_toward_statement(self):
        """The stiffness-based wing (658 lb at design weight) is nearer the 873 lb statement than Raymer (452 lb), so
        the uncalibrated closure is heavier than Tier 10a's 11,315 lb."""
        raymer = solve_xv15_closure(mass_engine_kg=self.mass_engine_kg)
        afdd = solve_xv15_closure(mass_engine_kg=self.mass_engine_kg, wing_weight_model="afdd_tiltrotor")
        self.assertAlmostEqual(raymer.mass_takeoff_kg / u.lbm, 11315, delta=10)
        self.assertGreater(afdd.mass_takeoff_kg, raymer.mass_takeoff_kg)
        self.assertLess(afdd.mass_takeoff_kg / u.lbm, 13000)


class CrossCheckTests(unittest.TestCase):
    """No further fitting: the XV-15-calibrated model against the second calibration aircraft."""

    @classmethod
    def setUpClass(cls):
        cls.factors = calibration_factors()

    def test_v22_wing_within_25_percent(self):
        check = v22_wing_check(self.factors)
        error = check.mass_predicted_calibrated_kg / check.mass_actual_kg - 1
        self.assertTrue(-0.25 < error < 0.0, error)     # -18 %: under-predicted (fold/rotate scope unstated)

    def test_d266_wing_within_10_percent(self):
        check = d266_wing_check(self.factors)
        error = check.mass_predicted_calibrated_kg / check.mass_actual_kg - 1
        self.assertLess(abs(error), 0.10, error)

    def test_d266_rotor_implied_solidity(self):
        """AFDD82 x the XV-15 rotor factor needs solidity 0.062 for the D266 rotor group: below typical proprotors,
        so at a typical 0.09 the calibrated rotor model over-predicts the D266 (a 1968 design estimate)."""
        self.assertAlmostEqual(d266_rotor_implied_solidity(self.factors), 0.062, delta=0.003)


if __name__ == "__main__":
    unittest.main()
