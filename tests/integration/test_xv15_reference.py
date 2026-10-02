import unittest
from dataclasses import fields, replace

import aerosandbox.tools.units as u

from examples.xv15_reference import (Xv15GroupMasses, Xv15MassFactors, Xv15Reference, calibration_factors,
                                     compare_groups, mass_turboshaft_from_power_kg, published_groups,
                                     solve_xv15_closure)
from aerosandbox.library.power_turboshaft import power_turboshaft


class Xv15ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mass_engine_kg = mass_turboshaft_from_power_kg(Xv15Reference().power_takeoff_engine_W)
        cls.predicted = compare_groups(mass_engine_kg=cls.mass_engine_kg)
        cls.factors = calibration_factors(mass_engine_kg=cls.mass_engine_kg)
        cls.uncalibrated = solve_xv15_closure(mass_engine_kg=cls.mass_engine_kg)
        cls.calibrated = solve_xv15_closure(factors=cls.factors, mass_engine_kg=cls.mass_engine_kg)

    def test_published_statement_sums_to_basic_empty_weight(self):
        self.assertAlmostEqual(published_groups.total() / u.lbm, 9076, places=6)
        reference = Xv15Reference()
        self.assertAlmostEqual((reference.mass_basic_empty_kg + reference.mass_useful_non_fuel_kg
                                + reference.mass_fuel_kg) / u.lbm, 13000, places=6)

    def test_engine_mass_inverts_the_aerosandbox_regression(self):
        self.assertAlmostEqual(float(power_turboshaft(self.mass_engine_kg)) / (1550 * u.hp), 1.0, places=6)
        # Bracket: T53-L-701 with nose gearbox is 688 lb; the LTC1K-4K removed it.
        self.assertTrue(450 < self.mass_engine_kg / u.lbm < 688)

    def test_equipment_is_carried_not_predicted(self):
        self.assertAlmostEqual(self.predicted.equipment, published_groups.equipment, places=9)

    def test_calibration_reproduces_the_statement(self):
        calibrated = compare_groups(factors=self.factors, mass_engine_kg=self.mass_engine_kg)
        for f in fields(Xv15GroupMasses):
            self.assertAlmostEqual(getattr(calibrated, f.name), getattr(published_groups, f.name), places=6,
                                   msg=f.name)

    def test_calibrated_closure_returns_design_gross_weight(self):
        self.assertLess(abs(self.calibrated.closure_residual_kg), 1e-6)
        self.assertAlmostEqual(self.calibrated.mass_takeoff_kg / u.lbm, 13000, places=3)
        self.assertAlmostEqual(self.calibrated.mass_empty_kg / u.lbm, 9076, places=3)

    def test_uncalibrated_closure_regression_band(self):
        """Uncalibrated framework closes light (11,315 lb in plan 011); guard against silent drift."""
        self.assertLess(abs(self.uncalibrated.closure_residual_kg), 1e-6)
        self.assertTrue(10500 < self.uncalibrated.mass_takeoff_kg / u.lbm < 12000)
        self.assertLess(self.uncalibrated.mass_empty_kg, Xv15Reference().mass_basic_empty_kg)

    def test_drive_system_model_agrees_with_statement(self):
        """AFDD83 gearbox plus AFDD82 interconnect against transmission/conversion: within 5 %."""
        self.assertAlmostEqual(self.predicted.transmission / published_groups.transmission, 1.0, delta=0.05)

    def test_rotor_mass_rises_with_coning_frequency(self):
        stiff = compare_groups(replace(Xv15Reference(), frequency_coning_per_rev=1.6), mass_engine_kg=self.mass_engine_kg)
        soft = compare_groups(replace(Xv15Reference(), frequency_coning_per_rev=1.0), mass_engine_kg=self.mass_engine_kg)
        self.assertGreater(stiff.rotor, self.predicted.rotor)
        self.assertGreater(self.predicted.rotor, soft.rotor)
        # Only the rotor group depends on the coning frequency.
        self.assertAlmostEqual(stiff.wing, soft.wing, places=9)

    def test_unit_factors_are_the_uncalibrated_model(self):
        self.assertEqual(Xv15MassFactors(), Xv15MassFactors(**{f.name: 1.0 for f in fields(Xv15MassFactors)}))


if __name__ == "__main__":
    unittest.main()
