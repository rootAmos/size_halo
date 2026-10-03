import unittest

from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor
from examples.jvx_rotor_calibration import calibration_report, load_airplane, load_hover


class JvxCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = calibration_report()

    def test_parsed_data_counts(self):
        hover = load_hover()
        self.assertEqual(sum(1 for t in hover.table if t == "D-1"), 58)
        self.assertEqual(sum(1 for t in hover.table if t == "D-2"), 13)
        self.assertEqual(len(load_airplane().advance_ratio), 42)

    def test_parsed_airplane_efficiency_is_consistent(self):
        data = load_airplane()
        for load, lam, power, eta in zip(data.blade_loading, data.advance_ratio, data.power_sigma, data.efficiency):
            self.assertAlmostEqual(load * lam / power, eta, delta=5e-4)

    def test_embedded_constants_are_the_fit(self):
        rotor = MomentumProfileRotor()
        for fitted, embedded in zip(self.report.polar + self.report.increment,
                                    rotor.drag_polar + rotor.drag_increment_airplane):
            self.assertAlmostEqual(fitted, embedded, delta=abs(embedded) * 1e-3)

    def test_hover_fit_quality(self):
        self.assertLess(self.report.hover_fm_rms, 0.015)

    def test_higher_tip_mach_hover_predicted(self):
        """Table D-2 (Mtip 0.73) is not in the fit; power within 3 %."""
        self.assertLess(self.report.hover_d2_power_error_max, 0.03)

    def test_airplane_fit_quality(self):
        self.assertLess(self.report.airplane_eta_rms, 0.015)
        self.assertLess(self.report.airplane_eta_max, 0.04)

    def test_drag_polar_is_physical(self):
        d0, d1, d2 = self.report.polar
        self.assertGreater(d2, 0)
        bucket = -d1 / (2 * d2)
        self.assertTrue(0.05 < bucket < 0.15)
        self.assertGreater(d0 + d1 * bucket + d2 * bucket**2, 0.005)


if __name__ == "__main__":
    unittest.main()
