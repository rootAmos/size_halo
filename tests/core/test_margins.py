import unittest

import aerosandbox as asb
import casadi as cas

from aircraft_closure.core.margins import margin_above, margin_below, margin_report


class MarginTests(unittest.TestCase):
    def test_zero_at_limit(self):
        self.assertEqual(margin_below("x", 100, 100).value, 0)
        self.assertEqual(margin_above("x", 100, 100).value, 0)

    def test_signs_and_normalization(self):
        self.assertAlmostEqual(margin_below("x", 90, 100).value, 0.1)
        self.assertAlmostEqual(margin_below("x", 110, 100).value, -0.1)
        self.assertAlmostEqual(margin_above("x", 440, 400).value, 0.1)
        self.assertAlmostEqual(margin_above("x", 360, 400).value, -0.1)

    def test_report_sorted_and_flagged(self):
        report = margin_report([margin_below("a", 50, 100), margin_below("b", 120, 100),
                                margin_above("c", 101, 100)])
        self.assertEqual([entry.label for entry in report], ["b", "c", "a"])
        self.assertEqual([entry.compatible for entry in report], [False, True, True])

    def test_symbolic_margin_constrains_opti(self):
        opti = asb.Opti()
        power_W = opti.variable(init_guess=50)
        margin = margin_below("power", power_W, 100)
        self.assertIsInstance(margin.value, cas.MX)
        opti.minimize(-power_W)
        opti.subject_to(margin.value >= 0.25)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(power_W)), 75, places=5)
        report = margin_report([margin], solution.value)
        self.assertAlmostEqual(float(report[0].value), 0.25, places=6)

    def test_report_rejects_unevaluated_symbolic(self):
        opti = asb.Opti()
        with self.assertRaises(Exception):
            margin_report([margin_below("power", opti.variable(init_guess=1), 100)])


if __name__ == "__main__":
    unittest.main()
