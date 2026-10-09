import unittest

import aerosandbox as asb

from aircraft_closure.core.sensitivity import objective_sensitivities


def solve(p_value=2.5, q_value=0.0, r_value=1.0):
    opti = asb.Opti()
    x = opti.variable(init_guess=1.0)
    y = opti.variable(init_guess=1.0)
    p, q, r = opti.parameter(p_value), opti.parameter(q_value), opti.parameter(r_value)
    opti.subject_to([x >= p, x + y == 3 + q, x * r + y >= -10 * r])
    opti.minimize(r * x ** 2 + 2 * y ** 2)
    return opti, opti.solve(verbose=False), (p, q, r)


class ObjectiveSensitivityTest(unittest.TestCase):
    def test_matches_closed_form(self):
        # x = p (active), y = 3 + q - p: J = r p^2 + 2 (3 + q - p)^2.
        opti, solution, parameters = solve()
        dp, dq, dr = objective_sensitivities(opti, solution, list(parameters))
        self.assertAlmostEqual(dp, 2 * 2.5 - 4 * 0.5, places=5)
        self.assertAlmostEqual(dq, 4 * 0.5, places=5)
        self.assertAlmostEqual(dr, 2.5 ** 2, places=5)

    def test_matches_finite_difference(self):
        h = 1e-4
        opti, solution, parameters = solve()
        dp = objective_sensitivities(opti, solution, list(parameters))[0]
        low, high = solve(p_value=2.5 - h), solve(p_value=2.5 + h)
        fd = (high[1].value(high[0].f) - low[1].value(low[0].f)) / (2 * h)
        self.assertAlmostEqual(dp, fd, places=4)

    def test_inactive_constraint_has_no_price(self):
        opti = asb.Opti()
        x = opti.variable(init_guess=3.0)
        p = opti.parameter(-5.0)
        opti.subject_to(x >= p)                      # x* = 1 > p: inactive
        opti.minimize((x - 1) ** 2)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(objective_sensitivities(opti, solution, [p])[0], 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
