import unittest

import aerosandbox as asb

from aircraft_closure.core.sensitivity import elasticities, objective_sensitivities, parametric_jacobian


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


class ParametricJacobianTest(unittest.TestCase):
    def test_matches_closed_form(self):
        # x = p (active bound moved into lbg), y = 3 + q - p (equality): outputs x, y, x y.
        opti, solution, (p, q, r) = solve()
        x, y = opti.x[0], opti.x[1]
        jacobian = parametric_jacobian(opti, solution, [x, y, x * y], [p, q, r])
        expected = [[1, 0, 0], [-1, 1, 0], [0.5 - 2.5, 2.5, 0]]
        for row, row_expected in zip(jacobian, expected):
            for value, value_expected in zip(row, row_expected):
                self.assertAlmostEqual(value, value_expected, places=5)

    def test_objective_row_matches_envelope_theorem(self):
        opti, solution, parameters = solve()
        jacobian = parametric_jacobian(opti, solution, [opti.f], list(parameters))
        for value, envelope in zip(jacobian[0], objective_sensitivities(opti, solution, list(parameters))):
            self.assertAlmostEqual(value, envelope, places=5)

    def test_interior_optimum_matches_finite_difference(self):
        # No active constraint: the Hessian alone carries the sensitivity.
        def solve_interior(a_value):
            opti = asb.Opti()
            x = opti.variable(init_guess=1.0)
            a = opti.parameter(a_value)
            opti.minimize((x - a ** 2) ** 2 + 0.1 * x ** 4)
            return opti, opti.solve(verbose=False), a, x
        h = 1e-4
        opti, solution, a, x = solve_interior(1.3)
        value = parametric_jacobian(opti, solution, [x ** 2], [a])[0, 0]
        low, high = solve_interior(1.3 - h), solve_interior(1.3 + h)
        fd = (high[1].value(high[3]) ** 2 - low[1].value(low[3]) ** 2) / (2 * h)
        self.assertAlmostEqual(value, fd, places=4)

    def test_elasticity(self):
        # y = x^2 at x = 3: dy/dx = 6, elasticity 6 * 3 / 9 = 2; zero output gives NaN.
        result = elasticities([[6.0], [1.0]], [9.0, 0.0], [3.0])
        self.assertAlmostEqual(result[0, 0], 2.0)
        self.assertTrue(result[1, 0] != result[1, 0])


if __name__ == "__main__":
    unittest.main()
