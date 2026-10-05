"""Plan 033: gearbox stage count, efficiency and mass factor."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.powertrain.components.gearbox import Gearbox, GearStageModel


class GearStageTests(unittest.TestCase):
    def test_one_stage_up_to_the_max_ratio(self):
        model = GearStageModel()
        for ratio in (1.0, 1.5, 3.0, 4.9, 5.0):
            self.assertAlmostEqual(float(model.count_stages(ratio)), 1.0, delta=0.05)
        relaxed = GearStageModel(staircase=False)
        for ratio in (1.0, 3.0, 5.0):
            self.assertAlmostEqual(float(relaxed.count_stages(ratio)), 1.0, delta=0.05)

    def test_staircase_is_ceil_away_from_steps(self):
        model = GearStageModel()
        for ratio, stages in ((6.5, 2), (24.0, 2), (36.0, 3), (100.0, 3), (160.0, 4), (600.0, 4)):
            self.assertAlmostEqual(float(model.count_stages(ratio)), stages, delta=0.02)

    def test_monotonic(self):
        for model in (GearStageModel(), GearStageModel(staircase=False)):
            ratios = np.geomspace(1.0, 3000.0, 2000)
            counts = np.array([float(model.count_stages(r)) for r in ratios])
            self.assertTrue(np.all(np.diff(counts) >= -1e-12))

    def test_relaxed_is_smooth_max(self):
        relaxed = GearStageModel(staircase=False)
        for ratio in (10.0, 36.0, 400.0):
            x = np.log(ratio) / np.log(5.0)
            self.assertAlmostEqual(float(relaxed.count_stages(ratio)), x, delta=0.01)

    def test_efficiency_and_mass_identities(self):
        model = GearStageModel()
        for ratio, stages in ((3.0, 1), (20.0, 2), (36.0, 3)):
            self.assertAlmostEqual(float(model.efficiency(ratio)), 0.99 * 0.99 ** stages, delta=2e-4)
            self.assertAlmostEqual(float(model.mass_factor(ratio)), 1 + 0.3 * (stages - 1), delta=0.01)
        self.assertAlmostEqual(float(model.efficiency(20.0)), 0.970, delta=1e-3)    # Tier 13 constant at 2 stages

    def test_more_stages_cost_mass_and_efficiency(self):
        model = GearStageModel()
        self.assertLess(float(model.efficiency(36.0)), float(model.efficiency(4.0)))
        self.assertGreater(float(model.mass_factor(36.0)), float(model.mass_factor(4.0)))

    def test_ratio_per_stage_parameter(self):
        self.assertAlmostEqual(float(GearStageModel(ratio_max_stage=7.0).count_stages(36.0)), 2.0, delta=0.02)

    def test_gearbox_takes_symbolic_efficiency(self):
        opti = asb.Opti()
        ratio = opti.variable(init_guess=8.0, lower_bound=1.5, upper_bound=40.0)
        model = GearStageModel()
        gearbox = Gearbox(reduction_ratio=ratio, efficiency=model.efficiency(ratio))
        result = gearbox.evaluate(1000.0, 100.0)
        self.assertIsInstance(result.power_output_W, cas.MX)
        # Most efficient ratio above 10: the lowest stage count that reaches it (2 stages), i.e. any ratio <= 25.
        opti.subject_to(ratio >= 10.0)
        opti.minimize(-result.power_output_W / 1e5 + 1e-4 * ratio)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(model.count_stages(solution.value(ratio))), 2.0, delta=0.05)


if __name__ == "__main__":
    unittest.main()
