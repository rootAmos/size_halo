import math
import tempfile
import unittest
from dataclasses import dataclass

import numpy as np

from aircraft_closure.uncertainty.manifest import build_manifest, read_manifest, write_manifest
from aircraft_closure.uncertainty.model import UncertainInput
from aircraft_closure.uncertainty.rank import rank_drivers, variance_shares
from aircraft_closure.uncertainty.registry import apply_overrides, inputs_from_dataclass


@dataclass(frozen=True)
class Assumptions:
    aspect_ratio: float = 10.0
    energy_density_cell_Wh_kg: float = 300.0
    count_blades: int = 3
    thermal: bool = True
    wing_model: str = "afdd"


class RegistryTest(unittest.TestCase):
    def test_every_field_listed(self):
        inputs = inputs_from_dataclass(Assumptions(), "assumptions", default_kind="technology")
        self.assertEqual([i.key for i in inputs], ["assumptions." + n for n in
                                                    ("aspect_ratio", "energy_density_cell_Wh_kg", "count_blades",
                                                     "thermal", "wing_model")])
        self.assertEqual([i.kind for i in inputs][-2:], ["setting", "setting"])   # bool and str are settings

    def test_overrides(self):
        inputs = apply_overrides(inputs_from_dataclass(Assumptions(), "a"), {
            "a.aspect_ratio": dict(kind="design"),
            "a.energy_density_cell_Wh_kg": dict(kind="technology", low=270.0, high=330.0, source="vendor")})
        self.assertEqual(inputs[0].kind, "design")
        self.assertAlmostEqual(inputs[1].spread(), 60 / math.sqrt(12))
        with self.assertRaises(KeyError):
            apply_overrides(inputs, {"a.typo": dict(sigma=1.0)})
        with self.assertRaises(ValueError):
            apply_overrides(inputs, {"a.aspect_ratio": dict(value=12.0)})
        with self.assertRaises(ValueError):
            apply_overrides(inputs, {"a.aspect_ratio": dict(kind="guess")})


class RankTest(unittest.TestCase):
    def setUp(self):
        self.inputs = [UncertainInput("ar", 10.0, kind="design"),
                       UncertainInput("cell", 300.0, kind="technology", sigma=15.0),
                       UncertainInput("drag", 1.0, kind="model", sigma=0.1),
                       UncertainInput("hover_s", 60.0, kind="requirement"),          # no spread: blind spot
                       UncertainInput("lanes", 2, kind="setting")]
        # One output (range, nm): d/d(ar) 20, d/d(cell) 1, d/d(drag) -300, d/d(hover) -0.5, lanes unused.
        self.jacobian = np.array([[20.0, 1.0, -300.0, -0.5, 0.0]])
        self.elasticity = np.array([[0.45, 0.67, -0.67, -0.07, 0.0]])

    def test_shares_sum_to_one_over_quantified_inputs(self):
        shares = variance_shares(self.jacobian, [None, 15.0, 0.1, None, None])
        self.assertAlmostEqual(float(np.nansum(shares)), 1.0)
        self.assertAlmostEqual(shares[0, 2], 900 / (900 + 225))          # (300 * 0.1)^2 vs (1 * 15)^2

    def test_risks_levers_blind_spots(self):
        risks, levers, blind = rank_drivers(["range"], self.inputs, self.jacobian, self.elasticity)["range"]
        self.assertEqual([d.input_key for d in risks], ["drag", "cell"])
        self.assertAlmostEqual(risks[0].spread, 30.0)
        self.assertEqual([d.input_key for d in levers], ["ar"])
        self.assertEqual(blind, ["hover_s"])                               # the setting is not listed at all

    def test_uncomputed_derivative_is_blind(self):
        jacobian = self.jacobian.copy()
        jacobian[0, 1] = np.nan
        risks, _, blind = rank_drivers(["range"], self.inputs, jacobian, self.elasticity)["range"]
        self.assertEqual([d.input_key for d in risks], ["drag"])
        self.assertIn("cell", blind)


class ManifestTest(unittest.TestCase):
    def test_round_trip(self):
        inputs = inputs_from_dataclass(Assumptions(), "assumptions")
        manifest = build_manifest("toy solve", inputs, dict(range_m=8.2e5, margin=float("nan")),
                                  sensitivity=dict(mode="resized", outputs=["range_m"], inputs=["assumptions.aspect_ratio"],
                                                   jacobian=np.array([[1.5]])))
        with tempfile.TemporaryDirectory() as directory:
            back = read_manifest(write_manifest(directory, manifest))
        self.assertEqual(len(back["inputs"]), 5)
        self.assertEqual(back["inputs"][4]["value"], "afdd")
        self.assertIsNone(back["outputs"]["margin"])                       # NaN written as null
        self.assertEqual(back["sensitivity"]["jacobian"], [[1.5]])
        self.assertIn("commit", back)


if __name__ == "__main__":
    unittest.main()
