"""Plan 050: our offset-strip-fin core against OpenConcept's HXGroup, code to code.

OpenConcept's outputs are stored (tests/powertrain/data/compact_heat_exchanger_openconcept.json, OpenConcept 1.2.6,
2026-10-11, written by `examples/ntu_openconcept_check.py`), so OpenConcept is not needed here. Same inputs on both
sides: geometry, material, fluid properties, flows and inlet temperatures.

The one modelling difference is the coolant-side hydraulic diameter: ours is Manglik and Bergles' offset-strip-fin
d_h = 4 w h l / (2 (w l + h l + t h) + t w) on both sides, OpenConcept's hot side is the plain duct 2 w h / (w + h)
(1.26 % larger for the 1 x 1 x 6 mm coolant fins). With OpenConcept's d_h substituted, the two codes agree to
round-off; as is, the coolant-side Reynolds number, j, f, h and pressure drop differ by the amounts below and the
heat by under 0.05 %.
"""
import json
import unittest

from examples.ntu_openconcept_check import quantities, relative_differences, stored_path

data = json.loads(stored_path.read_text())


class OpenConceptReferenceTests(unittest.TestCase):
    def test_reference_metadata_and_cases(self):
        self.assertIn("openconcept 1.2.6", data["source"])
        names = [entry["inputs"]["name"] for entry in data["cases"]]
        self.assertIn("openconcept_defaults", names)
        self.assertIn("halo", names)
        self.assertGreaterEqual(len(names), 11)
        # Both capacity-rate branches are covered: the coolant is C_min in OpenConcept's case and at 12 kg/s of air.
        ratios = [e["inputs"]["mdot_cold"] * e["inputs"]["air"]["cp"] / (e["inputs"]["mdot_hot"] * e["inputs"]["coolant"]["cp"])
                  for e in data["cases"]]
        self.assertTrue(any(r > 1 for r in ratios) and any(r < 1 for r in ratios))

    def test_identical_with_openconcept_coolant_hydraulic_diameter(self):
        # Substituting OpenConcept's hot-side d_h leaves no other difference: every quantity to round-off.
        for entry in data["cases"]:
            _, differences = relative_differences(entry, hydraulic_diameter_coolant="duct")
            for name in quantities:
                with self.subTest(case=entry["inputs"]["name"], quantity=name):
                    self.assertLess(abs(differences[name]), 1e-9)

    def test_as_is_geometry_air_side_and_mass_identical(self):
        coolant_affected = {"diameter_hydraulic_coolant_m", "reynolds_coolant", "colburn_coolant", "fanning_coolant",
                            "coefficient_coolant_W_m2_K", "efficiency_surface_coolant", "conductance_W_K", "ntu",
                            "effectiveness", "power_heat_W", "temperature_air_out_K", "temperature_coolant_out_K",
                            "pressure_drop_coolant_Pa"}
        for entry in data["cases"]:
            _, differences = relative_differences(entry)
            for name in set(quantities) - coolant_affected:
                with self.subTest(case=entry["inputs"]["name"], quantity=name):
                    self.assertLess(abs(differences[name]), 1e-9)

    def test_as_is_coolant_side_differences_bounded(self):
        # Upper bounds on |relative difference| from the d_h difference alone (temperatures: fraction of the change).
        bounds = {"diameter_hydraulic_coolant_m": 0.013, "reynolds_coolant": 0.013, "colburn_coolant": 0.0075,
                  "fanning_coolant": 0.010, "coefficient_coolant_W_m2_K": 0.0075, "efficiency_surface_coolant": 5e-4,
                  "conductance_W_K": 0.001, "ntu": 0.001, "effectiveness": 5e-4, "power_heat_W": 5e-4,
                  "temperature_air_out_K": 5e-4, "temperature_coolant_out_K": 5e-4, "pressure_drop_coolant_Pa": 0.025}
        for entry in data["cases"]:
            case = entry["inputs"]
            _, differences = relative_differences(entry)
            w, h, l, t = case["channel_width_hot"], case["channel_height_hot"], case["fin_length_hot"], case["fin_thickness"]
            ratio = (4 * w * h * l / (2 * (w * l + h * l + t * h) + t * w)) / (2 * w * h / (w + h))
            self.assertAlmostEqual(1 + differences["diameter_hydraulic_coolant_m"], ratio, places=12)
            for name, bound in bounds.items():
                with self.subTest(case=case["name"], quantity=name):
                    self.assertLess(abs(differences[name]), bound)
            # Smaller d_h: lower Re, so higher j and f; the loss scales as f / d_h, so it rises most.
            self.assertGreater(differences["pressure_drop_coolant_Pa"], differences["fanning_coolant"])
            self.assertGreater(differences["fanning_coolant"], 0)
            self.assertGreater(differences["power_heat_W"], 0)


if __name__ == "__main__":
    unittest.main()
