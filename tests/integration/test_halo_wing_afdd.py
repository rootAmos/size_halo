"""Tier 20 (plan 024) and plan 026: Halo with the AFDD tiltrotor wing (the default from plan 026); equipment."""
import unittest
from dataclasses import replace

import aerosandbox.tools.units as u

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_tier20, halo_equipment_items,
                                  mass_equipment_from_items_kg, requirements_plan022, assumptions_plan026,
                                  requirements_plan026, solve_halo_sizing,
                                  uncrewed_equipment_adjustments)


class DefaultsTests(unittest.TestCase):
    def test_default_wing_model_is_afdd_at_900_kg(self):
        """Plan 026 (adopted 2026-10-03)."""
        self.assertEqual(HaloAssumptions().wing_weight_model, "afdd_tiltrotor")
        self.assertEqual(HaloRequirements().mass_payload_kg, 900.0)
        self.assertEqual(assumptions_tier20.wing_weight_model, "afdd_tiltrotor")

    def test_itemised_equipment_keeps_the_reference_mass(self):
        self.assertAlmostEqual(HaloAssumptions().mass_equipment_kg / u.lbm, 587.0, places=9)
        self.assertAlmostEqual(mass_equipment_from_items_kg(halo_equipment_items) / u.lbm, 587.0, places=9)
        removals = [i for i in uncrewed_equipment_adjustments if i.mass_kg < 0]
        self.assertTrue(any("ejection seats" in i.label for i in removals))
        self.assertTrue(any("environmental control" in i.label for i in removals))
        self.assertTrue(all(i.source for i in halo_equipment_items))


class HaloAfddWingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = solve_halo_sizing(requirements_plan022, assumptions_tier20)      # 780 kg, as in plan 024

    def test_closes_lighter_than_reference(self):
        r = self.result
        self.assertLess(abs(r.closure_residual_kg), 1e-3)
        self.assertAlmostEqual(r.mass_takeoff_kg, 6144, delta=40)          # reference (Raymer): 6,548 kg
        self.assertAlmostEqual(r.mass_payload_kg, 780.0, places=6)

    def test_wing_breakdown_is_reported_and_matches_the_group(self):
        r = self.result
        parts = dict(r.wing_masses_kg)
        self.assertAlmostEqual(sum(parts.values()) * 1.327, dict(r.component_masses_kg)["wing"], delta=3.0)
        self.assertLess(dict(r.component_masses_kg)["wing"], 547.5)        # Raymer reference wing

    def test_whirl_flutter_margins_hold_and_bind_at_max_speed(self):
        r = self.result
        torsion = HaloAssumptions().frequency_torsion_wing_per_rev
        beam = HaloAssumptions().frequency_beam_wing_per_rev
        self.assertTrue(r.whirl_flutter)
        for w in r.whirl_flutter:
            self.assertGreaterEqual(w["torsion_per_rev"], torsion - 1e-6, w)
            self.assertGreaterEqual(w["beam_per_rev"], beam - 1e-6, w)
        self.assertIn("whirl flutter torsion per rev (max_speed)", r.binding)
        fastest = max(r.whirl_flutter, key=lambda w: w["speed_rotor_rad_s"])
        self.assertAlmostEqual(fastest["speed_rotor_rad_s"], r.design.speed_rotor_wing_design_rad_s, delta=1e-3)

    def test_max_payload(self):
        """Reference (Raymer wing) maximum: 785 kg (plan 022); with the AFDD wing about 959 kg."""
        initial = solve_halo_sizing(HaloRequirements(), replace(assumptions_tier20, battery_model="constant"),
                                    objective="payload")
        result = solve_halo_sizing(HaloRequirements(), assumptions_tier20, objective="payload", initial=initial)
        self.assertAlmostEqual(result.mass_payload_kg, 959, delta=10)


class Plan026ReferenceTests(unittest.TestCase):
    """The plan 026 reference: 900 kg, AFDD wing, equivalent-circuit battery, SimpleAerodynamics."""

    @classmethod
    def setUpClass(cls):
        cls.result = solve_halo_sizing(requirements_plan026, assumptions_plan026)

    def test_reference_closes_at_900_kg(self):
        r = self.result
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-5)
        self.assertEqual(r.mass_payload_kg, 900.0)
        self.assertAlmostEqual(r.mass_takeoff_kg / u.lbm, 14247, delta=5)

    def test_whirl_flutter_and_battery_voltage_size_the_design(self):
        self.assertIn("whirl flutter torsion per rev (max_speed)", self.result.binding)
        self.assertIn("engine-out hover 3/3: battery end voltage_V", self.result.binding)


if __name__ == "__main__":
    unittest.main()
