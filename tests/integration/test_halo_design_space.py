"""Tier 22 (plan 029): starting-point strategy, freed trades, cost objective, enumeration, sensitivity."""
import unittest
from dataclasses import replace

import aerosandbox.tools.units as u

from examples.halo_design_space import UncertainInput, enumerate_architectures, sensitivity_study
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, StartRecord, assumptions_plan026,
                                  default_starts, perturbed_start, requirements_plan026, solve_halo_sizing,
                                  solve_halo_sizing_multistart)

# Thermal model pinned off: these cases are the plan 027 (thermal-off) aircraft whatever the default is.
reference = replace(HaloAssumptions(), thermal_model=False)
scholz = replace(reference, aerodynamics_model="scholz")


class StartListTests(unittest.TestCase):
    def test_default_order(self):
        r = HaloRequirements()
        self.assertEqual(default_starts(r, reference, "mass_takeoff"),
                         ("scholz_aero", "constant_battery", "payload_continuation", "perturbed_low",
                          "perturbed_high", "generic"))
        self.assertEqual(default_starts(r, scholz, "payload"),
                         ("constant_battery", "perturbed_low", "perturbed_high", "generic"))
        self.assertEqual(default_starts(r, replace(scholz, battery_model="constant"), "mass_takeoff"), ("generic",))
        # Thermal model on (plan 028): the thermal-off problem is the first start.
        self.assertEqual(default_starts(r, replace(reference, thermal_model=True), "cost")[:3],
                         ("thermal_off", "mass_objective", "scholz_aero"))
        self.assertEqual(default_starts(replace(r, mass_payload_kg=200.0), scholz, "cost"),
                         ("mass_objective", "constant_battery", "perturbed_low", "perturbed_high", "generic"))

    def test_bad_arguments(self):
        with self.assertRaises(ValueError):
            solve_halo_sizing_multistart(assumptions=scholz, starts=("warm",))
        with self.assertRaises(ValueError):
            solve_halo_sizing_multistart(assumptions=scholz, starts=("caller",))
        with self.assertRaises(ValueError):
            solve_halo_sizing_multistart(assumptions=scholz, select="worst")


class StartStrategyTests(unittest.TestCase):
    """The previously fragile cases solve without a hand-supplied `initial`, and record which start worked."""

    @classmethod
    def setUpClass(cls):
        cls.scholz = solve_halo_sizing(assumptions=scholz)
        cls.plan026 = solve_halo_sizing(requirements_plan026, assumptions_plan026)
        cls.payload = solve_halo_sizing(assumptions=scholz, objective="payload")
        cls.best = solve_halo_sizing_multistart(assumptions=scholz, select="best")

    def test_scholz_reference(self):
        record = self.scholz.start
        self.assertIsInstance(record, StartRecord)
        self.assertEqual(record.label, "constant_battery")
        self.assertEqual(len(record.attempts), 1)
        self.assertAlmostEqual(self.scholz.mass_takeoff_kg / u.lbm, 13702, delta=10)

    def test_900_kg_ecm_needs_payload_continuation(self):
        """Plan 026: from the constant-battery design IPOPT fails; the 85 % payload design succeeds."""
        record = self.plan026.start
        self.assertEqual([a.label for a in record.attempts], ["constant_battery", "payload_continuation"])
        self.assertFalse(record.attempts[0].success)
        self.assertEqual(record.label, "payload_continuation")
        self.assertAlmostEqual(self.plan026.mass_takeoff_kg / u.lbm, 14247, delta=5)

    def test_ecm_max_payload_without_initial(self):
        """Plan 022: from the constant-battery design the max-payload problem fails; a perturbed start solves."""
        record = self.payload.start
        self.assertFalse(record.attempts[0].success)
        self.assertTrue(record.attempts[-1].success)
        self.assertGreater(self.payload.mass_payload_kg, 1500.0)
        self.assertGreater(self.payload.min_margin, -1e-6)

    def test_best_of_all_starts_and_spread(self):
        record = self.best.start
        self.assertEqual(len(record.attempts), len(default_starts(HaloRequirements(), scholz, "mass_takeoff")))
        self.assertGreaterEqual(len(record.successes()), 3)
        self.assertEqual(record.label, "constant_battery")      # ties go to the earlier start
        self.assertLess(record.spread_mass_takeoff_kg(), 1.0)
        self.assertAlmostEqual(self.best.mass_takeoff_kg, self.scholz.mass_takeoff_kg, delta=0.1)

    def test_perturbed_start_scales_the_design(self):
        start = perturbed_start(self.scholz, 1.15)
        self.assertAlmostEqual(start.design.area_wing_m2, 1.15 * self.scholz.design.area_wing_m2)
        self.assertAlmostEqual(start.mass_takeoff_kg, 1.15 * self.scholz.mass_takeoff_kg)
        self.assertIsNone(start.design.aspect_ratio_wing)


class FreedTradeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = solve_halo_sizing(assumptions=scholz)
        cls.freed = {flag: solve_halo_sizing_multistart(assumptions=replace(scholz, **{flag: True}), initial=cls.base)
                     for flag in ("free_aspect_ratio_wing", "free_altitude_cruise", "free_soc_reserve")}

    def test_each_closes_inside_its_bounds_and_is_no_heavier(self):
        a = scholz
        for flag, field_name, bounds in (("free_aspect_ratio_wing", "aspect_ratio_wing", a.bounds_aspect_ratio_wing),
                                         ("free_altitude_cruise", "altitude_cruise_m", a.bounds_altitude_cruise_m),
                                         ("free_soc_reserve", "soc_reserve", a.bounds_soc_reserve)):
            result = self.freed[flag]
            value = getattr(result.design, field_name)
            self.assertGreater(result.min_margin, -1e-6, flag)
            self.assertLess(abs(result.closure_residual_kg), 1e-5, flag)
            self.assertGreaterEqual(value, bounds[0] * (1 - 1e-6) - 1e-6, flag)     # IPOPT relaxes bounds slightly
            self.assertLessEqual(value, bounds[1] * (1 + 1e-6) + 1e-6, flag)
            self.assertLessEqual(result.mass_takeoff_kg, self.base.mass_takeoff_kg + 0.5, flag)

    def test_fixed_values_when_not_freed(self):
        d = self.base.design
        self.assertIsNone(d.aspect_ratio_wing)
        self.assertIsNone(d.altitude_cruise_m)
        self.assertIsNone(d.soc_reserve)


class CostObjectiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mass_optimal = solve_halo_sizing(assumptions=scholz)
        cls.cost_optimal = solve_halo_sizing(assumptions=scholz, objective="cost")

    def test_closes(self):
        r = self.cost_optimal
        self.assertGreater(r.min_margin, -1e-6)
        self.assertEqual(r.mass_payload_kg, HaloRequirements().mass_payload_kg)

    def test_each_optimum_wins_its_own_objective(self):
        self.assertLessEqual(self.cost_optimal.cost.per_mission_usd, self.mass_optimal.cost.per_mission_usd + 1e-3)
        self.assertGreaterEqual(self.cost_optimal.mass_takeoff_kg, self.mass_optimal.mass_takeoff_kg - 1e-3)

    def test_cost_items_are_positive(self):
        c = self.cost_optimal.cost
        for item in (c.capital_usd, c.fuel_usd, c.battery_usd, c.time_usd):
            self.assertGreater(item, 0.0)
        # The cost optimum may recharge the pack to the take-off SOC in flight: ground electricity is then zero.
        self.assertGreaterEqual(c.electricity_usd, -1e-6)
        self.assertAlmostEqual(c.per_mission_usd, c.capital_usd + c.fuel_usd + c.electricity_usd + c.battery_usd
                               + c.time_usd)


class EnumerationAndSensitivityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = enumerate_architectures({"battery_model": ("ecm", "constant")}, assumptions=scholz)
        cls.study = sensitivity_study(assumptions=scholz, base=cls.cases[0].result, fraction=0.1,
                                      inputs=(UncertainInput("fittings drag area", "assumptions",
                                                             "drag_area_misc_buildup_m2"),))

    def test_enumeration(self):
        self.assertEqual([c.settings for c in self.cases],
                         [(("battery_model", "ecm"),), (("battery_model", "constant"),)])
        for case in self.cases:
            self.assertTrue(case.converged, case.message)
            self.assertGreater(case.result.min_margin, -1e-6)
        # Plan 022: the end-of-life equivalent-circuit pack is heavier than the constant-OCV battery.
        self.assertGreater(self.cases[0].result.mass_takeoff_kg, self.cases[1].result.mass_takeoff_kg + 10.0)

    def test_sensitivity_sign(self):
        """More fittings drag needs a heavier aircraft; less, a lighter one."""
        ((label, low, high),) = self.study.tornado()
        self.assertEqual(label, "fittings drag area")
        self.assertLess(low, 0.0)
        self.assertGreater(high, 0.0)


class ReferenceStartTests(unittest.TestCase):
    """The plan 027 reference (AeroBuildup, ECM, AFDD wing, no thermal model) from cold: the Scholz-aero start,
    unchanged at 13,639 lb."""

    @classmethod
    def setUpClass(cls):
        cls.reference = solve_halo_sizing(assumptions=reference)

    def test_reference_unchanged_and_recorded(self):
        self.assertAlmostEqual(self.reference.mass_takeoff_kg / u.lbm, 13639, delta=15)
        self.assertEqual(self.reference.start.label, "scholz_aero")
        self.assertTrue(self.reference.start.attempts[0].success)
        self.assertIsNotNone(self.reference.cost)


if __name__ == "__main__":
    unittest.main()
