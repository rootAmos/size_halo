import unittest
from dataclasses import replace

import aerosandbox as asb
from aerosandbox.library.power_turboshaft import power_turboshaft

from aircraft_closure.vehicle.condition import StructuralDesignCondition
import aerosandbox.tools.units as u

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_tier11a, assumptions_tier12,
                                  assumptions_tier12b,
                                  build_halo_aircraft, requirements_tier10c, solve_halo_sizing, soc_emergency_floor,
                                  soc_minimum)


class HaloSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = solve_halo_sizing()   # plan 017: fixed 2 x 1,120 hp, 900 kg at 210 kt
        # The figure-of-merit assumption only drives the actuator-disk rotor (Tier 11a baseline).
        cls.better_rotor = solve_halo_sizing(requirements=requirements_tier10c,
                                             assumptions=replace(assumptions_tier11a, figure_of_merit=0.75))
        cls.light_payload = solve_halo_sizing(requirements=replace(HaloRequirements(), mass_payload_kg=800.0),
                                              initial=cls.base)

    def test_constraints_satisfied(self):
        self.assertGreater(self.base.min_margin, -1e-6)
        self.assertLess(abs(self.base.closure_residual_kg), 1e-5)
        self.assertGreaterEqual(self.base.soc_end, soc_minimum - 1e-6)
        self.assertGreaterEqual(self.base.soc_after_engine_out, soc_emergency_floor - 1e-6)
        self.assertGreaterEqual(self.base.static_margin, 0.10 - 1e-6)

    def test_closed_mass_is_a_fixed_point(self):
        aircraft = build_halo_aircraft(self.base.design)
        design = self.base.design
        condition = StructuralDesignCondition(
            mass_design_kg=self.base.mass_takeoff_kg, load_factor_ultimate=HaloAssumptions().load_factor_ultimate,
            velocity_cruise_m_s=self.base.velocity_cruise_m_s, altitude_cruise_m=HaloRequirements().altitude_cruise_m,
            lift_to_drag_cruise=self.base.lift_to_drag_cruise)
        self.assertAlmostEqual(float(aircraft.get_mass(condition)) / self.base.mass_takeoff_kg, 1.0, places=6)
        self.assertGreater(design.mass_fuel_kg, 0)

    def test_turboshaft_obeys_the_aerosandbox_regression(self):
        d = self.base.design
        self.assertAlmostEqual(float(power_turboshaft(d.mass_turboshaft_bare_kg)) / d.power_rated_turboshaft_W, 1.0,
                               places=6)

    def test_rotor_fits_the_span(self):
        a = HaloAssumptions()
        self.assertLessEqual(self.base.radius_rotor_m,
                             (self.base.span_m - a.diameter_fuselage_m) / 2 - a.clearance_rotor_fuselage_m + 1e-6)

    def test_stall_speed_limit(self):
        r = HaloRequirements()
        lift_N = 0.5 * float(asb.Atmosphere(altitude=0).density()) * r.velocity_stall_max_m_s**2 \
            * self.base.design.area_wing_m2 * r.cl_max
        self.assertGreaterEqual(lift_N, self.base.mass_takeoff_kg * 9.80665 * (1 - 1e-6))

    def test_halo_class_weight_band(self):
        """1x,xxx lb class (user's expectation); loose regression band on the reference case."""
        self.assertTrue(5000 < self.base.mass_takeoff_kg < 11000)

    def test_mission_is_flown_and_engine_out_uses_the_battery(self):
        labels = [s["label"] for s in self.base.segments]
        self.assertEqual(labels[0], "take-off hover")
        self.assertEqual(labels[-1], "landing hover")
        self.assertTrue(all(-1 - 1e-6 <= s["hybridization"] <= 1 + 1e-6 for s in self.base.segments))  # < 0: recharge
        self.assertIn("soc_after_engine_out_hover", self.base.binding)

    def test_rotor_slows_in_cruise_and_obeys_its_limits(self):
        """Tier 12: cruise rotor speed below hover for profile-power reasons, inside the JVX validity bound."""
        segments = {s["label"]: s for s in self.base.segments}
        self.assertLess(segments["cruise"]["speed_motor_rad_s"], segments["take-off hover"]["speed_motor_rad_s"])
        d = self.base.design
        speed_rotor = segments["cruise"]["speed_motor_rad_s"] / HaloAssumptions().reduction_ratio
        advance_ratio = self.base.velocity_cruise_m_s / (speed_rotor * self.base.radius_rotor_m)
        self.assertLessEqual(advance_ratio, 0.60 + 1e-5)
        self.assertTrue(0.06 - 1e-9 <= d.solidity <= 0.14 + 1e-9)

    def test_legacy_baselines_reproduce(self):
        tier11a = solve_halo_sizing(requirements=requirements_tier10c, assumptions=assumptions_tier11a)
        tier12 = solve_halo_sizing(requirements=requirements_tier10c, assumptions=assumptions_tier12)
        self.assertAlmostEqual(tier11a.mass_takeoff_kg / u.lbm, 18506, delta=5)
        self.assertAlmostEqual(tier12.mass_takeoff_kg / u.lbm, 17228, delta=5)

    def test_tier12b_baseline_reproduces(self):
        self.assertAlmostEqual(solve_halo_sizing(assumptions=assumptions_tier12b).mass_takeoff_kg / u.lbm, 14877, delta=5)

    def test_torque_sized_machines_and_drive_choices(self):
        """Tier 13: fast geared machines; a step-up gearbox beats a generator on the engine's 1,210 rpm shaft."""
        d = self.base.design
        self.assertIsNotNone(d.reduction_ratio)
        self.assertGreater(d.speed_peak_generator_rad_s, HaloAssumptions().speed_output_turboshaft_rad_s)
        masses = dict(self.base.powertrain_masses_kg)
        self.assertIn("generator_gearbox", masses)
        on_shaft = solve_halo_sizing(assumptions=replace(HaloAssumptions(), generator_step_up=False), objective="payload")
        stepped = solve_halo_sizing(objective="payload", initial=self.base)
        self.assertLess(on_shaft.mass_payload_kg, 0.25 * stepped.mass_payload_kg)

    def test_turboshafts_are_the_fixed_deck_engine(self):
        self.assertEqual(self.base.design.power_rated_turboshaft_W, 1120 * u.hp)
        self.assertIn("max_speed: turboshaft power_shaft_W", self.base.binding)

    def test_battery_supplements_hover_and_is_recharged(self):
        shares = {s["label"]: s["hybridization"] for s in self.base.segments}
        self.assertGreater(shares["take-off hover"], 0.05)
        self.assertGreater(shares["landing hover"], 0.05)
        self.assertTrue(any(share < -0.01 for share in shares.values()))
        self.assertTrue(all(soc_minimum - 1e-6 <= s["soc_end"] <= 0.95 + 1e-6 for s in self.base.segments))

    def test_payload_objective_around_fixed_engines(self):
        best = solve_halo_sizing(objective="payload", initial=self.base)
        self.assertGreaterEqual(best.mass_payload_kg, 900.0 - 1e-3)
        faster = solve_halo_sizing(requirements=replace(HaloRequirements(), velocity_max_m_s=215 * u.knot),
                                   objective="payload", initial=best)
        self.assertLess(faster.mass_payload_kg, best.mass_payload_kg)

    def test_sensitivity_trends(self):
        self.assertLess(self.better_rotor.mass_takeoff_kg / 0.45359237, 18506)  # actuator-disk FM 0.67 baseline
        self.assertLess(self.light_payload.mass_takeoff_kg, self.base.mass_takeoff_kg)


if __name__ == "__main__":
    unittest.main()
