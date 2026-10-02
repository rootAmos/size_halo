import unittest
from dataclasses import replace

import aerosandbox as asb
from aerosandbox.library.power_turboshaft import power_turboshaft

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, build_halo_aircraft, solve_halo_sizing,
                                  soc_emergency_floor, soc_minimum)


class HaloSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = solve_halo_sizing()
        cls.better_rotor = solve_halo_sizing(assumptions=replace(HaloAssumptions(), figure_of_merit=0.75),
                                             initial=cls.base)
        cls.heavy_payload = solve_halo_sizing(requirements=replace(HaloRequirements(), mass_payload_kg=1200.0),
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
        self.assertTrue(all(-1e-6 <= s["hybridization"] <= 1 + 1e-6 for s in self.base.segments))
        self.assertIn("soc_after_engine_out_hover", self.base.binding)

    def test_sensitivity_trends(self):
        self.assertLess(self.better_rotor.mass_takeoff_kg, self.base.mass_takeoff_kg)
        self.assertGreater(self.heavy_payload.mass_takeoff_kg, self.base.mass_takeoff_kg)


if __name__ == "__main__":
    unittest.main()
