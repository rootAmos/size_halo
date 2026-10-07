"""Tier 21 (plan 025): Halo sized and flown with the AeroBuildup-based aerodynamics."""
import unittest
from dataclasses import replace

import aerosandbox.tools.units as u

from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.scholz import ScholzAerodynamics
from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_plan027, build_halo_aerodynamics,
                                  requirements_plan027, solve_halo_sizing)
from examples.trajectory_optimization import halo_trajectory_case, solve_min_energy_transition

buildup = replace(assumptions_plan027, aerodynamics_model="buildup")    # thermal off (plan 027)


class HaloAerodynamicsSwitchTests(unittest.TestCase):
    def test_default_is_buildup(self):
        """Plan 027 (decided 2026-10-04)."""
        self.assertEqual(HaloAssumptions().aerodynamics_model, "buildup")
        r = HaloRequirements()
        self.assertIsInstance(build_halo_aerodynamics(r, replace(buildup, aerodynamics_model="simple")),
                              SimpleAerodynamics)
        self.assertIsInstance(build_halo_aerodynamics(r, buildup), BuildupAerodynamics)
        self.assertIsNotNone(build_halo_aerodynamics(r, buildup).blown_wing)
        self.assertIsNone(build_halo_aerodynamics(r, replace(buildup, blown_wing=False)).blown_wing)
        self.assertIsInstance(build_halo_aerodynamics(r, replace(buildup, aerodynamics_model="scholz")),
                              ScholzAerodynamics)
        with self.assertRaises(ValueError):
            build_halo_aerodynamics(r, replace(buildup, aerodynamics_model="cfd"))


class HaloBuildupSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sized = solve_halo_sizing(requirements_plan027, assumptions_plan027)    # the plan 027 reference
        cls.scholz = solve_halo_sizing(assumptions=replace(buildup, aerodynamics_model="scholz"))

    def test_closes_with_all_margins(self):
        r = self.sized
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-5)
        self.assertEqual(r.mass_payload_kg, HaloRequirements().mass_payload_kg)

    def test_lighter_than_the_simple_reference(self):
        """The guessed 0.8 m2 miscellaneous area gives way to the component build-up: about 610 lb lighter
        than the 14,247 lb plan 026 reference, with a higher cruise L/D (8.0 with SimpleAerodynamics)."""
        r = self.sized
        self.assertAlmostEqual(r.mass_takeoff_kg / u.lbm, 13639, delta=15)
        self.assertGreater(r.lift_to_drag_cruise, 9.0)

    def test_scholz_hand_check_sizes_within_one_percent(self):
        self.assertAlmostEqual(self.scholz.mass_takeoff_kg / self.sized.mass_takeoff_kg, 1.0, delta=0.01)

    def test_trajectory_solves_with_the_buildup(self):
        case = halo_trajectory_case(self.sized, HaloRequirements(), buildup)
        self.assertIsInstance(case.model.aerodynamics, BuildupAerodynamics)
        transition = solve_min_energy_transition(case)
        self.assertGreater(transition.duration_s, 10.0)
        self.assertLess(transition.duration_s, 40.0)


if __name__ == "__main__":
    unittest.main()
