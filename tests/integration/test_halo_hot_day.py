"""Tier 16: hot/high OGE hover at the destination after the mission (Halo sizing)."""
import unittest
from dataclasses import replace

import aerosandbox.tools.units as u

from examples.halo_sizing import (assumptions_tier16, requirements_tier12b, requirements_tier16, soc_emergency_floor,
                                  soc_minimum, solve_halo_sizing)


class HaloHotDayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # The Tier 16 reference (plan 020): 900 kg, constant battery; 4,000 ft / 95 F after the mission.
        cls.hot = solve_halo_sizing(requirements_tier16, assumptions_tier16)
        cls.standard = solve_halo_sizing(requirements_tier12b, assumptions_tier16, initial=cls.hot)
        cls.hovers = {h.label: h for h in cls.hot.hovers}

    def test_hot_hover_point_is_built_at_the_mission_end_state(self):
        h = self.hovers["hot-day hover"]
        self.assertAlmostEqual(h.altitude_m, 4000 * u.foot, places=6)
        self.assertAlmostEqual(h.temperature_offset_K, 27.66, delta=0.01)
        self.assertAlmostEqual(h.soc, self.hot.soc_end, places=9)
        self.assertAlmostEqual(h.mass_kg, self.hot.mass_takeoff_kg - self.hot.mass_fuel_burnt_kg, places=4)
        self.assertNotIn("hot-day hover", {g.label for g in self.standard.hovers})

    def test_every_constraint_satisfied_including_hot_hover(self):
        self.assertGreater(self.hot.min_margin, -1e-6)
        self.assertGreaterEqual(self.hot.soc_after_hover_hot, soc_emergency_floor - 1e-6)
        self.assertGreaterEqual(self.hot.soc_end, soc_minimum - 1e-6)

    def test_battery_covers_more_of_the_hot_hover(self):
        """Turbine-limited battery share: hot/high > standard-day 4,000 ft hover (at MTOM) > landing hover."""
        hot = self.hovers["hot-day hover"]
        self.assertGreater(hot.battery_share_min, self.hovers["hover"].battery_share_min + 0.05)
        self.assertGreater(hot.battery_share_min, self.hovers["landing hover"].battery_share_min)
        self.assertGreater(hot.battery_share_min, 0.1)
        self.assertGreaterEqual(hot.hybridization, hot.battery_share_min - 1e-6)
        self.assertLess(hot.power_available_turboshafts_W, 0.8 * self.hovers["hover"].power_available_turboshafts_W)

    def test_four_k_ninety_five_does_not_resize_the_reference(self):
        """Not binding at 4k/95: engine-out hover sizes battery power, the MTOM 4,000 ft hover sizes the motors."""
        self.assertAlmostEqual(self.hot.mass_takeoff_kg / self.standard.mass_takeoff_kg, 1.0, delta=1e-3)
        self.assertFalse(any("hot" in b for b in self.hot.binding))

    def test_hotter_higher_destination_grows_the_aircraft(self):
        """The hot hover starts to bind above ~6,000 ft / 95 F (Tier 13 machines; ~5,750 ft with Tier 12b machines);
        no closed design is found at 8,000 ft / 95 F."""
        previous = self.hot
        for altitude_ft in (5000, 6000, 6500, 7000):
            previous = solve_halo_sizing(requirements=replace(requirements_tier16, altitude_hover_hot_m=altitude_ft
                                                              * u.foot), assumptions=assumptions_tier16, initial=previous)
        self.assertGreater(previous.mass_takeoff_kg, self.hot.mass_takeoff_kg + 20.0)


if __name__ == "__main__":
    unittest.main()
