import unittest

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import (ClimbSegment, CruiseSegment, DescentSegment, HoverSegment,
                                               LoiterSegment, ground_distance_m)
from examples.aircraft_mass_closure import build_reference_aircraft

aircraft = build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92)
aero = SimpleAerodynamics()
battery = aircraft.powertrain.topology.instances["battery"].component


def fly(segments, mass_kg=1550.0, soc=0.9):
    opti = asb.Opti()
    flown = build_mission(opti, aircraft, aero, Mission(tuple(segments)), mass_kg, soc)
    opti.minimize(flown.mass_fuel_burnt_kg)
    return flown, opti.solve(verbose=False)


class SegmentTests(unittest.TestCase):
    def test_durations_and_distances(self):
        self.assertEqual(ClimbSegment(0, 1000, 4.0).duration_s(), 250.0)
        self.assertEqual(DescentSegment(1000, 0, 2.5).duration_s(), 400.0)
        self.assertEqual(CruiseSegment(90000.0, velocity_m_s=60.0).duration_s(), 1500.0)
        self.assertEqual(ground_distance_m(HoverSegment()), 0.0)
        self.assertAlmostEqual(float(ground_distance_m(ClimbSegment(0, 1000, 3.0, 50.0))),
                               np.sqrt(50.0**2 - 9.0) * 1000 / 3.0)

    def test_mean_altitude_and_signs(self):
        condition = DescentSegment(1000, 200, 2.0).flight_condition(0.8)
        self.assertEqual(condition.altitude_m, 600)
        self.assertEqual(condition.climb_rate_m_s, -2.0)
        self.assertEqual(HoverSegment().flight_condition(0.8).mode, "hover")


class MissionTests(unittest.TestCase):
    def test_cruise_fuel_is_flow_times_time(self):
        flown, s = fly([CruiseSegment(60000.0, 1000.0, 60.0, 0.0)])
        segment = flown.segments[0]
        self.assertAlmostEqual(float(s.value(segment.mass_fuel_burnt_kg)),
                               float(s.value(segment.point.fuel_flow_kg_s)) * 1000.0, places=9)

    def test_mass_chain_and_soc_chain(self):
        flown, s = fly([HoverSegment(60.0, 0.0, 0.6), CruiseSegment(30000.0, 1000.0, 60.0, 0.2),
                        LoiterSegment(600.0, 1000.0, 45.0, 0.0)])
        mass_kg, soc = 1550.0, 0.9
        for segment in flown.segments:
            self.assertAlmostEqual(float(s.value(segment.mass_start_kg)), mass_kg, places=6)
            self.assertAlmostEqual(float(s.value(segment.soc_start)), soc, places=9)
            mass_kg -= float(s.value(segment.mass_fuel_burnt_kg))
            soc -= float(s.value(segment.energy_battery_chemical_J)) / battery.energy_capacity_J
        self.assertAlmostEqual(float(s.value(flown.mass_end_kg)), mass_kg, places=6)
        self.assertAlmostEqual(float(s.value(flown.soc_end)), soc, places=9)

    def test_turbo_only_mission_keeps_soc(self):
        flown, s = fly([CruiseSegment(20000.0, 1000.0, 60.0, 0.0), LoiterSegment(300.0, 1000.0, 45.0, 0.0)])
        self.assertAlmostEqual(float(s.value(flown.soc_end)), 0.9, places=9)

    def test_battery_energy_includes_joule_losses(self):
        flown, s = fly([HoverSegment(60.0, 0.0, 1.0)])
        segment = flown.segments[0]
        self.assertGreater(float(s.value(segment.energy_battery_chemical_J)),
                           float(s.value(segment.energy_battery_terminal_J)))

    def test_heavier_aircraft_burns_more(self):
        light, s_light = fly([CruiseSegment(30000.0, 1000.0, 60.0, 0.0)], mass_kg=1400.0)
        heavy, s_heavy = fly([CruiseSegment(30000.0, 1000.0, 60.0, 0.0)], mass_kg=1700.0)
        self.assertGreater(float(s_heavy.value(heavy.mass_fuel_burnt_kg)), float(s_light.value(light.mass_fuel_burnt_kg)))


if __name__ == "__main__":
    unittest.main()
