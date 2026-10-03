"""Tier 16: flight conditions, requirements and segments carry an ISA temperature offset."""
import unittest

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.segments import (ClimbSegment, CruiseSegment, DescentSegment, HoverSegment,
                                               LoiterSegment)
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point
from aircraft_closure.requirements.capability import (CeilingRequirement, ClimbRequirement, HoverRequirement,
                                                      SpeedRequirement)
from examples.aircraft_mass_closure import build_reference_aircraft

aircraft = build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92)
aero = SimpleAerodynamics()
mass_kg = 1553.0


def solve_point(condition):
    opti = asb.Opti()
    point = build_flight_point(opti, aircraft, aero, condition, mass_kg)
    opti.minimize(point.fuel_flow_kg_s * 100)
    return point, opti.solve(verbose=False)


class HotDayConditionTests(unittest.TestCase):
    def test_default_is_standard_day(self):
        self.assertEqual(FlightCondition().temperature_offset_K, 0.0)
        for item in (HoverRequirement(), ClimbRequirement(), SpeedRequirement(), CeilingRequirement()):
            self.assertEqual(item.flight_condition().temperature_offset_K, 0.0)
        for segment in (HoverSegment(), ClimbSegment(), CruiseSegment(), LoiterSegment(), DescentSegment()):
            self.assertEqual(segment.flight_condition(0.5).temperature_offset_K, 0.0)

    def test_requirements_and_segments_pass_the_offset(self):
        for item in (HoverRequirement(temperature_offset_K=20.0), ClimbRequirement(temperature_offset_K=20.0),
                     SpeedRequirement(temperature_offset_K=20.0), CeilingRequirement(temperature_offset_K=20.0)):
            self.assertEqual(item.flight_condition().temperature_offset_K, 20.0)
        for segment in (HoverSegment(temperature_offset_K=20.0), ClimbSegment(temperature_offset_K=20.0),
                        CruiseSegment(temperature_offset_K=20.0), LoiterSegment(temperature_offset_K=20.0),
                        DescentSegment(temperature_offset_K=20.0)):
            self.assertEqual(segment.flight_condition(0.5).temperature_offset_K, 20.0)
        # Positional construction is unchanged (the offset is the last field).
        self.assertEqual(HoverSegment(60.0, 0.0, None, "landing").label, "landing")

    def test_hot_hover_needs_more_rotor_power(self):
        """Less density at the same pressure altitude: induced power ~ 1 / sqrt(rho)."""
        standard, s0 = solve_point(FlightCondition(mode="hover", altitude_m=1000.0, label="hover"))
        hot, s1 = solve_point(FlightCondition(mode="hover", altitude_m=1000.0, temperature_offset_K=25.0,
                                              label="hover"))
        self.assertGreater(float(s1.value(hot.power_shaft_rotor_W)), float(s0.value(standard.power_shaft_rotor_W)))

    def test_zero_offset_airplane_point_unchanged_and_hot_mach_lower(self):
        base = aero.evaluate(aircraft, 70.0, 1000.0, 4.0)
        zero = aero.evaluate(aircraft, 70.0, 1000.0, 4.0, temperature_offset_K=0.0)
        hot = aero.evaluate(aircraft, 70.0, 1000.0, 4.0, temperature_offset_K=25.0)
        self.assertEqual(float(zero.drag_N), float(base.drag_N))
        self.assertLess(float(hot.mach), float(base.mach))
        self.assertLess(float(hot.dynamic_pressure_Pa), float(base.dynamic_pressure_Pa))
        point, s = solve_point(FlightCondition(velocity_m_s=70.0, altitude_m=1000.0, temperature_offset_K=25.0))
        self.assertGreater(float(s.value(point.alpha_deg)), 0.0)


if __name__ == "__main__":
    unittest.main()
