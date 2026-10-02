import unittest

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.core.margins import margin_report
from aircraft_closure.performance.flight_point import (FlightCondition, acceleration_gravity_m_s2,
                                                       build_flight_point)
from aircraft_closure.powertrain.topologies import SeriesHybridSizing, build_series_hybrid_from_sizing
from aircraft_closure.requirements.capability import RequirementSet
from examples.aircraft_mass_closure import build_reference_aircraft
from examples.series_hybrid_point import build_reference_topology

aircraft = build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92)
aero = SimpleAerodynamics()
mass_kg = 1553.0


def solve_point(condition, hybridization=None):
    opti = asb.Opti()
    point = build_flight_point(opti, aircraft, aero, condition, mass_kg, hybridization)
    opti.minimize(point.fuel_flow_kg_s * 100)
    return point, opti.solve(verbose=False)


class SizingBuilderTests(unittest.TestCase):
    def test_default_sizing_reproduces_reference_masses(self):
        sized = build_series_hybrid_from_sizing(SeriesHybridSizing())
        reference = build_reference_topology(4)
        for name, instance in sized.instances.items():
            self.assertAlmostEqual(float(instance.count * instance.component.get_mass()),
                                   float(reference.instances[name].count * reference.instances[name].component.get_mass()))

    def test_rubber_ratios(self):
        motor = build_series_hybrid_from_sizing(SeriesHybridSizing(torque_peak_motor_Nm=300.0)).instances["motor"].component
        self.assertEqual((motor.max_torque_Nm, motor.power_rated_W, motor.max_speed_rad_s), (750.0, 150000.0, 1000.0))


class FlightPointTests(unittest.TestCase):
    def test_hover_thrust_and_power_chain(self):
        point, s = solve_point(FlightCondition(mode="hover", altitude_m=0.0, thrust_to_weight=1.0, label="hover"))
        weight_N = mass_kg * acceleration_gravity_m_s2
        self.assertAlmostEqual(float(s.value(4 * point.thrust_per_rotor_N)), weight_N, places=6)
        rotor = aircraft.powertrain.topology.instances["propulsor"].component
        expected = rotor.evaluate(0, asb.Atmosphere(altitude=0), thrust_N=weight_N / 4).shaft_power_W
        self.assertAlmostEqual(float(s.value(point.power_shaft_rotor_W)), float(expected), places=4)
        # Motor shaft power = rotor shaft power / gearbox efficiency.
        self.assertAlmostEqual(float(s.value(point.speed_motor_rad_s * point.torque_motor_Nm)),
                               float(expected) / 0.97, places=3)

    def test_airplane_force_balance(self):
        condition = FlightCondition(velocity_m_s=50.0, altitude_m=1000.0, climb_rate_m_s=5.0, label="climb")
        point, s = solve_point(condition)
        weight_N = mass_kg * acceleration_gravity_m_s2
        sin_gamma = 5.0 / 50.0
        self.assertAlmostEqual(float(s.value(point.aero.lift_N)), weight_N * np.sqrt(1 - sin_gamma**2), places=5)
        self.assertAlmostEqual(float(s.value(4 * point.thrust_per_rotor_N)),
                               float(s.value(point.aero.drag_N)) + weight_N * sin_gamma, places=5)

    def test_bus_split(self):
        point, s = solve_point(FlightCondition(velocity_m_s=60.0, label="cruise"), hybridization=0.3)
        demand_W = float(s.value(point.power_electric_motors_W))
        self.assertAlmostEqual(float(s.value(point.power_battery_W)), 0.3 * demand_W, places=4)
        self.assertAlmostEqual(float(s.value(point.generator.power_electric_W)), 0.7 * demand_W, places=4)
        engine = aircraft.powertrain.topology.instances["turboshaft"].component
        self.assertAlmostEqual(float(s.value(point.fuel_flow_kg_s)),
                               float(s.value(engine.evaluate(point.generator.power_shaft_W).fuel_flow_kg_s)), places=12)

    def test_climb_needs_more_power_than_level(self):
        level, s_level = solve_point(FlightCondition(velocity_m_s=50.0, label="level"), hybridization=0.0)
        climb, s_climb = solve_point(FlightCondition(velocity_m_s=50.0, climb_rate_m_s=5.0, label="climb"),
                                     hybridization=0.0)
        self.assertGreater(float(s_climb.value(climb.power_shaft_rotor_W)), float(s_level.value(level.power_shaft_rotor_W)))

    def test_margins_are_labelled_and_reported(self):
        point, s = solve_point(FlightCondition(mode="hover", label="hover"))
        report = margin_report(point.margins, s.value)
        self.assertTrue(all(entry.label.startswith("hover: ") for entry in report))

    def test_unknown_mode_rejected(self):
        with self.assertRaises(ValueError):
            FlightCondition(mode="transition")


class RequirementTests(unittest.TestCase):
    def test_requirement_set_conditions(self):
        conditions = RequirementSet().flight_conditions()
        self.assertEqual([c.label for c in conditions], ["hover", "climb", "max_speed", "ceiling"])
        self.assertIsNone(conditions[0].hybridization_electric)
        self.assertTrue(all(c.hybridization_electric == 0.0 for c in conditions[1:]))


if __name__ == "__main__":
    unittest.main()
