"""Tier 19 (plan 028): heat loads, cooling drag, fan power and temperatures in flight points and missions."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import CruiseSegment, HoverSegment
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point
from aircraft_closure.powertrain.components.heat_exchanger import RamAirHeatExchanger
from aircraft_closure.powertrain.components.thermal import LumpedThermalModel
from aircraft_closure.powertrain.topologies import build_series_hybrid
from aircraft_closure.thermal.heat import coolant_temperatures_C, thermal_parameters, total_heat_W
from aircraft_closure.vehicle.powertrain_installation import InstalledCooling
from examples.aircraft_mass_closure import build_reference_aircraft

base = build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92)
aero = SimpleAerodynamics()
mass_kg = 1553.0
thermal = LumpedThermalModel()


def thermal_aircraft(cooling=True, machines=True, rating_W=100000.0):
    instances = {name: i.component for name, i in base.powertrain.topology.instances.items()}
    motor, generator = instances["motor"], instances["generator"]
    if machines:
        motor, generator = replace(motor, thermal_model=thermal), replace(generator, thermal_model=thermal)
    topology = build_series_hybrid(motor, generator, instances["battery"], instances["turboshaft"],
                                   instances["gearbox"], instances["propulsor"], count_rotors=4)
    installed = InstalledCooling(RamAirHeatExchanger(power_rated_W=rating_W), x_m=3.0) if cooling else None
    return replace(base, powertrain=replace(base.powertrain, topology=topology, cooling=installed))


def solve(aircraft, condition, hybridization=0.0, **kwargs):
    opti = asb.Opti()
    point = build_flight_point(opti, aircraft, aero, condition, mass_kg, hybridization, **kwargs)
    opti.minimize(point.fuel_flow_kg_s * 100)
    return point, opti.solve(verbose=False)


cruise = FlightCondition(velocity_m_s=60.0, altitude_m=1000.0, label="cruise")
hover = FlightCondition(mode="hover", altitude_m=0.0, label="hover")


class HeatLoadPointTests(unittest.TestCase):
    def test_heat_loads_match_component_losses(self):
        point, s = solve(base, cruise)
        loads = {load.source: load for load in point.heat_loads}
        self.assertEqual(set(loads), {"motor", "gearbox", "generator", "battery"})
        self.assertEqual(loads["motor"].count, 4)
        motor_loss_W = s.value(point.power_electric_motors_W / 4 - point.speed_motor_rad_s * point.torque_motor_Nm)
        self.assertAlmostEqual(float(s.value(loads["motor"].power_W)), float(motor_loss_W), places=6)
        gearbox_loss_W = s.value(point.speed_motor_rad_s * point.torque_motor_Nm * 0.03)
        self.assertAlmostEqual(float(s.value(loads["gearbox"].power_W)), float(gearbox_loss_W), places=6)
        self.assertAlmostEqual(float(s.value(loads["generator"].power_W)),
                               float(s.value(point.generator.power_shaft_W - point.generator.power_electric_W)), places=6)
        for load in point.heat_loads:
            self.assertGreaterEqual(float(s.value(load.power_W)), 0.0)

    def test_without_cooling_point_is_unchanged(self):
        """No cooling and no thermal machines: identical solution, no thermal margins (the reference path)."""
        point, s = solve(base, cruise)
        self.assertIsNone(point.thermal.cooling)
        self.assertEqual(point.thermal.margins, ())
        self.assertEqual(point.thermal.power_heat_W, 0.0)


class CoolingCouplingTests(unittest.TestCase):
    def test_cooling_drag_raises_thrust_and_matches_exchanger(self):
        plain, s0 = solve(base, cruise)
        cooled, s1 = solve(thermal_aircraft(machines=False), cruise)
        drag_N = float(s1.value(cooled.thermal.cooling.drag_N))
        self.assertGreater(drag_N, 0.0)
        thrust_extra_N = float(s1.value(4 * cooled.thrust_per_rotor_N) - s0.value(4 * plain.thrust_per_rotor_N))
        self.assertAlmostEqual(thrust_extra_N / drag_N, 1.0, places=4)
        self.assertEqual(float(s1.value(cooled.thermal.cooling.power_fan_W)), 0.0)
        heat_W = float(s1.value(total_heat_W(cooled.heat_loads)))
        self.assertAlmostEqual(float(s1.value(cooled.thermal.power_heat_W)), heat_W, places=6)

    def test_hover_fan_power_joins_bus_demand(self):
        point, s = solve(thermal_aircraft(machines=False), hover, hybridization=0.0)
        fan_W = float(s.value(point.thermal.cooling.power_fan_W))
        self.assertGreater(fan_W, 0.0)
        supplied_W = float(s.value(point.generator.power_electric_W))
        self.assertAlmostEqual(supplied_W, float(s.value(point.power_electric_motors_W)) + fan_W, places=4)
        self.assertEqual(float(s.value(point.thermal.cooling.drag_N)), 0.0)

    def test_rating_margin(self):
        point, s = solve(thermal_aircraft(machines=False, rating_W=100000.0), hover)
        margin = next(m for m in point.margins if m.label == "hover: heat_exchanger power_heat_W")
        required_W = float(s.value(point.thermal.power_heat_equivalent_W))
        self.assertAlmostEqual(float(s.value(margin.value)), 1 - required_W / 100000.0, places=9)

    def test_excluded_sources_are_not_rejected(self):
        aircraft = thermal_aircraft(machines=False)
        aircraft = replace(aircraft, powertrain=replace(aircraft.powertrain, cooling=replace(
            aircraft.powertrain.cooling, sources_excluded=("gearbox",))))
        point, s = solve(aircraft, cruise)
        gearbox_W = float(s.value(next(l for l in point.heat_loads if l.source == "gearbox").total_W()))
        self.assertAlmostEqual(float(s.value(point.thermal.power_heat_W)),
                               float(s.value(total_heat_W(point.heat_loads))) - gearbox_W, places=6)

    def test_heat_exchanger_mass_in_powertrain(self):
        aircraft = thermal_aircraft(rating_W=50000.0)
        items = dict((i.instance_name, i.mass_properties.mass) for i in aircraft.powertrain.get_instance_mass_properties())
        self.assertAlmostEqual(items["heat_exchanger"], 50.0)
        self.assertAlmostEqual(aircraft.powertrain.get_mass_properties().mass - base.powertrain.get_mass_properties().mass,
                               50.0, places=6)


class TemperatureTests(unittest.TestCase):
    def test_steady_point_temperature_and_margin(self):
        point, s = solve(thermal_aircraft(cooling=False), cruise)
        motor = point.thermal.temperatures_end_C["motor"]
        self.assertGreater(float(s.value(motor)), 60.0)
        labels = [m.label for m in point.margins]
        self.assertIn("cruise: motor temperature_C", labels)
        self.assertIn("cruise: generator temperature_C", labels)
        self.assertNotIn("cruise: motor power_shaft_W", labels)

    def test_short_hover_is_cooler_than_steady(self):
        aircraft = thermal_aircraft(cooling=False)
        steady, s0 = solve(aircraft, hover)
        cold = coolant_temperatures_C(aircraft.powertrain)
        short, s1 = solve(aircraft, hover, duration_s=30.0, temperature_start_C=cold)
        self.assertLess(float(s1.value(short.thermal.temperatures_end_C["motor"])),
                        float(s0.value(steady.thermal.temperatures_end_C["motor"])))
        self.assertGreater(float(s1.value(short.thermal.temperatures_end_C["motor"])), 60.0)

    def test_thermal_mass_absorbs_a_cold_start_peak(self):
        """From the coolant temperature the machines pass less heat to the cooler than they lose; steady, equal."""
        aircraft = thermal_aircraft()
        cold = coolant_temperatures_C(aircraft.powertrain)
        point, s = solve(aircraft, hover, duration_s=60.0, temperature_start_C=cold)
        loss = {load.source: load for load in point.heat_loads}
        for name in ("motor", "generator"):
            self.assertLess(float(s.value(point.thermal.power_to_coolant_W[name])), float(s.value(loss[name].power_W)))
        self.assertLess(float(s.value(point.thermal.power_heat_W)), float(s.value(total_heat_W(point.heat_loads))))
        self.assertGreater(float(s.value(point.thermal.power_heat_end_W)), float(s.value(point.thermal.power_heat_W)))
        steady, s1 = solve(aircraft, hover)
        self.assertAlmostEqual(float(s1.value(steady.thermal.power_heat_W)),
                               float(s1.value(total_heat_W(steady.heat_loads))), places=6)

    def test_mission_chains_temperatures(self):
        aircraft = thermal_aircraft()
        mission = Mission((HoverSegment(60.0, 0.0, 0.0, "hover"), CruiseSegment(30000.0, 1000.0, 60.0, 0.0, "cruise")))
        opti = asb.Opti()
        flown = build_mission(opti, aircraft, aero, mission, mass_kg, 0.9, subsegments=(1, 2), thermal_start="coolant")
        opti.minimize(flown.mass_fuel_burnt_kg)
        s = opti.solve(verbose=False)
        points = [flown.segments[0]] + list(flown.segments[1].subsegments)
        start_C = coolant_temperatures_C(aircraft.powertrain)
        for p in points:
            self.assertEqual(set(p.point.thermal.temperatures_end_C), {"motor", "generator"})
            # End temperature is the closed form from the previous end (continuity).
            heat_W = {load.source: load.power_W for load in p.point.heat_loads}
            component = aircraft.powertrain.topology.instances["motor"].component
            q = thermal_parameters(component)
            expected = thermal.temperature_end_C(float(s.value(heat_W["motor"])), float(s.value(p.duration_s)),
                                                 float(q.capacity_J_K), float(q.resistance_K_W),
                                                 float(s.value(start_C["motor"])))
            self.assertAlmostEqual(float(s.value(p.point.thermal.temperatures_end_C["motor"])), expected, places=6)
            start_C = p.point.thermal.temperatures_end_C
        self.assertIs(flown.temperatures_end_C, points[-1].point.thermal.temperatures_end_C)

    def test_mission_thermal_start_options(self):
        aircraft = thermal_aircraft(cooling=False)
        mission = Mission((HoverSegment(60.0, 0.0, 0.0, "hover"),))
        opti = asb.Opti()
        steady = build_mission(opti, aircraft, aero, mission, mass_kg, 0.9)
        self.assertIsNone(steady.temperatures_end_C)
        given = build_mission(opti, aircraft, aero, mission, mass_kg, 0.9, thermal_start={"motor": 80.0, "generator": 70.0})
        self.assertEqual(set(given.temperatures_end_C), {"motor", "generator"})
        with self.assertRaises(ValueError):
            build_mission(opti, aircraft, aero, mission, mass_kg, 0.9, thermal_start="hot")


if __name__ == "__main__":
    unittest.main()
