"""Plan 042: flight points with the ε-NTU ram-air cooler (air-flow unknown, duct residual, hover fan, drag sign)."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from aircraft_closure.powertrain.components.compact_heat_exchanger import OffsetStripFinCore
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct
from aircraft_closure.vehicle.powertrain_installation import InstalledCooling
from tests.performance.test_flight_point_thermal import aero, base, cruise, hover, mass_kg, thermal_aircraft

exchanger = NtuRamAirHeatExchanger(core=OffsetStripFinCore(count_wide=400.0, count_long=10.0, count_tall=25.0),
                                   duct=RamAirDuct(area_nozzle_m2=0.08), power_rated_fan_W=20e3)


def ntu_aircraft(machines=False, cooler=exchanger):
    aircraft = thermal_aircraft(machines=machines)
    return replace(aircraft, powertrain=replace(aircraft.powertrain, cooling=InstalledCooling(cooler, x_m=3.0)))


def solve(aircraft, condition, hybridization=0.0, **kwargs):
    from aircraft_closure.performance.flight_point import build_flight_point
    opti = asb.Opti()
    point = build_flight_point(opti, aircraft, aero, condition, mass_kg, hybridization, **kwargs)
    opti.subject_to([m.value >= 0 for m in point.thermal.margins])
    opti.minimize(point.fuel_flow_kg_s * 100)
    return point, opti.solve(verbose=False)


class NtuCruiseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plain, cls.s0 = solve(base, cruise)
        cls.point, cls.s = solve(ntu_aircraft(), cruise)

    def test_duct_flow_closes_and_core_rejects_the_heat(self):
        c, s = self.point.thermal.cooling, self.s
        self.assertAlmostEqual(float(s.value(c.residual_mass_flow_kg_s)), 0.0, delta=1e-7)
        self.assertGreater(float(s.value(c.mass_flow_air_kg_s)), 0.1)
        labels = [m.label for m in self.point.margins]
        self.assertIn("cruise: heat_exchanger heat_rejection", labels)
        self.assertIn("cruise: heat_exchanger power_fan_W", labels)
        self.assertNotIn("cruise: heat_exchanger power_heat_W", labels)
        heat_W = float(s.value(self.point.thermal.power_heat_end_W))
        capacity_W = float(s.value(exchanger.power_heat_capacity_W(heat_W, c)))
        self.assertGreaterEqual(capacity_W, heat_W * (1 - 1e-9))

    def test_cooling_drag_is_the_duct_force_in_the_thrust(self):
        c, s = self.point.thermal.cooling, self.s
        drag_N = float(s.value(c.drag_N))
        thrust_extra_N = float(s.value(4 * self.point.thrust_per_rotor_N) - self.s0.value(4 * self.plain.thrust_per_rotor_N))
        self.assertAlmostEqual(thrust_extra_N, drag_N, delta=1e-3 * abs(drag_N) + 1e-6)
        self.assertEqual(float(s.value(c.power_fan_W)), 0.0)


class NtuHoverTests(unittest.TestCase):
    def test_fan_power_joins_bus_demand_and_no_drag(self):
        point, s = solve(ntu_aircraft(), hover)
        c = point.thermal.cooling
        fan_W = float(s.value(c.power_fan_W))
        self.assertGreater(fan_W, 0.0)
        self.assertAlmostEqual(float(s.value(c.residual_mass_flow_kg_s)), 0.0, delta=1e-7)
        supplied_W = float(s.value(point.generator.power_electric_W))
        self.assertAlmostEqual(supplied_W, float(s.value(point.power_electric_motors_W)) + fan_W, places=3)
        margin = next(m for m in point.margins if m.label == "hover: heat_exchanger heat_rejection")
        self.assertGreaterEqual(float(s.value(margin.value)), -1e-8)

    def test_mass_in_powertrain(self):
        aircraft = ntu_aircraft()
        items = dict((i.instance_name, i.mass_properties.mass) for i in aircraft.powertrain.get_instance_mass_properties())
        self.assertAlmostEqual(items["heat_exchanger"], exchanger.get_mass())


if __name__ == "__main__":
    unittest.main()
