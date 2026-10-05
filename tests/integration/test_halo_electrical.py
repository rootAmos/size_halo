"""Tier 15 / plan 023: Halo-class sizing with the electrical layer (off by default)."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_for_bus_voltage, assumptions_tier15,
                                  build_halo_aircraft, bus_voltage_window, solve_halo_sizing)
from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point

# Plan 035 changed the defaults; these tests reproduce Tier 15 on the plan 030 settings.
from functools import partial  # noqa: E402
from examples.halo_sizing import pre_plan035  # noqa: E402
HaloAssumptions = partial(HaloAssumptions, **pre_plan035)
build_halo_aircraft = partial(build_halo_aircraft, assumptions=HaloAssumptions())
solve_halo_sizing = partial(solve_halo_sizing, assumptions=HaloAssumptions())

electrical_names = ("inverter_motor", "inverter_generator", "cable_motor", "cable_generator", "cable_battery",
                    "protection_motor", "protection_generator", "protection_battery")


class HaloElectricalLayerTests(unittest.TestCase):
    """On the Scholz hand-check aerodynamics (within about 1 % in mass of the AeroBuildup reference, and fast)."""

    @classmethod
    def setUpClass(cls):
        cls.assumptions = replace(assumptions_tier15, aerodynamics_model="scholz")
        cls.start = solve_halo_sizing(assumptions=replace(cls.assumptions, battery_model="constant",
                                                          electrical_layer=False))
        cls.off = solve_halo_sizing(assumptions=replace(cls.assumptions, electrical_layer=False), initial=cls.start)
        cls.on = solve_halo_sizing(assumptions=cls.assumptions, initial=cls.start)

    def test_flag_defaults_off(self):
        self.assertFalse(HaloAssumptions().electrical_layer)
        aircraft = build_halo_aircraft(self.start.design)
        self.assertFalse(any(name in aircraft.powertrain.topology.instances for name in electrical_names))

    def test_closes_at_900_kg_with_explicit_items(self):
        r = self.on
        self.assertEqual(r.mass_payload_kg, 900.0)
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-4)
        masses = dict(r.powertrain_masses_kg)
        for name in electrical_names:
            self.assertGreater(masses[name], 0.0, name)
        # The layer adds mass and losses: the aircraft is heavier than without it, by more than the items alone.
        electrical_kg = sum(masses[name] for name in electrical_names)
        self.assertGreater(r.mass_takeoff_kg - self.off.mass_takeoff_kg, electrical_kg - 1e-6)

    def test_mission_losses_and_bus_voltage(self):
        window = bus_voltage_window(assumptions_tier15)
        for segment in self.on.segments:
            self.assertGreater(segment["power_loss_electrical_W"], 0.0)
            self.assertGreater(segment["power_loss_inverters_W"], segment["power_loss_cables_W"])
            self.assertGreater(segment["voltage_bus_V"], 0.9 * window.voltage_min_V)
            self.assertLess(segment["voltage_bus_V"], window.voltage_max_V)
            # Losses are a few per cent of the rotor power.
            self.assertLess(segment["power_loss_electrical_W"], 0.08 * segment["power_rotors_W"])

    def test_flight_point_energy_balance(self):
        """Sources minus sinks at the machine terminals equal the electrical-layer losses (exact identity)."""
        aircraft = build_halo_aircraft(self.on.design, HaloRequirements(), assumptions=self.assumptions)
        opti = asb.Opti()
        point = build_flight_point(opti, aircraft, SimpleAerodynamics(), FlightCondition(
            mode="airplane", velocity_m_s=100.0, altitude_m=3000.0, soc=0.6, hybridization_electric=0.2),
            self.on.mass_takeoff_kg)
        solution = opti.solve(verbose=False)
        value = lambda x: float(solution.value(x))
        e = point.electrical
        terminals_W = (value(point.battery.power_electric_W) + 2 * value(point.generator.power_electric_W)
                       - 2 * value(e.inverter_motor.power_ac_W))
        self.assertAlmostEqual(terminals_W / value(e.power_loss_total_W), 1.0, delta=1e-6)
        # Tier 19 heat loads: per-instance losses sum to the layer total.
        self.assertAlmostEqual(sum(value(load.total_W()) for load in e.heat_loads)
                               / value(e.power_loss_total_W), 1.0, delta=1e-9)
        self.assertTrue(set(electrical_names) <= {load.source for load in point.heat_loads})
        labels = [m.label for m in point.margins]
        self.assertTrue(any("cable_motor partial_discharge_V" in label for label in labels))
        self.assertTrue(any("inverter_motor max_voltage_V" in label for label in labels))

    def test_bus_voltage_options_couple_the_pack(self):
        low, high = assumptions_for_bus_voltage(HaloAssumptions(), 540.0), assumptions_for_bus_voltage(
            HaloAssumptions(), 1000.0)
        self.assertEqual(low.count_series_battery, 150)
        self.assertEqual(high.count_series_battery, 278)
        self.assertEqual(low.voltage_blocking_inverter_V, 1200.0)
        self.assertEqual(high.voltage_blocking_inverter_V, 1700.0)     # 1,168 V pack > 900 V derated 1,200 V class
        boosted = assumptions_for_bus_voltage(HaloAssumptions(), 1000.0, dcdc=True)
        self.assertEqual(boosted.count_series_battery, HaloAssumptions().count_series_battery)
        self.assertEqual(bus_voltage_window(boosted).voltage_min_V, 1000.0)


if __name__ == "__main__":
    unittest.main()
