"""Tier 15 / plan 023: Halo-class sizing with the electrical layer (off by default)."""
import unittest
from dataclasses import replace

import aerosandbox as asb

from examples.halo_sizing import (HaloAssumptions, HaloRequirements, assumptions_for_bus_voltage, assumptions_tier15,
                                  build_halo_aircraft, bus_voltage_window, solve_halo_max_payload,
                                  solve_halo_sizing)
from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point

electrical_names = ("inverter_motor", "inverter_generator", "cable_motor", "cable_generator", "cable_battery",
                    "protection_motor", "protection_generator", "protection_battery")


class HaloElectricalLayerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.start = solve_halo_sizing(assumptions=replace(HaloAssumptions(), battery_model="constant"))
        cls.on = solve_halo_max_payload(assumptions=assumptions_tier15, initial=cls.start)

    def test_flag_defaults_off(self):
        self.assertFalse(HaloAssumptions().electrical_layer)
        aircraft = build_halo_aircraft(self.start.design)
        self.assertFalse(any(name in aircraft.powertrain.topology.instances for name in electrical_names))

    def test_max_payload_closes_with_explicit_items(self):
        r = self.on
        self.assertGreater(r.min_margin, -1e-6)
        self.assertLess(abs(r.closure_residual_kg), 1e-4)
        masses = dict(r.powertrain_masses_kg)
        for name in electrical_names:
            self.assertGreater(masses[name], 0.0, name)
        # The layer costs payload against the 959 kg flag-off maximum (plan 024), but the aircraft still closes.
        self.assertGreater(r.mass_payload_kg, 300.0)
        self.assertLess(r.mass_payload_kg, 959.0)

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
        aircraft = build_halo_aircraft(self.on.design, HaloRequirements(mass_payload_kg=self.on.mass_payload_kg),
                                       assumptions_tier15)
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
        self.assertAlmostEqual(sum(value(source.power_loss_W) for source in e.loss_sources)
                               / value(e.power_loss_total_W), 1.0, delta=1e-9)
        self.assertEqual({source.instance_name for source in e.loss_sources},
                         set(electrical_names))
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
