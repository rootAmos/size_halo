"""Tier 19 (plan 028): the Halo reference with the thermal model switched on."""
import unittest
from dataclasses import replace

from examples.halo_sizing import (HaloAssumptions, HaloDesign, assumptions_plan030, build_halo_aircraft,
                                  requirements_plan030, solve_halo_sizing)

# Plan 035 changed the defaults; these tests reproduce their tier on the plan 030 settings.
from functools import partial  # noqa: E402
from examples.halo_sizing import pre_layout, pre_plan035  # noqa: E402
HaloAssumptions = partial(HaloAssumptions, **pre_plan035, **pre_layout)
build_halo_aircraft = partial(build_halo_aircraft, assumptions=HaloAssumptions())
solve_halo_sizing = partial(solve_halo_sizing, assumptions=HaloAssumptions())

thermal = HaloAssumptions(thermal_model=True)


def numeric_design():
    return HaloDesign(x_le_wing_m=4.1, area_wing_m2=21.4, area_horizontal_tail_m2=3.7, area_vertical_tail_m2=1.7,
                      torque_peak_motor_Nm=400.0, torque_peak_generator_Nm=460.0, power_rated_turboshaft_W=835e3,
                      mass_turboshaft_bare_kg=184.0, power_max_discharge_battery_W=None,
                      energy_capacity_battery_J=None, area_disk_m2=66.0, mass_fuel_kg=846.0,
                      speed_peak_motor_rad_s=1376.0, reduction_ratio=31.3, speed_peak_generator_rad_s=1405.0,
                      count_parallel_battery=18.3, speed_rotor_wing_design_rad_s=39.3,
                      power_rated_heat_exchanger_W=150e3, power_rated_gearbox_W=800e3)


class HaloThermalSwitchTests(unittest.TestCase):
    def test_on_by_default_and_off_switch(self):
        """Plan 030 (user-approved 2026-10-04) makes the thermal model the default."""
        self.assertTrue(HaloAssumptions().thermal_model)
        aircraft = build_halo_aircraft(numeric_design(), assumptions=HaloAssumptions(thermal_model=False))
        self.assertIsNone(aircraft.powertrain.cooling)
        self.assertIsNone(aircraft.powertrain.topology.instances["motor"].component.thermal_model)
        # Off: the gearbox keeps the motor's rating whatever the design says.
        instances = aircraft.powertrain.topology.instances
        self.assertEqual(instances["gearbox"].component.power_rated_W, instances["motor"].component.power_rated_W)

    def test_on_installs_cooler_and_thermal_machines(self):
        aircraft = build_halo_aircraft(numeric_design(), assumptions=thermal)
        instances = aircraft.powertrain.topology.instances
        self.assertIsNotNone(instances["motor"].component.thermal_model)
        self.assertIsNotNone(instances["generator"].component.thermal_model)
        self.assertEqual(instances["gearbox"].component.power_rated_W, 800e3)
        masses = dict((i.instance_name, i.mass_properties.mass) for i in aircraft.powertrain.get_instance_mass_properties())
        self.assertAlmostEqual(masses["heat_exchanger"], 150.0)
        self.assertEqual(aircraft.powertrain.cooling.sources_excluded, ("gearbox", "generator_gearbox"))


class HaloThermalSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sized = solve_halo_sizing(requirements_plan030, assumptions=assumptions_plan030)    # the plan 030 thermal reference

    def test_closes_with_all_margins(self):
        self.assertGreater(self.sized.min_margin, -1e-6)
        self.assertLess(abs(self.sized.closure_residual_kg), 1e-3)
        self.assertAlmostEqual(self.sized.mass_payload_kg, 900.0)
        self.assertAlmostEqual(self.sized.mass_takeoff_kg / 0.45359237, 14037, delta=10)    # plan 030 reference

    def test_heat_exchanger_covers_every_point_and_is_sized_by_one(self):
        hx = self.sized.heat_exchanger
        required_W = [row["power_heat_equivalent_W"] for row in self.sized.thermal_trace]
        self.assertLessEqual(max(required_W), hx["power_rated_W"] * (1 + 1e-6))
        self.assertAlmostEqual(max(required_W) / hx["power_rated_W"], 1.0, places=4)
        self.assertAlmostEqual(hx["mass_kg"], hx["power_rated_W"] / thermal.specific_power_heat_exchanger_W_kg)
        sizing_point = max(self.sized.thermal_trace, key=lambda row: row["power_heat_equivalent_W"])
        self.assertEqual(sizing_point["label"], "hot-day hover")
        self.assertAlmostEqual(dict(self.sized.powertrain_masses_kg)["heat_exchanger"], hx["mass_kg"])

    def test_cooling_drag_in_flight_fans_in_hover(self):
        for row in self.sized.thermal_trace:
            if row["mode"] == "hover":
                self.assertGreater(row["power_fan_W"], 0.0)
                self.assertEqual(row["drag_cooling_N"], 0.0)
            else:
                self.assertGreater(row["drag_cooling_N"], 0.0)
                self.assertEqual(row["power_fan_W"], 0.0)

    def test_heat_loads_hover_above_cruise(self):
        rows = {row["label"]: row for row in self.sized.thermal_trace}
        hover = sum(rows["take-off hover"]["heat_W"].values())
        cruise = sum(rows["cruise 1/4"]["heat_W"].values())
        self.assertGreater(hover, cruise)
        # From cold, the machines' thermal mass takes most of the take-off hover heat; in cruise (near steady)
        # nearly all of it reaches the cooler.
        self.assertLess(rows["take-off hover"]["power_heat_W"], rows["cruise 1/4"]["power_heat_W"])
        self.assertEqual(set(rows["cruise 1/4"]["heat_W"]),
                         {"motor", "gearbox", "generator", "battery", "generator_gearbox"})

    def test_temperatures_within_limit_and_short_time_rating_used(self):
        limits_C = dict(motor=(thermal.temperature_coolant_C, thermal.temperature_max_machine_C),
                        generator=(thermal.temperature_coolant_C, thermal.temperature_max_machine_C),
                        battery=(thermal.temperature_cell_battery_C, thermal.temperature_max_battery_C))
        for row in self.sized.thermal_trace:
            self.assertEqual(set(row["temperatures_end_C"]), set(limits_C))
            for name, temperature_C in row["temperatures_end_C"].items():
                self.assertLessEqual(temperature_C, limits_C[name][1] + 1e-3)
                self.assertGreaterEqual(temperature_C, limits_C[name][0] - 1e-6)
        # The pack's thermal mass absorbs the engine-out battery heat: less reaches the cooler than is generated.
        for row in self.sized.thermal_trace:
            if row["label"].startswith("engine-out"):
                self.assertLess(row["power_to_coolant_W"]["battery"], row["heat_W"]["battery"])
        motor_rated_W = self.sized.heat_exchanger["machines"]["motor"]["power_rated_W"]
        peak_W = max(row["power_shaft_motor_W"] for row in self.sized.thermal_trace if row["mode"] == "hover")
        self.assertGreater(peak_W / motor_rated_W, 1.0)          # a hover peak above the continuous rating
        self.assertLessEqual(peak_W, self.sized.heat_exchanger["power_rated_gearbox_W"] * (1 + 1e-6))

    def test_mission_temperatures_chain_from_coolant(self):
        mission_rows = [row for row in self.sized.thermal_trace if row["duration_s"] is not None
                        and not row["label"].startswith("engine-out")]
        self.assertEqual(mission_rows[0]["label"], "take-off hover")
        self.assertLess(mission_rows[0]["temperatures_end_C"]["motor"], thermal.temperature_max_machine_C)


if __name__ == "__main__":
    unittest.main()
