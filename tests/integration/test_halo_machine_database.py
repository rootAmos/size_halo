"""Plan 033: Halo with the database machine mass model and explicit gearbox stages.

The defaults are unchanged: the 14,037 lb reference is asserted in test_halo_thermal. The sizing here uses the fast
set (`assumptions_plan027` with Scholz aerodynamics), so a solve takes seconds rather than minutes.
"""
import unittest
from dataclasses import replace

import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.powertrain.components.gearbox import GearStageModel
from aircraft_closure.powertrain.components.motor import DatabaseMassModel, Motor, TorqueDensityMassModel
from aircraft_closure.weights import afdd
from examples.halo_sizing import (HaloAssumptions, HaloDesign, assumptions_plan027, build_halo_aircraft,
                                  ratio_reference_gear_stages, solve_halo_sizing, stage_mass_ratio)
from examples.xv15_reference import calibration_factors

# Plan 035 changed the defaults; these tests reproduce their tier on the plan 030 settings.
from functools import partial  # noqa: E402
from examples.halo_sizing import pre_plan035  # noqa: E402
HaloAssumptions = partial(HaloAssumptions, **pre_plan035)
build_halo_aircraft = partial(build_halo_aircraft, assumptions=HaloAssumptions())
solve_halo_sizing = partial(solve_halo_sizing, assumptions=HaloAssumptions())


def numeric_design():
    """A numeric Halo design near the reference (as in test_halo_thermal)."""
    return HaloDesign(x_le_wing_m=4.1, area_wing_m2=21.4, area_horizontal_tail_m2=3.7, area_vertical_tail_m2=1.7,
                      torque_peak_motor_Nm=400.0, torque_peak_generator_Nm=460.0, power_rated_turboshaft_W=835e3,
                      mass_turboshaft_bare_kg=184.0, power_max_discharge_battery_W=None,
                      energy_capacity_battery_J=None, area_disk_m2=66.0, mass_fuel_kg=846.0,
                      speed_peak_motor_rad_s=1376.0, reduction_ratio=31.3, speed_peak_generator_rad_s=1405.0,
                      count_parallel_battery=18.3, speed_rotor_wing_design_rad_s=39.3,
                      power_rated_heat_exchanger_W=150e3, power_rated_gearbox_W=800e3)

fast = replace(assumptions_plan027, aerodynamics_model="scholz")
both = dict(machine_mass_model="database", gearbox_stages=True)


def instances(assumptions, design=None):
    aircraft = build_halo_aircraft(design or numeric_design(), assumptions=assumptions)
    return {name: instance.component for name, instance in aircraft.powertrain.topology.instances.items()}


class HaloMachineDatabaseSwitchTests(unittest.TestCase):
    def test_defaults_off(self):
        a = HaloAssumptions()
        self.assertEqual(a.machine_mass_model, "torque_density")
        self.assertFalse(a.gearbox_stages)
        self.assertFalse(a.gear_stage_model.staircase)          # relaxed stage count when switched on
        self.assertEqual(a.reduction_ratio_max, 40.0)
        parts = instances(a)
        self.assertIsInstance(parts["motor"].mass_model, TorqueDensityMassModel)
        self.assertEqual(parts["gearbox"].efficiency, 0.97)
        self.assertEqual(parts["generator_gearbox"].efficiency, 0.97)

    def test_database_model_and_inverter_split(self):
        integrated = instances(replace(fast, machine_mass_model="database"))["motor"]
        self.assertIsInstance(integrated.mass_model, DatabaseMassModel)
        self.assertEqual(integrated.mass_model.specific_power_inverter_W_kg, 20000.0)
        self.assertAlmostEqual(float(integrated.mass_model.mass_inverter_kg(integrated)),
                               integrated.power_rated_W / 20000.0)
        bare = instances(replace(fast, machine_mass_model="database", electrical_layer=True))["motor"]
        self.assertIsNone(bare.mass_model.specific_power_inverter_W_kg)
        self.assertAlmostEqual(float(integrated.get_mass() - bare.get_mass()), integrated.power_rated_W / 20000.0,
                               delta=1e-6)
        with self.assertRaises(ValueError):
            build_halo_aircraft(numeric_design(), assumptions=replace(fast, machine_mass_model="catalogue"))

    def test_stage_model_is_neutral_at_the_xv15_ratio(self):
        self.assertAlmostEqual(ratio_reference_gear_stages(), 20000 / 565, places=10)
        self.assertAlmostEqual(stage_mass_ratio(GearStageModel(), ratio_reference_gear_stages()), 1.0)

    def test_stage_wiring(self):
        a = replace(fast, gearbox_stages=True)
        design = numeric_design()
        parts = instances(a, design)
        model = a.gear_stage_model
        self.assertAlmostEqual(float(parts["gearbox"].efficiency), float(model.efficiency(design.reduction_ratio)))
        radius_m = np.sqrt(design.area_disk_m2 / np.pi)
        speed_rotor_rad_s = design.speed_tip_m_s / radius_m if design.speed_tip_m_s else a.speed_tip_m_s / radius_m
        expected_kg = calibration_factors().transmission * afdd.mass_gearbox_rotor_shaft_afdd00_kg(
            2, 2 * parts["gearbox"].power_rated_W, speed_rotor_rad_s * ratio_reference_gear_stages(),
            speed_rotor_rad_s) * stage_mass_ratio(model, design.reduction_ratio)
        self.assertAlmostEqual(float(parts["gearbox"].get_mass()) * 2, float(expected_kg), places=6)
        ratio_step_up = design.speed_peak_generator_rad_s / a.speed_output_turboshaft_rad_s
        self.assertAlmostEqual(float(parts["generator_gearbox"].efficiency), float(model.efficiency(ratio_step_up)))


class HaloMachineDatabaseSizingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assumptions = replace(fast, **both)                      # relaxed (softplus) stage count, the default
        cls.sized = solve_halo_sizing(assumptions=cls.assumptions)
        cls.staircase = replace(cls.assumptions, gear_stage_model=GearStageModel(staircase=True))
        cls.sized_staircase = solve_halo_sizing(assumptions=cls.staircase)

    def test_closes_with_all_margins(self):
        for r in (self.sized, self.sized_staircase):
            self.assertGreater(r.min_margin, -1e-6)
            self.assertLess(abs(r.closure_residual_kg), 1e-3)
            self.assertAlmostEqual(r.mass_payload_kg, 900.0)

    def test_stage_count_matches_ratio(self):
        d, model = self.sized.design, self.assumptions.gear_stage_model
        x = np.log(d.reduction_ratio) / np.log(model.ratio_max_stage)
        self.assertAlmostEqual(float(model.count_stages(d.reduction_ratio)), max(1.0, x), delta=0.04)
        # Staircase: within a step the count is near an integer; on a step the ratio sits at a boundary.
        d, model = self.sized_staircase.design, self.staircase.gear_stage_model
        stages = float(model.count_stages(d.reduction_ratio))
        x = np.log(d.reduction_ratio) / np.log(model.ratio_max_stage)
        self.assertTrue(abs(stages - round(stages)) < 0.05 or abs(x - round(x)) < 0.1)

    def test_drive_masses_consistent(self):
        a, r = self.assumptions, self.sized
        masses = dict(r.powertrain_masses_kg)
        model = DatabaseMassModel(specific_power_inverter_W_kg=a.specific_power_inverter_W_kg)
        d = r.design
        machine = Motor(power_rated_W=r.power_rated_motor_W, max_torque_Nm=2.5 * d.torque_peak_motor_Nm)
        self.assertAlmostEqual(masses["motor"], 2 * float(model.mass_kg(machine)), delta=1e-3 * masses["motor"])

    def test_result_recorded(self):
        """Plan 033 fast-set results (Scholz, thermal off; 13,702 lb with both options off).
        Relaxed stages: 13,978 lb, rotor ratio about 36 (2.23 relaxed stages), motor at the 2,000 rad/s bound.
        Staircase: 13,603 lb, a two-stage 24.6:1 rotor gearbox (the optimizer stops at the 5^2 = 25 boundary)."""
        self.assertAlmostEqual(self.sized.mass_takeoff_kg / u.lbm, 13978, delta=30)
        self.assertAlmostEqual(self.sized_staircase.mass_takeoff_kg / u.lbm, 13603, delta=30)
        self.assertAlmostEqual(float(self.staircase.gear_stage_model.count_stages(
            self.sized_staircase.design.reduction_ratio)), 2.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
