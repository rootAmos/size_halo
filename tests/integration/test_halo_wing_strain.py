"""Plan 043: the wing spar-cap strain option in the Halo sizing (wiring only; the solve is in the plan log)."""
import unittest
from dataclasses import replace

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import HaloAssumptions, HaloDesign, build_halo_aircraft, factor_load_carrying_wing


def numeric_design(**changes):
    return replace(HaloDesign(
        x_le_wing_m=4.6, area_wing_m2=23.3, area_horizontal_tail_m2=5.2, area_vertical_tail_m2=2.7,
        torque_peak_motor_Nm=1617.0, torque_peak_generator_Nm=891.0, power_rated_turboshaft_W=835e3,
        mass_turboshaft_bare_kg=184.0, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
        area_disk_m2=73.4, mass_fuel_kg=936.0, solidity=0.06, speed_tip_m_s=238.0, speed_peak_motor_rad_s=204.0,
        reduction_ratio=4.53, speed_peak_generator_rad_s=580.0, count_parallel_battery=23.0,
        speed_rotor_wing_design_rad_s=56.3, power_rated_heat_exchanger_W=127e3, power_rated_gearbox_W=757e3),
        **changes)


assumptions = HaloAssumptions(count_units_motor=2, count_units_generator=3)
condition = StructuralDesignCondition(mass_design_kg=6885.0)


class HaloWingStrainTests(unittest.TestCase):
    def test_off_by_default(self):
        self.assertFalse(HaloAssumptions().wing_strain_margin)
        wing = build_halo_aircraft(numeric_design(), assumptions=assumptions).wing
        self.assertEqual(wing.mass_model.area_spar_added_m2, 0.0)

    def test_load_carrying_factor_is_half_the_increment(self):
        wing = build_halo_aircraft(numeric_design(), assumptions=assumptions).wing
        self.assertAlmostEqual(factor_load_carrying_wing(wing, assumptions), 1 + 0.5 * (wing.mass_factor - 1))
        self.assertAlmostEqual(factor_load_carrying_wing(wing, replace(assumptions,
                                                         fraction_load_carrying_calibration_wing=0.0)), 1.0)

    def test_added_caps_reach_the_wing_and_lower_the_strain(self):
        base = build_halo_aircraft(numeric_design(), assumptions=assumptions).wing
        added = build_halo_aircraft(numeric_design(area_spar_strain_m2=5e-4), assumptions=assumptions).wing
        self.assertEqual(added.mass_model.area_spar_added_m2, 5e-4)
        self.assertGreater(added.get_mass_properties(condition).mass, base.get_mass_properties(condition).mass)
        strain = lambda w: w.mass_model.strain_cap_jump_ultimate(w.mass_model.masses(w, condition), w, condition,
                                                                 factor_peak=assumptions.factor_peak_strain_cap_wing)[1]
        self.assertLess(strain(added), strain(base))
        # The reference-like wing without the calibration credit sits above the allowable (the FE finding).
        self.assertGreater(strain(base), base.mass_model.material.strain_ultimate)


if __name__ == "__main__":
    unittest.main()
