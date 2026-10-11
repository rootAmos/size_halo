"""Plan 048: machine units uprated in continuous power (unit mass, peak torque and speed unchanged)."""
import unittest
from dataclasses import replace

from examples.halo_sizing import HaloAssumptions, HaloDesign, build_halo_aircraft

# The v3.6 reference design (numeric), as in tests/aerodynamics/test_wing_section.py.
design = HaloDesign(
    x_le_wing_m=4.62, area_wing_m2=23.3, area_horizontal_tail_m2=5.25, area_vertical_tail_m2=2.73,
    torque_peak_motor_Nm=1617.0, torque_peak_generator_Nm=891.0, power_rated_turboshaft_W=835183.7,
    mass_turboshaft_bare_kg=184.2, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
    area_disk_m2=73.4, mass_fuel_kg=936.0, solidity=0.06, speed_tip_m_s=238.2, speed_peak_motor_rad_s=204.5,
    reduction_ratio=4.53, speed_peak_generator_rad_s=580.5, count_parallel_battery=22.95,
    speed_rotor_wing_design_rad_s=56.3, power_rated_heat_exchanger_W=1.27e5, power_rated_gearbox_W=7.57e5)
units = replace(HaloAssumptions(), count_units_motor=2, count_units_generator=2)


def machines(factor):
    instances = build_halo_aircraft(design, assumptions=replace(units, factor_uprating_power_units=factor)) \
        .powertrain.topology.instances
    return instances["motor"].component, instances["generator"].component


class UnitUpratingTests(unittest.TestCase):
    def test_v3_7_default_and_v3_6_set(self):
        from examples.halo_sizing import assumptions_v3_6
        self.assertEqual(HaloAssumptions().factor_uprating_power_units, 1.10)
        self.assertEqual(assumptions_v3_6.factor_uprating_power_units, 1.0)

    def test_power_uprated_torque_and_unit_mass_not(self):
        (motor, generator), (motor_up, generator_up) = machines(1.0), machines(1.10)
        for base, up in ((motor, motor_up), (generator, generator_up)):
            self.assertAlmostEqual(float(up.power_rated_W) / float(base.power_rated_W), 1.10, places=12)
            self.assertAlmostEqual(float(up.max_torque_Nm), float(base.max_torque_Nm), places=9)
            self.assertAlmostEqual(float(up.max_speed_rad_s), float(base.max_speed_rad_s), places=9)
            # The same drive run harder: unit and inverter mass unchanged.
            self.assertAlmostEqual(float(up.get_mass()), float(base.get_mass()), places=9)

    def test_relaxed_power_count_falls_by_the_factor(self):
        # Two catalogue generator units deliver 2 x 315 kW continuous; uprated by 1.10 they deliver 693 kW, so a
        # demand of 2.004 units of catalogue power (631 kW) fits in two units.
        _, generator_up = machines(1.10)
        self.assertGreater(float(generator_up.power_rated_W), 2.004 * 315e3)


if __name__ == "__main__":
    unittest.main()
