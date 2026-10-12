"""Plan 046: the wing-section thickness reaches the AFDD wing and its stiffness; the family is the default (plan 059)."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.wing_section import SectionAirfoil
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import HaloAssumptions, HaloDesign, HaloRequirements, build_halo_aircraft, halo_wing_section

design = HaloDesign(
    x_le_wing_m=4.62, area_wing_m2=23.3, area_horizontal_tail_m2=5.25, area_vertical_tail_m2=2.73,
    torque_peak_motor_Nm=1617.0, torque_peak_generator_Nm=891.0, power_rated_turboshaft_W=835183.7,
    mass_turboshaft_bare_kg=184.2, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
    area_disk_m2=73.4, mass_fuel_kg=936.0, solidity=0.06, speed_tip_m_s=238.2, speed_peak_motor_rad_s=204.5,
    reduction_ratio=4.53, speed_peak_generator_rad_s=580.5, count_parallel_battery=22.95,
    speed_rotor_wing_design_rad_s=56.3, power_rated_heat_exchanger_W=1.27e5, power_rated_gearbox_W=7.57e5)
family = replace(HaloAssumptions(), count_units_motor=2, count_units_generator=3)
base = replace(family, wing_section_model="naca2423")
condition = StructuralDesignCondition(mass_design_kg=6885.0, load_factor_ultimate=4.5, velocity_cruise_m_s=85.0,
                                      altitude_cruise_m=3000.0, lift_to_drag_cruise=9.0)


def wing_masses(design_, assumptions):
    wing = build_halo_aircraft(design_, HaloRequirements(), assumptions).wing
    return wing.mass_model.masses(wing, condition), wing


class HaloWingSectionTests(unittest.TestCase):
    def test_coordinate_naca2423_option(self):
        self.assertEqual(HaloAssumptions().wing_section_model, "family")
        a = replace(HaloAssumptions(), wing_section_model="naca2423")
        self.assertFalse(a.free_thickness_to_chord_wing)
        self.assertIsNone(a.stiffness_min_wing_Nm2)
        thickness, _, airfoil = halo_wing_section(design, a)
        self.assertEqual(thickness, 0.23)
        self.assertNotIsInstance(airfoil, SectionAirfoil)
        self.assertEqual(airfoil.name, "naca2423")
        with self.assertRaises(ValueError):
            halo_wing_section(replace(design, thickness_to_chord_wing=0.2), a)

    def test_family_at_23_percent_is_the_reference_wing(self):
        masses_ref, _ = wing_masses(design, base)
        masses_fam, wing = wing_masses(design, family)
        self.assertIsInstance(wing.airfoil, SectionAirfoil)
        self.assertAlmostEqual(float(masses_fam.total()), float(masses_ref.total()), places=9)

    def test_thickness_reaches_box_depth_and_stiffness(self):
        """Thinner section: the torsion-frequency box grows (GJ ~ t^2 per wall area) and the wing gets heavier; the
        realized torsion stiffness stays at least at the frequency target."""
        thick, _ = wing_masses(replace(design, thickness_to_chord_wing=0.23), family)
        thin, wing = wing_masses(replace(design, thickness_to_chord_wing=0.18), family)
        self.assertEqual(wing.mass_model.thickness_to_chord, 0.18)
        self.assertGreater(float(thin.mass_torque_box_kg), float(thick.mass_torque_box_kg))
        self.assertGreater(float(thin.total()), float(thick.total()))
        self.assertGreaterEqual(float(thin.stiffness_torsion_Nm2), 0.99 * float(thick.stiffness_torsion_Nm2))

    def test_spar_cap_depth_follows_the_assumed_thickness(self):
        """The cap depth scales with t/c whether t/c is a design value or the assumption (a lifted input)."""
        ratios = [build_halo_aircraft(design, HaloRequirements(), replace(family, thickness_to_chord_wing=t)
                                      ).wing.mass_model.ratio_depth_spar_cap for t in (0.18, 0.23)]
        self.assertAlmostEqual(ratios[0], ratios[1], places=12)

    def test_symbolic_thickness_in_the_wing_mass(self):
        opti = asb.Opti()
        t = opti.variable(init_guess=0.2, lower_bound=0.15, upper_bound=0.28)
        masses, _ = wing_masses(replace(design, thickness_to_chord_wing=t), family)
        opti.subject_to(t == 0.2)
        opti.minimize(masses.total() / 100)
        solution = opti.solve(verbose=False)
        numeric, _ = wing_masses(replace(design, thickness_to_chord_wing=0.2), family)
        self.assertAlmostEqual(float(np.ravel(solution(masses.total()))[0]), float(numeric.total()), places=6)


if __name__ == "__main__":
    unittest.main()
