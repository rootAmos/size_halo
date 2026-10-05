"""AFDD tiltrotor wing (Tier 20, plan 024): identities, limits, trends, symbols."""
import math
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.tools.units as u
import casadi as cas

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.surfaces import (TiltrotorWingMassModel, Wing, aluminium_wing_material,
                                               graphite_epoxy_wing_material)
from aircraft_closure.weights import afdd

# XV-15-like wing (SI): 32.17 ft span, 169 ft2, 2,142 lb per tip, 458 rpm design speed, aluminium.
material = aluminium_wing_material()
inputs = dict(
    span_m=32.17 * u.foot, chord_m=169.0 / 32.17 * u.foot, thickness_to_chord=0.23, fraction_chord_torque_box=0.45,
    mass_design_kg=13000 * u.lbm, mass_tip_kg=2142 * u.lbm, radius_gyration_pylon_m=2.77 * u.foot,
    speed_rotor_design_rad_s=458 * u.rpm, frequency_torsion_per_rev=1.087, frequency_beam_per_rev=0.432,
    frequency_chord_per_rev=0.825, density_torque_box_kg_m3=material.density_torque_box_kg_m3,
    density_spar_kg_m3=material.density_spar_kg_m3, modulus_shear_torque_box_Pa=material.modulus_shear_torque_box_Pa,
    modulus_torque_box_Pa=material.modulus_torque_box_Pa, modulus_spar_Pa=material.modulus_spar_Pa,
    strain_ultimate=material.strain_ultimate, area_control_surfaces_m2=31.2 * u.foot**2,
    unit_mass_fairing_kg_m2=10.92, unit_mass_control_surfaces_kg_m2=15.18, width_fuselage_m=5.5 * u.foot,
    width_attachment_m=56 * u.inch, fraction_fittings=0.129, efficiency_torque_box=0.583,
    correction_spar_stiffness=0.526)


def masses(**changes):
    return afdd.wing_tiltrotor_afdd_masses(**{**inputs, **changes})


class SectionFormFactorTests(unittest.TestCase):
    def test_published_polynomials_by_hand(self):
        tau, w = 0.23, 0.45
        beam, chord, torsion, cap = afdd.section_form_factors_tiltrotor_wing(tau, w)
        self.assertAlmostEqual(cap, 0.25 * math.sin(5.236 * w) + 0.325, places=12)
        self.assertAlmostEqual(chord, 0.640424 * w**2 - 0.89717 * w + 0.4615 * tau + 0.655317, places=12)
        self.assertAlmostEqual(torsion, (0.27 - tau) / 0.12 * 0.12739 * (-0.96 + math.sqrt(
            3.32 + 94.6788 * w - (w / 0.08344)**2)) - 2.7545 * w**2 + 5.1799 * w - 0.2683, places=12)
        self.assertAlmostEqual(beam, 0.073 * math.sin(2 * math.pi * (tau - 0.151) / 0.1365) + 0.14598 * tau
                               + 0.610 * math.sin(2 * math.pi * (w + 0.080) / 2.1560)
                               - (0.4126 - 1.6309 * tau) * (w - 0.131) + 0.0081, places=12)

    def test_thinner_sections_are_better_torsion_tubes(self):
        self.assertGreater(afdd.section_form_factors_tiltrotor_wing(0.15, 0.45)[2],
                           afdd.section_form_factors_tiltrotor_wing(0.23, 0.45)[2])


class IdentityTests(unittest.TestCase):
    def setUp(self):
        self.m = masses()
        self.factors = afdd.section_form_factors_tiltrotor_wing(0.23, 0.45)

    def test_torsional_stiffness_from_frequency(self):
        omega = 1.087 * inputs["speed_rotor_design_rad_s"]
        expected = omega**2 * 0.5 * inputs["span_m"] * 0.5 * inputs["mass_tip_kg"] * inputs["radius_gyration_pylon_m"]**2
        self.assertAlmostEqual(self.m.stiffness_torsion_Nm2 / expected, 1.0, places=12)

    def test_torque_box_area_and_mass(self):
        thickness_m = 0.23 * inputs["chord_m"]
        area = 4 * self.m.stiffness_torsion_Nm2 / (inputs["modulus_shear_torque_box_Pa"] * self.factors[2] * thickness_m**2)
        self.assertAlmostEqual(self.m.area_torque_box_m2 / area, 1.0, places=12)
        self.assertAlmostEqual(self.m.mass_torque_box_kg / (area * material.density_torque_box_kg_m3 * inputs["span_m"]
                                                            / 0.583), 1.0, places=12)

    def test_fairing_and_control_surfaces(self):
        area_fairing = (inputs["span_m"] - inputs["width_attachment_m"]) * inputs["chord_m"] * 0.55 \
            - inputs["area_control_surfaces_m2"]
        self.assertAlmostEqual(self.m.mass_fairing_kg, area_fairing * 10.92, places=9)
        self.assertAlmostEqual(self.m.mass_control_surfaces_kg, 31.2 * u.foot**2 * 15.18, places=9)

    def test_fittings_fraction_of_total_excluding_fold(self):
        rest = self.m.mass_primary_kg() + self.m.mass_fairing_kg + self.m.mass_control_surfaces_kg
        self.assertAlmostEqual(self.m.mass_fittings_kg / (rest + self.m.mass_fittings_kg), 0.129, places=12)

    def test_total_is_sum_of_parts(self):
        m = masses(fraction_fold=0.1)
        parts = (m.mass_torque_box_kg + m.mass_spar_stiffness_kg + m.mass_spar_jump_kg + m.mass_fairing_kg
                 + m.mass_control_surfaces_kg + m.mass_fittings_kg + m.mass_fold_kg)
        self.assertAlmostEqual(m.total(), parts, places=9)
        self.assertAlmostEqual(m.mass_fold_kg, 0.1 * (parts - m.mass_fold_kg + 2 * inputs["mass_tip_kg"]), places=9)

    def test_realized_torsion_frequency_is_the_requirement(self):
        self.assertAlmostEqual(self.m.frequency_torsion_rad_s / inputs["speed_rotor_design_rad_s"], 1.087, places=10)

    def test_realized_bending_frequencies_meet_requirements(self):
        speed = inputs["speed_rotor_design_rad_s"]
        self.assertGreaterEqual(self.m.frequency_beam_rad_s / speed, 0.432 - 1e-9)
        self.assertGreaterEqual(self.m.frequency_chord_rad_s / speed, 0.825 - 1e-9)

    def test_frequency_per_rev_scales_inversely_with_rotor_speed(self):
        torsion, beam, chord = TiltrotorWingMassModel.frequency_per_rev(self.m, 0.5 * inputs["speed_rotor_design_rad_s"])
        self.assertAlmostEqual(torsion, 2 * 1.087, places=9)


class LimitingCaseTests(unittest.TestCase):
    def test_no_bending_requirement_no_stiffness_caps(self):
        m = masses(frequency_beam_per_rev=0.0, frequency_chord_per_rev=0.0)
        self.assertEqual(m.mass_spar_stiffness_kg, 0.0)

    def test_no_jump_no_jump_caps(self):
        self.assertEqual(masses(load_factor_jump=0.0).mass_spar_jump_kg, 0.0)

    def test_strong_jump_needs_caps(self):
        self.assertGreater(masses(load_factor_jump=6.0).mass_spar_jump_kg, masses().mass_spar_jump_kg)

    def test_box_scales_with_torsion_frequency_squared(self):
        ratio = masses(frequency_torsion_per_rev=2 * 1.087).mass_torque_box_kg / masses().mass_torque_box_kg
        self.assertAlmostEqual(ratio, 4.0, places=9)

    def test_no_fittings_no_fold(self):
        m = masses(fraction_fittings=0.0)
        self.assertEqual(m.mass_fittings_kg, 0.0)
        self.assertEqual(m.mass_fold_kg, 0.0)

    def test_smoothing_converges_to_exact(self):
        exact = masses().total()
        self.assertAlmostEqual(masses(smoothing=1e-6).total() / exact, 1.0, places=5)
        self.assertGreaterEqual(masses(smoothing=0.05).total(), exact)


class TrendTests(unittest.TestCase):
    def test_mass_rises_with_torsion_frequency(self):
        self.assertGreater(masses(frequency_torsion_per_rev=1.3).total(), masses().total())

    def test_mass_rises_with_span(self):
        self.assertGreater(masses(span_m=1.2 * inputs["span_m"]).total(), masses().total())

    def test_mass_rises_with_design_mass_when_jump_sizes_caps(self):
        heavier = masses(mass_design_kg=1.3 * inputs["mass_design_kg"])
        self.assertGreater(heavier.mass_spar_jump_kg, masses().mass_spar_jump_kg)
        self.assertGreater(heavier.total(), masses().total())

    def test_mass_rises_with_tip_mass_and_rotor_speed(self):
        self.assertGreater(masses(mass_tip_kg=1.2 * inputs["mass_tip_kg"]).mass_torque_box_kg,
                           masses().mass_torque_box_kg)
        self.assertGreater(masses(speed_rotor_design_rad_s=1.2 * inputs["speed_rotor_design_rad_s"]).total(),
                           masses().total())

    def test_composite_wing_is_lighter(self):
        g = graphite_epoxy_wing_material()
        composite = masses(density_torque_box_kg_m3=g.density_torque_box_kg_m3, density_spar_kg_m3=g.density_spar_kg_m3,
                           modulus_shear_torque_box_Pa=g.modulus_shear_torque_box_Pa,
                           modulus_torque_box_Pa=g.modulus_torque_box_Pa, modulus_spar_Pa=g.modulus_spar_Pa,
                           strain_ultimate=g.strain_ultimate)
        self.assertLess(composite.mass_torque_box_kg, masses().mass_torque_box_kg)


class WingSubmodelTests(unittest.TestCase):
    def model(self, **changes):
        return TiltrotorWingMassModel(**{**dict(mass_tip_kg=inputs["mass_tip_kg"],
                                                radius_gyration_pylon_m=inputs["radius_gyration_pylon_m"],
                                                speed_rotor_design_rad_s=inputs["speed_rotor_design_rad_s"],
                                                width_fuselage_m=5.5 * u.foot, width_attachment_m=56 * u.inch),
                                         **changes})

    def test_wing_uses_submodel_and_factor(self):
        wing = Wing(area_m2=169 * u.foot**2, aspect_ratio=32.17**2 / 169, mass_model=self.model(), mass_factor=1.3)
        condition = StructuralDesignCondition(mass_design_kg=13000 * u.lbm)
        expected = 1.3 * self.model().mass_kg(wing, condition)
        self.assertAlmostEqual(float(wing.get_mass_properties(condition).mass), float(expected), places=9)
        # The model defaults round the inputs above, hence 0.1 %.
        self.assertAlmostEqual(float(expected) / 1.3, float(masses().total()), delta=1e-3 * float(expected))

    def test_symbolic_through_opti(self):
        """Wing area, tip mass and design rotor speed as Opti variables; minimize wing mass."""
        opti = asb.Opti()
        area_m2 = opti.variable(init_guess=16.0, lower_bound=10.0, upper_bound=25.0)
        speed_rad_s = opti.variable(init_guess=45.0, lower_bound=30.0, upper_bound=60.0)
        wing = Wing(area_m2=area_m2, aspect_ratio=6.0, mass_model=self.model(
            speed_rotor_design_rad_s=speed_rad_s, smoothing=0.01))
        mass = wing.get_mass_properties(StructuralDesignCondition(mass_design_kg=6000.0)).mass
        self.assertIsInstance(mass, cas.MX)
        opti.minimize(mass / 100)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution.value(speed_rad_s), 30.0, places=4)   # slower design rotor, lighter wing
        self.assertGreater(solution.value(mass), 0.0)



class CapDepthAndMinimumGaugeTests(unittest.TestCase):
    """Plan 035 options: spar-cap lever arm and minimum torque-box gauge."""

    def test_defaults_are_ndarc(self):
        base, explicit = masses(), masses(ratio_depth_spar_cap=1.0, thickness_min_torque_box_m=0.0)
        self.assertEqual(base.mass_primary_kg(), explicit.mass_primary_kg())

    def test_shallower_caps_need_more_cap_mass(self):
        base, shallow = masses(), masses(ratio_depth_spar_cap=0.8)
        self.assertGreater(shallow.mass_spar_jump_kg + shallow.mass_spar_stiffness_kg,
                           base.mass_spar_jump_kg + base.mass_spar_stiffness_kg)
        self.assertAlmostEqual(shallow.mass_torque_box_kg, base.mass_torque_box_kg, places=9)

    def test_beam_frequency_requirement_is_still_met(self):
        required_rad_s = inputs["frequency_beam_per_rev"] * inputs["speed_rotor_design_rad_s"]
        self.assertGreaterEqual(masses(ratio_depth_spar_cap=0.8).frequency_beam_rad_s, required_rad_s * (1 - 1e-9))

    def test_minimum_gauge_floors_the_box_and_raises_torsion(self):
        chord_m, tau = inputs["chord_m"], inputs["thickness_to_chord"]
        thickness_min_m = 0.004                                            # far above the XV-15 frequency need
        floored = masses(thickness_min_torque_box_m=thickness_min_m)
        area_min_m2 = thickness_min_m * 2 * (0.45 * chord_m + tau * chord_m)
        self.assertAlmostEqual(floored.area_torque_box_m2, area_min_m2, places=12)
        self.assertGreater(floored.frequency_torsion_rad_s, masses().frequency_torsion_rad_s)

    def test_inactive_minimum_gauge_changes_nothing(self):
        self.assertAlmostEqual(masses(thickness_min_torque_box_m=1e-6).mass_primary_kg(), masses().mass_primary_kg(),
                               places=9)


if __name__ == "__main__":
    unittest.main()
