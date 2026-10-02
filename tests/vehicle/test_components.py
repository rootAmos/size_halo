import unittest

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer
import casadi as cas

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.items import FixedEquipment, InterconnectShaft, LandingGear, Nacelles, Payload, Systems
from aircraft_closure.weights import afdd
from aircraft_closure.vehicle.surfaces import HorizontalTail, VerticalTail, Wing

condition = StructuralDesignCondition(mass_design_kg=1500.0)


class SurfaceGeometryTests(unittest.TestCase):
    def test_wing_planform_recovered_by_aerosandbox(self):
        wing = Wing(area_m2=12.0, aspect_ratio=9.0, taper_ratio=0.5)
        asb_wing = wing.to_asb()
        self.assertAlmostEqual(float(asb_wing.area()), 12.0, places=9)
        self.assertAlmostEqual(float(asb_wing.span()), (12.0 * 9.0) ** 0.5, places=9)
        self.assertAlmostEqual(float(asb_wing.aspect_ratio()), 9.0, places=9)
        self.assertAlmostEqual(float(asb_wing.taper_ratio()), 0.5, places=9)
        chord_root_m = wing.chord_root_m()
        mac_closed_form_m = 2 / 3 * chord_root_m * (1 + 0.5 + 0.25) / 1.5
        self.assertAlmostEqual(float(asb_wing.mean_aerodynamic_chord()), mac_closed_form_m, places=9)

    def test_vertical_tail_is_one_fin(self):
        fin = VerticalTail(area_m2=1.6, aspect_ratio=1.6)
        asb_fin = fin.to_asb()
        self.assertFalse(asb_fin.symmetric)
        self.assertAlmostEqual(float(asb_fin.area()), 1.6, places=9)
        self.assertAlmostEqual(fin.height_m(), (1.6 * 1.6) ** 0.5, places=12)

    def test_cg_at_forty_percent_mac(self):
        wing = Wing(x_le_root_m=2.0)
        properties = wing.get_mass_properties(condition)
        self.assertAlmostEqual(float(properties.x_cg), 2.0 + 0.4 * float(wing.to_asb().mean_aerodynamic_chord()))


class CorrelationIdentityTests(unittest.TestCase):
    """Masses must be AeroSandbox's Raymer functions, not re-implementations."""

    def test_surfaces_equal_aerosandbox_raymer(self):
        op_point = condition.operating_point()
        wing = Wing()
        self.assertAlmostEqual(float(wing.get_mass_properties(condition).mass),
                               float(raymer.mass_wing(wing.to_asb(), 1500.0, 5.7, 0, op_point)), places=9)
        tail = HorizontalTail()
        self.assertAlmostEqual(float(tail.get_mass_properties(condition).mass),
                               float(raymer.mass_hstab(tail.to_asb(), 1500.0, 5.7, op_point)), places=9)
        fin = VerticalTail()
        self.assertAlmostEqual(float(fin.get_mass_properties(condition).mass),
                               float(raymer.mass_vstab(fin.to_asb(), 1500.0, 5.7, op_point)), places=9)

    def test_fuselage_equals_aerosandbox_raymer(self):
        fuselage = Fuselage()
        expected = raymer.mass_fuselage(fuselage.to_asb(), 1500.0, 5.7, 12.0, condition.operating_point(), 3.8)
        self.assertAlmostEqual(float(fuselage.get_mass_properties(condition, 3.8).mass), float(expected), places=9)


class ScalingTests(unittest.TestCase):
    def mass(self, item, design_condition):
        return float(item.get_mass_properties(design_condition).mass)

    def test_wing_mass_exponent_on_design_load(self):
        doubled = StructuralDesignCondition(mass_design_kg=3000.0)
        self.assertAlmostEqual(self.mass(Wing(), doubled) / self.mass(Wing(), condition), 2 ** 0.49, places=9)
        self.assertAlmostEqual(self.mass(HorizontalTail(), doubled) / self.mass(HorizontalTail(), condition),
                               2 ** 0.414, places=9)
        self.assertAlmostEqual(self.mass(VerticalTail(), doubled) / self.mass(VerticalTail(), condition),
                               2 ** 0.376, places=9)

    def test_load_factor_and_mass_enter_as_product(self):
        heavier = StructuralDesignCondition(mass_design_kg=3000.0)
        more_load = StructuralDesignCondition(mass_design_kg=1500.0, load_factor_ultimate=11.4)
        self.assertAlmostEqual(self.mass(Wing(), heavier), self.mass(Wing(), more_load), places=9)

    def test_physical_trends(self):
        self.assertGreater(self.mass(Wing(area_m2=16), condition), self.mass(Wing(area_m2=12), condition))
        self.assertGreater(self.mass(Wing(aspect_ratio=12), condition), self.mass(Wing(aspect_ratio=9), condition))
        thicker = Wing(airfoil=asb.Airfoil("naca4424"))
        self.assertLess(self.mass(thicker, condition), self.mass(Wing(), condition))
        fuselage_short = Fuselage(length_m=6).get_mass_properties(condition, 3.8).mass
        fuselage_long = Fuselage(length_m=8).get_mass_properties(condition, 3.8).mass
        self.assertGreater(float(fuselage_long), float(fuselage_short))

    def test_mass_factor_is_linear(self):
        self.assertAlmostEqual(self.mass(Wing(mass_factor=0.8), condition), 0.8 * self.mass(Wing(), condition))
        self.assertAlmostEqual(self.mass(LandingGear(mass_factor=1.2), condition),
                               1.2 * self.mass(LandingGear(), condition))


class ItemTests(unittest.TestCase):
    def test_landing_gear_cg_between_main_and_nose(self):
        gear = LandingGear(x_main_m=3.6, x_nose_m=1.0).get_mass_properties(condition)
        self.assertTrue(1.0 < float(gear.x_cg) < 3.6)
        main = raymer.mass_main_landing_gear(0.6, 1500.0, is_retractable=False)
        nose = raymer.mass_nose_landing_gear(0.5, 1500.0, is_retractable=False)
        self.assertAlmostEqual(float(gear.mass), float(main + nose), places=9)
        self.assertAlmostEqual(float(gear.x_cg), float((main * 3.6 + nose * 1.0) / (main + nose)), places=9)

    def test_systems_combine_flight_controls_and_avionics(self):
        systems = Systems(mass_avionics_uninstalled_kg=30.0)
        mass_kg = float(systems.get_mass_properties(condition, Wing(), Fuselage()).mass)
        self.assertGreater(mass_kg, float(raymer.mass_avionics(30.0)))
        heavier_avionics = Systems(mass_avionics_uninstalled_kg=60.0)
        self.assertGreater(float(heavier_avionics.get_mass_properties(condition, Wing(), Fuselage()).mass), mass_kg)

    def test_payload_is_a_point_mass(self):
        payload = Payload(mass_kg=250.0, x_m=3.1, z_m=-0.2).get_mass_properties()
        self.assertEqual((payload.mass, payload.x_cg, payload.z_cg), (250.0, 3.1, -0.2))

    def test_retractable_gear_is_heavier_and_matches_raymer(self):
        fixed = float(LandingGear().get_mass_properties(condition).mass)
        retractable = float(LandingGear(is_retractable=True).get_mass_properties(condition).mass)
        expected = (raymer.mass_main_landing_gear(0.6, 1500.0, is_retractable=True)
                    + raymer.mass_nose_landing_gear(0.5, 1500.0, is_retractable=True))
        self.assertAlmostEqual(retractable, float(expected), places=9)
        self.assertGreater(retractable, fixed)

    def test_nacelles_sum_the_afdd_engine_section(self):
        nacelles = Nacelles(mass_engines_kg=500.0, count_engines=2, area_wetted_m2=17.0, x_m=3.2, z_m=0.7,
                            mass_factor=1.2)
        expected = (afdd.mass_engine_support_afdd82_kg(500.0, 2) + afdd.mass_air_induction_afdd82_kg(500.0, 2)
                    + afdd.mass_engine_cowling_afdd82_kg(17.0))
        properties = nacelles.get_mass_properties()
        self.assertAlmostEqual(float(properties.mass), 1.2 * expected, places=9)
        self.assertEqual((properties.x_cg, properties.z_cg), (3.2, 0.7))

    def test_interconnect_shaft_is_the_afdd_drive_shaft(self):
        shaft = InterconnectShaft(power_drive_limit_W=2e6, speed_rotor_rad_s=60.0, length_m=10.0)
        self.assertAlmostEqual(float(shaft.get_mass_properties().mass),
                               afdd.mass_drive_shaft_afdd82_kg(2e6, 60.0, 10.0, 2, 0.6), places=9)

    def test_fixed_equipment_is_a_point_mass(self):
        equipment = FixedEquipment(mass_kg=40.0, x_m=2.0).get_mass_properties()
        self.assertEqual((equipment.mass, equipment.x_cg), (40.0, 2.0))


class SymbolicTests(unittest.TestCase):
    def test_geometry_and_design_mass_as_opti_variables(self):
        opti = asb.Opti()
        area_m2 = opti.variable(init_guess=12, lower_bound=1)
        mass_design_kg = opti.variable(init_guess=1500, lower_bound=100)
        properties = Wing(area_m2=area_m2).get_mass_properties(StructuralDesignCondition(mass_design_kg))
        self.assertIsInstance(properties.mass, cas.MX)
        opti.subject_to([area_m2 == 12, mass_design_kg == 1500])
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(properties.mass)),
                               float(Wing().get_mass_properties(condition).mass), places=6)


if __name__ == "__main__":
    unittest.main()
