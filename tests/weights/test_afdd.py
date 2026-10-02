import unittest

import aerosandbox as asb
import aerosandbox.tools.units as u
import casadi as cas

from aircraft_closure.weights import afdd

# XV-15-like inputs: two 25 ft, 3-blade rotors, 14 in chord, 740 ft/s, nu 1.55;
# 3,100 hp drive at 565 rpm rotor / 20,000 rpm engine; 32.17 ft interconnect.
rotor = dict(count_rotors=2, count_blades=3, radius_m=12.5 * u.foot, chord_m=14 * u.inch,
             speed_tip_m_s=740 * u.foot, frequency_flap_per_rev=1.55)
power_drive_W = 3100 * u.hp
speed_rotor_rad_s = 565 * u.rpm


class PublishedEquationTests(unittest.TestCase):
    """Each wrapper equals the NDARC equation evaluated by hand in lb, ft, ft/s, hp and rpm."""

    def test_blades(self):
        # 0.02606 * 2 * 3^0.6592 * 12.5^1.3371 * (14/12)^0.9959 * 740^0.6682 * 1.55^2.5279
        self.assertAlmostEqual(afdd.mass_blades_afdd82_kg(**rotor) / u.lbm, 918.826599472869, places=6)

    def test_hub(self):
        mass_blades_kg = 918.826599472869 * u.lbm
        mass_hub_kg = afdd.mass_hub_afdd82_kg(2, 3, 12.5 * u.foot, 740 * u.foot, 1.55, mass_blades_kg)
        self.assertAlmostEqual(mass_hub_kg / u.lbm, 625.853370798386, places=6)

    def test_rotor_group_is_blades_plus_hub(self):
        self.assertAlmostEqual(afdd.mass_rotor_group_afdd82_kg(**rotor) / u.lbm, 918.826599472869 + 625.853370798386,
                               places=6)

    def test_gearbox_and_rotor_shaft(self):
        mass_kg = afdd.mass_gearbox_rotor_shaft_afdd83_kg(power_drive_W, speed_rotor_rad_s, 20000 * u.rpm, 3, 0.6)
        self.assertAlmostEqual(mass_kg / u.lbm, 1168.2101000180462, places=6)

    def test_drive_shaft(self):
        mass_kg = afdd.mass_drive_shaft_afdd82_kg(power_drive_W, speed_rotor_rad_s, 32.17 * u.foot, 2, 0.6)
        self.assertAlmostEqual(mass_kg / u.lbm, 96.31113528242616, places=6)

    def test_engine_section(self):
        mass_engines_kg = 2 * 600 * u.lbm
        self.assertAlmostEqual(afdd.mass_engine_support_afdd82_kg(mass_engines_kg, 2) / u.lbm, 112.33964849155925,
                               places=6)
        self.assertAlmostEqual(afdd.mass_air_induction_afdd82_kg(mass_engines_kg, 2) / u.lbm, 48.145563639239676,
                               places=6)
        self.assertAlmostEqual(afdd.mass_engine_cowling_afdd82_kg(190 * u.foot**2) / u.lbm, 272.5206072521294,
                               places=6)


class ScalingTests(unittest.TestCase):
    def test_blade_exponents(self):
        base = afdd.mass_blades_afdd82_kg(**rotor)
        for name, exponent in (("radius_m", 1.3371), ("chord_m", 0.9959), ("speed_tip_m_s", 0.6682),
                               ("frequency_flap_per_rev", 2.5279)):
            scaled = afdd.mass_blades_afdd82_kg(**{**rotor, name: 2 * rotor[name]})
            self.assertAlmostEqual(scaled / base, 2**exponent, places=10, msg=name)
        self.assertAlmostEqual(afdd.mass_blades_afdd82_kg(**{**rotor, "count_rotors": 4}) / base, 2.0, places=12)

    def test_hub_grows_with_blade_mass(self):
        light = afdd.mass_hub_afdd82_kg(2, 3, 3.8, 225.0, 1.2, 200.0)
        heavy = afdd.mass_hub_afdd82_kg(2, 3, 3.8, 225.0, 1.2, 400.0)
        self.assertAlmostEqual(heavy / light, 2**0.5505, places=10)

    def test_drive_trends(self):
        """More power or a slower rotor (more torque) means a heavier drive system."""
        gearbox = lambda power_W, speed_rad_s: afdd.mass_gearbox_rotor_shaft_afdd83_kg(power_W, speed_rad_s,
                                                                                    2000.0, 3, 0.6)
        self.assertGreater(gearbox(2e6, 60.0), gearbox(1e6, 60.0))
        self.assertGreater(gearbox(1e6, 30.0), gearbox(1e6, 60.0))
        shaft = lambda length_m: afdd.mass_drive_shaft_afdd82_kg(1e6, 60.0, length_m, 2, 0.6)
        self.assertAlmostEqual(shaft(20.0) / shaft(10.0), 2**1.0455, places=10)

    def test_air_induction_fraction_splits_one_correlation(self):
        mass_engines_kg, count = 500.0, 2
        total = (afdd.mass_engine_support_afdd82_kg(mass_engines_kg, count, 0.25)
                 + afdd.mass_air_induction_afdd82_kg(mass_engines_kg, count, 0.25))
        self.assertAlmostEqual(total, 0.0412 * (250 / u.lbm)**1.1433 * 2**1.3762 * u.lbm, places=9)


class SymbolicTests(unittest.TestCase):
    def test_functions_accept_opti_variables(self):
        opti = asb.Opti()
        radius_m = opti.variable(init_guess=3.0, lower_bound=0.5)
        power_W = opti.variable(init_guess=5e5, lower_bound=1e3)
        mass_engines_kg = opti.variable(init_guess=200.0, lower_bound=1.0)
        expressions = [
            afdd.mass_rotor_group_afdd82_kg(2, 3, radius_m, 0.3, 220.0, 1.2),
            afdd.mass_gearbox_rotor_shaft_afdd83_kg(power_W, 60.0, 2000.0, 3, 0.6),
            afdd.mass_drive_shaft_afdd82_kg(power_W, 60.0, 9.0, 2, 0.6),
            afdd.mass_engine_support_afdd82_kg(mass_engines_kg, 2) + afdd.mass_engine_cowling_afdd82_kg(radius_m),
        ]
        self.assertTrue(all(isinstance(expression, cas.MX) for expression in expressions))
        opti.subject_to([radius_m == 3.0, power_W == 5e5, mass_engines_kg == 200.0])
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(expressions[0])),
                               afdd.mass_rotor_group_afdd82_kg(2, 3, 3.0, 0.3, 220.0, 1.2), places=8)
        self.assertAlmostEqual(float(solution.value(expressions[1])),
                               afdd.mass_gearbox_rotor_shaft_afdd83_kg(5e5, 60.0, 2000.0, 3, 0.6), places=8)


if __name__ == "__main__":
    unittest.main()
