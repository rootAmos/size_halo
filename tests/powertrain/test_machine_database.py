"""Plan 033: supplier machine database, its torque-density fit, and DatabaseMassModel."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import casadi as cas

from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import DatabaseMassModel, Motor, rubber_machine
from aircraft_closure.powertrain.machine_database import fit_torque_density, load_machine_database

ratios = dict(torque_ratio=2.5, power_ratio=1.25, speed_ratio=2.5)    # the Halo rubber machine
exact = dict(smoothing_kg=1e-6)


class DatabaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records = load_machine_database()
        cls.fit = fit_torque_density(cls.records)

    def test_every_row_cited_and_classified(self):
        self.assertGreaterEqual(len(self.records), 20)
        for r in self.records:
            self.assertTrue(r.source_url.startswith("https://"), r.name)
            self.assertIn(r.inverter_included, ("y", "n", "unknown"))
            self.assertIn(r.stackable, ("y", "n", "unknown"))
            self.assertIn(r.topology, ("axial", "radial", "unknown"))
            self.assertGreater(r.mass_kg, 0)
            self.assertTrue(r.torque_peak_Nm is not None or r.torque_continuous_Nm is not None, r.name)
            if r.torque_peak_Nm is not None and r.torque_continuous_Nm is not None:
                self.assertGreaterEqual(r.torque_peak_Nm, r.torque_continuous_Nm, r.name)

    def test_fit_rows_are_bare_with_continuous_ratings(self):
        for r in self.records:
            if r.fit:
                self.assertNotEqual(r.inverter_included, "y", r.name)
                self.assertIsNotNone(r.torque_continuous_Nm, r.name)
                self.assertIsNotNone(r.power_continuous_W, r.name)
        self.assertEqual(len(self.fit.residuals), sum(r.fit for r in self.records))

    def test_defaults_are_the_fit(self):
        model = DatabaseMassModel()
        self.assertAlmostEqual(model.torque_density_ref_Nm_kg, self.fit.torque_density_ref_Nm_kg, delta=0.01)
        self.assertAlmostEqual(model.exponent_speed, self.fit.exponent_speed, delta=0.001)
        self.assertEqual(model.speed_ref_rad_s, self.fit.speed_ref_rad_s)

    def test_fit_reproduces_each_machine_within_tolerance(self):
        """Stated tolerance: every fitted machine within a factor of 2.6 in mass (the worst, the Evolito D250, is
        2.56); RMS log residual below 0.4 (a factor of 1.5); the Helix SPX242 family and EMRAX within 30 %."""
        for name, ratio in self.fit.residuals:
            self.assertLess(abs(np.log(ratio)), np.log(2.6), name)
            if name.startswith(("SPX242", "EMRAX")):
                self.assertLess(abs(ratio - 1), 0.30, name)
        self.assertLess(self.fit.rms_log_residual, 0.4)

    def test_model_mass_equals_fit_on_database_rows(self):
        """DatabaseMassModel on a database machine's own continuous ratings returns fit residual x database mass."""
        model = DatabaseMassModel(ratio_torque_continuous_peak=1.0, specific_power_max_W_kg=1e9, **exact)
        residuals = dict(self.fit.residuals)
        for r in self.records:
            if r.fit:
                machine = Motor(power_rated_W=r.power_continuous_W, max_torque_Nm=r.torque_continuous_Nm)
                self.assertAlmostEqual(float(model.mass_kg(machine)) / r.mass_kg, residuals[r.name], delta=0.01)

    def test_torque_density_falls_with_speed(self):
        self.assertGreater(self.fit.exponent_speed, 0.0)
        self.assertLess(self.fit.exponent_speed, 1.0)       # specific power still rises with speed
        model = DatabaseMassModel()
        self.assertGreater(model.torque_density_Nm_kg(150.0), model.torque_density_Nm_kg(1500.0))

    def test_reference_rows_against_model(self):
        """Not fitted: peak-only and integrated rows sit within a factor of 2.1 of the model (with the inverter term
        for integrated rows, and continuous = 0.5 x peak for peak-only rows)."""
        bare = DatabaseMassModel(specific_power_max_W_kg=1e9, **exact)
        for r in self.records:
            if r.fit:
                continue
            torque_Nm = r.torque_continuous_Nm if r.torque_continuous_Nm is not None else 0.5 * r.torque_peak_Nm
            # Peak-only rows: the rated speed, else the maximum (D1500 1x3 publishes only 2,500 rpm max).
            speed_rad_s = r.speed_base_rad_s() if r.torque_continuous_Nm is not None else (
                r.speed_rated_rad_s if r.speed_rated_rad_s is not None else r.speed_max_rad_s)
            mass_kg = torque_Nm / bare.torque_density_Nm_kg(speed_rad_s)
            if r.inverter_included == "y":
                power_W = r.power_continuous_W if r.power_continuous_W is not None else 0.5 * r.power_peak_W
                mass_kg += power_W / 20000.0
            self.assertLess(abs(np.log(mass_kg / r.mass_kg)), np.log(2.1), r.name)


class DatabaseMassModelTests(unittest.TestCase):
    def test_base_speed_is_the_rubber_machine_peak_efficiency_speed(self):
        model = DatabaseMassModel()
        motor = rubber_machine(Motor, 400.0, 1000.0, mass_model=model, **ratios)
        self.assertAlmostEqual(model.speed_base_rad_s(motor), 400.0)
        self.assertAlmostEqual(model.torque_continuous_Nm(motor), 1250.0)

    def test_closed_form_torque_limited(self):
        model = DatabaseMassModel(**exact)
        motor = rubber_machine(Motor, 100.0, 2000.0, mass_model=model, **ratios)
        tau = 11.79 * (100.0 / 500.0) ** -0.271
        self.assertAlmostEqual(float(model.mass_kg(motor)), 2500.0 / tau, places=4)

    def test_power_cap_at_high_speed(self):
        model = DatabaseMassModel(**exact)
        motor = rubber_machine(Motor, 20000.0, 10.0, mass_model=model, **ratios)
        self.assertAlmostEqual(float(model.mass_kg(motor)), motor.power_rated_W / 20000.0, places=4)

    def test_faster_is_lighter_at_equal_power(self):
        model = DatabaseMassModel()
        slow, fast = (rubber_machine(Motor, s, 4e5 / s, mass_model=model, **ratios) for s in (200.0, 1000.0))
        self.assertAlmostEqual(slow.power_rated_W, fast.power_rated_W)
        self.assertLess(float(fast.get_mass()), float(slow.get_mass()))
        # Mass ~ w^(a - 1) at fixed power while the torque term binds.
        self.assertAlmostEqual(float(fast.get_mass() / slow.get_mass()), 5.0 ** (0.271 - 1), delta=0.02)

    def test_inverter_term(self):
        bare = DatabaseMassModel()
        integrated = DatabaseMassModel(specific_power_inverter_W_kg=20000.0)
        motor = rubber_machine(Motor, 400.0, 1000.0, **ratios)
        self.assertAlmostEqual(float(integrated.mass_kg(motor) - bare.mass_kg(motor)), motor.power_rated_W / 20000.0)
        self.assertEqual(bare.mass_inverter_kg(motor), 0.0)

    def test_stack_scaling(self):
        """n stacks of torque T at one speed weigh n times one stack, plus n stack overheads."""
        unit = rubber_machine(Motor, 200.0, 600.0, **ratios)
        stacked = rubber_machine(Motor, 200.0, 3 * 600.0, **ratios)
        plain = DatabaseMassModel(**exact)
        self.assertAlmostEqual(float(plain.mass_kg(stacked)), 3 * float(plain.mass_kg(unit)), places=4)
        model = DatabaseMassModel(torque_continuous_max_stack_Nm=0.5 * 2.5 * 600.0, mass_overhead_stack_kg=1.5, **exact)
        self.assertAlmostEqual(float(model.count_stacks(unit)), 1.0)
        self.assertAlmostEqual(float(model.count_stacks(stacked)), 3.0)
        self.assertAlmostEqual(float(model.mass_kg(stacked)), 3 * float(model.mass_kg(unit)), places=4)
        self.assertAlmostEqual(float(model.mass_kg(stacked) - plain.mass_kg(stacked)), 4.5, places=6)
        self.assertEqual(plain.count_stacks(stacked), 1.0)

    def test_generator_uses_the_same_model(self):
        model = DatabaseMassModel()
        generator = rubber_machine(Generator, 400.0, 1000.0, mass_model=model, **ratios)
        motor = rubber_machine(Motor, 400.0, 1000.0, mass_model=model, **ratios)
        self.assertAlmostEqual(float(generator.get_mass()), float(motor.get_mass()))

    def test_symbolic_through_opti(self):
        """Minimum machine mass at fixed rated power over speed: the cap binds (mass falls with speed until then)."""
        opti = asb.Opti()
        speed_rad_s = opti.variable(init_guess=300.0, lower_bound=50.0, upper_bound=5000.0)
        model = DatabaseMassModel(specific_power_inverter_W_kg=20000.0)
        motor = rubber_machine(Motor, speed_rad_s, 4e5 / speed_rad_s, mass_model=model, **ratios)
        self.assertIsInstance(motor.get_mass(), cas.MX)
        opti.minimize(motor.get_mass())
        solution = opti.solve(verbose=False)
        power_W = 1.25 * 4e5
        self.assertAlmostEqual(solution.value(motor.get_mass()), power_W / 20000.0 * 2, delta=2.0 * np.log(2) + 0.5)


if __name__ == "__main__":
    unittest.main()
