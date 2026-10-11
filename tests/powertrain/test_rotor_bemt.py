"""Blade-element momentum rotor (plan 047): momentum limits, Prandtl loss, signs, CasADi."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np
import numpy as onp

from aircraft_closure.powertrain.components.rotor_bemt import (AirState, BladeElementRotor, BladeGeometry,
                                                               OperatingPoint, SectionSchedule, add_operating_point,
                                                               interpolation_basis, solve_point, station_fractions)

# Effectively incompressible air and one section along the span, so an ideal-twist blade sees one lift curve.
air = AirState(density_kg_m3=1.225, speed_sound_m_s=3403.0)
uniform_sections = SectionSchedule(fraction_radius=(0.0, 1.0), thickness_to_chord=(0.12, 0.12))
radius_m = 4.0
speed_rad_s = 50.0          # tip Mach 0.59
fraction_root = 0.25


inflow_ideal = 0.04
alpha_ideal_deg = 5.0


def _cl_ideal():
    airfoil = uniform_sections.airfoils([0.5])[0]
    return float(onp.ravel(airfoil.get_aero_from_neuralfoil(alpha=alpha_ideal_deg, Re=5e6, mach=0.0,
                                                            model_size=uniform_sections.model_size)["CL"])[0])


def ideal_hover_blade():
    """Optimum hover rotor (uniform inflow, constant angle of attack): pitch = alpha + atan(lambda / x),
    chord = c_tip / x with c_tip = 8 pi R lambda^2 / (B cl); twist indexed to 0.75 R."""
    fractions = onp.linspace(fraction_root, 1.0, 19)
    chord_tip_m = 8 * onp.pi * radius_m * inflow_ideal**2 / (3 * _cl_ideal())
    pitch = onp.degrees(onp.arctan(inflow_ideal / fractions))
    return BladeGeometry(fraction_control=tuple(fractions), chord_control_m=tuple(chord_tip_m / fractions),
                         twist_control_deg=tuple(pitch - onp.degrees(onp.arctan(inflow_ideal / 0.75))))


def ideal_rotor(**kwargs):
    defaults = dict(radius_m=radius_m, count_blades=3, fraction_root=fraction_root, blade=ideal_hover_blade(),
                    drag_factor=0.0, tip_loss=False, hub_loss=False, stall_delay=False, count_stations=24,
                    sections=uniform_sections)
    defaults.update(kwargs)
    return BladeElementRotor(**defaults)


def hover_point(thrust_N=None):
    """Default thrust: momentum thrust of uniform inflow lambda over the annulus, 2 lambda^2 (1 - x_root^2)."""
    if thrust_N is None:
        thrust_N = (2 * inflow_ideal**2 * (1 - fraction_root**2) * air.density_kg_m3 * onp.pi * radius_m**2
                    * (speed_rad_s * radius_m)**2)
    return OperatingPoint("hover", 0.0, 0.0, speed_rad_s, thrust_N, air=air)


class InterpolationTests(unittest.TestCase):
    def test_basis_recovers_control_values_and_lines(self):
        control = (0.1, 0.4, 0.75, 1.0)
        basis = interpolation_basis(control, onp.array(control))
        onp.testing.assert_allclose(basis, onp.eye(4), atol=1e-12)
        x = onp.linspace(0.1, 1.0, 7)
        onp.testing.assert_allclose(interpolation_basis(control, x) @ (2 + 3 * onp.array(control)), 2 + 3 * x, atol=1e-12)

    def test_stations_cover_the_blade(self):
        fractions, widths = station_fractions(0.1, 12)
        self.assertAlmostEqual(widths.sum(), 0.9, places=12)
        self.assertTrue(onp.all((fractions > 0.1) & (fractions < 1.0)))

    def test_twist_is_zero_at_three_quarter_radius(self):
        blade = BladeGeometry(twist_control_deg=(30.0, 12.0, 4.0, -6.0))
        self.assertAlmostEqual(float(blade.twist_deg(onp.array([0.75]))[0]), 0.0, places=10)


class MomentumLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rotor = ideal_rotor()
        cls.result, cls.collective = solve_point(cls.rotor, hover_point())

    def test_residual_closed_and_thrust_trimmed(self):
        self.assertLess(onp.max(onp.abs(self.result.residual)), 1e-7)
        self.assertAlmostEqual(float(self.result.thrust_N), hover_point().thrust_N, delta=1e-3)

    def test_annulus_momentum_balance(self):
        """Zero axial speed, no loss: blade-element thrust equals 4 pi rho r u^2 per unit span, u = W sin phi."""
        r = self.result
        fractions, widths = station_fractions(fraction_root, self.rotor.count_stations)
        chord_m = self.rotor.blade.chord_m(fractions)
        phi = r.angle_inflow_rad
        sigma_local = 3 * chord_m / (2 * onp.pi * fractions * radius_m)
        normal = r.cl * onp.cos(phi)
        # Residual in hover: sin^2 phi = sigma' cn / 4 (F = 1), so the blade-element and momentum forms agree.
        sin_abs = onp.sqrt(onp.sin(phi)**2 + self.rotor.sin_inflow_smoothing**2)
        onp.testing.assert_allclose(onp.sin(phi) * sin_abs, sigma_local * normal / 4, rtol=1e-6, atol=1e-9)
        # The smoothing of |sin phi| changes the momentum side by less than 0.3 % here.
        onp.testing.assert_allclose(onp.sin(phi)**2, sigma_local * normal / 4, rtol=3e-3)

    def test_ideal_twist_without_drag_reaches_the_momentum_limit(self):
        """Uniform inflow over the annulus root..tip: FM -> sqrt(1 - x_root^2) less a small swirl loss."""
        limit = onp.sqrt(1 - fraction_root**2)
        self.assertLessEqual(float(self.result.figure_of_merit), limit + 1e-3)
        self.assertGreater(float(self.result.figure_of_merit), limit - 0.03)

    def test_inflow_is_near_uniform_for_the_ideal_blade(self):
        fractions, _ = station_fractions(fraction_root, self.rotor.count_stations)
        inflow = onp.tan(self.result.angle_inflow_rad) * fractions
        self.assertLess(onp.std(inflow) / onp.mean(inflow), 0.03)
        self.assertAlmostEqual(float(onp.mean(inflow)), inflow_ideal, delta=0.03 * inflow_ideal)

    def test_signs(self):
        self.assertGreater(float(self.result.thrust_N), 0)
        self.assertGreater(float(self.result.torque_Nm), 0)
        self.assertEqual(float(self.result.efficiency_propulsive), 0.0)


class ProfileAndLossTests(unittest.TestCase):
    def test_profile_drag_lowers_figure_of_merit(self):
        clean, _ = solve_point(ideal_rotor(), hover_point())
        draggy, _ = solve_point(ideal_rotor(drag_factor=1.0), hover_point())
        self.assertLess(float(draggy.figure_of_merit), float(clean.figure_of_merit) - 0.05)

    def test_prandtl_tip_loss_costs_power(self):
        no_loss, _ = solve_point(ideal_rotor(), hover_point())
        loss, _ = solve_point(ideal_rotor(tip_loss=True), hover_point())
        self.assertGreater(float(loss.shaft_power_W), float(no_loss.shaft_power_W) * 1.01)

    def test_prandtl_loss_fades_with_blade_count(self):
        """Same solidity, more blades: the tip-loss power penalty shrinks."""
        def penalty(count_blades):
            base_blade = ideal_hover_blade()
            blade = replace(base_blade, chord_control_m=tuple(c * 3 / count_blades for c in base_blade.chord_control_m))
            base, _ = solve_point(ideal_rotor(count_blades=count_blades, blade=blade), hover_point())
            lossy, _ = solve_point(ideal_rotor(count_blades=count_blades, blade=blade, tip_loss=True), hover_point())
            return float(lossy.shaft_power_W) / float(base.shaft_power_W) - 1
        self.assertGreater(penalty(2), penalty(4))
        self.assertGreater(penalty(4), penalty(12))


class AxialFlightTests(unittest.TestCase):
    def test_efficiency_below_ideal_momentum_value(self):
        """Propeller mode: eta <= 2 / (1 + sqrt(1 + T / (q A))), approached without drag or loss."""
        velocity_m_s = 60.0
        fractions = onp.linspace(0.1, 1.0, 10)
        pitch_deg = onp.degrees(onp.arctan(velocity_m_s / (speed_rad_s * radius_m * fractions)))
        pitch_75_deg = onp.degrees(onp.arctan(velocity_m_s / (speed_rad_s * radius_m * 0.75)))
        cruise_blade = BladeGeometry(fraction_control=tuple(fractions), chord_control_m=(0.3,) * 10,
                                     twist_control_deg=tuple(pitch_deg - pitch_75_deg))
        rotor = ideal_rotor(blade=cruise_blade)
        thrust_N = 3000.0
        point = OperatingPoint("cruise", velocity_m_s, 0.0, speed_rad_s, thrust_N, air=air)
        result, _ = solve_point(rotor, point)
        area_m2 = onp.pi * radius_m**2 * (1 - fraction_root**2)
        ideal = 2 / (1 + onp.sqrt(1 + thrust_N / (0.5 * air.density_kg_m3 * velocity_m_s**2 * area_m2)))
        self.assertLessEqual(float(result.efficiency_propulsive), ideal + 1e-3)
        self.assertGreater(float(result.efficiency_propulsive), ideal - 0.03)
        lossy, _ = solve_point(replace(rotor, drag_factor=1.0, tip_loss=True, hub_loss=True), point)
        self.assertLess(float(lossy.efficiency_propulsive), float(result.efficiency_propulsive))


class CasadiTests(unittest.TestCase):
    def test_geometry_as_opti_variables(self):
        """Chord and twist control values and collective as variables: maximise FM at fixed thrust."""
        opti = asb.Opti()
        chord_m = opti.variable(init_guess=0.3, lower_bound=0.15, upper_bound=0.5)
        twist_root_deg = opti.variable(init_guess=20.0, lower_bound=0.0, upper_bound=40.0)
        blade = BladeGeometry(fraction_control=(0.25, 0.75, 1.0), chord_control_m=[chord_m, chord_m, chord_m],
                              twist_control_deg=[twist_root_deg, 0.0, -5.0], interpolation="linear")
        rotor = BladeElementRotor(radius_m=radius_m, fraction_root=fraction_root, blade=blade, count_stations=10)
        result, collective = add_operating_point(opti, rotor, hover_point(15000.0), collective_guess_deg=10.0)
        opti.subject_to(result.blade_loading <= 0.12)
        opti.minimize(-result.figure_of_merit)
        solution = opti.solve(verbose=False)
        self.assertTrue(0.15 <= float(solution(chord_m)) <= 0.5)
        self.assertGreater(float(solution(result.figure_of_merit)), 0.6)


if __name__ == "__main__":
    unittest.main()
