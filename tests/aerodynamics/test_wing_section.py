"""Plan 046: CST wing-section family, NeuralFoil polars differentiable in thickness, AeroBuildup with it."""
import unittest

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.wing_section import (SectionClMaxSurrogate, WingSectionFamily,
                                                         section_cl_max)


def scalar(x):
    return float(np.asarray(x).ravel()[0])


family = WingSectionFamily()


class WingSectionFamilyTests(unittest.TestCase):
    def test_base_member_is_the_coordinate_airfoil(self):
        """At 23 % and 2 % camber the polar is the one `asb.Airfoil('naca2423')` gives (same CST fit)."""
        reference = asb.Airfoil("naca2423")
        for alpha in (0.0, 4.0, 8.0):
            kwargs = dict(alpha=alpha, Re=1.2e7, mach=0.3, model_size="small")
            ours, theirs = family.airfoil().get_aero_from_neuralfoil(**kwargs), \
                reference.get_aero_from_neuralfoil(**kwargs)
            self.assertAlmostEqual(scalar(ours["CL"]), scalar(theirs["CL"]), delta=2e-3)
            self.assertAlmostEqual(scalar(ours["CD"]) / scalar(theirs["CD"]), 1.0, delta=2e-3)

    def test_thickness_scales_the_section(self):
        x = np.linspace(0, 1, 401)
        for t in (0.15, 0.18, 0.25):
            airfoil = family.airfoil(t)
            self.assertAlmostEqual(float(np.max(airfoil.local_thickness(x))), t, places=6)
            self.assertEqual(airfoil.max_thickness(), t)
        # Symmetric when camber is zero.
        symmetric = family.airfoil(0.2, 0.0)
        self.assertAlmostEqual(float(np.max(np.abs(symmetric.local_camber(x)))), 0.0, places=10)

    def test_profile_drag_rises_with_thickness(self):
        cd = [scalar(family.airfoil(t).get_aero_from_neuralfoil(alpha=2.0, Re=1.2e7, mach=0.3, xtr_upper=0.1,
                                                                 xtr_lower=0.1, model_size="small")["CD"])
              for t in (0.15, 0.18, 0.21, 0.23, 0.25)]
        self.assertTrue(all(b > a for a, b in zip(cd, cd[1:])))

    def test_neuralfoil_polar_inside_opti(self):
        """Thickness as an Opti variable: lift held, drag minimized -> the lower thickness bound; and an equality
        on thickness reproduces the numeric polar."""
        opti = asb.Opti()
        t = opti.variable(init_guess=0.23, lower_bound=0.15, upper_bound=0.28)
        alpha = opti.variable(init_guess=3.0)
        aero = family.airfoil(t).get_aero_from_neuralfoil(alpha=alpha, Re=1.2e7, mach=0.3, model_size="small")
        opti.subject_to(aero["CL"] == 0.5)
        opti.minimize(aero["CD"] * 1e3)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(solution(t), 0.15, places=4)

        opti = asb.Opti()
        t = opti.variable(init_guess=0.2)
        aero = family.airfoil(t).get_aero_from_neuralfoil(alpha=4.0, Re=1.2e7, mach=0.3, model_size="small")
        opti.subject_to(t == 0.21)
        opti.minimize(aero["CD"])
        solution = opti.solve(verbose=False)
        numeric = family.airfoil(0.21).get_aero_from_neuralfoil(alpha=4.0, Re=1.2e7, mach=0.3, model_size="small")
        self.assertAlmostEqual(float(np.ravel(solution(aero["CD"]))[0]), scalar(numeric["CD"]), places=8)

    def test_cl_max_surrogate(self):
        surrogate = SectionClMaxSurrogate()
        self.assertEqual(surrogate.ratio(0.23, 0.02), 1.0)
        self.assertGreater(surrogate.ratio(0.18), surrogate.ratio(0.23))
        self.assertGreater(surrogate.ratio(0.23), surrogate.ratio(0.28))
        reference = section_cl_max(family.airfoil(), 8e6, 0.18)
        for t in (0.18, 0.25):
            self.assertAlmostEqual(surrogate.ratio(t), section_cl_max(family.airfoil(t), 8e6, 0.18) / reference,
                                   delta=0.01)


class BuildupWithSectionFamilyTests(unittest.TestCase):
    def test_symbolic_thickness_through_aerobuildup(self):
        """The AeroBuildup wing profile drag is a CasADi expression in the section thickness and equals the
        numeric evaluation at the solved value; the coordinate airfoil gives the same drag at 23 %."""
        from examples.halo_sizing import HaloAssumptions, HaloDesign, build_halo_aircraft
        from dataclasses import replace
        design = HaloDesign(
            x_le_wing_m=4.62, area_wing_m2=23.3, area_horizontal_tail_m2=5.25, area_vertical_tail_m2=2.73,
            torque_peak_motor_Nm=1617.0, torque_peak_generator_Nm=891.0, power_rated_turboshaft_W=835183.7,
            mass_turboshaft_bare_kg=184.2, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
            area_disk_m2=73.4, mass_fuel_kg=936.0, solidity=0.06, speed_tip_m_s=238.2, speed_peak_motor_rad_s=204.5,
            reduction_ratio=4.53, speed_peak_generator_rad_s=580.5, count_parallel_battery=22.95,
            speed_rotor_wing_design_rad_s=56.3, power_rated_heat_exchanger_W=1.27e5, power_rated_gearbox_W=7.57e5)
        assumptions = replace(HaloAssumptions(), wing_section_model="family", count_units_motor=2,
                              count_units_generator=3)
        aerodynamics = BuildupAerodynamics()
        altitude_m, velocity_m_s = 10000 * u.foot, 85.0

        def wing_cd0(aircraft, alpha=3.0):
            out = aerodynamics.evaluate_buildup(aircraft, velocity_m_s, altitude_m, alpha)
            return next(item.cd0 for item in out.breakdown if item.label == "wing")

        opti = asb.Opti()
        t = opti.variable(init_guess=0.2)
        symbolic = wing_cd0(build_halo_aircraft(replace(design, thickness_to_chord_wing=t), assumptions=assumptions))
        opti.subject_to(t == 0.19)
        opti.minimize(symbolic * 1e3)
        solution = opti.solve(verbose=False)
        numeric = wing_cd0(build_halo_aircraft(replace(design, thickness_to_chord_wing=0.19), assumptions=assumptions))
        self.assertAlmostEqual(float(np.ravel(solution(symbolic))[0]), scalar(numeric), places=8)

        family_23 = scalar(wing_cd0(build_halo_aircraft(design, assumptions=assumptions)))
        naca = scalar(wing_cd0(build_halo_aircraft(design, assumptions=replace(assumptions,
                                                                               wing_section_model="naca2423"))))
        self.assertAlmostEqual(family_23 / naca, 1.0, delta=2e-3)
        self.assertLess(scalar(numeric), family_23)


class SectionBaseTests(unittest.TestCase):
    """Plan 062: the NACA 64A223 base (the XV-15 wing section)."""

    def test_64a223_geometry(self):
        from aircraft_closure.aerodynamics.wing_section import section_airfoil, section_camber
        import numpy as onp
        airfoil = section_airfoil("naca64a223")
        x = onp.linspace(0, 1, 2001)
        thickness = onp.asarray(airfoil.local_thickness(x), dtype=float)
        self.assertAlmostEqual(thickness.max(), 0.23, delta=0.002)
        self.assertAlmostEqual(x[thickness.argmax()], 0.40, delta=0.03)
        self.assertAlmostEqual(section_camber("naca64a223"), 0.0133, delta=0.0005)
        self.assertEqual(section_camber("naca2423"), 0.02)

    def test_family_on_64a223_recovers_its_base(self):
        from aircraft_closure.aerodynamics.wing_section import WingSectionFamily, section_camber
        import numpy as onp
        camber = section_camber("naca64a223")
        member = WingSectionFamily(base_name="naca64a223", camber_base=camber).airfoil(0.23, camber)
        x = onp.linspace(0, 1, 401)
        self.assertAlmostEqual(float(onp.max(member.local_thickness(x))), 0.23, places=6)


if __name__ == "__main__":
    unittest.main()
