"""Plan 043: reduced-order spar-cap strain at ultimate load and the added spar caps."""
import unittest

import aerosandbox as asb
import aerosandbox.tools.units as u
import casadi as cas

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.surfaces import TiltrotorWingMassModel, Wing, graphite_epoxy_wing_material
from aircraft_closure.weights import afdd
from tests.weights.test_tiltrotor_wing import inputs, masses

# The v3.3.5 wing (plan 035/038 reference, 13,038 lb) the last CalculiX check ran on, and its result
# (halo-plan035/output/structure/reference.json and output/fe_wing/summary.json).
v335 = dict(mass_takeoff_kg=5913.911639845456, span_wing_m=11.238345561543689, area_wing_m2=20.63732205239685,
            width_fuselage_m=1.6764, mass_tip_kg=567.0049107430481, ratio_depth_spar_cap=0.826,
            area_torque_box_m2=0.002356559676558735, area_spar_m2=0.002798889592561221,
            modulus_torque_box_Pa=62052803149.60629, modulus_spar_Pa=124105606299.21259, factor_wing=1.3273944247612506)
strain_fe_v335, strain_allowable = 0.005179369814814815, 0.0047
factor_peak = 1.3324                     # HaloAssumptions.factor_peak_strain_cap_wing


def strain_v335(factor_load_carrying=1.0):
    v = v335
    moment_Nm = afdd.moment_jump_station_ultimate_Nm(v["mass_takeoff_kg"], v["mass_tip_kg"], v["span_wing_m"],
                                                     v["width_fuselage_m"], 0.3)
    return afdd.strain_cap_jump_ultimate(v["area_wing_m2"] / v["span_wing_m"], 0.23, 0.45, v["ratio_depth_spar_cap"],
                                         v["area_torque_box_m2"], v["area_spar_m2"], v["modulus_torque_box_Pa"],
                                         v["modulus_spar_Pa"], moment_Nm, factor_load_carrying, factor_peak)


class StrainModelTests(unittest.TestCase):
    def test_beam_theory_by_hand(self):
        # 1 m chord, t/c 0.2, box 0.5 c, caps the full depth apart: d = 0.2, w = 0.5, wall t = A / (2 (w + d)).
        area_box_m2, area_caps_m2, modulus_Pa, moment_Nm = 0.0014, 0.002, 70e9, 1e5
        t_m = area_box_m2 / 1.4
        stiffness_Nm2 = modulus_Pa * (2 * t_m * 0.5 * 0.1**2 + 2 * t_m * 0.2**3 / 12 + area_caps_m2 * 0.1**2)
        beam, peak = afdd.strain_cap_jump_ultimate(1.0, 0.2, 0.5, 1.0, area_box_m2, area_caps_m2, modulus_Pa,
                                                   modulus_Pa, moment_Nm, factor_peak=1.5)
        self.assertAlmostEqual(beam / (moment_Nm * 0.1 / stiffness_Nm2), 1.0, places=12)
        self.assertAlmostEqual(peak / beam, 1.5, places=12)

    def test_load_carrying_factor_divides_strain(self):
        self.assertAlmostEqual(strain_v335(1.2)[1] * 1.2 / strain_v335(1.0)[1], 1.0, places=12)

    def test_station_moment_by_hand(self):
        g = 9.80665
        moment_Nm = afdd.moment_jump_station_ultimate_Nm(6000.0, 500.0, 12.0, 2.0, 0.5)
        self.assertAlmostEqual(moment_Nm, 1.5 * 2 * g * (3000 - 500) * (6 - 1 - 0.5), places=6)

    def test_reproduces_the_calculix_peak(self):
        """The calibrated model reproduces the v3.3.5 FE peak (1.10 x allowable: a negative margin)."""
        beam, peak = strain_v335()
        self.assertAlmostEqual(peak / strain_fe_v335, 1.0, delta=1e-3)
        self.assertAlmostEqual(beam / strain_allowable, 0.828, delta=0.002)   # beam theory alone looks safe

    def test_half_the_calibration_increment_closes_the_v335_margin(self):
        factor = 1 + 0.5 * (v335["factor_wing"] - 1)
        self.assertAlmostEqual(strain_v335(factor)[1] / strain_allowable, 1.102 / factor, delta=0.002)
        self.assertLess(strain_v335(factor)[1], strain_allowable)


class AddedCapTests(unittest.TestCase):
    def test_default_adds_nothing(self):
        self.assertEqual(masses().mass_spar_strain_kg, 0.0)
        self.assertEqual(masses(area_spar_added_m2=0.0).total(), masses().total())

    def test_added_caps_mass_and_stiffness(self):
        base, added = masses(), masses(area_spar_added_m2=1e-3)
        density = inputs["density_spar_kg_m3"]
        self.assertAlmostEqual(added.mass_spar_strain_kg / (1e-3 * density * inputs["span_m"]), 1.0, places=12)
        ratio_fittings = 0.129 / (1 - 0.129)
        self.assertAlmostEqual((added.total() - base.total()) / (added.mass_spar_strain_kg * (1 + ratio_fittings)),
                               1.0, places=9)
        depth_m = 0.23 * inputs["chord_m"]
        self.assertAlmostEqual((added.stiffness_beam_Nm2 - base.stiffness_beam_Nm2)
                               / (inputs["modulus_spar_Pa"] * 1e-3 * depth_m**2 / 4), 1.0, places=9)
        self.assertAlmostEqual(added.area_spar_m2 - base.area_spar_m2, 1e-3, places=12)

    def test_optimizer_adds_just_enough_cap(self):
        """A small Opti: least added cap area for peak strain <= allowable reaches the allowable exactly."""
        model = TiltrotorWingMassModel(mass_tip_kg=600.0, radius_gyration_pylon_m=0.85,
                                       speed_rotor_design_rad_s=56.0, width_fuselage_m=1.68,
                                       ratio_depth_spar_cap=0.826, thickness_min_torque_box_m=0.001,
                                       material=graphite_epoxy_wing_material())
        condition = StructuralDesignCondition(mass_design_kg=6500.0)
        wing = Wing(area_m2=21.0, aspect_ratio=6.12, mass_model=model)
        unconstrained = model.strain_cap_jump_ultimate(model.masses(wing, condition), wing, condition,
                                                       factor_peak=1.6)[1]
        self.assertGreater(unconstrained, 0.0047)         # this wing needs caps
        opti = asb.Opti()
        area_m2 = opti.variable(init_guess=1e-4, scale=1e-3, lower_bound=0.0)
        sized = TiltrotorWingMassModel(**{**model.__dict__, "area_spar_added_m2": area_m2})
        wing_sized = Wing(area_m2=21.0, aspect_ratio=6.12, mass_model=sized)
        w = sized.masses(wing_sized, condition)
        peak = sized.strain_cap_jump_ultimate(w, wing_sized, condition, factor_peak=1.6)[1]
        self.assertIsInstance(peak, cas.MX)
        opti.subject_to(peak <= 0.0047)
        opti.minimize(w.total())
        solution = opti.solve(verbose=False)
        self.assertGreater(solution(area_m2), 0.0)
        self.assertAlmostEqual(solution(peak) / 0.0047, 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
