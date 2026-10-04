"""Layout-based primary-structure sizing: identities, limits and trends (no OpenVSP needed)."""
import math
import unittest

import numpy as np

from aircraft_closure.export.openvsp.structure_check import (BoxMaterial, SurfaceLoads, elliptic_panel_loads,
                                                             naca_four_digit_thickness, size_box, size_fuselage)

material = BoxMaterial(density_box_kg_m3=1600.0, modulus_box_Pa=50e9, modulus_shear_box_Pa=20e9,
                       density_cap_kg_m3=1600.0, modulus_cap_Pa=120e9, strain_ultimate=0.004)


class GeometryTests(unittest.TestCase):
    def test_naca_thickness_peaks_at_about_thirty_percent(self):
        x = np.linspace(0.01, 1.0, 1000)
        thickness = naca_four_digit_thickness(x, 0.23)
        self.assertAlmostEqual(thickness.max(), 0.23, delta=0.002)
        self.assertAlmostEqual(x[np.argmax(thickness)], 0.30, delta=0.01)


class LoadTests(unittest.TestCase):
    def test_elliptic_root_shear_and_moment(self):
        semispan_m, lift_N = 5.0, 1.0e4
        moment_Nm, shear_N = elliptic_panel_loads(np.array([0.0, semispan_m]), semispan_m, lift_N)
        self.assertAlmostEqual(shear_N[0], lift_N, delta=1e-3 * lift_N)
        self.assertAlmostEqual(moment_Nm[0], lift_N * 4 * semispan_m / (3 * math.pi), delta=1e-3 * moment_Nm[0])
        self.assertAlmostEqual(moment_Nm[1], 0.0, places=6)

    def test_point_load_superposes(self):
        y_m = np.array([0.0, 1.0])
        moment_Nm, shear_N = elliptic_panel_loads(y_m, 4.0, 0.0, point_loads=((4.0, -500.0),))
        np.testing.assert_allclose(moment_Nm, [-2000.0, -1500.0])
        np.testing.assert_allclose(shear_N, [-500.0, -500.0])


class BoxTests(unittest.TestCase):
    y_m = np.linspace(0.0, 5.0, 11)
    chord_m = np.full(11, 2.0)

    def loads(self, moment_Nm):
        return SurfaceLoads(y_m=self.y_m, moment_Nm=np.full(11, moment_Nm), shear_N=np.zeros(11), case=())

    def test_torsion_driven_skin_meets_bredt(self):
        stiffness_Nm2 = 5.0e7
        box = size_box(self.y_m, self.chord_m, 0.23, 0.15, 0.60, self.loads(0.0), material,
                       stiffness_torsion_Nm2=stiffness_Nm2)
        area_m2 = box.width_box_m * box.height_box_m
        perimeter_m = 2 * (box.width_box_m + box.height_box_m)
        realized_Nm2 = 4 * area_m2**2 * material.modulus_shear_box_Pa * box.thickness_skin_m / perimeter_m
        np.testing.assert_allclose(realized_Nm2, stiffness_Nm2, rtol=1e-12)

    def test_minimum_gauge_without_loads(self):
        box = size_box(self.y_m, self.chord_m, 0.23, 0.15, 0.60, self.loads(0.0), material)
        np.testing.assert_allclose(box.thickness_skin_m, material.thickness_min_m)
        np.testing.assert_allclose(box.area_caps_m2, 0.0)
        self.assertEqual(set(box.driver_caps), {"none"})

    def test_strength_caps_carry_the_moment_beyond_the_skins(self):
        moment_Nm = 5.0e5
        box = size_box(self.y_m, self.chord_m, 0.23, 0.15, 0.60, self.loads(moment_Nm), material)
        stress_cap_Pa = material.modulus_cap_Pa * material.strain_ultimate
        stress_skin_Pa = material.modulus_box_Pa * material.strain_ultimate
        carried_Nm = (box.area_caps_m2 / 2 * stress_cap_Pa * box.height_box_m
                      + stress_skin_Pa * 2 * box.width_box_m * box.thickness_skin_m * box.height_box_m / 2)
        np.testing.assert_allclose(carried_Nm, moment_Nm, rtol=1e-12)
        self.assertEqual(set(box.driver_caps), {"strength"})

    def test_mass_grows_with_load(self):
        light = size_box(self.y_m, self.chord_m, 0.23, 0.15, 0.60, self.loads(1e5), material)
        heavy = size_box(self.y_m, self.chord_m, 0.23, 0.15, 0.60, self.loads(1e6), material)
        self.assertGreater(heavy.mass_caps_kg, light.mass_caps_kg)


class FuselageTests(unittest.TestCase):
    def test_bending_gauge_identity_and_minimum(self):
        sizing = size_fuselage(50.0, 20.0, 3e-4, 5.0, 8.0, width_m=1.6, height_m=2.0, moment_bending_ultimate_Nm=1e6)
        inertia_m4 = sizing.thickness_skin_bending_m * (2.0**3 / 6 + 1.6 * 2.0**2 / 2)
        self.assertAlmostEqual(1e6 * 1.0 / inertia_m4, 2.0e8, delta=1.0)
        small = size_fuselage(50.0, 20.0, 3e-4, 5.0, 8.0, width_m=1.6, height_m=2.0, moment_bending_ultimate_Nm=1.0)
        self.assertEqual(small.thickness_skin_m, 0.001)
        self.assertAlmostEqual(small.mass_skin_kg, 50.0 * 0.001 * 2780.0)


if __name__ == "__main__":
    unittest.main()
