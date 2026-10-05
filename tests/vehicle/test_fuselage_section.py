"""Plan 032: optional super-ellipse (boxy) fuselage section."""
import unittest

import aerosandbox as asb

from aircraft_closure.vehicle.fuselage import Fuselage


class FuselageSectionTests(unittest.TestCase):
    def test_round_by_default(self):
        fuselage = Fuselage(length_m=10.0, diameter_m=1.6)
        self.assertIsNone(fuselage.height_m)
        self.assertEqual(fuselage.diameter_equivalent_m(), 1.6)
        self.assertTrue(all(xsec.width == xsec.height for xsec in fuselage.to_asb().xsecs))

    def test_elliptic_section_of_equal_axes_is_the_round_one(self):
        round_ = Fuselage(length_m=10.0, diameter_m=1.6).to_asb()
        ellipse = Fuselage(length_m=10.0, diameter_m=1.6, height_m=1.6, shape=2.0).to_asb()
        self.assertAlmostEqual(float(ellipse.area_wetted()), float(round_.area_wetted()), places=9)
        self.assertAlmostEqual(float(ellipse.volume()), float(round_.volume()), places=9)

    def test_boxier_section_has_more_volume_and_area(self):
        ellipse = Fuselage(length_m=10.0, diameter_m=1.6, height_m=2.0, shape=2.0).to_asb()
        boxy = Fuselage(length_m=10.0, diameter_m=1.6, height_m=2.0, shape=3.2).to_asb()
        self.assertGreater(float(boxy.volume()), float(ellipse.volume()))
        self.assertGreater(float(boxy.area_wetted()), float(ellipse.area_wetted()))

    def test_equivalent_diameter_and_symbolic_height(self):
        self.assertAlmostEqual(float(Fuselage(diameter_m=1.6, height_m=2.5).diameter_equivalent_m()), 2.0, places=12)
        opti = asb.Opti()
        height_m = opti.variable(init_guess=2.0, lower_bound=1.0)
        volume_m3 = Fuselage(length_m=10.0, diameter_m=1.6, height_m=height_m, shape=3.0).to_asb().volume()
        opti.subject_to(volume_m3 == 25.0)
        solution = opti.solve(verbose=False)
        self.assertAlmostEqual(float(solution.value(volume_m3)), 25.0, places=6)


if __name__ == "__main__":
    unittest.main()
