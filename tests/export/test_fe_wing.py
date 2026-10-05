"""FE wing-box property mapping and deck reading (no OpenVSP or CalculiX needed)."""
import tempfile
import unittest
from pathlib import Path

from aircraft_closure.export.openvsp.fe_wing import properties_from_afdd, read_mesh
from aircraft_closure.export.openvsp.structure_check import naca_four_digit_thickness

afdd = dict(area_torque_box_m2=0.0027, area_spar_m2=0.0018, mass_fairing_kg=68.0)
material = dict(modulus_shear_torque_box_Pa=25.9e9, modulus_torque_box_Pa=62.1e9, density_torque_box_kg_m3=1661.0,
                modulus_spar_Pa=124.1e9, density_spar_kg_m3=1661.0)


class PropertyMappingTests(unittest.TestCase):
    chord_m, span_m, width_fuselage_m = 1.9, 11.5, 1.68
    p = properties_from_afdd(afdd, material, 1.9, 0.23, 0.15, 0.60, 11.5, 1.68)

    def test_box_wall_area_is_the_afdd_area(self):
        height_m = 0.5 * (naca_four_digit_thickness(0.15, 0.23) + naca_four_digit_thickness(0.60, 0.23)) * self.chord_m
        perimeter_m = 2 * (0.45 * self.chord_m + height_m)
        self.assertAlmostEqual(self.p.thickness_box_m * perimeter_m, afdd["area_torque_box_m2"], places=12)

    def test_four_cap_strips_carry_the_afdd_cap_area(self):
        self.assertAlmostEqual(4 * self.p.thickness_cap_pad_m * self.p.width_cap_strip_m, afdd["area_spar_m2"],
                               places=12)

    def test_fairing_carries_the_afdd_fairing_mass(self):
        area_m2 = 2 * (1 - 0.45) * self.chord_m * (self.span_m - self.width_fuselage_m)
        self.assertAlmostEqual(area_m2 * self.p.thickness_fairing_m * self.p.density_fairing_kg_m3, 68.0, places=9)

    def test_isotropic_shell_reproduces_the_laminate_shear_modulus(self):
        shear_Pa = self.p.modulus_box_Pa / (2 * (1 + self.p.poisson_box))
        self.assertAlmostEqual(shear_Pa / material["modulus_shear_torque_box_Pa"], 1.0, places=9)


class DeckReadingTests(unittest.TestCase):
    def test_reads_nodes_and_element_sets(self):
        deck = """** comment
*NODE, NSET=NSkin
1,0.0,0.0,0.0
2,1.0,0.0,0.0
3,1.0,1.0,0.0
4,0.0,1.0,0.0
*ELEMENT, TYPE=S4, ELSET=ESkin_WingBox_0
10,1,2,3,4
*SHELL SECTION, ELSET=ESkin_WingBox_0, MATERIAL=X
0.001
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "deck.inp"
            path.write_text(deck)
            nodes, elsets = read_mesh(path)
        self.assertEqual(sorted(nodes), [1, 2, 3, 4])
        self.assertEqual(elsets["ESkin_WingBox_0"], ("S4", [(10, [1, 2, 3, 4])]))


if __name__ == "__main__":
    unittest.main()
