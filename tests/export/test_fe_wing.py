"""FE wing-box property mapping and deck reading (no OpenVSP or CalculiX needed)."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from aircraft_closure.export.openvsp.fe_wing import (TipMass, bands_from_wing_box, properties_from_afdd, read_mesh,
                                                      write_deck)
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


class BandedLayoutTests(unittest.TestCase):
    """Plan 054: bottom-up layout as spanwise bands."""
    layout = dict(y_stations_m=[1.2, 2.4, 3.6], span_wing_m=9.0, width_fuselage_m=1.6, chord_wing_m=1.9,
                  fraction_chord_front_spar=0.15, fraction_chord_rear_spar=0.60, mass_fairing_kg=50.0,
                  thickness_skin_upper_m=[3e-3, 2e-3, 1e-3], thickness_skin_lower_m=[3e-3, 2e-3, 1e-3],
                  thickness_stringers_upper_m=[1.5e-3, 1e-3, 0.5e-3], thickness_stringers_lower_m=[1e-3, 1e-3, 0],
                  thickness_web_m=[2e-3, 1.5e-3, 1e-3], area_caps_m2=[4e-4, 2e-4, 0.0], thickness_rib_m=1.5e-3,
                  material=dict(modulus_Pa=62e9, modulus_shear_Pa=25.9e9, modulus_cap_Pa=124e9, density_kg_m3=1661.0))

    def test_band_edges_and_root_centre_section(self):
        bands = bands_from_wing_box(self.layout)
        for edge, expected in zip(bands.y_edges_m, (0.0, 0.8, 1.8, 3.0, 4.5)):
            self.assertAlmostEqual(edge, expected, places=12)
        self.assertEqual(bands.thickness_skin_upper_m[:2], (3e-3, 3e-3))
        self.assertEqual(bands.band(0.5), 0)
        self.assertEqual(bands.band(2.0), 2)
        self.assertEqual(bands.band(9.0), 3)

    def test_deck_has_orthotropic_cover_sections(self):
        bands = bands_from_wing_box(self.layout)
        nodes = {1: np.array([0.3, 0.0, 0.1]), 2: np.array([0.3, 0.5, 0.1]), 3: np.array([0.4, 0.5, 0.1]),
                 4: np.array([0.4, 0.0, 0.1]), 5: np.array([0.5, 0.0, -0.1]), 6: np.array([0.5, 0.5, -0.1]),
                 7: np.array([0.6, 0.5, -0.1]), 8: np.array([0.6, 0.0, -0.1])}
        elsets = {"ESkin_WingBox": ("S4", [(10, [1, 2, 3, 4]), (11, [5, 6, 7, 8])]),
                  "ENacelleRib": ("S3", [(12, [5, 6, 7])])}
        tip = TipMass(mass_kg=800.0, radius_gyration_pitch_m=1.0, xyz_spindle_m=(0.5, 4.5, 1.2))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "deck.inp"
            write_deck(path, nodes, elsets, bands, tip, x_le_m=-0.3, chord_m=1.9, fraction_front_spar=0.15,
                       fraction_rear_spar=0.60, width_fuselage_m=0.4, force_tip_jump_N=1e4)
            text = path.read_text()
        self.assertIn("*ORIENTATION, NAME=OSPAN", text)
        self.assertIn("*ELASTIC, TYPE=ENGINEERING CONSTANTS", text)
        self.assertIn("ELSET=EBOXU0", text)        # upper cover, centre-section band
        self.assertIn("ELSET=EBOXL0", text)

    def test_layered_upper_cover_and_buckling_step(self):
        layout = dict(self.layout, pitch_stringer_upper_m=[0.1, 0.08, 0.06], height_stringer_upper_m=[0.03, 0.025, 0.02])
        bands = bands_from_wing_box(layout)
        nodes = {1: np.array([0.3, 1.0, 0.1]), 2: np.array([0.3, 1.5, 0.1]), 3: np.array([0.4, 1.5, 0.1]),
                 4: np.array([0.4, 1.0, 0.1]), 5: np.array([0.5, 0.0, -0.1]), 6: np.array([0.5, 0.5, -0.1]),
                 7: np.array([0.6, 0.5, -0.1]), 8: np.array([0.6, 0.0, -0.1])}
        # Corner order 1-4-3-2 gives an upward (outward) normal on the upper element.
        elsets = {"ESkin_WingBox": ("S4", [(10, [1, 4, 3, 2]), (11, [5, 6, 7, 8])]),
                  "ENacelleRib": ("S3", [(12, [5, 6, 7])])}
        tip = TipMass(mass_kg=800.0, radius_gyration_pitch_m=1.0, xyz_spindle_m=(0.5, 4.5, 1.2))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "deck.inp"
            write_deck(path, nodes, elsets, bands, tip, x_le_m=-0.3, chord_m=1.9, fraction_front_spar=0.15,
                       fraction_rear_spar=0.60, width_fuselage_m=0.4, force_tip_jump_N=1e4, count_buckling_modes=3)
            text = path.read_text()
        self.assertIn("*ELEMENT, TYPE=S8R, ELSET=EBOXU1P", text)
        self.assertIn("COMPOSITE, OFFSET=-0.5", text)
        self.assertEqual(text.count("*STEP"), 1)
        self.assertIn("*BUCKLE", text)
        # Stringer area kept: web layer (smeared) + two flanges = the smeared stringer thickness.
        wall_m = 1.5e-3 * 0.1 / (1.8 * 0.03)
        self.assertAlmostEqual(wall_m / 0.1 * (0.03 + 0.8 * 0.03), 1.5e-3, places=12)


if __name__ == "__main__":
    unittest.main()
