"""GeometrySnapshot identities (no OpenVSP needed)."""
import math
import unittest

from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot, v_tail_equivalent


class VTailEquivalentTests(unittest.TestCase):
    def test_projected_areas_reproduce_the_conventional_tail(self):
        area_horizontal_m2, area_vertical_m2 = 3.7, 1.6
        tail = v_tail_equivalent(area_horizontal_m2, area_vertical_m2, aspect_ratio=3.3, taper_ratio=0.5,
                                 sweep_le_deg=30.0, x_le_root_m=9.0, z_m=0.8, thickness_to_chord=0.12)
        area_m2 = 0.5 * (tail.chord_root_m + tail.chord_tip_m) * tail.span_m
        dihedral_rad = math.radians(tail.dihedral_deg)
        self.assertAlmostEqual(area_m2, area_horizontal_m2 + area_vertical_m2, places=9)
        self.assertAlmostEqual(area_m2 * math.cos(dihedral_rad) ** 2, area_horizontal_m2, places=9)
        self.assertAlmostEqual(area_m2 * math.sin(dihedral_rad) ** 2, area_vertical_m2, places=9)
        self.assertAlmostEqual(tail.span_m ** 2 / area_m2, 3.3, places=9)

    def test_limiting_dihedrals(self):
        self.assertAlmostEqual(v_tail_equivalent(1.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.1).dihedral_deg, 0.0, places=12)
        self.assertAlmostEqual(v_tail_equivalent(1e-12, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.1).dihedral_deg, 90.0,
                               delta=1e-3)
        self.assertAlmostEqual(v_tail_equivalent(1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.1).dihedral_deg, 45.0, places=12)


class HaloSnapshotTests(unittest.TestCase):
    snapshot = halo_plan027_snapshot()

    def test_tapered_wing_keeps_sized_area_span_and_quarter_chord(self):
        wing = self.snapshot.wing
        self.assertAlmostEqual(0.5 * (wing.chord_root_m + wing.chord_tip_m) * wing.span_m, 21.388, places=9)
        self.assertAlmostEqual(wing.chord_tip_m / wing.chord_root_m, 0.6, places=12)
        x_quarter_root_m = wing.x_le_root_m + 0.25 * wing.chord_root_m
        x_quarter_tip_m = wing.x_le_tip_m() + 0.25 * wing.chord_tip_m
        self.assertAlmostEqual(x_quarter_root_m, x_quarter_tip_m, places=12)
        self.assertAlmostEqual(x_quarter_root_m, 4.079 + 0.25 * 21.388 / 11.441, places=12)

    def test_spindle_at_the_tip_quarter_chord(self):
        wing, nacelle = self.snapshot.wing, self.snapshot.nacelle
        x_m, y_m, z_m = self.snapshot.spindle_xyz_m()
        self.assertAlmostEqual(x_m, wing.x_le_tip_m() + nacelle.fraction_chord_spindle * wing.chord_tip_m, places=12)
        self.assertAlmostEqual(y_m, wing.span_m / 2, places=12)
        self.assertAlmostEqual(z_m, wing.z_m + nacelle.offset_z_spindle_m, places=12)

    def test_rotor_clears_the_fuselage_side(self):
        """Tip-to-fuselage gap at the plan 027 reference (the sizing margin uses the 1.68 m width)."""
        width_fuselage_m = max(station.width_m for station in self.snapshot.fuselage.stations)
        gap_m = self.snapshot.wing.span_m / 2 - self.snapshot.rotor.radius_m - width_fuselage_m / 2
        self.assertAlmostEqual(gap_m, 0.3, delta=0.01)

    def test_body_stations_increase_along_the_body(self):
        for body in (self.snapshot.fuselage, self.snapshot.wing_fairing):
            x_m = [station.x_m for station in body.stations]
            self.assertEqual(x_m, sorted(x_m))
            self.assertTrue(all(station.width_m > 0 and station.height_m > 0 for station in body.stations))

    def test_aerosandbox_airplane_matches_the_snapshot(self):
        airplane = self.snapshot.to_asb()
        wing, v_tail = airplane.wings
        self.assertAlmostEqual(float(wing.area()), 21.388, places=6)
        self.assertAlmostEqual(airplane.s_ref, 21.388, places=9)
        self.assertAlmostEqual(airplane.c_ref, float(wing.mean_aerodynamic_chord()), places=6)
        area_v_tail_m2 = float(v_tail.area())
        dihedral_rad = math.radians(self.snapshot.v_tail.dihedral_deg)
        self.assertAlmostEqual(area_v_tail_m2 * math.cos(dihedral_rad) ** 2, 3.732, places=6)
        self.assertAlmostEqual(area_v_tail_m2 * math.sin(dihedral_rad) ** 2, 1.658, places=6)
        names = [fuselage.name for fuselage in airplane.fuselages]
        self.assertEqual(names, ["fuselage", "wing_fairing", "nacelle_right", "nacelle_left"])
        x_spindle_m, y_spindle_m, z_spindle_m = self.snapshot.spindle_xyz_m()
        nose = airplane.fuselages[2].xsecs[0].xyz_c
        nacelle = self.snapshot.nacelle
        self.assertAlmostEqual(nose[0], x_spindle_m - nacelle.length_mast_m - nacelle.offset_nose_m, places=9)
        self.assertAlmostEqual(nose[1], y_spindle_m, places=9)
        self.assertAlmostEqual(nose[2], z_spindle_m, places=9)


if __name__ == "__main__":
    unittest.main()
