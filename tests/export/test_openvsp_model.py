"""OpenVSP build of the Halo outer mold line (skipped without the OpenVSP Python API)."""
import tempfile
import unittest
from pathlib import Path

try:
    import openvsp as vsp
except ImportError:
    vsp = None

from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot


@unittest.skipIf(vsp is None, "OpenVSP Python API not installed (see README)")
class OpenvspModelTests(unittest.TestCase):
    snapshot = halo_plan027_snapshot()

    def build(self, angle_nacelle_deg):
        from aircraft_closure.export.openvsp.model import build_openvsp_model
        return build_openvsp_model(self.snapshot, angle_nacelle_deg)

    @staticmethod
    def location_m(geom_id):
        return tuple(vsp.GetParmVal(geom_id, f"{axis}_Location", "XForm") for axis in "XYZ")

    def test_tree_follows_the_skeleton(self):
        geoms = self.build(90.0)
        parent = {name: vsp.GetGeomParent(geom_id) for name, geom_id in geoms.items()}
        self.assertEqual(parent["Wing"], geoms["Fuselage"])
        self.assertEqual(parent["NacelleTilt"], geoms["Wing"])
        self.assertEqual(parent["Nacelle"], geoms["NacelleTilt"])
        self.assertEqual(parent["Rotor"], geoms["Nacelle"])
        self.assertEqual(parent["RotorTipPath"], geoms["Rotor"])
        self.assertEqual(vsp.GetGeomTypeName(geoms["NacelleTilt"]), "Hinge")

    def test_hub_follows_the_nacelle_angle(self):
        x_spindle_m, y_spindle_m, z_spindle_m = self.snapshot.spindle_xyz_m()
        length_mast_m = self.snapshot.nacelle.length_mast_m
        for angle_nacelle_deg, (dx_m, dz_m) in ((90.0, (0.0, length_mast_m)), (0.0, (-length_mast_m, 0.0))):
            geoms = self.build(angle_nacelle_deg)
            x_m, y_m, z_m = self.location_m(geoms["Rotor"])
            self.assertAlmostEqual(x_m, x_spindle_m + dx_m, places=6, msg=angle_nacelle_deg)
            self.assertAlmostEqual(y_m, y_spindle_m, places=6)
            self.assertAlmostEqual(z_m, z_spindle_m + dz_m, places=6, msg=angle_nacelle_deg)

    def test_rotor_diameter_and_blades(self):
        geoms = self.build(90.0)
        self.assertAlmostEqual(vsp.GetParmVal(geoms["Rotor"], "Diameter", "Design"), 2 * self.snapshot.rotor.radius_m)
        self.assertEqual(int(vsp.GetParmVal(geoms["Rotor"], "NumBlade", "Design")), self.snapshot.rotor.count_blades)

    def test_aerosandbox_export_matches_the_openvsp_surfaces(self):
        """Sweep and span conventions agree: trailing-edge extremes of wing and V-tail within 1 cm."""
        geoms = self.build(0.0)
        wing, v_tail = self.snapshot.to_asb().wings
        x_max_wing_m = vsp.GetGeomBBoxMax(geoms["Wing"], 0, True).x()
        self.assertAlmostEqual(x_max_wing_m, wing.xsecs[0].xyz_le[0] + wing.xsecs[0].chord, delta=0.01)
        x_max_v_tail_m = vsp.GetGeomBBoxMax(geoms["VTail"], 0, True).x()
        self.assertAlmostEqual(x_max_v_tail_m, v_tail.xsecs[-1].xyz_le[0] + v_tail.xsecs[-1].chord, delta=0.01)

    def test_exports_are_written_and_mirrored(self):
        from aircraft_closure.export.openvsp.model import export_outer_mold_line
        with tempfile.TemporaryDirectory() as directory:
            paths = export_outer_mold_line(self.snapshot, directory, angles_nacelle_deg=(0.0,))
            for path in paths.values():
                self.assertGreater(Path(path).stat().st_size, 0, msg=str(path))
            vertices = [line.split()[1:4] for line in Path(paths[("stl", 0.0)]).read_text().splitlines()
                        if line.strip().startswith("vertex")]
            y_m = [float(vertex[1]) for vertex in vertices]
            tip_m = self.snapshot.wing.span_m / 2 + self.snapshot.rotor.radius_m
            self.assertAlmostEqual(max(y_m), tip_m, delta=0.05)
            self.assertAlmostEqual(min(y_m), -tip_m, delta=0.05)


if __name__ == "__main__":
    unittest.main()
