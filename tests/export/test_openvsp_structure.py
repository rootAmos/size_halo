"""OpenVSP internal structure of the Halo (skipped without the OpenVSP Python API)."""
import tempfile
import unittest
from pathlib import Path

try:
    import openvsp as vsp
except ImportError:
    vsp = None

from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot


@unittest.skipIf(vsp is None, "OpenVSP Python API not installed (see README)")
class OpenvspStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from aircraft_closure.export.openvsp.structure import build_structure
        cls.geoms, cls.structures = build_structure(halo_plan027_snapshot())

    def part_names(self, name):
        return [vsp.GetFeaPartName(part_id) for part_id in vsp.GetFeaPartIDVec(self.structures[name])]

    def test_structures_and_parts(self):
        self.assertEqual(set(self.structures), {"WingBox", "Fuselage", "VTail"})
        self.assertEqual(self.part_names("WingBox"),
                         ["Skin", "FrontSpar", "RearSpar", "Ribs", "FairingAttachRib", "NacelleRib"])
        self.assertEqual(self.part_names("Fuselage"),
                         ["Skin", "Frames", "NoseBulkhead", "FrontSparBulkhead", "RearSparBulkhead",
                          "CabinEndBulkhead", "Floor"])
        self.assertEqual(self.part_names("VTail"), ["Skin", "FrontSpar", "RearSpar", "Ribs"])

    def test_v_tail_meshes_and_writes_decks(self):
        """The smallest structure: STL, CalculiX, Nastran and mass report, mirrored panels, mass reported."""
        from aircraft_closure.export.openvsp.structure import export_structure_meshes
        with tempfile.TemporaryDirectory() as directory:
            paths = export_structure_meshes({"VTail": self.structures["VTail"]}, directory, length_max_m=0.3,
                                            length_min_m=0.06)
            for path in paths.values():
                self.assertGreater(Path(path).stat().st_size, 0, msg=str(path))
            solids = [line.split()[1] for line in Path(paths[("VTail", "stl")]).read_text().splitlines()
                      if line.startswith("solid")]
            self.assertIn("FrontSpar_0", solids)
            self.assertIn("FrontSpar_1", solids)
            self.assertIn("*NODE", Path(paths[("VTail", "calculix")]).read_text())
            mass_kg = float(Path(paths[("VTail", "mass")]).read_text().split()[-1])
            self.assertGreater(mass_kg, 0.0)


if __name__ == "__main__":
    unittest.main()
