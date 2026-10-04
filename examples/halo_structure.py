"""Halo internal structure in OpenVSP: wing box, fuselage frames/bulkheads/floor, V-tail; meshes and a render.

Run from the repo root: `python -m examples.halo_structure`. Writes STL,
CalculiX (.inp) and Nastran (.dat) decks and an OpenVSP mass report per
structure to `output/structure/`, the structure model as a .vsp3, and a
render. Placeholder gauges (plan 031). The fuselage frames take a few minutes
to mesh. Needs OpenVSP and the `geometry` group.
"""
import time
from pathlib import Path

import openvsp as vsp

from aircraft_closure.export.openvsp.model import export_outer_mold_line
from aircraft_closure.export.openvsp.render import render_structure
from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot
from aircraft_closure.export.openvsp.structure import build_structure, export_structure_meshes

directory_output = Path("output/structure")

if __name__ == "__main__":
    snapshot = halo_plan027_snapshot()
    directory_output.mkdir(parents=True, exist_ok=True)
    paths_outer_mold_line = export_outer_mold_line(snapshot, directory_output / "outer_mold_line",
                                                   angles_nacelle_deg=(0.0,))
    geoms, structures = build_structure(snapshot)
    vsp.WriteVSPFile(str(directory_output / "halo_structure.vsp3"))
    paths = {}
    for name, struct_id in structures.items():
        time_start_s = time.time()
        paths.update(export_structure_meshes({name: struct_id}, directory_output))
        print(f"{name:<10} meshed in {time.time() - time_start_s:5.1f} s")
    render_structure([paths[(name, "stl")] for name in structures],
                     paths_outer_mold_line[("stl_airframe", 0.0)], directory_output / "halo_structure.png")
    for name in structures:
        print(Path(paths[(name, "mass")]).read_text().split("FeaStruct_Name")[-1].strip())
