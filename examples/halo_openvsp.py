"""Halo outer mold line in OpenVSP: .vsp3 with Modes, STEP and STL per nacelle angle, and a render sheet.

Run from the repo root: `python -m examples.halo_openvsp`. Outputs go to
`output/openvsp/` (not committed). Needs the OpenVSP Python API and the
`geometry` dependency group (PyVista) for the renders; see README.
"""
from pathlib import Path

from aircraft_closure.export.openvsp.model import export_outer_mold_line
from aircraft_closure.export.openvsp.render import render_sheet, views_orthographic, views_perspective
from aircraft_closure.export.openvsp.snapshot import halo_plan027_snapshot

directory_output = Path("output/openvsp")


def render_views(paths, path_png):
    """Top row: the nacelle angles in perspective. Bottom row: side, top and front of the cruise model."""
    angles_nacelle_deg = sorted({key[1] for key in paths if isinstance(key, tuple)}, reverse=True)
    views = [(f"nacelle {angle_deg:.0f} deg", paths[("stl_airframe", angle_deg)], paths[("stl_rotors", angle_deg)],
              views_perspective) for angle_deg in angles_nacelle_deg]
    angle_cruise_deg = min(angles_nacelle_deg)
    views += [(f"{view[0]} (nacelle {angle_cruise_deg:.0f} deg)", paths[("stl_airframe", angle_cruise_deg)],
               paths[("stl_rotors", angle_cruise_deg)], view) for view in views_orthographic]
    render_sheet(views, path_png)


if __name__ == "__main__":
    paths = export_outer_mold_line(halo_plan027_snapshot(), directory_output)
    render_views(paths, directory_output / "halo_views.png")
    for key, path in paths.items():
        print(f"{str(key):<30}{path}")
