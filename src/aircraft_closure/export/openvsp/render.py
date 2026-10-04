"""Off-screen renders of OpenVSP STL exports with PyVista (the optional `geometry` dependency group).

Axes follow AeroSandbox (x aft, y right, z up). Views put the nose to the
viewer's left; the perspective view looks from ahead, left and above.
"""
import numpy as np
import pyvista as pv

colour_airframe = "#dfe3e8"
colour_rotors = "#30343b"
colour_background = "white"

# (title, camera direction from the aircraft centre, view-up vector, orthographic)
views_perspective = ("perspective", (-1.0, -1.15, 0.7), (0.0, 0.0, 1.0), False)
views_orthographic = (("side", (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), True),
                      ("top", (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), True),
                      ("front", (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0), True))


def _read_mesh(path):
    mesh = pv.read(str(path)).clean()
    return mesh.compute_normals(feature_angle=40.0, split_vertices=True, auto_orient_normals=False)


def _add_aircraft(plotter, path_airframe, path_rotors):
    plotter.add_mesh(_read_mesh(path_airframe), color=colour_airframe, smooth_shading=True, specular=0.35,
                     specular_power=20, ambient=0.15, diffuse=0.85)
    if path_rotors is not None:
        plotter.add_mesh(_read_mesh(path_rotors), color=colour_rotors, smooth_shading=True, specular=0.2)


def _set_camera(plotter, direction, view_up, orthographic, zoom):
    bounds = np.array(plotter.bounds).reshape(3, 2)
    centre = bounds.mean(axis=1)
    size_m = (bounds[:, 1] - bounds[:, 0]).max()
    direction = np.asarray(direction, dtype=float) / np.linalg.norm(direction)
    plotter.camera_position = [tuple(centre + 2.5 * size_m * direction), tuple(centre), view_up]
    if orthographic:
        plotter.renderer.enable_parallel_projection()
    plotter.reset_camera()
    plotter.camera.zoom(zoom)


def render_sheet(views, path_png, window_size=(2400, 1500), columns=3):
    """`views`: (title, path_airframe_stl, path_rotors_stl, camera) rows, `camera` one of the view tuples above."""
    rows = int(np.ceil(len(views) / columns))
    plotter = pv.Plotter(shape=(rows, columns), off_screen=True, window_size=window_size, border=False)
    plotter.set_background(colour_background)
    for index, (title, path_airframe, path_rotors, (_, direction, view_up, orthographic)) in enumerate(views):
        plotter.subplot(index // columns, index % columns)
        _add_aircraft(plotter, path_airframe, path_rotors)
        plotter.add_text(title, position="upper_edge", font_size=11, color="#333333")
        _set_camera(plotter, direction, view_up, orthographic, zoom=1.25 if orthographic else 1.4)
    plotter.enable_anti_aliasing("ssaa", all_renderers=True)
    plotter.screenshot(str(path_png))
    plotter.close()


def read_stl_solids(path):
    """{solid name: PolyData} of an ASCII STL with named solids (OpenVSP FEA exports one solid per part)."""
    solids, name, vertices = {}, None, []
    with open(path) as file:
        for line in file:
            words = line.split()
            if not words:
                continue
            if words[0] == "solid":
                name, vertices = (words[1] if len(words) > 1 else "solid"), []
            elif words[0] == "vertex":
                vertices.append([float(value) for value in words[1:4]])
            elif words[0] == "endsolid" and vertices:
                points = np.array(vertices)
                faces = np.hstack([np.full((len(points) // 3, 1), 3), np.arange(len(points)).reshape(-1, 3)])
                solids[name] = pv.PolyData(points, faces.ravel()).clean()
    return solids


# Part-family colours of the structure render (dataviz reference palette, light).
# Checked in order: "FrontSparBulkhead" is a bulkhead, not a spar.
colours_structure = (("Bulkhead", "#4a3aa7"), ("Spar", "#2a78d6"), ("Rib", "#eb6834"), ("Frame", "#1baf7a"),
                     ("Floor", "#eda100"))


def render_structure(paths_structure_stl, path_airframe_stl, path_png, window_size=(2400, 1500)):
    """Internal structure under a faint outer mold line: skins hidden, part families coloured.

    Left: from ahead, left and above. Right: from behind, right and above (webs are edge-on in plan view).
    """
    plotter = pv.Plotter(shape=(1, 2), off_screen=True, window_size=window_size, border=False)
    plotter.set_background(colour_background)
    solids = {}
    for path in paths_structure_stl:
        solids.update({f"{path.stem}:{name}": mesh for name, mesh in read_stl_solids(path).items()})
    legend = []
    for index, (title, direction, view_up) in enumerate((("front left", (-1.0, -1.15, 0.9), (0.0, 0.0, 1.0)),
                                                         ("rear right", (1.0, 1.0, 0.8), (0.0, 0.0, 1.0)))):
        plotter.subplot(0, index)
        plotter.add_mesh(_read_mesh(path_airframe_stl), color=colour_airframe, opacity=0.12, smooth_shading=True)
        for key, mesh in solids.items():
            part = key.split(":", 1)[1]
            if part.startswith("Skin"):
                continue
            colour = next((c for family, c in colours_structure if family in part), "#7a7a75")
            plotter.add_mesh(mesh, color=colour, smooth_shading=False, show_edges=False)
        if index == 0:
            legend = [[family, colour] for family, colour in colours_structure]
            plotter.add_legend(legend, bcolor="white", border=False, size=(0.16, 0.18), loc="upper right")
        plotter.add_text(title, position="upper_edge", font_size=11, color="#333333")
        plotter.enable_depth_peeling()   # the translucent outer mold line must not hide the structure
        _set_camera(plotter, direction, view_up, orthographic=False, zoom=1.5)
    plotter.enable_anti_aliasing("ssaa", all_renderers=True)
    plotter.screenshot(str(path_png))
    plotter.close()
