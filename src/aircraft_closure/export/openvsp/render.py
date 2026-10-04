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
