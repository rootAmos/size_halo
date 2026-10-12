"""Plan 054 figure: which aerodynamic model handles each part of the Halo in the sizing (AeroBuildup and additions).

    python -m examples.halo_aero_model_map_figure output/halo_v3_9.pkl

Reads a `HaloSizingResult` pickle and writes docs/figures/halo_aero_model_map.png: the drawn Halo (V-tail, wing
fairing, tip nacelles; `halo_reference_snapshot`) at the sized v3.9 dimensions, each part coloured by the model that
handles it, with the whole-aircraft items listed. The sizing evaluates the same models on a conventional-tail
stand-in with the V-tail's projected areas.
"""
import pickle
import sys
from pathlib import Path

import aerosandbox.numpy as np
import matplotlib
from matplotlib.colors import to_rgb

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from aircraft_closure.export.openvsp.snapshot import halo_reference_snapshot
from examples.halo_sizing import HaloAssumptions

path_figure = Path("docs/figures/halo_aero_model_map.png")
surface, ink, muted = "#FFFFFF", "#1D2433", "#5A6474"
colour_surface, colour_body, colour_rotor = "#2A6FC2", "#C2611F", "#8E9AAD"


light = np.array([-0.4, -0.5, 0.75]) / np.linalg.norm([-0.4, -0.5, 0.75])


def add_mesh(ax, points, faces, colour):
    """Faces shaded by a fixed light (Lambert, both sides lit), so curved bodies read as 3-D."""
    polygons = [points[face] for face in faces]
    base = np.array(to_rgb(colour))
    shades = []
    for polygon in polygons:
        normal = np.cross(polygon[1] - polygon[0], polygon[2] - polygon[0])
        length = np.linalg.norm(normal)
        intensity = 0.55 + 0.45 * abs(normal @ light) / length if length > 0 else 0.8
        shades.append(np.clip(base * intensity + (1 - intensity) * 0.15, 0, 1))
    ax.add_collection3d(Poly3DCollection(polygons, facecolors=shades, edgecolors=shades, linewidths=0.1))


def rotor_disc(ax, centre, radius, count=60):
    """Airplane-mode disc in the y-z plane at the nacelle nose."""
    angle = np.linspace(0, 2 * np.pi, count)
    ring = np.stack([np.full(count, centre[0]), centre[1] + radius * np.cos(angle),
                     centre[2] + radius * np.sin(angle)], axis=1)
    ax.add_collection3d(Poly3DCollection([ring], facecolor=colour_rotor, edgecolor=colour_rotor, alpha=0.08,
                                         linewidth=1.2, linestyle=(0, (4, 3))))


def plot(result, path):
    d = result.design
    snapshot = halo_reference_snapshot(dict(
        span_wing_m=result.span_m, area_wing_m2=d.area_wing_m2, x_le_wing_m=d.x_le_wing_m,
        radius_rotor_m=result.radius_rotor_m, solidity=d.solidity if d.solidity is not None else HaloAssumptions().solidity,
        area_horizontal_tail_m2=d.area_horizontal_tail_m2, area_vertical_tail_m2=d.area_vertical_tail_m2))
    airplane = snapshot.to_asb()
    figure = plt.figure(figsize=(16, 9.0), dpi=150)
    figure.patch.set_facecolor(surface)
    ax = figure.add_axes((0.02, 0.06, 0.56, 0.86), projection="3d")
    ax.set_facecolor(surface)
    all_points = []
    for fuselage in airplane.fuselages:            # discs first, so the airframe draws over them
        if fuselage.name.startswith("nacelle"):
            rotor_disc(ax, fuselage.xsecs[0].xyz_c, result.radius_rotor_m)
    for wing in airplane.wings:
        points, faces = wing.mesh_body(method="quad", chordwise_resolution=12)
        add_mesh(ax, points, faces, colour_surface)
        all_points.append(points)
    for fuselage in airplane.fuselages:
        points, faces = fuselage.mesh_body(method="quad", tangential_resolution=24)
        add_mesh(ax, points, faces, colour_body)
        all_points.append(points)
    points = np.concatenate(all_points)
    centre, half = (points.max(axis=0) + points.min(axis=0)) / 2, 0.47 * (points.max(axis=0) - points.min(axis=0)).max()
    for setter, index in ((ax.set_xlim, 0), (ax.set_ylim, 1), (ax.set_zlim, 2)):
        setter(centre[index] - half, centre[index] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=22, azim=-135)
    ax.set_axis_off()

    text = figure.add_axes((0.58, 0.07, 0.41, 0.85))
    text.set_axis_off()
    blocks = (
        (colour_surface, "Wing and V-tail: lifting surfaces",
         ("NeuralFoil 2-D section polars on each spanwise strip,", "at the local Reynolds and Mach numbers, fully turbulent",
          "3-D lift slope from aspect ratio, Mach and sweep",
          "+ interference factor Q (Scholz Table 13.4)",
          "+ blown-wing increments in the rotor slipstream")),
        (colour_body, "Fuselage, wing fairing, nacelles: slender bodies",
         ("skin friction × body form factor × wetted area", "+ potential-flow and viscous crossflow lift and moment",
          "(Jorgensen, NASA TR R-474)", "+ interference factor Q (1.5 for tip nacelles)")),
        (colour_rotor, "Proprotors (airplane mode)",
         ("not a drag body here: rotor power from the BEM / momentum", "model; slipstream velocity feeds the blown wing,",
          "disk geometry feeds the hover download")),
        (ink, "Whole aircraft",
         ("induced drag from the combined effective span,", "with Nita-Scholz fuselage and Mach factors",
          "× 1.19 excrescence on component drag (XV-15 to NDARC)",
          "+ 3.00 ft² fittings and fixtures (NDARC)",
          "+ trim drag from the tail load that zeroes C_M about the CG")),
    )
    y = 0.97
    for colour, title, lines in blocks:
        text.add_patch(plt.Rectangle((0.0, y - 0.03), 0.035, 0.035, color=colour, transform=text.transAxes,
                                     alpha=0.35 if colour == colour_rotor else 1.0))
        text.text(0.055, y - 0.012, title, fontsize=14, color=ink, fontweight="bold", va="center",
                  transform=text.transAxes)
        y -= 0.06
        for line in lines:
            text.text(0.055, y, line, fontsize=12, color=muted, va="center", transform=text.transAxes)
            y -= 0.04
        y -= 0.03
    figure.suptitle("Halo aerodynamics in the sizing: which model handles which part (AeroSandbox AeroBuildup)",
                    x=0.02, ha="left", fontsize=17, color=ink, fontweight="bold")
    figure.text(0.02, 0.02, f"Drawn Halo at the sized v3.9 dimensions: {result.mass_takeoff_kg / 0.45359237:,.0f} lb, "
                f"wing {result.design.area_wing_m2:.1f} m², span {result.span_m:.1f} m, rotor radius "
                f"{result.radius_rotor_m:.2f} m. The sizing uses a conventional tail of the same projected areas.",
                fontsize=10, color=muted)
    figure.savefig(path, facecolor=surface)
    plt.close(figure)


if __name__ == "__main__":
    with open(sys.argv[1], "rb") as file:
        result = pickle.load(file)
    plot(result, path_figure)
    print(f"wrote {path_figure}")
