"""Layout-based primary-structure masses: a back-check of the sizing's weight correlations (plan 031).

The sizing weighs the wing with the AFDD tiltrotor model (torque box from the
torsion frequency, spar caps from the bending frequencies and the jump
take-off, a uniform idealized section) and the tails and fuselage with Raymer's
general-aviation correlations. This module weighs the same primary structure
from the drawn layout instead: the real tapered box between the layout's spars
with the airfoil's depth at each spar, spanwise bending-moment distributions,
and the OpenVSP structure's part areas. Gauges come from simple, stated
criteria (strength, stiffness, minimum gauge); nothing here feeds the sizing.

First-order methods throughout; every assumption is a field or a docstring.
"""
from dataclasses import dataclass

import numpy as np

g_m_s2 = 9.80665


# ---- Geometry helpers ------------------------------------------------------------------

def naca_four_digit_thickness(fraction_chord, thickness_to_chord):
    """Full thickness / chord of a NACA 4-digit section at `fraction_chord` (open trailing edge form)."""
    x = np.asarray(fraction_chord, dtype=float)
    # The classic expression (5 t [...]) is the half-thickness.
    return 10 * thickness_to_chord * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3
                                     - 0.1015 * x**4)


def stl_solid_areas_m2(path):
    """{solid name: area} of an ASCII STL with named solids (OpenVSP FEA: one solid per part and side)."""
    areas, name, vertices = {}, None, []
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
                triangles = np.array(vertices).reshape(-1, 3, 3)
                cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
                areas[name] = areas.get(name, 0.0) + 0.5 * np.linalg.norm(cross, axis=1).sum()
    return areas


def area_by_part_m2(path, prefix):
    """Total area of all solids whose name starts with `prefix` (both mirrored sides)."""
    return sum(area for name, area in stl_solid_areas_m2(path).items() if name.startswith(prefix))


# ---- Lifting-surface box ------------------------------------------------------------------

@dataclass(frozen=True)
class BoxMaterial:
    """Torque-box (skins, webs) and spar-cap material; strain-limited like the AFDD model."""
    density_box_kg_m3: float
    modulus_box_Pa: float
    modulus_shear_box_Pa: float
    density_cap_kg_m3: float
    modulus_cap_Pa: float
    strain_ultimate: float
    thickness_min_m: float = 0.001          # minimum handling / damage gauge (assumed, composite)
    density_rib_kg_m3: float = 1600.0
    thickness_rib_m: float = 0.0015
    fraction_rib_solid: float = 0.6         # rib web left after lightening holes (assumed)


@dataclass(frozen=True)
class SurfaceLoads:
    """Ultimate loads on one panel (root at `y_root_m`, tip at `y_tip_m`) as spanwise moment and shear arrays."""
    y_m: np.ndarray
    moment_Nm: np.ndarray
    shear_N: np.ndarray
    case: tuple                              # governing case name per station


@dataclass(frozen=True)
class BoxSizing:
    y_m: np.ndarray
    chord_m: np.ndarray
    width_box_m: np.ndarray
    height_box_m: np.ndarray
    thickness_skin_m: np.ndarray
    thickness_web_m: np.ndarray
    area_caps_m2: np.ndarray                 # all four caps together
    driver_caps: tuple                       # per station: "strength", "beam stiffness", "chord stiffness" or "none"
    mass_skins_kg: float
    mass_webs_kg: float
    mass_caps_kg: float
    mass_ribs_kg: float

    def mass_box_kg(self):
        return self.mass_skins_kg + self.mass_webs_kg

    def mass_primary_kg(self):
        return self.mass_skins_kg + self.mass_webs_kg + self.mass_caps_kg + self.mass_ribs_kg


def elliptic_panel_loads(y_m, semispan_m, lift_panel_N, point_loads=()):
    """Moment and shear along a panel from an elliptic lift of total `lift_panel_N` (root at y = 0 of the
    ellipse) plus point loads (y, force, positive up) outboard of each station. Returns (moment, shear)."""
    eta = np.linspace(0.0, semispan_m, 2001)
    distribution = np.sqrt(np.clip(1 - (eta / semispan_m) ** 2, 0.0, None))
    loading_N_m = lift_panel_N * distribution / np.trapezoid(distribution, eta)
    moment_Nm, shear_N = np.zeros_like(y_m), np.zeros_like(y_m)
    for index, y in enumerate(y_m):
        outboard = eta >= y
        shear_N[index] = np.trapezoid(loading_N_m[outboard], eta[outboard])
        moment_Nm[index] = np.trapezoid(loading_N_m[outboard] * (eta[outboard] - y), eta[outboard])
        for y_load_m, force_N in point_loads:
            if y_load_m >= y:
                shear_N[index] += force_N
                moment_Nm[index] += force_N * (y_load_m - y)
    return moment_Nm, shear_N


def size_box(y_m, chord_m, thickness_to_chord, fraction_front_spar, fraction_rear_spar, loads, material,
             stiffness_torsion_Nm2=0.0, stiffness_beam_Nm2=0.0, stiffness_chord_Nm2=0.0, area_ribs_m2=0.0,
             count_panels=2):
    """Gauges of a two-spar box along one panel, then masses of `count_panels` panels.

    - Skins and webs: Bredt-Batho torsion stiffness for the (uniform) required GJ, webs also for shear at
      the shear strain limit, both at least the minimum gauge.
    - Spar caps: the larger of the ultimate bending moment at the strain limit and the beam and chord bending
      stiffnesses still missing after the skins; caps sit at the box corners.
    """
    m = material
    height_front_m = naca_four_digit_thickness(fraction_front_spar, thickness_to_chord) * chord_m
    height_rear_m = naca_four_digit_thickness(fraction_rear_spar, thickness_to_chord) * chord_m
    height_m = 0.5 * (height_front_m + height_rear_m)
    width_m = (fraction_rear_spar - fraction_front_spar) * chord_m
    area_enclosed_m2 = width_m * height_m
    perimeter_m = 2 * (width_m + height_m)
    thickness_torsion_m = stiffness_torsion_Nm2 * perimeter_m / (4 * area_enclosed_m2**2 * m.modulus_shear_box_Pa)
    thickness_skin_m = np.maximum(thickness_torsion_m, m.thickness_min_m)
    stress_shear_allow_Pa = m.modulus_shear_box_Pa * m.strain_ultimate
    thickness_web_m = np.maximum(thickness_skin_m, np.abs(loads.shear_N) / (2 * height_m * stress_shear_allow_Pa))

    # Caps: strength (moment over cap separation, skins also carry bending) and stiffness deficits.
    stress_cap_allow_Pa = m.modulus_cap_Pa * m.strain_ultimate
    stress_skin_allow_Pa = m.modulus_box_Pa * m.strain_ultimate
    moment_skins_Nm = stress_skin_allow_Pa * 2 * width_m * thickness_skin_m * height_m / 2
    area_caps_strength_m2 = 2 * np.maximum(np.abs(loads.moment_Nm) - moment_skins_Nm, 0.0) / (
        stress_cap_allow_Pa * height_m)
    stiffness_beam_skins_Nm2 = m.modulus_box_Pa * 2 * width_m * thickness_skin_m * (height_m / 2) ** 2
    area_caps_beam_m2 = np.maximum(stiffness_beam_Nm2 - stiffness_beam_skins_Nm2, 0.0) / (
        m.modulus_cap_Pa * (height_m / 2) ** 2)
    stiffness_chord_skins_Nm2 = m.modulus_box_Pa * 2 * thickness_skin_m * width_m**3 / 12 \
        + m.modulus_box_Pa * 2 * height_m * thickness_web_m * (width_m / 2) ** 2
    area_caps_chord_m2 = np.maximum(stiffness_chord_Nm2 - stiffness_chord_skins_Nm2, 0.0) / (
        m.modulus_cap_Pa * (width_m / 2) ** 2)
    candidates = np.vstack([area_caps_strength_m2, area_caps_beam_m2, area_caps_chord_m2])
    area_caps_m2 = candidates.max(axis=0)
    names = ("strength", "beam stiffness", "chord stiffness")
    driver = tuple("none" if area <= 0 else names[int(np.argmax(column))]
                   for area, column in zip(area_caps_m2, candidates.T))

    mass_skins_kg = count_panels * np.trapezoid(2 * width_m * thickness_skin_m, y_m) * m.density_box_kg_m3
    mass_webs_kg = count_panels * np.trapezoid(
        (height_front_m + height_rear_m) * thickness_web_m, y_m) * m.density_box_kg_m3
    mass_caps_kg = count_panels * np.trapezoid(area_caps_m2, y_m) * m.density_cap_kg_m3
    mass_ribs_kg = area_ribs_m2 * m.thickness_rib_m * m.fraction_rib_solid * m.density_rib_kg_m3
    return BoxSizing(y_m=y_m, chord_m=chord_m, width_box_m=width_m, height_box_m=height_m,
                     thickness_skin_m=thickness_skin_m, thickness_web_m=thickness_web_m, area_caps_m2=area_caps_m2,
                     driver_caps=driver, mass_skins_kg=float(mass_skins_kg), mass_webs_kg=float(mass_webs_kg),
                     mass_caps_kg=float(mass_caps_kg), mass_ribs_kg=float(mass_ribs_kg))


# ---- Fuselage ---------------------------------------------------------------------------------

@dataclass(frozen=True)
class FuselageMaterial:
    """Aluminium semi-monocoque (2024 skins and frames); areal masses for floor and stringers are assumed."""
    density_kg_m3: float = 2780.0
    stress_allow_Pa: float = 2.0e8           # ultimate, compression with buckling knock-down (assumed)
    thickness_min_skin_m: float = 0.001      # unpressurized minimum gauge (assumed)
    factor_stringers: float = 0.3            # stringer + longeron mass / skin mass (assumed, typical)
    thickness_bulkhead_m: float = 0.0015
    fraction_bulkhead_solid: float = 0.5     # bulkhead web left after cut-outs (assumed)
    unit_mass_floor_kg_m2: float = 3.0       # sandwich floor panel (assumed)


@dataclass(frozen=True)
class FuselageSizing:
    thickness_skin_m: float
    thickness_skin_bending_m: float
    moment_bending_ultimate_Nm: float
    mass_skin_kg: float
    mass_stringers_kg: float
    mass_frames_kg: float
    mass_bulkheads_kg: float
    mass_floor_kg: float

    def mass_primary_kg(self):
        return (self.mass_skin_kg + self.mass_stringers_kg + self.mass_frames_kg + self.mass_bulkheads_kg
                + self.mass_floor_kg)


def size_fuselage(area_skin_m2, length_frames_m, area_section_frame_m2, area_bulkheads_m2, area_floor_m2,
                  width_m, height_m, moment_bending_ultimate_Nm, material=FuselageMaterial()):
    """Skin gauge from the ultimate bending moment at the critical station (a thin rectangular tube) or the
    minimum gauge; frames, bulkheads and floor from the layout areas and lengths."""
    m = material
    # Thin-walled rectangle: I = t (h^3/6 + w h^2 / 2); stress = M (h/2) / I.
    thickness_bending_m = moment_bending_ultimate_Nm * (height_m / 2) / (
        m.stress_allow_Pa * (height_m**3 / 6 + width_m * height_m**2 / 2))
    thickness_skin_m = max(m.thickness_min_skin_m, thickness_bending_m)
    mass_skin_kg = area_skin_m2 * thickness_skin_m * m.density_kg_m3
    return FuselageSizing(
        thickness_skin_m=thickness_skin_m, thickness_skin_bending_m=thickness_bending_m,
        moment_bending_ultimate_Nm=moment_bending_ultimate_Nm, mass_skin_kg=mass_skin_kg,
        mass_stringers_kg=m.factor_stringers * mass_skin_kg,
        mass_frames_kg=length_frames_m * area_section_frame_m2 * m.density_kg_m3,
        mass_bulkheads_kg=area_bulkheads_m2 * m.thickness_bulkhead_m * m.fraction_bulkhead_solid * m.density_kg_m3,
        mass_floor_kg=area_floor_m2 * m.unit_mass_floor_kg_m2)
