"""CalculiX check of the sized tiltrotor wing box on the OpenVSP mesh (plan 031, deferred FE step).

The OpenVSP wing-box mesh (S8 shells: skins, spars, ribs; both wing halves)
supplies nodes and elements only. This module assigns properties so the
model represents the structure the AFDD wing model weighed, adds the tip
masses, supports and load steps, runs CalculiX and reads the results back:

- Box walls (skins between the spars, and the spar webs): the AFDD torque-box
  wall area spread over the box perimeter as one thickness.
- Spar caps: the AFDD spar-cap area as thickened skin strips over each spar
  (four caps, upper and lower at both spars).
- Leading and trailing edges outside the box: a non-structural fairing (very
  low modulus) carrying the AFDD fairing mass.
- Ribs: a stated gauge of the box material.
- Supports: for the jump take-off every node over the fuselage width is clamped (two cantilevers, AFDD's
  root-moment case); for the frequencies the same nodes form a rigid fuselage body carrying the rest of the
  aircraft's mass and pitch and roll inertia, and the model is free (the free-flight symmetric modes that
  AFDD's single-mode relations estimate).
- Each tip: the nacelle rib plus two point masses (the AFDD tip mass, split
  fore and aft of the spindle by the pylon radius of gyration, so the pitch
  inertia matches) form a rigid body.

The modal deck is a frequency analysis (beam, chord and torsion modes are
identified from the rigid-body motion of the nacelles; rigid-body modes are
dropped); the static deck is the jump take-off at ultimate load (AFDD's
definition: rotor thrust at the tips less tip inertia, times 1.5), giving tip
deflection and spanwise cap strain in a band outboard of the clamp (the clamp
edge itself is a stress singularity).

A check only; nothing here feeds the sizing.
"""
import math
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np

g_m_s2 = 9.80665


@dataclass(frozen=True)
class WingBoxProperties:
    """Section properties mapped from the AFDD wing breakdown (lengths in m, SI throughout)."""
    thickness_box_m: float
    thickness_cap_pad_m: float          # extra skin thickness over each spar (cap area / strip width)
    width_cap_strip_m: float
    thickness_rib_m: float
    thickness_fairing_m: float
    modulus_box_Pa: float
    poisson_box: float
    density_box_kg_m3: float
    modulus_cap_Pa: float
    density_cap_kg_m3: float
    density_fairing_kg_m3: float
    modulus_fairing_Pa: float = 1.0e8   # non-structural


@dataclass(frozen=True)
class TipMass:
    mass_kg: float                      # one tip
    radius_gyration_pitch_m: float
    xyz_spindle_m: tuple                # right tip; the left is mirrored


@dataclass(frozen=True)
class FuselageMass:
    """The rest of the aircraft as a rigid centre body for free-free modes (radii of gyration assumed)."""
    mass_kg: float
    xyz_cg_m: tuple
    radius_gyration_pitch_m: float
    radius_gyration_roll_m: float


@dataclass(frozen=True)
class Mode:
    frequency_rad_s: float
    kind: str                           # "beam", "chord", "torsion" or "local" (panel mode, nacelles still)
    symmetric: bool


def properties_from_afdd(afdd, material, chord_m, thickness_to_chord, fraction_front_spar, fraction_rear_spar,
                         span_m, width_fuselage_m, thickness_rib_m=0.0015, width_cap_strip_fraction=0.04,
                         thickness_fairing_m=0.0015):
    """Map the AFDD breakdown to shell gauges on a box between the layout's spars.

    The AFDD torque-box area is a wall cross-section area (constant along the span); it is spread uniformly over
    the real box perimeter. The spar-cap area (all caps) is split into four strips of `width_cap_strip_fraction`
    of the chord. The fairing density makes the fairing shells carry the AFDD fairing mass.
    """
    from aircraft_closure.export.openvsp.structure_check import naca_four_digit_thickness

    height_m = 0.5 * (naca_four_digit_thickness(fraction_front_spar, thickness_to_chord)
                      + naca_four_digit_thickness(fraction_rear_spar, thickness_to_chord)) * chord_m
    width_m = (fraction_rear_spar - fraction_front_spar) * chord_m
    thickness_box_m = afdd["area_torque_box_m2"] / (2 * (width_m + height_m))
    width_strip_m = width_cap_strip_fraction * chord_m
    thickness_cap_pad_m = afdd["area_spar_m2"] / 4 / width_strip_m
    area_fairing_m2 = 2 * (1 - (fraction_rear_spar - fraction_front_spar)) * chord_m * (span_m - width_fuselage_m)
    density_fairing_kg_m3 = afdd["mass_fairing_kg"] / (area_fairing_m2 * thickness_fairing_m)
    modulus_shear_Pa, modulus_Pa = material["modulus_shear_torque_box_Pa"], material["modulus_torque_box_Pa"]
    return WingBoxProperties(
        thickness_box_m=thickness_box_m, thickness_cap_pad_m=thickness_cap_pad_m, width_cap_strip_m=width_strip_m,
        thickness_rib_m=thickness_rib_m, thickness_fairing_m=thickness_fairing_m, modulus_box_Pa=modulus_Pa,
        # An isotropic shell with E and G of the AFDD laminate.
        poisson_box=min(max(modulus_Pa / (2 * modulus_shear_Pa) - 1, 0.0), 0.45),
        density_box_kg_m3=material["density_torque_box_kg_m3"], modulus_cap_Pa=material["modulus_spar_Pa"],
        density_cap_kg_m3=material["density_spar_kg_m3"], density_fairing_kg_m3=density_fairing_kg_m3)


def read_mesh(path_inp):
    """Nodes {id: xyz} and element sets {name: (type, [(id, node ids)])} of an OpenVSP CalculiX deck."""
    nodes, elsets, mode, current = {}, {}, None, None
    for line in Path(path_inp).read_text().splitlines():
        if not line.strip() or line.startswith("**"):
            continue
        if line.startswith("*"):
            keyword = line.split(",")[0].strip().upper()
            mode = keyword
            if keyword == "*ELEMENT":
                kind = re.search(r"TYPE=([^,\s]+)", line, re.I).group(1)
                current = re.search(r"ELSET=([^,\s]+)", line, re.I).group(1)
                elsets[current] = (kind, [])
            continue
        values = [value.strip() for value in line.split(",") if value.strip()]
        if mode == "*NODE":
            nodes[int(values[0])] = np.array([float(v) for v in values[1:4]])
        elif mode == "*ELEMENT":
            elsets[current][1].append((int(values[0]), [int(v) for v in values[1:]]))
    return nodes, elsets


def _centroid(nodes, connectivity):
    return np.mean([nodes[n] for n in connectivity], axis=0)


def write_deck(path_inp, nodes, elsets, properties, tip, x_le_m, chord_m, fraction_front_spar,
               fraction_rear_spar, width_fuselage_m, fuselage=None, force_tip_jump_N=None, count_modes=30,
               band_strain_m=(0.3, 1.0)):
    """Write a CalculiX deck: a free-free frequency step when `fuselage` is given, else the clamped jump step.

    Returns ({side: (reference node, rotation node)}, element groups).
    """
    p = properties
    lines = ["** Halo wing box: OpenVSP mesh, AFDD-mapped properties (plan 031 FE check)", "*NODE"]
    lines += [f"{n},{xyz[0]:.6f},{xyz[1]:.6f},{xyz[2]:.6f}" for n, xyz in nodes.items()]
    groups = {"BOX": [], "CAP": [], "FAIRING": [], "WEB": [], "RIB": []}
    element_type = {}
    for name, (kind, elements) in elsets.items():
        for element_id, connectivity in elements:
            element_type[element_id] = (kind, connectivity)
            if name.startswith("ESkin"):
                fraction = (_centroid(nodes, connectivity)[0] - x_le_m) / chord_m
                half_strip = 0.5 * p.width_cap_strip_m / chord_m
                if min(abs(fraction - fraction_front_spar), abs(fraction - fraction_rear_spar)) <= half_strip:
                    groups["CAP"].append(element_id)
                elif fraction_front_spar < fraction < fraction_rear_spar:
                    groups["BOX"].append(element_id)
                else:
                    groups["FAIRING"].append(element_id)
            elif "Spar" in name:
                groups["WEB"].append(element_id)
            else:
                groups["RIB"].append(element_id)
    for group, ids in groups.items():
        by_type = {}
        for element_id in ids:
            by_type.setdefault(element_type[element_id][0], []).append(element_id)
        for kind, kind_ids in by_type.items():
            lines.append(f"*ELEMENT, TYPE={kind}, ELSET=E{group}")
            lines += [f"{e}," + ",".join(str(n) for n in element_type[e][1]) for e in kind_ids]
    # Cap strips: skin of box thickness plus the cap pad, with a modulus giving the right axial stiffness.
    thickness_cap_m = p.thickness_box_m + p.thickness_cap_pad_m
    modulus_cap_strip_Pa = (p.modulus_box_Pa * p.thickness_box_m + p.modulus_cap_Pa * p.thickness_cap_pad_m) \
        / thickness_cap_m
    density_cap_strip = (p.density_box_kg_m3 * p.thickness_box_m + p.density_cap_kg_m3 * p.thickness_cap_pad_m) \
        / thickness_cap_m
    materials = (("MBOX", p.modulus_box_Pa, p.poisson_box, p.density_box_kg_m3),
                 ("MCAP", modulus_cap_strip_Pa, p.poisson_box, density_cap_strip),
                 ("MFAIRING", p.modulus_fairing_Pa, 0.3, p.density_fairing_kg_m3))
    for name, modulus, poisson, density in materials:
        lines += [f"*MATERIAL, NAME={name}", "*ELASTIC", f"{modulus:.6e},{poisson:.4f}", "*DENSITY", f"{density:.6e}"]
    for group, material, thickness in (("BOX", "MBOX", p.thickness_box_m), ("WEB", "MBOX", p.thickness_box_m),
                                       ("RIB", "MBOX", p.thickness_rib_m), ("CAP", "MCAP", thickness_cap_m),
                                       ("FAIRING", "MFAIRING", p.thickness_fairing_m)):
        if groups[group]:
            lines += [f"*SHELL SECTION, ELSET=E{group}, MATERIAL={material}", f"{thickness:.6e}"]

    # Supports: everything over the fuselage width is either clamped or the rigid fuselage body.
    clamped = [n for n, xyz in nodes.items() if abs(xyz[1]) <= width_fuselage_m / 2]
    # Cap elements in a band outboard of the clamp, for the root strain.
    y_low_m, y_high_m = (width_fuselage_m / 2 + band_strain_m[0], width_fuselage_m / 2 + band_strain_m[1])
    band = [e for e in groups["CAP"] if y_low_m <= abs(_centroid(nodes, element_type[e][1])[1]) <= y_high_m]
    lines.append("*ELSET, ELSET=ECAPROOT")
    lines += _chunks(band)
    # Rigid nacelles: tip-rib nodes plus two point masses split by the pylon radius of gyration.
    next_node = max(nodes) + 1
    next_element = max(element_type) + 1
    rib_nodes = {side: set() for side in (1, -1)}
    for name, (kind, elements) in elsets.items():
        if name.startswith("ENacelleRib"):
            for _, connectivity in elements:
                for n in connectivity:
                    rib_nodes[1 if nodes[n][1] > 0 else -1].add(n)
    references = {}
    for side in (1, -1):
        x_m, y_m, z_m = tip.xyz_spindle_m
        reference, rotation = next_node, next_node + 1
        mass_nodes = (next_node + 2, next_node + 3)
        next_node += 4
        lines += ["*NODE", f"{reference},{x_m:.6f},{side * y_m:.6f},{z_m:.6f}",
                  f"{rotation},{x_m:.6f},{side * y_m:.6f},{z_m:.6f}",
                  f"{mass_nodes[0]},{x_m - tip.radius_gyration_pitch_m:.6f},{side * y_m:.6f},{z_m:.6f}",
                  f"{mass_nodes[1]},{x_m + tip.radius_gyration_pitch_m:.6f},{side * y_m:.6f},{z_m:.6f}"]
        name_side = "R" if side > 0 else "L"
        lines.append(f"*ELEMENT, TYPE=MASS, ELSET=EMASS{name_side}")
        lines += [f"{next_element},{mass_nodes[0]}", f"{next_element + 1},{mass_nodes[1]}"]
        next_element += 2
        lines += [f"*MASS, ELSET=EMASS{name_side}", f"{tip.mass_kg / 2:.6e}"]
        lines.append(f"*NSET, NSET=NTIP{name_side}")
        lines += _chunks(sorted(rib_nodes[side]) + list(mass_nodes))
        lines.append(f"*RIGID BODY, NSET=NTIP{name_side}, REF NODE={reference}, ROT NODE={rotation}")
        references[side] = (reference, rotation)
    lines.append("*NSET, NSET=NREF")
    lines += _chunks([n for pair in references.values() for n in pair])
    if fuselage is None:
        lines += ["*NSET, NSET=NCLAMP"] + _chunks(clamped) + ["*BOUNDARY", "NCLAMP,1,6"]
        lines += ["*STEP", "*STATIC", "*CLOAD"]
        lines += [f"{references[side][0]},3,{force_tip_jump_N:.6e}" for side in (1, -1)]
        lines += ["*NODE PRINT, NSET=NREF", "U", "*EL PRINT, ELSET=ECAPROOT", "E", "*END STEP"]
    else:
        # Rigid fuselage: the centre-section nodes plus four point masses giving the pitch and roll inertia.
        f = fuselage
        reference, rotation = next_node, next_node + 1
        x_m, y_m, z_m = f.xyz_cg_m
        offset_pitch_m, offset_roll_m = math.sqrt(2) * f.radius_gyration_pitch_m, math.sqrt(2) * f.radius_gyration_roll_m
        points = ((x_m - offset_pitch_m, y_m, z_m), (x_m + offset_pitch_m, y_m, z_m),
                  (x_m, y_m - offset_roll_m, z_m), (x_m, y_m + offset_roll_m, z_m))
        lines += ["*NODE", f"{reference},{x_m:.6f},{y_m:.6f},{z_m:.6f}", f"{rotation},{x_m:.6f},{y_m:.6f},{z_m:.6f}"]
        lines += [f"{next_node + 2 + k},{px:.6f},{py:.6f},{pz:.6f}" for k, (px, py, pz) in enumerate(points)]
        lines.append("*ELEMENT, TYPE=MASS, ELSET=EMASSF")
        lines += [f"{next_element + k},{next_node + 2 + k}" for k in range(4)]
        lines += ["*MASS, ELSET=EMASSF", f"{f.mass_kg / 4:.6e}"]
        lines += ["*NSET, NSET=NFUSELAGE"] + _chunks(clamped + [next_node + 2 + k for k in range(4)])
        lines.append(f"*RIGID BODY, NSET=NFUSELAGE, REF NODE={reference}, ROT NODE={rotation}")
        # Rigid-body modes come first (near zero); ask for enough to leave count_modes elastic ones.
        lines += ["*STEP", "*FREQUENCY", f"{count_modes + 6}", "*NODE PRINT, NSET=NREF", "U", "*END STEP"]
    Path(path_inp).write_text("\n".join(lines) + "\n")
    return references, groups


def _chunks(ids, size=12):
    ids = list(ids)
    return [",".join(str(i) for i in ids[k:k + size]) + "," for k in range(0, len(ids), size)]


def run_calculix(path_inp, path_ccx, timeout_s=3600):
    path_inp = Path(path_inp)
    completed = subprocess.run([str(path_ccx), path_inp.stem], cwd=path_inp.parent, capture_output=True, text=True,
                               timeout=timeout_s)
    if completed.returncode != 0 or "Job finished" not in completed.stdout:
        raise RuntimeError(f"CalculiX failed ({completed.returncode}):\n{completed.stdout[-2000:]}")
    return path_inp.with_suffix(".dat")


def read_modes(path_dat, references, radius_gyration_m, frequency_min_rad_s=1.0, fraction_participation_min=0.1):
    """Elastic modes (rigid-body modes below `frequency_min_rad_s` dropped), classified by nacelle motion.

    CalculiX normalises modes to unit modal mass, so a local panel mode (soft fairing, thin skin) barely moves the
    nacelles: modes whose nacelle motion is below `fraction_participation_min` of the largest found are "local".
    """
    text = Path(path_dat).read_text()
    frequencies = [float(m.group(2)) for m in re.finditer(
        r"^\s+(\d+)\s+[-\d.E+]+\s+([-\d.E+]+)\s+[-\d.E+]+\s+[-\d.E+]+\s*$", text, re.M)]
    blocks = re.split(r"displacements \(vx,vy,vz\) for set NREF", text)[1:]
    reference_right, rotation_right = references[1]
    reference_left, rotation_left = references[-1]
    modes = []
    for frequency_rad_s, block in zip(frequencies, blocks):
        if frequency_rad_s < frequency_min_rad_s:
            continue
        values = {int(m.group(1)): np.array([float(m.group(k)) for k in (2, 3, 4)])
                  for m in re.finditer(r"^\s*(\d+)\s+([-\d.E+]+)\s+([-\d.E+]+)\s+([-\d.E+]+)", block, re.M)}
        u_right, theta_right = values[reference_right], values[rotation_right]
        u_left, theta_left = values[reference_left], values[rotation_left]
        measures = {"beam": abs(u_right[2]), "chord": abs(u_right[0]),
                    "torsion": abs(theta_right[1]) * radius_gyration_m}
        kind = max(measures, key=measures.get)
        component = {"beam": (u_right[2], u_left[2]), "chord": (u_right[0], u_left[0]),
                     "torsion": (theta_right[1], theta_left[1])}[kind]
        modes.append((Mode(frequency_rad_s=frequency_rad_s, kind=kind,
                           symmetric=bool(component[0] * component[1] > 0)), measures[kind]))
    largest = max((participation for _, participation in modes), default=0.0)
    return tuple(mode if participation >= fraction_participation_min * largest else
                 Mode(mode.frequency_rad_s, "local", mode.symmetric) for mode, participation in modes)


def read_static(path_dat, references):
    """Right-tip vertical deflection and the largest element-mean spanwise strain in the root cap band."""
    text = Path(path_dat).read_text()
    reference_right = references[1][0]
    match = re.search(rf"^\s*{reference_right}\s+[-\d.E+]+\s+[-\d.E+]+\s+([-\d.E+]+)", text, re.M)
    deflection_m = float(match.group(1)) if match else float("nan")
    strains_by_element = {}
    section = text.split("strains (elem, integ.pnt.,exx,eyy,ezz,exy,exz,eyz)")
    if len(section) > 1:
        for m in re.finditer(r"^\s*(\d+)\s+\d+\s+[-\d.E+]+\s+([-\d.E+]+)", section[1], re.M):
            strains_by_element.setdefault(int(m.group(1)), []).append(float(m.group(2)))
    strain_max = max((abs(np.mean(values)) for values in strains_by_element.values()), default=float("nan"))
    return deflection_m, strain_max
