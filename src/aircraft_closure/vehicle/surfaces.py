"""Unswept trapezoidal lifting surfaces with Raymer general-aviation masses.

Geometry is parameterized by planform area, aspect ratio and taper ratio and
handed to `asb.Wing`, which owns span, MAC and area calculations. Masses call
AeroSandbox's Raymer correlations (Aircraft Design: A Conceptual Approach,
5th ed., sec. 15.3.3; general aviation scope) on that geometry; `mass_factor`
is a dimensionless calibration multiplier. CG sits at 40 % of the MAC.

Tier 20: `Wing.mass_model` selects an interchangeable wing mass submodel.
None keeps Raymer (the simplest model); `TiltrotorWingMassModel` is the AFDD
tiltrotor wing (NDARC sec. 19-1.1), sized by torsion and bending frequencies
in per rev and by a jump take-off.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.weights import afdd


def _trapezoid_chords_m(area_m2, aspect_ratio, taper_ratio):
    """Span (one panel pair or one fin) and root/tip chords of a trapezoid."""
    span_m = np.sqrt(area_m2 * aspect_ratio)
    chord_root_m = 2 * area_m2 / (span_m * (1 + taper_ratio))
    return span_m, chord_root_m, chord_root_m * taper_ratio


def _cg_x_m(asb_wing, x_le_root_m):
    # Unswept: the MAC leading edge lies at the root leading-edge x.
    return x_le_root_m + 0.4 * asb_wing.mean_aerodynamic_chord()


class _SymmetricSurface:
    """Shared planform geometry of the symmetric surfaces (wing, horizontal tail)."""
    asb_name = ""

    def span_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[0]

    def chord_root_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[1]

    def to_asb(self):
        span_m, chord_root_m, chord_tip_m = _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)
        return asb.Wing(name=self.asb_name, symmetric=True, xsecs=[
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_m], chord=chord_root_m, airfoil=self.airfoil),
            asb.WingXSec(xyz_le=[self.x_le_root_m, span_m / 2, self.z_m], chord=chord_tip_m, airfoil=self.airfoil),
        ])


@dataclass(frozen=True)
class Wing(_SymmetricSurface):
    asb_name = "wing"
    area_m2: Any = 12.0
    aspect_ratio: Any = 9.0
    taper_ratio: Any = 1.0
    x_le_root_m: Any = 2.5
    z_m: Any = 0.6
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca4418"))
    mass_factor: Any = 1.0
    mass_model: Any = None        # None: Raymer GA; or a submodel with mass_kg(wing, condition)

    def chord_mean_m(self):
        return self.area_m2 / self.span_m()

    def get_mass_properties(self, condition):
        wing = self.to_asb()
        if self.mass_model is None:
            mass_unfactored_kg = raymer.mass_wing(
                wing, design_mass_TOGW=condition.mass_design_kg,
                ultimate_load_factor=condition.load_factor_ultimate,
                mass_fuel_in_wing=0, cruise_op_point=condition.operating_point())
        else:
            mass_unfactored_kg = self.mass_model.mass_kg(self, condition)
        return asb.MassProperties(mass=self.mass_factor * mass_unfactored_kg, x_cg=_cg_x_m(wing, self.x_le_root_m),
                                  z_cg=self.z_m)


@dataclass(frozen=True)
class HorizontalTail(_SymmetricSurface):
    asb_name = "horizontal_tail"
    area_m2: Any = 2.4
    aspect_ratio: Any = 4.5
    taper_ratio: Any = 0.7
    x_le_root_m: Any = 6.2
    z_m: Any = 0.3
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca0012"))
    mass_factor: Any = 1.0
    elevator_chord_fraction: Any = 0.3

    def get_mass_properties(self, condition):
        tail = self.to_asb()
        mass_kg = self.mass_factor * raymer.mass_hstab(
            tail, design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, cruise_op_point=condition.operating_point())
        return asb.MassProperties(mass=mass_kg, x_cg=_cg_x_m(tail, self.x_le_root_m), z_cg=self.z_m)


@dataclass(frozen=True)
class VerticalTail:
    """Single fin; aspect ratio is height^2 / area."""
    area_m2: Any = 1.6
    aspect_ratio: Any = 1.6
    taper_ratio: Any = 0.6
    x_le_root_m: Any = 6.0
    z_root_m: Any = 0.5
    airfoil: Any = field(default_factory=lambda: asb.Airfoil("naca0012"))
    mass_factor: Any = 1.0
    rudder_chord_fraction: Any = 0.3

    def height_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[0]

    def chord_root_m(self):
        return _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)[1]

    def to_asb(self):
        height_m, chord_root_m, chord_tip_m = _trapezoid_chords_m(self.area_m2, self.aspect_ratio, self.taper_ratio)
        return asb.Wing(name="vertical_tail", symmetric=False, xsecs=[
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_root_m], chord=chord_root_m, airfoil=self.airfoil),
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0, self.z_root_m + height_m], chord=chord_tip_m,
                         airfoil=self.airfoil),
        ])

    def get_mass_properties(self, condition):
        fin = self.to_asb()
        mass_kg = self.mass_factor * raymer.mass_vstab(
            fin, design_mass_TOGW=condition.mass_design_kg,
            ultimate_load_factor=condition.load_factor_ultimate, cruise_op_point=condition.operating_point())
        return asb.MassProperties(mass=mass_kg, x_cg=_cg_x_m(fin, self.x_le_root_m),
                                  z_cg=self.z_root_m + 0.4 * self.height_m())


@dataclass(frozen=True)
class WingMaterial:
    """Torque-box and spar-cap material for the tiltrotor wing; `strain_ultimate` is the strain allowable
    (minimum of spar and torque box) used against the jump take-off moment."""
    density_torque_box_kg_m3: Any
    density_spar_kg_m3: Any
    modulus_shear_torque_box_Pa: Any
    modulus_torque_box_Pa: Any
    modulus_spar_Pa: Any
    strain_ultimate: Any


def aluminium_wing_material():
    """XV-15 aluminium wing: Acree, Peyran and Johnson, NASA/TP-2004-212262, table 1, and AHS 55th Forum
    (1999), table 5.

    E = 10e6 psi (box and spars), G = 3.8e6 psi, density 0.1 lb/in3, strain 0.0068 (the table's limit strain,
    used here as the NDARC allowable).
    """
    psi = u.lbf / u.inch**2
    density_kg_m3 = 0.1 * u.lbm / u.inch**3
    return WingMaterial(density_torque_box_kg_m3=density_kg_m3, density_spar_kg_m3=density_kg_m3,
                        modulus_shear_torque_box_Pa=3.8e6 * psi, modulus_torque_box_Pa=10e6 * psi,
                        modulus_spar_Pa=10e6 * psi, strain_ultimate=0.0068)


def graphite_epoxy_wing_material():
    """Graphite-epoxy conceptual tiltrotor wing, same source and table: E box 9e6 psi, E spar 18e6 psi,
    G 3.75e6 psi, density 0.06 lb/in3, strain 0.0047."""
    psi = u.lbf / u.inch**2
    density_kg_m3 = 0.06 * u.lbm / u.inch**3
    return WingMaterial(density_torque_box_kg_m3=density_kg_m3, density_spar_kg_m3=density_kg_m3,
                        modulus_shear_torque_box_Pa=3.75e6 * psi, modulus_torque_box_Pa=9e6 * psi,
                        modulus_spar_Pa=18e6 * psi, strain_ultimate=0.0047)


@dataclass(frozen=True)
class TiltrotorWingMassModel:
    """AFDD tiltrotor wing mass submodel for `Wing.mass_model` (NDARC sec. 19-1.1; equations in `weights.afdd`).

    Span is the torque-box length (rotor centreline to centreline); chord is the mean chord. Frequencies are in
    per rev of `speed_rotor_design_rad_s`: the wing is built so that its torsion, beam and chord modes sit at those
    fractions of that rotor speed. Whirl-flutter placement at other rotor speeds is checked by the caller with
    `frequency_per_rev` (a margin, not a hidden loop). `mass_tip_kg` is the mass on one tip.

    Defaults are the XV-15 (plan 024): published modes at the 458 rpm airplane-mode rotor speed (Acree et al.
    stick model: symmetric torsion 8.3 Hz, beam 3.3 Hz, chord 6.3 Hz); torque-box chord ratio 0.45 (assumed); the
    torque-box efficiency, spar taper correction, fairing and control-surface unit masses and fittings fraction
    reproduce the published XV-15 wing breakdown (torque box 567, spars 52, control surfaces 97, fairings 108,
    fittings and other 122 lb) from its published stiffness (`examples/tiltrotor_wing_calibration.py`).
    """
    mass_tip_kg: Any
    radius_gyration_pylon_m: Any
    speed_rotor_design_rad_s: Any
    width_fuselage_m: Any
    width_attachment_m: Any = None                       # wing-to-body attachment width; None: fuselage width
    frequency_torsion_per_rev: Any = 1.087
    frequency_beam_per_rev: Any = 0.432
    frequency_chord_per_rev: Any = 0.825
    thickness_to_chord: Any = 0.23
    fraction_chord_torque_box: Any = 0.45
    ratio_depth_spar_cap: Any = 1.0                      # plan 038; 1.0 is NDARC
    thickness_min_torque_box_m: Any = 0.0                # plan 038; 0 is NDARC
    material: Any = field(default_factory=aluminium_wing_material)
    fraction_area_control_surfaces: Any = 0.185          # XV-15 flaps 11.0 + flaperons 20.2 ft2 of 169 ft2
    unit_mass_fairing_kg_m2: Any = 10.92                 # XV-15: 108 lb on 48.3 ft2
    unit_mass_control_surfaces_kg_m2: Any = 15.18        # XV-15: 97 lb on 31.2 ft2
    fraction_fittings: Any = 0.129                       # XV-15: 122 lb fittings and other
    fraction_fold: Any = 0.0                             # no fold/tilt structure (V-22 style stowage would add it)
    efficiency_torque_box: Any = 0.583
    correction_spar_stiffness: Any = 0.526
    count_rotors: int = 2
    load_factor_jump: Any = 2.0                          # NDARC / Johnson practice (2-g jump take-off)
    smoothing: Any = 0.0                                 # > 0 rounds the max(0, .) steps for gradient-based sizing

    def masses(self, wing, condition):
        m = self.material
        return afdd.wing_tiltrotor_afdd_masses(
            span_m=wing.span_m(), chord_m=wing.chord_mean_m(), thickness_to_chord=self.thickness_to_chord,
            fraction_chord_torque_box=self.fraction_chord_torque_box, mass_design_kg=condition.mass_design_kg,
            mass_tip_kg=self.mass_tip_kg, radius_gyration_pylon_m=self.radius_gyration_pylon_m,
            speed_rotor_design_rad_s=self.speed_rotor_design_rad_s,
            frequency_torsion_per_rev=self.frequency_torsion_per_rev,
            frequency_beam_per_rev=self.frequency_beam_per_rev, frequency_chord_per_rev=self.frequency_chord_per_rev,
            density_torque_box_kg_m3=m.density_torque_box_kg_m3, density_spar_kg_m3=m.density_spar_kg_m3,
            modulus_shear_torque_box_Pa=m.modulus_shear_torque_box_Pa, modulus_torque_box_Pa=m.modulus_torque_box_Pa,
            modulus_spar_Pa=m.modulus_spar_Pa, strain_ultimate=m.strain_ultimate,
            area_control_surfaces_m2=self.fraction_area_control_surfaces * wing.area_m2,
            unit_mass_fairing_kg_m2=self.unit_mass_fairing_kg_m2,
            unit_mass_control_surfaces_kg_m2=self.unit_mass_control_surfaces_kg_m2,
            width_fuselage_m=self.width_fuselage_m,
            width_attachment_m=self.width_attachment_m if self.width_attachment_m is not None else self.width_fuselage_m,
            count_rotors=self.count_rotors, load_factor_jump=self.load_factor_jump,
            fraction_fittings=self.fraction_fittings, fraction_fold=self.fraction_fold,
            efficiency_torque_box=self.efficiency_torque_box,
            correction_spar_stiffness=self.correction_spar_stiffness, smoothing=self.smoothing,
            ratio_depth_spar_cap=self.ratio_depth_spar_cap,
            thickness_min_torque_box_m=self.thickness_min_torque_box_m)

    def mass_kg(self, wing, condition):
        return self.masses(wing, condition).total()

    @staticmethod
    def frequency_per_rev(masses, speed_rotor_rad_s):
        """Realized (torsion, beam, chord) wing frequencies as fractions of a rotor speed (whirl-flutter margins)."""
        return (masses.frequency_torsion_rad_s / speed_rotor_rad_s, masses.frequency_beam_rad_s / speed_rotor_rad_s,
                masses.frequency_chord_rad_s / speed_rotor_rad_s)
