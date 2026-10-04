"""Numeric geometry of a solved tiltrotor: the only input the OpenVSP export reads.

Coordinates follow AeroSandbox: x aft, y right, z up, metres, origin at the
fuselage nose. Every field is a plain float; nothing here is symbolic, so the
export can never take part in a solve.

Nacelle angle follows the XV-15/NDARC convention: 90 deg is helicopter mode,
0 deg is airplane mode. The nacelle is described in airplane mode; the hub lies
`length_mast_m` ahead of the conversion spindle along the nacelle axis.
"""
import math
from dataclasses import dataclass

import aerosandbox as asb
import aerosandbox.tools.units as u


@dataclass(frozen=True)
class BodyStation:
    """Super-ellipse cross-section of a body, `x_m` from the body nose, `z_m` its centre.

    Exponent 2 is an ellipse; larger is boxier. A separate bottom exponent gives
    a flat bottom under a rounder top without the crease a rounded rectangle
    leaves where its corner radius changes.
    """
    x_m: float
    z_m: float
    width_m: float
    height_m: float
    exponent: float = 2.0
    exponent_bottom: float = None     # None: same as the top


@dataclass(frozen=True)
class BodySnapshot:
    """Smoothly skinned body through `stations` (nose to tail), its nose at (x_nose_m, 0, 0)."""
    stations: tuple
    x_nose_m: float = 0.0

    def to_asb(self, name, offset_xyz_m=(0.0, 0.0, 0.0)):
        """AeroSandbox fuselage through the same stations (super-ellipse `shape` is the top exponent)."""
        dx_m, dy_m, dz_m = offset_xyz_m
        return asb.Fuselage(name=name, xsecs=[
            asb.FuselageXSec(xyz_c=[self.x_nose_m + station.x_m + dx_m, dy_m, station.z_m + dz_m],
                             width=station.width_m, height=station.height_m, shape=station.exponent)
            for station in self.stations])


@dataclass(frozen=True)
class SurfaceSnapshot:
    """Trapezoid; `span_m` is tip to tip (along the panels) for symmetric surfaces, root to tip for a fin."""
    span_m: float
    chord_root_m: float
    chord_tip_m: float
    x_le_root_m: float
    z_m: float
    thickness_to_chord: float
    camber: float = 0.0               # NACA 4-series maximum camber / chord
    camber_location: float = 0.4      # NACA 4-series position of maximum camber / chord
    dihedral_deg: float = 0.0
    sweep_deg: float = 0.0
    fraction_chord_sweep: float = 0.0  # chord station the sweep is measured at (0: leading edge)

    def x_le_tip_m(self):
        """Tip leading edge of a symmetric surface (span measured along the panels)."""
        semispan_m = self.span_m / 2
        return (self.x_le_root_m + self.fraction_chord_sweep * (self.chord_root_m - self.chord_tip_m)
                + semispan_m * math.tan(math.radians(self.sweep_deg)))

    def naca_name(self):
        return f"naca{round(100 * self.camber)}{round(10 * self.camber_location) if self.camber else 0}" \
               f"{round(100 * self.thickness_to_chord):02d}"

    def to_asb(self, name):
        """AeroSandbox symmetric wing with the same root and tip sections as the OpenVSP surface."""
        semispan_m = self.span_m / 2
        dihedral_rad = math.radians(self.dihedral_deg)
        airfoil = asb.Airfoil(self.naca_name())
        return asb.Wing(name=name, symmetric=True, xsecs=[
            asb.WingXSec(xyz_le=[self.x_le_root_m, 0.0, self.z_m], chord=self.chord_root_m, airfoil=airfoil),
            asb.WingXSec(xyz_le=[self.x_le_tip_m(), semispan_m * math.cos(dihedral_rad),
                                 self.z_m + semispan_m * math.sin(dihedral_rad)],
                         chord=self.chord_tip_m, airfoil=airfoil),
        ])

    def mean_aerodynamic_chord_m(self):
        taper = self.chord_tip_m / self.chord_root_m
        return 2 / 3 * self.chord_root_m * (1 + taper + taper ** 2) / (1 + taper)


@dataclass(frozen=True)
class NacelleSnapshot:
    """Flat-sided nacelle: `width_m` lateral, `height_m` normal to the wing in airplane mode."""
    length_m: float
    width_m: float
    height_m: float
    fraction_chord_spindle: float     # spindle x on the wing tip chord
    offset_z_spindle_m: float         # spindle z above the wing reference plane
    length_mast_m: float              # spindle to rotor hub along the nacelle axis
    offset_nose_m: float = 0.0        # nacelle nose (spinner tip) ahead of the hub

    def stations(self):
        """Airplane-mode body along +x from the spinner tip: spinner, flat-sided cowling, tapering tail."""
        w_m, h_m, length_m = self.width_m, self.height_m, self.length_m
        rows = ((0.00, 0.05, 0.05, 2.0), (0.05, 0.6 * w_m, 0.6 * w_m, 2.0), (0.14, 0.95 * w_m, 0.85 * h_m, 2.4),
                (0.40, w_m, h_m, 2.8), (0.80, 0.85 * w_m, 0.9 * h_m, 2.8), (1.00, 0.45 * w_m, 0.55 * h_m, 2.4))
        return tuple(BodyStation(x_m=fraction_x * length_m, z_m=0.0, width_m=width_m, height_m=height_m,
                                 exponent=exponent) for fraction_x, width_m, height_m, exponent in rows)


@dataclass(frozen=True)
class RotorSnapshot:
    radius_m: float
    count_blades: int
    solidity: float
    angle_precone_deg: float = 2.5
    angle_twist_deg: float = -40.0        # tip minus root (r/R 0.2), linear; XV-15 class (assumed)
    angle_collective_deg: float = 10.0    # pitch at 0.75 R in the export (display only)


@dataclass(frozen=True)
class GeometrySnapshot:
    """Empennage: a conventional `horizontal_tail` and `vertical_tail`, or a `v_tail`, or both kinds absent."""
    fuselage: BodySnapshot
    wing: SurfaceSnapshot
    nacelle: NacelleSnapshot
    rotor: RotorSnapshot
    horizontal_tail: SurfaceSnapshot = None
    vertical_tail: SurfaceSnapshot = None
    v_tail: SurfaceSnapshot = None
    wing_fairing: BodySnapshot = None     # dorsal fairing blending a high wing into the fuselage

    def spindle_xyz_m(self):
        """Conversion spindle of the right nacelle."""
        w, n = self.wing, self.nacelle
        return (w.x_le_tip_m() + n.fraction_chord_spindle * w.chord_tip_m, w.span_m / 2, w.z_m + n.offset_z_spindle_m)

    def to_asb(self):
        """AeroSandbox airplane of the airplane-mode (0 deg) outer mold line, without rotors.

        Reference area and span are the wing's; reference chord is its MAC.
        """
        x_spindle_m, y_spindle_m, z_spindle_m = self.spindle_xyz_m()
        x_nose_nacelle_m = x_spindle_m - self.nacelle.length_mast_m - self.nacelle.offset_nose_m
        nacelle = BodySnapshot(stations=self.nacelle.stations(), x_nose_m=x_nose_nacelle_m)
        wings = [self.wing.to_asb("wing")]
        wings += [surface.to_asb(name) for name, surface in (("horizontal_tail", self.horizontal_tail),
                                                             ("v_tail", self.v_tail)) if surface is not None]
        if self.vertical_tail is not None:
            raise NotImplementedError("A conventional vertical tail is not exported to AeroSandbox yet.")
        fuselages = [self.fuselage.to_asb("fuselage")]
        if self.wing_fairing is not None:
            fuselages.append(self.wing_fairing.to_asb("wing_fairing"))
        fuselages += [nacelle.to_asb(f"nacelle_{side}", (0.0, sign * y_spindle_m, z_spindle_m))
                      for side, sign in (("right", 1.0), ("left", -1.0))]
        return asb.Airplane(name="halo", wings=wings, fuselages=fuselages, s_ref=self.wing.span_m * 0.5 * (
            self.wing.chord_root_m + self.wing.chord_tip_m), c_ref=self.wing.mean_aerodynamic_chord_m(),
                            b_ref=self.wing.span_m)


def v_tail_equivalent(area_horizontal_tail_m2, area_vertical_tail_m2, aspect_ratio, taper_ratio, sweep_le_deg,
                      x_le_root_m, z_m, thickness_to_chord):
    """V-tail with the projected areas of a conventional tail (the classic equal-projected-area rule).

    Total panel area is the sum of the two tail areas and the dihedral is
    atan(sqrt(S_v / S_h)), so the planform and side projections reproduce the
    horizontal and vertical tail areas. `aspect_ratio` is (tip-to-tip panel span)^2 / total area.
    """
    area_m2 = area_horizontal_tail_m2 + area_vertical_tail_m2
    span_m = (area_m2 * aspect_ratio) ** 0.5
    chord_root_m = 2 * area_m2 / (span_m * (1 + taper_ratio))
    return SurfaceSnapshot(span_m=span_m, chord_root_m=chord_root_m, chord_tip_m=taper_ratio * chord_root_m,
                           x_le_root_m=x_le_root_m, z_m=z_m, thickness_to_chord=thickness_to_chord,
                           dihedral_deg=math.degrees(math.atan((area_vertical_tail_m2 / area_horizontal_tail_m2) ** 0.5)),
                           sweep_deg=sweep_le_deg)


def _stations_from_profile(length_m, width_m, height_m, profile):
    """`profile` rows: (fraction of length, top z / height, bottom z / height, width / width_m, exponent,
    bottom exponent)."""
    return tuple(BodyStation(x_m=fraction_x * length_m, z_m=0.5 * (top + bottom) * height_m,
                             width_m=fraction_width * width_m, height_m=(top - bottom) * height_m,
                             exponent=exponent, exponent_bottom=exponent_bottom)
                 for fraction_x, top, bottom, fraction_width, exponent, exponent_bottom in profile)


def halo_fuselage(length_m, width_m, height_m):
    """Unpressurized boxy fuselage read from Halo stills (2026-10-04).

    A wide wedge (shovel) nose rises to a long constant cabin with a flat
    bottom; from about mid-length the bottom line sweeps up into a tapering
    tail boom while the top line stays nearly level. Few aft stations keep the
    boom a single smooth sweep.
    """
    profile = ((0.000, 0.04, -0.02, 0.50, 2.0, 2.0),
               (0.035, 0.22, -0.20, 0.80, 2.6, 3.0),
               (0.100, 0.44, -0.44, 0.96, 3.0, 4.5),
               (0.170, 0.50, -0.50, 1.00, 3.2, 5.0),
               (0.460, 0.50, -0.50, 1.00, 3.2, 5.0),
               (0.760, 0.48, -0.02, 0.58, 2.8, 3.0),
               (1.000, 0.44, 0.30, 0.18, 2.2, 2.2))
    return BodySnapshot(stations=_stations_from_profile(length_m, width_m, height_m, profile))


def dorsal_wing_fairing(wing, z_top_fuselage_m, width_fuselage_m, length_ahead_m=2.6, length_aft_m=3.0):
    """Broad, low hump on the fuselage top carrying a high wing up to its upper surface.

    Nearly as wide as the cabin, with both ends sunk below the fuselage top so
    the hump rises out of the cabin rather than sitting on it.
    """
    x_nose_m = wing.x_le_root_m - length_ahead_m
    chord_m = wing.chord_root_m
    z_upper_wing_m = wing.z_m + (0.5 * wing.thickness_to_chord + wing.camber) * chord_m
    z_bottom_m = z_top_fuselage_m - 0.45
    rows = ((0.0, z_top_fuselage_m - 0.12, 0.35),
            (length_ahead_m + 0.3 * chord_m, z_upper_wing_m, 0.92),
            (length_ahead_m + chord_m + 0.3, z_upper_wing_m - 0.06, 0.88),
            (length_ahead_m + chord_m + length_aft_m, z_top_fuselage_m - 0.12, 0.3))
    stations = tuple(BodyStation(x_m=x_m, z_m=0.5 * (z_top_m + z_bottom_m), width_m=fraction_width * width_fuselage_m,
                                 height_m=z_top_m - z_bottom_m, exponent=2.6)
                     for x_m, z_top_m, fraction_width in rows)
    return BodySnapshot(stations=stations, x_nose_m=x_nose_m)


def halo_plan027_snapshot():
    """Halo reference from plan 027 (13,639 lb, solved 2026-10-04), drawn as Halo.

    Rotor, wing area and span are the solved plan 027 values; the wing taper
    (0.6, about the sized quarter chord) is an aesthetic modification, as the
    sizing uses a constant chord. The V-tail carries the solved horizontal- and
    vertical-tail areas by the projected-area rule; its aspect ratio is the
    horizontal tail's, and its taper, sweep and position are Halo-like assumptions. The fuselage is 11 m
    (length/depth about 5.4 in the Halo side still, against the XV-15 12.8 m the
    sizing uses), as wide as the XV-15 diameter and 2.0 m deep. Nacelle and
    dorsal fairing follow the stills. None of these shape choices is a sizing
    input yet (plan 032); in particular the V-tail sits about 2 m further
    forward than the sized tail position.
    """
    span_wing_m, area_wing_m2, x_le_wing_untapered_m = 11.441, 21.388, 4.079
    # Taper 0.6 at the solved area and span, quarter-chord line unswept through the sized quarter chord.
    taper_wing = 0.6
    chord_root_wing_m = 2 * area_wing_m2 / (span_wing_m * (1 + taper_wing))
    x_quarter_chord_m = x_le_wing_untapered_m + 0.25 * area_wing_m2 / span_wing_m
    height_fuselage_m, width_fuselage_m = 2.0, 5.5 * u.foot
    wing = SurfaceSnapshot(span_m=span_wing_m, chord_root_m=chord_root_wing_m,
                           chord_tip_m=taper_wing * chord_root_wing_m,
                           x_le_root_m=x_quarter_chord_m - 0.25 * chord_root_wing_m, z_m=1.2,
                           thickness_to_chord=0.23, camber=0.02, fraction_chord_sweep=0.25)
    return GeometrySnapshot(
        fuselage=halo_fuselage(length_m=11.0, width_m=width_fuselage_m, height_m=height_fuselage_m),
        wing=wing,
        wing_fairing=dorsal_wing_fairing(wing, z_top_fuselage_m=0.5 * height_fuselage_m,
                                         width_fuselage_m=width_fuselage_m),
        nacelle=NacelleSnapshot(length_m=3.4, width_m=0.65, height_m=1.0, fraction_chord_spindle=0.25,
                                offset_z_spindle_m=0.0, length_mast_m=1.0, offset_nose_m=0.35),
        rotor=RotorSnapshot(radius_m=4.582, count_blades=3, solidity=0.089),
        v_tail=v_tail_equivalent(area_horizontal_tail_m2=3.732, area_vertical_tail_m2=1.658, aspect_ratio=3.27,
                                 taper_ratio=0.5, sweep_le_deg=40.0, x_le_root_m=9.1, z_m=0.85,
                                 thickness_to_chord=0.12),
    )
