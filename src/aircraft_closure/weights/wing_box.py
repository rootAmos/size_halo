"""Bottom-up (class II-1/2) tiltrotor wing box with its sized layout, no calibration factor (plan 054).

Each primary item is sized for its own load case at fixed spanwise stations, and the result keeps the layout
(skin, stringer, web and spar-cap sizes along the span, stringer pitch and height, rib pitch and thickness) so the
OpenVSP structure and the CalculiX check can use it. Everything else is Torenbeek's statistics.

- Loads (Torenbeek LR-693 eq. 17; NDARC jump take-off): ultimate flight with elliptic lift less tip inertia, and
  the jump take-off, 1.5 x n_jump x (W/2 - tip weight) at the tip; the larger at each station (downward flight
  bending is smaller than the jump for a tiltrotor's tip masses). Airplane-mode inertia torque of the tip mass
  offset.
- Stiffness: the AFDD frequency placement (`afdd.wing_tiltrotor_mode_inertias`) as a floor at every station.
- Skin (Howe 15.2.2-15.2.3): at least the Bredt-Batho wall for the torsion stiffness, the torque shear flow and
  minimum gauge.
- Covers, skin plus stringers (Howe 15.2.4, 15.2.7; EMWET eqs. 25-27): lower at the tension allowable less 10 %
  for torsion (Torenbeek 8.3), upper at min(compression allowable, stiffened-panel buckling F sqrt(N E / L)).
  Stringers are a fixed share of the bending cover; the skin is the rest, or the torsion skin if thicker. Stringer
  pitch from skin buckling between stringers at the cover stress (Howe 16.2.3, K = 3.62); height = pitch / 3.5;
  Zed section area 1.8 h t.
- Webs (Howe 15.2.6): vertical shear plus torque shear flow, + 20 % (Torenbeek 8.3), at least the torsion skin.
- Spar caps: the beam and chord bending stiffness still missing.
- Root peak (plan 043): near the fuselage side the bending allowables are divided by the CalculiX peak factor.
- Ribs (Torenbeek eq. 47) at his 0.75 m statistical pitch, scaled by 0.75 m / pitch; non-optimum sheet and joint
  allowance (eq. 49); wing-body and tip-powerplant mountings (sec. 8.2, eq. 50); fixed leading and trailing edges
  (eqs. 52, 53); control surfaces (eq. 56) with supports (sec. 9.6).

Everything accepts CasADi symbols; `smoothing` > 0 rounds the max steps for gradient-based sizing.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np

from aircraft_closure.weights.afdd import _positive_part, wing_tiltrotor_mode_inertias

g_m_s2 = 9.80665


@dataclass(frozen=True)
class WingBoxMaterial:
    """Covers and webs (`modulus_Pa`, `modulus_shear_Pa`) and spar caps (`modulus_cap_Pa`); ultimate strains."""
    density_kg_m3: float
    modulus_Pa: float
    modulus_cap_Pa: float
    modulus_shear_Pa: float
    strain_tension: float
    strain_compression: float
    strain_shear: float
    thickness_min_m: float              # minimum gauge
    thickness_non_optimum_m: float      # Torenbeek eq. 49 extra sheet for joints, splices, cut-outs
    thickness_min_web_m: float = 0.0    # spar-web minimum laminate (sandwich faces together); the larger applies
    # Hot/wet environment (plan 054): the modulus in every buckling check and the shear allowable are multiplied by
    # these. The cover strain allowables are design values already (EMWET cut-offs) and are not knocked down.
    knockdown_modulus_environment: float = 1.0
    knockdown_shear_environment: float = 1.0

    @property
    def strain_ultimate(self):
        """The tension allowable, under the name the AFDD wing material uses (sizing strain margins)."""
        return self.strain_tension


def aluminium_wing_box_material():
    """XV-15 aluminium (Acree et al. table 5: E 10e6 psi, G 3.8e6 psi, 0.1 lb/in3, strain 0.0068); compression at
    0.85 of tension, shear stress 0.55 of the tensile; 0.8 mm minimum gauge; 1 mm non-optimum (built-up)."""
    psi = 6894.757293168361
    return WingBoxMaterial(density_kg_m3=2767.99, modulus_Pa=10e6 * psi, modulus_cap_Pa=10e6 * psi,
                           modulus_shear_Pa=3.8e6 * psi, strain_tension=0.0068, strain_compression=0.0068 * 0.85,
                           strain_shear=0.55 * 0.0068 * 10e6 / 3.8e6, thickness_min_m=0.8e-3,
                           thickness_non_optimum_m=1.0e-3)


def graphite_epoxy_wing_box_material():
    """Graphite epoxy (Acree et al. table 5: quasi-isotropic box E 9e6 psi, unidirectional caps 18e6 psi, G 3.75e6
    psi, 0.06 lb/in3, strain 0.0047); EMWET cutoff strains for compression (0.0045) and shear (0.006); 1 mm minimum
    gauge; 0.5 mm non-optimum (Torenbeek's value for integral construction: co-cured, fewer joints). Spar webs at
    least 1.5 mm (eight balanced +-45 plies); hot/wet (82 C wet) knockdowns 0.90 on the modulus for stability and
    0.80 on the shear allowable (matrix-dominated, typical carbon-epoxy design values)."""
    psi = 6894.757293168361
    return WingBoxMaterial(density_kg_m3=1660.79, modulus_Pa=9e6 * psi, modulus_cap_Pa=18e6 * psi,
                           modulus_shear_Pa=3.75e6 * psi, strain_tension=0.0047, strain_compression=0.0045,
                           strain_shear=0.006, thickness_min_m=1.0e-3, thickness_non_optimum_m=0.5e-3,
                           thickness_min_web_m=1.5e-3, knockdown_modulus_environment=0.90,
                           knockdown_shear_environment=0.80)


@dataclass(frozen=True)
class WingBoxLayout:
    """Structural layout and construction constants (assumed values from Howe and Torenbeek where noted)."""
    fraction_chord_front_spar: Any = 0.15
    fraction_chord_rear_spar: Any = 0.60
    pitch_rib_m: Any = 0.5
    ratio_depth_effective: float = 0.95     # cover centroid at this fraction of the mean box depth
    efficiency_panel: float = 0.75          # stiffened-panel efficiency F: Z stringers 0.8 at best (Torenbeek, Howe);
                                            # CalculiX buckled the 0.8 Halo covers at 0.97 x ultimate (plan 054)
    fraction_stringer: float = 0.35         # stringer share of the bending cover (Howe 15.2.7)
    ratio_pitch_height_stringer: float = 3.5
    coefficient_skin_buckling: float = 3.62  # simply supported plate between stringers (Howe 16.2.3)
    # Spar webs (plan 054): CalculiX buckled 1 mm strength-sized Halo webs in shear at 0.99 x ultimate. "sandwich":
    # carbon faces (strength, minimum laminate) on a honeycomb core deep enough for no shear buckling of the
    # depth x rib-pitch panel at ultimate (k_s = 5.35 + 4 (b / a)^2, simply supported); "plate": a solid web thick
    # enough on its own (K = 4.84 + 3.62 (b / a)^2 on E (t / b)^2); "none": strength and gauge only.
    construction_web: str = "sandwich"
    thickness_core_min_m: float = 6.0e-3        # thinnest practical honeycomb
    density_core_kg_m3: float = 48.0            # Nomex honeycomb, 3 lb/ft3
    areal_mass_adhesive_kg_m2: float = 0.30     # two film-adhesive layers
    factor_peak_root: Any = 1.0             # root peak over beam theory; CalculiX on the sized covers: 0.97
    length_peak_root_m: float = 1.0         # extent of the root peak outboard of the fuselage side (FE band)
    fraction_area_control_surfaces: Any = 0.185   # XV-15 flaps and flaperons
    pitch_rib_reference_m: float = 0.75     # Torenbeek's statistical mean rib pitch
    count_stations: int = 12
    # NDARC's single-mode torsion relation over-predicts the wing torsion frequency: the XV-15 driven by its
    # published modes needs 0.574 x its published GJ (`tiltrotor_wing_calibration.xv15_frequency_stiffness_ratios`),
    # and CalculiX on the Halo wing box gives (58.5 / 77.6)^2 = 0.568 clamped (plan 054). Torsion frequency =
    # this factor x the NDARC value, sqrt(0.572); 1.0 is NDARC.
    correction_torsion_frequency: float = 0.756
    # Plan 062: effective beam (flapwise) bending stiffness over the box value, the square of the CalculiX
    # free-free over bottom-up beam frequency (0.94^2 on the free wing D); 1.0 is the box value. The caps are sized
    # so the effective stiffness meets the requirement, and the effective stiffness is what flutter sees.
    factor_stiffness_beam: float = 1.0


@dataclass(frozen=True)
class WingBoxSizing:
    """Result of `size_wing_box`. Masses in kg for the whole wing; per-station tuples run from the fuselage side to
    the tip at `fraction_stations` of the panel; stiffness and frequencies are the realized effective values."""
    mass_covers_kg: Any
    mass_webs_kg: Any
    mass_caps_kg: Any
    mass_ribs_kg: Any
    mass_non_optimum_kg: Any
    mass_mountings_kg: Any
    mass_fairing_kg: Any
    mass_control_surfaces_kg: Any
    stiffness_torsion_Nm2: Any
    stiffness_beam_Nm2: Any
    stiffness_chord_Nm2: Any
    frequency_torsion_rad_s: Any
    frequency_beam_rad_s: Any
    frequency_chord_rad_s: Any
    moment_jump_ultimate_Nm: Any
    strain_root_lower_cover: Any        # ultimate tension strain at the root station, peak factor included
    fraction_stations: tuple
    y_stations_m: tuple
    width_box_m: Any
    depth_box_m: Any
    thickness_skin_upper_m: tuple
    thickness_skin_lower_m: tuple
    thickness_stringers_upper_m: tuple   # smeared over the cover width
    thickness_stringers_lower_m: tuple
    pitch_stringer_upper_m: tuple
    height_stringer_upper_m: tuple
    thickness_web_m: tuple               # web laminate (sandwich: both faces together)
    thickness_core_web_m: tuple          # sandwich-web core (0: solid web)
    area_caps_m2: tuple                  # all four caps at a station
    pitch_rib_m: Any
    thickness_rib_m: Any

    def mass_primary_kg(self):
        return self.mass_covers_kg + self.mass_webs_kg + self.mass_caps_kg + self.mass_ribs_kg \
            + self.mass_non_optimum_kg

    def total(self):
        return self.mass_primary_kg() + self.mass_mountings_kg + self.mass_fairing_kg + self.mass_control_surfaces_kg


def naca_thickness_fraction(fraction_chord, thickness_to_chord):
    """Full thickness / chord of a NACA 4-digit section at `fraction_chord` (open trailing edge)."""
    x = fraction_chord
    return 10 * thickness_to_chord * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3
                                      - 0.1015 * x**4)


def moment_elliptic_fraction(eta):
    """Bending moment of an elliptic lift L on a semispan s at eta = y / s, as M / (L s) (Torenbeek eqs. 16-17)."""
    return 2 / (3 * np.pi) * ((2 + eta**2) * np.sqrt(1 - eta**2) - 3 * eta * np.arccos(eta))


def shear_elliptic_fraction(eta):
    """Shear of an elliptic lift L outboard of eta = y / s, as V / L."""
    return 2 / np.pi * (np.arccos(eta) - eta * np.sqrt(1 - eta**2))


def _max(a, b, scale, smoothing):
    return b + _positive_part(a - b, smoothing * scale)


def size_wing_box(span_m, chord_m, thickness_to_chord, mass_design_kg, mass_tip_kg, radius_gyration_pylon_m,
                  offset_cg_tip_m, speed_rotor_design_rad_s, frequency_torsion_per_rev, frequency_beam_per_rev,
                  frequency_chord_per_rev, width_attachment_m, material, layout=WingBoxLayout(),
                  load_factor_ultimate=4.5, load_factor_jump=2.0, count_rotors=2, smoothing=0.0,
                  stiffness_required_Nm2=None, thickness_shape=None):
    """Size the wing box at `layout.count_stations` midpoint stations per panel (constant chord, span between rotor
    centrelines). `stiffness_required_Nm2` = (GJ, EI beam, EI chord) overrides the frequency placement (e.g. a
    published wing's stiffness). `thickness_shape` (plan 062): the section's local thickness over its maximum at a
    chord fraction (numbers), so the depths are `thickness_to_chord` x the shape; None: the NACA 4-digit formula.
    Returns `WingBoxSizing`."""
    m, lay = material, layout
    semispan_m = span_m / 2
    y_side_m = width_attachment_m / 2
    length_panel_m = semispan_m - y_side_m
    weight_N = mass_design_kg * g_m_s2
    tip_weight_N = mass_tip_kg * g_m_s2
    # Spar depths (webs, caps) and the airfoil's mean depth across the box (covers, enclosed area): the section is
    # deepest between the spars, so the spar average understates the torsion area (CalculiX GJ 1.41 x, plan 054).
    def thickness_fraction(fraction_chord):
        if thickness_shape is None:
            return naca_thickness_fraction(fraction_chord, thickness_to_chord)
        return thickness_to_chord * thickness_shape(fraction_chord)

    depth_m = 0.5 * (thickness_fraction(lay.fraction_chord_front_spar)
                     + thickness_fraction(lay.fraction_chord_rear_spar)) * chord_m
    fractions_box = [lay.fraction_chord_front_spar + (lay.fraction_chord_rear_spar - lay.fraction_chord_front_spar)
                     * k / 8 for k in range(9)]
    weights_simpson = [1, 4, 2, 4, 2, 4, 2, 4, 1]
    depth_box_m = sum(w * thickness_fraction(x) for w, x in zip(weights_simpson, fractions_box)) \
        / 24 * chord_m
    width_m = (lay.fraction_chord_rear_spar - lay.fraction_chord_front_spar) * chord_m
    area_enclosed_m2 = width_m * depth_box_m
    perimeter_m = 2 * (width_m + depth_m)
    depth_effective_m = lay.ratio_depth_effective * depth_box_m

    bending_mass_kg_m3, torsion_inertia_kg_m3 = wing_tiltrotor_mode_inertias(
        span_m, mass_tip_kg, radius_gyration_pylon_m, mass_design_kg, count_rotors)
    if stiffness_required_Nm2 is None:
        torsion_req_Nm2 = (frequency_torsion_per_rev * speed_rotor_design_rad_s
                           / lay.correction_torsion_frequency) ** 2 * torsion_inertia_kg_m3
        beam_req_Nm2 = (frequency_beam_per_rev * speed_rotor_design_rad_s) ** 2 * bending_mass_kg_m3
        chord_req_Nm2 = (frequency_chord_per_rev * speed_rotor_design_rad_s) ** 2 * bending_mass_kg_m3
    else:
        torsion_req_Nm2, beam_req_Nm2, chord_req_Nm2 = stiffness_required_Nm2

    sigma_tension_Pa = 0.90 * m.modulus_Pa * m.strain_tension
    sigma_compression_Pa = m.modulus_Pa * m.strain_compression
    tau_Pa = m.modulus_shear_Pa * m.strain_shear * m.knockdown_shear_environment
    modulus_stability_Pa = m.modulus_Pa * m.knockdown_modulus_environment
    thickness_web_min_m = max(m.thickness_min_m, m.thickness_min_web_m)
    torque_Nm = load_factor_ultimate * tip_weight_N * offset_cg_tip_m
    flow_torque_N_m = torque_Nm / (2 * area_enclosed_m2)
    thickness_skin_torsion_m = torsion_req_Nm2 * perimeter_m / (4 * area_enclosed_m2**2 * m.modulus_shear_Pa)
    thickness_skin_min_m = _max(_max(thickness_skin_torsion_m, flow_torque_N_m / tau_Pa, m.thickness_min_m,
                                     smoothing), m.thickness_min_m, m.thickness_min_m, smoothing)

    fractions = tuple((i + 0.5) / lay.count_stations for i in range(lay.count_stations))
    rows = []
    for zeta in fractions:
        y_m = y_side_m + zeta * length_panel_m
        eta = np.fmin(y_m / semispan_m, 0.995)          # guards iterates with the fuselage side beyond the tip
        arm_tip_m = semispan_m - y_m
        lift_panel_N = load_factor_ultimate * weight_N / 2
        moment_flight_Nm = lift_panel_N * semispan_m * moment_elliptic_fraction(eta) \
            - load_factor_ultimate * tip_weight_N * arm_tip_m
        shear_flight_N = lift_panel_N * shear_elliptic_fraction(eta) - load_factor_ultimate * tip_weight_N
        force_jump_N = 1.5 * load_factor_jump * _positive_part(weight_N / 2 - tip_weight_N, smoothing * weight_N / 2)
        moment_jump_Nm = force_jump_N * arm_tip_m
        moment_Nm = _max(moment_flight_Nm, moment_jump_Nm, force_jump_N * length_panel_m, smoothing)
        shear_N = _max(shear_flight_N, force_jump_N, force_jump_N, smoothing)
        # Root peak: allowables divided by the peak factor near the fuselage side, blending out over the FE band.
        distance_m = y_m - y_side_m
        factor_peak = 1 + (lay.factor_peak_root - 1) * 0.5 * (1 - np.tanh((distance_m - lay.length_peak_root_m)
                                                                          / (0.1 * lay.length_peak_root_m)))
        intensity_N_m = _positive_part(moment_Nm, smoothing * force_jump_N * length_panel_m) / (depth_effective_m
                                                                                                * width_m)
        cover_lower_m = factor_peak * intensity_N_m / sigma_tension_Pa
        cover_upper_material_m = factor_peak * intensity_N_m / sigma_compression_Pa
        cover_upper_buckling_m = np.sqrt(intensity_N_m * lay.pitch_rib_m / modulus_stability_Pa) / lay.efficiency_panel
        cover_upper_m = _max(cover_upper_buckling_m, cover_upper_material_m, cover_upper_material_m, smoothing)
        stringers_lower_m = lay.fraction_stringer * cover_lower_m
        stringers_upper_m = lay.fraction_stringer * cover_upper_m
        skin_lower_m = _max(thickness_skin_min_m, cover_lower_m - stringers_lower_m, thickness_skin_min_m, smoothing)
        skin_upper_m = _max(thickness_skin_min_m, cover_upper_m - stringers_upper_m, thickness_skin_min_m, smoothing)
        flow_web_N_m = shear_N / (2 * depth_m) + flow_torque_N_m
        web_m = _max(1.2 * flow_web_N_m / tau_Pa, _max(thickness_skin_min_m, thickness_web_min_m,
                                                       thickness_web_min_m, smoothing),
                     thickness_web_min_m, smoothing)
        core_web_m = 0.0
        # Shear buckling of the web panel, b the shorter side (depth or rib pitch), a the longer.
        side_short_m = np.fmin(depth_m, lay.pitch_rib_m)
        ratio_web = side_short_m / np.fmax(depth_m, lay.pitch_rib_m)
        flow_buckling_N_m = _positive_part(flow_web_N_m, smoothing * tau_Pa * m.thickness_min_m)
        if lay.construction_web == "plate":
            coefficient = 4.84 + 3.62 * ratio_web**2
            web_buckling_m = (flow_buckling_N_m * side_short_m**2 / (coefficient * modulus_stability_Pa)) ** (1 / 3)
            web_m = _max(web_m, web_buckling_m, web_buckling_m, smoothing)
        elif lay.construction_web == "sandwich":
            # Faces t / 2 each at a centroid distance h: D = E (t / 2) h^2 / 2 >= q b^2 / (k_s pi^2).
            coefficient = 5.35 + 4.0 * ratio_web**2
            stiffness_required_Nm = flow_buckling_N_m * side_short_m**2 / (coefficient * np.pi**2)
            distance_m = np.sqrt(4 * stiffness_required_Nm / (modulus_stability_Pa * web_m))
            core_web_m = _max(distance_m - web_m / 2, lay.thickness_core_min_m, lay.thickness_core_min_m, smoothing)
        covers_m = skin_lower_m + stringers_lower_m + skin_upper_m + stringers_upper_m
        beam_box_Nm2 = m.modulus_Pa * width_m * covers_m * (depth_effective_m / 2) ** 2 \
            + m.modulus_Pa * 2 * web_m * depth_m**3 / 12
        chord_box_Nm2 = m.modulus_Pa * covers_m * width_m**3 / 12 + m.modulus_Pa * 2 * depth_m * web_m * (width_m / 2) ** 2
        area_caps_m2 = _max(_positive_part(beam_req_Nm2 / lay.factor_stiffness_beam - beam_box_Nm2,
                                           smoothing * beam_req_Nm2)
                            / (m.modulus_cap_Pa * (depth_m / 2) ** 2),
                            _positive_part(chord_req_Nm2 - chord_box_Nm2, smoothing * chord_req_Nm2)
                            / (m.modulus_cap_Pa * (width_m / 2) ** 2), beam_req_Nm2 / (m.modulus_cap_Pa * depth_m**2),
                            smoothing)
        # Stringer pitch: skin between stringers buckles at the upper cover's working stress.
        stress_upper_Pa = np.fmax(intensity_N_m / (cover_upper_m + 1e-9), 1e6)   # floor: unloaded outboard panels
        pitch_stringer_m = skin_upper_m * np.sqrt(lay.coefficient_skin_buckling * modulus_stability_Pa / stress_upper_Pa)
        torsion_Nm2 = 4 * area_enclosed_m2**2 * m.modulus_shear_Pa / (
            width_m / skin_upper_m + width_m / skin_lower_m + 2 * depth_m / web_m)
        rows.append(dict(
            y=y_m, moment=moment_Nm, arm=arm_tip_m, lower=skin_lower_m + stringers_lower_m,
            skin_lower=skin_lower_m, skin_upper=skin_upper_m, str_lower=stringers_lower_m, str_upper=stringers_upper_m,
            pitch_str=pitch_stringer_m, web=web_m, core=core_web_m, caps=area_caps_m2,
            beam=beam_box_Nm2 + m.modulus_cap_Pa * area_caps_m2 * (depth_m / 2) ** 2,
            chord=chord_box_Nm2 + m.modulus_cap_Pa * area_caps_m2 * (width_m / 2) ** 2,
            torsion=torsion_Nm2, covers=covers_m, factor_peak=factor_peak))

    dy_m = length_panel_m / lay.count_stations                        # midpoint rule
    both = lambda key, per_m: 2 * sum(per_m(r) for r in rows) * dy_m + per_m(rows[0]) * width_attachment_m
    rho = m.density_kg_m3
    mass_covers_kg = both("covers", lambda r: rho * width_m * r["covers"])
    core_areal = lambda r: (lay.density_core_kg_m3 * r["core"] + lay.areal_mass_adhesive_kg_m2
                            if lay.construction_web == "sandwich" else 0.0)
    mass_webs_kg = both("webs", lambda r: 2 * depth_m * (rho * r["web"] + core_areal(r)))
    mass_caps_kg = both("caps", lambda r: rho * r["caps"])

    # Realized effective stiffness: flexibility-weighted for bending (weight (L - y)^2), uniform for torsion.
    weight_bending = [r["arm"] ** 2 for r in rows]
    stiffness_beam_Nm2 = lay.factor_stiffness_beam * sum(weight_bending) / sum(
        w / r["beam"] for w, r in zip(weight_bending, rows))
    stiffness_chord_Nm2 = sum(weight_bending) / sum(w / r["chord"] for w, r in zip(weight_bending, rows))
    stiffness_torsion_Nm2 = len(rows) / sum(1 / r["torsion"] for r in rows)

    area_wing_m2 = span_m * chord_m
    area_exposed_m2 = (span_m - width_attachment_m) * chord_m
    area_section_m2 = 0.685 * thickness_to_chord * chord_m**2          # NACA 4-digit section area
    mass_ribs_kg = 0.5e-3 * rho * area_wing_m2 * (1.0 + thickness_to_chord * chord_m) \
        * lay.pitch_rib_reference_m / lay.pitch_rib_m
    count_ribs = 2 * length_panel_m / lay.pitch_rib_m + 2
    thickness_rib_m = mass_ribs_kg / (rho * count_ribs * area_section_m2)
    mass_non_optimum_kg = rho * area_wing_m2 * (1 + 2 * thickness_to_chord) * m.thickness_non_optimum_m
    mass_mountings_kg = 0.001 * mass_design_kg + 0.025 * (1 + 0.2 * count_rotors) * count_rotors * mass_tip_kg
    root_weight = np.sqrt(weight_N / 1e6)
    area_controls_m2 = lay.fraction_area_control_surfaces * area_exposed_m2
    area_leading_m2 = lay.fraction_chord_front_spar * area_exposed_m2
    area_trailing_m2 = (1 - lay.fraction_chord_rear_spar) * area_exposed_m2 - area_controls_m2
    mass_fairing_kg = (75 * (1 + root_weight) * area_leading_m2 + 60 * (1 + 1.6 * root_weight) * area_trailing_m2) \
        / g_m_s2
    mass_control_surfaces_kg = 1.2 * 125 * (1 + 0.5 * (weight_N / 1e6) ** 0.25) * area_controls_m2 / g_m_s2

    root = rows[0]
    return WingBoxSizing(
        mass_covers_kg=mass_covers_kg, mass_webs_kg=mass_webs_kg, mass_caps_kg=mass_caps_kg,
        mass_ribs_kg=mass_ribs_kg, mass_non_optimum_kg=mass_non_optimum_kg, mass_mountings_kg=mass_mountings_kg,
        mass_fairing_kg=mass_fairing_kg, mass_control_surfaces_kg=mass_control_surfaces_kg,
        stiffness_torsion_Nm2=stiffness_torsion_Nm2, stiffness_beam_Nm2=stiffness_beam_Nm2,
        stiffness_chord_Nm2=stiffness_chord_Nm2,
        frequency_torsion_rad_s=lay.correction_torsion_frequency * np.sqrt(stiffness_torsion_Nm2 / torsion_inertia_kg_m3),
        frequency_beam_rad_s=np.sqrt(stiffness_beam_Nm2 / bending_mass_kg_m3),
        frequency_chord_rad_s=np.sqrt(stiffness_chord_Nm2 / bending_mass_kg_m3),
        moment_jump_ultimate_Nm=1.5 * load_factor_jump * (weight_N / 2 - tip_weight_N) * length_panel_m,
        strain_root_lower_cover=root["factor_peak"] * root["moment"] * (depth_m / 2)
        / (root["beam"]),
        fraction_stations=fractions, y_stations_m=tuple(r["y"] for r in rows), width_box_m=width_m,
        depth_box_m=depth_m,
        thickness_skin_upper_m=tuple(r["skin_upper"] for r in rows),
        thickness_skin_lower_m=tuple(r["skin_lower"] for r in rows),
        thickness_stringers_upper_m=tuple(r["str_upper"] for r in rows),
        thickness_stringers_lower_m=tuple(r["str_lower"] for r in rows),
        pitch_stringer_upper_m=tuple(r["pitch_str"] for r in rows),
        height_stringer_upper_m=tuple(r["pitch_str"] / lay.ratio_pitch_height_stringer for r in rows),
        thickness_web_m=tuple(r["web"] for r in rows), thickness_core_web_m=tuple(r["core"] for r in rows),
        area_caps_m2=tuple(r["caps"] for r in rows),
        pitch_rib_m=lay.pitch_rib_m, thickness_rib_m=thickness_rib_m)
