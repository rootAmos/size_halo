"""AFDD tiltrotor wing: XV-15 section calibration and cross-checks on the V-22 and Bell D266 (Tier 20, plan 024).

Data: `data/weights/tiltrotor_wing_reference.csv` (values with a source key per
row) and `data/weights/sources.csv` (citations).

1. XV-15 section calibration. The published XV-15 wing stiffness, materials and
   component weights (Acree et al. 1999, table 5) fix NDARC's torque-box
   structural efficiency, the spar-taper correction, the fairing and
   control-surface unit masses and the fittings fraction. These are the
   `TiltrotorWingMassModel` defaults.
2. XV-15 wing group: the model from the published wing modes and tip masses
   against the 873 lb group statement gives `Xv15MassFactors.wing_tiltrotor`.
3. Cross-checks with no further fitting:
   - V-22 FSD wing (about 2,470 lb, Popelka et al. 1995), composite;
   - Bell D266 wing group (1,886 lb) and rotor group (2,439 lb), from a 1968
     preliminary-design statement (Harris, SP-2015-215959 vol. III).
   Public V-22 and AW609 group weight statements were not found, so neither
   aircraft gives a full group-by-group check. Inputs that are not published
   are marked "assumed" where they are set.
"""
import csv
import pathlib
from dataclasses import dataclass, replace

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.surfaces import (TiltrotorWingMassModel, Wing, aluminium_wing_material,
                                               graphite_epoxy_wing_material)
from aircraft_closure.weights import afdd
from examples.xv15_reference import Xv15MassFactors, Xv15Reference, calibration_factors, mass_turboshaft_from_power_kg

data_path = pathlib.Path(__file__).resolve().parents[1] / "data" / "weights" / "tiltrotor_wing_reference.csv"
stiffness_lb_in2 = u.lbf * u.inch**2


def load_reference_data(path=data_path):
    """{(aircraft, item): value} in the CSV's own units (see its `unit` column)."""
    with open(path, newline="", encoding="utf-8") as f:
        return {(row["aircraft"], row["item"]): float(row["value"]) for row in csv.DictReader(f)}


@dataclass(frozen=True)
class WingSectionCalibration:
    efficiency_torque_box: float
    correction_spar_stiffness: float
    unit_mass_fairing_kg_m2: float
    unit_mass_control_surfaces_kg_m2: float
    fraction_fittings: float
    fraction_area_control_surfaces: float


def xv15_section_calibration(fraction_chord_torque_box=0.45, data=None):
    """NDARC section constants that reproduce the published XV-15 wing breakdown from its published stiffness.

    Torque box: the area NDARC needs for the published GJ, against the 567 lb box, gives e_tb. Spar caps: the cap
    area NDARC adds for the published chord and beam EI, against the 52 lb of spars, gives C_t (e_sp = 1).
    Fairings, control surfaces and fittings follow from their weights and areas. The torque-box chord ratio is
    not published (assumed 0.45).
    """
    data = data if data is not None else load_reference_data()
    x = lambda item: data[("xv15", item)]
    span_m = x("span_torque_box") * u.foot
    area_m2 = x("area_wing") * u.foot**2
    chord_m = area_m2 / span_m
    thickness_m = x("thickness_to_chord") * chord_m
    chord_box_m = fraction_chord_torque_box * chord_m
    psi = u.lbf / u.inch**2
    modulus_shear_Pa, modulus_Pa = x("modulus_shear_torque_box") * psi, x("modulus_torque_box") * psi
    modulus_spar_Pa, density_kg_m3 = x("modulus_spar") * psi, x("density") * u.lbm / u.inch**3
    factor_beam, factor_chord, factor_torsion, factor_spar_cap = afdd.section_form_factors_tiltrotor_wing(
        x("thickness_to_chord"), fraction_chord_torque_box)
    area_box_m2 = 4 * x("stiffness_torsion") * stiffness_lb_in2 / (modulus_shear_Pa * factor_torsion * thickness_m**2)
    area_cap_chord_m2 = max(0.0, x("stiffness_chord") * stiffness_lb_in2
                            - modulus_Pa * factor_chord * area_box_m2 * chord_box_m**2 / 4) / (modulus_spar_Pa
                                                                                             * chord_box_m**2 / 4)
    area_cap_beam_m2 = max(0.0, x("stiffness_beam") * stiffness_lb_in2
                           - modulus_Pa * factor_beam * area_box_m2 * thickness_m**2 / 4
                           - modulus_spar_Pa * factor_spar_cap * area_cap_chord_m2 * thickness_m**2 / 4) / (
        modulus_spar_Pa * thickness_m**2 / 4)
    area_control_m2 = (x("area_flaps") + x("area_flaperons")) * u.foot**2
    area_fairing_m2 = (span_m - x("width_attachment") * u.inch) * chord_m * (1 - fraction_chord_torque_box) \
        - area_control_m2
    mass_rest_lb = x("wing_torque_box") + x("wing_spars") + x("wing_control_surfaces") + x("wing_fairings")
    ratio_fittings = x("wing_fittings_other") / mass_rest_lb
    return WingSectionCalibration(
        efficiency_torque_box=area_box_m2 * density_kg_m3 * span_m / (x("wing_torque_box") * u.lbm),
        correction_spar_stiffness=x("wing_spars") * u.lbm / ((area_cap_chord_m2 + area_cap_beam_m2) * density_kg_m3
                                                              * span_m),
        unit_mass_fairing_kg_m2=x("wing_fairings") * u.lbm / area_fairing_m2,
        unit_mass_control_surfaces_kg_m2=x("wing_control_surfaces") * u.lbm / area_control_m2,
        fraction_fittings=ratio_fittings / (1 + ratio_fittings),
        fraction_area_control_surfaces=area_control_m2 / area_m2)


@dataclass(frozen=True)
class WingCheck:
    aircraft: str
    mass_actual_kg: float
    mass_predicted_kg: float              # model x 1 (uncalibrated)
    mass_predicted_calibrated_kg: float   # model x the XV-15 factor `wing_tiltrotor`
    masses: object                        # afdd.TiltrotorWingMasses
    mass_tip_kg: float
    note: str


def _evaluate(model, span_m, area_m2, mass_design_kg):
    wing = Wing(area_m2=area_m2, aspect_ratio=span_m**2 / area_m2, mass_model=model)
    return model.masses(wing, StructuralDesignCondition(mass_design_kg=mass_design_kg))


def v22_wing_check(factors=None, data=None):
    """V-22 FSD wing from the XV-15-calibrated AFDD model, graphite epoxy, XV-15 frequency placement.

    Tip mass is a framework model-chain estimate (no public V-22 nacelle weights): AFDD82 rotor (sigma 0.105,
    coning 1.55/rev assumed as the XV-15), AeroSandbox turboshaft at 6,150 shp, AFDD82 engine section (XV-15
    nacelle wetted area scaled with rotor radius squared, assumed), half the AFDD83 drive system (assumed at the
    tips; engine 15,000 rpm assumed), each times its XV-15 factor. Fuselage width 7.9 ft (CTR30 table, assumed for
    the V-22). The fold/rotate structure is excluded (fraction 0): the source's wing scope does not state it.
    """
    data = data if data is not None else load_reference_data()
    factors = factors if factors is not None else calibration_factors()
    v = lambda item: data[("v22", item)]
    radius_m = v("radius_rotor") * u.foot
    speed_tip_m_s = v("speed_tip_hover") * u.foot
    chord_blade_m = v("solidity") * np.pi * radius_m / 3
    power_engine_W = v("power_engine") * u.hp
    mass_rotor_kg = factors.rotor * afdd.mass_rotor_group_afdd82_kg(2, 3, radius_m, chord_blade_m, speed_tip_m_s, 1.55) / 2
    mass_engine_kg = factors.powerplant * mass_turboshaft_from_power_kg(power_engine_W)
    area_wetted_nacelle_m2 = 95 * u.foot**2 * (v("radius_rotor") / 12.5)**2
    mass_nacelle_kg = factors.powerplant * (
        afdd.mass_engine_support_afdd82_kg(2 * mass_engine_kg, 2) + afdd.mass_air_induction_afdd82_kg(2 * mass_engine_kg, 2)
        + afdd.mass_engine_cowling_afdd82_kg(2 * area_wetted_nacelle_m2)) / 2
    mass_drive_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd83_kg(
        2 * power_engine_W, speed_tip_m_s / radius_m, 15000 * u.rpm, 3, 0.6)
    mass_tip_kg = mass_rotor_kg + mass_engine_kg + mass_nacelle_kg + mass_drive_kg / 4
    model = TiltrotorWingMassModel(
        mass_tip_kg=mass_tip_kg, radius_gyration_pylon_m=0.222 * radius_m,
        speed_rotor_design_rad_s=v("speed_rotor_airplane") * u.rpm, width_fuselage_m=7.9 * u.foot,
        frequency_torsion_per_rev=Xv15Reference().frequency_torsion_wing_per_rev,
        frequency_beam_per_rev=Xv15Reference().frequency_beam_wing_per_rev,
        frequency_chord_per_rev=Xv15Reference().frequency_chord_wing_per_rev,
        thickness_to_chord=v("thickness_to_chord"), material=graphite_epoxy_wing_material())
    masses = _evaluate(model, v("span_wing") * u.foot, v("area_wing") * u.foot**2, v("mass_design") * u.lbm)
    mass_kg = float(masses.total())
    return WingCheck("V-22 FSD", v("wing_overall") * u.lbm, mass_kg, factors.wing_tiltrotor * mass_kg, masses,
                     float(mass_tip_kg), "wing without fold/rotate; tip mass from the framework model chain")


def d266_wing_check(factors=None, data=None, thickness_to_chord=0.23):
    """Bell D266 wing group (1968 design statement): aluminium, XV-15 frequency placement at the 297 rpm maximum
    airplane-mode rotor speed. Not published, assumed: t/c 0.23, fuselage width 6.5 ft (5.5 ft cargo bay plus
    structure), and half the AFDD83 drive system at the tips (proprotor gearboxes; the engines sit inboard).
    Tip mass otherwise from the statement: half the rotor and nacelle groups."""
    data = data if data is not None else load_reference_data()
    factors = factors if factors is not None else calibration_factors()
    d = lambda item: data[("d266", item)]
    radius_m = d("radius_rotor") * u.foot
    mass_drive_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd83_kg(
        2 * d("power_engine") * u.hp, d("speed_rotor_hover") * u.rpm, 13600 * u.rpm, 3, 0.6)
    mass_tip_kg = (d("rotor_group") + d("nacelle_group")) / 2 * u.lbm + mass_drive_kg / 4
    model = TiltrotorWingMassModel(
        mass_tip_kg=mass_tip_kg, radius_gyration_pylon_m=0.222 * radius_m,
        speed_rotor_design_rad_s=d("speed_rotor_airplane_max") * u.rpm, width_fuselage_m=6.5 * u.foot,
        frequency_torsion_per_rev=Xv15Reference().frequency_torsion_wing_per_rev,
        frequency_beam_per_rev=Xv15Reference().frequency_beam_wing_per_rev,
        frequency_chord_per_rev=Xv15Reference().frequency_chord_wing_per_rev,
        thickness_to_chord=thickness_to_chord, material=aluminium_wing_material())
    masses = _evaluate(model, d("span_wing") * u.foot, d("area_wing") * u.foot**2, d("mass_design") * u.lbm)
    mass_kg = float(masses.total())
    return WingCheck("Bell D266", d("wing_group") * u.lbm, mass_kg, factors.wing_tiltrotor * mass_kg, masses,
                     float(mass_tip_kg), "1968 preliminary-design statement; t/c and fuselage width assumed")


def d266_rotor_implied_solidity(factors=None, data=None, frequency_coning_per_rev=1.55):
    """Blade solidity at which the XV-15-calibrated AFDD82 rotor group equals the D266 statement (2,439 lb).

    The D266 blade chord is not in the public summary, so the rotor model is checked through the solidity it
    implies (one explicit Opti equality), to be judged against typical proprotors (0.08-0.11).
    """
    data = data if data is not None else load_reference_data()
    factors = factors if factors is not None else calibration_factors()
    d = lambda item: data[("d266", item)]
    radius_m = d("radius_rotor") * u.foot
    speed_tip_m_s = d("speed_rotor_hover") * u.rpm * radius_m
    opti = asb.Opti()
    solidity = opti.variable(init_guess=0.09, lower_bound=0.01)
    mass_kg = factors.rotor * afdd.mass_rotor_group_afdd82_kg(2, 3, radius_m, solidity * np.pi * radius_m / 3,
                                                               speed_tip_m_s, frequency_coning_per_rev)
    opti.subject_to(mass_kg / (d("rotor_group") * u.lbm) == 1)
    return float(opti.solve(verbose=False).value(solidity))


def xv15_frequency_stiffness_ratios(reference=Xv15Reference(), data=None):
    """Model / published XV-15 stiffness (torsion, beam, chord) when the model is driven by the published modes."""
    data = data if data is not None else load_reference_data()
    from examples.xv15_reference import xv15_wing_mass_model
    r = reference
    masses = _evaluate(xv15_wing_mass_model(r), r.area_wing_m2**0.5 * r.aspect_ratio_wing**0.5, r.area_wing_m2,
                       r.mass_design_kg)
    return (float(masses.stiffness_torsion_Nm2) / (data[("xv15", "stiffness_torsion")] * stiffness_lb_in2),
            float(masses.stiffness_beam_Nm2) / (data[("xv15", "stiffness_beam")] * stiffness_lb_in2),
            float(masses.stiffness_chord_Nm2) / (data[("xv15", "stiffness_chord")] * stiffness_lb_in2))


if __name__ == "__main__":
    print(xv15_section_calibration())
    factors = calibration_factors()
    print(f"XV-15 factors: wing (Raymer) {factors.wing:.3f}, wing_tiltrotor {factors.wing_tiltrotor:.3f}")
    print("XV-15 model/published stiffness (torsion, beam, chord):", xv15_frequency_stiffness_ratios())
    for check in (v22_wing_check(factors), d266_wing_check(factors)):
        print(f"{check.aircraft}: actual {check.mass_actual_kg / u.lbm:,.0f} lb, model {check.mass_predicted_kg / u.lbm:,.0f}"
              f" lb (x factor {check.mass_predicted_calibrated_kg / u.lbm:,.0f} lb), tip {check.mass_tip_kg / u.lbm:,.0f} lb")
    print(f"D266 implied solidity: {d266_rotor_implied_solidity(factors):.3f}")
