"""Calibrate `MomentumProfileRotor` on full-scale JVX proprotor data (Tier 12, plan 016).

Data: NASA/TM-2016-219070 (C. W. Acree, "Assessment of JVX Proprotor
Performance Data in Hover and Airplane-Mode Flight Conditions", 2016),
Appendix D, parsed by word coordinates into `data/rotors/` (public domain):
  jvx_hover.csv     Tables D-1 (Mtip 0.67-0.68) and D-2 (Mtip 0.73), CP and FM wind-corrected;
  jvx_airplane.csv  Phase II NFAC 40x80 Tables D-7a (conditions) joined with D-7c (performance).
JVX: 25 ft diameter, 3 blades, thrust-weighted solidity 0.1138 (Table 1).

The model is linear in its drag coefficients, so both fits are ordinary least
squares. kappa is not identifiable separately from the loading terms over
this data range (free fits give kappa < 1), so it is fixed at 1.15.
"""
import csv
from dataclasses import dataclass
from pathlib import Path

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor, profile_integral

data_dir = Path(__file__).resolve().parents[1] / "data" / "rotors"
solidity_jvx = 0.1138
kappa = 1.15


def _load(name):
    lines = [line for line in open(data_dir / name, encoding="utf-8") if not line.startswith("#")]
    rows = list(csv.reader(lines))
    return [dict(zip(rows[0], row)) for row in rows[1:]]


@dataclass(frozen=True)
class HoverData:
    table: tuple
    mach_tip: object
    blade_loading: object
    power_sigma: object
    figure_of_merit: object


@dataclass(frozen=True)
class AirplaneData:
    advance_ratio: object
    mach_tip: object
    blade_loading: object
    power_sigma: object
    efficiency: object


def load_hover():
    rows = _load("jvx_hover.csv")
    column = lambda key: np.array([float(r[key]) for r in rows])
    return HoverData(tuple(r["table"] for r in rows), column("mtip"), column("ct_sigma"), column("cp_sigma"), column("fm"))


def load_airplane():
    rows = _load("jvx_airplane.csv")
    column = lambda key: np.array([float(r[key]) for r in rows])
    return AirplaneData(column("lambda"), column("mtip"), column("ct_sigma"), column("cp_sigma"), column("eta"))


def _hover_induced_sigma(blade_loading, solidity=solidity_jvx):
    """kappa CT^1.5 / sqrt(2) / sigma."""
    return kappa * blade_loading**1.5 * solidity**0.5 / 2**0.5


def fit_hover_polar(data=None):
    """(d0, d1, d2) from Table D-1: CP/sigma - induced = (d0 + d1 x + d2 x^2) / 8."""
    data = data or load_hover()
    mask = np.array([t == "D-1" for t in data.table])
    x = data.blade_loading[mask]
    design = np.stack([np.ones_like(x), x, x**2], axis=1) / 8
    target = data.power_sigma[mask] - _hover_induced_sigma(x)
    return tuple(np.linalg.lstsq(design, target, rcond=None)[0])


def _airplane_base_sigma(data, polar):
    d0, d1, d2 = polar
    thrust_coefficient = data.blade_loading * solidity_jvx
    lam = data.advance_ratio
    inflow = -lam / 2 + np.sqrt(lam**2 / 4 + thrust_coefficient / 2)
    x = data.blade_loading / (1 + 3 * lam**2)
    return (thrust_coefficient * lam + kappa * thrust_coefficient * inflow) / solidity_jvx \
        + (d0 + d1 * x + d2 * x**2) * profile_integral(lam) / 2


def fit_airplane_increment(polar, data=None):
    """(a, b) of the airplane-mode drag increment a + b lambda^2."""
    data = data or load_airplane()
    weight = profile_integral(data.advance_ratio) / 2
    design = np.stack([weight, weight * data.advance_ratio**2], axis=1)
    target = data.power_sigma - _airplane_base_sigma(data, polar)
    return tuple(np.linalg.lstsq(design, target, rcond=None)[0])


def jvx_rotor(polar=None, increment=None):
    """A JVX-geometry rotor (25 ft, sigma 0.1138) with the given or default constants."""
    rotor = MomentumProfileRotor(area_disk_m2=np.pi * (12.5 * 0.3048)**2, solidity=solidity_jvx)
    if polar is not None:
        rotor = MomentumProfileRotor(area_disk_m2=rotor.area_disk_m2, solidity=solidity_jvx, drag_polar=tuple(polar),
                                     drag_increment_airplane=tuple(increment))
    return rotor


def predict(rotor, blade_loading, advance_ratio, mach_tip=0.68):
    """Rotor CP/sigma and FM or efficiency at test coefficients (any consistent dimensional point)."""
    atmosphere = asb.Atmosphere(altitude=0)
    speed_tip_m_s = mach_tip * float(atmosphere.speed_of_sound())
    speed_rad_s = speed_tip_m_s / float(rotor.radius_m())
    scale_N = float(atmosphere.density()) * rotor.area_disk_m2 * speed_tip_m_s**2
    thrust_N = blade_loading * rotor.solidity * scale_N
    model = rotor if advance_ratio == 0 else rotor.in_airplane_mode()
    result = model.evaluate(advance_ratio * speed_tip_m_s, atmosphere, thrust_N=thrust_N, speed_rad_s=speed_rad_s)
    power_sigma = float(result.shaft_power_W) / (scale_N * speed_tip_m_s) / rotor.solidity
    thrust_coefficient = blade_loading * rotor.solidity
    if advance_ratio == 0:
        merit = thrust_coefficient**1.5 / 2**0.5 / (power_sigma * rotor.solidity)
    else:
        merit = thrust_coefficient * advance_ratio / (power_sigma * rotor.solidity)
    return power_sigma, merit


@dataclass(frozen=True)
class CalibrationReport:
    polar: tuple
    increment: tuple
    hover_fm_rms: float
    hover_d2_power_error_max: float
    airplane_eta_rms: float
    airplane_eta_max: float


def calibration_report():
    hover, airplane = load_hover(), load_airplane()
    polar = fit_hover_polar(hover)
    increment = fit_airplane_increment(polar, airplane)
    rotor = jvx_rotor(polar, increment)
    fm_errors, d2_errors = [], []
    for table, mach, load, power, merit in zip(hover.table, hover.mach_tip, hover.blade_loading, hover.power_sigma,
                                               hover.figure_of_merit):
        p, fm = predict(rotor, load, 0.0, mach)
        (fm_errors if table == "D-1" else d2_errors).append(fm - merit if table == "D-1" else p / power - 1)
    eta_errors = [predict(rotor, load, lam, mach)[1] - eta for lam, mach, load, eta in
                  zip(airplane.advance_ratio, airplane.mach_tip, airplane.blade_loading, airplane.efficiency)]
    rms = lambda values: float(np.sqrt(np.mean(np.array(values)**2)))
    return CalibrationReport(polar, increment, rms(fm_errors), float(np.max(np.abs(np.array(d2_errors)))),
                             rms(eta_errors), float(np.max(np.abs(np.array(eta_errors)))))


if __name__ == "__main__":
    report = calibration_report()
    print(f"hover polar d0, d1, d2 = {report.polar}")
    print(f"airplane increment a, b = {report.increment}")
    print(f"hover FM RMS {report.hover_fm_rms:.4f}; D-2 (Mtip 0.73) power error max {report.hover_d2_power_error_max:.4f}")
    print(f"airplane efficiency RMS {report.airplane_eta_rms:.4f}, max {report.airplane_eta_max:.4f}")
