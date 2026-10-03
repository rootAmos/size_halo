"""XV-15 power checks: engine lapse, hover figure of merit with download, part-power fuel use.

Digitized from NASA TM X-62407 (1975) by pixel crossings of the scanned
figures (accessed 2026-10-02, https://ntrs.nasa.gov/citations/19750016648):
  fig. 6.2.2  rotor shaft power available per engine, take-off rating,
              helicopter mode, 740 ft/s, standard day (about +/- 15 shp);
  fig. 5.1.1  OGE hover gross weight, twin engine, take-off power, standard
              day, UT/W = 1.0 (about +/- 60 lb); sec. 5.1 states 7 % wing
              download and 0.93 transmission efficiency;
  sec. 6.2    LTC1K-4K sfc at contingency, take-off, military and normal.
SP-4517 appendix A gives an 8,650 ft OGE hover ceiling (rating not stated).

The engine model is the framework's `SimpleTurboshaft` on a rotor-shaft
basis (sea-level power available = the digitized 1,374 shp, 0.887 x the
1,550 shp engine rating); the rotor is `ActuatorDiskPropulsor`, whose
`coefficient_of_performance` is the hover figure of merit.
"""
from dataclasses import dataclass

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import (GeissPartPowerModel, SimpleTurboshaft,
                                                               density_sea_level_kg_m3)
from examples.xv15_reference import Xv15Reference

acceleration_gravity_m_s2 = 9.80665


@dataclass(frozen=True)
class Xv15PowerData:
    altitude_power_m: tuple = tuple(h * u.foot for h in (0, 4000, 8000, 12000, 16000, 20000))
    power_available_rotor_W: tuple = tuple(p * u.hp for p in (1374, 1285, 1181, 1061, 942, 788))
    altitude_hover_m: tuple = tuple(h * u.foot for h in (229, 2214, 4198, 6183, 8168, 10153, 20000))
    mass_hover_kg: tuple = tuple(w * u.lbm for w in (15578, 14897, 14225, 13554, 12882, 12211, 8850))
    download_fraction_hover: float = 0.07
    altitude_hover_ceiling_published_m: float = 8650 * u.foot     # SP-4517, rating not stated
    power_ratings_engine_W: tuple = tuple(p * u.hp for p in (1802, 1550, 1401, 1250))
    sfc_ratings_lb_hp_h: tuple = (0.564, 0.584, 0.601, 0.622)


def density_ratio(altitude_m):
    return asb.Atmosphere(altitude=altitude_m).density() / density_sea_level_kg_m3


def fit_lapse_exponent(data=Xv15PowerData()):
    """Least squares on ln(P/P_SL) = n ln(sigma) through the origin (post-processing of fixed data)."""
    x = np.log(np.array([density_ratio(h) for h in data.altitude_power_m[1:]]))
    y = np.log(np.array(data.power_available_rotor_W[1:]) / data.power_available_rotor_W[0])
    return float(np.sum(x * y) / np.sum(x * x))


def xv15_engine(lapse_exponent, data=Xv15PowerData()):
    """One engine on a rotor-shaft basis."""
    return SimpleTurboshaft(power_rated_W=data.power_available_rotor_W[0], lapse_exponent=lapse_exponent)


def xv15_rotor(figure_of_merit, reference=Xv15Reference()):
    return ActuatorDiskPropulsor(area_disk_m2=np.pi * reference.radius_rotor_m**2,
                                 coefficient_of_performance=figure_of_merit)


def thrust_per_rotor_N(mass_kg, data=Xv15PowerData(), reference=Xv15Reference()):
    """UT/W = 1: rotors carry the weight plus the download."""
    return mass_kg * acceleration_gravity_m_s2 / (1 - data.download_fraction_hover) / reference.count_rotors


def calibrate_figure_of_merit(lapse_exponent, data=Xv15PowerData()):
    """Figure of merit that makes the sea-level hover weight use exactly the power available there."""
    atmosphere = asb.Atmosphere(altitude=data.altitude_hover_m[0])
    power_ideal_W = xv15_rotor(1.0).evaluate(0.0, atmosphere,
                                             thrust_N=thrust_per_rotor_N(data.mass_hover_kg[0], data)).shaft_power_W
    return float(power_ideal_W / xv15_engine(lapse_exponent, data).power_available_W(atmosphere))


def hover_mass_kg(altitude_m, figure_of_merit, lapse_exponent, data=Xv15PowerData()):
    """Largest OGE hover mass at an altitude: rotor power required = power available (one Opti equality)."""
    opti = asb.Opti()
    mass_kg = opti.variable(init_guess=6000.0, scale=1000.0, lower_bound=100.0)
    atmosphere = asb.Atmosphere(altitude=altitude_m)
    power_required_W = xv15_rotor(figure_of_merit).evaluate(
        0.0, atmosphere, thrust_N=thrust_per_rotor_N(mass_kg, data)).shaft_power_W
    opti.subject_to(power_required_W / xv15_engine(lapse_exponent, data).power_available_W(atmosphere) == 1)
    return float(opti.solve(verbose=False).value(mass_kg))


def hover_ceiling_m(mass_kg, figure_of_merit, lapse_exponent, data=Xv15PowerData()):
    """OGE hover ceiling at a given mass: the altitude is the Opti variable."""
    opti = asb.Opti()
    altitude_m = opti.variable(init_guess=2000.0, scale=1000.0, lower_bound=-500.0, upper_bound=12000.0)
    atmosphere = asb.Atmosphere(altitude=altitude_m)
    power_required_W = xv15_rotor(figure_of_merit).evaluate(
        0.0, atmosphere, thrust_N=thrust_per_rotor_N(mass_kg, data)).shaft_power_W
    opti.subject_to(power_required_W / xv15_engine(lapse_exponent, data).power_available_W(atmosphere) == 1)
    return float(opti.solve(verbose=False).value(altitude_m))


def part_power_sfc_ratios(data=Xv15PowerData(), part_power_model=GeissPartPowerModel()):
    """(throttle, published sfc / sfc at max, model) with contingency as the knockdown's 100 % point."""
    engine = SimpleTurboshaft(power_rated_W=data.power_ratings_engine_W[0], part_power_model=part_power_model)
    rows = []
    for power_W, sfc in zip(data.power_ratings_engine_W, data.sfc_ratings_lb_hp_h):
        # sfc is inverse to thermal efficiency at fixed fuel heating value.
        model = engine.thermal_efficiency / engine.thermal_efficiency_at(power_W)
        rows.append((power_W / data.power_ratings_engine_W[0], sfc / data.sfc_ratings_lb_hp_h[0], float(model)))
    return tuple(rows)


def thermal_efficiency_from_sfc(sfc_lb_hp_h, fuel_lower_heating_value_J_kg=43e6):
    return 1 / (sfc_lb_hp_h * u.lbm / (u.hp * 3600) * fuel_lower_heating_value_J_kg)


def tier9_hover_power_ratio(figure_of_merit, data=Xv15PowerData()):
    """Hover power of the calibrated model over Tier 9's (FM 0.8, no download), same aircraft."""
    return (0.8 / figure_of_merit) / (1 - data.download_fraction_hover)**1.5


if __name__ == "__main__":
    data = Xv15PowerData()
    n = fit_lapse_exponent()
    figure_of_merit = calibrate_figure_of_merit(n)
    print(f"lapse exponent n = {n:.3f}; calibrated figure of merit = {figure_of_merit:.3f}")
    for h, p in zip(data.altitude_power_m, data.power_available_rotor_W):
        model = xv15_engine(n).power_available_W(asb.Atmosphere(altitude=h))
        print(f"  {h / u.foot:6.0f} ft: digitized {p / u.hp:5.0f} shp, model {model / u.hp:5.0f} shp")
    for h, m in zip(data.altitude_hover_m, data.mass_hover_kg):
        print(f"  hover {h / u.foot:6.0f} ft: figure {m / u.lbm:6.0f} lb, model "
              f"{hover_mass_kg(h, figure_of_merit, n) / u.lbm:6.0f} lb")
    ceiling_m = hover_ceiling_m(Xv15Reference().mass_design_kg, figure_of_merit, n)
    print(f"hover ceiling at 13,000 lb: {ceiling_m / u.foot:,.0f} ft (SP-4517: 8,650 ft)")
    for throttle, published, model in part_power_sfc_ratios():
        print(f"  throttle {throttle:.3f}: sfc ratio published {published:.3f}, model {model:.3f}")
    print(f"Tier 9 hover power was low by a factor {tier9_hover_power_ratio(figure_of_merit):.2f}")
