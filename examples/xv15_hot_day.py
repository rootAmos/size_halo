"""XV-15 hot day (T = 95 F): turboshaft temperature lapse fit and OGE hover-weight prediction (Tier 16).

Digitized from NASA TM X-62407 (1975), https://ntrs.nasa.gov/citations/19750016648, by pixel analysis of the
scanned figures rendered at 200 dpi (2026-10-03). Axes were calibrated on the gridlines found from row and
column pixel sums, with the zero-altitude row checked against the axis line; each curve was traced row by row
and read at an altitude by a local straight-line fit:
  fig. 6.2.2  rotor shaft power available per engine, take-off rating, helicopter mode, 740 ft/s, T = 95 F
              (dashed). The same procedure re-reads the standard-day curve (Tier 10b) within 7 shp, so the
              accuracy is about +/- 10 shp (+/- 15 shp at 12,000 ft, where the dashed line meets its label).
              The curve ends near 13,000 ft.
  fig. 5.1.2  OGE hover gross weight, twin engine, T = 95 F, UT/W = 1.0: the left twin-engine line, the
              counterpart of the fig. 5.1.1 line digitized in Tier 10b as take-off power. The same procedure
              re-reads that fig. 5.1.1 line 15 to 91 lb above Tier 10b, so the accuracy is about +/- 100 lb.
"T = 95 F" is taken as 95 F ambient at every pressure altitude, so the offset from ISA grows with altitude.

The temperature exponent m of `DensityTemperatureLapse` is fitted to the ratio of the 95 F to the standard-day
curve at the same pressure altitude, P_95F / P_std = (T / T_ISA)^-(n + m), with n the Tier 10b density exponent.
The hover weights of fig. 5.1.2 are then a prediction: Tier 10b figure of merit (calibrated on the standard day
at sea level), the hot-day density, and the hot-day power available.
"""
from dataclasses import dataclass

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.powertrain.components.turboshaft import DensityTemperatureLapse, SimpleTurboshaft
from examples.xv15_performance import Xv15PowerData, fit_lapse_exponent, thrust_per_rotor_N, xv15_rotor


def temperature_from_fahrenheit_K(temperature_F):
    return (temperature_F - 32) * 5 / 9 + 273.15


def temperature_offset_K(altitude_m, temperature_K):
    """Offset from the AeroSandbox standard atmosphere at a pressure altitude that gives this ambient temperature."""
    return temperature_K - asb.Atmosphere(altitude=altitude_m).temperature()


@dataclass(frozen=True)
class Xv15HotDayData:
    temperature_K: float = temperature_from_fahrenheit_K(95.0)
    altitude_power_m: tuple = tuple(h * u.foot for h in (0, 2000, 4000, 6000, 8000, 10000, 12000))
    power_available_rotor_W: tuple = tuple(p * u.hp for p in (1103, 1021, 941, 864, 791, 720, 647))
    altitude_hover_m: tuple = tuple(h * u.foot for h in (229, 2000, 4000, 6000, 8000, 10000, 12000,
                                                         14000, 16000, 18000))
    mass_hover_kg: tuple = tuple(w * u.lbm for w in (13339, 12445, 11442, 10458, 9546, 8708, 7902,
                                                     7046, 6248, 5572))
    # Above the end of the 95 F power curve (about 13,000 ft) the hover prediction extrapolates the lapse fit.
    altitude_power_data_max_m: float = 12000 * u.foot


def hot_atmosphere(altitude_m, data=Xv15HotDayData()):
    return asb.Atmosphere(altitude=altitude_m, temperature_deviation=temperature_offset_K(altitude_m, data.temperature_K))


def fit_temperature_lapse_exponent(lapse_exponent=None, data=Xv15HotDayData(), standard=Xv15PowerData()):
    """m from ln(P_95F / P_std) = -(n + m) ln(T / T_ISA), least squares through the origin over the pressure
    altitudes digitized on both curves (0, 4,000, 8,000, 12,000 ft). Post-processing of fixed data."""
    lapse_exponent = fit_lapse_exponent(standard) if lapse_exponent is None else lapse_exponent
    hot = dict(zip(data.altitude_power_m, data.power_available_rotor_W))
    x, y = [], []
    for h, p_std in zip(standard.altitude_power_m, standard.power_available_rotor_W):
        if h in hot:
            atmosphere = hot_atmosphere(h, data)
            x.append(np.log(atmosphere.temperature() / (atmosphere.temperature() - atmosphere.temperature_deviation)))
            y.append(np.log(hot[h] / p_std))
    x, y = np.array(x), np.array(y)
    return float(-np.sum(x * y) / np.sum(x * x)) - lapse_exponent


def xv15_lapse_model(lapse_exponent=None, lapse_exponent_temperature=None):
    n = fit_lapse_exponent() if lapse_exponent is None else lapse_exponent
    m = fit_temperature_lapse_exponent(n) if lapse_exponent_temperature is None else lapse_exponent_temperature
    return DensityTemperatureLapse(lapse_exponent=n, lapse_exponent_temperature=m,
                                   source="XV-15 TM X-62407 fig. 6.2.2, take-off, standard day and 95 F")


def xv15_engine_hot(lapse_model, standard=Xv15PowerData()):
    """One XV-15 engine on a rotor-shaft basis with the density-temperature lapse."""
    return SimpleTurboshaft(power_rated_W=standard.power_available_rotor_W[0], lapse_model=lapse_model)


def hover_mass_kg(atmosphere, figure_of_merit, lapse_model, standard=Xv15PowerData()):
    """Largest OGE hover mass (UT/W = 1, 7 % download): rotor power = power available, one Opti equality."""
    opti = asb.Opti()
    mass_kg = opti.variable(init_guess=6000.0, scale=1000.0, lower_bound=100.0)
    power_required_W = xv15_rotor(figure_of_merit).evaluate(
        0.0, atmosphere, thrust_N=thrust_per_rotor_N(mass_kg, standard)).shaft_power_W
    opti.subject_to(power_required_W / xv15_engine_hot(lapse_model, standard).power_available_W(atmosphere) == 1)
    return float(opti.solve(verbose=False).value(mass_kg))


if __name__ == "__main__":
    from examples.xv15_performance import calibrate_figure_of_merit

    data = Xv15HotDayData()
    n = fit_lapse_exponent()
    model = xv15_lapse_model(n)
    figure_of_merit = calibrate_figure_of_merit(n)
    print(f"n = {n:.3f}, m = {model.lapse_exponent_temperature:.3f}, figure of merit {figure_of_merit:.3f}")
    for h, p in zip(data.altitude_power_m, data.power_available_rotor_W):
        atmosphere = hot_atmosphere(h)
        print(f"  {h / u.foot:6.0f} ft ISA{float(atmosphere.temperature_deviation):+5.1f} K: digitized "
              f"{p / u.hp:5.0f} shp, model {float(xv15_engine_hot(model).power_available_W(atmosphere)) / u.hp:5.0f}")
    for h, m in zip(data.altitude_hover_m, data.mass_hover_kg):
        predicted_kg = hover_mass_kg(hot_atmosphere(h), figure_of_merit, model)
        print(f"  hover {h / u.foot:6.0f} ft: figure {m / u.lbm:6.0f} lb, predicted {predicted_kg / u.lbm:6.0f} lb "
              f"({predicted_kg / m - 1:+.1%})")
