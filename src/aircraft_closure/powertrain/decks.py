"""Engine decks: parsing and derived curves (lowest layer: data, no optimization).

GASP_TS-style turboshaft decks list corrected shaft power, tailpipe thrust
and corrected fuel flow on a full Mach x altitude x throttle grid, with
`# key: value` metadata lines before the column header. Throttle is the deck's
turbine-temperature parameter (e.g. 20 = flight idle, 50 = `t4max`), not a
power fraction.

Only the normalised part-power curve is derived here. sfc = fuel flow /
shaft power, and sfc / sfc at maximum throttle is independent of the
correction convention (both columns carry the same correction), so it can be
used even where a deck's altitude scaling is in doubt.
"""
from dataclasses import dataclass, field
from typing import Any

import aerosandbox.numpy as np
import aerosandbox.tools.units as u


@dataclass(frozen=True)
class TurboshaftDeck:
    mach: Any
    altitude_m: Any
    throttle: Any
    power_shaft_corrected_W: Any
    thrust_tailpipe_N: Any
    fuel_flow_corrected_kg_s: Any
    metadata: dict = field(default_factory=dict)

    def row(self, mach, altitude_m):
        """Throttle-sorted (throttle, power, fuel flow) at one grid Mach and altitude."""
        mask = (self.mach == mach) & (self.altitude_m == altitude_m)
        order = np.argsort(self.throttle[mask])
        return (self.throttle[mask][order], self.power_shaft_corrected_W[mask][order],
                self.fuel_flow_corrected_kg_s[mask][order])


def load_gasp_turboshaft_deck(path):
    """Columns: Mach, altitude [ft], throttle, shaft power [hp], tailpipe thrust [lbf], fuel flow [lb/h]."""
    metadata, values = {}, []
    header_seen = False
    for line in open(path, encoding="utf-8"):
        text = line.strip()
        if not text:
            continue
        if text.startswith("#"):
            key, _, value = text[1:].partition(":")
            if value.strip():
                metadata[key.strip()] = value.strip()
            continue
        if not header_seen:
            header_seen = True
            continue
        values.append(np.array(text.split(","), dtype=float))
    data = np.array(values)
    return TurboshaftDeck(
        mach=data[:, 0], altitude_m=data[:, 1] * u.foot, throttle=data[:, 2],
        power_shaft_corrected_W=data[:, 3] * u.hp, thrust_tailpipe_N=data[:, 4] * u.lbf,
        fuel_flow_corrected_kg_s=data[:, 5] * u.lbm / u.hour, metadata=metadata,
    )


@dataclass(frozen=True)
class PartPowerCurve:
    power_fraction: Any
    sfc_ratio_median: Any
    sfc_ratio_min: Any
    sfc_ratio_max: Any


def part_power_curve(deck, power_fractions):
    """sfc / sfc_max against power / power_max, interpolated per (Mach, altitude) row, then summarised."""
    ratios = []
    for mach in np.unique(deck.mach):
        for altitude_m in np.unique(deck.altitude_m):
            _, power_W, fuel_kg_s = deck.row(mach, altitude_m)
            if len(power_W) == 0:
                continue
            sfc = fuel_kg_s / power_W
            ratios.append(np.interp(power_fractions, power_W / power_W[-1], sfc / sfc[-1]))
    ratios = np.array(ratios)
    return PartPowerCurve(np.array(power_fractions), np.median(ratios, axis=0), np.min(ratios, axis=0),
                          np.max(ratios, axis=0))


def fit_cubic_part_power(power_fraction, sfc_ratio):
    """Least-squares (a, b, c) of 1 / sfc_ratio = 1 + a x + b x^2 + c x^3, x = power fraction - 1."""
    x = np.array(power_fraction) - 1
    design = np.stack([x, x**2, x**3], axis=1)
    coefficients = np.linalg.lstsq(design, 1 / np.array(sfc_ratio) - 1, rcond=None)[0]
    return tuple(coefficients)
