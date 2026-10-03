"""Hover download from wing and rotor geometry (Tier 21, plan 025).

NDARC form (Johnson, "NDARC Validation and Demonstration", AHS 2010): the wake reaches its fully developed
velocity 2 v_h quickly below the disk, so the wake dynamic pressure is 1/2 rho (2 v_h)^2 = T / A, and

    DL / T = K_int C_D,v S_immersed / A_disk,

with the vertical drag coefficient C_D,v set to reproduce a known download and the flap effect carried by the
projected wing chord, c (1 - c_f (1 - cos delta_f)). Immersed area for tip-mounted rotors: the strip of wing
under the inboard half of the contracted wake (radius R / sqrt 2), chord_projected x R / sqrt 2 per rotor.

Calibration: XV-15 (wing 168.88 ft2, span 32.17 ft, rotor radius 12.5 ft; NDARC Table 1) with 7 % download with
flaps (NASA TM X-62407 sec. 5.1) at the default flap setting gives C_D,v = 0.846.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class HoverDownload:
    drag_coefficient_vertical: Any = 0.846     # calibrated on the XV-15 (module docstring)
    chord_fraction_flap: Any = 0.25            # flap plus flaperon chord / wing chord (assumed)
    deflection_flap_hover_deg: Any = 60.0      # hover flap setting (assumed)
    interference: Any = 1.0                    # NDARC default K_int


def chord_projected_m(chord_m, chord_fraction_flap, deflection_flap_deg):
    return chord_m * (1 - chord_fraction_flap * (1 - np.cosd(deflection_flap_deg)))


def download_fraction(chord_wing_m, radius_rotor_m, count_rotors, download=HoverDownload()):
    """DL / T for `count_rotors` tip-mounted rotors over a wing of (mean) chord `chord_wing_m`."""
    radius_wake_m = radius_rotor_m / np.sqrt(2)
    area_immersed_m2 = count_rotors * radius_wake_m * chord_projected_m(chord_wing_m, download.chord_fraction_flap,
                                                                        download.deflection_flap_hover_deg)
    area_disk_m2 = count_rotors * np.pi * radius_rotor_m**2
    return download.interference * download.drag_coefficient_vertical * area_immersed_m2 / area_disk_m2


def hover_download_fraction(aircraft, download=HoverDownload()):
    """Geometry from the aircraft: mean wing chord and the propulsor instance's radius and count."""
    propulsor = aircraft.powertrain.topology.instances["propulsor"]
    chord_wing_m = aircraft.wing.area_m2 / aircraft.wing.span_m()
    return download_fraction(chord_wing_m, np.sqrt(propulsor.component.area_disk_m2 / np.pi), propulsor.count,
                             download)
