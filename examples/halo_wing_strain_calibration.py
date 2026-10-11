"""Calibrate the reduced-order spar-cap strain model on a CalculiX wing-box result (plan 043).

Reads a structure reference (`examples/halo_structure_reference.py` format) and the `summary.json` the CalculiX
check (`examples/halo_wing_fe.py`) wrote for it, and prints the beam-theory cap strain of
`afdd.strain_cap_jump_ultimate` at the station, the FE peak, and their ratio: the peak factor
(`HaloAssumptions.factor_peak_strain_cap_wing`). The FE ran on the raw AFDD gauges, so the load-carrying factor is
1 here (or `--load-carrying-share` if the FE mapped calibration material in). Run from the repo root:
`python -m examples.halo_wing_strain_calibration --reference <reference.json> --summary <summary.json>`.
"""
import argparse
import json
from pathlib import Path

from aircraft_closure.weights import afdd

distance_station_m = 0.3        # inner edge of the FE strain band (0.3-1.0 m outboard of the clamp)


def beam_strain_from_reference(reference, factor_load_carrying=1.0, distance_m=distance_station_m):
    r, w, material = reference, reference["wing_afdd"], reference["material_wing"]
    chord_m = r["area_wing_m2"] / r["span_wing_m"]
    moment_Nm = afdd.moment_jump_station_ultimate_Nm(r["mass_takeoff_kg"], r["mass_tip_kg"], r["span_wing_m"],
                                                     r["width_fuselage_m"], distance_m,
                                                     load_factor_jump=r["load_factor_jump"])
    strain_beam, _ = afdd.strain_cap_jump_ultimate(
        chord_m, r["thickness_to_chord_wing"], r["fraction_chord_torque_box"], r["ratio_depth_spar_cap"],
        w["area_torque_box_m2"], w["area_spar_m2"], material["modulus_torque_box_Pa"], material["modulus_spar_Pa"],
        moment_Nm, factor_load_carrying=factor_load_carrying)
    return strain_beam, moment_Nm


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", default="output/structure/reference.json")
    parser.add_argument("--summary", default="output/fe_wing/summary.json")
    parser.add_argument("--load-carrying-share", type=float, default=0.0)
    args = parser.parse_args()
    reference = json.loads(Path(args.reference).read_text())
    summary = json.loads(Path(args.summary).read_text())
    factor = 1 + args.load_carrying_share * (reference["factor_wing"] - 1)
    strain_beam, moment_Nm = beam_strain_from_reference(reference, factor)
    print(f"Station moment {moment_Nm / 1e3:.1f} kN m; beam strain {strain_beam:.5f}; FE peak "
          f"{summary['strain_max_jump']:.5f}; allowable {summary['strain_allowable']:.4f}; "
          f"peak factor {summary['strain_max_jump'] / strain_beam:.4f}")
