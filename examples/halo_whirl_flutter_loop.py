"""Plan 055: the Halo sized with whirl flutter in the loop, against the per-rev frequency proxy.

Each case is one coupled solve from the sized reference (`--sized <pickle>`, a HaloSizingResult; default
`output/payload_range/sized.pkl` of plan 045), re-seeded when the eigenvalue scan finds an untracked mode unstable
(`rounds`). The scan's calibrated flutter speed per airplane-mode rotor speed is printed against 1.15 V_D.

    python -m examples.halo_whirl_flutter_loop                    # every case
    python -m examples.halo_whirl_flutter_loop loop delta3        # these cases
"""
import sys
import time
from dataclasses import replace
from pathlib import Path

import aerosandbox.tools.units as u

from examples.halo_payload_range import load_sized, path_sized
from examples.halo_sizing import HaloAssumptions, HaloRequirements, solve_halo_sizing_once, whirl_flutter_cleared

loop = dict(whirl_flutter_model="eigenpair")
wing = dict(free_aspect_ratio_wing=True, wing_section_model="family", free_thickness_to_chord_wing=True)
cases = dict(proxy=dict(), loop=loop, delta3=dict(loop, free_delta3=True), wing=dict(loop, **wing),
             all=dict(loop, free_delta3=True, **wing), proxy_wing=wing)


def run(name, sized, requirements=HaloRequirements()):
    assumptions = replace(HaloAssumptions(), **dict(dict(free_aspect_ratio_wing=False), **cases[name]))   # plan 055 study: AR fixed unless freed
    start = time.time()
    result = solve_halo_sizing_once(requirements, assumptions, initial=sized)
    d = result.design
    print(f"{name}: {time.time() - start:.0f} s, rounds {result.whirl_rounds}, take-off "
          f"{result.mass_takeoff_kg / u.lbm:,.0f} lb, wing {sum(m for _, m in result.wing_masses_kg):.0f} kg, AR "
          f"{d.aspect_ratio_wing or assumptions.aspect_ratio_wing:.2f}, t/c "
          f"{d.thickness_to_chord_wing or assumptions.thickness_to_chord_wing:.3f}, delta-3 {result.delta3_deg:.1f} deg, "
          f"wing design rotor speed {d.speed_rotor_wing_design_rad_s:.1f} rad/s, cruise L/D "
          f"{result.lift_to_drag_cruise:.2f}, cleared {whirl_flutter_cleared(result, requirements, assumptions)}",
          flush=True)
    for row in result.whirl_flutter_check:
        print(f"  {row['speed_rotor_rad_s']:6.2f} rad/s: {row['velocity_flutter_kt']:5.0f} kt ({row['label']})")
    return result


if __name__ == "__main__":
    args = sys.argv[1:]
    path = Path(path_sized)
    if "--sized" in args:
        i = args.index("--sized")
        path = Path(args[i + 1])
        del args[i:i + 2]
    sized = load_sized(path)
    for name in args or cases:
        run(name, sized)
