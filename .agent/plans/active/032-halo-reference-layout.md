# Halo reference from the drawn layout (plan 032)

Status: APPROVED in parts, 2026-10-04.

- **Items 2 and 3** were approved as plan 032 when plan 031 was approved: "yes yes yes" to "turbogenerators into
  the fuselage and the boxy fuselage section become the Halo reference".
- **Item 1** was decided after the plan 031 back-check: the fuselage calibration is the layout-anchored factor
  of about 1.7 ("Layout-anchored ~1.7").

Branch: `feat/plan-032-halo-reference`, in the worktree `../halo-plan032`. Another session is working on `main`.

## Goal

Bring three findings from plan 031 (the OpenVSP layout and the weight back-check) into the sizing reference,
one item at a time. Each item is solved and reported before the next.

1. **Fuselage mass factor.** Raw Raymer GA (unpressurized, ΔP = 0) times 1.70, replacing the XV-15 group
   calibration of 2.07.
   - Basis: the plan 031 layout estimate of the uncrewed cargo fuselage is 530 kg at the plan 030 reference:
     - primary structure 310 kg (skin, stringers, ring frames, bulkheads, floor);
     - secondary 220 kg (doors, cut-outs, fuselage-side wing fittings, fairing, access panels, floor fittings,
       fasteners and paint).
   - That is 1.73x raw Raymer on the sized round tube (307 kg). Raw Raymer on the drawn boxy fuselage is within
     2 % of the tube's (same wetted area), so the factor holds for either shape.
   - The XV-15 factor 2.07 carries a crewed fuselage group: cockpit, canopy and crew doors.
2. **Turbogenerators in the fuselage** (user, 2026-10-04: "turbines are likely inside the fuselage").
   - `turbogenerators_on_wing_tips = False` becomes the default.
   - The wing-tip mass is then the rotor, motor, gearbox and nacelle only. This changes the AFDD wing (torsion,
     bending, jump take-off) and whirl flutter.
   - The turbogenerator CG moves into the fuselage at an assumed station (behind the wing box under the dorsal
     fairing).
3. **Boxy fuselage section** (user: "archer halo is not pressurized. that's why it's so boxy").
   - `Fuselage` gains optional `width_m`, `height_m` and `shape` (super-ellipse exponent), passed to
     `asb.FuselageXSec`. With `None`, the section is the current circle (simplest model kept).
   - Halo uses width 5.5 ft (the XV-15 diameter), depth 2.0 m and exponent 3.2, from the plan 031 drawing.
   - Length stays 42.1 ft. The drawing's 11 m is not adopted, because the tail arm and tail sizing would move
     with it; that is a separate decision.

## Interfaces

- `HaloAssumptions`:
  - `mass_factor_fuselage` (None: the XV-15 calibration, for legacy named sets);
  - `turbogenerators_on_wing_tips` default False;
  - `width_fuselage_m`, `height_fuselage_m`, `shape_fuselage`.
- `vehicle.fuselage.Fuselage`: optional `width_m`, `height_m`, `shape`.
- Legacy named assumption sets (plan 026/027/030 and earlier tiers) pin the old values so their notebooks keep
  reproducing.

## Tests

- **Fuselage section:**
  - a circle when width and height are `None`;
  - width = height = d with shape 2 gives the circle's area and volume;
  - a boxier shape has more volume;
  - symbolic width works.
- **References:** each item's new reference closes and is recorded. Integration tests that pin the plan 030
  reference use the pinned legacy set.
- **Notebooks:** Tier 10, 20, 21 and 30-thermal notebooks re-executed where the default changes them.

## Progress

- 2026-10-04: plan written; worktree created from `main` at 8de0835.
- 2026-10-04: **item 1** (fuselage factor 1.70): 6,036.0 kg (13,307 lb), down from the plan 030 reference of
  14,037 lb.
  - Fuselage 636 -> 522 kg.
  - Knock-on: powertrain -133, fuel -29, systems -24, wing -19, gear -10 kg; growth factor about 2.9 on the
    fuselage saving.
  - The generic start chain ended in IPOPT restoration failure. `solve_halo_sizing` now falls back to the same
    problem with the XV-15 fuselage calibration as the start (`start_from_fuselage_calibration`).
  - `tests/integration/test_halo_plan032.py` passed with the default solve (36 min on the loaded machine).
- 2026-10-04: **item 2** (turbogenerators in the fuselage): 5,973.7 kg (13,170 lb), -137 lb.
  - Wing 362.5 -> 328.0 kg:
    - torque box 86.5 -> 51.5 kg, since there is no turbogenerator pitch inertia at the tip;
    - jump spar caps 23.5 -> 36.9 kg, since there is less tip-mass relief.
  - Whirl-flutter torsion (max speed) and static margin are binding.
  - The pylon radius of gyration keeps the XV-15 ratio (0.222 R), although the pylon no longer carries the
    engine. Open.
  - Turbogenerators sit 1.0 m behind the wing quarter chord, at z = 0.5 m (assumed).
- 2026-10-04: item 3 (boxy section, 2.0 m deep, exponent 3.2) implemented and being solved.
- 2026-10-04: **item 3** (boxy section).
  - At the XV-15 length of 12.8 m: 13,952 lb (+782 lb), because the wetted area is 59.9 vs 50.3 m2.
  - The user chose "11 m, as drawn", with the tail root leading edge at 9.8 m.
  - **Final plan 032 reference: 5,815.5 kg (12,821 lb).**
    - L/D 9.60, span 11.16 m, R 4.44 m.
    - Tails 4.66 / 2.49 m2 (item 2: 4.02 / 1.79) for the 1.6 m shorter arm.
    - Fuselage 536.8 kg, wetted area 51.3 m2.
  - **To check:** systems mass fell 451 -> 347 kg. The Raymer systems correlations depend on fuselage length, so
    verify that this is physical.
  - **Not yet done:**
    - the full integration and notebook re-run on the final defaults;
    - IMPLEMENTATION_NOTES, MODEL_INTERFACES, roadmap and README updates;
    - merging to `main`.
- 2026-10-04: wrap-up, at the user's request ("rein in the new features; focus on robustness and documenting what
  the tool does; update the readme").
  - **Systems drop checked:** Raymer flight controls scale with fuselage length^1.536, which gives 0.79x for
    11 / 12.8 m, and with the weight and span changes 0.77x, matching 451 -> 347 kg.
  - **Docs:** IMPLEMENTATION_NOTES, MODEL_INTERFACES, HALO_REFERENCE, a roadmap Tier 23 row and the README
    (current reference, tools table, status).
  - **Bug fix:** the back-check now reads the fuselage factor the reference used (`factor_fuselage` in
    `reference.json`) instead of assuming the XV-15 calibration.
  - **Tests:** 351 fast tests pass.
  - **Still to do:** the integration suite and notebook re-execution on the final defaults, and merging to `main`.
- 2026-10-04 (overnight): **integration suite on the final defaults: all 21 test files pass**, run one file at a
  time. This includes `test_halo_plan032`, which reproduces 12,821 lb with the default solve in 13 min. Notebook
  re-execution was then stopped by Claude Code because the machine ran low on memory (desktop apps plus the other
  session's notebook run). Still pending: re-run `output/overnight.sh` notebooks section (Tiers 10, 12, 12b, 13, 14,
  15, 16, 17, 19, 20, 21) when memory allows, then merge to `main`.
