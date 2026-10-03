# Tiltrotor airframe weights: AFDD tiltrotor wing, whirl-flutter margins, second calibration aircraft

Status: COMPLETED 2026-10-03. Tier 20 (roadmap "20 Tiltrotor airframe weights", review item 7). Implemented on
branch `tier20-wing-weights`. The default is unchanged; the user decides at merge whether the AFDD wing becomes
the default.

## Goal and scope

- **AFDD tiltrotor wing** (NDARC sec. 19-1.1), a distinct mass submodel selectable alongside the Raymer GA wing
  (kept; the simplest model):
  - torque box sized by the torsion frequency;
  - spar caps sized by the chord and beam bending frequencies, then by the jump take-off moment;
  - fairings, control surfaces, fittings and fold/tilt.
- **Reduced-order whirl flutter:**
  - frequencies are in per rev of a wing design rotor speed (NDARC's parameterisation);
  - in the Halo sizing that speed is a design variable;
  - every airplane-mode point carries torsion and beam frequency margins at its own rotor speed. These are
    constraints, not a hidden loop.
- **Second calibration aircraft:**
  - public V-22 and AW609 group weight statements were not found (see Decisions);
  - the V-22 FSD wing (one group) and the Bell D266 MIL-STD-451 statement (wing and rotor groups) are used;
  - each is model against actual, as for the XV-15 in Tier 10a.
- **Uncrewed adjustments:** the Halo equipment is itemised from the XV-15 groups, with explicit removals and
  additions.
- **Switch:**
  - `HaloAssumptions.wing_weight_model`, default "raymer"; "afdd_tiltrotor" selects the new wing;
  - `Xv15MassFactors.wing_tiltrotor` is a separate factor;
  - the Raymer `wing` factor is unchanged.
- **Not in scope:**
  - stress or FEM (structures stay mass estimation);
  - flutter speed prediction, i.e. aeroelastic stability (the frequency placement is its reduced-order
    surrogate);
  - AFDD fuselage, gear and controls models;
  - Tier 15 electrical and Tier 21 aerodynamics.

## References

- W. Johnson, NDARC Theory, NASA/TP-2009-215402, sec. 19-1.1, eqs. and table 19-1 (read from the PDF pages).
  The model follows Chappell and Peyran, SAWE Paper 2107 (1992).
- XV-15:
  - wing breakdown, stiffness and materials: Acree, Peyran and Johnson, AHS 55th Forum (1999), table 5;
  - stick-model modes, tip masses and inertia: NASA/TP-2004-212262, appendices C and D;
  - group statement and control-surface areas: TM X-62407, sec. 3.1.2 and 3.4.
- V-22 FSD:
  - wing about 2,470 lb: Popelka et al., AHS 51st Forum (1995). The 2,500 lb tailored wing is +29 lb (1.2 %).
    The scope of fold/rotate items is not stated;
  - design gross weight 47,500 lb and empty weight 32,340 lb: Harris, NASA/SP-2015-215959 vol. III, p. 312;
  - geometry: NASA/CR-2011-215960, CTR30 table 2-6;
  - airplane-mode 333 rpm: Acree, AHS 2002;
  - engine 6,150 shp: T406.
- Bell D266 (1968 Composite Research Aircraft, not built): Harris vol. III, tables 2-19 to 2-21, from USAAVLABS
  TR 68-32. That TR is on DTIC, but the download was refused (HTTP 403).
- Data: `data/weights/tiltrotor_wing_reference.csv`, with a source key per row; `data/weights/sources.csv`.

## Assumptions

- **Torque-box chord ratio 0.45** (not published).
  - With it, the published XV-15 section data give e_tb = 0.583 and C_t = 0.526 (e_sp = 1).
  - The sensitivity to the ratio is shown in the notebook.
- **XV-15 frequency placement for every aircraft:** the symmetric modes at 458 rpm are torsion 1.087, beam 0.432
  and chord 0.825 per rev.
  - The 8.3 Hz symmetric torsion mode is used. The antisymmetric torsion mode, 7.1 Hz, is lower.
- **Strain allowable:** the "limit strain" of Acree's table 5 (0.0068 aluminium, 0.0047 graphite) is used as
  NDARC's allowable.
- **Jump take-off:** 2 g at the design mass (NDARC and Johnson practice), with thrust from n W / N only.
  - The optional thrust-capability input is offered, but unused.
- **Pylon radius of gyration** 0.222 R, from the XV-15 stick model.
- **Fold fraction 0.**
- **Halo:**
  - graphite epoxy, t/c 0.23;
  - each tip carries the rotor, motor, rotor gearbox, turbogenerator and nacelle section
    (`turbogenerators_on_wing_tips=True`);
  - attachment width = fuselage width;
  - smoothing 0.01 on the max(0, .) steps.
- **V-22 tip mass:** a framework model-chain estimate, each term times its XV-15 factor:
  - AFDD82 rotor at sigma 0.105 and coning 1.55;
  - AeroSandbox turboshaft regression;
  - AFDD82 nacelle section, with the XV-15 wetted area scaled by R^2;
  - a quarter of the AFDD83 drive (engine speed 15,000 rpm assumed).
  - V-22 fuselage width 7.9 ft (CTR30).
- **D266:**
  - t/c 0.23 and fuselage width 6.5 ft (both assumed);
  - tip mass is half the rotor and nacelle groups plus a quarter of the AFDD83 drive (engine 13,600 rpm
    assumed).

## Interfaces

- **`weights/afdd.py`:**
  - `section_form_factors_tiltrotor_wing(tau, w_tb)` returns F_B, F_C, F_T, F_VH;
  - `TiltrotorWingMasses`, a frozen result: masses, realized stiffness and frequencies, jump moment,
    `mass_primary_kg()` and `total()`;
  - `wing_tiltrotor_afdd_masses(...)`, in consistent SI.
- **`vehicle/surfaces.py`:**
  - `Wing.mass_model` (None means Raymer) and `Wing.chord_mean_m()`;
  - `WingMaterial`, `aluminium_wing_material()` and `graphite_epoxy_wing_material()`;
  - `TiltrotorWingMassModel`: `masses`, `mass_kg` and `frequency_per_rev`.
- **`examples/xv15_reference.py`:**
  - new `Xv15Reference` fields: tip mass, pylon gyration, attachment width, airplane rpm and mode frequencies;
  - `Xv15MassFactors.wing_tiltrotor`;
  - `xv15_wing_mass_model()`;
  - `wing_weight_model=` on `build_xv15_aircraft`, `compare_groups` and `solve_xv15_closure`.
- **`examples/tiltrotor_wing_calibration.py`:**
  - `load_reference_data` and `xv15_section_calibration`;
  - `v22_wing_check`, `d266_wing_check` and `d266_rotor_implied_solidity`;
  - `xv15_frequency_stiffness_ratios`.
- **`examples/halo_sizing.py`:**
  - new `HaloAssumptions` fields: `wing_weight_model` ("raymer"), wing frequencies, `thickness_to_chord_wing`,
    `wing_material`, `ratio_radius_gyration_pylon`, `turbogenerators_on_wing_tips`, `load_factor_jump` and
    `smoothing_wing_tiltrotor`;
  - `HaloDesign.speed_rotor_wing_design_rad_s`;
  - `HaloSizingResult.wing_masses_kg` and `whirl_flutter`;
  - `EquipmentItem`, `xv15_equipment_items`, `uncrewed_equipment_adjustments`, `halo_equipment_items` and
    `mass_equipment_from_items_kg`;
  - the named set `assumptions_tier20`.
- No powertrain-component changes.

## Symbolic considerations

- Every input may be an Opti variable.
- The NDARC sequence is a single explicit pass: the jump moment uses the pre-jump wing mass, as NDARC does, so
  there is no recalculation loop.
- **max(0, .):** `np.fmax` when smoothing is numerically 0, otherwise a hyperbola,
  0.5 (x + sqrt(x^2 + s^2)), with s scaled by the required stiffness or moment.
  - The only Python branch is on that numeric parameter (and on an optional thrust input being None).
- **Whirl flutter:** realized frequency / point rotor speed >= required per rev. The point rotor speeds are the
  flight-point variables.

## Tests

- **`tests/weights/test_tiltrotor_wing.py`** (24):
  - section polynomials by hand;
  - GJ, A_tb and box-mass identities;
  - fairing and control-surface areas;
  - fittings fraction;
  - total = sum of the parts; fold;
  - limits: no bending requirement, no jump, no fittings, smoothing -> exact;
  - box mass proportional to omega^2;
  - trends: torsion frequency, span, design mass, tip mass, rotor speed, composite;
  - the Wing submodel and factor;
  - an Opti solve with symbolic area and design rotor speed.
- **`tests/integration/test_tiltrotor_wing_calibration.py`** (10):
  - data completeness;
  - section calibration = model defaults;
  - stiffness ratio band;
  - factors: `wing_tiltrotor` 1.327 and the Raymer factor unchanged;
  - the calibrated statement is reproduced;
  - closures: the AFDD calibrated closure is 13,000 lb, and the uncalibrated one moves toward the statement;
  - V-22 within 25 % (under), D266 within 10 %, D266 implied solidity.
- **`tests/integration/test_halo_wing_afdd.py`** (6):
  - the default is Raymer; the itemised equipment = 587 lb;
  - AFDD Halo: 6,144 +- 40 kg, wing below Raymer, breakdown reported, whirl margins hold and bind at maximum
    speed, max payload 959 +- 10 kg.

## Acceptance

- The full unittest suite passes; the Tier 20 notebook is executed with all checks passing.
- The default reference is unchanged: 14,436 lb.

## Progress and decisions

- 2026-10-03: plan written; NDARC equations read from the PDF (the text layer drops Greek letters).
- **Data search:**
  - V-22 group weight statement: not public (NASA, DTIC and SAWE searches; SAWE 3342 "V-22 Weight History" is
    paywalled);
  - AW609: not public;
  - Harris vol. III has the Bell D266 statement (a full MIL-STD-451 statement, but a 1968 design) and the FSD
    V-22 empty weight;
  - Popelka et al. 1995 give the V-22 wing.
  - Decision: V-22 (wing) plus D266 (wing and rotor), reported as such.
- **XV-15 results:**
  - frequency-driven stiffness is 0.57 (torsion), 0.58 (beam) and 0.62 (chord) of published: the single-mode
    relations ignore wing inertia and root fixity;
  - AFDD wing x1 is 658 lb against the 873 lb statement, so `wing_tiltrotor` = 1.327 (Raymer: 452 lb, factor
    1.930);
  - uncalibrated XV-15 closure: Raymer 11,315 lb; with the AFDD wing it is heavier (see notebook);
  - calibrated: 13,000 lb with both models.
- **Cross-checks (x 1.327, no further fitting):**

  | Aircraft | Model (lb) | Actual (lb) | Error |
  |---|---|---|---|
  | V-22 FSD wing | 2,023 | 2,470 | -18 % |
  | Bell D266 wing | 1,840 | 1,886 | -2 % |

  - The V-22 tip mass is an estimate (6,482 lb per side) and the fold scope is unknown.
  - D266 rotor: AFDD82 x the XV-15 factor 0.69 needs solidity 0.062 to give 2,439 lb. Typical proprotors are
    0.08-0.11, so the calibrated rotor model is heavy for that design. It is a 1968 estimate; no change is made.
  - Drive systems: no public V-22 or D266 split (the D266 propulsion group lumps engines, drive and fuel
    system), so the drive models stay checked on the XV-15 only.
- **Halo reference with the AFDD wing** (780 kg payload, ECM battery):
  - take-off mass 6,144 kg (13,546 lb), against 6,548 kg (14,436 lb) with Raymer;
  - wing 372 kg, against 547.5 kg; wing area 21.3 m2 (Raymer: 22.6 m2);
  - max payload 959 kg, against 785 kg;
  - the whirl-flutter torsion margin binds at the 210 kt maximum-speed point: the wing design rotor speed,
    39.4 rad/s, equals the maximum-speed rotor speed. Beam is not binding (0.63 against 0.43 per rev);
  - other binding constraints are as in the reference, except that turbine power at maximum speed binds only in
    the max-payload case.
- Equipment itemised with the same 587 lb total. The heating group had been relabelled "autonomy"; it is now an
  explicit ECS removal plus an autonomy allocation.
- Environment note: `uv` could not build a fresh worktree environment (TLS certificate error fetching
  setuptools). Runs used the main checkout's `.venv` with `PYTHONPATH` pointing at this worktree.

## Deferred

- Flutter speed (CAMRAD-class) and mode coupling: the frequency placement is the surrogate.
- A V-22 or AW609 full group comparison, if public statements surface.
- AFDD fuselage, gear and controls models for the remaining weak Raymer groups.
- Crew items inside the fuselage and flight-control groups: no public split.
- Wing fold (fraction 0) for shipboard-style stowage.
