# Tiltrotor geometry layout and OpenVSP export (Tier 23, plan 031)

Status: APPROVED 2026-10-04 ("yes yes yes. but calculix evaluation comes later").

## Goal

Give the framework a strong tiltrotor geometry module. It has two layers:

1. **`TiltrotorLayout`** — a symbolic (CasADi) layout.
   - It derives every component position from a small set of drivers.
   - It returns clearance and packaging margins to `asb.Opti`.
2. **OpenVSP export** — numeric, run after the solve.
   - It builds an OpenVSP 3.53.1 model of the solved aircraft through the Python API.
   - It writes CAD (STEP), a mesh of the internal structure (CalculiX `.inp` and Nastran `.dat`), and checks.
   - It never takes part in optimization or convergence.

## User decisions (2026-10-04)

- **Tool:** OpenVSP scripted through its Python API. Version 3.53.1 is at
  `C:\Users\alexa\Documents\Xenon\Software\OpenVSP-3.53.1-win64-Python3.13\OpenVSP-3.53.1-win64`.
  - It imports into the project venv (Python 3.13).
  - It loads the skeleton below.
  - The FEA API and STEP structure export are present.
- **Configuration:** XV-15 class, focused on the Archer Halo concept.
  - Nacelles at the wing tips.
  - The whole nacelle tilts.
  - Turbines are inside the fuselage, not in the nacelles.
- **Fuselage:** start with a tube and add a lofted fuselage later.
  - Halo is unpressurized, which is why it is boxy.
  - So "tube" here means a constant section with a rounded-rectangle (super-ellipse) shape plus nose and tail tapers. A circle stays the special case.
- **Outputs:** CAD and structural analysis.
- **Skeleton:** `C:\Users\alexa\Downloads\TiltRotor_2025-07-15T02_42_20.641Z.vsp3`, a V-22-style model in inches.
  - It is used as the structural pattern for the OpenVSP model tree, not loaded as a template.
  - Reasons: it is in inches, and its V-22 wing-stow hinge, pilot and inch-based attach lengths would all need undoing.
  - Tree to reproduce:

    ```
    Fuselage
    ├─ PayloadBay (Conformal)
    ├─ HTail → VTail
    └─ Wing
       └─ NacelleTilt (Hinge)
          └─ Nacelle
             ├─ Spinner
             └─ Rotor (Propeller)
                └─ RotorClear (Auxiliary)
    PowerRouting (Routing)
    ```
  - `Modes`: hover, conversion and cruise. Hinge `JointRotate` sets the nacelle angle.

## Scope

**In scope:**

- **Milestone A — symbolic layout:**
  - `TiltrotorLayout`, positions as a function of nacelle angle, and the CG at that angle;
  - clearance margins and fuselage packaging margins;
  - the rounded-rectangle fuselage section.
- **Milestone B — OpenVSP outer mold line:**
  - model build, with snapshots at 0/45/90 deg nacelle angle;
  - STEP and STL export;
  - CompGeom wetted-area and volume readback compared against AeroSandbox;
  - an Auxiliary rotor-clearance check.
- **Milestone C — OpenVSP internal structure:**
  - wing spars, ribs and skins; nacelle-attach and fuselage-attach ribs;
  - fuselage frames, bulkheads, floor and skins;
  - export to STEP structure, CalculiX and Nastran meshes. Running CalculiX is deferred (user, 2026-10-04).

**Not in scope:**

- **Lofted fuselage, sponsons and nose turret.** These are the next fuselage plan.
- **V-tail.** The Halo model photo appears to show one. The framework tail stays conventional (deferred since Tier 21). The export mirrors whatever tail the aircraft has.
- **Moving the turbogenerators to the fuselage in the sizing reference** (`turbogenerators_on_wing_tips` is True today).
  - Moving them changes wing tip mass, the AFDD wing weight and whirl flutter, so the 13,639 lb reference changes.
  - Proposed as plan 032 directly after this one.
  - This plan supports both placements geometrically, and the export follows the flag.
- **VSPAERO** and any feedback of OpenVSP results into sizing other than explicit, reported correction factors.
- **Structural sizing of skins and spars by FE.** Structures remain mass estimation (AGENTS.md). FE is a check only.

## Architecture and ownership

- **`vehicle/layout.py`: `TiltrotorLayout`.**
  - A frozen dataclass of drivers (symbolic or numeric).
  - It owns no optimization variables and is built by the Halo builder.
  - Physical components still own their geometry. The layout only computes the shared reference points (spindle, hub, zone boundaries) the components are placed at.
- **`vehicle/fuselage.py`:** optional `width_m`, `height_m` and `shape` (super-ellipse exponent).
  - When they are `None`, the section is the current circle of `diameter_m`.
  - `to_asb()` passes them to `asb.FuselageXSec(width=, height=, shape=)`, which already accepts symbolic values.
  - Raymer mass and AeroBuildup see the new section through `to_asb()`; no formula is duplicated.
- **`export/openvsp/`** (new package; depends on `vehicle`, never the reverse):
  - `snapshot.py`: `GeometrySnapshot`, a numeric frozen dataclass built from a solved aircraft and layout via `sol.value`. It is the only thing the OpenVSP code reads.
  - `model.py`: `build_openvsp_model(snapshot, nacelle_angle_deg)` builds the outer mold line; `export_cad(...)` writes STEP and STL; `comp_geom(...)` reads back.
  - `structure.py`: `build_structure(snapshot, ...)` builds the FEA structures; `export_structure(...)` writes STEP structure, CalculiX and Nastran.
- **OpenVSP dependency:** an optional install, never imported by `vehicle/` or the sizing code.
  - It is installed into `.venv` from the release's `python/requirements.txt` (`uv pip install -r ...`).
  - The import is guarded; tests and notebook cells skip with a clear message when it is missing.
  - The install command goes in the README.

## Layout drivers and derived quantities (Milestone A)

Coordinates follow AeroSandbox: x aft, y right, z up, metres.

**Drivers** (with defaults chosen to reproduce today's hand-set Halo positions exactly):

| Driver | Default |
|---|---|
| `fraction_chord_spindle` | 0.25 (today `x_rotor = x_le + 0.25 c`) |
| `offset_z_spindle_m` | 0.0 |
| `length_mast_m`: spindle to rotor hub along the nacelle axis | 1.0 (today `z_rotor = z_wing + 1.0`) |
| `offset_nacelle_cg_m` | to be confirmed against today's `x_rotor_m` / `z_rotor_m` locations (motor, gearbox at the hub) |
| `angle_flap_max_rad` | 0.21 (12 deg, XV-15 class flapping stop) |
| `clearance_blade_wing_m` | 0.15 |
| `fraction_chord_front_spar` | 0.15 |
| `fraction_chord_rear_spar` | 0.60 |
| `fraction_depth_box` | 0.9 |
| Fuselage zone lengths | nose/avionics, payload bay, fuel, battery, turbogenerator bay (when in the fuselage), tail cone |

**Derived:**

- **Spindle position:**
  - `y_spindle_m = span / 2`;
  - `x_spindle_m = x_le + fraction_chord_spindle * c`;
  - `z_spindle_m = z_wing + offset`.
- **Nacelle angle convention** (XV-15/NDARC): 90 deg is helicopter mode, 0 is airplane mode.
- **Hub position at nacelle angle a:** `(x_spindle - L cos a, y_spindle, z_spindle + L sin a)`.
- **Nacelle group CG at angle a.** `Aircraft.get_mass_breakdown` keeps its signature; the tilting items are placed by the layout at a given angle, and the reference case uses 90 deg (today's z_rotor = z_wing + 1.0).

**Margins** (`margin_above` / `margin_below`, same form as the existing design margins):

1. **Rotor to fuselage** (replaces today's inline expression):

   `R <= y_hub - width_fuselage / 2 - clearance_rotor_fuselage`.

   It is the same expression today, with the width replacing the diameter.
2. **Blade to wing, airplane mode** (flapping toward the leading edge):

   `x_le - x_hub(0) >= R sin(angle_flap_max) + clearance_blade_wing`.
3. **Blade to wing, helicopter mode** (blade over the upper surface):

   `z_hub(90) - z_wing_upper >= R sin(angle_flap_max) + clearance_blade_wing`.
4. **Fuselage packaging:**
   - zone lengths sum to at most the constant-section length;
   - each zone's required volume is at most its available volume × packing efficiency;
   - fuel, from mass and density;
   - battery, from energy and pack volumetric energy density;
   - payload, from mass and an assumed bulk density;
   - turbogenerators, from a power-scaled envelope, only when they are in the fuselage.
5. **Wing-box volume** (reported, not constrained): between the spars, net of the fuselage carry-through.

**Reported only:** airplane-mode rotor-tip ground clearance (the XV-15 cannot land in airplane mode).

Each new margin is added to the Halo report. The new margins become constraints only after the plan 027 reference has been reproduced with them reported. If any is violated at the reference, that is recorded as a finding, not silently fixed.

## OpenVSP model (Milestone B)

- **Built in metres** from `GeometrySnapshot`:
  - **Fuselage:** a Stack with rounded-rectangle sections (VSP "Rounded Rectangle" or super-ellipse XSec).
  - **Wing:** NACA 2423 via a 4-series XSec, at today's thickness and position.
  - **Tails:** conventional, as in the aircraft.
  - **Nacelle:** a Stack under a Hinge at the spindle.
  - **Rotor:** a Propeller with diameter and blade count, precone 2.5 deg as in the skeleton.
  - **Rotor clearance:** an Auxiliary disc.
  - **Power cables:** a Routing path from the fuselage turbogenerator bay to the nacelle motor.
- **Modes:** `hover` (90 deg), `conversion` (45 deg) and `cruise` (0 deg).
- **Exports:**
  - `.vsp3`;
  - STEP (`EXPORT_STEP`);
  - STL per mode, for notebook and dashboard renders with matplotlib — no GUI dependency.
- **CompGeom readback:** wetted area per component and fuselage volume, compared against the AeroSandbox values the sizing used.

## Internal structure (Milestone C)

- **Wing FeaStructure:**
  - front and rear spars at the layout chord fractions;
  - a rib array at a set pitch;
  - attach ribs at ±fuselage half-width, and a tip rib at the spindle;
  - upper and lower skins.
- **Fuselage FeaStructure:**
  - a frame array;
  - bulkheads at the zone boundaries, wing spar attach, gear and turbogenerator mounts;
  - a floor at the payload-bay floor height;
  - skins.
- **Materials and thicknesses:**
  - Wing: graphite-epoxy and aluminium properties from `WingMaterial`.
  - Smeared skin and spar thicknesses come from the AFDD wing mass breakdown (`TiltrotorWingMassModel.masses`), as equivalent thicknesses: mass / (density × area).
  - The FE model therefore represents the same structure the sizing weighed.
  - Fuselage gauges are an assumed placeholder, recorded as such.
- **Point masses:** the nacelle group mass and pylon inertia are a FixedPoint mass at the spindle.
- **Exports:**
  - STEP structure;
  - CalculiX `.inp` and Nastran `.dat` (OpenVSP FEA Mesh).
## Symbolic considerations

- Every layout expression uses `aerosandbox.numpy`, with no branching on symbolic values.
- The placement of the turbogenerators is a Python flag, as today.
- No OpenVSP call happens before `sol.value`. `GeometrySnapshot` refuses CasADi inputs.

## Tests

**Analytic (`tests/vehicle/test_layout.py`):**

- **Hub kinematics:** hub at 0 and 90 deg; hub-to-spindle distance equals the mast length at every angle.
- **Default layout:** reproduces today's Halo positions exactly.
- **Rotor-fuselage margin:** matches the current inline expression.
- **Flapping trend:** larger flapping or rotor radius reduces both blade-wing margins.
- **CG with nacelle angle:** the CG moves forward and down as the nacelle goes from 90 to 0 deg, and the shift equals the nacelle mass × the arm.
- **Zone sum and volume identities.**
- **Symbolic compatibility:** each item builds and solves in `asb.Opti`.

**Fuselage:**

- A circle when width and height are `None`.
- With width = height = d and shape = 2, the AeroSandbox area and volume equal the circle.
- A rounded rectangle has more volume than the inscribed circle.
- Symbolic width and height work.

**OpenVSP (skipped without `openvsp`):**

- The model builds; the geometry count and parent tree match the specified tree.
- **Hinge check:** the hub position from the VSP bounding box or Hinge at 0/90 deg matches the layout to 1 mm.
- **CompGeom agreement with AeroSandbox:**
  - wing wetted area within 3 %;
  - fuselage wetted area and volume within 5 %.
- The STEP, STL, CalculiX and Nastran files are written and non-empty.
- The FEA mesh mass is within 10 % of the framework wing structural mass (box plus spar).

## Reference case

- **Plan 027 defaults.** Halo closes at 13,639 lb unchanged, within 1 lb, with the layout in place and the new margins reported.
- **Geometry export** of that solution at 0/45/90 deg.

## Notebook and dashboard

- **Notebook** in `notebooks/tier23_geometry/`:
  - layout checks and a margin table;
  - nacelle-conversion CG track;
  - renders at the three angles;
  - CompGeom comparison;
  - structure views;
- **Dashboard:** republished with geometry renders and the new margins.

## Implementation sequence

1. Install OpenVSP into `.venv` and record the command in the README. Add the guarded import helper.
2. Fuselage section fields, plus tests.
3. `TiltrotorLayout`, plus tests. Wire it into `examples/halo_sizing.py` in place of the hand-set `x_rotor_m` / `z_rotor_m`. Confirm the reference closes unchanged.
4. Add the new margins to the Halo report (reported only). Record any violations.
5. `GeometrySnapshot` and the outer-mold-line build, Modes, exports and CompGeom, plus tests.
6. FEA structures and exports, plus tests.
7. Notebook, `IMPLEMENTATION_NOTES.md`, `MODEL_INTERFACES.md`, a roadmap row for Tier 23, then the dashboard.

## Acceptance

- The full unittest suite passes; OpenVSP tests pass locally.
- Tier 10–21 notebooks pass, and the plan 027 reference is unchanged.
- The Tier 23 notebook is executed.
- STEP (outer mold line and structure), CalculiX and Nastran files are produced for the Halo reference.

## Decisions (user, 2026-10-04)

1. **CalculiX evaluation comes later.** This plan stops at mesh export (`.inp`, `.dat`).
2. **Blade–wing margins are active constraints** in the reference, next to rotor–fuselage.
   Packaging margins stay reported until the lofted fuselage exists. If a blade–wing constraint
   binds at the reference, the weight change is recorded rather than hidden.
3. **Plan 032:** turbogenerators into the fuselage and the boxy section become the Halo reference.

## Deferred work

- **CalculiX:** a wing-only modal check against the AFDD per-rev targets (0.43, 0.83, 1.09)
  and a static check with the jump-take-off tip load against beam theory.
- Lofted fuselage, sponsons and nose turret; V-tail; VSPAERO.

## Progress

- 2026-10-04: draft written and approved. Checked:
  - OpenVSP 3.53.1 imports in the venv and loads the skeleton (15 geoms, all types recognised);
  - the FEA and STEP-structure API is present;
  - no CalculiX or Nastran solver is installed.
- 2026-10-04: user asked to start with the OpenVSP script before the AeroSandbox wiring, so Milestone B goes first,
  driven by `halo_plan027_snapshot()` (solved plan 027 numbers, boxy fuselage assumed: 1.68 m wide, 2.0 m deep,
  0.35 m corners).
  - OpenVSP installed into `.venv` with `uv pip install --system-certs` from the release's `python/` packages
    (openvsp_config, utilities, degen_geom, vsp_airfoils, openvsp).
  - `export/openvsp/{snapshot,model,render}.py` and `examples/halo_openvsp.py` write `.vsp3` (with hover,
    conversion and cruise Modes), STEP and STL at 90/45/0 deg, and a render, in about 9 s.
  - OpenVSP 3.53.1 quirk: after the first export, changing only the hinge angle (directly or through a Mode) does
    not re-tessellate the hinge children, so each angle is exported from a fresh build.
  - Hinge children are not mirrored with the wing; nacelle, rotor and tip-path auxiliary each carry XZ symmetry
    about the global origin (`Sym_Ancestor` 0).
  - CompGeom, airplane mode:
    - wing wetted area 44.7 m2 against 45.3 m2 in AeroSandbox (-1.5 %);
    - boxy fuselage wetted area 75.0 m2 and volume 32.4 m3, against 50.3 m2 and 18.3 m3 for the XV-15 circular
      tube the sizing uses — an input for plan 032;
    - nacelle wetted area 7.0 m2 each, against the 8.8 m2 cowling assumption.
- 2026-10-04: blending pass after user feedback ("a bit stubby ... work on the blending"), following the Joby S4
  and Kitty Hawk Heaviside models the user supplied:
  - **Bodies:** fuselage, a new dorsal wing fairing and the nacelles are OpenVSP Fuselage geoms with round end
    caps. Each section's top, bottom and side tangent angles come from the neighbouring stations' slopes; the
    default horizontal tangents at every section were what made the bodies look segmented.
  - **V-tail:** a 65 deg root strake (Kitty Hawk-style root panel) blends it into the boom.
  - **Shape:** the fuselage profile is a 9-station table read from the Halo stills, and the nacelle is
    3.4 m long and flat-sided.
- 2026-10-04: second shape pass (user: "the empennage is a bit clunky ... taper the wing ... aesthetic
  modifications are okay ... fix the rendering"):
  - **Sections:** body sections are super-ellipses with a separate bottom exponent (flat bottom, no corner
    crease).
  - **Aft body and tail:** the aft fuselage has two stations instead of four, and the V-tail strake is removed.
  - **Wing:** tapered 0.6 at the sized area and span about an unswept quarter chord. This is aesthetic only; the
    sizing keeps a constant chord.
  - **Fairing:** the dorsal fairing is near cabin width, with its ends sunk into the cabin top.
  - **Renders:** PyVista off-screen (new optional `geometry` dependency group). There are separate airframe and
    rotor STLs per angle, from the new OpenVSP sets "Airframe" and "Rotors".
  - **Install note:** `uv sync` removes the OpenVSP packages, which are installed outside the lock file; reinstall
    them with `uv pip install`, or sync with `--inexact`.
- 2026-10-04: OpenVSP outer mold line committed on branch `feat/openvsp-geometry` (3704dbf).
  - The plan is renumbered 028 -> 031, because 028/030 are the parallel thermal work; the follow-up becomes 032.
  - The commit was built with a temporary index, so the shared `main` working tree, where another session has
    uncommitted plan 030 changes, was not touched.
- 2026-10-04: aero cross-check, added at the user's request ("run some kind of aero analysis ... with vsp panel
  methods and compare it to what aero sandbox is doing"). It is a check only; nothing feeds the sizing.
  - **Code:** `GeometrySnapshot.to_asb()` gives both tools the same geometry; `export/openvsp/aero.py` runs VSPAERO
    7.2.2 and the OpenVSP parasite tool; `examples/halo_aero_compare.py` does the comparison.
  - **Condition:** 210 kt, 10,000 ft (Mach 0.329, Re(MAC) 1.1e7), moments about the wing MAC quarter chord
    (x 4.546 m), airplane mode, no rotors.
  - **VSPAERO models:** an all-vortex-lattice model is singular, because a VLM body is a flat plate on its centre
    plane and coincides with the wing tip inside the tip nacelle. So the "mixed" model panels the bodies and keeps
    the lifting surfaces thin. With thick bodies, VSPAERO's wake induced drag is not credible, so induced drag is
    compared on the thin model only.
  - **Results:**

    | | VSPAERO thin | VSPAERO mixed | VSPAERO panel | AeroSandbox VLM | AeroBuildup |
    |---|---|---|---|---|---|
    | CL_alpha (/deg) | 0.0873 | 0.0988 | 0.1114 | 0.0836 | 0.0954 |
    | CL at 0 deg | 0.158 | 0.100 | 0.093 | 0.148 | 0.245 |
    | Neutral point x (m) | 4.867 | 4.674 | 4.699 | 4.935 | 4.598 |
    | Oswald e (wing AR 6.12) | 1.14 | — | — | 1.04 | 0.82 |

  - **Profile drag at alpha 0:** OpenVSP tool CD0 0.0205 against AeroBuildup 0.0221 (+8 %).
    - Fuselage 61 vs 64 counts.
    - Wing 84 vs 62 counts (VSP form factor 1.62 at t/c 0.23).
    - Nacelles 26 vs 42 counts.
    - Fairing 15 vs 40 counts: AeroBuildup counts the buried area, 24.5 against 12.0 m2 exposed.
    - V-tail 20 vs 13 counts.
  - **Reading:**
    - The thin-surface lift slopes agree within 4 %.
    - Bodies add about 13 % in VSPAERO, and AeroBuildup sits between the thin and body models.
    - AeroBuildup places the neutral point furthest forward (0.08–0.10 m ahead of VSPAERO with bodies) and has
      about 25 % more induced drag than either vortex lattice, so it is conservative on both counts for this
      geometry.
    - The difference in CL at 0 deg (about 1 deg in zero-lift angle) and the Cm offset are open.
    - This is the drawn Halo geometry (V-tail about 2 m forward of the sized tail position), not the sizing's
      aircraft, so absolute static margins are not the sizing's.
- 2026-10-04: Milestone C started; the user approved going to structure after the aero check ("closer than I
  expected").
  - **Code:** `export/openvsp/structure.py`, uncommitted. It builds:
    - a wing box (right half): spars at 0.15/0.60 c, 0.5 m ribs, fairing-attach and nacelle ribs, carbon skins;
    - the fuselage: 0.6 m frames, nose, front-spar, rear-spar and cabin-end bulkheads, a floor;
    - the V-tail: spars and ribs.
    Gauges are placeholders.
  - **Meshing:**
    - OpenVSP's "FeaMeshAnalysis" did not finish (>10 min, 5.4 GB); the per-structure `ComputeFeaMesh` calls work.
    - The V-tail meshes in 3 s.
    - The wing box mesh wrote STL, CalculiX `.inp`, Nastran `.dat`/`.bdf` and the mass file within seconds.
    - The STEP export (`FEA_STEP_FILE_NAME`) appeared to stall; the run was stopped there.
  - **Next:**
    - Make the STEP export optional or find out why it stalls.
    - Mesh the fuselage.
    - Render the structure (PyVista, skins hidden) for user review.
    - Map the AFDD wing-box masses to gauges.
    - Commit to `feat/openvsp-geometry`.
- 2026-10-04: Milestone C first layout rendered for user review (`examples/halo_structure.py`,
  `output/structure/halo_structure.png`).
  - **Meshing:**
    - `ComputeFeaMesh` writes one file type per call and re-meshes each time, whatever the export flags; the
      `FeaMeshAnalysis` wrapper never finished, even with STEP off. `export_structure_meshes(kinds=...)` therefore
      asks only for what is needed.
    - Times on the loaded machine: wing box about 14 s per kind, fuselage about 8 min per kind (each frame slice
      10–60 s), V-tail about 2 s.
    - The frame pitch is 1.0 m for now.
  - **Placeholder wing-box mass:** 431 kg for both halves (skins 266, spars 91, ribs 74). The gauges are not yet
    tied to the AFDD wing.
  - **Open items:**
    - OpenVSP slices are full planes, so the "frames" are solid discs. Ring frames would mean beam-only frames,
      or cut-outs.
    - There is no fairing, nacelle or gear structure.
    - Map the AFDD wing-box masses to gauges.
    - Note on `timeout`: on Windows it kills only the `.venv` launcher, so the child python keeps running. Kill the
      children by command line after each timeout.
- 2026-10-04: user: "ideally we can back check our weight build up methods against the layout and thicknesses
  ... for the primary structure of the wing, tail, and fuselage".
  - **Code:** `export/openvsp/structure_check.py` (8 analytic tests), `examples/halo_structure_reference.py` (solves
    the current reference and writes loads and weights to `output/structure/reference.json`), and
    `examples/halo_structure_check.py`. Gauges come from simple ultimate loads and the AFDD stiffness requirements,
    not from the weight models, so the check is independent.
  - **Reference:** plan 030 thermal, 6,367 kg (14,037 lb), n_ult 4.5, jump n 2.0, tip mass 998 kg.
  - **Wing, layout vs AFDD primary** (torque box 89.8 + stiffness caps 4.1 + jump caps 26.7 = 120.5 kg):
    - As sized (constant chord): 129.3 kg (1.07x). Skins 1.37 mm from the torsion GJ; caps at the root 24.2 cm2
      from beam stiffness; ribs 19.2 kg. Without ribs, 110 kg (0.92x).
    - As drawn (taper 0.6): 138.5 kg (1.15x). The uniform GJ requirement thickens the small tip box to 3.2 mm.
    - Root jump moment: 318 kN m in the layout vs AFDD's 310 kN m.
  - **V-tail:** 12.0 kg of layout primary structure, all minimum gauge (1 mm skins, caps 0.5 cm2 at a 45.7 kN panel
    ultimate), against 56.2 kg for the Raymer horizontal plus vertical tail group.
  - **Fuselage:** 310 kg of layout primary structure against the 636 kg Raymer group.
    - Skin is minimum gauge: 1.0 mm used, 0.41 mm needed for the 387 kN m bending at the rear spar.
    - Breakdown: skin 164, stringers 49, 17 ring frames 48, bulkheads 22, floor 27.
    - Layout areas are from the drawn 11 m boxy fuselage; Raymer uses the 12.8 m round tube.
  - **Reading:**
    - The AFDD wing agrees with an explicit layout within about 10 %.
    - The Raymer tails and fuselage are whole-group correlations; their primary structure is minimum-gauge
      dominated at this size. A like-for-like check needs secondary-structure allowances (control surfaces,
      fittings, doors and cut-outs, fairings) or a layout-based secondary estimate.
- 2026-10-04: frames switched to ring frames (I-section beams, 75 x 30 x 1.6 mm) at 0.6 m pitch, with full-plate
  bulkheads only. The fuselage mesh with ring frames is still running.
