# Bottom-up tiltrotor wing box: sized layout, no calibration factor

Status: DONE (2026-10-11). Branch `feat/bottom-up-wing-box`.

## Goal and scope

The AFDD tiltrotor wing (plan 024) sizes the torque box for torsion stiffness and the spar caps for bending
stiffness and the jump take-off, then needs an XV-15 factor of 1.33 for everything it does not size (stiffeners,
ribs, joints, mountings). It gives masses, not a structure. This plan adds a class II-1/2 ("bottom-up") wing box
that:

- sizes every primary item for its own load case and returns the **layout**: spanwise skin, stringer, web and
  spar-cap sizes, stringer pitch and height, rib pitch and rib thickness;
- needs **no calibration factor** (mass factor 1.0), checked on the XV-15, V-22 FSD and Bell D266;
- is a third `HaloAssumptions.wing_weight_model`, `"bottom_up_tiltrotor"`, symbolic so it sizes inside the
  optimization with the existing whirl-flutter frequency margins;
- exports its layout so the OpenVSP structure and the CalculiX check use the sized gauges and rib pitch.

The default (`"afdd_tiltrotor"`) and every existing reference result stay unchanged.

Not in scope: fatigue, impact damage tolerance, joints and cut-outs (statistical, Torenbeek), a sized fuselage.

## References

- E. Torenbeek, LR-693, TU Delft (1992): bending and shear material, ribs (eq. 47), non-optimum allowance
  (eq. 49), mountings (sec. 8.2, eq. 50), torsion penalties (sec. 8.3), fixed leading/trailing edges (eqs. 52-53),
  ailerons (eq. 56), supports (sec. 9.6). Validated within 4 % on four transports.
- A. Elham, G. La Rocca, M. van Tooren, Aerospace Science and Technology 29 (2013) 100-113 (EMWET): equivalent
  panels at an effective distance, upper panel allowable min(material, stiffened-panel buckling F sqrt(P E / L)),
  composite cutoff strains (tension 0.005, compression 0.0045, shear 0.006).
- D. Howe, Aircraft Loading and Structural Layout (2004), ch. 13-16: Bredt-Batho torsion skin (15.2.2-15.2.3),
  bending surfaces and stringer share (15.2.4, 15.2.7: stringers about 35 % of the cover), spar webs for shear plus
  torque (15.2.6), ribs (15.3), skin local buckling coefficients (16.2.3).
- XV-15 wing breakdown and stiffness: Acree, Peyran and Johnson (1999), table 5 (`data/weights`).

## Model

At fixed fractions of each panel (fuselage side to tip), with the closed-form elliptic lift moment
(Torenbeek eq. 17) and tip point loads:

- **Loads:** ultimate flight (n_ult, elliptic lift less tip inertia) and jump take-off (AFDD definition: 1.5 x
  n_jump x (W/2 - tip weight) at the tip); the larger moment and shear at each station; airplane-mode inertia torque
  of the tip mass offset.
- **Stiffness required:** AFDD's frequency relations (shared function), applied as a floor at every station.
- **Skin:** at least the Bredt-Batho thickness for the torsion stiffness, the torque shear flow and minimum gauge.
- **Covers (skin + stringers):** lower at the tension allowable (-10 % for torsion), upper at
  min(compression allowable, F sqrt(N E / L)); stringers 35 % of the bending cover (Howe), skin the rest unless the
  torsion skin is thicker. Stringer pitch from skin local buckling at the cover stress (K = 3.62), height = pitch /
  3.5, Zed area 1.8 h t.
- **Webs:** shear plus torque shear flow, +20 % (Torenbeek 8.3), at least the torsion skin and the web minimum
  laminate. Composite webs are sandwich panels (`WingBoxLayout.construction_web="sandwich"`): carbon faces carry
  the shear, a honeycomb core is deep enough that the depth x rib-pitch panel does not buckle in shear at ultimate
  (D = E (t/2) h^2 / 2 >= q b^2 / (k_s pi^2), k_s = 5.35 + 4 (b/a)^2); core and adhesive mass included. "plate"
  (solid web thick enough on its own) and "none" (strength only) remain options.
- **Hot/wet:** carbon-epoxy modulus x 0.90 in every buckling check (upper cover panels, stringer pitch, webs) and
  shear allowable x 0.80 (webs, torque skin); the cover strain allowables are EMWET design values and are not
  knocked down. Aluminium: 1.0.
- **Spar caps:** beam and chord stiffness still missing.
- **Root peak:** near the fuselage side the bending allowable can be divided by a peak factor; 1.0 for the
  distributed covers (CalculiX / beam theory 0.97; the plan 043 value 1.33 was fitted on discrete AFDD caps).
- **Ribs:** Torenbeek eq. 47 at its 0.75 m statistical pitch, scaled by 0.75 m / pitch; thickness from that mass
  over the rib area. The pitch is an optional design variable (`free_pitch_rib_wing`, 0.25-1.0 m); in OpenVSP the
  ribs run in equal bays between the fairing-attach and nacelle ribs (`StructureLayout.ribs_between_attach_ribs`).
- **Torsion frequency:** NDARC's single-mode relation times 0.756 (`correction_torsion_frequency`; XV-15 and
  CalculiX both need about 0.57 x GJ). **Non-optimum, mountings, fairings, controls:** Torenbeek as referenced.
- **Realized stiffness** for the frequencies: flexibility-weighted along the span (bending: weight (L - y)^2;
  torsion: uniform), so a station above the floor raises it.

## Verification

- Unit tests: closed-form elliptic moment against quadrature; limiting cases (no tip mass, zero torque);
  trends (thicker section -> lighter covers, longer rib pitch -> lower upper allowable); CasADi compatibility.
- Uncalibrated wing weight: XV-15, V-22 FSD and D266 within the prototype's errors (+17 %, -5 %, +6 %).
- Sizing: a `"bottom_up_tiltrotor"` solve converges; the Halo wing against the AFDD x 1.33 wing.
- CalculiX: the exported layout's cap strain and frequencies against the bottom-up predictions.

## Assumptions

- Spars at 0.15 / 0.60 chord (the layout), cover centroid at 0.95 of the mean box depth, panel efficiency 0.8
  (Z stringers), rib pitch 0.5 m (layout) unless a design variable.
- Graphite epoxy non-optimum allowance 0.5 mm (Torenbeek's integral construction value; co-cured, fewer joints);
  aluminium 1.0 mm.
- Torenbeek's secondary-structure correlations are for aluminium transports; used unchanged.
- Composite spar webs: at least 1.5 mm laminate (eight balanced +-45 plies), Nomex core at least 6 mm (48 kg/m3),
  0.30 kg/m2 adhesive. Hot/wet knockdowns 0.90 (modulus, stability) and 0.80 (shear allowable).
- Buckling criterion: no buckling of primary structure below ultimate load, hot/wet.

## Progress log

- 2026-10-11: prototype (standalone): XV-15 +17 % (946 lb breakdown), V-22 -5 %, D266 +6 % with no factor;
  AFDD uncalibrated -25 / -38 / -27 %, AFDD x XV-15 factor 0 / -18 / -2 %. Halo v3.6 wing 429 kg against 405 kg.
- 2026-10-11: `weights/wing_box.py` (symbolic), `BottomUpWingMassModel`, `wing_weight_model="bottom_up_tiltrotor"`,
  banded orthotropic CalculiX deck (`fe_wing.WingBoxBands`), `examples/halo_wing_box.py` and `halo_wing_box_fe.py`.
  NaN guards for optimizer iterates (negative jump load, fuselage side beyond the tip, unloaded panels).
  - Module, no factor: XV-15 +18 %, V-22 FSD -3 %, D266 +9 %.
  - Halo sizing with the bottom-up wing converges: take-off 6,688 kg (14,745 lb), wing 409.9 kg (primary 200.7:
    covers 113.5, webs 15.1, caps 0, ribs 43.2, non-optimum 28.9; mountings 68.5, fairings 67.7, controls 72.9).
    Root upper skin 3.5 mm + 1.9 mm smeared stringers at 116 mm pitch, tapering to 1.2 + 0.6 mm; webs at the 1 mm
    minimum gauge; no stiffness caps; ribs 1.9 mm at 0.5 m. Root lower-cover strain 0.00447 (margin +5 %).
  - CalculiX on that layout (OpenVSP mesh at the sized rib pitch, banded gauges): symmetric beam 23.9 rad/s against
    23.3 (FE +3 %), chord 38.0 against 38.7 (-2 %), **torsion 48.0 against 64.2 (-25 %)**. Ultimate jump take-off root
    cap strain 0.00326, margin +31 % (the sizing's +5 % includes the plan 043 peak factor 1.33, fitted on the AFDD
    wing with discrete cap pads; the distributed covers do not show that peak).
  - Same-code AFDD default: 6,885.1 kg (15,179 lb), wing 407.9 kg (x 1.327).
  - Torque case (CalculiX, clamped, pure tip torque): FE GJ 8.47 MN m2 against the bottom-up 6.02 (FE +41 %): the
    rectangular box on the spar depths understates the enclosed area. The NDARC single-mode relation with the FE GJ
    gives 76.1 rad/s against the free-free FE torsion mode 48.0: the frequency relation, not the stiffness, is the
    optimistic part (it is shared with the AFDD wing).
  - Changes: enclosed area and cover lever arm from the airfoil's mean depth across the box (Simpson); root peak
    factor 1.0 for the distributed covers (`HaloAssumptions.factor_peak_strain_wing_box`; FE 3 % below beam theory);
    rib pitch as an optional design variable (`free_pitch_rib_wing`, 0.25-1.0 m). No factor: XV-15 +8 %, V-22 FSD
    -12 %, D266 -1 % (mean |error| 7 %).
  - Re-solve, free rib pitch, against the AFDD default: take-off 6,569.2 kg (14,483 lb, -316 kg); wing 360.7 kg
    (-51.5: covers 95.8, webs 14.2, caps 0, ribs 27.0, non-optimum 27.2, mountings 65.2, fairings 63.2, controls
    68.1); powertrain -203.9, systems -17.7, gear -8.8, fuel -23.6 (the growth chain). Optimum rib pitch 0.745 m
    (rib 1.82 mm); root upper skin 2.86 mm, tip 1.0 mm (minimum gauge). The frequency floor does not bind: strength
    and minimum-gauge material already give torsion 1.82 / rev and beam 0.66 / rev at 39 rad/s (targets 1.087,
    0.432), so the placement rotor speed variable sits at its 5 rad/s bound (inactive, harmless).
  - Buckling in CalculiX (`*BUCKLE` on the ultimate jump take-off): upper covers as layered S8R shells inside the
    mould line (skin + bottom flanges, stringer webs as a spanwise-only core, top flanges: the stringers' area and
    lever arm), modes located by their peak node. At F = 0.8 the first mode had a margin of -3 % over ultimate; at F = 0.75
    (rib pitch 0.714 m, take-off 6,579.7 kg) the first, -9 %, peaked at y = 0 inside the clamped centre section (an
    artefact of offset layered shells on clamped nodes; layered covers are now outboard of the clamp only); the
    next modes are the 1 mm spar webs in shear at +9 to +13 % over ultimate and the non-structural fairing at +13 %; no
    cover mode in the first five. The layered covers also add a spurious soft mode to the free-free dynamics, so
    the modal decks keep plain orthotropic covers.
  - Torsion frequency: with 90 modes the FE torsion mode is 57.7 rad/s free-free and 58.5 clamped; NDARC's relation
    from the FE GJ gives 77.6 (stiffness ratio 0.568). The XV-15 driven by its published modes needs 0.574 x its
    published GJ. The relation's span / 4 effective length against the clamped panel explains it
    (sqrt(5.02 / 2.93) = 1.31). The bottom-up box now multiplies the torsion frequency by sqrt(0.572) = 0.756
    (`WingBoxLayout.correction_torsion_frequency`); bending frequencies agree with the FE within 8 % and are not
    corrected. The AFDD default keeps the relation: its torsion whirl margins are optimistic by the same 1.32
    (a separate decision).
  - Spar webs: the strength-sized webs sat at the 1 mm minimum, and CalculiX buckled them in shear at margins of -1
    to +3 % over ultimate (+49 % over limit: no buckling below limit, none of the margin needed below ultimate or hot/wet). A solid
    web sized on its own (bare plate between ribs) needs 3.6 mm: webs 52 kg, take-off 6,904.9 kg (15,223 lb). Real
    composite webs are sandwich or stiffened, so the webs are now sandwich panels with a 1.5 mm minimum laminate,
    and hot/wet knockdowns apply to all buckling checks and the shear allowable. No factor: XV-15 +9 %, V-22 FSD
    -10 %, D266 +0 %.
  - Final re-solve (free rib pitch, sandwich webs, hot/wet): take-off 6,640.7 kg (14,640 lb, -244 kg against the
    AFDD default); wing 389.8 kg (covers 101.8, webs 26.9, caps 0, ribs 30.0, non-optimum 27.9, mountings 68.0,
    fairings 65.1, controls 70.1). Rib pitch 0.691 m (1.82 mm). Root upper cover 3.08 mm skin + 1.66 mm stringers at
    94 mm pitch, lower 2.52 mm skin; both to the 1 mm minimum outboard. Webs 1.5 mm faces (minimum laminate; strength
    needs about 1.3 mm hot/wet) on a 6.0 mm core (minimum core; buckling needs less) along the whole span: the jump
    take-off shear enters at the nacelle, so the shear flow is nearly uniform.
  - CalculiX on that layout (equal rib bays, sandwich webs as layered shells): root strain FE 0.00347 against sizing
    0.00341 (margin +26 %); FE GJ 18 % above the bottom-up; free-free frequencies FE against the bottom-up beam: bending
    -8 %, chord -2 %, torsion +7 %. Buckling under the ultimate jump take-off: no web or cover mode in the first five; first mode at
    the clamped centreline (artefact, margin +10 % over ultimate), then the non-structural leading/trailing-edge
    fairing skins at +12 to +39 % (+1 % hot/wet). Primary structure clears no-buckling-below-ultimate, hot/wet.
  - Halo numbers in this plan are on the v3.6 baseline (AFDD default 15,179 lb), so the comparison is like-for-like.
