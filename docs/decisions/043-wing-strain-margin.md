# Plan 043: wing spar-cap strain at ultimate load as a sizing constraint

Status: implemented 2026-10-10 (overnight), option default off. `docs/NEXT_STEPS.md` item 2, first step.

## Goal and scope

The linear CalculiX wing-box check (`examples/halo_wing_fe.py`, last run on v3.3.5, plan 035/038) puts the peak
front spar-cap strain in the ultimate jump take-off **10 % above the ultimate allowable: a negative margin of
-10 %**. The sizing never saw it: the wing mass is the AFDD/NDARC handbook model times the XV-15 factor, and the FE
is a check outside the loop. This plan makes ultimate cap strength a sized constraint.

In:
- a reduced-order spar-cap strain model inside the sizing, calibrated to reproduce the FE peak;
- a margin "ultimate cap strain <= allowable" behind an `HaloAssumptions` option (default off, every reference
  unchanged), with the optimizer free to add spar-cap area (and wing mass);
- an FE re-check of the newly sized wing;
- a stated plate-buckling estimate for the 1 mm covers and webs (open item, not fixed).

Out: buckling sizing (stiffened covers), root fittings instead of the clamp, other load cases (gusts,
manoeuvres, 1-g with rotor loads), feeding FE gauges back as calibration factors.

## References

- Plans 024 and 026: AFDD tiltrotor wing (NDARC Theory, NASA/TP-2009-215402, sec. 19-1.1), XV-15 factor 1.327.
- Plan 031: CalculiX wing-box check on the OpenVSP mesh (`aircraft_closure.export.openvsp.fe_wing`).
- Plan 035/038: cap depth (kappa 0.826) and 1 mm minimum gauge; FE re-check on v3.3.5: peak 0.00518 against
  0.0047 (graphite-epoxy, NASA/TP-2004-212262 table 1).
- Bruhn, *Analysis and Design of Flight Vehicle Structures*, C5 (flat-plate compression and shear buckling).
- Diagnosis carried over (NEXT_STEPS item 2): not torsion; the FE omits the calibration material; root shear lag
  under the full clamp puts the front cap at 1.7x the rear (beam theory 1.2x); no buckling step.

## Model

Reduced-order, in `aerosandbox.numpy` (`aircraft_closure.weights.afdd`), so it stays symbolic in the sizing:

- **Load** (`moment_jump_station_ultimate_Nm`): the FE deck's jump take-off. At each rotor (span end) the force
  1.5 x n_jump x g x (m_design / 2 - m_tip), n_jump = 2; moment at a station 0.3 m outboard of the fuselage side
  (the inner edge of the FE strain band; the clamp edge itself is a singularity). Wing inertia relief neglected,
  as in the FE.
- **Section** (`strain_cap_jump_ultimate`): the section `fe_wing.properties_from_afdd` meshes. A box between the
  spars (0.45 c wide, cap depth kappa x t/c x c deep, kappa = 0.826), walls of one gauge (AFDD torque-box area over
  the perimeter, E_box), plus the four caps (AFDD cap area, E_spar) at half depth. Both areas times the
  load-carrying factor 1 + s (f_wing - 1). EI = E_box t (2 w (d/2)^2 + d^3/6) + E_spar A_caps (d/2)^2.
- **Strain:** beam strain M (d/2) / EI times a peak factor k for root shear lag under the clamp, the cap strips
  and the mesh peak. **k = 1.3324**, calibrated so the model reproduces the v3.3.5 FE peak (0.00518 from beam
  strain 0.00389; `examples/halo_wing_strain_calibration.py`). Beam theory alone gives 0.83 of the allowable: it
  misses the negative margin entirely. k is assumed constant across designs (checked on a second design below).
- **Sizing hook:** `HaloAssumptions.wing_strain_margin` (default False). On: the margin
  `wing spar-cap strain, ultimate jump take-off` (peak <= 0.0047, graphite-epoxy) and a design variable
  `HaloDesign.area_spar_strain_m2` >= 0. It is added to the AFDD wing as constant-area caps (like NDARC's jump
  caps): mass rho A b, fittings fraction and the XV-15 wing factor on top, beam stiffness E A d^2 / 4 (so the
  frequencies and whirl-flutter margins see it). `HaloSizingResult.wing_strain` reports the strain for every AFDD
  wing, with the option on or off.
- **Buckling (open item, `examples/halo_wing_buckling.py`):** simply supported flat plates between the 0.5 m rib
  pitch and the spars (covers in compression) and between the ribs and covers (webs in shear), at ultimate.

## Verification

- `tests/weights/test_wing_strain.py`: beam theory by hand; load-carrying factor divides the strain; station
  moment by hand; the calibrated model reproduces the v3.3.5 FE peak to 0.1 % (beam theory alone: 0.83);
  half the increment brings v3.3.5 to 0.95; added caps: zero by default, mass rho A b x (1 + fittings), stiffness
  E A d^2 / 4; a small `asb.Opti` adds just enough cap to sit on the allowable (symbolic through CasADi).
- `tests/integration/test_halo_wing_strain.py`: off by default, the factor wiring, added caps reach the wing.
- FE re-check of the newly sized wing (below).

## Assumptions

- **Load-carrying share of the calibration (decision taken for the author, overnight):** half of the XV-15 wing
  calibration increment, 0.5 x (1.327 - 1) = 0.164, is load-carrying spar-cap and cover material; the rest is
  fittings, sealant and non-structural mass. Applied as a multiplier 1.164 on the AFDD box-wall and spar-cap
  areas (the same frame as the earlier FE diagnosis, where the full 1.327 on the gauges gave 0.83 of the
  allowable). Counting half of the increment's *mass* instead would credit more (about +39 % of the primary
  structure) and is not used.

## Progress log

- 2026-10-10: plan written.
- 2026-10-10: model, option, tests, FE options and buckling estimate implemented. Calibration: beam strain 0.00389
  on the v3.3.5 wing against the FE peak 0.00518 gives k = 1.3324.
- 2026-10-10: two sizing solves from the cached v3.6 baseline (15,179 lb), option on, machine unit counts held
  at the baseline's (2 motor, 3 generator units):

| Case | Take-off | Wing | Added caps | Peak strain margin (model) | CalculiX margin |
|---|---|---|---|---|---|
| v3.6 reference, AFDD gauges only (no solve) | 15,179 lb | 407.9 kg | - | -6.9 % | not run |
| Option on, half the calibration load-carrying (the assumption) | 15,179 lb (+0) | 407.9 kg | 0 | +8.1 % (inactive) | +4.5 % |
| Option on, no credit (share 0, FE as built) | 15,230 lb (+51) | 418.3 kg (+10.4) | 299 mm2 (5.9 kg raw) | 0.0 % (binding) | +4.0 % |

  - With the stated assumption the v3.6 wing already meets ultimate strength: the margin is inactive and the
    reference is unchanged (751 s solve, it reproduces 15,179 lb).
  - Without any credit for the calibration material the optimizer adds 299 mm2 of cap (all four caps, full span):
    +10.4 kg of wing after fittings and the 1.327 factor, +51 lb take-off after the weight spiral; the margin
    binds. Other binding constraints are unchanged.
  - **FE re-check of the sized (no-credit) wing**, raw gauges plus the added caps, static jump step only:
    peak 0.00451, a **+4.0 % margin**: the negative margin is closed. The reduced model is 4 % conservative on this
    second design (FE peak factor 1.28 against the 1.33 calibrated on v3.3.5), so k is not exactly constant but
    errs on the safe side here.
  - **FE re-check of the v3.6 wing with half the calibration material mapped in** (walls and caps x 1.164): peak
    0.00449, a **+4.5 % margin** (spooles failed on this deck three times with exit -1; solved with CalculiX's
    iterative Cholesky solver instead). The reduced model said +8.1 %, so here it is 3.6 % unconservative (FE
    peak factor 1.39). Across the three FE runs the peak factor spans 1.28-1.39 around the calibrated 1.33: the
    reduced model is good to about +-4 % in strain, and both re-checked designs have a positive FE margin. A
    designer wanting the model always conservative would set `factor_peak_strain_cap_wing` to 1.39 (not done; it
    would not change the half-credit result, still +4.5 % in FE).
  - CalculiX was found at the script's default path (`C:/Users/alexa/Documents/Xenon/Software/CalculiX/
    CalculiX-2.22.0-win-x64/bin/ccx.exe`, overridable with `CCX`); OpenVSP 3.53.1 (for the mesh) was put on
    `PYTHONPATH` from its install folder, nothing installed into the venv.
- Buckling (open item, `examples/halo_wing_buckling.py`, simply supported plates, 0.5 m rib pitch):

| Wing | Wall gauge | Cover stress at ultimate / buckling | Web shear at ultimate / buckling |
|---|---|---|---|
| v3.6, half credit | 1.17 mm | 201 MPa / 0.51 MPa (0.25 %) | 91 MPa / 4.0 MPa (4.4 %) |
| Sized, no credit | 1.00 mm | 219 MPa / 0.38 MPa (0.17 %) | 106 MPa / 2.9 MPa (2.8 %) |

  The unstiffened 0.9 m wide, 1 mm covers buckle at well under 1 % of the ultimate load, so the strain margin
  above is a material margin on a box that is not yet stable. A sandwich cover (same faces, about 13-14 mm core)
  would be stable at ultimate for about 17-18 kg of core and adhesive (before the wing factor; some of it may sit
  inside the non-load-carrying half of the calibration). Not sized here; the next step of NEXT_STEPS item 2.

## Slide summary

**Wing strength: a negative margin found by FE, closed in the sizing.**

- CalculiX found the front spar cap 10 % over the ultimate strain allowable (margin -10 %) in the jump take-off;
  a reduced-order cap-strain model calibrated on that run (beam theory x 1.33 peak factor) is now a sizing
  constraint (`wing_strain_margin`).
- With half of the XV-15 wing calibration material counted as load-carrying, the v3.6 wing has a positive margin
  (+8 % model, +4.5 % CalculiX): no weight change (15,179 lb).
- Counting none of it, the optimizer adds 299 mm2 of spar cap: +10 kg wing, +51 lb take-off; CalculiX on the
  re-sized wing confirms a +4 % margin.
- Caveat: the 1 mm unstiffened covers and webs buckle far below ultimate (covers at under 1 % of it); stiffening
  them (sandwich covers, about 18 kg before factors) is still open.
