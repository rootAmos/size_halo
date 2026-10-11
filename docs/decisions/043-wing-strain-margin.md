# Plan 043: wing spar-cap strain at ultimate load as a sizing constraint

Status: in progress (overnight, 2026-10-10). `docs/NEXT_STEPS.md` item 2, first step.

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

To be filled in with the implementation (below).

## Verification

To be filled in.

## Assumptions

- **Load-carrying share of the calibration (decision taken for the author, overnight):** half of the XV-15 wing
  calibration increment, 0.5 x (1.327 - 1) = 0.164, is load-carrying spar-cap and cover material; the rest is
  fittings, sealant and non-structural mass. Applied as a multiplier 1.164 on the AFDD box-wall and spar-cap
  areas (the same frame as the earlier FE diagnosis, where the full 1.327 on the gauges gave 0.83 of the
  allowable). Counting half of the increment's *mass* instead would credit more (about +39 % of the primary
  structure) and is not used.

## Progress log

- 2026-10-10: plan written.
