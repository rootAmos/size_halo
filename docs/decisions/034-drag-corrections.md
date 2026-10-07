# Excrescence and trim drag corrections

Status: COMPLETED 2026-10-04 (as an option; the default switch is the author's
call). Follows plans 025 and 027.

The author noted on 2026-10-04 that AeroBuildup "doesn't have all the drag
correction factors for trim and excrescence, so 15–20 % makes sense".

## Goal and scope

Add the drag that AeroBuildup and the Scholz build-up leave out, as
explicit, documented items:

- **Excrescence, leakage and protuberance:** (factor − 1) × the component
  profile-plus-interference drag, as a separate breakdown item.
- **Trim drag:** a fraction of parasite plus induced drag, added to CD (not
  CD0).

## References

Johnson 2010, NDARC validation, Table 1: XV-15 cruise D/q 9.25 ft². Of
that, 6.25 ft² is wing, tails, fuselage and pylons, and 3.00 ft² is
fittings and fixtures.

## Assumptions

- **Calibration:**
  - AeroBuildup gives 4.935 ft² for the XV-15 components; Scholz gives
    5.344 ft².
  - Factors 1.27 and 1.17 match the 6.25 ft².
- **Trim drag:** 2 % (assumed; usually 1–5 % for an aft tail at cruise).

## Interfaces

- `BuildupAerodynamics` and `ScholzAerodynamics` take `factor_excrescence`
  (default 1.0) and `fraction_trim_drag` (default 0.0).
- `HaloAssumptions` adds:
  - `drag_corrections` (default False);
  - `factor_excrescence_buildup` = 1.27;
  - `factor_excrescence_scholz` = 1.17;
  - `fraction_trim_drag` = 0.02.

## Symbolic considerations

At default settings the terms are not added at all, so the expression graph
is unchanged. A numerically zero term changed IPOPT's path enough to fail
the fragile thermal-off start (Restoration_Failed). The settings are
numbers, never Opti variables, so the Python check is legitimate.

## Tests

`tests/aerodynamics/test_drag_corrections.py`:

- default models are clean;
- the Halo factors match the XV-15 components to NDARC within 1 %;
- the excrescence item is proportional to component drag;
- trim scales CD, not CD0;
- the Halo switch works.

## Acceptance

- The full suite passes.
- Defaults are unchanged (14,037 lb).

## Progress and decisions

- **2026-10-04, with corrections:**

| | Uncorrected | Corrected |
|---|---|---|
| Take-off mass | 14,037 lb | 14,387 lb |
| Cruise L/D | 9.86 | 9.02 |
| Cruise speed | 162 kt | 159 kt |
| Max payload at 210 kt | ~1,650 kg | 1,446 kg (18,706 lb) |

  The binding set is unchanged.

## Deferred

- Trim drag from the actual tail load (requires trim with downwash).
- Calibration of the transition location.
