# AeroBuildup aerodynamics as the Halo reference

Status: COMPLETED 2026-10-04. Follows plan 025 (Tier 21). Decided on
2026-10-03: "aerobuild up as default".

## Goal and scope

- `HaloAssumptions.aerodynamics_model` defaults to `"buildup"`
  (`asb.AeroBuildup` plus the Scholz items AeroBuildup lacks, the blown-wing
  correction, and geometric download).
- Requirements stay at 900 kg and 210 kt, as recommended when the switch was
  proposed. The added payload capability (about 1,842 kg at 210 kt) is a
  sensitivity, not a new requirement, until the drag calibration is settled.
- Legacy sets pin `aerodynamics_model="simple"`:
  - `assumptions_tier12`, `assumptions_tier12b`, `assumptions_tier16`,
    `assumptions_tier17`, `assumptions_plan022` and `assumptions_tier20`;
  - the new `requirements_plan026` / `assumptions_plan026` keep the
    simple-aero 900 kg reference at 14,247 lb.
  - The notebook shims pin "simple" as well.
- Not in scope:
  - drag calibration against XV-15 flight data (transition location,
    fittings area);
  - Tier 15, still in progress.

## References

- Plan 025 (`docs/decisions/025-aerodynamics-buildup.md`).
- Plans 022 and 026 (starting-point rules and legacy sets).

## Assumptions

Unchanged from plan 025.

## Interfaces

- **Changed default:** `aerodynamics_model`.
- **New named sets:** `requirements_plan026` and `assumptions_plan026`.

## Symbolic considerations

None new.

## Tests

- **Plan 027 reference** (defaults): closes at 900 kg; take-off mass within
  5 lb of the plan 025 result (13,639 lb).
- **Plan 026 reference:** reproduces 14,247 lb with the pinned sets.
- **Trajectory problems:** still solve on the default aircraft.

## Acceptance

- Full unittest suite passes.
- Tier 0, 10–17, 20 and 21 notebooks pass.
- Dashboard republished.

## Progress and decisions

- 2026-10-03: plan written.
- 2026-10-04: done.
  - Reference: 13,639 lb, from the defaults (starting from a Scholz solve).
  - Unittest: 431 pass.
  - Notebooks: Tier 0 401/401, 10 14/14, 12 11/11 and 9/9, 13 8/8, 14
    26/26, 16 18/18, 17 21/21, 20 22/22, 21 26/26.
  - With 11 Jupyter kernels at once, Windows ran out of socket buffers
    (ZMQ "No buffer space available"). Keep batches to about 6 kernels.

## Deferred

- Drag calibration (Tier 21 follow-up).
- Multistart (Tier 22).
