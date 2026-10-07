# Thermal model as the Halo reference

Status: COMPLETED 2026-10-04. Follows plan 028 (Tier 19). The author answered
"yes" on 2026-10-04 to making the thermal model the default.

## Goal and scope

- `HaloAssumptions.thermal_model` defaults to True:
  - heat loads to a ram-air heat exchanger (mass from its rating);
  - cooling drag in airplane mode, fan power in hover;
  - lumped motor, generator and battery temperatures with short-time
    ratings.
- Requirements stay at 900 kg and 210 kt.
- Every named legacy set pins `thermal_model=False`. The new
  `requirements_plan027` / `assumptions_plan027` keep the 13,639 lb plan 027
  reference.
- Notebook shims for Tiers 10–16, 19, 20 and 21 pin thermal off. Tier 14's
  shim pins `assumptions_tier16`.
- Trajectory tests fly the plan 027 aircraft: the trajectory model has no
  thermal states yet (deferred).

## References

Plan 028 (`docs/decisions/028-thermal.md`).

## Assumptions

Unchanged from plan 028.

## Interfaces

- **Changed default:** `thermal_model`.
- **New named sets:** `requirements_plan027` and `assumptions_plan027`.

## Symbolic considerations

None new.

## Tests

- **Plan 030 reference** (defaults): closes at 900 kg; 14,037 ± 10 lb.
- **Plan 027 reference:** reproduces 13,639 lb with the pinned sets.

## Acceptance

- The full unittest suite passes.
- The Tier 0, 14, 19, 20 and 21 notebooks pass. The other tier notebooks
  are pinned to legacy sets whose values do not change.

## Progress and decisions

- 2026-10-04: implemented.

## Deferred

- Thermal states along trajectories.
- Temperature-dependent losses and cell resistance.
