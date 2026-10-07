# AFDD tiltrotor wing as the Halo reference, payload back to 900 kg

Status: COMPLETED 2026-10-03. Follows plan 024 (Tier 20). Approved
the switch on 2026-10-03, answering "yes" to "make the NDARC tiltrotor wing
the default and raise the reference payload toward 900 kg".

## Goal and scope

- `HaloAssumptions.wing_weight_model` defaults to `"afdd_tiltrotor"`.
- `HaloRequirements.mass_payload_kg` defaults to 900 kg, the original target.
  The maximum is about 959 kg at 210 kt.
- Legacy sets pin `wing_weight_model="raymer"`:
  - `assumptions_tier12`, `assumptions_tier12b`, `assumptions_tier16` and
    `assumptions_tier17`;
  - the new `requirements_plan022` / `assumptions_plan022` keep the 780 kg
    plan 022 reference.
  - Notebook shims for Tiers 10–16 and Tier 20 pin their own references.
- Not in scope: Tier 15 and Tier 21, still in progress.

## References

- Plan 024 (`docs/decisions/024-tiltrotor-wing-weights.md`): the
  model, calibration and sources.
- Plan 022: the equivalent-circuit battery reference and its warm start.

## Assumptions

Unchanged from plans 022 and 024.

## Interfaces

- **Changed defaults:** `wing_weight_model` and `mass_payload_kg`.
- **New named sets:** `requirements_plan022` and `assumptions_plan022`.
- **`solve_halo_sizing` starting point** (equivalent-circuit battery, no
  `initial`):
  - first, the constant-battery solve, as before;
  - if that start fails, the equivalent-circuit solve at 85 % of the payload,
    by the same rule, is the start.
  - Each step is one coupled solve; nothing is iterated to convergence.

## Symbolic considerations

None new.

## Tests

- **Plan 026 reference:** 900 kg closes at 14,247 ± 5 lb. Whirl-flutter
  torsion at maximum speed and the engine-out end voltage both bind.
- **Plan 022 reference:** reproduces 14,436 lb with `requirements_plan022`
  and `assumptions_plan022`.
- **Tier 20:** tests at 780 kg are unchanged (6,144 kg).

## Acceptance

- The full unittest suite passes.
- The Tier 0, 10–17 and 20 notebooks pass.
- The dashboard is republished.

## Progress and decisions

- 2026-10-03: from the constant-battery start, the 900 kg
  equivalent-circuit solve reaches local infeasibility. Started from the
  780 kg equivalent-circuit solution it converges, which is why the
  fallback start was added.
- Reference result: 6,462 kg (14,247 lb).
  - Battery: 426 kg, 62.7 kWh.
  - Wing: 22.2 m².
  - Binding constraints:
    - engine-out end voltage;
    - whirl-flutter torsion at 210 kt;
    - hover motor, gearbox and rotor power;
    - rotor radius (span limit);
    - static margin and Cn_beta;
    - mission end SOC.

## Deferred

Multistart (Tier 22) to replace the hand-written starting-point rules.
