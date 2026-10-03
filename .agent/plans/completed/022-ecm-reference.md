# Equivalent-circuit battery as the Halo reference

Status: COMPLETED 2026-10-03. Follows plan 021 (Tier 17). User decision
2026-10-03, choosing between a lower payload, a redefined reserve, or a
different cell:

> take a lower payload … not much more we can do to stretch the cells.

## Goal and scope

- **Reference battery:** `HaloAssumptions.battery_model` defaults to
  `"ecm"`. This is the Samsung 50G-shaped pack at end of life, with power
  factor 5, from plan 021.
- **Reference payload:** `HaloRequirements.mass_payload_kg` = 780 kg. The
  maximum on the current aircraft (Tier 13 machines, Tier 16 hot-day hover)
  is 785 kg, so 780 kg leaves 5 kg of slack.
- **Legacy sets:** `requirements_tier16` and `assumptions_tier16` reproduce
  the previous reference: 900 kg, constant battery, 13,760 lb.
  - The earlier sets (`*_tier10c`, `*_tier12*`, `*_tier11a`) pin
    `mass_payload_kg=900` and `battery_model="constant"` explicitly.
  - Tier notebooks and tests that reproduce earlier tiers use these sets.
- **Trajectory (Tier 14 code):** `TiltrotorPointMass` and
  `build_tiltrotor_trajectory` accept the equivalent-circuit battery:
  - the motors see the battery terminal voltage;
  - SOC is coulomb-counted;
  - the limits are current and voltage, plus the branch constraint
    V >= V*/2.
  - `halo_trajectory_case` takes `requirements` and `assumptions`.
- **Dashboard:** refreshed on the new reference, covering Tiers 13–17.
- **Not in scope:**
  - RC polarization states along trajectories: the trajectory uses steady
    polarization, which is conservative for short manoeuvres.
  - Cell changes or a reserve redefinition (the user declined both).

## Assumptions

- Power factor 5 stays. The user: "not much more we can do to stretch the
  cells."
- End-of-life sizing stays: capacity 0.8, resistance 1.5.

## Interfaces

- `HaloAssumptions.battery_model`: default changes from `"constant"` to
  `"ecm"`.
- `HaloRequirements.mass_payload_kg`: default changes from 900 to 780.
- New `requirements_tier16` and `assumptions_tier16`.
- `halo_trajectory_case(sizing=None, requirements=HaloRequirements(),
  assumptions=HaloAssumptions())`.
- `build_tiltrotor_trajectory`:
  - the supply is evaluated before the forces, so the motor voltage is the
    battery terminal voltage (previously it was the constant OCV);
  - the battery limits come from the battery type.

## Symbolic considerations

- Battery-type branches are Python `isinstance` checks on the component
  type, never on symbolic values.
- With the constant battery, the terminal voltage differs from OCV only by
  I·R, about 0.1 %. The Tier 14 results move by less than test tolerance.

## Tests

- Reference: closes at 780 kg with `battery_model == "ecm"`. Max payload is
  785 ± 5 kg.
- Legacy: `requirements_tier16` with `assumptions_tier16` gives
  13,760 ± 5 lb.
- Trajectory: a minimum-energy transition on the ECM reference solves; bus
  voltage stays inside the cell window; SOC falls.
- Existing Tier 10–17 tests and notebooks pass on their pinned sets.

## Acceptance

- Full unittest suite passes.
- Tier 0 and Tier 10–17 notebooks are executed and pass.
- The dashboard is republished on the 780 kg ECM reference.

## Progress and decisions

- 2026-10-03: plan written after the user's decision.
- Reference result: 6,548 kg (14,436 lb).
  - Battery: 440 kg, 64.9 kWh, 210s x 21.3p.
  - The engine-out end voltage (525 V) binds; so do the 210 kt turbine
    power and the hover motor power.
- Cold start: from the generic guess, IPOPT reaches local infeasibility with
  the equivalent-circuit pack. `solve_halo_sizing` therefore first solves the
  constant-battery problem when `initial` is None. This is documented in its
  docstring and is a starting point only.
  - The max-payload solve likewise starts from a constant-battery design.
  - Multistart is deferred to Tier 22.
- Trajectory on the reference:
  - minimum-energy transition: 22.1 s;
  - time to climb to 10,000 ft: 227 s, SOC 0.95 to 0.85, bus voltage inside
    the window.
  - Tier 14 on its pinned aircraft: unchanged within test tolerance.
- Test bug found: `test_closed_mass_is_a_fixed_point` rebuilt the aircraft
  with the default requirements (now 780 kg). Fixed by passing the legacy
  set.
- Parallel runs: seven notebooks concurrently with threaded BLAS gave IPOPT
  failures. With `OMP_NUM_THREADS=1`, all pass.
- Acceptance:
  - unittest: 353 pass;
  - notebooks: Tier 0 373/373, Tier 10 14/14, Tier 12 11/11 and 9/9,
    Tier 13 8/8, Tier 14 26/26, Tier 16 18/18, Tier 17 21/21;
  - dashboard version 9 published.

## Deferred

- RC-state propagation along trajectory nodes.
- Electrical layer (Tier 15): splitting the inverter out of the machine
  torque density.
