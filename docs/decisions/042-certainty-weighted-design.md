# Plan 042: certainty-weighted design (impact x uncertainty = war-room priority)

## Goal

Every input to the design carries a measure of certainty next to its value. The tool computes the impact of every
input on every performance output, and ranks what the team should work on next by **impact x uncertainty**, per
output and overall. The decision ledger (plan 041, PR #16) becomes a view of this framework instead of a
take-off-mass-only list.

## Concepts

**Input.** Anything the sizing reads: requirements, assumptions, calibration factors, technology levels, design
choices. Each input is an `UncertainInput`:

| field | meaning |
|---|---|
| `value` | nominal, in SI |
| `sigma` (or `low`/`high`) | one-sigma spread, or a range read as a uniform distribution |
| `kind` | `design` (we choose it), `technology` (cell, motor), `model` (calibration factor, model-form error), `requirement` |
| `source`, `fidelity` | where the number comes from (handbook, scaled, analysis, test); same rule as the ledger evidence |
| `owner`, `lock_date` | who retires it and when it must be frozen (schedule axis) |
| `retire` | the activity that would shrink it, its cost/duration and the expected sigma after |

Design choices (aspect ratio, disk loading, cruise speed) have no uncertainty in their value: we pick them. Their
uncertainty sits in the models that turn them into weight and performance (e.g. bottom-up wing weight: XV-15 +17 %,
V-22 -5 %), so each model gets a `model` input (a calibration factor with a sigma). Design inputs are ranked by
impact alone, as **levers**; the rest are ranked by impact x uncertainty, as **risks**.

**Output.** The performance metrics: range at fixed take-off mass, take-off mass at the fixed mission, hover power,
cruise L/D, battery capacity at end of life, whirl-flutter margin (1.15 V_D), wing strain margin, cost when present.
Each output may carry a requirement or limit, which turns a spread into a probability of missing it.

**Impact.** The elasticity `E_ij = (dy_i / dx_j) * (x_j / y_i)`: % change in output i per % change in input j.
Two modes, both reported:

- **fixed aircraft**: the sized geometry and masses held, input changed -> performance delta ("5 % aspect ratio
  is x % range on the aircraft we have");
- **re-sized**: the mission held, the aircraft re-closes -> the interdependencies (aspect ratio -> L/D up, wing
  mass up, torsion stiffness down, flutter margin down, take-off mass and battery follow).

**Priority.** For output i, first-order variance shares: `S_ij = (dy_i/dx_j * sigma_j)^2 / sum_k (...)^2`, the
fraction of the output's spread owed to each input. Across outputs, each output's spread is put in one currency
(take-off kg, or the margin it eats against its limit) so one ranked list exists. Value of retiring an input: the
drop in output spread (or in probability of missing the limit) if its `retire` activity happens, per week of
schedule. Inputs without a computed impact stay listed as blind spots, never zero.

## Computation

1. **One Jacobian per solve.** Post-optimal sensitivity of every output to every input from the KKT system
   (implicit function theorem): CasADi differentiates through `nlpsol`, so `dy*/dp` for all outputs and all
   parameters costs one factorisation, not 2N re-solves. The envelope-theorem `core/sensitivity.py` stays as the
   objective-only special case and a check.
2. **Fixed-aircraft mode** evaluates the performance functions at the frozen design (no re-solve): plain
   automatic differentiation.
3. **Inputs as Opti parameters.** Inputs the sizing currently bakes in as Python floats are lifted to
   `opti.parameter` so they are differentiable; done incrementally, unlifted inputs are blind spots.
4. **Checks.** First order and local: a handful of entries are checked against central-difference re-solves, and
   large-step claims (outside +-sigma, or an active-set change) are re-solved. Results record which was used.
5. **Non-optimizer analyses** (whirl flutter, FE wing strain, blade): finite differences or their own AD, ingested
   as rows of the same matrix with their source.

## Code

- `aircraft_closure/uncertainty/`: `UncertainInput`, the input registry (seeded from `HaloAssumptions`,
  calibration factors and the ledger gaps), output definitions.
- `aircraft_closure/core/sensitivity.py`: add `parametric_jacobian(opti, solution, outputs, parameters)`.
- `aircraft_closure/ledger/rank.py`: rank from the matrix (variance shares, levers, value of retirement); the current
  take-off-mass ranking is one column.
- Dashboard: input x output elasticity heat map; click a cell for the path; top drivers per output; the risk list
  (impact x uncertainty) against lock dates.

## Verification

- Analytic toy problem (Breguet range, a sized wing) with known elasticities, both modes.
- Halo: Jacobian entries vs central-difference re-solves (Scholz aero for speed), agree within a few per cent.
- Variance shares sum to 1; a zero-sigma input has zero share; a lever with zero elasticity ranks last.

## Order

1. Rebase and merge PR #16 (ledger) onto main.
2. Registry + `parametric_jacobian` + toy tests.
3. Halo: lift the first ~15 inputs to parameters, both modes, re-solve checks.
4. Ranking and dashboard; ingest whirl flutter, wing strain, bottom-up wing weight.

## Open

- Sigmas for most inputs are placeholders until owners set them; they are labelled as such, like the gap priors.
- Correlated inputs (e.g. cell energy density and cell-to-pack from one supplier) are independent at first.
