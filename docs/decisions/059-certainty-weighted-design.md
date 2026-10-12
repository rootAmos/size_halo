# Plan 059: certainty-weighted design (impact x uncertainty = war-room priority)

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

## Results (2026-10-11, Halo reference, re-sized mode, 18 inputs, one solve)

One-solve table against re-solves (`examples/halo_parametric.py`, manifest `ledger/runs/20261011T124255_*`):

| entry | one solve | re-solve | ratio |
|---|---|---|---|
| d take-off mass / d cell-to-pack fraction | -1,808 kg | -1,810 kg (central, +-2 %) | 0.999 |
| d take-off mass / d aspect ratio | -258 kg | -250 kg (forward, +2 %) | 1.03 |
| d battery energy / d aspect ratio | -2.78e7 J | -2.70e7 J (forward, +2 %) | 1.03 |

Aspect ratio 2 % lower does not close (IPOPT: infeasible): the reference sits on binding limits (rotor clearance
to the span, engine-out power with the fixed engines), so its aspect-ratio derivative holds upward only. The
ledger should mark such one-sided inputs.

### Fixed aircraft (36f08aa): range per input at 80 % payload

The reference point for the fixed-aircraft payload and range ranking is **80 % of the design payload** (720 kg),
maximum range with full tanks (459 nm, take-off 6,705 kg; one solve, 113-208 s). At 90-100 % the off-design solve
does not converge (the full-payload corner has take-off mass, tanks and payload all at their limits); the trends do
not depend on the payload fraction, so 80 % is accepted as the reference (decision 2026-10-11).

Calibration factors (wing and fuselage weight factors, excrescence factor; kind "calibration") are recorded but not
ranked: they are always large levers because they stand in for design confidence.

Range drivers (impact x uncertainty, share of the range variance):

| input | share | range at 1 sigma |
|---|---|---|
| miscellaneous drag area (+-25 %) | 68 % | +-10 nm |
| payload (+-50 kg) | 10 % | +-4 nm |
| hover duration (+-15 s) | 7 % | +-3 nm |
| equipment mass (+-15 %) | 6.5 % | +-3 nm |
| inverter specific power | 4 % | +-2 nm |

Design levers (elasticity, % per %):

| input | range | cruise L/D |
|---|---|---|
| wing aspect ratio | +0.290 | +0.468 |
| wing t/c | -0.076 | -0.114 |

- Wing t/c reaches the aerodynamics only since 36f08aa: the thickness-scaled section family is now the default
  (identical to the NACA 2423 coordinates at 23 %), and the spar-cap depth follows t/c. Before, t/c read +0.005 on
  range because the coordinate airfoil ignored it and the cap depth cancelled.
- Aspect ratio holds in one direction only: the reference sits on binding limits (rotor clearance, engine-out power),
  so its derivatives are valid for an increase.
- Thinner and longer wings are flutter-limited: plan 040 (calibrated flutter speed 262 kt against 302 kt) and plan
  055 (flutter in the sizing loop) bound these levers.
- Every sigma except the wing factor's is a placeholder prior until its owner sets it.
- Not yet checked by re-solve in fixed mode (`--check` re-solves aspect ratio); the static-margin row is suspect
  (static margin sits at its limit yet shows non-zero derivatives).

### War room (2026-10-11)

`examples/halo_war_room.py` feeds results into the master ledger without re-solving, and the dashboard gains a
"What drives range" panel (ledger field `drivers`):

- Range and cruise L/D drivers of the fixed aircraft (above), calibration factors not ranked.
- Plan 055 (whirl flutter in the loop, PR #37): `gap.whirl_flutter` (+393 lb take-off on stiffness alone over the
  frequency-placement reference) and two priced trades: delta-3 (-11 deg saves 393 lb; spread 100 % of the gain
  until the flapping cost, limit 9 deg, is modelled) and the wing planform (AR 7.0, 28 %: -304 lb with delta-3 held).
  They rank first and second.
