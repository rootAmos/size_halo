# Design-space practice: starting points, freed trades, cost, enumeration, robustness

Status: COMPLETED 2026-10-04. Tier 22 (roadmap "22 Design-space practice",
review items 9 and 10). Follows plan 027 (AeroBuildup reference). Tiers 15
(electrical layer) and 19 (thermal, made the default by plan 030) landed on
main while this plan ran; they are out of scope except that the start list
covers their flags.

## Goal and scope

Mostly additive tooling around the existing Halo sizing solve
(`examples/halo_sizing.py`):

1. **Starting-point strategy.** Replace the hand-written rules in
   `solve_halo_sizing` (Scholz-aero start for AeroBuildup, constant-battery
   start for the equivalent-circuit pack, 85 % payload start if that fails)
   with one explicit, documented, tested strategy: an ordered list of
   candidate starts, each one independent coupled solve, returning the first
   success (or, on request, the best of all) with a record of every attempt.
2. **Freed trades.** Three fixed assumptions become design variables behind
   flags: wing aspect ratio, mission cruise altitude and the reserve SOC.
3. **Cost objective.** `objective="cost"`: an acquisition-plus-operating cost
   per mission, with sourced or labelled-assumption prices.
4. **Architecture enumeration** over discrete flags that already exist, as
   separate solves, with a results table.
5. **Robustness:** one-at-a-time sensitivity (tornado data) to uncertain
   inputs.

Not in scope: new physics; Tier 15 electrical layer (bus voltage as a real
trade); Tier 19 thermal; optimization under uncertainty (robust objective);
payload-range diagrams.

The sizing is unchanged. Studies use the plan 027 aircraft
(`assumptions_plan027`: 900 kg, 210 kt, AeroBuildup, AFDD wing,
equivalent-circuit battery, thermal model off, 13,639 lb); the plan 030
default (thermal on, 14,037 lb) is reached by the same strategy through its
`thermal_off` start.

## References

- Plans 022, 025, 026, 027 and `docs/IMPLEMENTATION_NOTES.md` (the fragile
  starting points).
- `docs/OPTIMIZATION_PHILOSOPHY.md` (Opti owns the solve; no hidden loops).
- Cost inputs (all labelled assumptions in `CostModel`; anchors only):
  BloombergNEF 2024 battery price survey (automotive pack average about
  USD 115/kWh); EIA US Gulf Coast jet fuel spot price (about USD
  2.0-2.5/gal in 2024); EIA average US commercial electricity price (about
  USD 0.13/kWh, 2024).

## Assumptions

- Each candidate start is one coupled solve of the target problem from an
  initial guess. A candidate may first solve a simpler precursor problem.
  The list is finite and ordered; nothing iterates to a fixed point.
- Cost: straight-line depreciation over a fixed number of missions, no
  interest or residual value; battery replaced at the rate its cycle and
  calendar life are consumed (additive); a lumped cost per flight hour for
  maintenance, remote crew and ground operations; ground electricity to
  restore the take-off SOC. All prices are assumptions (see `CostModel`).
- Sweeps (enumeration, sensitivity, freed trades) run with
  `aerodynamics_model="scholz"` (about 1 % in mass from AeroBuildup, plan
  025; seconds per solve instead of minutes). The notebook says so.

## Interfaces

- `solve_halo_sizing(...)`: signature unchanged. With `initial`, one solve
  (`solve_halo_sizing_once`); without, `solve_halo_sizing_multistart`.
- `solve_halo_sizing_multistart(requirements, assumptions, factors,
  verbose, max_iter, objective, starts=None, select="first", cache=None)`.
- `default_starts(requirements, assumptions, objective)`; start labels
  `caller`, `electrical_off`, `thermal_off`, `mass_objective`, `scholz_aero`,
  `constant_battery`, `payload_continuation`, `perturbed_low`,
  `perturbed_high`, `generic`. `solve_halo_sizing_once` is the single solve.
- `StartAttempt`, `StartRecord` (frozen); `HaloSizingResult.start` and
  `HaloSizingResult.cost` (new, defaulted fields).
- `HaloAssumptions`: `free_aspect_ratio_wing`, `free_altitude_cruise`,
  `free_soc_reserve` with bounds tuples; `cost_model`.
- `HaloDesign`: `aspect_ratio_wing`, `altitude_cruise_m`, `soc_reserve`
  (None when fixed).
- `src/aircraft_closure/mission/cost.py`: `CostModel.evaluate(...)` returns
  `CostBreakdown` (per-mission items, acquisition).
- `examples/halo_design_space.py`: `enumerate_architectures`,
  `sensitivity_study`, `UncertainInput`, `ArchitectureCase`,
  `SensitivityCase`, `SensitivityStudy.tornado()`.

## Symbolic considerations

- Cost items are plain arithmetic on symbolic masses, ratings, energies and
  durations. Battery discharge throughput uses `np.softmax(E, 0)` per
  mission point (charge is not throughput) with softness 0.1 % of capacity.
- Freed trades are `opti.variable`s with bounds; the cruise altitude flows
  into the mission segments and `asb.Atmosphere` symbolically.
- Start selection branches on Python strings and on numeric results after a
  solve, never on symbolic values.

## Tests

- `tests/mission/test_cost.py`: zero prices, closed-form items, battery-wear
  limits (infinite life; throughput independence of pack size), positivity
  and monotonicity of every input, symbolic solve.
- `tests/integration/test_halo_design_space.py`: start order; bad
  arguments; Scholz reference start record; 900 kg ECM (plan 026 sets) needs
  payload continuation and reproduces 14,247 lb; ECM max payload without
  `initial`; best-of-all-starts spread; perturbed start; freed trades close
  inside their bounds and are no heavier; cost objective closes and each
  optimum wins its own objective; enumeration (generator step-up on/off);
  sensitivity sign; the reference from cold records the Scholz-aero start
  at 13,639 lb.

## Acceptance

- Full unittest suite passes; existing callers and tests unchanged.
- Tier 22 notebook executed with all checks passing and the suite passing.
- Roadmap row 22 `Implemented`; implementation notes updated.

## Progress and decisions

- 2026-10-04: plan written.
- **Strategy:** the earlier rules become the first entries of an ordered
  list, so every existing reference reproduces (13,639, 14,247 lb). Perturbed
  starts scale the first precursor design. Precursors run the strategy
  without perturbation; a continuation precursor has no continuation.
  `initial=` on the multistart adds a `caller` start (sweeps, sensitivity).
- **Fragile cases solved from cold:** plan 026 900 kg ECM (continuation);
  ECM max payload (`perturbed_low`, 1,753 kg Scholz); Tier 15 AeroBuildup +
  electrical layer + ECM at 900 kg, the Tier 15 open issue
  (`perturbed_low`, 6,862 kg / 15,129 lb, after four failed starts; 2.5 h).
- **Local optima:** no spread found (Scholz: five starts, < 1e-4 kg;
  AeroBuildup: two converged starts, 0.000 kg; cost optimum: five starts).
- **Freed trades:** first choice was the battery series count; dropped
  because with per-cell limits the pack mass depends only on the cell count
  (the optimizer returned 179s x 21.7p with the same 3,887 cells and mass:
  a degenerate trade). Replaced by mission cruise altitude. The reserve SOC
  runs to its upper bound (0.90, -386 lb); first bounds 0.15-0.60 failed
  from the fixed design (NaN in the ECM at low SOC), 0.20-0.90 converge from
  the constant-battery start.
- **Cost:** objective scaled by USD 100 (USD 1,000 gave local
  infeasibility from the mass optimum); `mass_objective` start added. Cost
  optimum USD 2,939 vs 3,191 per mission at the mass optimum (AeroBuildup),
  at 14,176 vs 13,639 lb and 210 vs 163 kt.
- **Enumeration:** the planned generator step-up pair could not be the test
  case: on-shaft generators do not close at 900 kg (max payload 515 kg with
  ECM); the test enumerates the battery model instead.
- **Merges:** main (Tier 19, plan 030, Tier 15) merged twice; thermal-off and
  electrical-off starts added; tests and notebook pin `assumptions_plan027`.
- **Acceptance:** full suite 451 tests before the plan 030/Tier 15 merges,
  518 after (inside the notebook); notebook checks in the executed notebook.

## Deferred

- Bus voltage (series count) as a trade, once Tier 15 voltage-dependent
  masses make it non-degenerate.
- Optimization under uncertainty; cycle life against depth of discharge;
  a cost-mass Pareto front.
- Start order or per-start iteration cap for slow failing starts (the
  Tier 15 case spends 2.3 h in failing starts).
- Three enumeration combinations converge neither at 900 kg nor at maximum
  payload.
