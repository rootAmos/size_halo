# Plan 044: where the battery pays, by objective (cost, fuel, range, payload, mass)

## Goal and scope

The author's question: where is the battery best used, and how big should it be, depending on what is optimised
for (cost, fuel, range, payload)?

The Halo is a series hybrid with two fixed 1,120 hp turboshafts (plan 017, never a design variable) and the
equivalent-circuit 50G pack on the DC bus (plans 021/022) in two strings (plan 032). The v3.6 reference
(15,179 lb, 70 kWh) is sized by the engine-out hover: one turbogenerator plus the battery from the 0.30 SOC floor,
and the cells reach the 2.5 V cutoff in the last third of the 60 s (voltage sag, not energy). The optimizer already
chooses the battery share of the bus power at every mission point (`hybridization_electric`, free in
`[-1, 1]`, negative = in-flight charging).

In:

1. A `"fuel"` objective (minimise mission fuel burnt at the required payload) and a `"range"` objective (maximise
   the mission range at the required payload; the range becomes a variable, as the payload does for
   `"payload"`). One function, `halo_objective`, returns the quantity each objective minimises, so the choice is
   testable on a small `asb.Opti` problem.
2. `HaloAssumptions.count_parallel_battery_fixed` (None: a design variable, the default): a fixed pack size for
   battery-energy sweeps.
3. One solve per objective with the battery free; a battery-energy sweep (fixed pack) for the mass and cost
   objectives; one battery-price and one cell-specific-energy sensitivity.
4. The figure `docs/figures/battery_energy_objectives.png` and a slide summary.

Out: changing any default or reference result; restructuring the take-off-weight handling (another branch); a
fuel-price or mission-profile study; integer machine units in the study solves (see Assumptions).

## References

- Plan 017 (fixed engines, the battery assists in hover), 021/022 (equivalent-circuit 50G pack, the voltage wall),
  029 (design space, `CostModel`, starting points), 032 (redundancy: strings and failure hovers).
- `src/aircraft_closure/mission/cost.py` (price assumptions, all labelled there).

## Model

- `halo_objective(objective, mass_takeoff_kg, mass_payload_kg, mass_fuel_burnt_kg, range_m, cost_per_mission_usd)`
  in `examples/halo_sizing.py`: the scaled quantity `opti.minimize` receives. `"mass_takeoff"` / 1,000 kg,
  `"payload"` -payload / 100 kg, `"cost"` / USD 100 (as before), `"fuel"` fuel burnt / 100 kg, `"range"`
  -range / 100 km. `objective_value` gains the matching numeric values (`"range"` reads the new
  `HaloSizingResult.range_m`).
- `"range"`: `HaloRequirements.range_m` becomes an `asb.Opti` variable before the mission is built; the cruise
  distance is `range - climb and descent ground distance`, so cruise time (and fuel) follow the variable. Payload
  stays at 900 kg. Bounded the same way as the payload objective: the fixed engines (and the battery) must still
  lift the heavier aircraft in the hover, engine-out and failure cases.
- `"fuel"` and `"range"` start, like `"cost"`, from the mass optimum when no initial guess is given
  (`default_starts`).
- `count_parallel_battery_fixed`: the pack's parallel count is that number instead of a variable (energy and power
  follow from it as before).
- Per-segment hybridization for the report: `segments[i]["hybridization_mean"]`, the motor-power-and-time
  weighted battery share over the segment's sub-segments (the existing `hybridization` is the first
  sub-segment's, which misreads the four-point cruise).

## Verification

- `tests/integration/test_halo_objectives.py`: on a small `asb.Opti` problem (a toy hybrid with a battery-fuel
  split) each objective of `halo_objective` drives the optimum the expected way; `objective_value` on a numeric
  result for every objective; unknown objectives raise; `default_starts` gives fuel and range the mass start.
- The default sizing is unchanged: the new options default to the old behaviour (existing objective tests).

## Assumptions

- Study solves use relaxed (continuous) machine unit counts (`integer_units=False`): one solve per case instead
  of two, and smooth sweep curves without unit-count steps. The relaxed mass optimum is the study reference; its
  difference to the integer v3.6 reference is reported.
- Initial guess: the cached v3.6 baseline (`halo-closure-system/output/ledger/baseline.pkl`), then the relaxed
  mass optimum for the sweep.
- The sweep fixes the parallel count at multiples of the relaxed mass optimum's (the engine-out wall), up to
  about twice the reference energy. Points below the wall are not solved: at the mass optimum the engine-out
  end-voltage margin binds, so a smaller pack is infeasible by construction there.
- Cell specific energy sensitivity: `factor_capacity_ageing_battery` 0.8 -> 1.0, i.e. +25 % capacity at the same
  cell mass and resistance (a higher-energy cell of the same power capability; an approximation).
- Battery price sensitivity: USD 500 -> 1,000 per kWh on the cost objective.

## Progress log

- 2026-10-10: plan written.
