# Fixed off-the-shelf turboshafts, battery-assisted hover, in-flight recharge

Status: COMPLETED 2026-10-03. Tier 12b, author direction 2026-10-03:

> turbine power required can never exceed turbo power available ... use
> [batteries] to supplement turbine power in hover, then recharge the
> batteries in cruise. Limit the turbine power to a fixed quantity. Consider
> the deck I gave you ... ~1100 HP. If you need to downsize the aircraft so
> be it.

## Goal and scope

- **Fixed engines:** each of the two turboshafts is fixed at the
  user-supplied deck's 1,120 hp SLS rating
  (`HaloAssumptions.power_rated_turboshaft_fixed_W`; None restores the sized
  engine).
  - Lapse is the XV-15 fit; the deck's own altitude scaling is unphysical
    (plan 014).
  - Power required never exceeds power available at the point (the existing
    turboshaft operating margin, now against a fixed rating).
- **Recharge:**
  - `build_flight_point(..., hybridization_electric_min)` and
    `build_mission(..., hybridization_electric_min)` allow a negative battery
    share, so the generators can recharge the battery, within its charge
    rating. The default of 0 keeps earlier tiers unchanged.
  - `build_mission` keeps SOC within the battery window (min_soc..max_soc)
    at every segment end.
- **Payload objective:** `solve_halo_sizing(objective="payload")` maximizes
  payload around the fixed engines. It produces the payload–speed trade.
- **Reference requirements:** max speed is reduced to what the fixed engines
  allow at 900 kg payload. Named legacy sets keep earlier tiers' notebooks
  reproducible.

## Findings driving the requirement change

With 2 x 1,120 hp, maximum payload is (continuation in speed, 445 nm,
13,000 ft ceiling, OGE hover 4,000 ft):

| Max speed (kt) | 190 | 210 | 215 | 220 | 225 |
|---|---|---|---|---|---|
| Max payload (kg) | 1,240 | 935 | 683 | 424 | 163 |

Payload reaches zero near 228 kt, so 250 kt is infeasible at any size.
Downsizing is limited because the fuselage is fixed at XV-15 dimensions; that
trade is deferred to Tier 22 (freed trades).

**Reference:** 900 kg at 210 kt gives 6,747 kg (14,875 lb). The battery
supplements both hovers (+27 %, +48 %), recharges in climb and descent, and
lands at the 0.30 floor.

## Tests

- The fixed rating holds (the turboshaft is not a variable).
- The turboshaft margin is non-negative at every point.
- The battery supplements take-off hover (share > 0) and recharges on at
  least one segment (share < 0).
- SOC stays within the window at every segment end.
- Payload objective: max payload at 210 kt is at least 900 kg (it is 935),
  and max payload falls as max speed rises.
- Legacy sets reproduce Tier 12 (17,228 lb) and Tier 11a (18,506 lb).

## Acceptance

- Tests and notebooks pass, including the new Tier 12b notebook.
- The payload–speed trade and the battery assist/recharge behaviour are
  shown, and the legacy baselines reproduce.
- The dashboard is updated: component-styled constraint lines, the trade
  chart, and legends that no longer overlap.

## Progress and decisions

- 2026-10-03: Plan written. Decision: max speed 210 kt is the reference,
  pending the author's choice (the payload–speed trade is reported). Speed and
  payload cannot both hold with fixed 1,120 hp engines.

## Deferred

- fuselage sized by payload (Tier 22);
- battery-assisted dash (sustained requirements stay on turbines alone);
- the deck's Mach ram effect;
- conversion of the engine's emergency rating.

- 2026-10-03: Implemented.
  - Found and fixed: SOC dipped to 0.20 in cruise, below the 0.30 floor the
    engine-out reserve assumes. `soc_floor` now holds 0.30 at every segment
    (`HaloAssumptions.soc_floor_every_segment`).
  - Cold starts are fragile for some payload cases; warm starts are used
    (multistart is in Tier 22).
  - Below about 205 kt, climb on turbines alone limits payload.
  - The dashboard constraint diagram now shows motor, generator and turbine
    power (solid, dashed, dotted) and the payload-speed trade; legends no
    longer overlap.
  - 264 tests pass.
