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

- Study solves keep the reference machines: whole units fixed at the v3.6 counts (2 Evolito units per motor lane,
  3 Helix units per generator; `count_units_motor=2`, `count_units_generator=3`). That means one solve per case,
  and the mass optimum reproduces the reference exactly (15,179 lb, 70.0 kWh). A larger aircraft (payload, range)
  must then fit the same machines, which is part of what limits it. A first pass with relaxed unit counts is kept
  as a finding (below).
- Initial guess: the cached v3.6 baseline (`halo-closure-system/output/ledger/baseline.pkl`, exported as a design
  dictionary), then the mass optimum for every other case.
- The sweep fixes the parallel count at 1.25, 1.5, 1.75 and 2.0 times the reference (87.5-140 kWh). Points below
  the wall are not solved: at every free optimum the engine-out end-voltage margin binds, so a smaller pack is
  infeasible at that weight by construction.
- Fuel reserve: unchanged (`fuel_factor` on the burnt fuel gives the carried fuel); the fuel objective minimises
  the burnt fuel. SOC: the mission still starts full and ends at or above the 0.30 floor that the engine-out hover
  starts from.
- Cell specific energy sensitivity: `factor_capacity_ageing_battery` 0.8 -> 1.0, i.e. +25 % capacity at the same
  cell mass and resistance (a higher-energy cell of the same power capability; an approximation).
- Battery price sensitivity: USD 500 -> 1,000 per kWh on the cost objective. All other `CostModel` prices are as
  labelled in `cost.py`: airframe USD 1,500/kg, Jet-A USD 0.90/kg, grid USD 0.15/kWh, 1,000 cycles, USD 500 per
  flight hour.

## Results (2026-10-10, `examples/halo_battery_objectives.py`, 15 solves, all converged)

### One solve per objective, pack free

| Objective | Battery (kWh) | Take-off weight (lb) | Fuel burnt (kg) | Cost / mission (USD) | Payload (kg) | Range (nm) | Cruise (kt) |
|---|---|---|---|---|---|---|---|
| Min. take-off weight (v3.6) | 70.0 | 15,179 | 851 | 3,309 | 900 | 445 | 166 |
| Min. cost per mission | 82.7 | 15,662 | 890 | **3,082** | 900 | 445 | 210 |
| Min. fuel | 82.9 | 16,139 | **806** | 3,347 | 900 | 445 | 170 |
| Max. payload | 98.0 | 18,163 | 936 | 3,526 | **1,359** | 445 | 164 |
| Max. range | 98.0 | 18,121 | 1,360 | 4,618 | 900 | **695** | 169 |

In every case the engine-out hover end voltage (cells at 2.5 V at the end of the 60 s) is a binding constraint.
**Each optimum sits on its own engine-out wall**, and the wall moves with the weight the objective chooses
(about 4.6-5.4 Wh per lb of take-off weight). No objective buys battery above its wall.

Battery share of motor power by segment (motor-energy weighted; negative means the generators recharge the pack).
The first column is the 4,000 ft OGE hover requirement point, the hover that sets how much the fixed engines must
be helped:

| Objective | 4,000 ft hover | Take-off hover | Climb | Cruise | Loiter | Descent | Landing hover |
|---|---|---|---|---|---|---|---|
| Min. take-off weight | 36 % | 7 % | 8 % | 1 % | -2 % | 8 % | 11 % |
| Min. cost | 41 % | 0 % | 0 % | 0 % | 0 % | 0 % | 7 % |
| Min. fuel | 42 % | 14 % | 12 % | 0 % | -15 % | 100 % | 15 % |
| Max. payload | 46 % | 4 % | 0 % | 2 % | -1 % | 5 % | 14 % |
| Max. range | 45 % | 4 % | 0 % | 2 % | -6 % | 6 % | 13 % |

The mission columns are weakly determined: the take-off-weight objective is nearly indifferent to how the carried
pack is used in flight, so the split moves between nearby solutions. The hover and engine-out columns are not.

### Battery energy sweep (pack fixed)

| Battery (kWh) | 70.0 (wall) | 82.7 (cost wall) | 87.5 | 105 | 122.5 | 140 |
|---|---|---|---|---|---|---|
| Min. take-off weight: weight (lb) | 15,179 | - | 15,358 | 15,752 | 16,250 | 16,802 |
| Min. cost: cost per mission (USD) | - | 3,082 | 3,083 | 3,106 | 3,145 | 3,201 |
| Min. cost: weight (lb) | - | 15,662 | 15,689 | 16,025 | 16,597 | 17,109 |

Above the wall every kWh costs about 23 lb of take-off weight (6.8 kg/kWh of pack, times the growth factor) and
about USD 2.1 per mission. Most of that is airframe capital and fuel for the heavier aircraft; the pack's own price
is about USD 0.13 per kWh per mission. Doubling the pack raises the 4,000 ft hover battery share from 36 % to 59 %
and the take-off hover share from 7 % to 33 %. The extra pack is used, but it buys nothing the objective values.
The cost along the mass sweep is not plotted: the mass objective is nearly indifferent to cruise speed, so its
cost scatters with the speed it happens to pick.

### Sensitivities

- **Battery price x 2** (USD 1,000/kWh, cost objective): 81.4 kWh (-1.3 kWh), cost USD 3,093 (+11, +0.4 %). The
  pack stays on the wall. The price hardly matters because the pack's capital per mission is small next to the
  time (USD 1,283), capital (USD 986) and fuel (USD 801) terms.
- **Cell capacity +25 %** (same mass and resistance, mass objective): 83.6 kWh in 4.5 % fewer cells, 15,087 lb
  (-92 lb, -0.6 %). The binding limit moves from the end voltage to the cell discharge current (10C). The pack is
  power-limited, so more Wh/kg is worth little; a lower-resistance or higher-C cell is what would shrink it.

### Finding: whole machine units cost about 880 lb

A first pass with relaxed (continuous) machine unit counts gave 14,300 lb and 61.9 kWh at minimum take-off weight.
The motors need 1.61 units per lane and the generators 2.004 units, against the reference 2 and 3 whole units: the
generator is 0.2 % over two units and so rounds up to three. Whole units cost about 880 lb at the reference
(motors 426 against 342 kg, generators 282 against 188 kg, plus growth, including a larger battery because the
wall moves with the weight). Two consequences follow. A 0.2 % reduction in generator demand (gear ratio, cooling,
a slightly higher-torque unit) would drop one generator unit per engine. And the motors' 24 % unit margin is what
lets the payload and range optima grow by 3,000 lb without new motors.

### Engineering explanation

- **The minimum is a safety wall, not an energy need.** With two fixed 1,120 hp turboshafts, the engine-out hover
  (one turbogenerator, 60 s at MTOM from the 0.30 SOC floor) needs the battery to supply the missing engine's
  power. The pack's internal resistance makes the cell voltage sag under that current, and by the end of the 60 s
  it reaches 2.5 V. The pack size is therefore set by power at low SOC (enough cells in parallel to share the
  current), so it scales with the aircraft weight, not with the mission. Every objective ends on that wall.
- **Where battery pays: hover peak shaving.** The battery carries 36-46 % of the 4,000 ft hover power at every
  optimum, which is what lets fixed engines lift 15,000-18,000 lb. Payload and range are limited by the same set
  of hover-power constraints: take-off hover, climb, hot-day and engine-out hovers on the fixed turboshafts, and
  motor torque on the fixed units. So both end near 18,150 lb with 98 kWh: +459 kg payload (+51 %) or +250 nm
  (+56 %).
- **Where it does not pay: cruise.** The pack weighs 6.8 kg/kWh, against fuel at about 0.3 kg per shaft kWh, so
  storing cruise energy in the pack is 20 times heavier. In cruise the optimizer runs the turbines; at most it
  spends what the engine-out reserve leaves (0-2 % of cruise power), or it recharges.
- **How the answer shifts with the objective.**
  - *Weight*: the smallest pack (70 kWh).
  - *Cost*: fly at the 210 kt limit, because time at USD 500/h dominates (-USD 255). Accept +480 lb, and do not
    cycle the pack in flight: its throughput costs about USD 0.67/kWh with wear and grid power, against turbine
    energy at about USD 0.3/kWh. The pack grows only because the aircraft does (82.7 kWh).
  - *Fuel*: -45 kg (-5.3 %), from a larger wing (L/D 10.4 against 9.1) and from load levelling. The pack flies the
    whole descent (the turbines idle at 17 kW, where their part-power fuel burn is worst) and the generators
    recharge it in the loiter, ending at the 0.30 floor. Again it sits on the wall (82.9 kWh at 16,139 lb).
  - *Payload and range*: the heaviest aircraft the fixed engines can hover with the battery, at 98 kWh.
- **Battery price and specific energy** barely move the answer (see Sensitivities): the wall is a power wall, and
  the pack's price per mission is small.

## Slide summary

**Headline:** Whatever the Halo is optimised for, the battery ends at the engine-out safety wall. More battery
never pays; the objective only moves the wall by changing the weight.

- Minimum weight: 70 kWh, 15,179 lb. Minimum cost: 83 kWh, USD 3,082 per mission (-7 %, by flying at 210 kt).
  Minimum fuel: 83 kWh, 806 kg (-5 %). Maximum payload 1,359 kg or range 695 nm: 98 kWh, 18,150 lb.
- The battery's job is hover peak shaving: 36-46 % of the 4,000 ft hover power at every optimum, about 0-2 % of
  cruise power. Each kWh above the wall costs about 23 lb and USD 2 per mission.
- Battery price x 2: +USD 11 per mission. Cell capacity +25 %: -92 lb (the pack is power-limited, not
  energy-limited). Whole machine units cost about 880 lb (the generator needs 2.004 units, which rounds to 3).

Caveat: the mission split between battery and turbines is weakly determined (flat directions), and the cost rests
on labelled price assumptions. The wall itself (cell resistance and its ageing factor, the 60 s engine-out) is the
assumption that matters most.

## Progress log

- 2026-10-10: plan written.
- 2026-10-10: `"fuel"` and `"range"` objectives, `halo_objective`, `count_parallel_battery_fixed`,
  `hybridization_mean`; 9 tests in `tests/integration/test_halo_objectives.py` pass.
- 2026-10-10: study of 15 solves with the reference whole machine units; all converged (62-367 s each) and there
  are no failed cases. A first pass with relaxed unit counts was stopped after its mass optimum, because it did not
  reproduce the reference; that result is kept as the 880 lb finding. Figure
  `docs/figures/battery_energy_objectives.png` (`examples/halo_battery_objectives_figure.py`).
