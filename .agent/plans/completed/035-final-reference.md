# Final Halo reference: real machine units, redundancy, drag corrections

Status: COMPLETED 2026-10-04. Consolidation plan; the user asked to stop
adding features ("let's start to wrap things up rather than add new
features").

User decisions on 2026-10-04:

- Motors: "best in class. no rubber motors."
- Redundancy: "redundancy as default but make it an option."
- Drag corrections (plan 034) as the default: "yes".

## Goal and scope

- **Integration** on one branch:
  - Tier 18 (redundancy, plan 032);
  - Tier 22 (design-space practice and multistart, plan 029);
  - plan 033 (machine database, gearbox stages).
  Tier 18's staged starts become two multistart candidates,
  "redundancy_off" and "strings_off".
- **Unit-built machines** (`machine_mass_model="units"`). Each machine is a
  whole number of units of a real product. Its limits are n × the unit's
  published ratings:
  - maximum torque and continuous power scale with n;
  - maximum speed is the unit's.
  The McDonald peak-efficiency point (design variables) only places the
  efficiency map. The inverter is added at 20 kW/kg unless the Tier 15
  layer models it.
  - **Motor unit:** Evolito D1500 2×3. 40 kg, 1,500 N·m peak, rated
    35 N·m/kg (1,400 N·m continuous), 2,500 rpm maximum. Continuous power
    at the 1,800 rpm peak-power speed: 264 kW.
  - **Generator unit:** Helix SPX242 demonstrator. 31.2 kg, 470 N·m peak,
    315 kW continuous, 17,000 rpm maximum.
  - **Integer counts.** First solve with the relaxed count, the smooth
    maximum of torque and power need. Round each count up, then re-solve
    with the counts fixed. These are two explicit solves; the relaxed
    design is the start of the integer one.
- **New defaults:**
  - `drag_corrections=True`;
  - `redundancy=True` (2 lanes, 2 buses, 2 strings, lane-, bus- and
    string-out hovers);
  - `machine_mass_model="units"`;
  - `gearbox_stages=True` (relaxed stage count).
- **Legacy sets** (`pre_plan035`): every named set pins those four
  settings to their plan 030 values. `requirements_plan030` /
  `assumptions_plan030` keep the 14,037 lb reference. Notebook shims and
  tier tests pin them too.

## References

Unit data are in `data/machines/aerospace_motors.csv` (plan 033), with
their sources:

- Evolito D1500: magneticsmag.com, newatlas.com;
- Helix SPX242: ehelix.com case study.

## Assumptions

- **Evolito continuous torque:** taken from its rated torque density,
  35 N·m/kg × 40 kg.
- **Continuous power:** at the 1,800 rpm peak-power speed.
- **Units stack** on one shaft: torque and power add, and the speed limit
  is unchanged.

## Interfaces

- **`UnitMachineMassModel`** (`powertrain/components/motor.py`): mass is
  count × unit mass (plus inverter); `count_units` is fixed, or None for
  relaxed.
- **`unit_machine`** (Halo): rebuilds a machine's limits from its units.
- **`HaloAssumptions`:**
  - the unit data fields;
  - `count_units_motor` and `count_units_generator` (None: chosen);
  - `integer_units`.
- **`HaloSizingResult.machine_units`:** `{machine: (count, fixed count)}`.
- **`solve_halo_sizing`** gains `stage_fallback` (plan 033) and keeps
  `staged_start` (plan 032).

## Symbolic considerations

- The relaxed count is a smooth maximum.
- The integer step is two explicit solves, not a loop.

## Tests

- `tests/integration/test_halo_plan035.py`:
  - the defaults;
  - closes at 900 kg, 16,231 ± 20 lb;
  - whole units on two lanes;
  - ratio ≤ 5.2 (single stage).
- Tier tests that reproduce earlier references pin `pre_plan035`.

## Acceptance

- The full suite passes.
- The tier notebooks pass, run one at a time (memory).

## Progress and decisions

- **Fast set** (plan 027, Scholz aero, thermal off, single lane):

| Machine model | Take-off mass | Rotor ratio |
|---|---|---|
| Rubber | 13,702 lb | 31.6:1 |
| Units | 14,833 lb | 3.5:1 |
| Units + stages | 14,598 lb | 3.9:1 |

  The units case uses 3 motor units and 3 generator units per machine.
- **Full defaults:** 7,362 kg (16,231 lb), cruise L/D 9.3.
  - Machines: 2 Evolito units per lane motor (8 in all); 3 Helix units per
    generator.
  - Gearbox: 5.0:1, one stage.
  - Masses: battery 550 kg, motors with inverters 426 kg, generators
    282 kg.
  - Binding: engine-out battery voltage, whirl flutter, rotor radius,
    hot-day heat exchanger, hover drive power, bus-out motor torque,
    stability.
- **First attempt:** tying the unit limits to the rubber ratios (max speed =
  2.5 × peak-efficiency speed) made the problem infeasible. The units'
  own ratings are the right limits.

## Deferred

- Unit catalogue choice (other products, mixed units), as an enumeration.
- Integer stage count on the full reference (the relaxed count gives
  5.0:1, i.e. one stage).
