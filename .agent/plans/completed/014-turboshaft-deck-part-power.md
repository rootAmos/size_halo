# Turboshaft deck: part-power fuel curve

Status: COMPLETED 2026-10-02. On 2026-10-02 the user supplied `turboshaft_1120hp.csv` (a
GASP_TS-derived deck converted from `MAPS_1120hp.eng`) and approved option 1:
use the deck's part-power fuel curve normalised to maximum power, keep the
XV-15-validated lapse, and compare against the Geiss knockdown.

## Goal and scope

Tier 11a.

1. **A GASP turboshaft deck loader** (`powertrain/decks.py`): parses the CSV
   header metadata and the Mach x altitude x throttle grid.
2. **A derived part-power curve:** for each (Mach, altitude) row, sfc / sfc
   at maximum throttle against shaft power / maximum shaft power. The
   reported curve is the median across rows, with the minimum and maximum as
   the spread.
3. **Interchangeable part-power submodels for `SimpleTurboshaft`:**
   - `part_power_model = None`: constant efficiency (the default);
   - `GeissPartPowerModel()`: the current AeroSandbox knockdown;
   - `TabulatedPartPowerModel(power_fraction, efficiency_ratio)`: a smooth
     B-spline through tabulated points (AeroSandbox `InterpolatedModel`),
     holding the end values outside the table.

   These replace the boolean `part_power_knockdown` (interface change).
4. **A deck-derived table** embedded with provenance:
   `deck_1120hp_part_power_model()`.
5. **Comparison:** the deck against Geiss against the four XV-15 ratings.
   The Halo-class sizing is re-run with the deck curve.

Out of scope: using the deck's altitude or Mach dependence, or its tailpipe
thrust. The altitude columns are not physically monotonic after the usual
correction, as reported to the user:

| Altitude (ft) | 0 | 1,500–10,000 | 15,000 | 17,500 |
|---|---|---|---|---|
| Power at max (hp) | 1,120 | flat at ~900 | 953 | 612 |

The XV-15-fitted lapse stays.

## Data observations (2026-10-02)

- **Grid:** 13 Mach (0–0.6) x 10 altitudes (0–25,000 ft) x 16 throttles
  (T4 parameter 20–50; 50 = `t4max`), all filled.
- **Conventions:** power and fuel flow are "corrected". The sfc
  (fuel / power) does not depend on the correction convention, so the
  normalised curve does not either.
- **Sea-level static:** sfc 0.575 lb/hp/h at maximum (the T53 is 0.564).
- **Normalised curve (median):**

  | Power fraction | 0.3 | 0.5 | 0.7 | 0.9 |
  |---|---|---|---|---|
  | sfc ratio | 1.54 | 1.22 | 1.09 | 1.02 |

  Geiss gives 1.61, 1.23, 1.09, 1.03. The spread between rows is about
  +/-10–15 %.

## Interfaces

- `decks.py`:
  - `TurboshaftDeck` (SI arrays and metadata);
  - `load_gasp_turboshaft_deck(path)`;
  - `part_power_curve(deck, power_fractions)`, which returns median, min
    and max sfc ratios.
- `turboshaft.py`:
  - `GeissPartPowerModel`;
  - `TabulatedPartPowerModel`;
  - `deck_1120hp_part_power_model()`;
  - the `SimpleTurboshaft.part_power_model` field.

## Data and provenance

The raw deck is not committed: it is user-supplied and the repository is
pushed to GitHub. The derived 10-point table is embedded with its source and
date. A test re-derives it from the raw file when that file is present at
`data/engines/turboshaft_1120hp.csv`, and skips otherwise. Loader tests use a
small synthetic deck.

## Tests (map/deck rules)

- **Grid-node recovery:** the tabulated model returns the table values at
  the nodes.
- **Interpolation:** monotonic between nodes for a monotonic table.
- **Boundary and extrapolation policy:** constant beyond the ends; returns 1
  at a power fraction of 1.
- **Reference points:** the deck table matches the XV-15 sfc ratios within
  3 %.
- Geiss model equals the previous behaviour.
- Symbolic throttle.
- **Loader:** synthetic deck round trip and metadata.
- Halo sizing still solves with the deck model.

## Acceptance

Tests and notebooks pass, the docs record the comparison, and the dashboard
shows the deck curve.

## Progress and decisions

- 2026-10-02: Plan written.
- 2026-10-02: Implemented.
  - Decision: B-spline (`TabulatedPartPowerModel`) stalled the Halo IPOPT
    solve (3,000 iterations, about 300 s). The deck model is therefore a
    cubic fit (`CubicPartPowerModel`), within 1.1 % above 20 % power; the
    table model is kept for general use.
  - Interpolation rule: the table holds its end values outside the grid
    (`fill_value=None`); the cubic is smooth everywhere and exactly 1 at
    full power.
  - Halo sizing with the deck curve: 18,506 lb (Geiss: 18,740 lb).
  - The raw deck is local and gitignored at `data/engines/`.
  - 242 tests pass.

## Deferred

The deck's altitude and Mach behaviour (pending the original `.eng` or a
statement of its convention), tailpipe thrust, and engine re-scaling.
