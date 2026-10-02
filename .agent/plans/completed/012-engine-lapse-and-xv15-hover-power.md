# Turboshaft lapse, part-power efficiency, hover download and XV-15 power validation

Status: COMPLETED 2026-10-02. The user approved continuing after plan 011 (2026-10-02). The
user also set the Halo-class ceiling requirement to 13,000 ft; that belongs in
plan 013, but it makes engine lapse to at least 13 kft necessary here.

## Goal and scope

Tier 10b. The Tier 9 powertrain was optimistic in the ways that matter most
for a Halo-class aircraft:

- engine power never fell with altitude;
- engine efficiency was constant at any throttle;
- hover used a 0.8 rotor figure of merit and ignored wing download.

This plan adds each effect as an interchangeable submodel whose default
reproduces Tiers 1–9 exactly. Each is then checked against published XV-15
power data.

In scope:

- `SimpleTurboshaft`:
  - `lapse_exponent` (power available = rated x sigma^n);
  - `part_power_knockdown` (AeroSandbox / Geiss efficiency knockdown versus
    throttle);
  - optional explicit `mass_kg`;
  - `power_available_W(atmosphere)`;
  - `get_limits(atmosphere=None)`;
  - `evaluate(shaft_power_W, atmosphere=None)`.
- `operating_margins(..., atmosphere=None)`: turboshaft power is checked
  against power available at the point.
- `build_flight_point`: passes the point's atmosphere to the engine and the
  margins.
- `SimpleAerodynamics.download_fraction_hover`: hover thrust becomes
  T/W x W / (1 - f).
- `examples/xv15_performance.py`:
  - lapse fit to digitized power available;
  - figure-of-merit calibration at sea-level hover;
  - hover-weight prediction against altitude;
  - hover ceiling at 13,000 lb;
  - part-power fuel consumption check.

Out of scope: airplane-mode cruise power validation (needs an XV-15 drag polar
and propeller-mode rotor efficiency), ram recovery in forward flight, engine
decks and maps (Tier 11), and the Halo sizing itself (plan 013).

## References and digitized data

NASA TM X-62407 (1975):

- **Fig. 6.2.2, rotor shaft power available per engine** (helicopter mode,
  740 ft/s, standard day, take-off rating). Read from pixel crossings, about
  +/- 15 shp:

  | Altitude (ft) | 0 | 4,000 | 8,000 | 12,000 | 16,000 | 20,000 |
  |---|---|---|---|---|---|---|
  | Power (shp) | 1,374 | 1,285 | 1,181 | 1,061 | 942 | 788 |

  The sea-level value is 0.887 x the 1,550 shp engine rating (transmission
  and installation losses).
- **Fig. 5.1.1, OGE hover gross weight at take-off power,** twin engine,
  standard day, UT/W = 1.0, read from pixel crossings, about +/- 60 lb:

  | Altitude (ft) | 229 | 2,214 | 4,198 | 6,183 | 8,168 | 10,153 |
  |---|---|---|---|---|---|---|
  | Weight (lb) | 15,578 | 14,897 | 14,225 | 13,554 | 12,882 | 12,211 |

  The 20,000 ft endpoint is 8,850 lb. Sec. 5.1 states 7 % wing download (flaps
  deflected) and transmission efficiency 0.93.
- **Sec. 6.2, specific fuel consumption** (lb/hp/h) at four ratings:

  | Rating | Contingency | Take-off | Military | Normal |
  |---|---|---|---|---|
  | Power (shp) | 1,802 | 1,550 | 1,401 | 1,250 |
  | sfc | 0.564 | 0.584 | 0.601 | 0.622 |

- **SP-4517:** OGE hover ceiling 8,650 ft (rating not stated).
- **AeroSandbox `library.power_turboshaft.thermal_efficiency_turboshaft`:**
  part-power knockdown from Geiss (2020), used as a ratio so the engine's own
  full-power efficiency stays an explicit input.

## Assumptions

1. **Lapse model:** a single exponent on the density ratio, fitted by least
   squares on ln(P/P0) = n ln(sigma). Residuals are reported. The real curve
   is flatter low down and steeper higher up; one exponent is the simplest
   useful model, and Tier 11 decks can replace it.
2. **Throttle** is shaft power over power available at the point. The
   knockdown's 100 % point is the engine's maximum rating; for the XV-15
   that is contingency.
3. **Hover:** actuator disk with a constant figure of merit (equal to
   `coefficient_of_performance` at zero airspeed). The figure of merit is
   calibrated at sea level only. Altitude points are predictions, not fits.
4. **Download** is a fraction of rotor thrust lost to the airframe in hover
   only.

## Interfaces

New fields default to the current behaviour: lapse exponent 0, knockdown off,
`mass_kg` None (specific-power mass), download 0. `get_limits`, `evaluate` and
`operating_margins` accept an optional atmosphere and treat None as sea level.
Design margins stay on sea-level ratings.

## Symbolic considerations

sigma^n and the cubic knockdown are smooth expressions. The throttle ratio is
symbolic. There is no branching on values: the knockdown flag is a Python bool
set at construction, not a symbolic quantity. The hover ceiling is one
explicit Opti with altitude as the variable.

## Tests

- Lapse:
  - exponent 0 recovers rated power;
  - sigma^n identity;
  - power falls with altitude;
  - symbolic altitude.
- Knockdown:
  - equals AeroSandbox's ratio;
  - 1 at full throttle;
  - fuel flow per watt rises at part power;
  - off by default.
- Margins: the turboshaft margin uses power available at altitude, and
  defaults are unchanged.
- Flight point: hover thrust scales by 1 / (1 - download); engine margin
  shrinks at altitude when lapse is on.
- XV-15:
  - the fitted exponent reproduces the digitized curve within 6 %;
  - the calibrated figure of merit reproduces the sea-level hover weight;
  - predicted hover weights are within 5 % at every digitized altitude;
  - the ceiling at 13,000 lb lies between the take-off line (about 7,800 ft)
    and SP-4517's 8,650 ft, within 1,000 ft;
  - the knockdown matches the sfc ratios within 2 %.

## Implementation sequence

1. Turboshaft submodels, then margins, the flight point and aerodynamics,
   with tests.
2. The XV-15 performance example, its tests and the notebook.
3. Docs, then commit.

## Acceptance

All tests and notebooks pass. The Tier 10b notebook shows:

- lapse fit residuals;
- hover weight versus altitude, model against the digitized figure;
- the calibrated figure of merit and its implication for Tier 9 (0.8, no
  download);
- the part-power fuel consumption check.

## Progress and decisions

- 2026-10-02: Plan written, data digitized.
- 2026-10-02: Implemented.
  - Lapse: n = 0.797, within 6 % to 20 kft (the fit is 3-4 % low between
    4 and 12 kft).
  - Hover: figure of merit 0.670 calibrated at sea level. Predicted hover
    weights are within 2 % to 10 kft and 3 % at 20 kft.
  - Ceiling at 13,000 lb: 7,141 ft.
  - Part-power sfc: within 0.7 % at all four ratings.
  - Tier 9 hover power was 33 % low.
  - 217 tests and 10 notebook checks pass.
- Decision: the battery-only engine-out reserve and the series-hybrid flight
  point keep the defaults until plan 013 chooses the Halo-class figure of
  merit, download, lapse and knockdown.
- 2026-10-02: At the user's request, a progress dashboard artifact was
  published. It shows constraint and disk-loading diagrams, the mission,
  drag polar, weights, component maps and XV-15 checks. Its data generator
  lives in the session scratchpad, so `CoupledSizingResult` now also exposes
  powertrain instance masses and per-segment motor operating points.

## Deferred

Cruise power validation (needs the XV-15 drag polar), ram recovery,
temperature (95 F) lapse, part-power data beyond the four ratings, and
plan 013.
