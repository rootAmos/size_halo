# Hot and high: ISA + delta-T atmosphere, temperature lapse, hot-day hover at destination

Status: COMPLETED 2026-10-03. Tier 16 of `docs/FIDELITY_ROADMAP.md` (review item 6).

## Goal and scope

- **Atmosphere:** flight conditions carry a temperature offset from ISA at
  their pressure altitude (`FlightCondition.temperature_offset_K`, default 0),
  passed to `asb.Atmosphere(altitude=..., temperature_deviation=...)` by the
  flight point and by the aerodynamics it calls. Requirements and mission
  segments can carry it.
- **Engine:** an interchangeable turboshaft lapse submodel in density and
  temperature, `DensityTemperatureLapse`: sigma^n (T / T_ISA(h))^-m. It is
  exactly sigma^n at zero offset. n is the Tier 10b fit (0.797); m is fitted
  to the XV-15 95 F take-off power curve (TM X-62407 fig. 6.2.2).
- **Validation:** predict (not fit) the XV-15 95 F OGE hover weights of
  fig. 5.1.2 with the Tier 10b calibrated figure of merit.
- **Requirement:** an OGE hover at the destination after the mission, at the
  mission's end mass and end SOC, at a hot/high condition (default 4,000 ft /
  95 F), as an option of the Halo sizing.

Not in scope: hot-day mission segments for the Halo reference, engine
temperature limits by rating, ground effect, battery temperature effects
(Tier 17).

## References

- NASA TM X-62407 (1975), XV-15 familiarization document: fig. 6.2.2 power
  available (standard day solid, T = 95 F dashed), figs. 5.1.1 / 5.1.2 OGE
  hover ceiling (standard day / 95 F), sec. 5.1 (7 % download).
- Tier 10b (plan 012): lapse sigma^0.797, figure of merit 0.67.
- AeroSandbox `Atmosphere(altitude, temperature_deviation)`: pressure follows
  the ISA at the pressure altitude, temperature is ISA + deviation, density
  from the gas law.

## Assumptions

- "T = 95 F" in the figures is an ambient temperature of 95 F at every
  pressure altitude, so the offset is 308.15 K - T_ISA(h).
- The take-off rating's temperature sensitivity applies to the Halo's
  1,120 hp deck engine (the deck has no hot-day data).
- The hot-day hover uses T/W 1.05 (as the standard-day hover requirement)
  for 60 s from the mission's end SOC, ending at or above the emergency
  floor (it is an alternative contingency to the engine-out reserve, not
  additional to it).

## Interfaces

- `FlightCondition.temperature_offset_K: Any = 0.0`.
- `SimpleAerodynamics` methods take keyword `temperature_offset_K=0.0`.
- `HoverRequirement`, `ClimbRequirement`, `SpeedRequirement`,
  `CeilingRequirement` and the mission segments gain
  `temperature_offset_K = 0.0` (last field, so positional use is unchanged).
- `turboshaft.DensityTemperatureLapse(lapse_exponent,
  lapse_exponent_temperature, source)` with `power_ratio(atmosphere)`;
  `SimpleTurboshaft.lapse_model = None` keeps sigma^lapse_exponent.
- `examples/xv15_hot_day.py`: digitized 95 F data, the fit, the hover
  prediction.
- `HaloRequirements`: `hover_hot_day` (bool), `altitude_hover_hot_m`,
  `temperature_hover_hot_K`, `thrust_to_weight_hover_hot`,
  `duration_hover_hot_s`. `HaloAssumptions.temperature_lapse` (bool).

## Symbolic considerations

The temperature factor uses `atmosphere.temperature()` and
`atmosphere.temperature_deviation`, both CasADi-compatible; the offset and
altitude may be Opti variables. No branching on symbolic values.

## Tests

- Identities: offset 0 reproduces sigma^n exactly; m = 0 reduces to the
  density lapse at any offset; the hot-day point has exactly the target
  temperature; pressure unchanged by the offset.
- Trends: hotter gives less power and less density, so more hover power.
- Fit: n + m from the paired 95 F / standard-day ratios; residuals.
- Prediction: 95 F hover weights within a stated band of fig. 5.1.2.
- Symbolic: lapse with Opti offset and altitude.
- Halo: solves with the requirement; the hot hover point's margins are
  satisfied; its minimum battery share exceeds the standard-day hover's;
  legacy sets reproduce 18,506 / 17,228 lb.

## Acceptance

All tests pass; the Tier 16 notebook executes with all checks passing; docs
updated; plan moved to completed.

## Progress and decisions

- 2026-10-03: plan written.
- 2026-10-03: implemented.
  - **Digitized** (200 dpi pixel analysis, gridline-calibrated, zero row
    checked against the axis line; the same procedure re-reads the Tier 10b
    standard-day data within 7 shp and 100 lb):
    - 95 F take-off power at 0–12,000 ft: 1,103 / 1,021 / 941 / 864 / 791 /
      720 / 647 shp (+/- 10);
    - 95 F twin-engine hover weights at 229–18,000 ft: 13,339 to 5,572 lb
      (+/- 100).
  - **Fit:** paired 95 F / standard ratios give n + m = 3.28, so m = 2.49
    (n = 0.797, Tier 10b). Ratio residuals are within 0.3 %. Absolute 95 F
    power is within 4.4 %, which is the Tier 10b lapse error.
  - **Prediction:** 95 F hover weights within -2.6 % to -1.5 % to 12,000 ft
    and +4.4 % to 18,000 ft (extrapolated lapse).
  - **Decision:** the hot-day hover is on by default (4k/95, T/W 1.05,
    60 s, end SOC down to the 0.10 floor).
    - It costs nothing at the reference: 6,747 kg against 6,748 kg without.
    - It exercises the battery: a turbine-limited battery share of 0.25,
      against 0.14 in the standard-day 4,000 ft hover at take-off mass.
    - It guards later tiers. For example, battery sag (Tier 17) will lower
      the power available at the 0.30 end SOC.
  - **Decision:** the density-temperature lapse is the Halo default
    (`HaloAssumptions.temperature_lapse`). It is identical on standard
    days.
  - **Legacy sets:**
    - `requirements_tier10c` disables the hot hover, so the 18,506 /
      17,228 lb baselines reproduce;
    - the new `requirements_tier12b` reproduces Tier 12b;
    - the Tier 12b unit tests and notebook source now use it, because the
      landing-hover battery share is a degenerate free split once the hot
      point is added. The notebook's outputs are unchanged.
  - **Finding:** the limit for hotter and higher destinations is rotor blade
    loading, not the battery.
    - The aircraft grows from 5,500 ft / 95 F, as the hot hover's
      CT/sigma = 0.14 at the Mach-limited tip speed forces more solidity.
    - The extra blade area costs cruise power against the fixed engines at
      210 kt.
    - No closed design is found from 6,000 ft / 95 F at 900 kg and 210 kt.
  - 285 tests pass (21 new); the Tier 16 notebook has 18 / 18 checks.

## Deferred

- Hot-day mission segments and a hot-day take-off (Halo reference flies ISA).
- Rating-specific temperature sensitivity and engine temperature/torque
  limits.
- Battery and motor thermal derating (Tier 17).
