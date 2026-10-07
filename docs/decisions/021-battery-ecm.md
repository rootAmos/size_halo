# Battery equivalent circuit with sag and ageing

Status: COMPLETED 2026-10-03. Tier 17 (roadmap "17 Battery with sag and
ageing"). Direction 2026-10-02:

> You have my permission to artificially scale the power output until it
> comes closer to our needs. The goal is to capture the shape of the
> discharge curve. You can make it more power dense as you need to.

The author also noticed that the dashboard's battery terminal-voltage curve was
"weird / linear". That is the constant-OCV `Battery`.

## Goal and scope

- **Component:** `EquivalentCircuitBattery`, a new class (`Battery` stays
  unchanged as the simplest model).
  - Pack: `count_series` x `count_parallel` cells, with `count_parallel`
    continuous in sizing.
  - OCV(SOC) per cell, from a smooth fit or a table.
  - R0 + two RC branches, R(SOC, T) from data, times an explicit
    power-density factor.
  - Terminal voltage; chemical vs terminal power; coulomb-counting SOC.
  - End-of-life capacity and resistance factors; cell temperature as an input.
- **Data:** digitize the Samsung INR21700-50G figures into `data/batteries/`.
- **Flight point:** the branch constraint V >= V*/2.
- **Mission:** sub-segments and RC-state propagation.
- **Halo sizing:** an option, with a power-density sweep.
- **Not in scope:** thermal model, cycle-life cost, charge-specific
  resistance, and the electrical layer (Tier 15).

## References

- S. Paudel, J. Zhang, B. Ayalew, R. Singh, "Systematic Characterization of
  Lithium-Ion Cells for Electric Mobility and Grid Storage: A Case Study on
  Samsung INR21700-50G", *Batteries* 2025, 11, 313, CC BY 4.0,
  https://doi.org/10.3390/batteries11080313.
  - Table 1: cell specification.
  - Fig. 7: OCV (raster).
  - Fig. 8: DCIR and pulse power (vector).
  - Fig. 12: 2-RC parameters (raster).
  - Eq. (12): P = Vmin (OCV - Vmin) / R with Vmin = 2.5 V.
- US DOE HPPC practice, as quoted by Paudel et al.

## Assumptions

- **OCV:** the 30 C curve, with no temperature term (Fig. 7: within about
  35 mV for 10–45 C above 30 % SOC).
- **Resistance:**
  - Time constants are fixed at 8 s and 43 s (Fig. 12, about 25 C).
  - R1 shares R0's temperature factor.
  - Discharge resistance is used for charging too.
- **Extrapolation:**
  - Below SOC 0.2 the resistance rise continues (conservative).
  - Above SOC 0.8 the high-SOC term is held (smooth `softmin`).
  - The OCV polynomial is not clamped; missions bound SOC.
- **Power-density factor F** (the author's decision): resistance / F, current
  rating x F, and no mass change.
  - Halo default F = 5, which gives a 10C continuous rating.
  - Implied cell specific power (BOL, 25 C, SOC 0.5) scales with F from
    these F = 1 values:

    | Measure | F = 1 | F = 5 |
    |---|---|---|
    | Matched load | 1.19 kW/kg | 6.0 kW/kg |
    | DOE 10 s pulse | 1.55 kW/kg | 7.7 kW/kg |
    | DOE steady | 1.05 kW/kg | 5.3 kW/kg |
    | Current rating | 0.51 kW/kg | 2.6 kW/kg |
- **Pack mass:** cells / pack = 0.7 (assumed; cylindrical-cell packs
  0.65–0.75).
- **End of life:** capacity 0.8 and resistance 1.5 (assumed); applied for
  sizing.
- **Charge rating:** 1C x F (assumed; half the discharge rating, the legacy
  convention).
- **Cell temperature:** 25 C (thermally managed pack).
- **Halo pack:** 210 series cells. That is 756 V nominal, with a
  525–882 V window inside the machines' 400–900 V.

## Interfaces

- **`components/battery_ecm.py`:**
  - `PolynomialOcvModel` and `TabulatedOcvModel` (interchangeable OCV
    submodels).
  - `ResistanceBranch` and `CellResistanceModel`, plus `LithiumIonCell`.
  - The factories `inr21700_50g_cell()`, `inr21700_50g_ocv_model()` and
    `inr21700_50g_resistance_model()`.
  - `EquivalentCircuitBattery`:
    - `get_mass()` and `get_limits()` (voltage window, discharge and charge
      current, SOC window).
    - `evaluate(current_A, soc, duration_s=0, voltage_rc_start_V=None,
      temperature_C=None)`. The result adds `voltage_open_circuit_V`,
      `voltage_rc_V`, `voltage_rc_end_V`, `voltage_end_V`,
      `voltage_driving_V`, `resistance_effective_ohm` and `power_max_W`.
- **`powertrain/cells.py` (data layer):**
  - CSV loaders and `fit_ocv_polynomial`.
  - `fit_resistance_model`, a least squares owned by an `asb.Opti`.
  - `resistance_dc_corrected_ohm`, `tabulated_ocv_model` and
    `pulse_power_W`.
- **`ports.py` and `compatibility.py`:**
  - An electrical OUT port that sets the voltage.
  - Envelope: cell window x `count_series`, with the rated power at nominal
    voltage.
  - Operating margins on discharge and charge current and on the voltage
    window.
- **`build_flight_point(..., duration_s=0.0, voltage_rc_start_V=None)`:** with
  the ECM, it adds V / V* >= 0.5.
- **`build_mission(..., subsegments=1, polarization_start="rest")`:**
  - `subsegments` is an int or one count per segment.
  - `SegmentResult.subsegments` holds the per-point results.
  - With the ECM the battery owns the SOC update, RC voltages propagate, and
    each point reports an end-of-interval voltage margin.
- **`HoverSegment.active_generator_count`:** default None, which leaves the
  behaviour unchanged.
- **Halo:**
  - `HaloAssumptions` gains `battery_model` (default "constant"),
    `factor_power_density_battery` (5), `count_series_battery` (210) and
    the end-of-life factors.
  - It also gains `temperature_cell_battery_C`,
    `fraction_mass_cells_battery`, `subsegments_mission` (1, 1, 4, 1, 1, 1)
    and `subsegments_engine_out` (3).
  - `HaloDesign.count_parallel_battery`.
  - `HaloSizingResult.battery_trace` and `engine_out_trace`.
  - `build_halo_battery`, and the named sets `assumptions_tier12b` and
    `assumptions_tier17`.

## Symbolic considerations

- **Inputs:** every expression accepts Opti variables: current,
  `count_parallel`, duration (cruise duration depends on a speed variable),
  SOC and temperature.
- **Branching:** the only Python branching is on numeric zero duration and on
  a None RC state.
- **No iteration:** the power balance is solved as an equality by IPOPT.
- **Root selection:** the branch constraint keeps solutions on the low
  root. From a high-root start IPOPT reports local infeasibility (restoration
  stalls at the power peak), so a high-root solution is never returned.
- **Smoothness:** OCV is a polynomial (cheap and smooth). The B-spline table
  is offered, but not used inside the sizing (plan 014 lesson).

## Tests

- **`tests/powertrain/test_battery_ecm.py`** (25 tests):
  - Data completeness and reference points: OCV(50 %, 30 C) = 3.72 V;
    30 C 2 s discharge DCIR = 23.5 mOhm.
  - Pulse-power cross-check between the two digitizations (rms 1.3 %).
  - OCV fit residuals and monotonicity.
  - Resistance refit reproduces the coefficients; rms per temperature.
  - Tabulated OCV: node recovery, interpolation, held ends,
    interchangeability.
  - Identities:
    - V = OCV - I R; chemical = OCV I; P_max = V*^2/4R; low and high root.
    - Pack scaling; F = 1 is the 50G; ageing.
    - RC limits and exact propagation; coulomb counting and signs.
  - Trends: curve shape, sag at low SOC and low temperature; the
    extrapolation policy.
  - Symbolic sized pack; numeric arrays; ports and margins.
- **`tests/mission/test_mission_ecm.py`** (10 tests):
  - Low-root flight point.
  - Branch exclusion of the high root.
  - Sub-segment labels.
  - Coulomb SOC chain.
  - Energy conservation against the exact OCV integral.
  - RC continuity; steady start; argument checks.
- **`tests/integration/test_halo_battery_ecm.py`** (5 tests):
  - The reference and Tier 12b are unchanged.
  - The ECM sizing closes.
  - The payload shortfall.
  - The low-SOC minimum-voltage case.
  - Mission voltage follows the OCV curve.

## Acceptance

- All 264 earlier tests pass unchanged, and 40 new tests pass (304 in all).
- The legacy sets reproduce 18,506 / 17,228 lb, and the reference
  14,875 lb.
- Notebook `notebooks/tier17_battery/battery_verification.ipynb` is
  executed: 21/21 checks and the full suite.

## Progress and decisions

- 2026-10-03: Data digitized.
  - Fig. 8 from the vector paths: 1,232 points. Vertices that MATLAB merged
    as collinear were restored exactly.
  - Fig. 7 and Fig. 12 by colour clusters.
- **Model form chosen.**
  - A plain Arrhenius fit failed: 45 C is no better than 30 C. The
    quadratic-in-(T_ref/T - 1) form fits with 4.7 % rms.
  - A free per-point RC split was ill-conditioned, so a single global fit
    is used.
  - The high-SOC exponential extrapolated R2 x 7.6 at SOC 0.95 (take-off), so
    it is now held above 0.8.
- **Decision: the reference keeps the constant `Battery`.**
  - The engine-out reserve fixes usable capacity (60 s at about 590 kW
    within SOC 0.30–0.10), whatever the power density.
  - With 147 Wh/kg (end of life, pack) against the old 250 Wh/kg, 900 kg at
    210 kt does not close on the fixed engines.
  - The ECM is the option `assumptions_tier17`, run with
    `objective="payload"`.
- **Results** (payload objective, 210 kt, end of life, 25 C):

  | F | 1.5 | 2 | 3 | 5 | 8 | 12 | 20 |
  |---|---|---|---|---|---|---|---|
  | Max payload (kg) | none | 62 | 384 | 638 | 687 | 707 | 722 |

  - The binding constraint changes with F:
    - F 2–3: the current rating.
    - F 5: the cutoff voltage at the end of the engine-out hover (525 V,
      SOC 0.106).
    - F 8 and above: the reserve energy.
  - Sensitivities at F = 5: beginning of life 693 kg; 0 C cell 431 kg.
  - The F = 5 pack: 4,730 cells (22.5 parallel), 466 kg, 68.7 kWh at end of
    life, 834 kW rated.
- Warm starts matter. The beginning-of-life case closes from the reference,
  not from the end-of-life optimum. Multistart is in Tier 22.

## Deferred

- **Requirement decision (author):** payload or speed; the reserve definition
  (SOC window, duration, end of life); a cell trade.
- **Charge-specific resistance** and the 4.2 V charge limit at high SOC
  (the data exists in `inr21700_50g_dcir.csv`).
- **Electrical and thermal coupling:**
  - Cell temperature from a thermal model (Tier 19).
  - OCV temperature dependence.
- **Cycle-life cost** and the depth-of-discharge ageing model.
- **Wider use:**
  - Making the ECM the reference once requirements change.
  - Dashboard plots of `battery_trace`.
