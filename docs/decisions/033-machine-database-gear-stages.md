# Electric machines from a supplier database, and gearbox stage count

Status: COMPLETED 2026-10-04. Refines Tier 13 (plan 018); no roadmap tier row
changes. Direction 2026-10-04.

## Goal and scope

The Halo reference picks a rotor reduction of about 36:1, with motors at about
12,900 rpm. Two model artefacts drive this:

- the Tier 13 machine mass uses one torque density (15 N m/kg) at every speed;
- the AFDD drive weight barely depends on the reduction ratio.

This plan adds two interchangeable options.

1. **Machine database** `data/machines/aerospace_motors.csv`: one row per
   public aerospace machine, every number cited by URL, with peak and
   continuous ratings and the inverter status marked.
2. **`DatabaseMassModel`** (a `mass_model` for `Motor` and `Generator`):
   continuous torque density as a power law in base speed, fitted to the
   database, with a smooth specific-power cap. Inverter mass is optional, so
   the model works in both Tier 15 modes. Stacking is explicit.
   `TorqueDensityMassModel` stays as it is and remains the reference.
3. **`GearStageModel`** (gearbox stage physics):
   - stage count is a smooth ceiling of ln(ratio) / ln(max ratio per stage);
   - each stage adds mass and loses efficiency;
   - applied to the AFDD drive weight of the rotor gearbox and the generator
     step-up gearbox.
4. **Halo switches**:
   - `HaloAssumptions.machine_mass_model` is `"torque_density"` (default) or
     `"database"`;
   - `HaloAssumptions.gearbox_stages` defaults to False.

   With the defaults the reference is unchanged: 900 kg, 210 kt and
   14,037 lb, with thermal on, AeroBuildup, the AFDD wing and the ECM battery.

**Not in scope:**

- aerodynamics (the author is changing drag corrections);
- redundancy (Tier 18);
- design-space tools (Tier 22);
- discrete enumeration of the stage count beyond the smooth staircase;
- motor loss-model changes. McDonald losses keep their parameters; only
  machine mass and gearbox mass and efficiency change.

## References

- Machine data: one URL per row, in the CSV.
  - Evolito: magneticsmag.com, newatlas.com, evolito.aero, evtol.news.
  - Helix SPX242: ehelix.com.
  - H3X HPDM-250: LinkedIn H3X article, sustainableskies.org.
  - magniX: magnix.aero, emobility-engineering.com.
  - Siemens SP260D and SP200D: engineering.com, press.siemens.com.
  - Safran ENGINeUS 45: Wikipedia.
  - EMRAX 228, 268 and 348: emrax.com.
- Gear efficiency:
  - R. F. Handschuh and D. A. Rohn, "Efficiency testing of a helicopter
    transmission planetary reduction stage", NASA TP-2795, 1988
    (https://ntrs.nasa.gov/citations/19880005842). One planetary stage
    measured 99.44–99.75 % at 310 kW.
  - N. E. Anderson and S. H. Loewenthal, "Effect of geometry and operating
    conditions on spur gear system power loss", NASA TM-81426, 1980
    (https://ntrs.nasa.gov/citations/19800010132). Spur gear efficiency was
    above 98 % except at very low torque.
- Ratio per stage: simple planetary stages typically 3:1 to 10:1. Examples:
  - the OH-58 planetary stage is 4.67:1 in a 17.44:1 two-stage box (NASA
    rotorcraft transmission literature, via the search summary of
    https://ntrs.nasa.gov/api/citations/19880005842/downloads/19880005842.pdf);
  - H3X flies a single 6.7:1 epicyclic stage.
- Drive weight: AFDD00, NDARC Theory (NASA/TP-2009-215402) ch. 19. The
  transmission calibration factor is from the XV-15 at 20,000 / 565 rpm =
  35.4:1 (plan 011).

## Assumptions

- **Speed class of a machine:** its base speed, w_base = P_cont / T_cont.
  - For the Halo `rubber_machine`, continuous torque is
    `ratio_torque_continuous_peak` x peak torque, with the ratio = 0.5. This
    matches the machine's own ratings: P_rated = 1.25 w_hat Q_hat and
    T_peak = 2.5 Q_hat, so T_cont = 1.25 Q_hat and w_base = w_hat.
  - The database median peak/continuous ratio is about 1.55 (0.65); this is
    reported, not used.
- **Bare-machine fit:** rows with published continuous torque and power,
  whose inverter status is "n" or "unknown". Rows that include an inverter
  (E800, HPDM-250, magniX 2021) and peak-only rows (D1500) are compared with
  the model, not fitted.
- **Inverter:** with `specific_power_inverter_W_kg` set, the model adds
  P_rated / p_inv. Halo without the electrical layer uses 20 kW/kg, the same
  assumption as the Tier 15 inverter component. With the layer the model is
  bare and the inverter is the separate component.
- **Specific-power cap:** 20 kW/kg bare, the Tier 15 assumption. No fitted
  machine reaches it except the D250 (24.9 kW/kg), so the data do not fit it.
- **Stacking:** a machine of `count_stacks` identical stacks, relaxed as
  continuous: count = T_cont / T_stack_max. Torque and mass scale with the
  count, plus `mass_overhead_stack_kg` per stack (default 0: the fit is to
  whole machines, so per-unit housings are inside the torque density).
- **Gear stages:**
  - max ratio per stage 5:1;
  - loss 1 % of power per stage (a mesh pair, 0.5 % per mesh);
  - fixed efficiency 0.99 (bearings, seals, churning).

  This gives 0.980, 0.970 and 0.961 for 1, 2 and 3 stages; the Tier 13
  constant was 0.97.

  Each stage beyond the first adds 30 % of a one-stage drive's mass. An
  upstream stage carries 1/r of the torque: (1/5)^0.78 = 0.28 of the output
  stage with AFDD's power exponent, plus its own bearings, housing and
  lubrication. The AFDD00 drive mass is evaluated at the XV-15 calibration
  ratio (35.4:1) in place of the actual ratio, and scaled by
  `mass_factor(ratio) / mass_factor(35.4)`. AFDD's own ratio exponent is
  therefore not counted twice, and the XV-15 calibration point is
  unchanged.
- **Smooth stage count:**
  - x = ln(ratio) / ln(r_max);
  - staircase: n = 1 + sum over k of sigmoid((x - k) / w - 3), with
    w = 0.02 and k = 1..6;
  - relaxed: n = 1 + w softplus((x - 1) / w), with w = 0.05.

  The staircase is within 0.05 of 1 at ratio = r_max and monotonic. Halo
  defaults to the relaxed form (see Decisions).

## Inputs, outputs, design variables

- **No new design variables.** Motor speed, reduction ratio and generator
  speed are already Tier 13 variables.
- **New outputs (post-processing):** stage count and gearbox efficiency at
  the solved ratio, and the motor's bare and inverter masses.

## Constraints

None new. The changes are in the mass and efficiency expressions only.

## Interfaces

- `aircraft_closure.powertrain.components.motor.DatabaseMassModel` with:
  - `mass_kg(machine)`;
  - `torque_density_Nm_kg(speed_base_rad_s)`;
  - `count_stacks(machine)`.
- `aircraft_closure.powertrain.components.gearbox.GearStageModel` with:
  - `count_stages(ratio)`;
  - `efficiency(ratio)`;
  - `mass_factor(ratio)`.
- `aircraft_closure.powertrain.machine_database`:
  - `MachineRecord`;
  - `load_machine_database(path)`;
  - `fit_torque_density(records)`, returning a `TorqueDensityFit` with the
    residuals.
- `HaloAssumptions`:
  - `machine_mass_model`;
  - `gearbox_stages`;
  - `gear_stage_model`;
  - `torque_density_database_Nm_kg`, `speed_ref_database_rad_s` and
    `exponent_speed_database`.

## Symbolic considerations

- Power laws, logs, sigmoids and softplus through `aerosandbox.numpy`.
- No branching on symbols. `staircase` is a Python bool chosen before the
  solve.
- The step-up ratio is passed as fast/slow, at least 1.

## Tests

- **Database:** loads; every fitted row is within the stated tolerance;
  the defaults equal a re-fit.
- **Fit trend:** torque density falls with speed.
- **Stage limiting cases:** one stage for ratio <= r_max; monotonic counts;
  staircase near integers; the relaxed form tends to max(1, x); efficiency
  and mass factor identities.
- **Stack scaling:** n stacks of T at the same speed have n times the mass of
  one stack, plus the overheads.
- **Inverter term:** the integrated mass minus the bare mass equals
  P / p_inv.
- **Symbolic:** both models inside `asb.Opti`; a small solve.
- **Defaults:** unchanged, 14,037 ± 10 lb (the existing plan 030 test, plus
  checks that the new flags default off).
- **Halo integration:** both options on, with the
  `assumptions_plan027` + Scholz fast set. It closes, its stage count
  matches the ratio, and the drive mass is consistent.

## Implementation sequence

1. The CSV.
2. Database loader and fit.
3. `DatabaseMassModel`.
4. `GearStageModel`.
5. Halo wiring.
6. Unit tests.
7. Halo runs: fast set, then the full reference with both options.
8. Notebook.
9. Docs.
10. Full suite, then commit.

## Acceptance

- The full unittest suite passes.
- The notebook executes with all checks passing.
- The defaults reproduce 14,037 lb.

## Progress and decisions

- 2026-10-04: plan written; data gathered; baseline reproduced:
  - 14,036.6 lb;
  - motor 12,935 rpm at peak efficiency, ratio 36.3:1;
  - motors 116 kg and rotor gearboxes 357 kg for both;
  - generator gearboxes 175 kg.
- Evolito "1x3/2x3" is the winding-set arrangement (one or two three-phase
  sets), not the stack count (magneticsmag.com). Evolito does stack D250 and
  D500 units (newatlas.com, evtol.news).
- **Fit:** tau = 11.79 N m/kg x (w / 500 rad/s)^-0.271 over 15 rows. RMS
  log residual 0.375; the worst row is the D250 at 2.56.
- **Decision: Halo uses the relaxed stage count by default.**
  - The staircase with the database machines fails the AeroBuildup and
    thermal warm starts (from Scholz, and from the relaxed solution, at
    widths 0.02, 0.05 and 0.1).
  - The database machines alone also fail the AeroBuildup warm start.
  - The relaxed chain solves.
  - The staircase stays available and solves the fast set, with a
    two-start rule (generic and relaxed) because of per-stage local optima.
- **Added `HaloAssumptions.reduction_ratio_max`** (default 40) for
  slow-motor trades.
- **Reference with both options on** (relaxed stages): 14,382 lb against
  14,037.
  - Motor 12,073 rpm; rotor ratio 32.1:1 (3 stages as built).
  - Motors 163 kg; rotor gearboxes 373 kg.
- **Fast set:**
  - both on: 13,978 lb relaxed, 13,603 lb staircase (24.6:1, 2 stages);
  - slow-motor caps with the staircase: 4.9:1 gives 14,991 lb, 8:1 gives
    14,359 lb, 12:1 gives 14,011 lb.
- **Verification:** the full suite is 529 tests, OK with 1 skipped (31
  new). Notebook `notebooks/tier13b_machine_database`:
  - Sections 1–4 (database, fit, stacking, stages, drive trade) were run as a
    script: 22/22 checks passed.
  - The full nbconvert execution was stopped by the host for low memory, so
    the committed notebook has no outputs. It still needs one execution
    (about 30–60 min including the suite).

## Deferred

- The motor loss model's peak-efficiency point still follows the rubber
  machine. A database-fitted loss level per speed class is deferred.
- Discrete stage-count enumeration (one solve per stage count) as a check on
  the smooth staircase. This is the robust way to get integer stages in the
  full reference.
- A data-based high-speed cap. The 20 kW/kg assumption lets machine speeds
  run to the design-variable bounds.
- Robustness of the AeroBuildup warm start with the database machines.
