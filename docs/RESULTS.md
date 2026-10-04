# Results and review guide

This page is for someone reviewing the repository from outside. It covers:

- what the framework is;
- how it is checked;
- what it concludes about a Halo-class unmanned series-hybrid tiltrotor;
- where the answers are still weak.

Every number below is reproducible from the code at this commit. All
inputs are illustrative public-domain engineering values. They are not
Archer or Halo data.

## 1. What this repository is

A **multidisciplinary sizing and closure framework** for hybrid-electric
VTOL aircraft, built on [AeroSandbox](https://github.com/peterdsharpe/AeroSandbox)
and CasADi.

- **One optimization problem.** Configuration, powertrain, aerodynamics,
  weights, stability, mission and energy management are expressed as
  equations in a single `asb.Opti` problem, solved at once by IPOPT. There
  are no hidden fixed-point loops: mass closure, power balance, the
  battery state of charge, temperatures and requirement margins are all
  explicit constraints.
- **Components build equations; the optimizer owns the variables.** Every
  physical component (motor, generator, battery, turboshaft, gearbox,
  rotor, inverter, cable, heat exchanger) exposes the same small interface:
  `get_mass`, `get_limits` and `evaluate`. Higher fidelity is added behind
  that interface, and the simpler model is always kept.
- **Fidelity is added in tiers.** Each tier has:
  - a written plan in `.agent/plans/completed/`;
  - unit tests;
  - an executed verification notebook in `notebooks/tierN_*`.
  Named legacy settings keep every earlier tier's result reproducible.
- **Trajectory optimization is a separate problem.** It flies the sized
  aircraft with AeroSandbox's point-mass dynamics.

Diagrams of the layering, the fidelity scaling and the solve levels:
[ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md). The tier list is in
[FIDELITY_ROADMAP.md](FIDELITY_ROADMAP.md); model detail per tier is in
[IMPLEMENTATION_NOTES.md](IMPLEMENTATION_NOTES.md).

## 2. How it is checked

| Check | What it shows |
|---|---|
| About 500 unit tests (`uv run python -m unittest discover -s tests`) | Closed-form identities, limiting cases, sign conventions and trends for every model. Every model is also exercised symbolically inside `asb.Opti` (CasADi compatibility). Integration tests pin each reference result. |
| One executed notebook per tier (`notebooks/`) | Each tier's claims as numbered checks with plots. Each notebook also runs the full test suite. |
| Tier 0 governance notebook | Repository rules: unit-suffixed names, no plain NumPy in models, every component documented, roadmap and plan bookkeeping. |
| CI (`.github/workflows/tests.yml`) | The test suite on every push. |

### External validation

- **Bell XV-15** (NASA TM X-62407 group-weight statement).
  - Uncalibrated, the weight models give 11,315 lb against the actual
    13,000 lb. Per group:
    - rotor model 1,545 lb against 1,070 actual;
    - fuselage 696 against 1,442;
    - flight controls 247 against 934.
  - Group calibration factors close the aircraft exactly. They are explicit
    and listed in `examples/xv15_reference.py`, not hidden.
  - The engine lapse and hover-ceiling models are checked against XV-15
    published data.
- **JVX proprotor** (NASA/TM-2016-219070, full-scale test). The rotor
  power model (momentum plus profile, airplane-mode increment) is fitted to
  hover and airplane-mode data:
  - figure of merit within 0.012;
  - cruise efficiency within 0.013.
- **Tiltrotor wing weight** (NASA NDARC tiltrotor wing, stiffness and
  whirl-flutter sized). It reproduces the XV-15 wing with its own factor,
  then predicts:
  - Bell D266 wing: −2 %;
  - V-22 FSD wing: −18 %.
  No public V-22 group-weight statement exists.
- **Battery cell:** Samsung INR21700-50G open-circuit voltage and DC
  internal resistance digitized from Paudel et al., *Batteries* 2025.
- **Drag:** AeroSandbox AeroBuildup and an independent Scholz (HAW Hamburg)
  hand build-up agree within about 2 % in CD0. Both read 15–20 % below
  NASA's NDARC drag estimate for the XV-15 (see Section 6).

## 3. The case study: a Halo-class unmanned series hybrid

**Configuration** (XV-15-like):

- two tip rotors driven by electric motors through reduction gearboxes;
- two turboshafts, each driving a generator through a step-up gearbox;
- a battery on the same DC bus.

**Requirements:**

- 900 kg payload, 445 nm mission with reserve;
- 210 kt sustained at 10,000 ft, 13,000 ft ceiling;
- out-of-ground-effect hover at 4,000 ft (T/W 1.05);
- hot-day hover at the destination (4,000 ft, ISA + 27.7 K);
- 60 s engine-out hover on one turbogenerator plus the battery;
- stall at or below 120 kt;
- stability and control margins.

**Fixed engines:** two off-the-shelf 1,120 hp turboshafts (a user-supplied
GASP deck). The engines are not sized, because a non-OEM cannot add
turbine power. The battery supplements hover and is recharged in flight.

### Reference result (`solve_halo_sizing()`, plan 030)

| Quantity | Value |
|---|---|
| Take-off mass | **6,367 kg (14,037 lb)** |
| Empty mass | 4,605 kg |
| Fuel (with reserve) | 862 kg |
| Battery | 60.7 kWh, 412 kg, 4,183 cells (end-of-life rating) |
| Wing | 21.9 m², span 11.6 m (tiltrotor wing, whirl-flutter sized) |
| Rotors | 2 × 9.3 m diameter; disk loading 47 kg/m² (9.6 lb/ft²) |
| Motors | 2 × 570 kW continuous (short-time ratings from thermal mass) |
| Turboshafts | 2 × 835 kW (1,120 hp), fixed |
| Cruise | 162 kt at 10,000 ft, L/D 9.9 |
| Heat exchanger | 133 kg ram-air unit, fans in hover |

**Mass breakdown (kg):**

| Group | Mass |
|---|---|
| Powertrain | 2,347 |
| Fuselage | 636 |
| Systems | 479 |
| Wing | 381 |
| Equipment | 266 |
| Landing gear | 244 |
| Nacelles | 195 |
| Tails | 57 |

Within the powertrain:

| Item | Mass |
|---|---|
| Rotors | 611 |
| Turboshafts | 422 |
| Battery | 412 |
| Rotor gearboxes | 356 |
| Generator gearboxes | 175 |
| Heat exchanger | 133 |
| Generators | 122 |
| Motors | 116 |

### What sizes the aircraft

These are the constraints active at the optimum:

- **Battery: the engine-out hover.** One turbogenerator plus the battery,
  starting from the 30 % reserve. The pack hits its 2.5 V-per-cell cutoff.
  Battery voltage, not energy, sizes it.
- **Wing: whirl-flutter torsion stiffness** at the 210 kt rotor speed.
- **Motors, gearboxes and rotors:** hover at 4,000 ft. The rotor radius is
  capped by the span.
- **Heat exchanger:** the hot-day hover. Climb sets the motor and generator
  temperatures.
- **Tails:** static margin and directional stability.

## 4. How the answer moved as fidelity was added

| Step | Payload | Take-off mass | What changed |
|---|---|---|---|
| Tier 10c | 900 kg at 250 kt | 18,740 lb | Engines sized freely; actuator-disk rotor |
| Tier 12 | 900 kg at 250 kt | 17,228 lb | Rotor-speed physics (JVX-calibrated) |
| Tier 12b | 900 kg at 210 kt | 14,877 lb | Engines fixed at 2 × 1,120 hp; 250 kt is infeasible with them (payload reaches zero near 228 kt) |
| Tiers 13–16 | 900 kg | 13,760 lb | Torque-sized machines and gear ratio; hot-and-high hover |
| Plan 022 | 780 kg | 14,436 lb | Equivalent-circuit battery: the realistic pack cannot carry 900 kg with the light-aircraft wing equations |
| Plan 026 | 900 kg | 14,247 lb | NDARC tiltrotor wing replaces the light-aircraft wing equations |
| Plan 027 | 900 kg | 13,639 lb | AeroBuildup aerodynamics: the guessed 0.8 m² miscellaneous drag area was most of the aircraft's drag |
| **Plan 030** | **900 kg** | **14,037 lb** | **Thermal model: heat exchanger +133 kg, short-time ratings −63 kg of machines** |

Two outcomes stand out:

- **Fixed engines cap the speed.** With two 1,120 hp engines, payload
  falls to zero near 228 kt, so 250 kt is unreachable at any size.
- **The realistic battery cost about 230 kg of payload** compared with an
  idealized constant-voltage pack. The binding limit is the cell voltage
  cutoff during the engine-out hover, not stored energy.

## 5. Trajectory optimization (Tier 14)

These problems fly the plan 027 aircraft (13,639 lb, thermal off); see
Section 6 for why.

- **Minimum-energy conversion** from hover at 500 ft to 1.3 × the
  airplane-mode stall speed:
  - optimized: 21.5 s and 8.5 kWh;
  - naive linear-nacelle, constant-acceleration schedule: 60 s and
    17.1 kWh.
  The optimized path uses about half the energy.
- **Minimum time to climb** from hover at sea level to 10,000 ft at cruise
  speed: 222 s, against about 508 s for the sizing mission's prescribed
  6 m/s climb.

## 6. Limits and open items, roughly by impact

1. **Drag corrections.** AeroBuildup models clean components only. It
   leaves out excrescence, leakage, protuberance and trim drag, which is
   why it reads 15–20 % below NDARC's XV-15 drag. Plan 034 adds them
   explicitly:
   - a component-drag factor calibrated to NDARC's XV-15 components (1.27
     for AeroBuildup, 1.17 for Scholz);
   - trim drag at 2 %.
   They are an option at the time of writing; see the implementation notes
   for their effect. Max payload at 210 kt is about 1,650 kg with uncorrected
   drag, so treat payload headroom as a sensitivity.
2. **Electrical layer (Tier 15) is implemented but not the default.**
   Inverters, cables, protection and partial-discharge insulation add
   about 330 kg of hardware and 3 % losses; with growth, about 685 kg
   (measured on the Scholz aero). With the full model set (AeroBuildup,
   realistic battery, electrical layer), IPOPT has not yet converged
   reliably.
3. **Weight calibration rests on one complete aircraft.** The XV-15 is the
   only complete weight statement. Two groups need large factors because
   the light-aircraft equations underpredict them: fuselage about 2×,
   flight controls about 4×. These are group factors, not aircraft mass;
   the Halo at 14,037 lb is about 8 % heavier than the 13,000 lb XV-15.
4. **Rotor reduction ratio of about 36:1** is a modelling artifact, being
   addressed in plan 033 (in progress):
   - a single motor torque density (15 N·m/kg) makes slow, high-torque
     machines look heavy;
   - the drive-system weight barely depends on ratio.
   Real machines span the trade: stackable axial-flux motors such as
   Evolito's (about 35 N·m/kg at up to 2,500 rpm) against fast radial
   machines such as Helix and H3X (17,000–20,000 rpm). Plan 033 adds a
   supplier database for machine mass and an explicit gearbox stage count.
5. **The trajectory model is point-mass.** It has no thermal or
   electrical-layer states, so it flies the thermal-off aircraft. 6-DOF is
   planned, not built (see the architecture diagrams).
6. **Thermal simplifications.** The battery chiller's power is not
   modelled. Gearbox heat is assumed to go to the gearboxes' own oil
   coolers. Losses do not depend on temperature.
7. **Deferred aero items:** V-tail, conversion-segment aerodynamics and
   trim drag.
8. **In progress at the time of writing:**
   - Tier 18, redundancy: motor lanes, cross-strapped buses, battery
     strings, failure cases;
   - Tier 22, design-space practice: multistart, cost objective,
     architecture enumeration, sensitivities.

## 7. Reproducing the results

```powershell
uv sync
uv run python -m unittest discover -s tests          # about 500 tests, 10–17 min
uv run python -m examples.halo_sizing                # the reference (about 4 min from cold)
```

Run a tier's notebook with Jupyter to reproduce that tier, e.g.
`notebooks/tier19_thermal/thermal_verification.ipynb`.

- **Earlier references:** the named sets in `examples/halo_sizing.py`
  reproduce each previous reference. Examples: `requirements_plan027` with
  `assumptions_plan027` gives 13,639 lb; `requirements_tier16` with
  `assumptions_tier16` gives 13,760 lb.
- **Parallel runs:** set `OMP_NUM_THREADS=1` when running several solves
  at once. Threaded BLAS under contention makes IPOPT fail spuriously.

## 8. Where to look first

| If you want to see… | Open |
|---|---|
| The whole coupled problem in one place | `examples/halo_sizing.py`, `solve_halo_sizing` |
| A component and its tests | `src/aircraft_closure/powertrain/components/battery_ecm.py`, `tests/powertrain/test_battery_ecm.py` |
| How a flight point couples the powertrain | `src/aircraft_closure/performance/flight_point.py` |
| Validation against the XV-15 | `notebooks/tier10_xv15_reference/` |
| Trajectory optimization | `examples/trajectory_optimization.py`, `notebooks/tier14_trajectory/` |
| Decisions and their reasons | `.agent/plans/completed/` (one plan per tier, with a progress log) |
