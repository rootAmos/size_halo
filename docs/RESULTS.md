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
  - a written plan in `docs/decisions/`;
  - unit tests;
  - checks in the discipline's executed notebook in `notebooks/`.
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
| About 560 unit tests (`uv run python -m unittest discover -s tests`) | Closed-form identities, limiting cases, sign conventions and trends for every model. Every model is also exercised symbolically inside `asb.Opti` (CasADi compatibility). Integration tests pin each reference result. |
| Six executed discipline notebooks (`notebooks/`) | The reference and each discipline's models as numbered checks with plots. |
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

### Reference result (`solve_halo_sizing()`, every model on)

The combined reference: plans 035–036 (real machine units, redundancy, drag
corrections, trim from the tail load) together with the geometry and layout
line (plans 031–038).

| Quantity | Value |
|---|---|
| Take-off mass | **6,885 kg (15,179 lb)** |
| Empty mass | 5,049 kg |
| Fuel (with reserve) | 936 kg |
| Battery | 70.0 kWh, 475 kg, 2 isolated strings (end-of-life rating) |
| Wing | 23.3 m², span 11.9 m (NDARC tiltrotor wing, whirl-flutter margins) |
| Rotors | 2 × 9.7 m diameter; disk loading 47 kg/m² (9.6 lb/ft²); tip speed 238 m/s |
| Motors | 2 lanes per rotor; each lane is 2 Evolito D1500-class units (528 kW continuous) |
| Generators | each is 3 Helix SPX242-class units |
| Rotor gearbox | 4.5:1, one stage |
| Turboshafts | 2 × 835 kW (1,120 hp), fixed, in the fuselage |
| Cruise | 165 kt at 10,000 ft, L/D 9.07 |
| Heat exchanger | 127 kg ram-air unit, fans in hover |
| Cost per mission | about USD 3,310 (Tier 22 cost model; prices are labelled assumptions) |

**Empty-mass breakdown (kg):**

![Empty mass breakdown](figures/oew_breakdown.png)

| Group | Mass |
|---|---|
| Powertrain | 2,871 |
| Fuselage | 560 |
| Wing | 408 |
| Systems | 407 |
| Equipment | 266 |
| Landing gear | 258 |
| Nacelles | 195 |
| Tails | 83 |

Within the powertrain:

| Item | Mass |
|---|---|
| Rotors | 675 |
| Battery | 475 |
| Motors with inverters | 426 |
| Turboshafts | 422 |
| Rotor gearboxes | 292 |
| Generators with inverters | 282 |
| Generator gearboxes | 144 |
| Heat exchanger | 127 |
| Bus tie and string protection | 28 |

### What sizes the aircraft

These are the constraints active at the optimum:

- **Battery: the engine-out hover.** One turbogenerator plus the battery,
  starting from the 30 % reserve. The pack hits its 2.5 V-per-cell cutoff.
  Battery voltage, not energy, sizes it.
- **Generators and their gearboxes: the engine-out hover**, at power and
  torque.
- **Motors: the bus-out hover.** One bus lost; the surviving lane in each
  rotor carries the torque.
- **Rotors and drive:** hover at 4,000 ft. The rotor radius is capped by the
  span.
- **Heat exchanger:** the hot-day hover.
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
| Plan 030 | 900 kg | 14,037 lb | Thermal model: heat exchanger +133 kg; short-time ratings save 63 kg of machines |
| Plan 035 | 900 kg | 16,231 lb | **Real machine units instead of idealized ("rubber") scaling; redundancy (2 lanes, 2 buses, 2 strings); drag corrections for excrescence and trim** |
| Plan 036 | 900 kg | 16,303 lb | Trim drag from the tail load (η_H 0.9, cos tail dihedral, Scholz downwash) replaces the flat 2 % |
| **Combined** | **900 kg** | **15,179 lb** | **Geometry and layout line merged (plans 031–038): fuselage weight anchored to a drawn structural layout, turbogenerators in the fuselage, AFDD spar caps at the real box depth with a minimum gauge, nacelle inertia built from components. Lighter mainly in the fuselage and systems groups** |

Two outcomes stand out:

- **Fixed engines cap the speed.** With two 1,120 hp engines, payload
  falls to zero near 228 kt, so 250 kt is unreachable at any size.
- **The realistic battery cost about 230 kg of payload** compared with an
  idealized constant-voltage pack. The binding limit is the cell voltage
  cutoff during the engine-out hover, not stored energy.
- **Real machines change the drive architecture, not just the mass.**
  - With an idealized scalable machine, the optimizer spins motors to about
    13,000 rpm behind a roughly 32:1 gearbox.
  - Built from whole units of real best-in-class products, it chooses slow,
    stacked axial-flux motors behind a single 5:1 stage.
  - A 21-machine supplier database (plan 033) shows torque density falls
    only weakly with speed across the fleet. The best low-speed machines
    (Evolito, Siemens SP200D) sit 1.5–1.8× above that trend.

## 5. Trajectory optimization (Tier 14)

Since plan 036 the trajectory model flies the current reference. It
carries machine and battery temperature states with short-time ratings,
cooling drag and fan power, and the lane motors.

On the 15,179 lb reference, flown inside the computed conversion corridor
(plan 039; `examples/halo_trajectory_computed_corridor.py`):

- **Minimum-energy conversion** from hover at 500 ft to 1.3 × the
  airplane-mode stall speed: 21.9 s and 9.6 kWh. It rides the corridor's
  low-speed side from about 15 to 105 kt and descends slightly (inside its
  ±30 m altitude band) through 15–90 kt.
- **Minimum time to climb** from hover at sea level to 10,000 ft at cruise
  speed: 228 s, SOC 0.95 → 0.78. That compares with about 508 s for the
  sizing mission's prescribed 6 m/s climb.
- **No level, constant-acceleration conversion fits the computed
  corridor.** With the nacelles high enough for the corridor, accelerating
  needs a nose-down attitude beyond the trajectory model's −10° limit. The
  naive schedule flown in the assumed XV-15-shaped corridor took 60 s and
  19.4 kWh on this aircraft, about twice the optimized energy.

## 6. Limits and open items, roughly by impact

1. **Drag calibration.** The excrescence factor is calibrated so the
   XV-15 components match NASA NDARC's drag (1.27 on AeroBuildup, plan
   034). Trim drag comes from the tail load needed for zero moment about
   the CG (plan 036): AeroBuildup's moment, Scholz's tail efficiency
   (0.9) and downwash, about 2 % of drag at cruise. Neither is checked against XV-15 flight data, and drag
   strongly drives payload headroom.
2. **Weight calibration rests on one complete aircraft.** The XV-15 is the
   only complete weight statement. Two groups need large factors because
   the light-aircraft equations underpredict them: fuselage about 2×,
   flight controls about 4×. These are group factors, not aircraft mass.
   The calibration factors are also the largest sensitivities (Tier 22:
   about ±540 lb each for ±15 %).
3. **The electrical layer (Tier 15) is implemented but not the default.**
   Inverters, cables, protection and partial-discharge insulation add about
   330 kg of hardware and 3 % losses. The full combination converges only
   through the multistart strategy, slowly.
4. **Machine units are one product per role.** Motors are Evolito
   D1500-class and generators Helix SPX242-class. Other catalogue choices
   (magniX, Siemens, EMRAX, mixed units) are not enumerated. Evolito's
   continuous rating is inferred from its rated torque density.
5. **Redundancy failure cases are single failures.** Double failures (a
   battery string or a lane, plus an engine) did not converge, and they are
   not proven infeasible. Degraded states are applied to both rotors at
   once, so roll trim is not modelled.
6. **The trajectory model is point-mass.** 6-DOF is planned, not built
   (see the architecture diagrams). The computed conversion corridor (plan
   039) leaves out the rotor's in-plane force in edgewise flow and an
   airplane-mode rotor-speed schedule. The trajectory model has no cyclic,
   so it is stricter than the trim that computes the corridor.
7. **Thermal simplifications.** The battery chiller's power is not
   modelled. Gearbox heat goes to the gearboxes' own oil coolers. Losses do
   not depend on temperature.
8. **Deferred aero items:** the V-tail (sizing still uses a conventional
   tail; the V-tail is only drawn) and conversion-segment aerodynamics.
9. **Solver robustness.** Each coupled problem is solved by IPOPT from an
   explicit list of starting points (Tier 22). Every start that converged
   reached the same optimum, but some feature combinations take several
   failed starts first.
10. **Wing finite-element check.** The CalculiX check of the wing box puts the
   jump take-off strain at 1.10 of the allowable after the spar-cap
   correction, and the torsion mode is not cleanly identified.

## 7. Reproducing the results

```powershell
uv sync
uv run python -m unittest discover -s tests          # about 500 tests, 10–17 min
uv run python -m examples.halo_sizing                # the reference (about 10 min from cold)
```

Run a discipline notebook with Jupyter to reproduce its checks, e.g.
`notebooks/02_powertrain.ipynb`.

- **Earlier references:** the named sets in `examples/halo_sizing.py`
  reproduce each previous reference. Examples:
  - `requirements_plan030` with `assumptions_plan030` gives 14,037 lb;
  - `requirements_plan027` with `assumptions_plan027` gives 13,639 lb;
  - `requirements_tier16` with `assumptions_tier16` gives 13,760 lb.
- **Parallel runs:** set `OMP_NUM_THREADS=1` when running several solves
  at once. Threaded BLAS under contention makes IPOPT fail spuriously.

## 8. Where to look first

| If you want to see… | Open |
|---|---|
| The whole coupled problem in one place | `examples/halo_sizing.py`, `solve_halo_sizing` |
| A component and its tests | `src/aircraft_closure/powertrain/components/battery_ecm.py`, `tests/powertrain/test_battery_ecm.py` |
| How a flight point couples the powertrain | `src/aircraft_closure/performance/flight_point.py` |
| Validation against the XV-15 | `notebooks/05_structures_weights.ipynb` (weights), `notebooks/04_aerodynamics_rotor.ipynb` (drag), `notebooks/02_powertrain.ipynb` (engine and hover) |
| Trajectory optimization | `examples/trajectory_optimization.py`, `examples/halo_trajectory_computed_corridor.py`, `notebooks/06_dynamics_control.ipynb` |
| Decisions and their reasons | `docs/decisions/` (one plan per tier, with a progress log) |
