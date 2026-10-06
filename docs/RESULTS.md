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
| About 650 unit tests (`uv run python -m unittest discover -s tests`) | Closed-form identities, limiting cases, sign conventions and trends for every model. Every model is also exercised symbolically inside `asb.Opti` (CasADi compatibility). Integration tests pin each reference result. |
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

### Reference result (`solve_halo_sizing()`, every model on)

The default combines the two lines that branched from plan 030: real machine units, redundancy and drag
corrections (plans 035–036), and the drawn layout with the wing corrections (plans 037–038).

| Quantity | Value |
|---|---|
| Take-off mass | **6,885 kg (15,179 lb)** |
| Empty mass | 5,049 kg |
| Fuel (with reserve) | 936 kg |
| Battery | 70.0 kWh, 475 kg, 2 isolated strings (end-of-life rating) |
| Fuselage | unpressurized, boxy, 11 m long, 1.68 × 2.0 m; turbogenerators inside |
| Wing | 23.3 m², span 11.9 m (tiltrotor wing; spar caps at the box depth, 1 mm minimum gauge) |
| Rotors | 2 × 9.7 m diameter; disk loading 47 kg/m² (9.6 lb/ft²) |
| Motors | 2 lanes per rotor; each lane is 2 Evolito D1500-class units (528 kW continuous), 8 units in all |
| Generators | each is 3 Helix SPX242-class units |
| Rotor gearbox | 4.5:1, one stage |
| Turboshafts | 2 × 835 kW (1,120 hp), fixed |
| Cruise | 165 kt at 10,000 ft, L/D 9.07 |
| Heat exchanger | 127 kg ram-air unit, fans in hover |
| Cost per mission | about USD 3,310 (Tier 22 cost model; prices are labelled assumptions) |

**Mass breakdown (kg):**

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
- **Motors: the bus-out hover.** One bus lost; the surviving lane in each
  rotor carries the torque.
- **Turboshafts: fully used.** Their power binds in the engine-out and
  hot-day hovers. They are fixed, so this is what sizes the battery and the
  rotors.
- **Rotors and drive:** hover at 4,000 ft. The rotor radius is capped by the
  span.
- **Heat exchanger:** the hot-day hover.
- **Wing:** whirl flutter is no longer binding (torsion 1.57 per rev at the
  maximum speed) once the torque box has its 1 mm minimum gauge (plan 038).
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
| Plan 035 | 900 kg | 16,231 lb | Real machine units instead of idealized ("rubber") scaling; redundancy (2 lanes, 2 buses, 2 strings); drag corrections for excrescence and trim |
| Plan 036 | 900 kg | 16,303 lb | Trim drag from the tail load (η_H 0.9, cos tail dihedral, Scholz downwash) replaces the flat 2 % |
| Plan 037 | 900 kg | 12,821 lb | From plan 030 on a separate line: the drawn layout. Fuselage raw Raymer × 1.70 (layout-anchored), turbogenerators in the fuselage, boxy 11 m fuselage |
| Plan 038 | 900 kg | 13,038 lb | Spar caps at the real box depth, 1 mm torque-box gauge, pylon inertia from the tip components (after the CalculiX check) |
| **Combined** | **900 kg** | **15,179 lb** | **Both lines merged, every model on (plans 035–038)** |

Three outcomes stand out:

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

Since plan 036 the trajectory model flies the full-feature aircraft. It
carries machine and battery temperature states with short-time ratings,
cooling drag and fan power, and the lane motors. It flies inside an assumed
XV-15-shaped conversion corridor.

On the plan 035 reference (16,231 lb; not yet re-flown on the 15,179 lb
combined reference):

- **Minimum-energy conversion** from hover at 500 ft to 1.3 × the
  airplane-mode stall speed: 22.5 s and 10.5 kWh.
- **Minimum time to climb** from hover at sea level to 10,000 ft at cruise
  speed: 225 s, SOC 0.95 → 0.73. That compares with about 508 s for the
  sizing mission's prescribed 6 m/s climb.

On the plan 027 aircraft (13,639 lb, thermal off), the optimized
conversion took 21.5 s and 8.5 kWh. A naive linear-nacelle,
constant-acceleration schedule took 60 s and 17.1 kWh, so the optimized
path uses about half the energy.

### Computed conversion corridor (plan 039)

`trajectory/corridor.py` trims the sized aircraft in level flight (thrust,
attitude, ruddervator, cyclic). At each nacelle angle it finds the least
and greatest airspeed inside the limits:

- pitch −5 to +12 deg, ruddervator ±25 deg;
- cyclic ±10 deg, washed out toward airplane mode;
- edgewise advance ratio 0.28 (a flapping and hub-load proxy);
- rotor power, unblown-wing stall, and a placard at 1.1 × 210 kt.

On the 15,179 lb reference at sea level:

| Corridor (kt) | 90 deg | 75 deg | 60 deg | 45 deg | 30 deg | 0 deg |
|---|---|---|---|---|---|---|
| Low side | hover | hover | 89 (pitch) | 106 (pitch) | 112 (pitch) | 119 (pitch) |
| High side | 130 (edgewise) | 134 (edgewise) | 144 (edgewise) | 174 (edgewise) | 231 (placard) | 219 (rotor power) |

- The high side is edgewise-limited from 45 deg up, close to the XV-15.
- The low side is attitude-limited, not stall-limited.
- Pitch authority is tightest in helicopter mode near 65 kt.
- The rotor in-plane force is not modelled, so the low side at 45–60 deg
  is conservative.

## 6. Limits and open items, roughly by impact

1. **Drag calibration.** The excrescence factor is calibrated so the
   XV-15 components match NASA NDARC's drag (1.27 on AeroBuildup, plan
   034). Trim drag comes from the tail load needed for zero moment about
   the CG (plan 036): AeroBuildup's moment, Scholz's tail efficiency
   (0.9) and downwash, about 2 % of drag at cruise. Neither is checked against XV-15 flight data, and drag
   strongly drives payload headroom.
2. **Weight calibration rests on one complete aircraft.** The XV-15 is the
   only complete weight statement. Flight controls need a factor of about
   4×, because the light-aircraft equations underpredict them. The XV-15
   fuselage factor (about 2×) was replaced in plan 037 by 1.70 on raw
   Raymer, anchored to a layout estimate of an uncrewed fuselage. These are
   group factors, not aircraft mass.
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
6. **The trajectory model is point-mass.** It has no electrical-layer
   states, and 6-DOF is planned, not built (see the architecture diagrams).
   The computed corridor (plan 039) still lacks the rotor in-plane force, a
   rotor speed schedule, lateral trim and linearized models.
7. **Wing strength by FE.** The CalculiX check of the plan 038 wing still
   shows 1.10× the allowable strain in the jump take-off, and wing torsion
   is not cleanly identified among the FE modes. Only the wing box has been
   solved by FE.
8. **Thermal simplifications.** The battery chiller's power is not
   modelled. Gearbox heat goes to the gearboxes' own oil coolers. Losses do
   not depend on temperature.
9. **Deferred aero items:** the V-tail is drawn but sized as a
   conventional tail; conversion-segment aerodynamics.
10. **Solver robustness.** Each coupled problem is solved by IPOPT from an
   explicit list of starting points (Tier 22). Every start that converged
   reached the same optimum, but some feature combinations take several
   failed starts first.

## 7. Reproducing the results

```powershell
uv sync
uv run python -m unittest discover -s tests          # about 650 tests, 10–17 min
uv run python -m examples.halo_sizing                # the reference (about 10 min from cold)
```

Run a tier's notebook with Jupyter to reproduce that tier, e.g.
`notebooks/tier19_thermal/thermal_verification.ipynb`.

- **Earlier references:** the named sets in `examples/halo_sizing.py`
  reproduce each previous reference. Examples:
  - `requirements_plan038` with `assumptions_plan038` gives 13,038 lb;
  - `requirements_plan037` with `assumptions_plan037` gives 12,821 lb;
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
| Validation against the XV-15 | `notebooks/tier10_xv15_reference/` |
| Trajectory optimization | `examples/trajectory_optimization.py`, `notebooks/tier14_trajectory/` |
| Conversion corridor and trim | `src/aircraft_closure/trajectory/corridor.py`, `examples/halo_conversion_corridor.py` |
| Geometry, aero cross-check and FE | `src/aircraft_closure/export/openvsp/`, `examples/halo_openvsp.py`, `examples/halo_wing_fe.py` |
| Decisions and their reasons | `.agent/plans/completed/` (one plan per tier, with a progress log) |
