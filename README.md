# size_halo: sizing a Halo-class hybrid tiltrotor from scratch

How I would size an unmanned, series-hybrid-electric tiltrotor starting from a clean sheet. The aircraft, its
powertrain and its mission are sized together in **one gradient-based optimization** (AeroSandbox, CasADi, IPOPT),
and every model is checked against public tiltrotor data before it is trusted.

> All numbers are illustrative engineering inputs drawn from public sources. Nothing here is Archer or Halo data.
> See [assumptions](docs/HALO_REFERENCE.md).

## Requirements

| | |
|---|---|
| Payload and range | 1,984 lb (900 kg) over 445 nm, plus a 20 min reserve loiter |
| Maximum speed | 210 kt sustained at 10,000 ft |
| Ceiling | 13,000 ft |
| Hover | out of ground effect at 4,000 ft (T/W 1.05), including a 95 °F day (ISA + 50 °F) at the destination |
| Failure hovers | 60 s after losing a turbogenerator, a bus or a battery string |
| Stall | at or below 120 kt in airplane mode |
| Stability | static margin and directional stability (Cn_β) margins |
| Fixed input | two off-the-shelf 1,120 hp turboshafts. A non-OEM cannot add turbine power, so the engines are not a design variable |

## The sized aircraft: baseline design v3.6

| | |
|---|---|
| **Take-off weight** | **15,179 lb** (6,885 kg) |
| Empty weight (OEW) | 11,130 lb (5,049 kg), of which powertrain 6,329 lb (57 %) |
| Fuel / battery | 2,065 lb, 10 % reserve included / 70 kWh, 1,047 lb, two isolated strings |
| Mission cruise | 165 kt at 10,000 ft, L/D 9.1 (cruise speed is optimized for weight; 210 kt is a dash capability) |
| Turboshafts | 2 × 1,120 hp in the fuselage |
| Rotors | 2 × 31.7 ft diameter, disk loading 9.6 lb/ft², tip speed 782 ft/s; rotor radius capped by the span |
| Wing | 251 ft², 39.2 ft span, wing loading 60.5 lb/ft² |
| Drive | per rotor, 2 motor lanes of stacked axial-flux units behind one 4.5:1 stage; two cross-strapped DC buses |
| Fuselage | 36.1 ft long, unpressurized box section 5.5 × 6.6 ft |

The empty-weight breakdown is in the pie chart under [Performance](#performance).

The model works in SI internally; [docs/RESULTS.md](docs/RESULTS.md) gives the SI values alongside.

![Halo-class 3-view and nacelle conversion](docs/figures/halo_views.png)

**What sizes the aircraft** (the active constraints at the optimum):

- **Battery:** the engine-out hover. The pack hits its cell voltage cutoff, so voltage sag, not stored energy, sets
  its size.
- **Generators and their gearboxes:** the engine-out hover.
- **Motors:** the bus-out hover, with one lane per rotor carrying the torque.
- **Rotors and drive:** the 4,000 ft hover.
- **Heat exchanger:** the hot-day hover.
- **Tails:** static margin and directional stability.

**Findings worth knowing:**

- With these engines, payload goes to zero near 228 kt, so 250 kt is out of reach at any size.
- A realistic (equivalent-circuit) battery costs about 510 lb (230 kg) of payload against an ideal one.
- Real catalogue machines change the architecture, not just the mass. A freely scalable motor wants about
  13,000 rpm behind a 32:1 gearbox; whole units of real products want slow, stacked axial-flux motors behind a
  single stage.
- In conversion, the optimized transition rides the low-speed side of the computed corridor; no level,
  constant-acceleration conversion fits inside it.

## Aircraft versions

Every aircraft sized during development is a numbered version and stays reproducible from a named
requirement and assumption set. A major version means a new definition (configuration or requirements), and a minor
version means the same definition re-sized with better models. Patch versions are side studies or a parallel line.
The table below shows the main line plus the layout line that merged into the baseline; the full list, with how to
reproduce each version, is in [aircraft versions](docs/AIRCRAFT_VERSIONS.md).

**Requirements used to size each version.** Every version carries 1,984 lb (900 kg) over 445 nm, a 13,000 ft
ceiling, a 4,000 ft out-of-ground-effect hover, a 60 s engine-out hover, stall at or below 120 kt and a 20 min
reserve loiter, except where the table says otherwise.

| Version | Take-off weight | Requirements used to size it | What changed |
|---|---|---|---|
| v1.0 | 18,740 lb | 250 kt; engines sized freely | First Halo-class sizing: two-rotor series hybrid, actuator-disk rotor |
| v1.1 | 18,506 lb | as v1.0 | Supplied 1,120 hp deck part-power fuel curve |
| v1.2 | 17,228 lb | as v1.0 | Rotor-speed physics calibrated on JVX proprotor data |
| v2.0 | 14,877 lb | **210 kt; engines fixed at 2 × 1,120 hp** (250 kt is infeasible with them) | Battery-assisted hover, in-flight recharge |
| v2.1 | 13,760 lb | as v2.0 | Electric machines sized by torque; machine speed and gear ratio as design variables |
| v3.0 | 13,760 lb | as v2.0, **plus a hot-day hover at the destination** (4,000 ft, ISA + 50 °F) | Temperature lapse; the hot day is not yet binding |
| v3.0.2 | 14,436 lb | as v3.0, but **1,720 lb (780 kg) payload** | Equivalent-circuit battery; 1,984 lb (900 kg) did not close with the light-aircraft wing equations |
| v3.0.3 | 13,546 lb | as v3.0.2 | NDARC/AFDD tiltrotor wing with whirl-flutter margins |
| v3.1 | 14,247 lb | as v3.0 (payload back to 1,984 lb) | Equivalent-circuit battery and AFDD wing on the full payload |
| v3.2 | 13,639 lb | as v3.0 | AeroBuildup aerodynamics replace the simple polar and a guessed drag area |
| v3.3 | 14,037 lb | as v3.0 | Thermal model: heat exchanger, cooling drag, short-time machine ratings |
| v3.3.4 | 12,821 lb | as v3.0 | Layout line: fuselage anchored to the drawn layout, turbogenerators in the fuselage, boxy 36 ft fuselage |
| v3.3.5 | 13,038 lb | as v3.0 | Layout line: AFDD spar caps at the real box depth, 1 mm minimum gauge, nacelle inertia from components |
| v3.4 | 16,231 lb | as v3.0, **plus bus-out and string-out failure hovers** | Whole units of real machines, 2 lanes/2 buses/2 strings redundancy, gearbox stages, drag corrections |
| v3.5 | 16,303 lb | as v3.4 | Trim drag from the tail load |
| **v3.6 (baseline)** | **15,179 lb** | as v3.4 (the requirements table above) | Layout line (v3.3.4, v3.3.5) merged into v3.5, with every model on |

## What it does

```
requirements ──┐
mission ───────┤      components return equations and residuals,
powertrain ────┤      never variables or loops
aero / rotor ──┼──►  ONE asb.Opti problem  ──►  IPOPT  ──►  sized aircraft + mission + energy split
weights ───────┤      caller owns every variable, constraint
stability ─────┤      and the objective
thermal ───────┤
failure cases ─┘
```

- **One problem, no hidden loops.** Mass closure, mission fuel and state of charge, battery-versus-generator energy
  allocation and every failure case are constraints in the same problem. Derivatives come from CasADi automatic
  differentiation.
- **Typed powertrain network.** Components connect through ports (shaft, DC bus) with multiplicity, so "two lanes
  per rotor, two buses, two strings" is a topology, not hand-written bookkeeping. Speed, torque, voltage, current
  and power compatibility are checked as normalized margins.
- **Named binding constraints.** Every margin has a name, so the solver reports *what* sizes the aircraft.
- **Fidelity in layers.** Each model sits behind a simple interface, and the simple version is kept, so the effect
  of each model on the answer is traceable
  ([how the answer moved](docs/RESULTS.md#4-how-the-answer-moved-as-fidelity-was-added)).
- **Conversion and trajectories.** The sized aircraft is trimmed at every nacelle angle to compute its conversion
  corridor, then flown by direct collocation (minimum-energy transition, time to climb) inside it.
- **Geometry and structure exports.** The sized aircraft goes to OpenVSP (outer mold line, internal structure,
  STEP/STL), VSPAERO and CalculiX as independent checks; nothing in the sizing depends on them.

## By discipline

Approach and effort for each next step: [docs/NEXT_STEPS.md](docs/NEXT_STEPS.md).

### Performance

**Does**
- Mission analysis inside the sizing:
  - take-off hover, climb, cruise, 20 min reserve loiter, descent and landing hover;
  - fuel burn and battery state of charge through every segment;
  - the battery-versus-generator energy split, optimized per segment;
  - cruise speed optimized for weight (165 kt; 210 kt is a dash capability).
- Point performance as requirements: hover at 4,000 ft and on a hot day, ceiling, maximum speed, stall, and 60 s
  failure hovers.
- Mass closure with a full breakdown, and a cost per mission (about $3,300, with labelled price assumptions).

**Doesn't do**
- No payload-range diagram and no maximum endurance. Performance is computed at the design mission only.
- No energy-flow (Sankey) diagram from fuel and battery to the rotors, although the per-segment powers and losses
  are already in the solution.

**Next**
- A payload-range diagram and endurance: re-solve the fixed aircraft at off-design payload.
- An energy-flow diagram per segment from the solved powers and losses.

![Empty mass breakdown](docs/figures/oew_breakdown.png)

### Aerodynamics

**Does**
- AeroSandbox AeroBuildup in the sizing, with Scholz excrescence corrections, trim drag from the tail load, the blown
  wing and hover download. An independent Scholz hand build-up agrees within about 2 % in CD0.
- Proprotor: momentum plus profile power with tip-Mach rise, fitted to full-scale JVX data (figure of merit within
  0.012, cruise efficiency within 0.013).
- Cross-check of the same geometry in OpenVSP VSPAERO (vortex lattice and panel) and its parasite-drag build-up.

**Doesn't do**
- Drag is not anchored to flight data. The excrescence factor matches NASA NDARC's XV-15 estimate, and drag drives
  payload headroom more than anything else.
- No conversion-mode aerodynamics. The V-tail is drawn, but the sizing uses a conventional tail.
- Airplane-mode rotor efficiency comes from the JVX test, not flight data.

**Next:** anchor drag and cruise rotor efficiency to the XV-15 power-required curve, then size the V-tail.

![Drag polar by model](docs/figures/drag_polar_models.png)
![JVX proprotor calibration](docs/figures/rotor_jvx_calibration.png)

### Powertrain

**Does**
- A typed series-hybrid network of ports and buses, with multiplicity: 2 motor lanes per rotor, 2 cross-strapped
  buses, 2 battery strings.
- Components:
  - McDonald-loss machines built from whole units of real products, chosen from a supplier database;
  - gearboxes with stage counts;
  - a Samsung 50G-shaped equivalent-circuit battery with sag and an end-of-life rating;
  - the fixed 1,120 hp turboshaft deck with lapse and a part-power fuel curve;
  - a heat exchanger and short-time thermal ratings.
- Speed, torque, voltage, current and power compatibility checked as named margins.
- What sizes it:
  - the battery: its cell voltage cutoff in the engine-out hover;
  - the generators: the engine-out hover;
  - the motors: the bus-out hover.

**Doesn't do**
- The electrical layer (inverters, cables, protection) is built but off by default. With it on, the full problem
  converges only through multistart.
- Failures are single and symmetric. Double failures did not converge, and roll trim is not modelled.
- One catalogue product per machine role.
- No battery chiller power, and losses do not depend on temperature.

**Next**
- The electrical layer on by default, with problem scaling and continuation.
- Asymmetric and double failures, including an interconnect shaft against electrical cross-strapping.
- Mixed machine catalogues.

![Battery OCV fit](docs/figures/battery_ocv_fit.png)

### Structures

**Does**
- Mass in the sizing is handbook-based:
  - NDARC/AFDD tiltrotor wing sized for stiffness, whirl-flutter frequency margins and the jump take-off, times
    1.33 from the XV-15 calibration;
  - AFDD rotor and drive equations;
  - Raymer fuselage times 1.70, anchored to a drawn structural layout.
- Downstream check only, never fed back: an OpenVSP structural layout exported to CalculiX. A CalculiX wing-box
  check gives modal frequencies and a linear static jump take-off at ultimate load.

**Doesn't do**
- **Wing strength at ultimate load is not demonstrated.** The linear finite-element check (v3.3.5) puts the peak
  spar-cap strain 10 % above the ultimate allowable, a negative margin.
- **No buckling analysis.** Plate estimates suggest the unstiffened 1 mm covers and webs buckle well below ultimate.
- **Whirl flutter is a frequency margin, not a stability analysis.**
- **Weights calibrate on one aircraft (the XV-15).** Flight controls carry a factor of about 4.

**Next:**
- Close the wing: realistic root support, the calibrated structure in the finite-element model, a CalculiX
  buckling step, stiffened panels, and a strain margin of at least zero enforced in the sizing.
- Then a coupled rotor-wing whirl-flutter model inside the optimization.

![Structural layout](docs/figures/halo_structure.png)

### Flight dynamics

**Does**
- Static stability and control in the sizing: neutral point, static margin, elevator trim, Cn_β, rudder for a
  failed rotor.
- Conversion corridor: the sized aircraft trimmed at every nacelle angle (ruddervator, rotor disc tilt, attitude,
  edgewise-flow, power and placard limits).
- Point-mass trajectories by direct collocation inside that corridor: minimum-energy transition and time to climb.

**Doesn't do**
- The corridor and trajectories are not part of the sizing; they are flown on the sized aircraft afterwards.
- No 6-DOF and no lateral trim. No rotor in-plane force or rotor-speed schedule in the corridor.

**Next:**
- Rotor in-plane force and a rotor-speed schedule.
- Linearized models at the corridor trim points as the first control-law step.
- Then 6-DOF.

![Computed conversion corridor](docs/figures/conversion_corridor.png)
![Trajectories in the computed conversion corridor](docs/figures/trajectory_conversion_corridor.png)

## How it is checked

- **658 unit tests:** closed-form identities, limiting cases, sign conventions and trends, and every model exercised
  symbolically inside `asb.Opti`. The full suite takes 10-17 min locally and currently exceeds the 30 min CI limit
  (see [next steps](docs/NEXT_STEPS.md)).
- **Six executed discipline notebooks** ([`notebooks/`](notebooks/)) size the baseline design and verify each
  discipline with numbered checks and plots (458 checks, all passing):

| Notebook | Contents |
|---|---|
| [Global sizing](notebooks/01_global_sizing.ipynb) | The baseline design, mass closure, binding constraints, empty-weight breakdown, mission and energy allocation; requirements, missions and coupled sizing building blocks |
| [Powertrain](notebooks/02_powertrain.ipynb) | Machine units, failure cases and temperatures; machines and losses, supplier database, gearboxes, topology, margins, electrical layer, redundancy, thermal, turboshaft lapse, hot and high |
| [Energy storage](notebooks/03_energy_storage.ipynb) | The pack through the mission and the engine-out hover; the 50G equivalent-circuit cell model |
| [Aerodynamics and rotor](notebooks/04_aerodynamics_rotor.ipynb) | AeroBuildup and Scholz on the baseline design, XV-15 drag, blown wing, hover download; the JVX-calibrated proprotor; the simple model |
| [Structures and weights](notebooks/05_structures_weights.ipynb) | Airframe groups, AFDD wing items and whirl-flutter frequency margins; XV-15 weight calibration; the AFDD tiltrotor wing |
| [Dynamics and control](notebooks/06_dynamics_control.ipynb) | Static stability, the computed conversion corridor and trim, trajectories inside the corridor; trim and tail-sizing building blocks |

- **External validation:**
  - **Bell XV-15** group weights. Uncalibrated, the models give 11,315 lb against 13,000 lb actual; explicit group
    factors close it.
  - **JVX full-scale proprotor:** figure of merit within 0.012 and cruise efficiency within 0.013, with hover points
    held out of the fit.
  - **Tiltrotor wing weight:** fitted on the XV-15, then −2 % on the Bell D266 wing and −18 % on the V-22 FSD wing.
  - **Drag:** AeroBuildup and an independent Scholz hand build-up agree within about 2 % in CD0.
- **Cross-checks from another direction:** OpenVSP geometry and VSPAERO against AeroSandbox, a structural layout
  back-check of the weight equations, and a CalculiX finite-element check of the wing box.

![JVX proprotor calibration](docs/figures/rotor_jvx_calibration.png)

## Running it

```bash
uv sync
uv run python -m examples.halo_sizing                    # the baseline design (about 10 min from cold)
uv run python -m unittest discover -s tests              # 10-17 min
uv sync --group notebooks && uv run jupyter lab notebooks
```

Set `OMP_NUM_THREADS=1` when running several solves at once; threaded BLAS under contention makes IPOPT fail
spuriously. The OpenVSP and CalculiX tools are optional and need separate installs
([details](docs/IMPLEMENTATION_NOTES.md)); nothing in the sizing depends on them.

## Repository map

| Path | Contents |
|---|---|
| `src/aircraft_closure/core` | Typed ports, topology with buses and multiplicity, connection residuals, normalized margins |
| `src/aircraft_closure/powertrain` | Motors, generators, battery, turboshaft, gearboxes, rotor; machine database; redundancy; series-hybrid builders |
| `src/aircraft_closure/vehicle`, `weights` | Airframe components, mass and CG aggregation; AFDD rotorcraft weights |
| `src/aircraft_closure/aerodynamics`, `controls` | Lift and drag build-ups, trim, download; static stability |
| `src/aircraft_closure/mission`, `requirements`, `performance` | Segments and missions; capability requirements; coupled flight points |
| `src/aircraft_closure/thermal`, `trajectory` | Heat rejection and ratings; collocation trajectories and the conversion corridor |
| `src/aircraft_closure/export/openvsp` | Optional geometry, aero cross-check and FE decks; never imported by sizing |
| `examples/` | The baseline design (`halo_sizing.py`), XV-15 and JVX validation, conversion corridor and trajectories |
| `notebooks/` | Six executed discipline notebooks |
| `docs/` | [Results](docs/RESULTS.md), [next steps](docs/NEXT_STEPS.md), [aircraft versions](docs/AIRCRAFT_VERSIONS.md), [architecture](docs/ARCHITECTURE.md), [coding conventions](docs/CODING_CONVENTIONS.md), [design log](docs/decisions/) |

Lower layers never import higher ones. Components build equations; callers own variables, constraints and
objectives.
