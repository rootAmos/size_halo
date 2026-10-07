# size_halo: sizing a Halo-class hybrid tiltrotor from scratch

How I would size an unmanned, series-hybrid-electric tiltrotor starting from a clean sheet. The aircraft, its
powertrain and its mission are sized together in **one gradient-based optimization** (AeroSandbox, CasADi, IPOPT),
and every model is checked against public tiltrotor data before it is trusted.

> All numbers are illustrative engineering inputs drawn from public sources. Nothing here is Archer or Halo data.
> See [assumptions](docs/HALO_REFERENCE.md).

## The baseline design (v3.6)

| | |
|---|---|
| **Take-off weight** | **6,885 kg (15,179 lb)** |
| Payload | 900 kg (1,984 lb) |
| Empty weight (OEW) | 5,049 kg (11,130 lb), of which powertrain 2,871 kg (57 %) |
| Fuel / battery | 936 kg (10 % reserve included) / 70 kWh, 475 kg, two isolated strings |
| Maximum speed | 210 kt sustained at 10,000 ft |
| Design range | 445 nm with 900 kg, plus a 20 min reserve loiter |
| Mission cruise | 165 kt at 10,000 ft, L/D 9.1 (cruise speed is optimized for mass; 210 kt is a dash capability) |
| Ceiling | 13,000 ft |
| Hover | out of ground effect at 4,000 ft, including an ISA + 27.7 K day at the destination |
| Failure hovers | 60 s after losing a turbogenerator, a bus or a battery string |
| Turboshafts | 2 × 835 kW (1,120 hp), fixed off-the-shelf engines in the fuselage |
| Rotors | 2 × 9.7 m, disk loading 47 kg/m² (9.6 lb/ft²), rotor radius capped by the span |
| Wing | 23.3 m², 11.9 m span |
| Drive | per rotor, 2 motor lanes of stacked axial-flux units behind one 4.5:1 stage; two cross-strapped DC buses |
| Fuselage | 11 m, unpressurized box section 1.68 × 2.0 m |

**Empty weight breakdown (kg):**

| Powertrain | | Airframe and systems | |
|---|---|---|---|
| Rotors and gearboxes | 1,111 | Wing, nacelles, tails | 686 |
| Motors and generators (with inverters) | 707 | Systems and equipment | 674 |
| Battery | 475 | Fuselage | 560 |
| Turboshafts | 422 | Landing gear | 258 |
| Heat exchanger, protection, bus tie | 155 | | |

Not computed yet: maximum range at reduced payload (the payload-range curve) and maximum endurance.

![Empty mass breakdown](docs/figures/oew_breakdown.png)

![Halo-class 3-view and nacelle conversion](docs/figures/halo_views.png)

**Fixed inputs:** two off-the-shelf 1,120 hp turboshafts. A non-OEM cannot add turbine power, so the engines are an
input, not a design variable.

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
- A realistic (equivalent-circuit) battery costs about 230 kg of payload against an ideal one.
- Real catalogue machines change the architecture, not just the mass. A freely scalable motor wants about
  13,000 rpm behind a 32:1 gearbox; whole units of real products want slow, stacked axial-flux motors behind a
  single stage.
- In conversion, the optimized transition rides the low-speed side of the computed corridor; no level,
  constant-acceleration conversion fits inside it.

Every aircraft sized during development is a numbered version and stays reproducible:
[aircraft versions](docs/AIRCRAFT_VERSIONS.md).

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

| Discipline | Model |
|---|---|
| Rotor | Momentum + profile power with tip-Mach rise; propeller-mode efficiency in J. Fitted to full-scale JVX data |
| Machines | McDonald loss model; mass from torque; built from whole units of real products (motor and generator database) |
| Battery | Samsung 50G-shaped equivalent circuit with sag and end-of-life rating |
| Turboshaft | Fixed 1,120 hp deck; lapse in density and temperature; part-power fuel curve |
| Aerodynamics | AeroSandbox AeroBuildup with Scholz corrections; trim drag from the tail load; hover download |
| Weights | AFDD rotorcraft equations (rotor, drive, engine section); NDARC tiltrotor wing with whirl-flutter frequency margins; Raymer GA elsewhere, anchored to a drawn structural layout |
| Thermal | Heat exchanger sized with the aircraft; short-time machine ratings from thermal mass |
| Stability and control | Neutral point, static margin, elevator trim, Cn_β, rudder for a failed rotor; conversion corridor from trim with ruddervator, rotor disc tilt, attitude, edgewise-flow, power and placard limits |
| Redundancy | Lane-out, bus-out and string-out hovers |

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

## What it does not do yet

In rough order of how much each could move the answer:

1. **Whirl flutter is a frequency margin, not a stability analysis.** It enters the sizing as wing torsion and beam
   frequencies per rev (the NDARC practice), not as a coupled rotor-wing stability constraint.
2. **Drag is not anchored to flight data.** The excrescence factor matches NASA NDARC's XV-15 estimate, not flight
   test. Drag drives payload headroom more than anything else.
3. **Weight calibration rests on one complete aircraft.** The fuselage needs a factor of about 2 and flight controls
   about 4. These are the largest sensitivities (about ±540 lb each for ±15 %).
4. **The wing is not closed structurally.** The finite-element check puts jump take-off spar-cap strain at 1.10 of
   the allowable, and wing torsion is not cleanly separated from nacelle modes. The check was last run on an
   earlier version (v3.3.5, 13,038 lb), not the baseline.
5. **The electrical layer is built but off by default.** Inverters, cables and protection add about 330 kg and 3 %
   losses; with every option on, the problem converges only through the multistart strategy.
6. **Failures are single and symmetric.** Double failures did not converge (not shown infeasible), and degraded
   states apply to both rotors, so roll trim is not modelled.
7. **Simplifications still in the baseline:** a conventional tail in sizing (the V-tail is only drawn);
   point-mass trajectories with no rotor disc tilt and no 6-DOF; no rotor in-plane force or rotor-speed schedule
   in the conversion corridor; battery chiller power not modelled.

## Where I would take it next

Approach and effort for each are in [docs/NEXT_STEPS.md](docs/NEXT_STEPS.md).

1. **Whirl flutter inside the optimization:** a coupled rotor-pylon-wing stability model written in CasADi, so the
   optimizer trades wing thickness, spar caps, nacelle station and pylon inertia directly against flutter speed.
2. **Close the wing:** re-run the finite-element check on the baseline, fold the cap-depth and skin corrections
   into the sizing, and identify torsion from mode shapes.
3. **Anchor drag** to the XV-15 power-required curve, then airplane-mode rotor efficiency.
4. **A second weight anchor** for the fuselage and flight controls, ideally an uncrewed fly-by-wire aircraft, to
   replace the 2× and 4× factors.
5. **Electrical layer on by default,** with problem scaling and continuation so the full model converges from one
   start.
6. **Asymmetric and double failures,** including the trade between an interconnect shaft and electrical
   cross-strapping.
7. **Uncertainty on the answer:** propagate the calibration factors to a take-off-weight band instead of a single
   number.
8. **Conversion controls:** rotor in-plane force, a rotor-speed schedule and rotor disc tilt in the trajectory model,
   then linearized models at the corridor trim points as the first control-law step; V-tail in the sizing.

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
