# Tutorials: size_halo from the bottom up

A notebook series that teaches the sizing model from first principles to the full Halo driver. It is written for engineers
who know multidisciplinary design optimization (NLPs, MDF/IDF/all-at-once formulations, KKT conditions) but not
AeroSandbox, CasADi or this repository. After working through it you should be able to explain any line of
`src/aircraft_closure/` or `examples/halo_sizing.py` in terms of the optimization problem, and you should be able to extend the model.

## How to run

```bash
uv sync --group notebooks
uv run jupyter lab tutorials
```

Every notebook starts with `from tutorial_helpers import *`, which puts `src/`, the repo root (for `examples/`) and `tutorials/` on the
path and loads `asb`, `np` (AeroSandbox's symbolic NumPy), `u` (units), `plt`, a plot style, and a few helpers (`check`, `lb`,
`describe_opti`, `capture_opti`, `cached`, `quiet_solver`). Set `OMP_NUM_THREADS=1` if you run several notebooks at once.

Each notebook ends with `check_summary()`: the `check(...)` lines verify the claims the text makes (closed-form identities,
reproductions of `examples/` drivers and documented results). A notebook that still passes still tells the truth about the code.

## Core series (run in order)

| # | Notebook | Run time | What you can do afterwards |
|---|---|---|---|
| **Part 1: machinery** | | | |
| 00 | [Orientation](00_orientation.ipynb) | 10 s | state the problem as an NLP; place it among MDO formulations; navigate the layers |
| 01 | [AeroSandbox symbolic NumPy](01_aerosandbox_numpy.ipynb) | 5 s | write physics once for numbers and variables; know what breaks a CasADi graph |
| 02 | [`asb.Opti` in depth](02_opti_in_depth.ipynb) | 5 s | scaling (and its inferred default), bounds, duals, the IPOPT log, failures, warm starts |
| 03 | [The sizing paradigm](03_sizing_paradigm.ipynb) | 10 s | MDF vs all-at-once on a toy; smoothing; multipliers as sensitivities; local optima |
| 04 | [Conventions and margins](04_conventions_and_margins.ipynb) | 5 s | read any class; the component contract; normalized, named margins |
| **Part 2: one component at a time** | | | |
| 05 | [Electric machines](05_electric_machines.ipynb) | 5 s | McDonald losses, rubber machines, four mass models, the machine database, a geared-drive trade |
| 06 | [Gearboxes and the turboshaft](06_gearbox_and_turboshaft.ipynb) | 5 s | stage counts, AFDD drive mass, lapse in density and temperature, part-power fuel, regression inversion |
| 07 | [Batteries](07_batteries.ipynb) | 10 s | constant pack, the two-root power equation, the 50G equivalent circuit, why voltage sag sizes the pack |
| 08 | [Rotors](08_rotors.ipynb) | 5 s | actuator disk in thrust and power mode; the JVX-calibrated momentum + profile rotor; the cruise pathology |
| 09 | [Electrical and thermal parts](09_electrical_and_thermal.ipynb) | 5 s | cables and partial discharge, converters, the heat exchanger, lumped temperatures |
| **Part 3: connecting components** | | | |
| 10 | [A coupled point by hand](10_coupled_point_by_hand.ipynb) | 5 s | unknowns, equations, degrees of freedom; energy split; infeasibility |
| 11 | [Ports, topology, margins](11_ports_topology_margins.ipynb) | 5 s | network as data, residuals, multiplicity, redundancy, degraded states, "by construction" |
| **Part 4: the airframe** | | | |
| 12 | [Airframe, mass and closure](12_airframe_mass_closure.ipynb) | 5 s | surfaces, correlations, calibration, installation, `Aircraft`, closure as an equality |
| 13 | [Aerodynamics](13_aerodynamics.ipynb) | 10 s | simple, Scholz, AeroBuildup; blown wing, trim drag, download; cruise closure |
| 14 | [Stability and tail sizing](14_stability_and_tails.ipynb) | 5 s | neutral point, trim, Cn_beta, rudder; tails inside the closure |
| **Part 5: flying it** | | | |
| 15 | [The flight point](15_flight_point.ipynb) | 15 s | `build_flight_point` line by line on a sized Halo; hover, engine-out, cruise, hover ceiling |
| 16 | [Capability requirements](16_requirements.ipynb) | 15 s | requirement points, rubber ratings as variables, what sizes what, what it costs |
| 17 | [Missions](17_missions.ipynb) | 10 s | segments, the SOC/mass chain, energy allocation, sub-segments, the Halo mission |
| 18 | [Coupled sizing](18_coupled_sizing.ipynb) | 15 s | aircraft + mission + requirements + reserves + stability; Jacobian sparsity; duals by name; objectives |
| **Part 6: the Halo** | | | |
| 19 | [The Halo driver](19_halo_driver.ipynb) | 2 min first time | inputs, switches, design vector, aircraft builder, the problem, the result, named sets |
| 20 | [Studies and extensions](20_studies_and_extensions.ipynb) | 2-3 min | starting points, discrete choices, warm-started studies, objectives, extending, testing |
| **After sizing** | | | |
| 21 | [Trim, corridor and trajectories](21_after_sizing_trajectories.ipynb) | 15 s | level-flight trim, the computed conversion corridor, direct-collocation transition and climb |

Tutorial 19 solves the current reference with the fast Scholz aerodynamics (about a minute, within 0.3 % of the 15,179 lb reference)
and caches it in `tutorials/.cache/`; tutorials 20 and 21 reuse it. The cache is keyed on a hash of `src/`, `examples/` and `data/`,
so any code change triggers a fresh solve.

## Deep dives (`deep_dives/`, on the full reference)

These read the current full-fidelity reference design top-down, discipline by discipline, with the three questions a reviewer asks
(why is it built this way, what can't it do, what's next). They call `baseline()`, the full reference solve (10-15 minutes the
first time, then cached).

| Notebook | Topic |
|---|---|
| [D0 Reference tour](deep_dives/D0_reference_tour.ipynb) | the v3.6 baseline: requirements, what sizes it, where the weight goes, how the mission is flown |
| [D1 Optimization](deep_dives/D1_optimization.ipynb) | the paradigm on a toy, then multipliers read off the full solve |
| [D2 Components, ports, topology](deep_dives/D2_components_ports_topology.ipynb) | the network of the sized aircraft; residuals in normal and degraded states |
| [D3 Rotor](deep_dives/D3_rotor.ipynb) | momentum + profile, JVX calibration, the rotor in the sizing |
| [D4 Machines, gearboxes, engine](deep_dives/D4_machines_gearboxes_engine.ipynb) | losses, torque sizing, whole units, gearboxes, the turboshaft |
| [D5 Battery](deep_dives/D5_battery.ipynb) | the equivalent circuit and the engine-out hover on the sized pack |
| [D6 Aerodynamics](deep_dives/D6_aerodynamics.ipynb) | the three models on the sized aircraft; the NDARC drag calibration |
| [D7 Weights and calibration](deep_dives/D7_weights_and_calibration.ipynb) | correlations, XV-15 calibration, the tiltrotor wing, OEW uncertainty |

## Where things are

- [GLOSSARY.md](GLOSSARY.md): terms and symbols used across the series.
- `tutorial_helpers.py`: the shared helpers.
- The model's own documentation: `README.md` (results and next steps), `docs/ARCHITECTURE.md`, `docs/CODING_CONVENTIONS.md`,
  `docs/OPTIMIZATION_PHILOSOPHY.md`, and the decision log `docs/decisions/001-039` (cited in code as "plan NNN").
