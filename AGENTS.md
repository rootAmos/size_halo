# Aircraft Closure Framework

This repository builds a lightweight multidisciplinary sizing and closure framework for unmanned hybrid-electric VTOL aircraft.

AeroSandbox and CasADi own symbolic mathematics, optimization, and coupled solves. This repository adds the aircraft-closure architecture that is missing around them.

## Non-negotiable rules

1. Do not duplicate AeroSandbox functionality without a concrete reason.
2. Preserve AeroSandbox/CasADi symbolic compatibility.
3. Components build equations; `asb.Opti` owns optimization variables, constraints, objectives, and coupled solves.
4. Do not hide iterative convergence loops when the same coupling can be exposed to CasADi. Numerical checks on a sized aircraft (an eigenvalue scan, a bisection for a flutter speed, an FE run) may iterate, provided they say so in their docstring, run outside the optimization and keep the model they evaluate symbolic.
5. Use SI units internally.
6. Append units to dimensional scalar variable names wherever practical.
7. Avoid global ALL_CAPS constants.
8. Prefer explicit classes/dataclasses over stable anonymous dictionaries.
9. Organize state around physical aircraft components, not quantity buckets.
10. Keep the simplest useful model available even after higher-fidelity models are added.
11. Do not add an abstraction until there is a concrete need for it.
12. Keep dependencies flowing from high-level orchestration toward low-level physics, never the reverse.

## Naming

Examples of preferred dimensional names:

- `mass_motor_kg`
- `power_rated_motor_W`
- `torque_motor_Nm`
- `speed_motor_rad_s`
- `voltage_bus_V`
- `current_battery_A`
- `area_wing_m2`
- `altitude_m`
- `velocity_m_s`

Dimensionless quantities do not need unit suffixes:

- `efficiency`
- `soc`
- `mach`
- `cl`
- `cd`
- `advance_ratio`
- `hybridization_electric`

Preferred qualifiers include:

- `max_`
- `min_`
- `rated_`
- `req_`
- `delta_`
- `ref_`

## Symbolic math

Use `aerosandbox.numpy` for calculations that may receive symbolic values.

Avoid:

- `float()` on symbolic values
- ordinary NumPy operations on symbolic values
- Python branching on symbolic expressions
- hidden nonlinear iterations inside models

## Architectural ownership

Physical objects own their properties.

Discipline models compute behavior.

Assemblies compose components.

Aircraft sizing and missions couple disciplines through AeroSandbox.

The main disciplines are:

- vehicle/configuration
- powertrain/propulsion
- aerodynamics
- structural mass
- controls
- requirements / mission integration
- energy management

Structural mass in the sizing is handbook-based (AFDD/NDARC, Raymer) with explicit calibration factors. FE (CalculiX/Nastran export), strength margins and aeroelasticity (whirl flutter) are checks on the sized aircraft unless a plan makes them explicit sizing constraints. State FE results as margins (a negative margin), never as a bare ratio.

## Powertrain

Initial component methods should be limited to:

- `get_mass()`
- `get_limits(...)`
- `evaluate(...)`

Do not add `validate()` or `summary()` without a concrete use case.

When fidelity changes but operating inputs remain stable, prefer interchangeable submodels.

When fidelity materially changes the required operating inputs or governing physics, use a distinct implementation class.

The powertrain graph (typed ports) describes components, ports, connections, multiplicity, buses, splitters, combiners, and gearboxes.

The graph must never own optimization or convergence.

## Testing

Analytic models require tests for:

- closed-form identities
- limiting cases
- physical trends
- sign conventions
- AeroSandbox/CasADi symbolic compatibility

Map/deck models additionally require:

- grid-node recovery
- interpolation tests
- boundary behavior
- explicit extrapolation/clamping policy
- reference operating points

## Planning

For new framework tiers, interface changes, multi-discipline changes, or graph/optimization architecture changes, write the plan document first: `docs/decisions/<number>-<slug>.md` with Goal and scope, References, Model, Verification, Assumptions and a Progress log (see `docs/decisions/040-whirl-flutter-model.md`). Update its progress log with results as the work lands.

Implement only the active plan. Do not opportunistically advance into later tiers.

New behaviour goes behind an option (an `HaloAssumptions` field or a new function). The default sizing and every existing reference result stay unchanged unless the plan says otherwise.

Read these documents when relevant:

- `docs/CODING_CONVENTIONS.md`
- `docs/ARCHITECTURE.md`
- `docs/MODEL_INTERFACES.md`
- `docs/FIDELITY_ROADMAP.md`
- `docs/TESTING_STRATEGY.md`
- `docs/OPTIMIZATION_PHILOSOPHY.md`

## Skills

Discipline guidance lives in `.claude/skills/<name>/SKILL.md`. Read the relevant one before working in that area:

- `framework-development`: architecture, interfaces, symbolic data flow, tests
- `aircraft-aero`: aerodynamic models and AeroSandbox aero integration
- `powertrain-modeling`: components, topology, hybridization, maps and decks, compatibility
- `mission-and-sizing`: mission analysis and the coupled sizing formulation
- `control-surface-sizing`: tails, controls, stability and trim
- `structural-mass-modeling`: structural mass buildup and calibration
- `agent-orchestration`: how many subagents, which model, run-directly rules, handoffs

## Working in this repository

- One feature, one git worktree and branch, created from `origin/main`. Stage explicit paths only (`git add <paths>`, never `-A` or `.`). Never use bare `git stash`. Never edit another worktree or the main checkout from a feature worktree.
- Environment: `uv sync --system-certs` (plain `uv sync` fails on this machine's TLS certificates). A worktree without its own `.venv` can use another worktree's interpreter with `PYTHONPATH=src;.` so its own code wins.
- Always set `OMP_NUM_THREADS=1` when solves or tests run in parallel; threaded BLAS makes IPOPT fail spuriously.
- Full `solve_halo_sizing` runs take minutes and several hundred MB. Start from an earlier result (`initial=`) where possible, keep the number of solves small, and check free memory before running several at once.
- Run targeted `unittest` modules for what you change; the full suite takes 10-17 minutes. Do not re-run the notebooks (`*.ipynb`); the user runs them.
- Never kill a process you did not start.
- Commits: `<type>(<area>): <what> (plan <number>)`, with the reason in the body. Open a PR against `main`; merge only with the user's approval.
