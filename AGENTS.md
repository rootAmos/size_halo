# Aircraft Closure Framework

This repository builds a lightweight multidisciplinary sizing and closure framework for unmanned hybrid-electric VTOL aircraft.

AeroSandbox and CasADi own symbolic mathematics, optimization, and coupled solves. This repository adds the aircraft-closure architecture that is missing around them.

## Non-negotiable rules

1. Do not duplicate AeroSandbox functionality without a concrete reason.
2. Preserve AeroSandbox/CasADi symbolic compatibility.
3. Components build equations; `asb.Opti` owns optimization variables, constraints, objectives, and coupled solves.
4. Do not hide iterative convergence loops when the same coupling can be exposed to CasADi.
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

Structures in the sizing loop are limited to mass estimation. Structural analysis (stress, deflection,
FEA via exported CalculiX/Nastran decks) is in scope as a downstream check outside the sizing loop
(user decision 2026-10-04); it must never be imported by sizing code.

## Powertrain

Initial component methods should be limited to:

- `get_mass()`
- `get_limits(...)`
- `evaluate(...)`

Do not add `validate()` or `summary()` without a concrete use case.

When fidelity changes but operating inputs remain stable, prefer interchangeable submodels.

When fidelity materially changes the required operating inputs or governing physics, use a distinct implementation class.

The future graph layer may describe components, ports, connections, multiplicity, buses, splitters, combiners, and gearboxes.

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

For new framework tiers, interface changes, multi-discipline changes, or graph/optimization architecture changes, create an ExecPlan following `.agent/PLANS.md`.

Implement only the active plan. Do not opportunistically advance into later tiers.

Read these documents when relevant:

- `docs/ARCHITECTURE.md`
- `docs/MODEL_INTERFACES.md`
- `docs/FIDELITY_ROADMAP.md`
- `docs/TESTING_STRATEGY.md`
- `docs/OPTIMIZATION_PHILOSOPHY.md`
