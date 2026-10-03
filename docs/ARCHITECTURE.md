# Architecture

Diagrams: [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) (layering, fidelity scaling, sizing/trajectory/6-DOF levels, coupled sizing problem).

## Purpose

This project is a lightweight aircraft-closure framework for unmanned hybrid-electric VTOL aircraft.

It composes AeroSandbox rather than replacing it.

AeroSandbox owns:

- symbolic mathematics
- `Opti`
- atmospheric state
- operating points
- existing geometry primitives
- higher-order aerodynamic models where suitable
- trajectory/dynamics capability where suitable

This project owns the reusable architecture needed to close the aircraft across configuration, powertrain, mass, controls, requirements, and mission constraints.

## Layering

Preferred dependency direction:

Sizing / mission orchestration
    ↓
Aircraft / vehicle assembly
    ↓
Disciplines
    ↓
Subsystem assemblies
    ↓
Components
    ↓
Maps / decks / empirical data

Lower layers must not import or depend on higher layers.

## Vehicle

Vehicle objects represent physical aircraft subsystems.

Examples:

- Wing
- Fuselage
- HorizontalTail
- VerticalTail
- Payload
- LandingGear
- Systems
- Powertrain installation

Physical objects own their own geometric and mass properties.

Prefer:

`aircraft.wing.area_m2`

over:

`geometry["wing"]["area"]`

Avoid cross-cutting god objects such as a giant `Geometry`, `Mass`, or `Area` container.

Aircraft-level aggregation may provide methods such as:

- `get_mass()`
- `get_cg()`

Discipline models should generally return results rather than silently mutate vehicle state.

## Powertrain

Powertrain is a component network.

Early components:

- Motor
- Generator
- Battery
- Turboshaft
- Gearbox
- ActuatorDiskPropulsor

Future components may include:

- inverter
- electrical bus
- shaft splitter
- shaft combiner
- thermal subsystem elements

The long-term topology layer should feel similar to an OpenMDAO Group:

- `add(...)`
- `connect(...)`

but remain much lighter.

It may own:

- topology
- typed ports
- connections
- multiplicity
- bus/splitter/combiner relationships

It must not own:

- optimization
- nonlinear solution
- execution scheduling
- design variables
- constraints
- objectives

CasADi/AeroSandbox resolves coupled equations.

## Ports and connectivity

Generic connectivity belongs in `core/`, not inside the powertrain package.

Initial physical domains:

### Mechanical
Eventually carries:
- `speed_rad_s`
- `torque_Nm`

### Electrical
Eventually carries:
- `voltage_V`
- `current_A`

### Fuel
Eventually carries appropriate fuel-flow state.

Power is normally derived from physical port variables where appropriate.

Cyclic connectivity is allowed.

The graph describes a cycle but does not solve it internally.

Coupling variables should be exposed to `asb.Opti` and resolved through explicit equality constraints.

## Powertrain fidelity

Use composition when fidelity changes only a submodel.

Example:

Motor
    + SimpleMotorLossModel
    + AnalyticMotorLossModel
    + MotorMapLossModel

The motor operating interface should remain physically based on:

- speed
- torque
- voltage

Do not make constant efficiency the stable public model.

Use separate implementations when fidelity changes the governing physics and required inputs.

Examples:

- ActuatorDiskPropulsor
- AdvanceRatioPropulsor
- BEMPropulsor
- PropulsorDeck

and likely:

- SimpleTurboshaft
- EngineDeckTurboshaft

## Structural discipline

Structures is intentionally a mass model.

V0 uses empirical mass buildup such as Raymer-style equations.

Do not add stress analysis, deflection analysis, beam sizing, or FEM unless the project scope is explicitly changed.

## Aerodynamics

V0 custom aerodynamics are deliberately low fidelity:

- linear lift curve
- constant or built-up parasite drag
- induced drag

The architecture should leave hooks for:

- HTP/VTP aerodynamic contribution
- control-surface coefficient increments
- DATCOM-style parasite drag
- trim drag
- propeller scrubbing/interference
- propwash lift augmentation

Higher fidelity should preferentially use AeroSandbox capability.

Intended aero sources:

1. Simple analytic model
2. AeroSandbox-native model
3. AeroSandbox airfoil/polar-driven model
4. CFD-derived lookup or surrogate

CFD should not normally run live inside the sizing optimization.

## Controls

V0 uses a conventional horizontal and vertical tail.

Sizing concerns:

- longitudinal static stability
- pitch trim/control authority
- directional stability
- failed-propulsor asymmetric-yaw control
- elevator sizing
- rudder sizing

V-tail is deferred.

Later V-tail support should transform required pitch/yaw authority into:

- canted-tail area
- cant angle
- ruddervator allocation
- control mixing

## Requirements and missions

Requirements and missions are separate.

Requirements define capability:
- payload
- climb rate
- max speed
- ceiling
- hover condition
- operating envelope
- failed-propulsor cases

Mission defines what is flown.

Do not duplicate AeroSandbox flight physics.

Mission support should evolve:

M0 — single operating point
M1 — isolated segment
M2 — prescribed multi-segment mission
M3 — semi-free mission
M4 — fully optimized AeroSandbox trajectory

A segment must be testable independently before being added to a full mission.

## Energy management

Powertrain topology defines what energy sources exist.

Energy management defines how their power is shared.

Future strategies include:

- prescribed electric fraction
- prescribed thermal fraction
- prescribed electric power
- prescribed thermal power
- optimized power split

A useful canonical dimensionless variable is:

`hybridization_electric = power_electric / power_total`

Energy-management strategy may vary across mission segments.

## Aircraft closure

The eventual closure loop couples:

Vehicle/configuration
    ↕
Structural mass
    ↕
Aerodynamics
    ↕
Controls
    ↕
Propulsor
    ↕
Powertrain
    ↕
Mission / requirements
    ↕
Energy management

through one explicit AeroSandbox optimization problem.

The aircraft-level formulation should remain readable. Major sizing constraints and objective terms should not be hidden inside opaque orchestration classes.
