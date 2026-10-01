---
name: powertrain-modeling
description: >
  Use when implementing or modifying hybrid-electric powertrain components,
  propulsors, powertrain topology, hybridization, component limits, maps/decks,
  or compatibility checks.
---

# Powertrain Modeling

## Component philosophy

Initial common methods:

- `get_mass()`
- `get_limits(...)`
- `evaluate(...)`

Do not add lifecycle or reporting methods without a real need.

Each component should be independently usable and testable.

## Motor

Stable operating inputs:

- `speed_rad_s`
- `torque_Nm`
- `voltage_V`

Do not make constant efficiency the public API.

Motor loss fidelity may evolve behind the component:

Simple loss
→ analytic loss
→ lookup map

## Battery

Keep the interface physically meaningful and suitable for later equivalent-circuit or cell-map models.

Preserve clean charge/discharge sign conventions and document them.

## Turboshaft

Use separate implementations when higher-fidelity engine-deck inputs materially differ from the simple analytic model.

## Propulsor

Initial model:

`ActuatorDiskPropulsor`

Do not require RPM.

Higher-fidelity implementations may be separate classes:

- `AdvanceRatioPropulsor`
- `BEMPropulsor`
- `PropulsorDeck`

Keep aircraft-facing outputs physical and useful.

## Graph

The future powertrain graph should support:

- add
- connect
- multiplicity
- mechanical/electrical/fuel ports
- buses
- gearboxes
- splitters/combiners

The graph describes topology only.

Never introduce a second solver system.

## Compatibility

Adjacent-component checks should eventually compare:

- speed envelope
- torque envelope
- voltage envelope
- current envelope
- power capability

Transform mechanical envelopes through gear ratios before comparison.

Do not assume maximum torque and maximum speed occur simultaneously.

Compatibility checks report margins; they do not silently resize components.

## Tests

For simple analytic models, test:

- identities
- limiting cases
- trends
- signs
- symbolic compatibility

For maps/decks, test:

- grid nodes
- interpolation
- bounds
- out-of-domain policy
- reference operating points
