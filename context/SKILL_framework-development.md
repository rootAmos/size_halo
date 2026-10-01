---
name: framework-development
description: >
  Use when creating or changing repository architecture, core framework primitives,
  model interfaces, symbolic data flow, naming conventions, tests, or AeroSandbox
  integration for the aircraft-closure project.
---

# Framework Development

## Goal

Keep the framework lightweight, explicit, symbolic, and easy for an aircraft engineer to read.

## Required principles

- AeroSandbox/CasADi owns optimization and coupled solves.
- Do not recreate existing AeroSandbox physics or infrastructure without a concrete gap.
- Components build equations.
- High-level sizing code owns variables, constraints, and objectives.
- Avoid hidden nonlinear iterations.
- Keep dependency direction downward.
- Preserve symbolic compatibility.
- Prefer SI units.
- Append units to dimensional scalar names.
- Avoid global ALL_CAPS constants.
- Prefer physical-component ownership over quantity-bucket ownership.
- Prefer explicit named result objects over stable anonymous dictionaries.
- Do not add abstractions for hypothetical future needs.

## Before changing architecture

Read:
- `docs/ARCHITECTURE.md`
- `docs/MODEL_INTERFACES.md`
- `docs/FIDELITY_ROADMAP.md`

If the change alters public interfaces, adds a framework tier, or spans several disciplines, create or update an ExecPlan before implementation.

## Testing

Framework primitives should be tested independently of aircraft physics.

Examples:
- port compatibility
- connection validity
- multiplicity
- graph symbolic propagation
- clear failure for incompatible connections

Do not hide physical sizing behavior inside graph mechanics.
