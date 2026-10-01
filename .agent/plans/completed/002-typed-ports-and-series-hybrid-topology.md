# Typed ports, connections and series-hybrid topology

Status: ACTIVE — approved 2026-10-01 with the proposed options.

## Goal and scope

Tier 2. Describe powertrain networks as data: components with typed physical
ports, connections between ports, electrical junctions (buses), and component
multiplicity. Rebuild the series-hybrid reference point on top of that
description and reproduce the Tier 1 result. The topology layer describes the
network; it does not create variables, add constraints or solve.

In scope: `core/` generic ports, junctions, connections and topology; a
powertrain port declaration for each of the six components; a series-hybrid
topology builder; a function that turns connections plus caller-supplied port
values into residual expressions; tests and an updated reference example.

Out of scope (Tier 3+): compatibility margins and reports, splitters, combiners
or multi-input gearboxes, inverters, thermal ports, fuel tanks, aircraft
geometry or mass, missions, energy-management strategies.

## References

`docs/ARCHITECTURE.md` (Ports and connectivity, Powertrain),
`docs/OPTIMIZATION_PHILOSOPHY.md`, `docs/MODEL_INTERFACES.md`,
`examples/series_hybrid_point.py`, completed plan 001.

## Assumptions

- Each port has one physical domain and one nominal power-flow direction.
- Mechanical ports carry `speed_rad_s`, `torque_Nm`; electrical ports carry
  `voltage_V`, `current_A`; fuel ports carry `fuel_flow_kg_s`.
- Positive flow means power (or fuel) flowing in the port's nominal direction.
  Battery charging is negative battery-port current, consistent with Tier 1.
- Shaft connections are one-to-one. Electrical buses are the only many-to-many
  junction at this tier. Fuel ports may stay unconnected (boundary ports).
- Multiplicity `count` means `count` identical instances sharing one set of
  port values (symmetric operation). Asymmetric/failed-instance cases are
  deferred.

## Inputs, outputs, parameters

Inputs to the topology: component instances, names, counts, connections.
Inputs to residual generation: a caller-built mapping from `"instance.port"` to
port values (numeric or CasADi) — normally pulled from component results.
Outputs: lists of residual expressions with labels; no constraints applied.
Parameters: component parameters remain on the components.

## Design variables, states, controls

None owned by this tier. The caller keeps creating `opti.variable(...)` for
torques, currents, induced velocity, etc. Battery SOC remains caller state.

## Equality and inequality constraints

Generated as residual expressions, applied by the caller:

- Shaft connection: `speed_a - speed_b`, `torque_a - torque_b`.
- Electrical bus: equal voltage at every attached port; Kirchhoff current law
  `sum(count_i * sign_i * current_i) = 0` with sign from port direction.
- Fuel connection: `fuel_flow_a - fuel_flow_b` (only if a fuel port is wired).

Inequalities (ratings, voltage windows) stay with the caller exactly as in
Tier 1; margins become Tier 3.

## Interfaces

```
core/ports.py
    Domain           enum: MECHANICAL, ELECTRICAL, FUEL
    Direction        enum: IN, OUT
    PortSpec         frozen dataclass(name, domain, direction)
    MechanicalPortValue(speed_rad_s, torque_Nm)
    ElectricalPortValue(voltage_V, current_A)
    FuelPortValue(fuel_flow_kg_s)

core/topology.py
    Topology
        add(name, component, ports, count=1) -> name
        add_bus(name, domain=ELECTRICAL) -> name
        connect("a.port", "b.port" | "bus") -> Connection
        instances / connections / buses (read-only views)
    connection_residuals(topology, port_values) -> list[Residual(label, expr)]

powertrain/ports.py
    port_specs_for(component) -> tuple[PortSpec, ...]   (type dispatch table)

powertrain/topologies.py
    build_series_hybrid(motor, generator, battery, turboshaft, gearbox,
                        propulsor, count_rotors=1) -> Topology
```

Components are not modified; port declarations live beside them in
`powertrain/ports.py` so the Tier 1 interface (`get_mass`, `get_limits`,
`evaluate`) is unchanged. `Topology.connect` validates domain match and
direction compatibility at build time with plain Python (structure is never
symbolic). No execution ordering, no variable creation, no solve.

## Symbolic considerations

Residual generation uses only `+`, `-`, `*` by numeric counts and
`aerosandbox.numpy` sums, so CasADi expressions pass through unchanged. No
branching on port values. Counts are Python ints (topology, not design).

## Tests

- Unit: domain mismatch rejected; direction mismatch rejected; duplicate names
  rejected; unknown port rejected; second connection on a shaft port rejected.
- Residuals: numeric balanced bus gives zero; unbalanced bus gives the exact
  current imbalance; multiplicity scales current; shaft residuals are exact.
- Symbolic: residuals built from Opti variables solve in a toy bus problem.
- Integration: series-hybrid via topology reproduces Tier 1 results
  (89.286 kW rotor shaft, 18.653 kW battery, 0.005852 kg/s fuel) within 1e-6
  relative tolerance for `count_rotors=1`.
- Multiplicity: `count_rotors=4` at 4x total thrust gives the same per-rotor
  state and 4x bus motor current.

## Reference cases

1. Tier 1 series-hybrid hover point (exact reproduction).
2. Same point with four rotor/motor/gearbox strings on one bus.

## Implementation sequence

1. `core/ports.py`, `core/topology.py` with unit tests.
2. `powertrain/ports.py` declarations and tests.
3. `connection_residuals` with numeric and symbolic tests.
4. `build_series_hybrid` and rewrite `examples/series_hybrid_point.py` to use
   it (keep the explicit Tier 1 variant as `series_hybrid_point_explicit.py`
   per rule 10).
5. Add `notebooks/tier2_powertrain_topology/topology_verification.ipynb`
   (one executed notebook per tier, outputs kept): port/bus rules, residual
   identities, multiplicity scaling, both reference cases, unit test suite.
6. Update ARCHITECTURE/MODEL_INTERFACES/IMPLEMENTATION_NOTES/roadmap; move
   plan to `completed`.

## Acceptance criteria

All existing and new tests pass; both reference cases reproduce; topology code
creates no Opti variables or constraints; components unchanged; Tier 2
notebook executes with all checks passing; docs updated.

## Resolved decisions

1. `connection_residuals` lives in `core/` and returns labelled residual
   expressions; the caller applies them with `opti.subject_to`.
2. Multiplicity is symmetric shared state; per-instance expansion is deferred.
3. Port declarations use a dispatch table in `powertrain/ports.py`; the Tier 1
   component interface is unchanged.

## Progress and decisions

- 2026-10-01: Plan drafted after Tier 0-1 completion; user approved the
  proposed options and requested one verification notebook per tier.

## Deferred

Compatibility margins (Tier 3), splitters/combiners, inverters, thermal and
fuel-tank ports, asymmetric multiplicity, failed-propulsor topology, graph
visualisation.
