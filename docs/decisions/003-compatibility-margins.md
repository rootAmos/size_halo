# Compatibility margins

Status: COMPLETED 2026-10-01 — acceptance met.

## Goal and scope

Tier 3. Express powertrain feasibility as named, normalized margins instead of
hand-written inequality lists, at two levels:

1. Operating margins: each instance's port values against its own limits
   (speed, torque, voltage window, power). Symbolic; the caller constrains
   `margin >= 0` in its Opti or reports them after a solve.
2. Design compatibility margins: adjacent ratings compared across connections
   and buses (shaft power/speed/torque envelopes, bus voltage window and bus
   power capability). Functions of component parameters only, so they remain
   symbolic when ratings become design variables.

Margins report; they never clip, resize or solve.
Out of scope: vehicle/mass (Tier 4), thermal limits, SOC limits (need mission
state, Tier 8), envelope propagation through gearboxes (see Decisions).

## References

`docs/CODING_CONVENTIONS.md` (Compatibility),
`docs/MODEL_INTERFACES.md`, plan 002, `examples/series_hybrid_point.py`.

## Assumptions

- Margin normalization: upper limit `(limit - value) / limit`; lower limit
  `(value - limit) / limit`. `>= 0` is compatible; 0.1 means 10 % headroom.
  Limits are positive (enforced by component docs, not by the margin code).
- Max torque and max speed are independent bounds, never combined into a
  single corner point; rated power is a separate bound.
- Battery terminal voltage range is the voltage at max discharge power and at
  max charge power from V = OCV - I R with V I = P (closed form roots).
- Bus power capability is loss-free and therefore optimistic: generator
  electrical output is bounded by its shaft rating, motor electrical demand by
  its shaft rating. Documented as an upper bound on the true margin.

## Inputs, outputs, parameters

Operating margins: topology + the same `port_values` mapping used for
connection residuals. Design margins: topology only. Outputs: tuples of
`Margin(label, value)`. Report: `margin_report(margins, value_of)` returns
entries sorted from most to least critical (numeric values only).

## Design variables, states, controls

None owned. Ratings may be Opti variables supplied by the caller; margins stay
symbolic. Battery SOC remains caller state and has no margin at this tier.

## Equality and inequality constraints

No equalities. Callers apply `margin.value >= 0` as inequalities. The
reference example replaces its hand-written rating list with operating
margins, keeping the domain constraint `generator.power_electric_W >= 0`.

## Interfaces

```
core/margins.py
    Margin(label, value)
    margin_below(label, value, limit)     # value must stay <= limit
    margin_above(label, value, limit)     # value must stay >= limit
    MarginReportEntry(label, value, compatible)
    margin_report(margins, value_of=identity) -> tuple sorted ascending

powertrain/compatibility.py
    MechanicalEnvelope(max_speed_rad_s, max_torque_Nm, max_power_W)  # None = unbounded
    ElectricalEnvelope(min_voltage_V, max_voltage_V, max_power_W, sets_voltage)
    port_envelope(component, port_name)
    operating_margins(topology, port_values) -> tuple[Margin]
    design_margins(topology) -> tuple[Margin]
```

Design margins per direct shaft connection compare the upstream OUT envelope
with the downstream IN envelope: `(downstream max - upstream max) / downstream
max` for each field both define ("downstream tolerates upstream's maximum"),
i.e. `margin_below(upstream max, downstream max)`, normalized like operating
margins.
Per bus: every voltage-setting source's range within every other port's
window; and `(sum OUT max power - sum IN max power) / sum IN max power`, with
instance counts.

## Symbolic considerations

Only arithmetic and `aerosandbox.numpy.sqrt`. No branching on values; branches
only on `None` fields and component types (structure). Report converts nothing
itself; `value_of` (e.g. `solution.value`) returns numbers.

## Tests

Identities (margin of value == limit is 0; signs above/below; 10 % headroom
gives 0.1); battery voltage-range roots satisfy V I = P; envelope per component;
operating margins numeric and symbolic; design margins of the default series
hybrid (turboshaft 150 kW vs generator 100 kW gives -0.5); count scaling of bus
power; report ordering; topology example reproduces Tier 2 results with
margins as constraints; Opti can use design margins with symbolic ratings.

## Reference cases

1. Tier 2 reference point with margins as constraints: identical results.
2. Design report of the default series hybrid (records the engine/generator
   rating mismatch as a negative margin, without resizing).
3. Max thrust per rotor at 20 % battery share subject to all operating margins;
   the binding margin is identified.
4. Minimum generator rating making all design margins non-negative (an Opti
   using margins as constraints): equals the turboshaft rating, 150 kW.

## Implementation sequence

1. `core/margins.py` + tests.
2. `powertrain/compatibility.py` envelopes, operating and design margins + tests.
3. Example uses operating margins; integration tests.
4. `notebooks/tier3_compatibility_margins/` executed notebook.
5. Docs, roadmap, plan completion; commits.

## Acceptance criteria

All tests pass; reference cases 1-4 reproduce; Tier 0-2 notebooks still pass;
Tier 3 notebook executes with all checks passing; no component resized or
clipped by margin code.

## Progress and decisions

- 2026-10-01: Plan written. Gear-ratio envelope propagation is deferred: no
  component downstream of a gearbox currently declares speed or torque limits,
  so a transform would have nothing to compare (project rule 11, docs/CODING_CONVENTIONS.md). Gearbox output
  envelope carries only its own power rating x efficiency.
- Design margins are normalized by the downstream limit, matching operating
  margins (turboshaft/generator gives -0.5, not -1/3 as first drafted).
- Example restructured: `build_point_problem(opti, ...)` returns the coupled
  expressions so callers choose thrust constraint or objective; optional
  `motor` override supports the rating sweep.
- Default motor and gearbox input ratings (both 100 kW) bind together at max
  thrust; tests accept the tie rather than an arbitrary single label.
- Verification: 77 unittest cases pass (19 new); Tier 3 notebook 47/47 checks;
  reference cases 1-4 reproduce (unchanged point; -0.5 engine/generator margin;
  max thrust 5284 N limited by motor/gearbox; minimum generator 150 kW).

## Deferred

Gear-transformed speed/torque envelopes, current-limit fields on machines,
thermal and SOC margins, failure-case margins, margin-driven sizing (Tier 9).
