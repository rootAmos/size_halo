# Vehicle geometry, CG and empirical mass closure

Status: COMPLETED 2026-10-01 — acceptance met.

## Goal and scope

Tier 4. Add physical vehicle components that own their geometry, an aircraft
assembly that aggregates mass and CG, and a caller-owned mass closure:
`mass_takeoff_kg == aircraft.get_mass(condition)` as one explicit Opti
equality, with no fixed-point iteration. Structures is a mass model only.

In scope: `vehicle/` package with Wing, HorizontalTail, VerticalTail,
Fuselage, LandingGear, Systems, Payload, PowertrainInstallation and Aircraft;
a structural design condition; a reference closure example with CG placed by
a wing-position variable; tests; Tier 4 notebook.

Out of scope: aerodynamics (Tier 5; lift-to-drag is a fixed assumption here),
stability, trim and tail sizing (Tier 6), requirements such as hover thrust
margins (Tier 7), fuel load and missions (Tier 8), inertia beyond point masses.

## References

AeroSandbox `library/weights/raymer_general_aviation_weights` (Raymer,
Aircraft Design: A Conceptual Approach, 5th ed., sec. 15.3.3), `asb.Wing`,
`asb.Fuselage`, `asb.Airplane`, `asb.MassProperties`;
`.agents/skills/structural-mass-modeling/SKILL.md`; `docs/ARCHITECTURE.md`
(Vehicle, Structural discipline); plan 003.

## Assumptions

- Raymer general-aviation correlations (SI wrappers inside AeroSandbox) apply
  to a light unmanned VTOL of order 1000-3000 kg. Each component has a
  dimensionless `mass_factor` (default 1) for calibration; no calibration to
  Halo exists.
- Geometry: unswept trapezoidal surfaces, straight cylinder fuselage with
  conical nose and tail. Body axes: x aft from the nose, z up.
- Component CGs: lifting surfaces at 40 % of the mean aerodynamic chord from
  the MAC leading edge; fuselage at 45 % length; landing gear at the
  mass-weighted main/nose positions; point items at stated positions.
- Masses are point masses for CG; no inertia tensor at this tier.
- Powertrain installation mass = sum(count x component.get_mass()) x
  installation factor; each topology instance is located once.
- Design condition: ultimate load factor 5.7 (3.8 x 1.5), cruise speed and
  altitude for dynamic pressure, cruise lift-to-drag assumed (fuselage
  correlation exponent -0.072).

## Inputs, outputs, parameters

Inputs: component geometry and positions, payload, powertrain topology and
instance locations, `StructuralDesignCondition(mass_design_kg, ...)`.
Outputs: `MassBreakdown` (one `asb.MassProperties` per physical item plus
total), `get_mass`, `get_cg_x_m`, `to_asb()` airplane geometry.

## Design variables, states, controls

Owned by the caller. Reference example: `mass_takeoff_kg` and the wing
leading-edge position `x_le_wing_m`. Rotor positions follow the wing.

## Equality and inequality constraints

Caller equalities: mass closure `mass_takeoff_kg - aircraft.get_mass(
condition(mass_takeoff_kg)) = 0`; CG placement `x_cg = x_le_wing + 0.25 MAC`
(reference choice, not a stability criterion). Bounds on variables only.

## Interfaces

```
vehicle/condition.py   StructuralDesignCondition(mass_design_kg,
                       load_factor_ultimate=5.7, velocity_cruise_m_s,
                       altitude_cruise_m, lift_to_drag_cruise)
                       .operating_point() -> asb.OperatingPoint
vehicle/surfaces.py    Wing, HorizontalTail, VerticalTail
                       .span_m(), .chord_root_m(), .to_asb(),
                       .get_mass_properties(condition)
vehicle/fuselage.py    Fuselage.to_asb(), .get_mass_properties(condition,
                       distance_wing_to_tail_m)
vehicle/items.py       LandingGear, Systems, Payload
vehicle/powertrain_installation.py
                       InstalledInstance(instance_name, x_m, z_m)
                       PowertrainInstallation(topology, locations,
                       installation_factor).get_mass_properties()
vehicle/aircraft.py    Aircraft(...).get_mass_breakdown(condition),
                       .get_mass(condition), .get_cg_x_m(condition), .to_asb()
```

Vehicle depends on powertrain (lower layer); powertrain never imports vehicle.

## Symbolic considerations

All geometry and correlations go through AeroSandbox objects that accept
CasADi values. Python branching only on structure (counts, None). No float()
outside post-solve reporting in examples.

## Tests

Identity with AeroSandbox correlations (no duplicated formulas); geometry
(span, area, MAC, AR recovered from asb); scaling exponents (wing mass ratio
for doubled design mass x load factor = 2^0.49; tail 0.414/0.376); mass_factor
linearity; CG aggregation equals hand mass-weighted mean; payload shift moves
CG predictably; powertrain installation mass with counts; location coverage
errors; symbolic geometry and design mass; closure solve with zero residual,
growth factor dMTOM/dpayload > 1; CG placement constraint satisfied.

## Reference cases

1. Reference aircraft (12 m2 AR 9 wing, 7 m fuselage, 300 kg payload, four
   rotor strings from the Tier 2/3 reference topology) closes in one solve.
2. Payload sweep 0-500 kg: MTOM rises monotonically with growth factor > 1.
3. CG placed at 25 % MAC by solving for the wing position.

## Implementation sequence

1. `vehicle/` components and condition + tests.
2. Aircraft assembly and powertrain installation + tests.
3. `examples/aircraft_mass_closure.py` + integration tests.
4. `notebooks/tier4_vehicle_mass_closure/` executed notebook (three-view, mass
   breakdown, payload growth).
5. Docs, roadmap, plan completion; commits.

## Acceptance criteria

All tests pass; reference cases 1-3 reproduce; closure is one explicit
equality with no hidden loop; Tier 0-3 notebooks still pass; Tier 4 notebook
executes with all checks passing.

## Progress and decisions

- 2026-10-01: Plan written. Vehicle components expose
  `get_mass_properties(...)` returning `asb.MassProperties` so CG aggregation
  reuses AeroSandbox instead of re-implementing mass-weighted sums.
- Wing and horizontal tail share planform geometry through a small private
  base class; the vertical tail is a single non-symmetric fin.
- Aircraft CG accessor named `get_cg_x_m` (x only; z is in the breakdown).
- The closure example imports the topology example, so it runs as a module
  (`python -m examples.aircraft_mass_closure`).
- AeroSandbox's three-view restyles matplotlib globally; the notebook draws it
  inside `plt.rc_context()`.
- Results: MTOM 1552.4 kg (empty 1252.4 kg, powertrain 880 kg), CG at 25 % MAC
  with wing LE 3.233 m; growth factor about 1.11; residual about 2e-13 kg.
- Verification: 102 unittest cases pass (25 new); Tier 4 notebook 62/62.

## Deferred

Fuel load and tanks (Tier 8), inertia tensors, wing-mounted fuel relief,
tilt-mechanism and nacelle structure correlations, advanced-composite factors
as defaults, aero-coupled lift-to-drag (Tier 5), CG envelopes and tail sizing
(Tier 6).
