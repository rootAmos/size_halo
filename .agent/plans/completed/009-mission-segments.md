# Mission segments and prescribed missions

Status: COMPLETED 2026-10-02 — continuing the user's request for a simultaneous mission and
sizing loop (2026-10-02); review delegated, self-reviewed against AGENTS.md and
the mission-and-sizing skill.

## Goal and scope

Tier 8, mission levels M0-M3 of the architecture: M0 the Tier 7 flight point;
M1 isolated, independently tested segments (hover, climb, cruise, loiter,
descent) as quasi-steady points with durations; M2 a prescribed multi-segment
mission with fuel burn, mass depletion and battery SOC; M3 a semi-free mission
where cruise speed and per-segment electric power fraction are Opti variables
for minimum fuel on a fixed aircraft. Fuel becomes a mass item. Out of scope:
sizing coupling (Tier 9), trajectory dynamics (M4), battery charging in flight,
reserves beyond a fuel factor and SOC floor, wind, transition dynamics.

## Physics

- Each segment is evaluated at its start mass (conservative quasi-steady).
- Duration: hover/loiter given; climb/descent = |delta h| / rate; cruise =
  distance / V (symbolic if V is a variable). Climb/descent evaluated at the
  mean altitude.
- Fuel = fuel flow x duration; mass_next = mass_start - fuel.
- Battery: chemical energy = OCV I x duration; SOC_next = SOC - E_chem / E_cap
  (Tier 1 battery identity). h_e in [0, 1] (no in-flight charging).
- Hover segments represent vertical take-off/landing phases at T/W = 1.
- Descent thrust must stay non-negative (actuator-disk domain).

## Interfaces

```
vehicle/items.py        FuelLoad(mass_kg, x_m, z_m)
vehicle/aircraft.py     Aircraft.fuel (optional), MassBreakdown.fuel
mission/segments.py     HoverSegment, ClimbSegment, CruiseSegment, LoiterSegment,
                        DescentSegment (.flight_condition(soc), .duration_s())
mission/mission.py      Mission(segments), SegmentResult, MissionResult
                        build_mission(opti, aircraft, aerodynamics, mission,
                                      mass_start_kg, soc_start)
examples/mission_analysis.py
```

`build_mission` is orchestration (creates per-point variables through the
flight point); it adds only per-segment physics equalities. Mission
constraints (fuel sufficiency, SOC floor, margins) are applied by the caller.

## Tests

Segment durations; cruise fuel = flow x distance / V; mass decreases by the
burnt fuel; SOC drop = chemical energy / capacity; all-turbo mission keeps SOC;
energy conservation sum; M3 optimizer lowers fuel versus the prescribed
mission; faster cruise burns more fuel per km beyond the optimum.

## Reference cases

Mission: hover 60 s (0 m), climb to 1000 m at 4 m/s (50 m/s), cruise 100 km at
60 m/s, loiter 20 min at 45 m/s, descent at 3 m/s (50 m/s), hover 60 s; hover
h_e 0.7, other segments h_e 0; payload 300 kg; fuel factor 1.1; final SOC >= 0.3.

## Acceptance

Tests and notebooks pass; prescribed and semi-free missions solve with all
operating margins >= 0; Tier 8 notebook; docs; commit and push.

## Progress and decisions

- 2026-10-02: Plan written. Fuel tank at the common rotor station, so fuel
  burn does not move the CG (hover trim holds over the mission).
- `MassBreakdown` gained a fuel field (zero without a load); the Tier 4 test
  and notebook check that it is zero there.
- SOC floor checked to IPOPT tolerance (1e-7).
- Results: prescribed 32.0 kg fuel / SOC 0.80; semi-free 25.0 kg fuel, cruise
  51.7 m/s, SOC 0.30. Verification: 166 tests; Tier 8 notebook 50/50.

## Deferred

In-flight charging, reserve segments and alternates, wind, trajectory
dynamics (M4), transition corridor, battery thermal limits.
