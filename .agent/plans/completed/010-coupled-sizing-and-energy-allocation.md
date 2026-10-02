# Coupled aircraft sizing, mission optimization and energy allocation

Status: COMPLETED 2026-10-02 — the user's target ("size a simultaneous mission optimization
and aircraft sizing loop", 2026-10-02); review delegated, self-reviewed
against AGENTS.md, ARCHITECTURE (Aircraft closure) and the mission-and-sizing
skill.

## Goal and scope

Tier 9. One explicit `asb.Opti` that simultaneously sizes the aircraft and
optimizes the mission and its energy allocation:

- design variables: take-off mass, fuel load, wing position and area, both
  tail areas, motor and generator peak torques (rubber machines), turboshaft
  rating, battery discharge power and energy, rotor disk area;
- mission/operational variables: cruise speed and the electric power fraction
  of every segment (energy allocation), plus per-point states from the flight
  points;
- constraints: mass closure (structural design condition fed by the solved
  cruise L/D), hover pitch trim, Tier 7 requirements at MTOM, the Tier 8
  reference mission (fuel load >= 1.1 x burnt, landing SOC >= 0.3),
  static margin >= 10 %, Cn_beta >= 0.06 /rad, failed-rotor rudder <= 20 deg at
  1.2 V_stall, and every operating margin >= 0;
- objectives: minimum MTOM (reference) and minimum fuel (trade case).

The formulation lives in `examples/coupled_sizing.py`, readable top to bottom
(AGENTS: major sizing constraints are not hidden in orchestration classes).
Out of scope: engine lapse and maps (Tier 10), trajectory dynamics (M4),
transition, multiple missions, cost objectives.

## Assumptions

All earlier tier assumptions carry over. Wing aspect ratio fixed at 9 (span
not limited). Rotor disk area <= 15 m2. Battery state window 0.95 -> 0.30.
No in-flight charging. Cruise speed bounded 40-85 m/s.

## Tests

Every constraint satisfied at the optimum (closure residual, SOC floor, fuel
reserve, requirement and mission margins, stability margins); the solution is a
fixed point of the mass buildup; min-fuel case burns less fuel and carries a
larger battery than min-MTOM; longer cruise grows MTOM; energy-allocation
fractions are within [0, 1].

## Reference cases

1. Min MTOM with the Tier 7 requirement set and Tier 8 reference mission.
2. Min fuel, same constraints.
3. Cruise-distance sweep (50, 100, 150 km) at min MTOM.

## Acceptance

Tests and all notebooks pass; Tier 9 notebook explains which constraints bind
and how energy is allocated; docs; commit and push.

## Progress and decisions

- 2026-10-02: Plan written. Uses the Tier 7 flight point and Tier 8 mission
  builders unchanged; stability terms reuse Tier 6.

## Deferred

Engine decks and lapse, motor maps (Tier 10), multi-mission design, cost,
trajectory optimization (M4), battery degradation, thermal management.
