# Plan 039: computed conversion corridor and trim

Status: PARTIAL 2026-10-05. Corridor and trim implemented and merged to `main`; linearized models, rotor H-force and
rotor speed schedule open (roadmap Tier 24).

## Goal

Replace the assumed XV-15-shaped conversion corridor with one computed from the sized aircraft: at each nacelle
angle, the airspeeds for which a level-flight trim exists inside stated limits, the limit that sets each side, and
a trim schedule (attitude, tail deflection, cyclic, thrust, power) through it. This is the first controls product
for the transition and the input a control-law or 6-DOF study would start from.

## Scope

In: steady level-flight longitudinal trim of a numeric (already sized) aircraft; corridor bounds by optimization;
example on the Halo reference; tests. Out (deferred): linearized models (CasADi Jacobians of the same trim
equations), lateral-directional trim, climbing or accelerating trims, blown-wing stall limits, rotor H-force and
flapping dynamics, CG travel with nacelle angle, the corridor as a sizing constraint.

## Model

Unknowns per trim: thrust per rotor T, attitude alpha (= pitch, zero flight-path angle), tail deflection delta,
cyclic fraction f (tip-path-plane tilt theta = f theta_max sin(tau), washed out toward airplane mode).
Equations: F_x = 0, F_z = 0 (wind axes), M_y = 0 about the CG. Forces from `TiltrotorPointMass` with the thrust
along the tip-path plane; tail-deflection lift and aerodynamic moment from `LongitudinalStability`; rotor moment
from the thrust at the hub (`moment_rotor_Nm`). The free freedom is spent on least rotor power.

Corridor bound: airspeed is an Opti variable, minimized (low side) or maximized (high side) with the trim
equations and limits as constraints. The binding limit is the one with zero normalized margin.

## Limits (user, 2026-10-05: "we can take 25 deg V-tail rotation"; defaults otherwise assumed)

| Limit | Default | Side it usually sets |
|---|---|---|
| Pitch attitude | -5 to +12 deg | low side (wing-borne, and forward thrust at mid nacelle angles) |
| Ruddervator deflection | +/- 25 deg | trim authority |
| Longitudinal cyclic | +/- 10 deg x sin(tau) | trim authority in helicopter mode |
| Wing stall (free stream, above 10 m/s) | model alpha_stall | low side |
| Edgewise advance ratio V sin(alpha + tau) / (Omega R) | 0.28 | high side at high nacelle angles (flapping, hub and pylon loads proxy) |
| Rotor shaft power | rotor drive rating | high side |
| Blade loading C_T / sigma | rotor limit | low speed, high attitude |
| Placard speed | 1.1 x design cruise (example) | airplane-mode high side |
| Momentum validity V cos(alpha + tau + theta) >= 0 | - | model validity |

Stall in conversion: the blown inboard wing's local angle (free stream plus slipstream along the thrust) is
reported, not limited; near hover it is the download, which the force model already carries.

## Interfaces

`aircraft_closure.trajectory.corridor`: `CorridorLimits`, `TrimGeometry`, `moment_rotor_Nm`, `TrimPoint`,
`build_trim(opti, ...)` (equations into a caller's Opti), `solve_trim`, `CorridorBound`, `solve_corridor_bound`,
`solve_corridor`. Example: `examples/halo_conversion_corridor.py` (writes `output/corridor/`).

## Tests (`tests/trajectory/test_corridor.py`)

Rotor-moment signs, zero moment through the spindle, cyclic moment identity, symbolic solve; hover trim
(level, no cyclic, n T (1 - f_dl) = W); hover CG offset needs cyclic atan(dx / h) and equal nose-up pitch;
helicopter-mode high side at mu_max; airplane-mode low side at the attitude limit with cyclic washed out and high
side at the placard; trends with nacelle angle and with the edgewise limit; tail sign.

## Progress

- [x] Module, example, tests (10 pass).
- [ ] Linearized longitudinal models at the trim schedule (next, if wanted).
