# Plan 065: nacelle rate, nacelle acceleration and pitch-rate limits on trajectories

## Goal and scope

The trajectory optimizer (plan 019) bounds the nacelle tilt rate at 8 deg/s, a value labelled "XV-15-like" without a
source, and nothing else about how fast the aircraft can change attitude: the tilt rate may jump from zero to full
between two nodes (bang-bang), and the fuselage pitch attitude (alpha + gamma, algebraic in the point-mass model) may
change by any amount between nodes. The author asked for realistic limits so the optimized take-off cannot rotate the
nacelles or the fuselage instantly.

In: `TrajectoryLimits` gets a sourced nacelle rate (7.5 deg/s), a nacelle acceleration bound and a fuselage
pitch-rate bound, each disabled by None; the trajectory and its numeric result expose the realized values. These
are trajectory (plan 019) settings, not sizing: no sizing result changes. The defaults change on purpose (the
author asked for them), so the plan 019 / 036 / 049 / 060 trajectory results move slightly.
Out: actuator dynamics or loads, rate limits that vary with airspeed or on the ground, a pitch-acceleration bound.

## References

- NASA NTRS 19780002103 (XV-15): the pilot's conversion switch holds a continuous conversion at 7.5 deg/s.
- Bell test-pilot report on the XV-15 and 609 (Clay Lacy, 1998): a 12 s maximum-rate conversion from 90 deg to level
  (7.5 deg/s).
- US 9,377,784 B2 (adaptable automatic nacelle conversion, V-22 era): nacelle rate up to 8.0 deg/s (emergency
  reconversion), 2 deg/s on the ground.
- Plan 019 (trajectory optimization), `src/aircraft_closure/trajectory/tiltrotor.py`.

## Model

On the collocation grid (node times t_i, intervals dt_i = t_{i+1} - t_i):

- Tilt rate per interval r_i = (tilt_{i+1} - tilt_i) / dt_i, |r_i| <= `tilt_rate_max_deg_s` (as before; exact on the
  piecewise-linear tilt history).
- Tilt acceleration between adjacent intervals a_i = (r_{i+1} - r_i) / ((dt_i + dt_{i+1}) / 2),
  |a_i| <= `tilt_acceleration_max_deg_s2`.
- Pitch rate per interval q_i = (theta_{i+1} - theta_i) / dt_i with theta = alpha + gamma,
  |q_i| <= `pitch_rate_max_deg_s`.

Defaults: 7.5 deg/s (XV-15 source), 7.5 deg/s^2 (full rate in about one second: assumed, no published actuator
acceleration found), 5 deg/s pitch rate (assumed: a moderate attitude change for a 6 t aircraft). None disables a
bound.

## Verification

- Unit tests on a small Opti problem: each bound holds on the solution and is active when the problem pushes against
  it; None removes it.
- The existing trajectory problems (min-energy and prescribed transitions, min-time climb) still solve and meet the
  new bounds.
- The optimized take-off (min-time climb in the computed corridor, v3.10) is re-flown: time to climb and fuel against
  the 8 deg/s, unbounded-acceleration result.

## Assumptions

- The acceleration and pitch-rate bounds are assumptions (see Model); the rate is the XV-15's.
- The bounds apply to interval averages on the grid; between nodes the histories are piecewise linear.

## Progress log

- 2026-10-12: plan, limits and tests. `tests/trajectory` (39 tests, the min-energy and prescribed transitions and the
  min-time climb solved with the new defaults) pass; the transition test's conversion-time floor is now
  90 / rate + rate / acceleration (13 s).
- 2026-10-12: optimized take-off re-flown (min-time climb in the computed corridor, v3.10 min take-off weight, 61 nodes):

  | | before (8 deg/s only) | plan 065 limits |
  |---|---|---|
  | time to 10,000 ft and 165 kt | 3.79 min | 3.80 min |
  | fuel | 65.1 lb | 65.1 lb |
  | ground covered | 7.35 nm | 7.35 nm |
  | max nacelle rate | 8.00 deg/s | 7.50 deg/s (active) |
  | max nacelle acceleration | 1.48 deg/s^2 | 1.62 deg/s^2 (bound 7.5) |
  | max pitch rate | 2.54 deg/s | 2.54 deg/s (bound 5) |

  Only the rate bound is active. On this grid (about 3.8 s per interval) the solution was already smooth: the
  acceleration and pitch-rate bounds are averages over an interval, so they cannot see faster motion between nodes;
  a finer grid or a node-level actuator state would be needed to resolve sub-interval transients.
