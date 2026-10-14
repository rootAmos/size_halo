# Plan 069: a mission flown as linked trajectory segments

## Goal and scope

Plan 068 flies one segment (take-off, conversion and climb) in its own `asb.Opti`. A whole mission (hover, conversion,
climb, cruise, reserve, descent, reconversion, hover) is today a set of separate solves whose start states are
typed in by hand. This plan links segments in one `asb.Opti`: each segment is a short collocated trajectory, the
state is continuous at every join, and one objective covers the mission. It is a check on a fixed aircraft (the
energy and the state of charge as flown, the margins to every limit on the way); it does not size anything.

In:

- `build_tiltrotor_trajectory(..., initial_from=None)`: when given a previous `TiltrotorTrajectory`, the segment's
  SOC and mass start from that segment's last node (symbolic links) instead of the numeric `soc_initial` and
  `mass_initial_kg`, which then serve as initial guesses only. Default unchanged.
- `trajectory/mission.py` (orchestration, owns its `asb.Opti`):
  - `SegmentEnd` (optional altitude, airspeed, flight-path angle, nacelle angle, trimmed flag): a boundary;
  - `MissionSegment` (name, guess, node count, duration bounds, horizontal distance or free, end conditions,
    optional per-segment model, limits, corridor, a `configure(opti, trajectory)` hook, whether it counts in the
    objective);
  - a mission-level `configure(opti, trajectories)` hook for constraints that span segments (a total ground
    distance over several segments);
  - `solve_linked_mission(model, segments, *, mass_kg, soc_initial, start, objective="energy")`, returning a
    `LinkedMissionResult` (per-segment `TransitionClimbResult`s, totals, objective).
- Tests on the notional all-electric aircraft (plan 068's fixture).

Out: the RC branch voltages (plan 060, continuous battery dynamics) across a join (the quasi-steady battery only);
wind; mass change of a hybrid across segments beyond the linked mass state (works, not tested); any change to
sizing.

## References

- Plan 068 (battery-only trajectory, the segment and the notional aircraft), plan 019 (trajectory), plan 065
  (rate limits).
- Betts, Practical Methods for Optimal Control and Estimation Using Nonlinear Programming, 2nd ed., SIAM 2010,
  ch. 4.7: multi-phase problems, phases linked by state continuity (linkage constraints) in one NLP.
- Patterson, Rao, "GPOPS-II", ACM TOMS 41(1), 2014: the same phase-linkage formulation.

## Model

Segment k is `build_tiltrotor_trajectory` on its own (or the mission's) model with n_k nodes and free duration
within its bounds. Links between segment k-1 (last node) and k (first node):

    h, V, gamma, tilt continuous;   SOC_k[0] = SOC_{k-1}[-1];   m_k[0] = m_{k-1}[-1]

Horizontal distance: each segment's x starts at 0; `distance_m` (when given) fixes x[-1]. Start conditions of the
first segment and end conditions of any segment come from `SegmentEnd` (a trimmed end: zero acceleration and
flight-path rate). Objective "energy": the sum of the counted segments' bus energy (battery terminal energy when
battery only), plus the plan 068 control-slew smoothing (below 0.1 %); "time": the sum of durations.

## Verification

- Continuity at every join to solver tolerance (h, V, gamma, tilt, SOC, mass).
- A steady level segment split in two linked halves reproduces the single segment's energy (1e-3).
- Linking a hover to the plan 068 transition-climb segment: the linked climb energy is within 2 % of the
  stand-alone segment started at the hover's end SOC (the only difference is the free start state).
- SOC drop identity: SOC drop x capacity = chemical energy summed over segments (quasi-steady battery).
- A total ground distance over two free-distance halves (mission-level hook) gives the single leg's energy
  (1e-3).
- `initial_from=None` leaves `build_tiltrotor_trajectory` unchanged (existing trajectory tests pass).

## Assumptions

- Quasi-steady battery at joins (no RC state carried). Each segment's own model may differ (e.g. a hot hover
  day and a standard cruise day, or a degraded powertrain), provided the battery capacity is the same.

## Progress log

- 2026-10-14: plan written.
- 2026-10-14: built; `tests/trajectory/test_linked_mission.py` (8 tests) pass in 5 s: continuity at the joins, split
  cruise and the mission-level distance within 1e-3 of the single leg, linked climb within 2 % of plan 068's
  stand-alone segment, the SOC-drop identity. A first use on a sized aircraft showed that long segments at 4-5 nodes
  need steady (point-performance) constraints from the caller (`configure`): with free dynamics the optimizer trades
  height and speed between distant nodes.
