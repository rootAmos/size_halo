# Plan 060: continuous battery voltage (RC states by collocation, exact end-of-interval voltage)

## Goal

Solve the battery terminal voltage continuously in time, so the minimum-voltage limit holds at every instant
of the mission and the trajectory, not only at one evaluation per point (user, 2026-10-11: "we should be
solving for voltage continuously"; "AeroSandbox should already have this capability for the trajectory
optimizer"). It does: `Opti.constrain_derivative` (trapezoidal collocation) already carries SOC, mass and the
plan 036 temperatures in the trajectory model.

## Scope

In: an option `battery_dynamics` ("quasi_steady", the default and every existing reference; "continuous")
for the trajectory builder, the sizing mission and `HaloAssumptions`; the battery's instantaneous terminal
voltage and RC-branch derivative; unit tests. Out: a full sizing or trajectory re-solve with the option on (a
memory-limited run, queued with the HV bus trade); OCV hysteresis; a thermal-to-resistance feedback (plan 028).

## References

- Paudel et al., Batteries 2025, 11, 313: the 2-RC INR21700-50G model (plan 021).
- Plan 021/022: `EquivalentCircuitBattery`, the exact constant-current RC update and the trajectory's steady
  polarization.
- AeroSandbox `Opti.constrain_derivative` (trapezoidal collocation, as in plan 019 and 036).

## Model

Per cell, scaled to the pack: V(t) = OCV(s) - I R0(s) - sum_k v_k, ds/dt = -I/Q, dv_k/dt = (I R_k(s) - v_k)/tau_k.

What each model does today, and the gap:

| | Sizing mission (quasi-steady points) | Trajectory (collocated nodes) |
|---|---|---|
| SOC | coulomb counting point to point | collocated state |
| RC voltages | exact exponential update for the point's constant current | steady value I R_k at every node (no states) |
| Voltage limit | end of each point, but with OCV and R0 at the **mid-interval** SOC | at every node |

**Sizing mission.** Within a point the current is constant, so the RC update is already the exact solution of
the ODE. With constant current the terminal voltage has its minimum at an end of the interval: when discharge
current is unchanged or rising, OCV falls and v_k rises, so V falls monotonically; when it steps down, the
recovery term decays while the OCV slope does not, so V is concave in time and its minimum is at an end point.
The start of a point is the end of the previous one with a lower or equal voltage drop on any step down, so
the end voltage is the continuous-time minimum; what is wrong today is that it uses OCV and R0 at the
mid-interval SOC, which overstates the end voltage while discharging (the engine-out hover's binding
2.5 V/cell limit is the case that matters). "continuous" evaluates the end voltage at the end SOC:
V_end = OCV(s1) - I R0(s1) - sum v_k(dt). Convergence in time then comes from `subsegments` (OCV, R and the
power-demand current vary between points), checked below.

**Trajectory.** "continuous" adds the RC voltages as states at every node, dv_k/dt by
`opti.constrain_derivative`, starting from rest (or a given `voltage_rc_start_V`), and the battery is evaluated
instantaneously at each node: V = OCV(s) - I R0(s) - sum v_k. The voltage limits stay at every node. The steady
polarization ("quasi_steady") overstates the sag on short pulses and misses the recovery after a load drops;
the states fix both. Trapezoidal collocation is A-stable; with tau_1 = 8 s, node spacing should stay below
about tau_1 / 2 for accuracy (error about (dt/tau)^2 / 12).

The battery owns the physics (`voltage_terminal_V`, `derivative_voltage_rc_V_s`); the builder and the mission
own the Opti states and constraints.

## Verification

1. Battery: the instantaneous voltage equals `evaluate(..., duration_s=0, voltage_rc_start_V=v)`; RC derivative
   zero at steady state, sign of recovery; the exact update satisfies the ODE (finite difference).
2. Endpoint minimum: constant current from rest and a step-down, on a fine time grid, has its minimum voltage at
   an interval end.
3. Sizing end voltage: constant-power discharge to low SOC; the "continuous" end voltage converges as the
   subsegments increase and lies below the mid-SOC one.
4. Trajectory RC states: the collocated states reproduce the closed-form constant-current rise and the
   zero-current relaxation, with second-order convergence in node spacing.
5. Option off: default `battery_dynamics` is "quasi_steady" everywhere; the existing battery, mission and
   trajectory tests unchanged.

### Part 2 (proposed, user 2026-10-11: "current should also be continuous")

Part 1 keeps the sizing mission's current constant within each point, so the end voltage converges at first
order in the subsegments. Continuous current needs the mission as collocation nodes: a flight point at each of
the n + 1 nodes of a segment (instantaneous battery: duration 0, its own power balance and current), SOC,
mass and RC voltages as node variables, linked by the trapezoidal rule (`constrain_derivative`-style
equalities), and the voltage limit at every node. Current is then piecewise linear within a segment (second
order), stepping only where the flight condition steps (segment boundaries). Consequences: one more flight
point per segment (about 13 more in the reference), the Tier 19 temperatures must also become node states,
and `SegmentResult`/`battery_trace` consumers in `halo_sizing` read nodes. Verification needs full sizings.

## Assumptions

- R_k and tau_k from the 25 C fit; the R_k SOC dependence is evaluated at the node SOC (trajectory) or the
  mid-interval SOC (sizing RC update).
- Trajectory RC states start from rest unless given.

## Progress log

- 2026-10-11: plan written.
- 2026-10-11: part 1 in. `EquivalentCircuitBattery.voltage_terminal_V` and `derivative_voltage_rc_V_s`;
  `build_mission(battery_dynamics=)`, `HaloAssumptions.battery_dynamics`, trajectory `battery_rc_states` and
  `build_tiltrotor_trajectory(battery_dynamics=, voltage_rc_start_V=)`; defaults "quasi_steady" (references
  unchanged). Engine-out-like check (500 kW, 60 s from SOC 0.3, developed polarization, the plan 021 pack):
  end voltage by points per segment

  | points | mid-SOC (reference) | end-SOC (continuous) |
  |---|---|---|
  | 1 | 582.4 V | 548.0 V |
  | 3 (the reference engine-out) | 549.4 V | 535.0 V |
  | 12 | 534.9 V | 530.9 V |
  | 96 | 530.2 V | 529.7 V |

  The reference's binding 525 V engine-out check is about 20 V optimistic at 3 points; the continuous end
  voltage converges at first order (constant current within a point). Trajectory RC states match the closed
  form within 1 % at 2 s spacing, second order in the spacing. Not yet run: a full sizing or trajectory with
  the option (free memory 0.5 GB).
