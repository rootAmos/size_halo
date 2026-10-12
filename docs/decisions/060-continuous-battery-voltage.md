# Plan 060: continuous battery voltage (RC states by collocation, exact end-of-interval voltage)

## Goal

Solve the battery terminal voltage continuously in time, so the minimum-voltage limit holds at every instant
of the mission and the trajectory, not only at one evaluation per point (user, 2026-10-11: "we should be
solving for voltage continuously"; "AeroSandbox should already have this capability for the trajectory
optimizer"). It does: `Opti.constrain_derivative` (trapezoidal collocation) already carries SOC, mass and the
plan 036 temperatures in the trajectory model.

## Scope

In: an option `battery_dynamics` ("quasi_steady", the default and every existing reference; in the sizing
mission "constant_current" and "piecewise", in the trajectory "continuous") for the trajectory builder, the
sizing mission and `HaloAssumptions`; the battery's instantaneous terminal
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
2.5 V/cell limit is the case that matters). "constant_current" evaluates the end voltage at the end SOC:
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
3. Sizing end voltage: constant-power discharge to low SOC; the "constant_current" end voltage converges as the
   subsegments increase and lies below the mid-SOC one.
4. Trajectory RC states: the collocated states reproduce the closed-form constant-current rise and the
   zero-current relaxation, with second-order convergence in node spacing.
5. Option off: default `battery_dynamics` is "quasi_steady" everywhere; the existing battery, mission and
   trajectory tests unchanged.

### Part 2: continuous current (user 2026-10-11: "current should also be continuous")

Part 1 keeps the sizing mission's current constant within each point, so the end voltage converges at first
order in the subsegments. Continuous current needs the mission as collocation nodes: a flight point at each of
the n + 1 nodes of a segment (instantaneous battery: duration 0, its own power balance and current), SOC,
mass and RC voltages as node variables, linked by the trapezoidal rule (`constrain_derivative`-style
equalities), and the voltage limit at every node. At each node the equivalent circuit is solved with the flight
point's power balance: P_required = V I, V = OCV(s) - I R0(s) - sum v_k, so current and voltage both vary
within the segment. Current is then piecewise linear within a segment (second
order), stepping only where the flight condition steps (segment boundaries). Consequences: one more flight
point per segment (about 13 more in the reference), the Tier 19 temperatures must also become node states,
and `SegmentResult`/`battery_trace` consumers in `halo_sizing` read nodes. Verification needs full sizings.

Implemented as `battery_dynamics="piecewise"` in `build_mission` (`_build_mission_nodes`); part 1's
end-SOC point model is "constant_current" (names fixed 2026-10-12 with the user: "piecewise" is current that
varies within each segment, solved at every node from the power required, and steps between segments). A node is a zero-duration flight point, which already returns the
instantaneous battery voltage from the given RC state and the temperature equal to its given start with the
loss and the heat to the coolant (so dT/dt = (loss - heat to coolant)/C). Nodes are reported as subsegments
with their trapezoidal weights as `duration_s`, so existing sums (fuel, energy, duration) are the trapezoidal
integrals; the segment's fuel is the mass difference. The trajectory option keeps the name "continuous"
(RC states; its current was already a node variable).

### Part 3: battery sub-steps between nodes (user 2026-10-12: "that sounds a lot better")

Part 2 integrated SOC and the RC voltages with the trapezoidal rule on the node currents, while the turbines
pay for the node powers. Over a 36 min cruise interval the node voltages differ by up to 190 V, so the mean of
the currents and the mean of P / V disagree, and the optimizer pumped the voltage between nodes for about 43 kWh
the turbines never delivered (SOC 0.30, 0.90, 0.36, 0.67, 0.30 in cruise). Part 2's sizing numbers are void.

The flight physics stay on the coarse nodes (each node is a full flight point). The battery, cheap to evaluate,
is integrated on a fine sub-grid inside each interval:
- the battery terminal power is linear between the two nodes' powers (the same linear power the fuel and energy
  sums integrate, so energy is consistent);
- each sub-step solves the current from P = V(I) I, V = OCV(s) - sum v_k - I R0(s) (the low-current root
  I = (E - sqrt(E^2 - 4 R0 P)) / (2 R0), E = OCV - sum v_k), explicit midpoint in time: the current at the
  start state, a half step, the current again at the half-step state and power, then the full step with it;
- the RC voltages advance exactly for that constant current (v -> I R_k + (v - I R_k) exp(-h / tau_k)) and the
  SOC by -I h / Q;
- the minimum and maximum voltage limits hold at the end of every sub-step;
- the next node's SOC and RC voltages equal the sub-stepped values (replacing the trapezoidal equalities);
  mass and temperatures keep the trapezoidal rule.
Sub-steps per interval: ceil(dt / step_battery_max_s), and at least ceil(8 / n) for a segment of n intervals,
so every segment has at least 9 battery nodes (user 2026-10-12: "we need at least 9 nodes per segment for the
battery"); default step 30 s (73 per cruise interval, 8 in a 60 s hover). Both go on `build_mission`
(`step_battery_max_s`, `count_nodes_battery_min`) and `HaloAssumptions`.

Verification: the sub-stepped SOC and RC voltages against a fine closed-form reference at constant and
linearly varying power (second order in the step), a power-balance identity at each sub-step, symbolic
compatibility, and on the Halo: a forward simulation at 1 s through the solved node powers, with the energy
mismatch between the SOC change and the delivered energy under 1 % in every interval, then the sizing at the
default and half step.

## Assumptions

- R_k and tau_k from the 25 C fit; the R_k SOC dependence is evaluated at the node SOC (trajectory) or the
  mid-interval SOC (sizing RC update).
- Trajectory RC states start from rest unless given.
- Collocated mission: "steady" polarization ties the first node's RC voltages to that node's current;
  node-state initial guesses are the previous node's initial values.

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
- 2026-10-11: part 2 in (`battery_dynamics="piecewise"` in the mission, first named "continuous"). On the four-rotor ECM fixture, 60 s
  hover from SOC 0.35 with developed polarization, end-voltage margin by subsegments: nodes 0.21513 (1),
  0.21509 (2), 0.21508 (4 to 32); points ("constant_current") 0.20520 (1), 0.21210 (4), 0.21469 (32), converging
  to the same value. Node current rises 286 -> 294 A through the hover (constant power, falling OCV).
  Trapezoidal SOC, mass, RC and motor/generator temperature links checked against the solved nodes. Not yet
  run: the Halo sizing with "piecewise" (free memory 0.4 GB; several hundred MB needed).
- 2026-10-12: Halo sizings (v3.7 reference, warm-started from it; closure < 1e-11, min margin -1e-8):

  | battery_dynamics, subsegments | take-off | empty | fuel burnt | battery | parallel |
  |---|---|---|---|---|---|
  | quasi_steady (1,1,4,1,1,1)/3, the reference | 14,356.9 lb | 10,364.6 lb | 1,825.6 lb | 61.49 kWh | 20.16 |
  | quasi_steady, doubled | 14,345.7 lb | 10,374.4 lb | 1,806.4 lb | 62.54 kWh | 20.51 |
  | constant_current, default | 14,428 lb | 10,431 lb | | 64.3 kWh | 21.08 |
  | piecewise, default | 14,343.0 lb | 10,392.8 lb | 1,787.3 lb | 64.08 kWh | 21.01 |
  | piecewise, doubled | 14,356.9 lb | 10,397.3 lb | 1,795.9 lb | 63.97 kWh | 20.98 |

  The pack converges at about 64.0 kWh with "piecewise" (0.2 % between default and doubled nodes); the
  reference's mid-SOC end voltage undersizes it by 2.5 kWh (4 %; 62.5 kWh at doubled points, still converging
  from below), about 30 lb of empty mass. The engine-out hover still binds at 525 V, now at the last node.
  Take-off barely moves (-14 / +0 lb): the reference evaluates each point at its start mass, so it also
  overstates fuel (1,826 vs 1,796 lb burnt), offsetting the battery. Within 0.1 % in take-off between node
  counts. The doubled constant_current run was stopped by the machine (low memory) and not repeated.
  "piecewise" binds the battery's maximum voltage at the first cruise node (in-flight recharge at the
  instantaneous voltage); the reference holds that limit on the interval-mean voltage.
- 2026-10-12: on v3.10 (plan 055 eigenpair flutter in the loop) "piecewise" first failed: the flutter warm start
  matched rotor speeds by point label, the nodes ("cruise 0/4") had none and started at the generic 100 rad/s
  (supersonic tip), giving NaN matrices. `warm_start_rotor_speeds` now falls back to the segment's first earlier
  point. Sized: 14,345.0 lb (reference 14,350.4), empty 10,396.6 lb, battery 64.10 kWh (61.44), min margin -1e-8;
  engine-out ends on 525 V at its last node.
  Open: with 4 cruise intervals the node currents zigzag (-515, +352, -206, +122, -22 A; SOC 0.30, 0.90, 0.36,
  0.67, 0.30), and the take-off hover runs at 0 A with the climb carrying the battery. The trapezoidal rule with
  current free at every node leaves this alternating pattern almost unpenalized, so the energy schedule within the
  mission is not physical although the sizing totals converge. Candidate fixes: a current-rate penalty or bound, or
  midpoint (Hermite-Simpson) collocation. Not yet done.
- 2026-10-12: the zigzag is an exploit, not a schedule: per cruise interval the SOC change (from the current)
  exceeds the delivered energy (from V I) by +18, +14, +8, +4 kWh. Part 2's numbers (64.1 kWh, 14,345 lb and
  the node-count convergence) are withdrawn. Part 3 (battery sub-steps between nodes) replaces the trapezoidal
  SOC and RC equalities.
- 2026-10-12: part 3 in (`EquivalentCircuitBattery.integrate_power`, used between the nodes of "piecewise").
  v3.10 sized at the default 30 s step: 14,369.5 lb, empty 10,405.0 lb, burnt 1,800.3 lb, battery 63.92 kWh,
  min margin -1e-8 (349 s solve). SOC at the nodes 0.95, 0.92 (take-off), 0.70 (climb), 0.70, 0.73, 0.70,
  0.58, 0.39 (cruise), 0.73 (loiter recharge), 0.33, 0.30; the battery gives about 130 kW in the take-off hover.
  Engine-out binds at 525 V on its last node; the maximum voltage no longer binds. 1 s replay through the solved
  node powers: SOC within 1.7e-4 of every node, at most 0.51 % of an interval's energy, losses positive in every
  interval (no free energy). Pack voltage 728-852 V through the mission; engine-out lowest 524.4 V in the replay.
  Open: the machine temperatures, still trapezoidal on 36 min cruise intervals, zigzag (generator 135, 90, 122,
  95 C); the same sub-stepping would fix them.
- 2026-10-12: half step (15 s, 691 s solve): 14,369.5 lb, 63.92 kWh, the same node SOCs to 0.001 and the same
  engine-out voltages: the sizing is independent of the battery step at 30 s.
