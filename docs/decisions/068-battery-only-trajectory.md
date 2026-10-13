# Plan 068: battery-only tiltrotor trajectory and a transition-climb segment

## Goal and scope

The tiltrotor trajectory model (plan 019, `trajectory/tiltrotor.py`) assumes the series-hybrid topology: it reads
a `generator`, a `turboshaft` and a `gearbox` instance and always creates a generator-torque control. It cannot fly
an all-electric aircraft such as the plan 066 topology (`build_all_electric`). This plan makes the generator path
optional and adds a reusable mission segment, so a caller can ask how much battery energy a take-off, conversion
and climb to cruise costs on a fixed aircraft and compare it with a fixed-power allowance.

In:

- `TiltrotorPointMass` and `build_tiltrotor_trajectory` on a topology without a `generator` instance (battery
  only): the battery alone supplies the bus; no generator-torque variable; no turboshaft or generator constraints;
  fuel flow identically zero, so the mass state is constant. A topology without a `gearbox` instance is direct
  drive (ratio 1, no loss). Selected by the topology structure (Python branching on the instance set, never on a
  symbolic value). The series hybrid is unchanged bit for bit.
- `trajectory/segments.py` (orchestration; each function owns its `asb.Opti`):
  `solve_transition_climb`, `solve_steady_level_power`, `energy_fixed_power_rule_J` and the
  `TransitionClimbResult` dataclass.
- The notional all-electric aircraft (plan 066) gets an optional `MomentumProfileRotor` (`rotor_model=
  "momentum_profile"`, solidity 0.10) and, in `examples/all_electric_transition_climb.py`, an airframe and
  aerodynamics model, so it can be flown.
- Tests: `tests/trajectory/test_battery_only_trajectory.py`.

Out: feeder losses (inverter, cable, protection) in the trajectory power chain (not modelled for the series hybrid
either); a channel failure during the segment; the computed corridor (plan 039) for the notional aircraft (it
needs a stability model and trim geometry); a vertical-climb or obstacle-clearance phase (a caller adds it through
the `configure` hook); sizing an all-electric aircraft.

## References

- Plan 019 (trajectory optimization), plan 022 (battery in the trajectory), plan 065 (rate limits), plan 066 (all-
  electric topology and the notional aircraft) in this repository.
- Chauhan, Martins, "Tilt-wing eVTOL takeoff trajectory optimization", Journal of Aircraft 57(1), 2020: minimum-
  energy take-off and transition of an electric VTOL by trajectory optimization (the same problem class).
- Betts, Practical Methods for Optimal Control and Estimation Using Nonlinear Programming, 2nd ed., SIAM 2010:
  trapezoidal direct collocation with free final time.
- Fredericks, Sripad, Bower, Viswanathan, ACS Energy Letters 3, 2018: eVTOL pack power and energy (the notional
  pack).
- Maisel et al., NASA SP-2000-4517 (XV-15): aspect ratio 6.12 and disk loading of the notional airframe; NASA
  TM X-62407 sec. 5.1: hover download fraction 0.07.

## Model

Battery-only supply (`TiltrotorPointMass.evaluate_supply`, no `generator` instance):

    P_bus = P_battery,terminal = V I,    generator = engine = None,    P_shaft,gen = P_avail,turb = mdot_fuel = 0

`build_tiltrotor_trajectory` then has the same states (x, h, V, gamma, mass, SOC, bus energy) and controls (alpha,
tilt, thrust and speed per rotor, battery current) as before, without the generator torque. dm/dt = 0 with
m(0) = m_0. The bus balance, rotor, motor, wing, attitude, rate, corridor and battery limits are the same
expressions as the series hybrid's.

Transition-climb segment (`solve_transition_climb(model, *, mass_kg, soc_initial, altitude_final_m,
velocity_final_m_s, altitude_initial_m=0, duration_bounds_s=(20, 900), objective="energy", corridor=None,
limits=TrajectoryLimits(), count_nodes=41, ...)`):

- start: trimmed hover (tilt 90 deg, V = 1 m/s, gamma 0, zero acceleration and flight-path rate) at
  `altitude_initial_m`; end: trimmed level flight at tilt 0, `altitude_final_m` and `velocity_final_m_s`;
  h >= h_0 throughout; free final time within `duration_bounds_s`.
- objective "energy": min E_bus(t_f) (the battery terminal energy on a battery-only aircraft); "time": min t_f; both
  with a 1e-4 control-slew regularization.
- corridor: the assumed XV-15-shaped `ConversionCorridor` on the aircraft's sea-level stall speed unless given.
- the guess is generic: hover rotor speed at C_T/sigma = 0.10, momentum hover power at figure of merit 0.75,
  a 25 % conversion ramp and a linear climb.

Result fields: `duration_s`, `distance_m`, `energy_electric_J` (bus), `energy_battery_terminal_J`,
`energy_battery_chemical_J` (= SOC drop x capacity), `power_battery_max_W`, `power_bus_max_W`,
`mass_fuel_burnt_kg`, and node histories (time, x, altitude, speed, gamma, alpha, pitch, tilt, thrust, rotor speed,
rotor shaft power, bus and battery power, current, voltage, SOC, bus energy, mass).

`solve_steady_level_power`: a three-node, 10 s segment held steady at every node at tilt 0, least energy; the bus
power of cruise from the same model. `energy_fixed_power_rule_J(P_ref, k, t) = t k P_ref`.

## Verification

- Series hybrid bit for bit: the Halo (Tier 16 aircraft) minimum-energy transition and minimum-time climb solved
  with the new module and with the module before this plan give identical node arrays (time, altitude, speed,
  tilt, bus power, SOC, mass, bus energy); `test_tiltrotor`, `test_trajectory_thermal`,
  `test_trajectory_ntu_cooling` and `test_topologies` pass.
- Battery-only model: no generator or engine, bus power = battery terminal power, zero fuel flow; direct drive
  (motor speed = rotor speed, channels share the rotor torque); symbolic `MX` outputs with full Jacobians.
- Segment on the notional aircraft (21 nodes, 500 m, 70 m/s): boundary conditions, SOC, battery power and tilt-rate
  limits; mass constant; energy identity (chemical energy = SOC drop x capacity to 1e-6, bus energy state = trapezoid
  of the battery terminal power, resistive loss between 0 and 2 %); kinematics against trapezoidal integrals; the
  minimum-time segment is no longer and the minimum-energy one no costlier than the other; fixed duration honoured
  and no cheaper than the free optimum; the segment's final node equals the steady level power.

## Assumptions

- As plan 019: axial-flow rotor model on the velocity component along the shaft, no edgewise power or
  translational lift; quasi-steady battery; the CG does not move. With the assumed corridor the optimizer climbs in
  partial conversion (tilt about 45-48 deg) because nothing in the model penalizes edgewise flow there; a caller
  who needs an airplane-mode climb bounds the tilt through `configure` or passes a computed corridor.
- Feeder losses (inverter about 2 %, cables) are outside the trajectory power chain, so the battery energy is
  slightly optimistic; the motor sees the battery terminal voltage.
- The notional airframe is illustrative (wing 7.5 m2, AR 6; miscellaneous drag area 0.10 m2 assumed).
- The fixed-power rule (180 s at 1.5 x cruise power) is an illustrative generic heuristic, not a sourced method.

## Progress log

- 2026-10-12: plan written (numbered 068: 067 was taken on main by the turbine MCP cap).
- 2026-10-12: landed. Battery-only path in `tiltrotor.py`; `trajectory/segments.py`; notional rotor option and
  `examples/all_electric_transition_climb.py`. Notional aircraft (1,600 kg, 3 channels per rotor, 90 kWh, 450 kW
  pack limit), sea level to 1,500 m at 75 m/s, 41 nodes: 95 s, 5.24 km, battery terminal energy 11.61 kWh
  (chemical 11.66 kWh, SOC 0.950 -> 0.820), peak battery power 450 kW (the pack limit, active from 7 s to the end
  of the climb), hover at the start 324 kW. Cruise battery power at 1,500 m and 75 m/s 151.3 kW, so the rule
  allows 11.34 kWh; the optimized segment needs 1.02 x the rule. Solve under 2 s (41 nodes); 11 tests in
  `tests/trajectory/test_battery_only_trajectory.py` in about 1 s. Series hybrid identical bit for bit.
