# Plan 049: the trajectory flies the ε-NTU cooler

## Goal and scope

The trajectory model (plans 019, 036) must fly an aircraft sized with the plan 042 ε-NTU ram-air cooler
(`HaloAssumptions(heat_exchanger_model="ntu")`, `NtuRamAirHeatExchanger`). Today `_thermal_states` in
`trajectory/tiltrotor.py` raises `NotImplementedError` for it. The cooler becomes the default with baseline v3.7, so
`examples/halo_trajectory_computed_corridor.py`, `halo_mission_profile.py` and `halo_conversion_corridor.py`
(default `solve_halo_sizing()`) would break.

In: the ε-NTU cooler's operating point at every trajectory node, as plan 042 put it in the flight points; a test that
a trajectory with the NTU-sized aircraft builds and solves; `examples/halo_trajectory_computed_corridor.run` accepts
the requirements and assumptions so a non-default sizing can be flown. Out: an exit flap (per-node nozzle area),
pump power, coolant-loop states (the coolant stays at the loop temperature, as in sizing). The mass-per-watt path is
unchanged.

## References

- Plan 042 (`docs/decisions/042-ntu-heat-exchanger.md`): core, duct, margins, positivity by log transform.
- Plan 036 (`docs/decisions/036-trim-and-trajectory-states.md`): temperature states and the mass-per-watt cooling
  blend in the trajectory.
- `performance/flight_point.py` and `thermal/heat.py`: how the sizing's flight points create the cooler's unknowns.

## Model

The sized cooler is a number: core counts, nozzle area and fan rating come from the sizing (`build_halo_aircraft` with
the sized design). Per trajectory node (`_thermal_states`):

- **Air mass flow** `mass_flow_air_kg_s[i]`: an Opti variable, log-transformed (positive at every iterate; the
  core's fractional Reynolds powers return NaN otherwise), with the duct residual (nozzle flow minus assumed flow)
  divided by the flow constrained to zero, as in `build_flight_point`.
- **Fan pressure rise** `pressure_rise_fan_Pa[i] >= 0`: an Opti variable at every node. Its electrical power
  ṁ Δp_fan / (ρ_face η_fan) joins the bus demand (so the bus energy state and the bus balance), and stays below the
  fan rating.
- **Heat rejection**: the core must reject the node's heat to the coolant Q (the sum of (T - T_c)/R over the
  rejected sources), with the coolant entering at T_c + Q/C_coolant: `NtuRamAirHeatExchanger.margins` at every node.
  The duct margin (core-exit total pressure at or above ambient) is applied at every node too; the nozzle-to-face
  margin is a constant of the sized aircraft and is not repeated.
- **Duct force**: the net force F = ṁ (V_exit - V) from the duct's momentum balance, weighted by w (below) and
  applied along the fuselage axis (the duct exhausts aft along it), i.e. drag D = -w F gives -D cos α along the
  path and D sin α in the wind z-axis (down). Heat addition raises V_exit (the Meredith effect) as in sizing.

**Blend with speed.** The mass-per-watt model has no flow solution, so plan 036 splits its pumping power between
cooling drag and fan by the weight q / (q + Δp). Here the fan is a per-node unknown and the duct's flow follows from
its residual: in hover the ram rise is nil, so the residual and the exit-pressure margin need Δp_fan > 0 and the fan
is paid on the bus; as q grows the ram rise drives the flow and the optimizer drops the fan (it costs bus power).
The duct force is counted in the ram share of the duct's driving pressure, w = q / (q + Δp_fan + 1 Pa) (the
plan 036 `ram` factor with the fan rise in place of the core loss): w = 1 wing-borne with the fan off, the full
momentum-balance drag of the sizing's airplane points; w -> 0 in hover, where the sizing's hover points ignore the
duct force. Without the weight the first solve ran the fan at its 8.6 kW rating as a low-speed thruster (about
-166 N of "drag" at 2 m/s, see the log). Everything is smooth; there is no branch on a symbolic value. Unlike
sizing's airplane points, the fan is available at every node; at speed it is near-useless (8.6 kW at η 0.55).

## Verification

- `tests/trajectory/test_trajectory_ntu_cooling.py`: the NTU reference aircraft (plan 042's converged design,
  14,297 lb, numbers in the test) flies the minimum-energy transition: it solves; the duct residual is zero and the
  heat-rejection, fan-rating and exit-pressure margins are met at every node (re-evaluated numerically from the
  temperature states); below 6 m/s the fan supplies more pressure than the ram and wing-borne it is off; the
  wing-borne cooling drag is the full duct drag along the fuselage axis (tens of N) and the hover node's is nil;
  the bus energy includes the fan energy.
- Existing trajectory tests (`tests/trajectory/`) pass unchanged (mass-per-watt path untouched).
- `examples/halo_trajectory_computed_corridor.py` run once with the NTU sizing (`output/ntu_ntu.pkl` from the
  plan 042 work): minimum-energy conversion time and energy against plan 039 (21.9 s, 9.6 kWh).

## Assumptions

- ISA atmosphere along the trajectory (as before); air at the core face at the freestream total state.
- Coolant at the loop temperature at the exchanger outlet, fixed coolant flow (plan 042); no coolant-loop
  dynamics; pump power not charged.
- The duct's flow settles instantly at each node (quasi-steady duct), as the fan and duct time constants are seconds
  or less against a 20-60 s manoeuvre.
- Initial guesses: 3 kg/s of air and 300 Pa of fan rise at every node.

## Progress log

- 2026-10-11: plan written.
- 2026-10-11: implemented (`trajectory/tiltrotor.py` `_ntu_cooling`; `TrajectoryThermal` gains the wind-z duct
  force and the per-node air flow and fan rise; `TrajectoryResult` reports them;
  `halo_trajectory_computed_corridor.run` takes requirements and assumptions). First solve with the full duct force
  at every node: the minimum-energy transition ran the fan at its rating at 2-5 m/s (air flow 4.2 kg/s on a cold
  start with almost no heat, a 166 N forward duct force); with the duct force zeroed the fan stayed off and the bus
  energy was the same, 9.2 kWh, so the fan was being used as a thruster. The ram-share weight above removes it: the
  fan now gives 12-74 Pa (under 0.4 kW) through the conversion, where the ram rise is a few to a few hundred Pa,
  and is off at the wing-borne end, where the duct drag is 60 N (plan 042 cruise: 53 N).
- Tests: `tests/trajectory/test_trajectory_ntu_cooling.py` 6 tests OK (the plan 042 NTU design, 21 nodes, 7-22 s);
  `tests/trajectory/` 39 tests OK (corridor 16, tiltrotor and thermal 17, NTU 6).
- `examples/halo_trajectory_computed_corridor.py` with the plan 042 NTU sizing (14,297 lb, `ntu_ntu.pkl`), 648 s
  including the corridor: minimum-energy transition **21.7 s, 8.94 kWh** (plan 039 on the 15,179 lb mass-per-watt
  reference: 21.9 s, 9.6 kWh), fan peak 0.44 kW, fan energy 0.001 kWh, duct drag 60 N at the end; minimum time to
  climb 227.4 s, 94.7 kWh, fuel 29.5 kg, with the fan at its 8.6 kW rating for part of it (0.54 kWh of fan energy;
  the generator reaches 103 C, the motor 90 C). The energy change is mostly the lighter aircraft (-882 lb).
- 2026-10-11 (plan 048 branch): is the climb's fan use cooling or thrust? `examples/halo_climb_fan_check.py` flies
  the minimum-time climb of v3.7 (14,357 lb) as sized and with the fan rating cut to 100 W: 227.4 s / 95.1 kWh /
  29.5 kg against 228.1 s / 94.9 kWh / 29.6 kg, every machine temperature limit met in both. Cooling does not need
  the fan in the climb; the optimizer uses it for a sliver of thrust the ram-share weight leaves (0.7 s, 0.3 %).
  Kept as a known small effect; a stricter fix would credit the duct force only from the ram share of the flow.
