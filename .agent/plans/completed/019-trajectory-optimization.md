# Trajectory optimization on a sized aircraft (Tier 14)

Status: COMPLETED 2026-10-03. Tier 14, user direction 2026-10-02/03:

> Can we toss trajectory optimization using AeroSandbox into Tier 14? You can
> start with minimal energy in transition / time to climb.
> ... for trajectory optimization once again lean into aerosandbox.

## Goal and scope

- **Separate from sizing.** A stand-alone optimal-control problem flies an
  already-sized aircraft: `solve_halo_sizing()` ->
  `build_halo_aircraft(result.design)`, with every design quantity numeric.
  - Each trajectory is its own `asb.Opti`, whose variables are states,
    controls and the final time.
  - Nothing is added to the sizing Opti.
- **Library** (`aircraft_closure.trajectory.tiltrotor`):
  - `TiltrotorPointMass`: an equation-only force and power model of the
    series-hybrid tiltrotor, vectorized over nodes.
  - `ConversionCorridor`: the corridor model.
  - `build_tiltrotor_trajectory`: the orchestration builder, the trajectory
    counterpart of `build_flight_point`, which creates node variables and the
    dynamics, energy and limit constraints.
- **Problems** (`examples/trajectory_optimization.py`), on the Halo reference
  (900 kg, 210 kt, fixed 2 x 1,120 hp):
  1. **Minimum-energy transition:** from hover at 500 ft to steady wing-borne
     flight at 1.3 V_stall.
     - Altitude within +/-30 m, free final time, nacelle rate <= 8 deg/s,
       through the conversion corridor.
     - Compared with a naive prescribed conversion.
  2. **Minimum time to climb:** from hover at sea level to steady level flight
     at 10,000 ft and the sizing cruise speed, power-limited.
- **Out of scope** (later tiers): edgewise rotor physics, hot day, battery
  sag, pitch dynamics, ground tracks.

## References

- AeroSandbox 4.2.10:
  - `DynamicsPointMass2DSpeedGamma` (wind axes, `add_force`,
    `add_gravity_force`, `constrain_derivatives`);
  - `Opti.constrain_derivative` (trapezoidal);
  - `Atmosphere`.
- NASA TM X-62407, XV-15 familiarization document:
  - fig. 5.4.1, the conversion corridor (sea level, 13,000 lb curves read to
    about +/-10 kt);
  - fig. 5.4.2, conversion power;
  - sec. 8.1.4, the 95 deg conversion range;
  - sec. 5.1, the 7 % download.
- Acree, NASA/TM-2016-219070 (JVX). This is the Tier 12 rotor calibration
  (plan 016).
- Chauhan & Martins, "Tilt-Wing eVTOL Takeoff Trajectory Optimization",
  J. Aircraft 57(1), 2020. Related practice: a direct-collocation tilt
  trajectory with rotor and wing forces.

## Assumptions

- **Dynamics:** longitudinal point mass in AeroSandbox speed-gamma form.
  - "Hover" is V = 1 m/s, because the speed-gamma states are singular at
    V = 0; the speed lower bound is 0.5 m/s.
  - Mass is a state, falling by the fuel flow.
- **Geometry:** wing incidence is zero (wing alpha = fuselage alpha), and the
  nacelle tilt is measured from the fuselage axis.
  - The rotor axis makes phi = alpha + tilt with the flight path.
  - Pitch attitude is gamma + alpha, bounded to [-10, 20] deg (an assumed
    attitude limit).
- **Rotor:** the Tier 12 `MomentumProfileRotor`, with the axial velocity
  taken as V cos(phi), the velocity component along the rotor axis.
  - The edgewise component is ignored: no translational-lift benefit, no
    edgewise profile-power rise and no flapping/hub-load penalty.
    **Approximation; deferred.**
  - The airplane-mode drag increment and induced-power factor are blended
    with cos^2(tilt), with tilt clipped at 0 so that tilt <= 0 is exactly
    airplane mode.
  - The axial velocity is smoothed, sqrt(Va^2 + (1e-4 m/s)^2), so the
    profile integral stays finite at zero inflow.
  - V cos(phi) >= 0 is enforced, because descent through the disk
    (vortex-ring state) is outside momentum theory.
- **Download:** the aerodynamics model's hover fraction (0.07),
  x sin^2(tilt) x v_h^2/(v_h^2 + V^2), with v_h the hover induced velocity.
  This is exactly the hover value at V = 0, tilt = 90 deg.
- **Wing:** `SimpleAerodynamics` coefficients are evaluated at max(V, 10 m/s)
  for the Reynolds and Mach numbers, and forces at the true dynamic pressure.
  Attached flow only: alpha is bounded by the model's stall angle and
  alpha >= -10 deg.
- **Conversion corridor** (`ConversionCorridor`):
  - Low-speed boundary: V >= V_stall cos(tilt), with V_stall the Halo's
    airplane-mode stall at CLmax. The XV-15 boundary is within about 10 kt
    of this shape with V_stall = 100 kt.
  - High-speed boundary: V <= V_90 + (V_0 - V_90)(1 - (tilt/90)^2), with
    V_90 = 115 kt and V_0 = 180 kt. This is an absolute fit to the XV-15
    (similar rotor size and tip speed).
- **Power chain:**
  - The gearbox takes its loss on torque, as in the flight point.
  - Motor losses come from its McDonald model, and bus power is the motors'
    electrical power.
  - Supply comes from both turbogenerators (generator at peak-efficiency
    speed, torque a control) plus the battery (current a control).
  - The generator shaft is bounded by the lapsed turboshaft
    (`power_available_W`) and by the generator rating.
  - The battery is bounded by its discharge rating and SOC >= 0.30.
  - No in-flight recharge by default (`allow_battery_charging=False`).
    Otherwise a minimum-energy or minimum-time objective burns extra fuel to
    fly lighter, a spurious split.
- **Objectives:**
  - Transition: bus (motor electrical) energy.
  - Climb: final time.
  - Both carry a small regularization: the battery share squared (turbines
    first) and control slew, below 0.1 % of the objective.

## Interfaces

- `TiltrotorPointMass(aircraft, aerodynamics)`:
  - `.evaluate(velocity_m_s, altitude_m, alpha_deg, tilt_deg, *, thrust_per_rotor_N, speed_rotor_rad_s)`
    -> `TiltrotorForces` (wind-axis forces without gravity, lift, drag,
    rotor result, drive-chain speeds, torques and powers, bus power);
  - `.evaluate_supply(altitude_m, *, current_battery_A, torque_generator_Nm, soc)`
    -> `PowerSupply`;
  - `.velocity_stall_m_s(mass_kg, altitude_m)`.
- `ConversionCorridor(velocity_stall_m_s, velocity_max_helicopter_m_s, velocity_max_airplane_m_s)`
  with `.velocity_min_m_s(tilt_deg)` and `.velocity_max_m_s(tilt_deg)`.
- `build_tiltrotor_trajectory(opti, model, *, mass_initial_kg, soc_initial, count_nodes, duration_bounds_s, guess, limits, corridor)`
  -> `TiltrotorTrajectory`:
  - node arrays of states, controls, forces and supply;
  - `acceleration_m_s2` and `rate_gamma_rad_s` from the AeroSandbox dynamics;
  - `tilt_rate_deg_s` per interval.
- Settings: `TrajectoryLimits` (tilt rate and range, alpha, pitch, gamma,
  speed, SOC floor, charging flag) and `TrajectoryGuess`.
- **States:** x, altitude, speed, flight-path angle, mass, SOC and bus energy.
- **Controls:** alpha, tilt, thrust per rotor, rotor speed, battery current
  and generator torque; also the final time.
- **Equalities:** AeroSandbox dynamics (trapezoidal), mass/SOC/energy
  derivatives, bus balance, and the initial x, mass, SOC and energy.
- **Inequalities:**
  - the rotor's blade loading, helical tip Mach, advance ratio, tip speed
    and shaft power;
  - motor speed, torque and rated power;
  - generator rating, torque and turbine lapse;
  - battery discharge (and charge);
  - stall alpha, pitch, tilt rate and the corridor.

## Symbolic considerations

- Everything is `aerosandbox.numpy` and vectorized over node arrays. There
  are no Python loops over nodes and no branching on symbolics.
- `np.fmax` (CasADi `fmax`) is used only for the coefficient-speed floor and
  the tilt clip, both outside the active range of the solved trajectories.
- The tilt rate is `np.diff(tilt) / np.diff(time)`, an exact per-interval
  bound. A trapezoidal `derivative_of` variable was tried first, but it only
  bounds the mean of adjacent node rates, which then alternate.
- No `float()` in `src/`, and no plain `numpy`.

## Tests

`tests/trajectory/test_tiltrotor.py` (13 tests):

- **Identities:**
  - airplane mode with thrust along the flight path (tilt = -alpha)
    reproduces `build_flight_point`: forces balance to 1e-9 W and bus power
    matches to 1e-9;
  - hover at tilt 90 deg, V = 0 reproduces flight-point hover: weight
    carried with the full download, power to 1e-5 (2.2e-6 actual).
- **Collocated frozen trajectories:**
  - every node steady, thrust along the path: within 5e-4 of the flight
    point (1.1e-4 actual, from 10 s of fuel burn);
  - tilt frozen at 0: 1.4 % lower, from the thrust's lift component.
- **Signs, trends and corridor shape.**
- **Symbolic:** MX node vectors and Jacobians, plus a numeric-array path.
- **Collocation:** the constant-acceleration point mass (AeroSandbox
  dynamics on a free-final-time grid) is exact.
- **Consistency:** energy, altitude and x states equal independent
  trapezoidal integrals of the model's rates.
- **Problems:**
  - the transition meets its boundary conditions, band, rate, corridor,
    stall and power limits, and beats the prescribed profile by more than
    30 %;
  - the climb reaches 10,000 ft at cruise speed, power never exceeds
    available (bus, turbine lapse, battery), and it is faster than the
    sizing mission's 6 m/s climb.

## Acceptance

- [x] Library, examples and tests in place, and the full suite passes:
  277 tests (264 before, plus 13).
- [x] Frozen limits recover the quasi-steady flight point.
- [x] Both problems solve, in under 1 s each after sizing.
- [x] The verification notebook `notebooks/tier14_trajectory/trajectory_verification.ipynb`
  is executed: 26/26 checks, and the suite passes.
- [x] Docs updated: MODEL_INTERFACES, IMPLEMENTATION_NOTES, and the roadmap
  status.

## Progress and decisions

- 2026-10-03: implemented. Decisions:
  - **Speed-gamma, not Cartesian.** It is AeroSandbox's longitudinal class
    and keeps wind-axis forces natural; the cost is the 1 m/s "hover".
  - **Rotor model reused, not duplicated.** `dataclasses.replace` blends its
    airplane-mode coefficients per node.
  - **No in-flight recharge,** because of the spurious lighter-aircraft
    split.
  - **Tilt-rate bound per interval** (see Symbolic considerations).
  - **Prescribed reference.** It is level and constant-acceleration over
    60 s, with the nacelles held 15 s and then linear. A linear schedule from
    t = 0 leaves the corridor, and 40 s is infeasible at the -10 deg attitude
    limit.
- **Results on the Halo reference** (6,748 kg):
  - **Transition:** 22.1 s and 10.4 kWh bus energy (2.9 kg fuel, SOC 0.95 ->
    0.92), against 20.4 kWh for the prescribed profile.
    - It pitches nose-down to the attitude limit and uses about 30 m of the
      altitude band.
    - It follows the low-speed corridor boundary to about 37 deg, then
      finishes at the 8 deg/s rate.
    - The motor ratings bind (about 1.7 MW bus).
  - **Climb:** 212 s to 10,000 ft (about 14.4 m/s mean), 26.5 kg fuel, SOC
    0.95 -> 0.62.
    - It climbs at about 125-130 kt with the nacelles at about 46 deg and
      pitch at its 20 deg limit, and converts at the top.
    - The generator rating, then the turbine lapse above about 7,000 ft,
      caps the turbines; the battery fills to the motor rating.
    - **The conversion-mode climb is likely flattered** by the axial-only
      rotor model (see Deferred).

## Deferred

- **Edgewise and conversion rotor physics** (Glauert inflow, edgewise
  profile power, flapping/hub loads). Re-check the conversion-mode climb
  first when it lands. Tier 21 conversion work.
- **Electric power fraction as an energy-management objective** (fuel
  versus battery cost), in-flight recharge, and the return transition
  (airplane to hover).
- **Pitch dynamics** (`DynamicsRigidBody2DBody`), trim with tail loads, and a
  ground track (`DynamicsPointMass3DSpeedGammaTrack`).
- **Hot-day lapse** (Tier 16) and battery sag (Tier 17) in the power limits.
- **`AeroBuildup` per node** (Tier 21), post-stall wing aerodynamics, and
  flaps in conversion.
- **Corridor refinement:** digitize fig. 5.4.1 properly, scale the
  high-speed boundary with rotor loads, and check conversion power against
  fig. 5.4.2.
