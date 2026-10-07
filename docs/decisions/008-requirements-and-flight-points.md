# Requirements: flight points and powertrain sizing to capability

Status: COMPLETED 2026-10-02 — part of the author's "go all the way" request; the author asked
(2026-10-01, overnight) to continue to a simultaneous mission and sizing loop.
Review delegated; decisions self-reviewed against the project rules.

## Goal and scope

Tier 7. Capability requirements (payload, hover thrust-to-weight, climb rate,
level speed, ceiling) expressed as quasi-steady flight points whose Tier 3
operating margins must be non-negative. A reusable flight-point builder couples
aerodynamics, actuator-disk rotors, gearbox, McDonald machines, battery,
generator and turboshaft for the series-hybrid reference topology. A sizing
example scales the powertrain (rubber machines, turboshaft, battery power,
rotor disk area) inside the mass closure to meet the requirement set at
minimum MTOM. Out of scope: time integration and energy (Tier 8), turboshaft
altitude lapse (Tier 10 deck), transition, failed-rotor hover beyond the
symmetric `active_rotor_count` approximation.

## Physics

- Hover: thrust per active rotor = (T/W) W / n_active, V = 0.
- Airplane mode (tiltrotor cruise): wing lift = W cos gamma with alpha a
  variable, thrust = D + W sin gamma, sin gamma = climb rate / V.
- Rotor shaft power from the actuator disk in thrust mode; rotor speed is a
  variable (motor speed = gear ratio x rotor speed), so machine efficiency
  enters through the McDonald map; gearbox input torque = P / (eta omega).
- Battery current is a variable with terminal power = h_e x motor electrical
  demand; generator torque is a variable with electrical output = (1 - h_e) x
  demand at a fixed generator speed; h_e in [0, 1] is a variable unless given.
- Feasibility = every Tier 3 operating margin >= 0 at the point.

## Interfaces

```
powertrain/topologies.py
    SeriesHybridSizing(torque_peak_motor_Nm, torque_peak_generator_Nm,
        power_rated_turboshaft_W, power_max_discharge_battery_W,
        energy_capacity_battery_J, area_disk_m2, count_rotors, ...)
    build_series_hybrid_from_sizing(sizing) -> Topology   (rubber machines)
performance/flight_point.py
    FlightCondition(mode, velocity_m_s, altitude_m, climb_rate_m_s,
                    thrust_to_weight, active_rotor_count, soc, label)
    FlightPoint(...expressions...)
    build_flight_point(opti, aircraft, aerodynamics, condition, mass_kg,
                       hybridization_electric=None)
requirements/capability.py
    HoverRequirement, ClimbRequirement, SpeedRequirement, CeilingRequirement
        .flight_condition()
    RequirementSet(payload_kg, hover, climb, speed, ceiling).flight_conditions()
examples/requirements_sizing.py
```

The flight-point builder lives above the vehicle and discipline layers
(orchestration); it creates Opti variables because it is caller-side
orchestration, documented as such, while components remain equation-only.

## Variables and constraints (example)

Variables: MTOM, wing LE, motor/generator peak torques, turboshaft rating,
battery discharge power, disk area, plus per-point alpha, rotor speed,
battery current, generator torque, h_e. Equalities: mass closure, hover pitch
trim, per-point lift and power balances. Inequalities: all point margins >= 0.
Objective: minimum MTOM. Battery energy fixed here (Tier 8 sizes it).

## Tests

Flight-point identities (hover thrust sum = T/W W; airplane thrust = D + W sin
gamma; power chain consistency rotor -> gearbox -> motor; bus balance; fuel =
engine model); requirement conditions; sizing meets every margin and names
binding requirements; tighter requirements grow ratings and MTOM.

## Reference cases

Requirement set: 300 kg payload; hover T/W 1.1 at 1000 m; climb 5 m/s at
50 m/s, 1000 m; level 80 m/s at 1000 m; ceiling 4000 m with 0.5 m/s at 55 m/s.

## Acceptance

Tests and notebooks pass; sizing solves with binding requirements reported;
Tier 7 notebook; docs; commit and push.

## Progress and decisions

- 2026-10-01: Plan written. Rotor mass scales with disk area (3 kg/m2, equal to
  the 30 kg reference at 10 m2); gearbox rating follows motor rating; battery
  charge rating is half the discharge rating; generator speed fixed at its peak
  efficiency speed.
- Without energy, a free battery split at every point would let the optimizer
  delete the turboshaft; sustained requirements therefore default to h_e = 0
  (turbogenerator only), hover may use the battery.
- Results: MTOM 1113 kg; max speed sizes the 200 kW turbogenerator, hover the
  73 kW motors; disk area 3.8 m2 (no energy cost yet). 155 tests; Tier 7
  notebook 51/51.

## Deferred

Engine altitude lapse, OEI hover with asymmetric control, transition corridor,
energy-limited requirements (Tier 8/9).
