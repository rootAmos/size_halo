# Plan 066: all-electric topology with per-rotor motor channels

## Goal and scope

Add an all-electric powertrain description to the framework: one battery feeding a DC bus, and on each rotor
`count_channels_per_rotor` independent channels (feeder protection, cable, inverter, motor) whose shafts combine
onto the rotor shaft. The topology reuses the Tier 15 electrical layer (`ElectricalLayer`, plan 023) and the
Tier 18 redundancy layer (`RedundancyLayer`, plan 032): a channel is a lane. It is the powertrain for an
electric tiltrotor that must hover after losing one motor or one inverter on one rotor.

In:

- `topologies.build_all_electric(motor, battery, propulsor, count_rotors=2, count_channels_per_rotor=2,
  electrical=None, redundancy=None, gearbox=None)`: a description only; the graph owns no variables or solves.
- `redundancy.degraded_state` for a lane (channel) lost on some rotors only: a condition may set
  `count_rotors_lane_out` (None keeps today's behaviour, every rotor degraded).
- A notional all-electric tiltrotor with literature-sourced data, kept for future evaluations
  (`examples/all_electric_tiltrotor.py`).
- Tests (`tests/powertrain/test_all_electric_topology.py`).

Out: an all-electric flight point or mission in `performance/flight_point.py` (it assumes a series hybrid with a
generator); sizing an all-electric aircraft; channel-count optimization (the count is an integer and is enumerated
outside a solve by the caller). Halo's default sizing and every reference result are unchanged: the new behaviour
is a new function and an optional condition attribute.

## References

- Plan 023 (electrical layer) and plan 032 (redundancy) in this repository: feeder components, lanes on a
  combining gearbox input, degraded states as flight-point inputs.
- Fredericks, Sripad, Bower, Viswanathan, "Performance Metrics Required of Next-Generation Batteries to Electrify
  Vertical Takeoff and Landing (VTOL) Aircraft", ACS Energy Letters 3, 2018: pack-level specific energy and the
  high discharge rates of hover; basis for the notional pack (200 Wh/kg, about 5C peak).
- Siemens eAircraft SP260D motor (2016 announcement; Anton, "Electric propulsion for aircraft", 2017): 260 kW
  continuous at 50 kg, 5.2 kW/kg, direct drive at propeller speed; basis for the notional motor.
- Plan 023 inverter default (20 kW/kg, between the NASA 19 kW/kg goal and SiC demonstrators).
- Maisel, Giulianetti, Dugan, "The History of the XV-15 Tilt Rotor Research Aircraft", NASA SP-2000-4517: XV-15
  disk loading (about 13 lb/ft2) as the tiltrotor reference for the notional disk loading.
- ARP4761 / CS-27 and SC-VTOL "continued safe flight and landing" after a single failure: the motivation for
  sizing each channel to carry its rotor with one channel out.

## Model

Topology (`build_all_electric`), with `n` channels per rotor and `r` rotors:

```
battery [-> n_s string contactors] [-> protection_battery -> cable_battery [-> dcdc]] -> bus (x count_buses)
bus -> n r x ([protection_motor -> cable_motor -> inverter_motor ->] motor) -> r x ([gearbox ->] propulsor)
```

- The motor shafts of one rotor combine on the rotor shaft (`connect(..., combine=True)`): equal speed, torques
  add. With `gearbox` given they combine on its input (as `build_series_hybrid` does), otherwise directly on the
  rotor shaft (direct drive, a shared shaft).
- `count_channels_per_rotor` is the `RedundancyLayer.count_lanes`. A `redundancy` layer passed in must agree
  (otherwise `ValueError`); without one the builder makes `RedundancyLayer(count_lanes=n)`, so buses and strings
  default to one.
- `electrical=None` connects the motors and the battery to the bus directly (the inverter is then lumped into the
  motor efficiency, as in the series hybrid). An `ElectricalLayer` adds the motor and battery feeders; its
  generator fields are unused and may be None.
- The series hybrid builder shares the redundancy-hardware and feeder helpers; its instances and connections are
  unchanged (same order).

Degraded state for one channel lost on one rotor (`degraded_state`):

- `condition.active_lane_count = n - 1` and `condition.count_rotors_lane_out = 1`.
- `DegradedState.count_lanes_active` is the lane count on the degraded rotors, `count_lanes_active_healthy` on
  the others (both reduced by any failed bus), `count_rotors_degraded` the number of degraded rotors (None means
  all, today's symmetric behaviour).
- `DegradedState.power_shaft_lane_W(power_shaft_rotor_W, degraded=True)`: each active lane carries the rotor shaft
  power over the active lanes, `P_rotor / (n - 1)` on the failed rotor and `P_rotor / n` on a healthy one. It is
  the identity behind sizing a channel from the failure case.
- The lumped topology carries one port value per instance (symmetric copies), so a single-rotor failure is
  evaluated on the degraded rotor; `flight_point` keeps treating lanes symmetrically, which is conservative for
  the healthy rotor.

## Verification (`tests/powertrain/test_all_electric_topology.py`)

- Instance counts and connections for 2 rotors and 2-6 channels, with and without the electrical layer, gearbox,
  strings and cross-strapped buses; `redundancy_counts` reads `n` lanes; a mismatched redundancy layer is rejected.
- Series hybrid unchanged: same instances, counts and connection order as before the refactor.
- Connection residuals close at a consistent hover point of the notional aircraft, with the inverter DC voltage
  solved by `asb.Opti` from the battery-terminal residual (all residuals constrained to zero; symbolic
  compatibility of the whole graph).
- Degraded state after one channel lost on one rotor: `n - 1` lanes on one rotor, `n` on the other; validation
  of `count_rotors_lane_out`; a failed bus on top of it.
- Channel power identity `P_rotor / (n - 1)` in the failed case and `P_rotor / n` healthy; symbolic rotor power.
- Design margins of the notional aircraft are finite, and the per-channel motor rating covers the one-out hover.

## Assumptions

- Channels are identical and share the rotor load equally (symmetric lanes); a failed channel delivers zero
  torque and is isolated by its feeder protection (no drag torque from a shorted machine).
- The notional aircraft is illustrative only: its numbers come from the references above, not from a closed design.

## Progress log

- 2026-10-12: plan written.
- 2026-10-12: landed. `build_all_electric`, the shared `_add_redundancy_hardware` helper and the generator-free
  feeder path; the series hybrid's instances, counts and connection order are identical to before (checked against
  the previous module for every electrical/redundancy combination, and pinned in a test). `degraded_state` takes
  `count_rotors_lane_out`. Notional aircraft (1,600 kg, 2 x 4.5 m rotors, ISA sea level): hover shaft power
  148.4 kW per rotor; per-channel rating and installed motor power for 2-6 channels per rotor 148.4/593.7,
  74.2/445.3, 49.5/395.8, 37.1/371.1, 29.7/356.2 kW (installed = 2 P_rotor n / (n - 1)); motors plus inverters
  145, 109, 97, 91, 87 kg. The 3-channel hover point closes every connection residual (largest 3e-12) at
  796.6 V inverter DC voltage and 398 A battery current, solved by `asb.Opti`. 18 tests in
  `tests/powertrain/test_all_electric_topology.py`; `test_redundancy`, `test_topologies`, `test_electrical` pass.
