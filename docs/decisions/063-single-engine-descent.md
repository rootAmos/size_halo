# Plan 063: single-engine descent (option)

## Goal and scope

Let the descent be flown on one of the two turbogenerators, the other shut down by choice, as an option
(`HaloAssumptions.count_turbogenerators_descent`, default None: both run, so every reference result is unchanged).

Why: the two 1,120 hp engines are fixed (plan 017) and sized by hover and the engine-out case. The descent needs about
300 kW of shaft power, about 20 % throttle per engine with both running, where the part-power curve gives about half the
full-power efficiency. With the battery stepped on 4 nodes per segment (plan 060, v3.11), the optimizer alternates the
descent nodes between about 22 kW (near idle) and about 620 kW of turbine power while the shaft demand stays at 295 kW:
it is reaching for the single-engine operating point through the node split, helped by the trapezoidal fuel integral
on a concave fuel-flow curve. One engine at about 40 % throttle is the honest version of that.

Out of scope: single-engine cruise or loiter, a split-smoothing penalty, engine relight or shutdown transients.

## References

- Plan 017 (fixed off-the-shelf turboshafts), plan 014 (the 1,120 hp GASP part-power deck).
- Tier 17 (`active_generator_count` on hover segments for the turbogenerator-out reserve).
- Plan 060 (piecewise battery nodes and sub-steps).

## Model

`DescentSegment` gains `active_generator_count` (None: all), passed to its `FlightCondition`. The flight point already
uses that count for fuel flow, the turbine power-available limit, generator, inverter, cable and protection losses and
heat loads (`build_flight_point`), as the engine-out hovers do. `halo_mission` takes
`count_turbogenerators_descent` and the sizing passes `HaloAssumptions.count_turbogenerators_descent`.

## Verification

- Unit: the descent condition carries the count; None keeps all running (`tests/mission/test_mission.py`).
- Sizing: v3.11 with the battery stepped on 4 nodes per segment, both engines vs one in the descent; compare take-off
  weight, fuel burnt and the descent turbine and battery powers per node.

## Assumptions

- The shut-down engine contributes no fuel flow, power or heat; its mass stays.
- No relight time, no engine-out exposure penalty: an operational choice for an uncrewed aircraft, for the user to
  accept.
- The descent's power demand is unchanged (same speed and sink rate).

## Progress log

- 2026-10-12: option, unit test.
- 2026-10-12: v3.11 with the battery stepped (15 s), 4 nodes per segment, warm-started:

  | | both engines in descent | one engine in descent |
  |---|---|---|
  | take-off weight | 14,171 lb | 14,145 lb (-26) |
  | empty weight | 10,240 lb | 10,225 lb |
  | fuel burnt | 803.0 kg | 798.3 kg |
  | battery | 59.8 kWh | 59.4 kWh |
  | descent turbine power at the nodes | 22, 619, 22, 350 kW | 339, 327, 279, 280 kW |
  | descent battery power at the nodes | +311, -266, +310, -9 kW | -5, +6, +52, +51 kW |

  One engine removes the node-to-node oscillation: the running engine sits near 50 % throttle and the battery
  stays near zero. Binding set unchanged (flutter, rotor radius, engine-out hover voltage and power). Engine-out
  hover still ends on 525 V.
