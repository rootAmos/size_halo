# Plan 067: turbine maximum continuous power cap (option), and the payload and objective trade on the fixed engines

## Goal and scope

The user (2026-10-12): "upscaling the aircraft to carry more payload so that the battery is actually used in
takeoff and the rotor cruises at the turbine MCP". On v3.10 the two fixed 1,120 hp turboshafts give about
1,070 hp in cruise (about half of what they can deliver at 10,000 ft) and the take-off hover needs 1,750-1,870 hp
of the 2,240 available, so the battery gives only about 130 kW at take-off. A heavier aircraft on the same
engines uses more of them. The trade finds the payload at which the cruise turbine power reaches the maximum
continuous power (MCP) and reports how the battery is used on the way.

Decisions (user, 2026-10-12): range fixed at 445 nm; cruise speed free (optimized, as today); battery model
`battery_dynamics="piecewise"` with the plan 060 sub-steps, so the battery's use is resolved.

In scope: an MCP cap on the turbines at the airplane-mode mission points (opt-in), the turbine power available
per segment in the result, a payload sweep and the MCP crossover. Out of scope: re-sizing the engines (fixed by
decision), other requirements.

## References

- Halo baseline decisions (engines fixed at 2 x 1,120 hp, the GASP deck engine; take-off rating).
- GE T700-GE-701D ratings: 1,994 shp intermediate (take-off), 1,716 shp maximum continuous: MCP / take-off =
  0.86. Typical turboshafts sit between about 0.82 and 0.90.
- Plan 060 (battery sub-steps), plan 044 (objectives), plan 045 (off-design).

## Model

- `HaloAssumptions.fraction_mcp_turboshaft` (None by default: no MCP cap, the baseline unchanged). When set, at
  every airplane-mode mission point (climb, cruise, reserve loiter, descent nodes) the shaft power of each
  active turboshaft stays at or below the fraction x its power available at that point (lapse included):
  margin "<point>: turboshaft MCP power_W". Hovers and failure cases keep the take-off rating.
- Each segment summary carries `power_available_turboshafts_W` (active engines x take-off-rated power available
  at the segment's first point), so the cruise turbine power can be read against MCP.
- The sweep: `solve_halo_sizing` (minimum take-off mass) at payloads 900, 1,100, 1,300, 1,500, 1,700 kg with
  range 445 nm, speed free, piecewise battery, MCP cap at 0.86; each start from the previous result. Reported per
  payload: take-off, empty, fuel, battery energy, cruise speed, cruise turbine power / MCP, battery share of the
  take-off hover power, cost per payload kg-nm, the binding constraints.
- The crossover: the payload at which the cruise turbine power reaches MCP (the cap binds), from the sweep, then
  one sizing at that payload.

## Verification

- The MCP margin: a unit check that the cap equals the fraction x power available at the point's altitude.
- Default sizing unchanged (fraction None): the default test suite pins.
- Each sweep point: converged, closure residual and min margin as the reference solves; the 1 s battery replay
  (plan 060) on the crossover aircraft.

## Assumptions

- Continuous limit = 0.90 x the 1,120 hp rating at every altitude (same lapse), from the deck itself: the GASP_TS
  deck (`Aviary/aviary/models/engines/turboshaft_1120hp.deck`, converted from `MAPS_1120hp.eng`) gives
  `sls_horsepower: 1120` at `t4max: 50`, with `t4climb: 48.33` and `t4cruise: 46.67`. Its power at the cruise T4 is
  0.900 x the power at t4max at sea level and at 10,000 ft (climb T4: 0.950). So 1,120 hp is the maximum (take-off)
  rating and the cruise limit is 0.90, replacing the T700 ratio of 0.86 first assumed.
- Range, requirements, failure cases and every other assumption as v3.10.

## Progress log

- 2026-10-12: the deck header fixes the ratings: 1,120 hp is at t4max (maximum rating); the cruise T4 gives 0.90 of
  it, so `fraction_mcp_turboshaft = 0.90`. The deck's corrected power at 10,000 ft, de-corrected (x delta sqrt(theta)),
  is 900 hp per engine at t4max against the sizing's 878 hp (XV-15-fitted lapse), 2.5 % low. v3.10 re-sized for least
  cost per mission (objective "cost", 900 kg) cruises at the 210 kt limit on 1,484 hp: 84 % of the 1,756 hp maximum
  at 10,000 ft and 94 % of the 0.90 continuous limit (1,580 hp).
- 2026-10-12: renumbered from 061 (taken by the span limit, PR #50) to 067 when the option was merged. The
  payload sweep was stopped after 1,100 kg: 1,300 and 1,500 kg ended on `Invalid_Number_Detected` (the voltage root,
  fixed since in fc9e3ab), 1,150 kg on `Restoration_Failed`; at 1,100 kg (min take-off mass, speed free) cruise sat at
  74 % of 0.86 x MCP at 166 kt. The objective study replaced the sweep (default v3.10 assumptions, no cap, 445 nm):

  | | min take-off mass, 900 kg | min take-off mass, 1,100 kg | min cost, 900 kg | min cost, 1,100 kg |
  |---|---|---|---|---|
  | take-off | 14,350 lb | 15,413 lb | 14,818 lb | 15,873 lb |
  | cruise | 165 kt | 167 kt | 210 kt (bound) | 210 kt (bound) |
  | cruise turbines / (0.90 x max at 10,000 ft) | 67 % | 71 % | 94 % | 98 % |
  | battery | 61.4 kWh | 72.6 kWh | 75.8 kWh | 83.9 kWh |
  | SOC at landing | 0.30 | 0.30 | 0.95 | 0.95 |
  | cost per mission | $3,245 | $3,430 | $3,135 | $3,209 |

  Min cost flies the 210 kt bound and keeps the battery for engine-out; min take-off mass spends ground charge for
  fuel. The engine matches about 1,100 kg at 210 kt. Cost model maintenance lines read $0.
