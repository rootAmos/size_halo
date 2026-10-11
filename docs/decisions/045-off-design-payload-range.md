# Plan 045: off-design solve of the sized aircraft and the payload-range diagram

## Goal and scope

Every solve in `examples/halo_sizing.py` sizes the aircraft: the structural and drive correlations take the
take-off mass variable as their design gross weight, so the aircraft grows or shrinks with the mission. The README
lists the payload-range diagram as missing. This plan adds a fixed-design (off-design) solve: the sized aircraft,
frozen, flown at a different payload and range. With it, the payload-range diagram of the v3.6 reference.

In: a fixed-design option in the coupled solve, `solve_halo_off_design` (maximum payload at a given range, or
maximum range at a given payload), the corner points and a few intermediate points, the larger-tank case, tests,
the figure. Out: endurance, a fuel-tank mass model, a cabin/floor payload limit from structure, cruise-speed
schedules other than the sized speeds (see Assumptions).

## References

- Raymer, *Aircraft Design: A Conceptual Approach*, payload-range (ch. 17) and design gross weight in the weight
  correlations (ch. 15): structure is sized at the design gross weight, the airplane then flies at any operating
  weight up to it.
- Torenbeek, *Synthesis of Subsonic Airplane Design*, payload-range corner points (maximum payload, maximum fuel at
  maximum take-off weight, ferry).
- The first-order estimate (Breguet calibrated on the solved cruise, scratch script `payload_range_quick.py`):
  900 kg over 445 nm at the design point, 526 nm tank-limited ferry, about 995 nm at zero payload with fuel up to
  maximum take-off weight, about 57 nm per 100 kg of payload traded for fuel.

## Model

Where the sizing uses take-off mass (found by searching `build_halo_aircraft`, `solve_halo_sizing_once` and
`aircraft_closure.vehicle`/`weights`):

| Use | Sizing | Off-design |
|---|---|---|
| `StructuralDesignCondition.mass_design_kg` (Raymer wing, tails, fuselage, gear, flight controls; AFDD tiltrotor wing tip fraction and jump take-off thrust) | take-off mass variable | design gross weight, a number (the sized MTOW) |
| Same condition: cruise speed and L/D (Raymer wing/tail/fuselage operating point, fuselage L/D) | the solved cruise point | the sized numbers |
| Rotor, drive, machines, battery, turboshaft, heat exchanger, tails, wing area | design variables | the sized values (numbers), unit counts fixed |
| Mission, hover requirement, engine-out hover, failure hovers, stall lift | take-off mass | operating take-off mass (a variable): kept as constraints |
| Hot-day hover | mission end mass | mission end mass of the off-design mission |
| Mass closure | take-off mass = total mass | operating take-off mass = total mass (fixed empty mass + payload + fuel loaded) |
| Fuel | fuel = 1.1 x fuel burnt | fuel loaded = 1.1 x fuel burnt (same reserve) |
| Hover pitch trim (CG at the wing quarter chord) | equality | dropped as a constraint; the CG offset is reported (payload and fuel sit at the rotor station, so it stays zero) |
| Turboshaft mass-power regression | equality | dropped (both are fixed numbers) |

New limits (margins, so they appear in the binding list): operating take-off mass at or below the design gross
weight; fuel loaded at or below the tank capacity; payload at or below the design payload.

Operating variables: payload (maximum-payload problem) or mission range (maximum-range problem), the loaded fuel, the
operating take-off mass, and every flight-point state (rotor speeds, powers, battery split, SOC, temperatures), as in
the sizing. Cruise and loiter speeds are held at the sized values by default (an option frees them).

`solve_halo_off_design(sized, range_nm=..., ...)` maximizes payload at that range; `solve_halo_off_design(sized,
mass_payload_kg=..., ...)` maximizes range at that payload. `payload_range_diagram` runs the corner points.

## Verification

- Design-point reproduction: maximum payload at 445 nm returns 900 kg, the sized fuel and take-off mass, within the
  solver tolerance; empty mass equals the sized empty mass.
- Monotonic payload-range: maximum range falls as payload rises along the tank-limited line; maximum payload falls
  as range rises.
- Fuel accounting: fuel loaded = 1.1 x the sum of the segment fuel burns; take-off mass = empty + payload + fuel.
- Every margin of the sizing (hover, engine-out, failure hovers, hot-day hover, stability, whirl placement) holds
  at each off-design point.

## Assumptions

- Tank capacity = the sized fuel mass (936 kg for v3.6): the tanks hold exactly the design mission's fuel. No tank
  mass is modelled, so the "larger tanks" line is the payload-range if tankage were free.
- Payload capacity = the design payload (900 kg); the cabin and floor are not designed for more. Below the design
  range the aircraft could lift more payload by mass alone; that is not shown.
- Cruise and loiter speeds held at the sized speeds (165 and 134 kt). An operator would fly long-range cruise at
  low weight; freeing them is an option (`hold_speeds=False`).
- Same mission shape as the sizing (60 s hovers, climb, cruise, 20 min reserve loiter, descent), same 10 % fuel
  reserve; range is take-off to landing ground distance.
- Battery starts each mission at the take-off SOC (0.95) and must end above the reserve SOC, as in the sizing.

## Progress log

- 2026-10-10: plan written.
- 2026-10-10: `OffDesign`, `solve_halo_off_design` and the `range` objective in `examples/halo_sizing.py`;
  `solve_halo_sizing_once` split into `halo_design_variables` and `_solve_halo_coupled` with the default problem
  built unchanged (same variables, constraints and order). Margins that depend only on the frozen design (rotor
  clearance) are numbers off-design: reported, not constrained. `OffDesign.start` takes the take-off mass, payload,
  fuel and range guesses from an earlier off-design point: the larger-tank zero-payload point (969 nm) failed
  (restoration) from the sized point and solves when chained from the 25 % payload point; the ferry at free speeds
  likewise. `examples/halo_payload_range.py` runs the points (one coupled solve each, about 1-2 min) and draws
  `docs/figures/payload_range.png`. Starting reference: the cached v3.6 solve (6,885 kg, 15,179 lb).

### Results (v3.6 reference: MTOW 6,885 kg, empty 5,049 kg, tanks 936 kg)

| Point | Payload kg (lb) | Range nm | TOW kg | Fuel kg | End SOC | Binding limits |
|---|---|---|---|---|---|---|
| Design (max payload at 445 nm) | 900.0 (1,984) | 445.0 | 6,885 | 936 | 0.43 | payload, MTOW, tanks |
| Max payload at 267 nm | 900 (1,984) | 267 | 6,553 | 604 | 0.59 | payload |
| Max payload at 490 nm | 188 (415) | 490 | 6,173 | 936 | 0.30 | tanks, SOC floor |
| Tanks full, 675 kg | 675 (1,488) | 462 | 6,660 | 936 | 0.30 | tanks, SOC floor |
| Tanks full, 450 kg | 450 (992) | 475 | 6,435 | 936 | 0.30 | tanks, SOC floor |
| Tanks full, 225 kg | 225 (496) | 487 | 6,210 | 936 | 0.30 | tanks, SOC floor |
| Ferry (tanks full, no payload) | 0 | 500 | 5,985 | 936 | 0.30 | tanks, SOC floor |
| Ferry, cruise/loiter speeds free | 0 | 505 | 5,985 | 936 | 0.30 | tanks, SOC floor |
| Larger tanks, 675 kg | 675 (1,488) | 576 | 6,885 | 1,161 | 0.30 | MTOW, SOC floor |
| Larger tanks, 450 kg | 450 (992) | 705 | 6,885 | 1,386 | 0.30 | MTOW, SOC floor |
| Larger tanks, 225 kg | 225 (496) | 836 | 6,885 | 1,611 | 0.30 | MTOW, SOC floor |
| Larger tanks, no payload | 0 | 969 | 6,885 | 1,836 | 0.30 | MTOW, SOC floor |

- Design point reproduced: 900.0 kg over 445 nm at 6,885.1 kg and 936.4 kg fuel, empty mass 5,048.6 kg (equal to the
  sizing within 0.1 kg); the CG stays on the trim point (offset < 1e-14 m, payload and fuel at the rotor station).
  Every sizing margin holds at every point (smallest -1e-8, the frozen rotor-clearance margin, which was already
  binding at the sizing).
- "Maximum fuel at MTOW" coincides with the design point: the tanks hold exactly the design fuel, so the tank and
  MTOW limits meet there. Below 445 nm the payload capacity (900 kg) limits; beyond it the tanks.
- Tank-limited line: 13 to 17 nm per 225 kg of payload removed (445 to 500 nm). Larger tanks: about 58 nm per
  100 kg of payload traded for fuel at MTOW, 969 nm at zero payload (1,836 kg fuel).
- Battery: the design mission ends at SOC 0.43 (the battery is sized by the engine-out and failure hovers, not by
  energy). Off-design the maximum-range solves use the battery down to the 0.30 mission floor: about 9 kWh of the
  70 kWh pack, worth about 3 kg of fuel (2 nm). The engine-out reserve is still held from that floor.

### Comparison with the first-order estimate

| | Breguet (calibrated on the solved cruise) | Coupled off-design solve |
|---|---|---|
| Design point | 445 nm | 445 nm |
| Ferry, tanks full | 526 nm | 500 nm (505 at free speeds) |
| Zero payload, fuel to MTOW | 995 nm | 969 nm |
| Payload traded for fuel at MTOW | 57 nm / 100 kg | 58 nm / 100 kg (first 225 kg) |

The estimate holds the powertrain efficiency constant. The solve does not: at the ferry weight the cruise power is
722 kW against 821 kW at the design point (L/D 9.26 against 9.07, so drag falls with weight as Breguet assumes),
but the turboshafts run at lower part power and their fuel per kWh rises from 339 to 355 g/kWh (+5 %). That is the
whole ferry difference (5 % of 450 nm of cruise, about 25 nm). On the larger-tank line the aircraft starts at MTOW
like the design mission, so the two agree until the long cruise takes it to light weights late in the flight
(26 nm, 3 %, at zero payload). Freeing the speeds gains only 5 nm: the optimizer cruises faster (178 kt against 165)
to keep the engines at higher power, and loiters slower.

## Slide summary

**The sized Halo can now be flown off-design: 900 kg over 445 nm, 500 nm ferry on its own tanks, 969 nm if the
tanks could be filled to MTOW.**

- Design point reproduced exactly by the fixed-design solve (900 kg, 445 nm, 6,885 kg, 936 kg fuel); every hover,
  engine-out, failure and stability requirement holds at every point.
- Tanks sized to the design mission make the payload-range almost vertical: removing all 900 kg of payload adds only
  55 nm (500 nm ferry). Larger tanks would trade about 58 nm per 100 kg of payload, to 969 nm.
- The first-order Breguet estimate was 26 nm optimistic on the ferry: the turboshafts burn 5 % more fuel per kWh at
  the lighter aircraft's part power.

Caveat: tankage is free in the larger-tank line (no tank mass or volume model), the cabin limit is taken as the
900 kg design payload, and the mission keeps the sized speeds and the design reserve rules.
