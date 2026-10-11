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

## Slide summary

(to follow)
