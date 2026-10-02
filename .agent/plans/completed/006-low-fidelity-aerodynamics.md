# Low-fidelity aerodynamics

Status: ACTIVE — user requested "go all the way" (Tiers 5-10) on 2026-10-01 and
delegated review; decisions are self-reviewed against AGENTS.md and the
aircraft-aero skill.

## Goal and scope

Tier 5. A simple analytic aerodynamic model on the Tier 4 geometry: linear lift,
component parasite-drag buildup and induced drag, with an additive drag
increment hook. Replace the assumed cruise lift-to-drag in the mass closure by
a cruise equilibrium (lift = weight) solved in the same Opti. Out of scope:
tail lift, trim and stability (Tier 6), stall/CLmax beyond a stated parameter,
propeller interference, compressibility beyond the AeroSandbox lift-slope
correction (cruise Mach < 0.3).

## References

AeroSandbox `library.aerodynamics`: `Cf_flat_plate`, `oswalds_efficiency`
(Nita and Scholz 2012), `CL_over_Cl` (Raymer 12.4.1 / DATCOM);
`aero_buildup_submodels.fuselage_aerodynamics_utilities.fuselage_form_factor`
(Götten et al. 2021); Raymer, Aircraft Design 5th ed., eqs. 12.30 (wing form
factor) and Table 12.6 interference factors; `asb.AeroBuildup` for comparison.

## Physics

- CL = CL_alpha (alpha - alpha_0L), CL_alpha = 2 pi CL_over_Cl(AR, M).
- CD0 = sum_i Cf(Re_i) FF_i Q_i S_wet,i / S_ref + CDA_misc / S_ref, with Re on
  the wing/tail MAC and the fuselage length; wing/tail FF = (1 + 0.6/(x/c)_m t/c
  + 100 (t/c)^4)(1.34 M^0.18); fuselage FF from AeroSandbox.
- CDi = CL^2 / (pi e AR), e from AeroSandbox (taper, AR, d_fuselage / span).
- CD = CD0 + CDi + sum(drag increments). Wing lift only (tail lift is Tier 6).

## Inputs, outputs, parameters

Inputs: aircraft geometry (duck-typed: wing, horizontal_tail, vertical_tail,
fuselage with `to_asb()`), velocity, altitude, alpha. Parameters: zero-lift
angle, interference factors, misc drag area, CLmax, (x/c) of max thickness.
Outputs: `AeroResult` (cl, cd, cd0, cdi, cl_alpha_per_rad, oswald_efficiency,
lift_N, drag_N, lift_to_drag, dynamic_pressure_Pa, mach, alpha_deg) and a
`ParasiteDragItem` breakdown.

## Variables, constraints

Caller: alpha_cruise_deg variable; cruise equilibrium lift_N == m g; mass
closure uses `lift_to_drag_cruise` from the aero result; alpha bounded by the
CLmax-derived stall angle as an inequality.

## Interfaces

```
aerodynamics/simple.py
    DragIncrement(label, cd)
    ParasiteDragItem(label, cd0)
    AeroResult(...)
    SimpleAerodynamics(alpha_zero_lift_deg=-4.0, cl_max=1.5,
                       thickness_location_chordwise=0.3, interference_wing=1.0,
                       interference_tail=1.05, interference_fuselage=1.0,
                       drag_area_misc_m2=0.25)
        .parasite_drag_breakdown(aircraft, velocity_m_s, altitude_m)
        .evaluate(aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=())
        .alpha_stall_deg(aircraft, velocity_m_s, altitude_m)
```

The discipline reads geometry from physical objects and imports no vehicle
module (layering).

## Tests

Linear lift identities (CL(alpha_0L) = 0, slope), CL_alpha below 2 pi and rising
with AR; CD0 equals the breakdown sum; Cf from AeroSandbox; drag polar identity
CD - CD0 = CL^2/(pi e AR); L/D max at CL* = sqrt(CD0 pi e AR) with
(L/D)max = 1/2 sqrt(pi e AR / CD0); drag increments add; symbolic alpha and
geometry; comparison with `asb.AeroBuildup` (same order: CL slope within 15 %,
CD0 within 50 %, reported, not tuned).

## Reference cases

1. Reference aircraft at 60 m/s, 1000 m: drag polar and L/D max.
2. Cruise closure: mass closure + lift = weight at 60 m/s gives the cruise L/D
   used by the fuselage correlation; MTOM changes negligibly versus the assumed
   12 (weak exponent), demonstrating the coupling.

## Implementation sequence

1. `aerodynamics/simple.py` + tests. 2. `examples/cruise_closure.py` +
integration tests. 3. Tier 5 notebook. 4. Docs, plan completion, commit, push.

## Acceptance criteria

All tests pass; identities hold; cruise closure solves with zero residuals;
Tier 5 notebook passes; earlier notebooks pass.

## Progress and decisions

- 2026-10-01: Plan written and self-reviewed. Raymer's wing form factor is
  written here (AeroSandbox has no standalone function for it); everything else
  reuses AeroSandbox.

## Deferred

Tail lift and trim drag (Tier 6), control-surface increments, propeller
interference and propwash, DATCOM parasite methods, AeroBuildup/VLM as an
alternative model class, ground effect, compressibility above M 0.3.
