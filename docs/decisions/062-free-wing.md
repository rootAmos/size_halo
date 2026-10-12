# Plan 062: the free wing (64A223 section, bottom-up box with strength, free aspect ratio, thickness and camber)

## Goal and scope

User (2026-10-12): "run these 4 steps", after reviewing why the v3.10 wing is area-plus-position only:

1. Wing section NACA 64A223 (the XV-15 section the plan 057 drag calibration used) in place of NACA 2423, with the
   wing-box depth from that section's own thickness distribution.
2. The bottom-up wing box (plan 054) with the ultimate spar-cap strain margin (plan 043) in the sizing.
3. Aspect ratio, thickness ratio and camber free (plans 046, 055), safe now that the eigenpair whirl-flutter
   constraint is in the loop (v3.10).
4. The CalculiX check on the sized wing.

Differentiability (checked 2026-10-12): 1-3 are smooth in the optimizer (CST section family scaled symbolically
through NeuralFoil; smoothed maxima in the wing box; beam-theory strain). 4 is a check on the sized aircraft
(rule 4), not in the loop.

Out of scope, with the reason:
- Taper: the bottom-up box sizes a constant chord; station-varying chord is a part 2.
- Twist: AeroBuildup uses only the area-weighted mean incidence (root +2 / tip -4 deg gives CL and CD identical to
  the untwisted wing), so a twist variable would act only as wing incidence. It needs a span-loading model
  (lifting line) first.

Every change is behind options; the default sizing (v3.10, 14,350 lb) is unchanged.

## References

- NASA TM X-62407 (XV-15 sections: wing NACA 64A223, horizontal tail 64A015); plan 057.
- Abbott and von Doenhoff, Theory of Wing Sections (6A series: thickness at 40 % chord, 64A210 camber line).
- Plans 035 (box depth), 043 (cap strain), 046 (section family), 054 (bottom-up box), 055 (eigenpair flutter).

## Model

- `naca_6a_section` moves from `examples/xv15_reference.py` to `aircraft_closure.aerodynamics.wing_section`
  (examples import it; src may not import examples). `section_airfoil(name)` returns the coordinate airfoil for
  "naca2423" (AeroSandbox) and "naca64a223" (the 64A210 camber line, thickness x 2.3).
- `WingSectionFamily(base_name=...)` fits its CST base from `section_airfoil`, so the family can sit on 64A223.
  `section_thickness_shape(name, x)`: the base's thickness at chord fraction x over its maximum (numbers).
- `size_wing_box(..., thickness_shape=None)`: the spar and box depths are thickness_to_chord x the base shape at
  the spar stations (None keeps the NACA 4-digit formula, the reference). Smooth in a symbolic thickness ratio.
- `HaloAssumptions.wing_section_base` ("naca2423" by default; "naca64a223"). The family's base camber is the
  base's own (2 % / 1.33 %); `camber_wing` sets the camber used (set 0.0133 with the 64A223 base for its own camber).

## Verification

- Section: the 64A223 coordinates reproduce 23.0 % thickness at 40 % chord and the 64A210 camber line; the family
  at the base thickness and camber recovers the base CST fit (as the plan 046 tests do for 2423).
- Box depth: the 2423 shape equals the NACA 4-digit formula (identity); the 64A223 depth at the spars is computed
  and smooth in a symbolic thickness ratio.
- Default sizing unchanged: existing pins.
- Sizings, each from the previous: A v3.10; B = A with the 64A223 section; C = B with the bottom-up box and the
  strain margin; D = C with aspect ratio, thickness and camber free. Then CalculiX on D.

## Assumptions

- The section maximum-lift surrogate (plan 046) stays the one fitted on the 2423 family; with the 64A223 base it
  is applied to the thickness and camber changes only.
- Battery model: the default (quasi_steady), so the wing effects are not mixed with plan 060.

## Progress log
