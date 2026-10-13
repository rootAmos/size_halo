# Plan 061: overall-width limit (span + rotor diameter) and a free wing aspect ratio

## Goal and scope

Free the wing aspect ratio in the sizing now that whirl flutter is in the loop, after first bounding the
aircraft's footprint (user, 2026-10-12: "now that flutter is in the loop we should make it free, but first
define a span limit"; "keep the gear and look into a resize with the span limit").

In: a requirement `HaloRequirements.max_width_overall_m` that constrains the overall width with the rotors
turning, span + 2R; a re-size of the v3.10 reference with `free_aspect_ratio_wing=True` with and without the
limit; on the user's approval (2026-10-12: "call this the new aircraft baseline"), both as the defaults, v3.11,
with v3.10 kept as `requirements_v3_10` / `assumptions_v3_10` and earlier sets fixing AR through
`pre_plan061`. Out: landing-gear sizing (track, stroke, tip-over: the existing
retractable tricycle gear stays, Raymer GA correlations with the XV-15 alighting-gear factor); wing fold or
removable-wing mass; short rolling take-off as an off-design case.

## References

- FAA AC 150/5390-2D, Heliport Design (2023-01-05): the controlling dimension D is the greater of the overall
  length and the overall width, rotors or wings at full extent (1.8.6, 1.8.32). Ground-taxi route 1.25 D,
  hover-taxi route 1.67 D, paved taxiway 2 x undercarriage width (Table 3-1); GA TLOF 0.83 D, FATO 1.50 D
  (Table 2-1). These scale with D and set no hard limit.
- FAA AC 150/5300-13 airport design groups and ICAO Annex 14 Vol. I aerodrome reference codes: ADG II is a
  wingspan of 49 to under 79 ft, ICAO code B 15 to under 24 m. Taxiway and apron widths and separations are
  set by the group.
- Comparators (overall width with rotors turning): XV-15 17.4 m, AW609 18.3 m (GlobalSecurity), V-22 25.8 m.
- Plan 055: AR and t/c freed against the eigenpair flutter constraint went to AR 7.0 and t/c at its bound.

## Model

The limit is taken from the airport, not the heliport: Halo keeps its wheeled gear so it can ground-taxi
with the nacelles up, and on an airport taxiway the rotor tips are its wingtips. The requirement is
ICAO code B / FAA ADG II, span + 2R < 24 m (79 ft). Code A / ADG I (15 m) is out of reach: the two rotors
alone are 9.4 m.

    margin_below("width_overall_m", span_m + 2 radius_m, max_width_overall_m)

The rotor hubs are at the wing tips (tip nacelles), so the tip-path planes reach R past each tip. With the
rotor-to-fuselage clearance R <= (b - w_f)/2 - c also active, the two together give
R <= (W - w_f - 2c)/4 = 5.4 m at W = 24 m.

## Verification

- With `max_width_overall_m=None` the margin is absent and the v3.10 sizing is unchanged (the plan 035
  pinned-weight test).
- Re-size from the v3.10 result (`initial=`): AR free without the limit, then with the 24 m limit. Report
  take-off mass, AR, span, R, the overall width, the flutter speed and which constraints bind; the eigen scan
  must clear the flutter speed.

## Assumptions

- Width = span + 2R: hub on the wing tip, no nacelle overhang past the tip-path plane, nacelles at 90 deg.
- 24 m is a strict class boundary ("under 24 m"); the constraint is <= 24 m.
- AR bounds stay (4, 12); t/c stays fixed.

## Progress log

- 2026-10-12: plan written.
- 2026-10-12: `HaloRequirements.max_width_overall_m` in. Re-sized from v3.10 (`initial=`), AR free, with no
  limit and with 24 m. The two solves give the same design: **the 24 m limit is not active** (20.8 m).

  | | v3.10 (AR 6.12 fixed) | AR free (with or without 24 m) |
  |---|---|---|
  | Take-off | 6,509 kg (14,350 lb) | 6,424 kg (14,163 lb, -188 lb) |
  | Empty | 4,702 kg | 4,629 kg |
  | AR | 6.12 | 7.42 |
  | Wing area | 22.3 m2 | 18.0 m2 |
  | Wing loading | 292 kg/m2 (60 lb/ft2) | 357 kg/m2 (73 lb/ft2) |
  | Span, R | 11.68 m, 4.70 m | 11.56 m, 4.64 m |
  | Width, span + 2R | 21.1 m | 20.8 m |
  | Wing mass | 386 kg | 386 kg |
  | Battery | 61.4 kWh | 57.5 kWh |
  | Cruise, L/D | 165 kt, 8.98 | 169 kt, 9.09 |
  | delta-3 | -10.9 deg | -10.0 deg (upper bound) |
  | Scanned flutter speed (least) | 311 kt | 302 kt (cleared, 1.15 V_D) |

  The span does not grow: the rotor-to-fuselage clearance ties span to rotor size, and both stay where hover
  power puts them. Freeing AR instead narrows the chord until stall at 120 kt binds (newly binding), so AR
  rises through a smaller wing. The wing mass is unchanged: the torque box gets heavier for stiffness (+20
  kg) and the fairing and control surfaces lighter (-26 kg). The saving is in the powertrain (-57 kg) and
  fuel (-12 kg) from the higher L/D and the lighter aircraft. Flutter binds at the 210 kt point with delta-3
  at its -10 deg bound; the eigen scan clears it.
- 2026-10-12: v3.11 baseline (user: "call this the new aircraft baseline"). Defaults `free_aspect_ratio_wing=True`,
  `max_width_overall_m=24.0`; `pre_plan061` keeps AR fixed for every earlier set (it is part of `pre_plan055`,
  hence `pre_plan057`), `requirements_v3_9` / `requirements_v3_10` have no width limit. The electrical layer's
  cable span now uses the design aspect ratio (it took the fixed 6.12); no change at this precision.
  `examples/halo_whirl_flutter_loop.py` (plan 055 study) keeps AR fixed unless a case frees it. Default solve
  from v3.11's start: 14,163 lb, eigen scan cleared (least 301.9 kt at the 210 kt point). Plan 035 pin
  14,350 -> 14,163 lb.
