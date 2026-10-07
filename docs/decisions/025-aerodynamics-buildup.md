# Aerodynamics build-up: AeroBuildup, Scholz hand check, blown wing, download

Status: COMPLETED 2026-10-03. Tier 21 (roadmap "21 Aerodynamics", review item 8). Author
direction 2026-10-02: lean heavily on AeroSandbox's built-in aerodynamics;
Scholz's level-0 method fills only what `AeroBuildup` does not provide and is
an independent hand check; `SimpleAerodynamics` stays as the simplest model.

## Goal and scope

- **`BuildupAerodynamics`** (new, `aerodynamics/buildup.py`): the
  `SimpleAerodynamics` operating interface (`evaluate`, `alpha_stall_deg`,
  `lift_curve_slope_per_rad`, `surface_lift_curve_slope_per_rad`, `cl_max`,
  hover download) on `asb.AeroBuildup` run on `Aircraft.to_asb()`:
  - wing, tails, fuselage and the nacelles (AeroSandbox fuselages; the
    spinner is the nacelle's nose) with AeroBuildup's own NeuralFoil section
    polars, compressibility, form factors and post-stall;
  - added on top, because AeroBuildup does not model them: interference
    factors Q (Scholz Table 13.4) as (Q - 1) x component profile drag; the
    fuselage correction of the Oswald factor k_e,F (AeroBuildup assumes no
    fuselage); a miscellaneous drag area (fittings and fixtures); fixed
    landing gear; boundary-layer transition (AeroBuildup does not pass a
    transition location to NeuralFoil, so a thin airfoil adapter does);
  - blown-wing increments in airplane mode, when the caller supplies the
    rotor state;
  - hover download from wing and rotor geometry.
- **`ScholzAerodynamics`** (new, `aerodynamics/scholz.py`): level-0
  component build-up as the hand check, plus its closed-form pieces
  (turbulent Cf with Mach, form factors, Torenbeek wetted areas,
  interference table, Korn-Lock wave drag, Nita-Scholz Oswald factor).
  Linear lift as `SimpleAerodynamics`.
- **Blown wing** (`aerodynamics/slipstream.py`): momentum-theory slipstream
  over the immersed span: dynamic-pressure ratio, swirl angle, the lift,
  profile-drag and swirl-recovery increments.
- **Download** (`aerodynamics/download.py`): DL/T = C_D,v x S_wing under the
  contracted wake (flap-projected) / A_disk, the NDARC form.
- **Vehicle:** `Nacelles` gains optional `length_m`, `diameter_m`, `y_m`
  and `to_asb()`; `Aircraft.to_asb()` includes them when present.
- **Callers:** hover download read through
  `aerodynamics.hover_download_fraction(aircraft)` (flight point,
  trajectory); the flight point exposes rotor thrust as a variable when the
  model has a blown wing.
- **Halo:** `HaloAssumptions.aerodynamics_model` = "simple" (default,
  reference unchanged) | "buildup" | "scholz"; nacelle geometry and buildup
  drag inputs as assumptions; `build_halo_aerodynamics`.
- **Not in scope (deferred):** V-tail, conversion segments and blown wing in
  conversion (trajectory uses the unblown polar), trim drag, a fitted
  surrogate (only if the direct route is too slow).

## References

- D. Scholz, *Aircraft Design*, ch. 13 "Drag Prediction", HAW Hamburg
  (fetched 2026-10-03): eq. 13.8/13.10 Torenbeek wetted areas, 13.15
  component method, 13.17 turbulent Cf with Mach, 13.22 surface FF, 13.23
  DATCOM fuselage FF, 13.24 Raymer nacelle FF = 1 + 0.35/(l/d), Table 13.4
  interference factors, "laminar flow may exist on the front 10 % to 20 % of
  the wing".
- M. Nita, D. Scholz, "Estimating the Oswald factor from basic aircraft
  geometrical parameters", DLRK 2012 (slides, method 1): e = e_theo k_e,F
  k_e,D0 k_e,M; f(lambda) = 0.0524 l^4 - 0.15 l^3 + 0.1659 l^2 - 0.0706 l +
  0.0119; delta lambda = -0.357 + 0.45 exp(-0.0375 sweep_25); k_e,F = 1 -
  2 (d_F/b)^2; k_e,D0 = 0.804 (turboprop/general aviation); k_e,M = -0.00152
  (M/0.3 - 1)^10.82 + 1 above M = 0.3.
- Korn equation with Lock's critical-Mach criterion, as given by W. H.
  Mason (Virginia Tech, "Transonic aerodynamics of airfoils and wings"):
  M_DD = kappa_A/cos L - (t/c)/cos^2 L - CL/(10 cos^3 L), kappa_A = 0.87
  conventional sections; M_crit = M_DD - (0.1/80)^(1/3); CD_wave = 20
  (M - M_crit)^4 above M_crit.
- W. Johnson, "NDARC - NASA Design and Analysis of Rotorcraft Validation and
  Demonstration", AHS Aeromechanics 2010, Table 1: XV-15 cruise D/q 9.25 ft2
  (fuselage 1.56, fittings and fixtures 3.00, pylons 2 x 0.76, horizontal
  tail 0.63, vertical tail 0.36, wing 2.18); hover download model: fully
  developed wake velocity, vertical drag set to the known download, flap
  effect by projected area.
- XV-15 hover download 0.07 with flaps (NASA TM X-62407 sec. 5.1, already
  used since Tier 10b).
- Rotor slipstream development v(x) = v_i (1 + x/sqrt(x^2 + R^2))
  (McCormick, *Aerodynamics, Aeronautics and Flight Mechanics*); tip
  propellers rotating inboard-up reduce induced drag (Snyder and Zumwalt,
  J. Aircraft 6(5), 1969).

## Assumptions

- AeroBuildup with the "small" NeuralFoil model; transition forced at 10 %
  chord on both surfaces (Scholz) unless overridden.
- Whole-aircraft lift from AeroBuildup includes tail lift at zero tail
  incidence (no downwash in AeroBuildup); the stability derivatives keep the
  Tier 6 analytic isolated-surface slopes.
- `alpha_stall_deg` for the build-up, given the point's `aero`: alpha +
  (cl_max - CL)/CL_alpha, i.e. exactly CL <= cl_max (without `aero`, the
  AeroBuildup lift curve linearized through 0 and 8 deg).
- Halo nacelles: 9 ft x 3.3 ft bodies (consistent with the 95 ft2 wetted
  area already used for the cowling mass), centred on the wing tips;
  interference Q = 1.5 (mounted directly on the wing).
- Halo miscellaneous drag area for the build-up: XV-15 "fittings and
  fixtures" 3.00 ft2 (NDARC), the same fuselage. Gear retracted.
- Blown wing: half of each tip rotor's slipstream is over the wing; disk
  0.4 R ahead of the wing quarter chord (assumed); inboard-up rotation;
  ideal swirl v_theta = T / (rho A Omega 2R/3).
- Download: vertical drag coefficient calibrated so the XV-15 geometry gives
  0.07 with the same flap setting.

## Interfaces

- `BuildupAerodynamics(cl_max, interference, drag_area_misc_m2,
  drag_area_landing_gear_fixed_m2, xtr_upper, xtr_lower, n_crit,
  blown_wing=None, download=HoverDownload(), download_fraction_hover=None)`.
- `evaluate(aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(),
  temperature_offset_K=0.0, rotor_state=None)` -> `AeroResult`;
  `drag_breakdown(...)` -> items for reports.
- `hover_download_fraction(aircraft)` on all three models (Simple: the
  constant).
- `RotorState(thrust_per_rotor_N, speed_rotor_rad_s)`.
- `Nacelles(length_m=None, diameter_m=None, y_m=None)`, `Nacelles.to_asb()`.
- `HaloAssumptions.aerodynamics_model`, `length_nacelle_m`,
  `diameter_nacelle_m`, `drag_area_misc_buildup_m2`, `blown_wing`.

## Symbolic considerations

- AeroBuildup and NeuralFoil are written in `aerosandbox.numpy`; geometry
  (wing area, span) and operating point may be Opti expressions.
- Model choices are Python branches on configuration strings and booleans,
  never on symbolic values. `fmax` guards the Nita-Scholz Mach term.
- The blown-wing coupling (thrust depends on drag, drag on thrust) is an
  explicit Opti variable and equality, not an iteration.

## Tests

- Closed forms: Scholz Cf (13.17) at M = 0 vs 0.455/(log Re)^2.58; nacelle
  FF; DATCOM fuselage FF; Korn M_DD; Nita-Scholz e for a known geometry;
  download DL/T formula; slipstream momentum velocity.
- Limits: zero thrust -> zero blown increments; zero transition location
  increases drag; retracted gear -> no gear drag; flap deflection lowers
  download; M below M_crit -> no wave drag.
- Trends: CD rises with CL; blown increment rises with thrust; download rises
  with wing chord.
- Signs: inboard-up swirl increases lift and reduces drag; outboard-up the
  reverse.
- Symbolic: `BuildupAerodynamics` inside `asb.Opti` (cruise trim solve).
- Cross-checks: AeroBuildup vs Scholz CD0 within 30 %; XV-15 D/q against
  NDARC 9.25 ft2 within 25 % for both build-ups.
- Integration: Halo sizes with "buildup"; the Tier 14 trajectory problems
  solve with the build-up model.

## Acceptance

- Full unittest suite passes; Tier 21 notebook executed with all checks
  passing; Halo default reference unchanged (14,436 lb).

## Progress and decisions

- 2026-10-03: plan written. Prototype: one AeroBuildup evaluation 30 ms
  numeric; an Opti cruise trim with symbolic wing area solves in 0.4 s.
- Direct route kept (no surrogate). The first sizing took 370 s; the stall
  bound used two extra AeroBuildup runs per point. Passing the point's `aero`
  to `alpha_stall_deg` (interface change, also in `SimpleAerodynamics` and the
  trajectory) cut it to about 140 s. `expand=True` was slower (490 s);
  NeuralFoil "xxsmall" saved under 10 %.
- Swirl recovery: the first form (blown lift tilted forward by the swirl
  angle) gave 8x the swirl energy flux; replaced by an energy-bounded
  recovery with efficiency 0.5.
- AeroBuildup's Oswald already includes the Nita-Scholz mean k_e,D0 and
  NeuralFoil's viscous polar; only k_e,F and k_e,M are added.
- Main merged mid-plan (Tier 20 AFDD wing, plan 026: 900 kg, 14,247 lb). From
  the generic guess, the build-up constant-battery start then stopped at local
  infeasibility; `solve_halo_sizing` now starts a build-up solve from the
  Scholz solution when `initial` is None.
- Results (plan 026 requirements): buildup 13,639 lb, L/D 9.84; Scholz
  13,702 lb; no blowing 13,646 lb; simple 14,247 lb. Max payload at 210 kt
  with the build-up 1,842 kg (959 kg simple). 230 kt closes at 900 kg (13,766
  lb); 250 kt did not solve. XV-15 components 4.94 ft2 (buildup), 5.34 ft2
  (Scholz) against NDARC 6.25 ft2.
- Acceptance: 431 unittest cases pass; Tier 21 notebook 26/26 checks and the
  full suite inside it pass.

## Deferred

- V-tail, conversion segments, blown wing in conversion, trim drag.
- Calibrating transition location and the fittings area against XV-15 flight
  data (the build-up is 15–20 % low on the clean XV-15 components).
- Whether the build-up becomes the Halo default (author decision at merge).
