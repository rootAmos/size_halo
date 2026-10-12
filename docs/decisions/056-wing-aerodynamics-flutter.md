# Plan 056: classic wing flutter, and the nacelle CG trade against whirl flutter

Status: DONE (2026-10-11). Branch `feat/wing-aero-flutter`.

## Goal

The plan 040 whirl-flutter model has no wing aerodynamics (CAMRAD practice for the published XV-15 studies) and
three assumed wing shapes, so it cannot see classic wing bending-torsion flutter. Its nacelle-CG lever says the whirl
flutter speed peaks with the CG on the wing elastic axis and falls on both sides; classic flutter favours mass ahead
of the elastic axis. This plan adds a classic-flutter check from the classical literature, verifies it, and plots
both boundaries against the nacelle CG position.

Not in scope: wing aerodynamics inside the whirl model (the coupled rotor-wing problem); compressibility beyond a
lift-slope factor; the flutter constraint in the sizing loop (plan 055).

## Literature

- Theodorsen, NACA Report 496 (1935); Theodorsen and Garrick, NACA Reports 685 (1940) and 741 (1942): typical-section
  flutter with unsteady aerodynamics; trends in the CG-to-elastic-axis offset. Some original plots are wrong (Zeiler
  2000); Perry recomputed all three (NASA TPs; J. Aircraft 62(1), 2024, doi 10.2514/1.C038025).
- Goland, J. Appl. Mech. (1945): the uniform cantilever benchmark; corrected strip-theory flutter 137.2 m/s,
  70.68 rad/s at 1.225 kg/m^3 (Cal, City University PhD thesis 1992, tables 2.4-2.5; lifting line 158.5 m/s).
- Goland and Luke (1948), Runyan and Watkins, NACA TN 1848 / Report 966 (1949): a uniform wing with a concentrated
  mass; "a definite region exists ... in which masses added forward of the elastic axis ... increase the flutter
  speed"; for a weight of 92 % of the wing mass 0.41 chord ahead of the elastic axis, a two-mode Rayleigh analysis
  "failed to give any reasonable results" (mass at mid-span), hence beam elements here, not assumed modes.
- Runyan and Sewall (NACA TN, 1948): concentrated weights on a cantilever wing, experiments.
- R. T. Jones' two-lag approximation of Theodorsen's C(k); Yates, NACA RM L57L10 (1958): modified strip analysis
  (finite-span lift slope and aerodynamic centre in strip theory).
- 14 CFR 25.629 and AC 25.629-1: flutter-free to 1.15 V_D.
- Whirl side, for context: Gul and Yeo (J. Aircraft, 2024) found the MTR pylon CG behind the wing elastic axis
  improved whirl stability; Akinwale and Datta (AIAA 2025-0429) tests point the same way. The plan 040 model says
  behind the axis is worse; unresolved here.

## Method

`aeroelasticity/wing_flutter.py` (new; `whirl_flutter.py` unchanged):
- Uniform cantilever in beam finite elements: Hermite bending (w, w'), linear torsion, optional chord bending; the
  section CG may sit off the elastic axis. Rigid tip body (nacelle): mass, CG offset, pitch, roll and yaw inertias.
- Theodorsen strip theory at four Gauss points per element: non-circulatory terms and the circulatory lift through
  Jones' C(k) (lags 0.0455 and 0.3), so the flutter problem is an exact eigenvalue problem in airspeed; flutter is
  the first oscillatory root with positive real part, divergence the first real one.
- Optional finite-span lift-slope factor (Helmbold) on the circulatory lift (Yates-type); a reduced two-mode basis
  (bare-wing first bending and torsion) for the Runyan-Watkins comparison.
- `examples/halo_flutter_nacelle_cg.py`: nacelle CG from 0.75 m ahead to 0.5 m behind the elastic axis (mass and
  pitch inertia about the CG held, hub 1.0 m ahead, as plan 040): classic flutter (2-D and finite-span lift slope,
  full and two-mode) and the plan 040 whirl flutter (calibrated, delta-3 -18 and -10 deg); figure
  `docs/figures/flutter_nacelle_cg.png`.

## Verification

- Goland wing: 137.5 m/s, 69.4 rad/s (16 elements; 4 elements 138.7 m/s) against 137.2 m/s, 70.68 rad/s.
- Uncoupled bending and torsion frequencies within 0.2 / 0.5 % of beam theory.
- A tip mass of 92 % of the Goland wing mass: flutter speed falls as it moves aft (0.3 m ahead: divergence first at
  252 m/s; on the axis 218; 0.3 m behind 151), the classical trend. At the tip the two-mode basis agrees within 2 %.
- `tests/aeroelasticity/test_wing_flutter.py` (8 tests).

## Assumptions

- Halo wing uniform: AFDD reference stiffnesses (plan 040 `reference.json`), mass per length from the wing mass,
  torsional inertia m (c / 4)^2, section CG on the elastic axis at 0.375 chord (mid-box), sea level.
- Incompressible strip theory (conservative against lifting line by about 15 % on the Goland wing; aspect ratio 6.1
  here); no structural damping (conservative).

## Results (Halo v3.6 reference)

| Nacelle CG | Classic, 2-D | Classic, finite span | Whirl d3 -18 (loiter / max) | Whirl d3 -10 |
|---|---|---|---|---|
| 0.75 m ahead | divergence 1,027 kt | 1,205 kt | 255 / 238 kt | 316 / 282 kt |
| 0.50 m ahead (baseline) | divergence 1,027 kt | 1,205 kt | 262 / 265 kt | 325 / 313 kt |
| 0.25 m ahead | divergence 1,027 kt | 1,205 kt | 269 / 291 kt | 333 / 342 kt |
| on the axis | 955 kt | 1,129 kt | 277 / 322 kt | 341 / 353 kt |
| 0.25 m behind | 664 kt | 764 kt | 269 / 298 kt | 316 / 326 kt |
| 0.50 m behind | 547 kt | 626 kt | 249 / 282 kt | 292 / 309 kt |

- Classic flutter follows the classical rule (aft is worse) but stays above 1.8 x the 302 kt requirement over the
  whole range: the 23 % thick, stiff box makes it inactive. Ahead of the axis the wing diverges before it flutters.
- Whirl flutter is the active boundary; it peaks near the elastic axis. A CG slightly ahead of the axis (0.25 m) with
  delta-3 -10 deg clears 302 kt and keeps the forward mass balance.
- Open: whirl flutter with the wing aerodynamics and the beam-element wing modes (the plan 040 airframe from these
  modes), and the MTR finding that a CG behind the axis helps.

## Progress log

- 2026-10-11: literature review (above); quasi-steady assumed-mode draft discarded for the beam-element Theodorsen
  model; Goland verified; Halo sweep and figure.
