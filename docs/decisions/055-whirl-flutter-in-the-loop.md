# Plan 055: whirl flutter in the sizing loop

## Goal

Size the Halo wing against the calibrated whirl-flutter model of plan 040 directly, instead of the per-rev frequency
placement proxy that the reference meets while fluttering at 262 kt against 302 kt (1.15 V_D). With flutter a
constraint, the wing aspect ratio, thickness and delta-3 can become design variables without the optimizer thinning
or stretching the wing into flutter (plan 046 freed t/c against the proxy and landed on a wing that flutters at
256 kt).

## Method

- `HaloAssumptions.whirl_flutter_model = "eigenpair"` replaces the per-rev margins. At every airplane-mode point
  (one per distinct rotor speed) the plan 040 model is built from the sized wing (AFDD stiffnesses, wing mass, tip
  mass and pylon inertia, rotor radius and solidity): cantilever semispan with the tip nacelle, XV-15 blade dynamics
  scaled to the Halo rotor.
- At V = 1.15 V_D / 0.84 (the knockdown applied as an airspeed factor, as in plan 040; sea level) the
  `count_modes_whirl` least-damped modes of the starting design are optimizer variables: eigenvalue
  lambda = sigma + i omega and eigenvector v with (lambda^2 M + lambda C + K) v = 0 and one component of v fixed to
  1 + 0 i. Each residual row is scaled by its starting magnitude. The margin is the damping ratio -sigma / |lambda| at
  or above `damping_min_whirl` (0).
- Smooth in every input of M, C, K; no eigenvalue routine inside the optimizer.
- Mode switching: only tracked modes are constrained, and a tracked eigenpair can slide onto another (better-damped)
  mode. After every solve the plan 040 eigenvalue scan (`flutter_speed`) runs on the solved design per rotor speed
  (`HaloSizingResult.whirl_flutter_check`); if it finds a flutter speed below 1.15 V_D, the solve is repeated from
  that solution with the eigenpairs seeded from its least-damped modes (`rounds_whirl_reseed`, 3 at most; a
  different starting point, counted in `whirl_rounds`).
- Stiffness lever: the AFDD wing is built for a wing design rotor speed (a design variable); a higher one is a
  stiffer, heavier wing. Delta-3 (`free_delta3`, bounds -30 to -10 deg) is the strongest plan 040 lever.

## Verification

- Unit: numerical eigenpairs satisfy the residual; on a pylon with a stiff propeller, the minimum pylon stiffness with
  the tracked mode neutrally stable at 150 m/s gives a scanned flutter speed of 150 m/s (within 0.5 m/s).
- Halo: the solved design's scanned calibrated flutter speed is at least 1.15 V_D at every rotor speed.

## Results (2026-10-11, `examples/halo_whirl_flutter_loop.py`)

One coupled solve per case from the same sized start (plan 045's cached reference, 6,486 kg), same code. Required:
1.15 V_D = 302 kt calibrated. "Flutter" is the post-solve eigenvalue scan's lowest calibrated flutter speed over the
airplane-mode rotor speeds (always wing beam).

| Case | Take-off | Wing | AR | t/c | delta-3 | Wing design rotor speed | Cruise L/D | Flutter | Rounds |
|---|---|---|---|---|---|---|---|---|---|
| Frequency proxy (current reference) | 13,834 lb | 282 kg | 6.12 | 0.23 | -18 | 57.4 rad/s | 8.88 | below 302 (plan 040: 262) | - |
| Flutter in the loop | 14,227 lb (+393) | 328 kg | 6.12 | 0.23 | -18 | 70.2 rad/s | 9.10 | 302 kt | 0 |
| + delta-3 free | 13,834 lb (+0) | 282 kg | 6.12 | 0.23 | -11.0 | 57.4 rad/s | 8.88 | 314 kt | 0 |
| + AR and t/c free | 13,923 lb (+89) | 295 kg | 6.99 | 0.280 (bound) | -18 | 70.4 rad/s | 8.79 | 302 kt | 2 |
| + AR, t/c and delta-3 free | 13,484 lb (-350) | 251 kg | 7.23 | 0.269 | -10 (bound) | 52.6 rad/s | 8.71 | 302 kt | 0 |
| Proxy, AR and t/c free | 13,279 lb | 266 kg | 8.54 | 0.244 | -18 | 39.9 rad/s | 9.43 | 188 kt | - |

- **Clearing flutter on stiffness alone costs +393 lb** (+2.8 %): the wing is built for a higher design rotor speed
  (torque box 79 -> 121 kg) and the airplane-mode rotor speeds rise off the 23.8 rad/s reserve loiter (the worst
  case in plan 040). About half of plan 040's torsion-only estimate (+361 kg wing).
- **Delta-3 clears it for nothing in this model:** -11 deg gives 314 kt with the proxy's wing. The model has no
  cost on delta-3 (flapping and loads are not modelled) and the knockdown was fitted at -18 deg, so this needs the
  flapping and loads check of plan 040 before it is a decision.
- **Freed against flutter, the wing gets longer and thicker** (AR 7.0-7.2, t/c 0.27-0.28): thickness buys
  stiffness cheaply, span buys induced drag; cruise L/D falls (8.7-8.8) but the aircraft is lighter. With delta-3
  free as well: -350 lb against the proxy reference.
- **The proxy with the same freedoms is unsafe:** AR 8.5 and a thinner wing reach 13,279 lb but flutter at 188 kt.
  The frequency-placement proxy must not be used with a free wing.
- **Mode tracking:** in the AR/t/c case the optimizer lowered the reserve-loiter rotor speed and that point's
  tracked eigenpair slid onto a well-damped mode (damping 0.52) while the wing beam mode went unstable (scan: 283 kt).
  Tracking four modes instead of two did not help; re-seeding from the solution (`rounds_whirl_reseed`, here 2
  rounds) did. Every result reports `whirl_rounds` and the scan (`whirl_flutter_check`); `whirl_flutter_cleared`
  tells whether a result passed.
- The rotor speeds of the airplane-mode points are warm-started from the start's `whirl_flutter` in this mode (the
  flight points otherwise start at 100 rad/s, a supersonic tip, where the eigen-solution is undefined). The proxy row
  was solved with the same warm start; the default path keeps its generic start (`test_max_payload` depends on it).

## Assumptions

- Plan 040's: XV-15 knockdown 0.84 fitted at delta-3 -18 deg, cantilever airframe (about 10 % conservative against
  the XV-15 NASTRAN modes), nacelle CG 0.5 m ahead of the spindle, sea level, V_D = 1.25 V_max, no wing aerodynamics.
- The knockdown is applied to the airspeed, not to a damping margin.

## Progress log

- 2026-10-11: eigenpair constraint and tests; sizing option `whirl_flutter_model="eigenpair"`; scan-verified
  re-seeding after a mode slid at reserve loiter; results table.

## Next

1. Delta-3: a flapping and loads model so it has a cost, and a knockdown check away from -18 deg.
2. Make the eigenpair model the reference once delta-3 is settled; re-baseline (the proxy reference flutters).
3. Spanwise wing: root and tip t/c (the user's direction: pick the section offline, scale thickness along the span).
4. Feed the flutter-bounded levers into the plan 042 ledger (aspect ratio and t/c are flutter-limited).
