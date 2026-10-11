# Plan 040: reduced-order proprotor whirl flutter

## Goal

Replace the frequency-placement proxy for whirl flutter (plan 024: wing torsion and beam frequencies above a set
number per rev) with a flutter speed from a coupled rotor-pylon-wing model, so the sizing can trade wing stiffness,
nacelle mass and offset, and rotor parameters against flutter speed directly (`docs/NEXT_STEPS.md` item 1).

## Scope

In: the coupled model, the XV-15 comparison against CAMRAD II with a calibration, tests. Out (next): the Halo
flutter check, the constraint inside `asb.Opti`.

## References

- Johnson, "Dynamics of Tilting Proprotor Aircraft in Cruise Flight", NASA TN D-7677, 1974: the proprotor and
  wing formulation (quasi-steady blade-element derivatives at high inflow, gimbal, lag, wing modes).
- Johnson, NASA TM X-73213: pitch couplings and the XV-15 delta-3 with the pitch horn level at 20.5 deg
  collective.
- Acree, Peyran and Johnson, NASA/TP-2004-212262: XV-15 stick-model hub modes for the 0.23 and 0.15 t/c wings,
  the steel-blade gimballed rotor deck, and the CAMRAD II flutter boundaries used for the calibration.
- Reed, classical propeller whirl flutter (rigid propeller on a flexible pylon): the limiting case in the tests.

## Model (`aircraft_closure.aeroelasticity.whirl_flutter`)

Johnson's proprotor formulation (NASA TN D-7677; TM X-73213) at conceptual order, in `aerosandbox.numpy` so the
matrices stay symbolic in the design inputs:

- **Rotor:** gimbal (tip-path-plane tilt, gimbal spring); blade lag (cyclic, collective) and coning bending as
  elastic rotations along the section principal axes pitched by the collective at 0.75 R (about 45 deg in cruise),
  which couples lag with flap; quasi-static pitch couplings: pitch-gimbal (delta-3, growing with collective from
  the pitch-horn-level collective), pitch-lag, pitch-cone; drive train locked to the nacelle (antisymmetric,
  interconnect shaft) or free on its inertia with a governor (symmetric).
- **Aerodynamics:** blade element linearized about the steady high-inflow state (inflow angle kept), uniform steady
  angle of attack from the rotor thrust, Prandtl-Glauert lift slope on the helical Mach number with optional lift
  divergence, quasi-steady, no tip loss, no dynamic inflow.
- **Hub loads:** thrust, in-plane forces (aerodynamic, thrust tilt with the blade slope, blade inertia), gimbal
  spring, gyroscopic hub, shaft torque perturbation, and the steady thrust and drag torque turning with the shaft.
- **Airframe:** any linear model seen from the hub: published hub modes (CAMRAD QFREQ/QMASS/QDAMP/LSHAPE/ASHAPE),
  or a three-mode cantilever wing with a rigid tip nacelle built from the AFDD wing stiffnesses. No wing
  aerodynamics (the CAMRAD practice in the XV-15 study).

`whirl_modes` and `flutter_speed` are numerical checks (eigenvalues, a scan, then bisection) outside the optimizer.

## Verification (tests/aeroelasticity)

- Gimbal modes at (nu - 1) and (nu + 1)/rev in vacuum; cantilever tip-mass frequencies; steady thrust reproduced.
- Classical propeller whirl (Reed): a stiff propeller on a soft pylon loses stability in **backward** whirl while
  forward whirl stays damped; flutter speed rises with pylon stiffness; a gimbal is more stable than a stiff
  propeller.
- Mirror image with opposite rotation gives identical eigenvalues (sign conventions).
- Damping and stiffness matrices stay CasADi expressions in wing torsion stiffness and rotor speed.
- XV-15 regression (below).

## XV-15 comparison (`examples/whirl_flutter_xv15.py`)

Inputs from NASA/TP-2004-212262 (Acree, Peyran, Johnson): stick-model hub modes for the 0.23 and 0.15 t/c wings,
the steel-blade gimballed rotor deck (Lock number 3.76 from the blade mass distribution; gimbal 1.02/rev; lag
1.80/rev and coning 1.52/rev by Ritz on the deck's chordwise and flapwise stiffness, the coning matching the 1.55
used by the weights; delta-3 -18 deg with the horn level at 20.5 deg collective, TM X-73213), 458 rpm, sea level,
limited-power trim. Pitch-lag coupling -0.2 (lag back / pitch up) is assumed, inside the range of Johnson's figure.

| Model build | 0.23 t/c | 0.15 t/c |
|---|---|---|
| Rigid in-plane, fixed delta-3, incompressible | stable to 500 kt | stable to 500 kt |
| + pitched lag/coning, delta-3(collective), Prandtl-Glauert, pitch-lag | 399 kt (sym beam) | 342 kt (sym beam) |
| x knockdown 0.84 (set on the thick wing) | 335 kt | **287 kt** |
| CAMRAD II (flexible blades and pitch dynamics) | 335 kt (antisym beam) | 275 kt (antisym beam) |

- The model reproduces the thick-to-thin drop (57 kt against 60) without tuning; the calibrated thin-wing boundary
  is 4 % above CAMRAD's.
- The rigid-blade result agrees with the report's rigid-blade case: the instability needs the blade bending and
  pitch couplings, as Acree and Johnson both state.
- Uncalibrated, the model is about 64 kt high (non-conservative), and its critical mode is symmetric beam where
  CAMRAD's is antisymmetric beam. Remaining differences: blade torsion and pitch dynamics (Johnson: mostly
  quasi-static, but they move the boundary), real bending mode shapes, CAMRAD's airfoil tables.
- Sensitivities: trim thrust has almost no effect; the structural damping convention matters (QDAMP as g/2 versus a
  damping ratio moves the thin wing from 342 to 383 kt); pitch-lag beyond -0.5 destabilizes the rotor's collective
  lag mode instead.

**Decision:** use the model with the 0.84 airspeed knockdown (XV-15-calibrated, like the weight factors) for
gimballed stiff-inplane proprotors, with the frequency-placement margins kept alongside until the Halo check is
done. Requirement: calibrated flutter speed at least 1.15 x V_D at every rotor speed.

## Halo check (`examples/halo_whirl_flutter.py`, 2026-10-10)

Airframe: the cantilever wing with a rigid tip nacelle from the sized AFDD wing (15,179 lb baseline:
`examples/halo_structure_reference.py`, now also exporting the whirl inputs). Rotor: the Halo rotor (4.83 m, 3
blades, solidity 0.060) with the XV-15 blade dynamics scaled (Lock number, per-rev frequencies, couplings).
Assumed: nacelle CG 0.5 m ahead of the spindle, spindle on the elastic axis, free electric drive, sea level,
V_D = 1.25 V_max.

Concept-airframe check on the XV-15: the cantilever builder gives 303 kt calibrated against 335 kt from the NASTRAN
hub modes, so it is about 10 % conservative (fuselage-fixed beam mode).

| Rotor speed | Calibrated flutter speed |
|---|---|
| 23.8 rad/s (reserve loiter) | 262 kt |
| 27.6 rad/s (climb, descent) | 272 kt |
| 29.4 rad/s (cruise) | 274 kt |
| 31.2 rad/s (climb, ceiling) | 275 kt |
| 39.0 rad/s (maximum speed) | 265 kt |
| Required, 1.15 x V_D (V_D = 1.25 x 210 kt) | 302 kt |

- The margin is -9 to -13 % with wing beam critical; with the cantilever's 10 % conservatism removed it is about
  -4 to 0 %: marginal, not cleared.
- Torsion stiffness is the lever (at the maximum-speed rotor speed: x0.75 -> 237 kt, x1.25 -> 283, x1.5 -> 296);
  beam stiffness helps less (x1.5 -> 276), chord stiffness does nothing.
- The frequency-placement margins of plan 024 are met by this wing, so the proxy does not catch the shortfall.

### Torsion stiffness to clear 1.15 V_D (`examples/halo_whirl_flutter_torsion.py`)

The AFDD wing is re-sized with a higher torsion frequency target (same design rotor speed, 56.3 rad/s, and same
gross weight). The torque box thickens, so beam stiffness also rises a little. The flutter speed is the minimum over
the airplane-mode rotor speeds.

| Torsion frequency | GJ | EI beam | Flutter (calibrated) | Wing mass (x factor_wing) |
|---|---|---|---|---|
| x1.00 | x1.00 | x1.00 | 262 kt | +0 kg |
| x1.40 | x1.96 | x1.04 | 284 kt | +110 kg |
| x1.73 | x2.98 | x1.09 | 294 kt | +222 kg |
| x2.05 | x4.19 | x1.27 | 302 kt | +361 kg |

- The baseline box is at the 1 mm minimum gauge, so stiffness up to that gauge is free. Beyond it, cost is roughly
  linear in GJ.
- Returns diminish: the critical mode stays wing beam at the 23.8 rad/s reserve-loiter rotor speed. Clearing
  302 kt on torsion alone costs about 360 kg of wing, 5 % of take-off mass, before the weight spiral. That is not a
  credible fix by itself.
- Cheaper levers come first:
  - a rotor-speed schedule that keeps the low reserve-loiter rpm away from high speed (the 39 rad/s point is 265 kt);
  - pylon geometry (nacelle CG and spindle relative to the elastic axis, both assumed here);
  - delta-3 and pitch-lag;
  - replacing the cantilever with whole-aircraft modes, worth about 10 % on the XV-15.

### Pylon and rotor-coupling levers (`examples/halo_whirl_flutter_levers.py`)

Each row is one change from the reference. The table gives the calibrated flutter speed at the reserve-loiter
(23.8 rad/s) and maximum-speed (39.0 rad/s) rotor speeds. The critical mode is wing beam unless noted. Required:
302 kt.

| Change | 23.8 rad/s | 39.0 rad/s |
|---|---|---|
| Reference (CG 0.5 m ahead, hub 1.0 m, delta-3 -18 deg, pitch-lag -0.2) | 262 | 265 |
| Nacelle CG on the spindle (0 m) | 277 | 322 |
| Nacelle CG 0.25 m ahead | 269 | 291 |
| Nacelle CG 0.75 m ahead | 255 | 238 |
| Hub 0.6 m ahead | 274 | 281 |
| Hub 0.8 m ahead | 268 | 273 |
| Hub 1.2 m ahead | 257 | 258 |
| delta-3 -30 deg | 195 | 197 |
| delta-3 -10 deg | 325 | 313 |
| delta-3 0 deg | 412 (coning) | 364 |
| Pitch-lag -0.4 | 251 | 253 |
| Pitch-lag 0 | 274 | 277 |
| Pitch-lag +0.2 | 285 | 289 |

- delta-3 is the strongest lever. Reducing it from -18 to -10 deg clears 1.15 V_D at both rotor speeds (+4 to +8 %)
  for no weight. It is also where the model is least validated: the XV-15 knockdown was fitted at -18 deg.
  Smaller delta-3 raises airplane-mode flapping and moves the gimbal frequency, and at 0 deg the rotor's own coning
  mode becomes critical. Any delta-3 change needs a flapping and loads check.
- Nacelle mass placement matters at high rotor speed: CG on the spindle gives 322 kt at 39 rad/s, but only 277 kt at
  reserve loiter.
- A shorter mast helps modestly (0.6 m: +12 to +16 kt). Pitch-lag toward positive helps (+0.2: +23 kt), but pitch-lag
  beyond about +0.2 has not been checked against the collective lag mode.
- Proposed path: delta-3 about -10 deg plus the CG as close to the spindle as the layout allows, verified on
  whole-aircraft modes; torsion stiffness only for what remains.

## Assumptions

Collected from the sections above:

- Rotor: XV-15 blade dynamics scaled to the Halo rotor (Lock number, per-rev frequencies, couplings);
  pitch-lag -0.2 (inside Johnson's range, not measured); quasi-steady aerodynamics, no tip loss, no dynamic
  inflow; Prandtl-Glauert on the helical Mach number.
- Airframe: three-mode cantilever wing with a rigid tip nacelle from the sized AFDD wing, about 10 %
  conservative against the XV-15 NASTRAN hub modes; no wing aerodynamics (the CAMRAD practice).
- Halo installation: nacelle CG 0.5 m ahead of the spindle, spindle on the elastic axis, free electric drive,
  sea level, V_D = 1.25 V_max.
- Calibration: airspeed knockdown 0.84, fitted on the XV-15 thick wing at delta-3 -18 deg; least validated
  away from that delta-3.
- Requirement: calibrated flutter speed at least 1.15 V_D at every airplane-mode rotor speed.
- `whirl_modes` and `flutter_speed` are numerical checks (eigenvalues, a scan, then bisection) outside the
  optimizer; the matrices stay symbolic (tested).

## Progress log

- 2026-10-10 15:21: model and XV-15 calibration (82a65fa).
- 2026-10-10 15:45: Halo check against 1.15 V_D (c491fcb).
- 2026-10-10 17:04: torsion stiffness and wing weight to clear it (0c02ac5).
- 2026-10-10 17:28: pylon and pitch-coupling levers (c596dae).
- 2026-10-10 17:33: tutorial 26 (d80106a); merged to main in PR #17.
- 2026-10-10: audited against the restored AGENTS.md; References, Assumptions and this log added. The
  pylon-CG follow-up that sat uncommitted on the merged branch moved to its own branch,
  `feat/whirl-pylon-cg` (work in progress, no plan yet).

## Next

1. Wing torsion for flutter: the GJ (or nacelle offset, delta-3) that clears 1.15 V_D, and its wing weight;
   whole-aircraft symmetric and antisymmetric modes instead of the cantilever to remove its conservatism.
2. In the loop: the least-damped eigenpair at 1.15 V_D / 0.84 as optimizer variables (A v = lambda v,
   normalization), Re(lambda) at or below a damping margin, so the wing is sized for flutter speed directly.
3. Fidelity: blade torsion with control-system stiffness (pitch-lag from first principles rather than assumed),
   then a second calibration aircraft (JVX/V-22 data) for the knockdown.
