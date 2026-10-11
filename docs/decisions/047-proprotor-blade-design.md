# Plan 047: proprotor blade design with a smooth blade-element momentum rotor

Status: IN PROGRESS 2026-10-10 (overnight, autonomous; assumptions are stated here, not asked).

## Goal and scope

The sizing's rotor (`MomentumProfileRotor`, plans 012 and 016) is momentum theory plus profile power, fitted to
full-scale JVX data. It knows solidity and tip speed, not blade geometry, so it cannot answer the proprotor
question that matters most: how much cruise efficiency a twist and chord distribution buys for a given hover
figure of merit. This plan adds:

1. `BladeElementRotor`: a blade-element momentum (BEM) rotor in `aerosandbox.numpy`, with Ning's single residual
   in the inflow angle per station and NeuralFoil section polars with local Mach. One Opti variable per station
   per operating point, with no hidden iteration.
2. A calibration and validation against the same JVX test data the current model uses.
3. A two-point (hover and cruise, with a maximum-speed check) blade design for the Halo rotor.

In scope: the standalone model, the JVX check, the Halo blade design, tests and one figure. Integration into the
full sizing (`rotor_model="bemt"`) is a stretch goal; it is attempted only if the standalone parts are solid.
The default sizing and every reference result stay unchanged.

## References

- A. Ning, "Using Blade Element Momentum Methods with Gradient-Based Design Optimization", Struct. Multidisc.
  Optim. 64(2), 2021 (CCBlade). This is the source for the single residual in the inflow angle, with Prandtl tip
  and hub loss.
- C. W. Acree, NASA/TM-2016-219070 (JVX hover and airplane-mode data, `data/rotors/`, plan 016).
- C. W. Acree, "Modeling Requirements for Analysis and Optimization of JVX Proprotor Performance", NASA/TM-2008-214581
  (JVX geometry: 25 ft, 3 blades, thrust-weighted solidity 0.1138, linear taper, tip chord 15.79 in, XN-28 root
  airfoil; bilinear twist with transition at 0.45 R, indexed to zero at 0.75 R; ideal hover/cruise twist).
- C. W. Acree, AHS Specialists' Conference 2008 (Table 1: JVX taper tip/root 0.65).
- XV-15 blade design (NASA ntrs 19810014531): 45 deg nonlinear twist between ideal rotor and ideal propeller.
  ntrs 19880013050: uniform-inflow hover analysis is insensitive to twist, so hover must stay calibrated.
- NeuralFoil (P. Sharpe), via `asb.KulfanAirfoil.get_aero_from_neuralfoil` (Mach: Laitone critical Mach and
  wave-drag model in AeroSandbox).

## Model (`aircraft_closure.powertrain.components.rotor_bemt`)

Stations: panel midpoints of a cosine-clustered (toward root and tip) split of [root cutout, R]; the loads are
integrated with the midpoint rule.

Per station with radius r, local solidity sigma' = B c / (2 pi r), inflow angle phi (Opti variable),
lambda_r = V / (Omega r):

- Pitch theta = collective + twist(r), with twist indexed to zero at 0.75 R; angle of attack alpha = theta - phi.
- Section cl, cd from NeuralFoil at alpha, with Reynolds and Mach on the undisturbed relative speed
  W0 = sqrt(V^2 + (Omega r)^2). Using W0 keeps the residual in phi alone (Ning does the same). The error is the
  induced part of the speed, a few percent in hover.
- cn = cl cos phi - cd sin phi (thrust), ct = cl sin phi + cd cos phi (torque).
- Prandtl factors: F = F_tip F_hub, F_tip = 2/pi acos(exp(-B/2 (R - r) / (r sin phi))), F_hub likewise.
- Residual: Ning's R(phi) = sin phi (1 - k) - lambda_r cos phi (1 + k') with k = sigma' cn / (4 F sin^2 phi) and
  k' = sigma' ct / (4 F sin phi cos phi), multiplied by sin phi:

      R = sin^2 phi - sigma' cn / (4 F) - lambda_r (sin phi cos phi + sigma' ct / (4 F)) = 0

  It is smooth for phi in (0, pi/2) and stays valid in hover (lambda_r = 0 gives the classical hover BEMT,
  sin^2 phi = sigma' cn / 4F). The propeller branch is used here: axial induction u = V / (1 - k), swirl
  1 - a' = 1 / (1 + k').
- Relative speed W = Omega r / ((1 + k') cos phi); dT/dr = B q c cn, dQ/dr = B q c ct r, with q = rho W^2 / 2.
- Outputs: thrust, torque, power, figure of merit T^1.5 / sqrt(2 rho A) / P, propulsive efficiency T V / P,
  section angle of attack, helical tip Mach, thrust-weighted solidity and C_T/sigma.

Blade: chord and twist at a few spanwise control points, interpolated by a fixed C2 cubic-spline basis, so the
station values are linear in the control values and stay symbolic; a linear basis is used for the bilinear JVX
twist. The section airfoil is a fixed thickness schedule (a NACA 64A2xx camber line with the thickness scaled
to the local t/c), not a design variable.

## Verification

- Zero-inflow limit: with lambda = 0 and no tip loss, F = 1 and a uniform-inflow blade reproduces momentum
  theory (ideal-twist blade at constant inflow gives the momentum induced power within the profile part).
- Betz-like case: an ideal (inflow-uniform) hover blade with zero drag reaches FM close to 1 without tip loss,
  and lower with it.
- Prandtl loss: thrust at a fixed collective falls when the loss is included; the loss vanishes as the blade
  count grows.
- CasADi: geometry control values and collective as Opti variables solve, and gradients exist.
- JVX: hover FM and airplane-mode efficiency against Tables D-1, D-2 and D-7 with a stated tolerance.

## Assumptions (made overnight, to be reviewed)

- JVX geometry: radius 150 in, 3 blades, linear chord with tip chord 15.79 in and taper 0.65 referred to r = 0,
  which reproduces the 0.1138 thrust-weighted solidity. Root cutout 0.10 R.
- JVX twist: bilinear, inboard -64 deg/R, outboard -34 deg/R, transition 0.45 R, zero at 0.75 R. This gives
  47.5 deg over the radius (the quoted V-22 effective twist); the exact rates in Acree's map are not in the text.
- Sections: t/c 0.28 at 0.10 R, 0.18 at 0.40 R, 0.12 at 0.75 R and 0.09 at the tip (the XN-28/18/12/09 family
  stand-in), on a NACA 64A210 camber line. NeuralFoil default transition (n_crit 9), no stall delay.
- Halo operating points from the cached 15,179 lb baseline (`halo-closure-system/output/ledger/baseline.pkl`):
  - hover, 4,000 ft ISA: 49.29 rad/s, thrust 38,004 N per rotor (T/W 1.05 plus download, blade loading 0.14);
  - cruise, 10,000 ft, 85.1 m/s: 29.36 rad/s, thrust 3,692 N;
  - maximum speed, 210 kt at 10,000 ft: 39.03 rad/s, with thrust from a drag polar D = a V^2 + b / V^2 fitted
    through the cruise and loiter thrusts.
  The rotor thrusts are backed out of the baseline's rotor powers with `MomentumProfileRotor`, so both models
  see the same operating points.

## Progress log

- 2026-10-10: plan written; JVX geometry gathered (TM-2008-214581, AHS 2008); NeuralFoil checked inside Opti
  (CasADi graph, about 0.05 s to build for 20 cases).
