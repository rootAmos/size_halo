# Plan 047: proprotor blade design with a smooth blade-element momentum rotor

Status: COMPLETED 2026-10-11 for the standalone model, the JVX check and the Halo blade design. Integration as
`rotor_model="bemt"` in the sizing is not done; it is the next step (see Next). This work was done overnight and
autonomously: the assumptions are stated here, not asked.

## Goal and scope

The sizing's rotor (`MomentumProfileRotor`, plans 012 and 016) is momentum theory plus profile power, fitted to
full-scale JVX data. It knows solidity and tip speed but not blade geometry. So it cannot answer the proprotor
question that matters most: how much cruise efficiency a twist and chord distribution buys for a given hover
figure of merit. This plan adds:

1. `BladeElementRotor`: a blade-element momentum (BEM) rotor in `aerosandbox.numpy`. It solves Ning's single
   residual in the inflow angle at each station, with NeuralFoil section polars at the local Mach. There is one
   Opti variable per station per operating point, and no hidden iteration.
2. A calibration and validation against the same JVX test data the current model uses.
3. A two-point blade design for the Halo rotor: hover and cruise, with a maximum-speed check.

In scope: the standalone model, the JVX check, the Halo blade design, tests and one figure. Integration into the
full sizing (`rotor_model="bemt"`) is a stretch goal, attempted only if the standalone parts are solid. The default
sizing and every reference result stay unchanged.

## References

- A. Ning, "Using Blade Element Momentum Methods with Gradient-Based Design Optimization", Struct. Multidisc.
  Optim. 64(2), 2021 (CCBlade): the single residual in the inflow angle, with Prandtl tip and hub loss.
- C. W. Acree, NASA/TM-2016-219070: JVX hover and airplane-mode data (`data/rotors/`, plan 016).
- C. W. Acree, "Modeling Requirements for Analysis and Optimization of JVX Proprotor Performance",
  NASA/TM-2008-214581. JVX geometry: 25 ft diameter, 3 blades, thrust-weighted solidity 0.1138, linear taper,
  tip chord 15.79 in, XN-28 root airfoil, bilinear twist with the transition at 0.45 R, indexed to zero at
  0.75 R. Also: the ideal hover and cruise twists, and that no stall delay gives poor hover.
- C. W. Acree, AHS Specialists' Conference 2008, Table 1: JVX taper (tip/root) 0.65.
- XV-15 blade design (NASA ntrs 19810014531): 45 deg nonlinear twist, between the ideal rotor and the ideal
  propeller. ntrs 19880013050: uniform-inflow hover analysis is insensitive to twist, so hover must stay
  calibrated.
- NeuralFoil (P. Sharpe), through `asb.KulfanAirfoil.get_aero_from_neuralfoil`. Mach effects come from
  AeroSandbox's Laitone critical Mach, shock-buffet and wave-drag model.

## Model (`aircraft_closure.powertrain.components.rotor_bemt`)

Stations are the panel midpoints of a cosine-clustered split of [root cutout, R], clustered toward root and tip.
The loads are integrated with the midpoint rule.

Per station with radius r, local solidity sigma' = B c / (2 pi r), inflow angle phi (an Opti variable) and
lambda_r = V / (Omega r):

- **Pitch and angle of attack.** Pitch theta = collective + twist(r), with twist indexed to zero at 0.75 R.
  Angle of attack alpha = theta - phi.
- **Section polars.** cl and cd come from NeuralFoil at alpha. Reynolds number and Mach are taken on the
  undisturbed relative speed W0 = sqrt(V^2 + (Omega r)^2), which keeps the residual in phi alone, as Ning does.
- **Force coefficients.** cn = cl cos phi - cd sin phi (thrust); ct = cl sin phi + cd cos phi (torque).
- **Prandtl loss.** F = F_tip F_hub, with F_tip = 2/pi acos(exp(-B/2 (R - r) / (r |sin phi|))) and F_hub
  likewise.
- **Residual.** Ning's R = sin phi (1 - k) - lambda_r cos phi (1 + k'), with k = sigma' cn / (4 F sin^2 phi) and
  k' = sigma' ct / (4 F |sin phi| cos phi), multiplied by sin phi:

      R = sin phi |sin phi| - sigma' cn / (4 F) - lambda_r (sin phi cos phi + sigma' ct / (4 F)) = 0

  - |sin phi| is smoothed as sqrt(sin^2 phi + 0.003^2) everywhere it appears.
  - For phi > 0 this is Ning's residual. In hover (lambda_r = 0) it reduces to the classical BEMT,
    sin^2 phi = sigma' cn / 4F.
  - The smoothing lets a lightly loaded tip carry upwash (phi < 0) without a branch. Without it, the low-thrust
    hover points have no solution.
  - The propeller branch is used: axial induction u = V / (1 - k), swirl 1 - a' = 1 / (1 + k').
- **Loads.** Relative speed W = Omega r / ((1 + k') cos phi). dT/dr = B q c cn and dQ/dr = B q c ct r, with
  q = rho W^2 / 2.
- **Rotational stall delay** (Snel's 3 (c/r)^2 factor, saturated by tanh): cl -> cl + K (cl_attached - cl), with
  cl_attached = 2 pi sin(alpha - alpha_0) / sqrt(1 - M^2). Without it, the inboard sections of the highly
  twisted JVX blade stall early and the high-thrust hover points are lost. Acree found the same with CAMRAD II.
- **Turbulent sections.** Transition is forced at 5 % chord on both surfaces, as on full-scale blades. With free
  transition, NeuralFoil's laminar runs made cruise about 0.05 too efficient against JVX.
- **One calibration constant.** `drag_factor` multiplies section drag; it is 0.964 from the JVX fit.
- **NeuralFoil inside Opti** (model size "medium"), not a fitted polar. A 3-point design problem (16 stations,
  56 variables) solves in 10-20 s; one trimmed point takes about 1 s.
- **Outputs:**
  - thrust, torque and power;
  - figure of merit T^1.5 / sqrt(2 rho A) / P and propulsive efficiency T V / P;
  - section angle of attack, cl and cd, inflow angle;
  - helical tip Mach, thrust-weighted solidity and C_T/sigma.

**Blade.** Chord and twist are given at a few spanwise control points. A fixed basis (C2 cubic spline or linear)
interpolates them, so the station values are linear in the control values and stay symbolic. Chord may use its
own control points: the Halo design uses a linear taper. The section airfoil follows a fixed thickness schedule
(a NACA 64A210 camber line with the thickness scaled to the local t/c) and is not a design variable.

Numerical analysis (`solve_point`, `solve_point_stepped`) trims one point to its thrust, outside any larger
problem. The stepped version walks a fixed thrust list (0.4, 0.7, 1.0 of the target), each step starting from the
last. Like the multistart of plan 029, this is a list of starting points, not a fixed-point iteration.

## Verification

### Unit tests (`tests/powertrain/test_rotor_bemt.py`, 13 tests)

- **Zero-axial-speed momentum limit.** The optimum hover rotor (chord proportional to 1/r, pitch alpha +
  atan(lambda/x)) with no drag and no loss, in effectively incompressible air with one section:
  - inflow comes out uniform: standard deviation below 3 %, mean equal to the design lambda;
  - each annulus satisfies the momentum balance sin^2 phi = sigma' cn / 4;
  - FM lies between sqrt(1 - x_root^2) - 0.03 and the momentum limit; the gap is swirl.
- **Propeller (Betz-like) limit.** Efficiency stays below the ideal momentum value 2 / (1 + sqrt(1 + T/(qA)))
  and comes within 0.03 of it with no drag or loss. Adding drag and Prandtl loss lowers it.
- **Prandtl loss.** Tip loss raises hover power, and the penalty shrinks with blade count at constant solidity
  (2 > 4 > 12 blades).
- **CasADi.** With chord, root twist and collective as Opti variables, the problem solves with the blade-loading
  constraint.
- **Interpolation.** The basis reproduces the control values and straight lines; twist is zero at 0.75 R.

### JVX check

`examples/jvx_rotor_bemt.py` runs every test point; `tests/integration/test_jvx_bemt.py` runs every 8th point
(5 tests). Each point is trimmed to its measured C_T at its own tip speed, density and speed of sound. Solutions
above the momentum limit are rejected as spurious branches.

| Quantity (drag factor 0.964, fitted to both modes) | BEM | Momentum-profile rotor (plan 016) |
|---|---|---|
| Hover FM, Table D-1, C_T/sigma 0.04-0.14 (50 points) | RMS 0.018 | RMS 0.012 (all D-1) |
| Hover FM, Table D-1, all 58 points | RMS 0.040 | |
| Airplane-mode efficiency, 42 points (lambda 0.26-0.56) | RMS 0.025, max 0.059, bias +0.015 | RMS 0.013, max 0.036 |
| Points without a solution | 3 of 113 | - |

- **Hover slope.** The BEM's FM curve is flatter than the data. It is about 0.05 low at C_T/sigma 0.02-0.035,
  within 0.02 from 0.04 to 0.11, and 0.03-0.04 high from 0.12 to 0.15.
- **Highest-thrust points.** The two highest-thrust points (C_T/sigma 0.144 and 0.160) trim only on a stalled
  branch: FM about 0.55 against 0.80 measured. They set the 0.040 full-set RMS and the 0.245 maximum D-2 error.
- **Airplane mode.** The BEM is slightly optimistic, +0.015 on average. The largest errors, +0.04 to +0.06, are
  at lambda 0.52-0.56 and low thrust.
- **Comparison with the sizing model.** The momentum-profile rotor fits the data better, with five fitted
  constants. The BEM has one constant and carries geometry.
- **XV-15.** No XV-15 geometry check was made: its performance tables are not in the repository, and the JVX blade
  is the closer Halo analogue. The XV-15 enters only as the linear-twist reference blade.

## Assumptions (made overnight, to be reviewed)

- **JVX planform.** Radius 150 in and 3 blades. Linear chord, with tip chord 15.79 in and taper 0.65 referred to
  r = 0. This reproduces the 0.1138 thrust-weighted solidity (the model gives 0.1138). Root cutout 0.10 R.
- **JVX twist.** Bilinear: -64 deg/R inboard and -34 deg/R outboard, with the transition at 0.45 R and zero at
  0.75 R. That gives 47.5 deg over the radius, the quoted V-22 effective twist. The exact rates of the test rotor
  are only marked in Acree's map figure, not given in the text.
- **Sections.** t/c is 0.28 at 0.10 R, 0.18 at 0.40 R, 0.12 at 0.75 R and 0.09 at the tip, standing in for the
  XN-28/18/12/09 family, on a NACA 64A210 camber line. Transition is forced at 5 % chord.
- **Halo operating points** from the cached 15,179 lb baseline (`halo-closure-system/output/ledger/baseline.pkl`):
  - hover at 4,000 ft ISA: 49.29 rad/s, thrust 38,004 N per rotor (T/W 1.05 plus download, blade loading 0.14);
  - cruise at 10,000 ft and 85.1 m/s (165 kt): 29.36 rad/s, thrust 3,692 N;
  - maximum speed, 210 kt at 10,000 ft: 39.03 rad/s, thrust 4,345 N.

  The maximum-speed thrust comes from a drag polar D = a V^2 + b / V^2 through the cruise and loiter thrusts.
  The rotor thrusts are backed out of the baseline's rotor powers with `MomentumProfileRotor`, so both models see
  the same operating points.
- **Halo blade sections.** The same family and t/c schedule as the JVX stand-in; root cutout 0.10 R.
- **Stall margin.** 2 deg below the first peak of the 2-D NeuralFoil lift curve at the hover Mach and Reynolds
  number, applied outboard of 0.40 R.
  - The first peak is stall at low Mach. Beyond the drag-divergence Mach it is AeroSandbox's shock-buffet onset:
    about 6 deg at 0.76 R and 2.5 deg at the tip.
  - Inboard of 0.40 R the model's rotational stall delay already raises the 2-D limit, so the margin is not
    applied there.
- **Design space.**
  - Chord: linear from root to tip, 0.10-0.80 m, taper (tip/root) 0.5-1.0. These bounds stand in for structure and
    manufacturing, which are not modelled.
  - Twist: given at 0.10, 0.40 and 1.0 R, cubic spline.
  - Fixed: rotor speeds at the baseline values, and the airfoil family.

## Results: Halo blade design (`examples/halo_blade_design.py`)

Objective: minimum cruise shaft power at 165 kt. Constraints:

- hover FM at or above a floor; the default is the sizing model's FM at the same point, 0.790;
- C_T/sigma <= 0.14;
- the stall margin above;
- helical tip Mach <= 0.80 (it is 0.66 at 210 kt);
- the chord bounds and taper above.

Powers are per rotor.

| Blade | sigma_tw | Hover FM (kW) | C_T/sigma | Cruise eta (kW) | 210 kt eta (kW) |
|---|---|---|---|---|---|
| Sizing model (momentum + profile, sigma 0.060) | 0.060 | 0.790 (742) | 0.140 | 0.880 (357) | 0.825 (569) |
| XV-15-like: constant chord, linear 40 deg, sigma 0.060 | 0.060 | no attached solution | 0.140 | 0.882 (356) | 0.843 (557) |
| Start: constant chord 0.40 m, twist 35/10/0/-5 deg | 0.079 | 0.816 (719) | 0.106 | 0.815 (386) | 0.754 (623) |
| **Optimum, FM >= 0.790** | **0.067** | **0.790 (742)** | 0.126 | **0.877 (358)** | 0.840 (559) |

**Optimum blade.**

- **Chord.** 0.530 m at 0.10 R, tapering linearly to 0.265 m at the tip. The taper bound of 0.5 is active.
- **Twist** relative to 0.75 R: +37.5 deg at 0.10 R, +16.7 deg at 0.40 R, 0 at 0.75 R and -6.5 deg at the tip. That
  is 44 deg in all, with more of it inboard than a linear twist: a nonlinear twist like the XV-15's.
- **Active constraints.** The FM floor, the outboard stall margin (at 0.4-0.6 R and about 0.76 R, where
  shock-buffet onset limits the section) and the taper bound.

Trade along the FM floor (each design starts from the previous one):

| FM floor | FM | sigma_tw | Cruise eta | 210 kt eta |
|---|---|---|---|---|
| 0.74 and 0.76 (inactive) | 0.773 | 0.067 | 0.880 | 0.845 |
| 0.78 | 0.780 | 0.066 | 0.879 | 0.844 |
| 0.79 (the sizing model's) | 0.790 | 0.067 | 0.877 | 0.840 |
| 0.80 | 0.800 | 0.069 | 0.871 | 0.831 |
| 0.81 | 0.810 | 0.073 | 0.855 | 0.808 |
| 0.82 | not reached with the stall margin | | | |

Sensitivity to the stall-margin definition, at the sizing model's FM floor:

| Stall check | sigma_tw | Cruise eta | 210 kt eta |
|---|---|---|---|
| None (only C_T/sigma <= 0.14) | 0.060 | 0.886 | 0.852 |
| 2 deg margin outboard of 0.40 R (chosen) | 0.067 | 0.877 | 0.840 |
| 0 deg margin outboard of 0.25 R | 0.068 | 0.868 | 0.823 |
| 2 deg margin outboard of 0.25 R | 0.076 | 0.850 | 0.799 |

**Findings.**

- **Cruise: the two models agree.** At the sizing's solidity, the BEM gives 0.882 against the sizing model's
  0.880.
- **Hover: they disagree.** At sigma 0.060, C_T/sigma 0.14 and a hover tip Mach of 0.71, a linearly twisted
  blade with JVX-like sections has no attached hover solution: the inboard sections stall.
  - Redistributing the twist alone recovers hover at sigma 0.060 (the "None" row), but with no stall margin.
  - With a 2 deg margin the blade needs sigma 0.067, i.e. C_T/sigma 0.126.
- **Why.** The momentum-profile model carries JVX's high-thrust performance over to a slender sigma 0.060 blade.
  That performance comes from stall delay on a sigma 0.114 blade, which a slender blade does not get.
- **Cost of the fix.** At equal hover FM the designed blade gives up 0.3 points of cruise efficiency against the
  sizing model, which is within the model's JVX error. The hover result is not within that error: the solidity
  floor should rise from 0.060 to about 0.067.
- **Caveat.** The result depends on the stall criterion (see the sensitivity table). The shock-buffet limit used
  outboard is AeroSandbox's model, not data. Both need a mid-fidelity check.

## Results: Halo sizing with the BEM solidity floor (`examples/halo_blade_solidity_sizing.py`)

One full solve, started from the cached baseline, with `HaloAssumptions(solidity_min=0.067)`. The rotor is still
the momentum-profile model, now held at sigma 0.067.

| | Baseline (floor 0.060) | Floor 0.067 |
|---|---|---|
| Take-off mass | 6,885 kg (15,179 lb) | 6,918 kg (15,252 lb), +33 kg (+0.5 %) |
| Fuel | 936 kg | 955 kg (+19 kg) |
| Solidity | 0.060 (at the bound) | 0.067 (at the bound) |

The solidity the blade design needs costs about 73 lb of take-off mass through the momentum-profile model's
profile power. This is small next to the hover finding itself: the 0.060 rotor may not hover at the required
thrust with a stall margin.

## Next

1. **`rotor_model="bemt"` in the sizing.**
   - Each airplane-mode and hover flight point would carry its 16 inflow angles and a collective, with the blade
     (chord, twist) shared across points. That is about 17 variables and one NeuralFoil graph per point, and
     roughly 15 points per sizing solve: heavy but feasible.
   - A cheaper first step is a fitted polar: a smooth surrogate of the section cl and cd per station in place of
     NeuralFoil inside the sizing.
   - The BEM's hover slope (FM too flat against C_T/sigma) should be calibrated before the BEM replaces the
     momentum-profile model.
2. **Mid-fidelity check.** Check the optimum and the stall finding with DUST or FLOWUnsteady (both validated on
   the XV-15). Take high-Mach section polars from a transonic source rather than NeuralFoil's buffet model.
3. **Rotor speed as a design variable.** Cruise sits at the momentum model's advance-ratio bound (lambda 0.60).
   The BEM could replace that bound with stall and compressibility physics.
4. **Airfoil schedule and structure.** Make camber and t/c design variables, and add a structural chord/taper
   constraint from the blade weights.

## Progress log

- 2026-10-10: plan written. JVX geometry gathered (TM-2008-214581, AHS 2008). NeuralFoil checked inside Opti
  (about 0.05 s to build the CasADi graph for 20 cases).
- 2026-10-10: model and tests implemented.
  - Low-thrust hover needed the smoothed |sin phi|.
  - High thrust needed the stall-delay model, a bounded collective (the starting guess +/- 15 deg) and the thrust
    step list.
- 2026-10-10: JVX check.
  - Free transition with drag factor 1.0-1.3 left cruise 0.05-0.07 optimistic.
  - Turbulent sections with drag factor 0.964 balance hover and airplane mode.
- 2026-10-11: Halo design.
  - The XV-15-like blade and the constant-chord sigma 0.06 blade both stall in hover.
  - A free cubic chord produced an unphysical planform that exploited stall delay, so chord was restricted to a
    linear taper.
  - The stall angle taken as the argmax of cl was noisy because of buffet. The first lift-curve peak is used
    instead, with the check outboard of 0.40 R (sensitivity table above).
  - New field `HaloAssumptions.solidity_min`; its default, 0.06, keeps the reference. One sizing solve with 0.067.

## Slide summary

**Headline: the sizing's 0.060-solidity rotor is too slender to hover at C_T/sigma 0.14 with a stall margin; the
designed blade needs 0.067.**

- **Model check.** The BEM (Ning's residual, NeuralFoil sections, one fitted constant) against full-scale JVX: hover
  FM within 0.018 RMS for C_T/sigma 0.04-0.14, airplane-mode efficiency within 0.025 RMS.
- **Halo blade** (R 4.83 m, 3 blades) at the sizing's hover FM of 0.790:
  - chord tapers from 0.53 to 0.27 m;
  - twist is 44 deg, nonlinear;
  - cruise efficiency is 0.877 against the sizing model's 0.880, and 0.840 against 0.825 at 210 kt.
- **Hover-cruise trade.** An XV-15-like linear-twist blade at sigma 0.060 stalls inboard in hover. Raising FM to
  0.81 costs 2.5 points of cruise efficiency. Re-sizing with the 0.067 solidity floor costs +33 kg (15,179 to 15,252 lb).
- **Caveat.** The hover stall finding rests on NeuralFoil's 2-D stall and shock-buffet limits and a simple
  stall-delay model. The BEM also misses the two highest-thrust JVX points, so it may be conservative near stall.
  A vortex-wake check (DUST or FLOWUnsteady) should confirm it before the reference solidity floor is changed.
