# Rotor speed physics, calibrated on full-scale JVX proprotor data

Status: COMPLETED 2026-10-03. This is Tier 12 of the 2026-10-02 roadmap (external review
item 1: "the rotor model can't see rotor speed"). The data is
user-supplied: NASA/TM-2016-219070.

## Goal and scope

Replace the actuator disk's constant hover figure of merit (0.67) and
constant cruise coefficient (0.87) with a rotor whose power depends on rotor
speed, blade loading and inflow. That makes rotor speed (the "optimum-speed
rotor"), solidity and tip speed real design trades. The actuator disk stays
as the simplest model.

### Model: `MomentumProfileRotor` (distinct class; it needs rotor speed as an input)

Coefficients use rotor-disk normalisation: CT = T / (rho A (Omega R)^2),
CP = P / (rho A (Omega R)^3), lambda = V / (Omega R), sigma = solidity.

- **Induced power:** CP_i = kappa CT (lambda + lambda_i) - CT lambda, with
  the momentum inflow lambda_i = -lambda/2 + sqrt(lambda^2/4 + CT/2), and
  separate kappa for hover and airplane mode. The useful power CT lambda is
  the propulsive part; total CP = CT lambda + (induced loss) + profile.
- **Profile power:** CP_0 = (sigma / 2) c_d I(lambda), with
  I(lambda) = integral over x from 0 to 1 of x^2 sqrt(x^2 + lambda^2) dx,
  in closed form, and I(0) = 1/4 (hover is evaluated with the exact hover
  value, not the log limit).
- **Blade drag:** c_d = d0 + d2 (CT/sigma)^2, which is the rise with
  blade loading.
- **Limits:** blade loading CT/sigma <= 0.14 (stall margin; JVX data
  reach 0.16), and helical tip Mach <= 0.80.
- **Inputs:** `evaluate(axial_velocity, atmosphere, thrust_N, speed_rad_s)`
  returns shaft power, figure of merit or propulsive efficiency, CT/sigma
  and tip Mach. Pure equations; the caller owns the limit constraints
  (`get_limits`).

### Calibration (least squares, explicit and reported)

- **Hover:** fit kappa_h, d0 and d2 to JVX Table D-1 (Mtip 0.67–0.68,
  wind-corrected CP/sigma against CT/sigma; 0.02 to 0.16). Check against
  Table D-2 (Mtip 0.73).
- **Airplane mode:** fit kappa_a (and an airplane-mode d0 if hover d0 does
  not carry over, because highly twisted blades are off-design in cruise)
  to JVX Phase II Tables D-6 and D-7. These cover lambda = 0.263, 0.349,
  0.438, 0.523 and 0.562 at Vtip of about 640–695 ft/s. The tables are
  parsed by word coordinates, because their text layer wraps.
- **Report:** residuals; propulsive efficiency against CT/sigma at each
  lambda, against the measured data (Fig. 33).

### Framework changes

- **Flight point:** passes rotor speed to the rotor model. Mode-specific
  constants are chosen by `in_airplane_mode()` as now. It adds the blade
  loading and tip-Mach constraints when the rotor defines them.
  `ActuatorDiskPropulsor.evaluate` accepts and ignores `speed_rad_s`, so
  both models are interchangeable.
- **Halo-class sizing:**
  - solidity and design hover tip speed become design variables (bounded:
    sigma 0.06–0.14; hover tip Mach <= 0.70);
  - rotor speed stays per flight point;
  - the AFDD rotor mass uses the variable solidity (chord) and tip speed;
  - the gearbox mass uses the design rotor speed;
  - the motor sees the speed schedule (still the specific-power mass until
    Tier 13).

## Assumptions

- Uniform-inflow momentum theory with empirical kappa; no swirl term (it
  folds into kappa_a); no tip-loss model (it folds into kappa).
- The JVX rotor (V-22-class, 25 ft, 3 blades, sigma 0.105) represents the
  Halo proprotor class.
- No compressibility drag rise is fitted, because the data span only tip
  Mach 0.58–0.73. The 0.80 helical tip-Mach bound keeps the solution inside
  that range or close to it.
- No profile-power factor for blade twist in hover beyond what d2 absorbs.

## Tests

- **Identities:**
  - hover CP = kappa CT^1.5 / sqrt(2) + sigma c_d / 8;
  - I(0) = 1/4 and I(lambda) matches numerical quadrature;
  - figure of merit and efficiency definitions;
  - power is unchanged by consistent unit scaling.
- **Limits:** CT -> 0 gives profile power only; efficiency tends to the
  ideal momentum value as profile drag goes to 0.
- **Trends:**
  - power falls when rotor speed falls at low thrust (profile-dominated);
  - optimum speed exists at cruise;
  - figure of merit rises with CT/sigma over the data range.
- **Calibration:**
  - fitted model within stated RMS of D-1;
  - D-2 predicted within 3 %;
  - airplane-mode efficiency within stated RMS of Phase II.
- **Symbolic:** CasADi throughout.
- **Integration:**
  - the flight point enforces the tip-Mach and blade-loading bounds;
  - Halo sizing solves;
  - cruise rotor speed falls below hover (an "optimum-speed rotor" result)
    for physical reasons rather than motor-map ones.

## Acceptance

- Tests and notebooks pass.
- The Tier 12 notebook shows the fits, the optimum-speed result, and the
  resized Halo-class aircraft against the 0.67/0.87 baseline.
- The dashboard is updated.

## Progress and decisions

- 2026-10-02: Plan written; JVX Appendix D located (D-1 hover clean text;
  D-6/D-7 airplane mode need coordinate parsing).
- 2026-10-03: Implemented.
  - Decision: kappa is fixed at 1.15. Free fits gave kappa < 1, because the
    induced and loading terms are collinear.
  - Decision: one drag polar serves both modes, with loading referred to
    (1 + 3 lambda^2), plus an airplane increment a + b lambda^2. This
    removed a lambda trend in the efficiency error.
  - Decision: an `advance_ratio_max` = 0.60 validity bound, after the
    optimizer exploited lambda ~0.83. The bound sets cruise rotor speed;
    this is documented as a limitation.
  - Result: Halo-class 17,228 lb (actuator disk 18,506 lb).
  - 261 tests pass.

## Deferred

- BEM with twist distribution;
- compressibility drag divergence;
- swirl and tip loss as separate terms;
- conversion-mode (edgewise) rotor (Tier 20/22).
