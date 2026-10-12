# Plan 052: BEM performance curves against full-scale JVX and XV-15 data, hover and airplane mode

Status: COMPLETED 2026-10-11.

## Goal and scope

Plan 047 checked the blade-element momentum rotor (`rotor_bemt`) against JVX point by point. This plan draws the
BEM as continuous performance curves over the test data, in the form the reports plot them, and adds the XV-15 as
an independent check. The drag factor stays at the JVX fit (0.964), so the XV-15 results are predictions. The XV-15 runs with its own NACA 64-series airfoils; plan 047's single camber line is kept as a comparison (dashed in the figure).

Scope: an example script, two data files, an integration test and one new section option
(`SectionSchedule.lift_design`, with `mean_line_a` and `n_crit`). There is no change to the sizing, the JVX
calibration or plan 047's Halo blade design.

![BEM curves against JVX and XV-15](../figures/proprotor_bemt_curves.png)

## References

- Felker, Betzina and Signor, NASA/TM-86833, 1985: full-scale XV-15 rotor hover data (Appendix A) and blade
  geometry (Table I, figs. 5-8).
- Bell Helicopter, NASA CR-114363, 1971: airplane-mode efficiency of the 25-ft Model 300 proprotor in the Ames
  40x80 (figs. IV-33 to IV-35) and its blade geometry (Table III-1, fig. IV-1).
- Abbott and von Doenhoff, *Theory of Wing Sections*, 1959: NACA 64-series thickness, the a-series mean line
  (eq. 4.26) and its thin-airfoil zero-lift angle.
- Ning, Wind Energy 17, 2014: the single-residual BEM used by `rotor_bemt` (plan 047).
- NeuralFoil (Sharma, through AeroSandbox): section polars, including free transition by e^N with `n_crit`.
- JVX test data and the 0.964 drag-factor fit: plan 047.

## Model

### Data added (`data/rotors/`)

- `xv15_hover.csv`: NASA/TM-86833 (Felker, Betzina, Signor, 1985). The full-scale XV-15 metal-blade rotor
  hovering at the Ames OARF.
  - Appendix A was transcribed by hand from the scanned tables; the PDF's OCR layer is unreadable.
  - 160 points from runs 14-16 and 22-24: 136 at Mtip 0.69, 7 at 0.60 and 8 at 0.73. Runs 25-26 had the wake rake
    installed and 2-3.5 m/s winds, and are not included.
  - Columns kept: tip speed, Mtip, collective, wind, density, C_T/sigma and the wind-corrected FM.
- `xv15_airplane.csv`: NASA CR-114363 (Bell, 1971). The 25-ft Bell Model 300 proprotor (the XV-15 rotor) in
  airplane mode in the Ames 40x80.
  - 46 points digitized by eye from figures IV-33 to IV-35, at 120, 160 and 185 kt and 400-740 ft/s tip speed.
  - The figures give efficiency against the propeller power coefficient C_P = P / (rho n^3 D^5), with J = V/(nD).
  - Reading accuracy is about +/-0.001 in C_P and +/-0.5 points in efficiency. These are the only public XV-15
    airplane-mode performance data found.

### XV-15 blade (`examples/proprotor_bemt_curves.py`, `xv15_rotor`)

| Parameter | Value | Source |
|---|---|---|
| Radius, blades | 150 in, 3 | TM-86833 Table I |
| Chord | 14 in, root cuff 16.6 in at 0.091 R tapering to 14 in at 0.25 R | CR-114363 Table III-1 (cuff 17 in at 0.0875 R), TM-86833 fig. 7 |
| Thrust-weighted solidity | 0.0890 (test: 0.0891) | model |
| Twist | Digitized from TM-86833 fig. 5: +29.7 deg at 0.10 R, 0 at 0.77 R, -5.1 deg at 0.95 R; tip extrapolated to -6.3 deg (about 36 deg over the blade, nonlinear) | fig. 5 |
| t/c | 0.32 at 0.09 R, 0.28 at 0.17 R, 0.185 at 0.51 R, 0.125 at 0.80 R, 0.082 at the tip | fig. 6 |
| Root cutout | 0.09 R | first airfoil station, fig. 8 |
| Sections (default) | The blade's own NACA 64-935, 64-528, 64-118, 64-(1.5)12 and 64-208 at 0.09, 0.17, 0.51, 0.80 and 1.0 R: 64-series thickness at the local t/c on an a = 0.3 mean line at the local design lift coefficient (0.9, 0.5, 0.1, 0.15, 0.2), interpolated in radius | TM-86833 fig. 8, CR-114363 Table III-1 |
| Sections (comparison) | plan 047 family: one NACA 64A210 camber line with the thickness scaled (`airfoils="plan-047"`) | plan 047 |

**Section model** (`SectionSchedule.lift_design`, new):

- The a-series mean line is Abbott and von Doenhoff's closed form.
- The stall-delay model's zero-lift angle comes per station from thin-airfoil theory for that mean line, -5.28 deg
  per unit design lift. NeuralFoil's own zero-lift angle is not used: on the thick (t/c > 0.25), cambered root
  sections its lift slope is 0.03-0.06 per degree instead of about 0.11, which is outside its training range.
  Inboard of about 0.15 R the stall-delay blend replaces most of that 2-D lift anyway.
- Check: the built 64-208 has NeuralFoil's zero-lift angle -1.11 deg against -1.06 deg from theory.
- The default schedule (no `lift_design`) is unchanged, so JVX and the Halo blade design are unaffected.

### Method

- **Sweep.** Each test condition (fixed tip speed, inflow and air) is swept over the thrust range of its data, on
  9-11 thrust points.
- **Continuation.** The sweep starts at 60 % of the range with the stepped solve and continues up, then down, from
  the neighbouring solution: a fixed list of starting points, not an iteration.
- **Rejection.** Solutions above the momentum limit, and failed solves, are dropped.
- **Error.** Each measured point is compared with the curve interpolated at its own C_T/sigma (hover and JVX
  airplane mode) or C_P (XV-15 airplane mode). No extra solves are needed.
- **Run time.** The four groups are cached separately to `output/proprotor_bemt_curves_<group>.pkl` and run in
  parallel in about 6 minutes, single-threaded:

      python examples/proprotor_bemt_curves.py <group>... --sweep-only
      python examples/proprotor_bemt_curves.py --cached

## Results

| Case | Points | RMS | Bias (BEM - test) |
|---|---|---|---|
| JVX hover, all | 71 | 0.036 | -0.004 |
| JVX hover, C_T/sigma <= 0.13 | 61 | 0.023 | -0.003 |
| JVX airplane mode, 5 advance ratios | 42 | 0.022 | +0.015 |
| XV-15 hover, 64-series airfoils, C_T/sigma <= 0.13 | 79 | 0.060 | -0.007 |
| XV-15 hover, plan 047 camber line, C_T/sigma <= 0.13 | 79 | 0.030 | +0.012 |
| **XV-15 airplane mode, 64-series airfoils** | 46 | **0.072** | **-0.041** |
| XV-15 airplane mode, plan 047 camber line | 45 | 0.086 | -0.067 |

- **JVX.** The curves reproduce plan 047's point-by-point numbers (hover 0.018-0.040, airplane mode 0.025), as
  they should.
- **XV-15 hover is predicted well up to C_T/sigma 0.12 with either airfoil set.** The curve follows the data from
  FM 0.2 to 0.8. The 64-series RMS below 0.13 is higher only because its drop starts a little earlier, between the
  0.12 and 0.136 grid points.
- **The real airfoils improve airplane mode.** The bias goes from -0.067 to -0.041 and every condition moves
  toward the data.
- **Above C_T/sigma about 0.12 to 0.14 the BEM hover curves drop sharply, and the cause is outboard shock-buffet,
  not inboard stall.**
  - The drop moves with tip Mach. At Mtip 0.60 the XV-15 curve stays attached to C_T/sigma 0.165 and follows the
    data; at 0.69 it drops near 0.13, at 0.73 near 0.12. JVX at Mtip 0.68 and 0.73 behaves the same way.
  - Spanwise check (64-series, Mtip 0.69, C_T/sigma 0.125): the inboard sections are attached with stall delay.
    The 0.76 R station sits at 6 deg.
  - NeuralFoil (AeroSandbox's Mach model) at that station, M 0.52: lift falls from 0.84 at 6 deg to 0.60 at 8 deg.
    At M 0.46 (Mtip 0.60) it rises to 1.09 at 9 deg. At 0.90 R and M 0.62 it falls above 4 deg.
  - So the cliff is AeroSandbox's shock-buffet onset on the outboard 12 % and 8 % sections. A real
    64-(1.5)12 or 64-208 at Mach 0.5 should not lose lift that abruptly at 7 deg.
  - Camber does not fix it. Branch selection does not either: at each hover point every start is tried and the
    highest-FM solution kept.
  - **This bears directly on plan 047's Halo hover finding.** Its outboard stall margin was defined from the same
    buffet onset (about 6 deg at 0.76 R, 2.5 deg at the tip), so the 0.060-solidity blade's "no attached hover
    solution" result is a product of this section model and not yet a physical finding.
- **XV-15 airplane mode is still about 0.04 low** against the 1971 data.
  - Bell's own calculated curves in the same figures are also below the data.
  - The data scatter by about +/-0.04, and several points read 0.93-0.95, close to the momentum limit.
  - The JVX airplane-mode data (2016, better instrumented) agree with the BEM within 0.022, so the 1971 XV-15
    numbers are the weaker evidence. A hub/spinner force convention in "propulsive force" and the reading error
    may account for the rest.

## Airplane-mode offset: model settings and laminar flow

The XV-15 airplane-mode curves are consistently low. Checked at three test thrusts, 160 kt and 600 ft/s (J 1.41):

| Setting | Effect on efficiency | Verdict |
|---|---|---|
| Stations 16 / 24 / 48 | identical to 3 decimals | converged |
| Hub loss off | none | ruled out |
| Stall delay off, tip loss off | +0.01 to +0.02 each | small |
| Twist (1971 blade, CR-114363 fig. IV-1) | matches the 1985 blade within about 1 deg | ruled out |
| Tunnel wall (Glauert) | about 0.15 % on speed | ruled out |
| Zero profile drag | 0.938 at the 0.940 test point | the test point equals a drag-free rotor |

Laminar flow, with free transition from NeuralFoil (pressure gradient per section) and the new
`SectionSchedule.n_crit` standing in for inflow turbulence:

| Transition | XV-15 hover FM (C_T/sigma 0.06 / 0.10) | XV-15 airplane (3 points) | JVX hover FM | JVX airplane, lambda 0.52 |
|---|---|---|---|---|
| Test | 0.665 / 0.775 | 0.731 / 0.874 / 0.940 | 0.675 / 0.794 | 0.629 / 0.864 / 0.866 |
| Forced 5 % (default) | 0.686 / 0.804 | 0.699 / 0.827 / 0.867 | 0.665 / 0.802 | 0.679 / 0.872 / 0.871 |
| Free, n_crit 9 | 0.764 / 0.845 | 0.778 / 0.875 / 0.900 | 0.719 / 0.832 | 0.770 / 0.911 / 0.911 |
| Free, n_crit 1 | 0.708 / 0.812 | 0.725 / 0.844 / 0.881 | 0.680 / 0.809 | 0.709 / 0.885 / 0.884 |

- **Hover is turbulent on both rotors.** Even n_crit 1 is 0.03-0.04 high in FM.
- **XV-15 airplane mode fits laminar flow.** This is the expected physics: low section lift, clean inflow and
  lower Reynolds number than hover.
- **JVX airplane mode does not.** Free transition makes it 0.04-0.09 high, while forced transition matches.
- **So the effect is blade-specific, not a mode rule.** A plausible cause is 1971 smooth metal blades against JVX
  blades with leading-edge protection or roughness, but that is unconfirmed.
- **Even laminar does not fix the shape.** The 1971 points rise more steeply with C_P than any model curve, which
  points to scatter or a data-definition issue (propulsive force, torque station).

**Full laminar sweep** (`xv15_airplane_laminar`, dotted in the figure): RMS 0.094 and bias +0.037 over 46
points, against 0.072 and -0.041 turbulent. The laminar curves land on the high-C_P plateau (about 0.90 at 120 and
160 kt) but overshoot on the steep low-C_P side. The test points lie between the two settings.

**Decision:** keep forced 5 % transition, the JVX calibration, as the default. Laminar flow in cruise is the
optimistic bound: about +0.04-0.05 in cruise efficiency for a blade that holds it.

## Verification

`tests/integration/test_xv15_bemt.py` (4 tests, about 25 s):

- the data files load;
- the geometry reproduces the 0.0891 solidity;
- hover FM at C_T/sigma 0.10, Mtip 0.69, is within 0.04 of the test mean;
- an airplane-mode point at 160 kt and 600 ft/s trims above 0.75 efficiency and below the momentum limit.

`tests/integration/test_jvx_bemt.py` still passes.

## Assumptions

Collected from the sections above:

- Calibration: one profile-drag factor, 0.964, fitted on JVX in plan 047; every XV-15 result is a prediction.
- Sections:
  - 64-series thickness on an a = 0.3 mean line at the local design lift, interpolated in radius.
  - The stall-delay zero-lift angle comes from thin-airfoil theory, not NeuralFoil, because the thick root
    sections are outside NeuralFoil's training range.
  - Compressibility is AeroSandbox's Mach model, including its shock-buffet onset (the cause of the hover cliff).
- Transition: forced at 5 % chord by default. The laminar bound is free transition from NeuralFoil at
  `n_crit` 9, not a forced x_tr = 1.
- Aerodynamics: steady, axisymmetric inflow; Prandtl tip and hub loss; Snel stall delay. No wake contraction model
  beyond momentum theory, no tunnel-wall correction (it is about 0.15 % on speed).
- Data:
  - XV-15 hover is the wind-corrected FM from TM-86833 Appendix A, transcribed by hand.
  - XV-15 airplane mode is digitized by eye, about +/-0.001 in C_P and +/-0.5 points in efficiency.
  - "Propulsive force" in CR-114363 is taken as rotor thrust; the hub and spinner convention is unknown.
- Sweeps: continuation from a fixed list of starting points (not an iteration). Solutions above the momentum limit
  and failed solves are dropped. In hover the highest-FM solution is kept.
- Option: `lift_design` defaults to off and `n_crit` to NeuralFoil's 9, so JVX, the Halo blade and the sizing are
  unchanged.

## Progress log

- 2026-10-11 12:21: JVX and XV-15 curves, data files, test (550156f).
- 2026-10-11 12:59: XV-15 on its own 64-series sections; hover cliff traced to outboard buffet (af67354).
- 2026-10-11 14:29: airplane-mode offset checks, `n_crit` setting, laminar sensitivity (059ce61).
- 2026-10-11 14:44: laminar bound curves in the figure (9e3b7c4).
- 2026-10-11 16:43: merged to main in PR #32.
- 2026-10-11: audited against AGENTS.md. This document was written alongside the code rather than first; Goal and
  scope, References, Assumptions and this log were added afterwards, with no change to the results.

## Next

1. **High-Mach section polars.**
   - Replace AeroSandbox's shock-buffet onset on the outboard sections with transonic data or a calibrated model,
     e.g. NACA 64-series high-speed data (NACA TN 1945 family), MSES or XFOIL with a compressibility correction.
   - Check that the hover curves then stay attached to C_T/sigma 0.18 at Mtip 0.69-0.73 on both rotors.
   - Then re-run plan 047's Halo blade design before the reference solidity floor is changed.
2. **JVX sections.** Apply the same per-station camber treatment to the JVX XN-series blade if its design lift
   coefficients can be found.
3. **Better XV-15 airplane-mode data.** If a tabulated source turns up, replace the digitized figures:
   - the 1978 complete-aircraft 40x80 test;
   - the TTR / NFAC entries.
