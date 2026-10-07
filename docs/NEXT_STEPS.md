# Next steps

What I would do next with this framework, in order of value to a tiltrotor programme. Each item says why it
matters, how it fits the framework (one CasADi problem, components that return equations, the simplest model always
kept), and roughly what it takes.

## 1. Whirl flutter as a stability constraint in the sizing

**Why.** Whirl flutter is the constraint that sizes a tiltrotor wing. Today it enters as frequency placement:
wing torsion and beam frequencies above a set number per rev at every airplane-mode point (the NDARC practice).
That is a proxy. The optimizer cannot trade wing thickness, spar caps, nacelle station, pylon inertia or the gimbal
spring directly against flutter speed.

**How.** A coupled rotor-pylon-wing model, written directly in CasADi so it stays inside the one optimization:

- **States (about 20 per side, symmetric and antisymmetric):** gimbal pitch and yaw (tip-path-plane tilt), pylon
  pitch and yaw on a conversion-actuator spring, and three or four wing modes (beam, chord, torsion).
- **Wing stiffness and mass** from the AFDD wing model already in the sizing (EI, GJ, box mass), as assumed-mode
  beam matrices with the nacelle mass and pitch inertia at the tip, so they stay symbolic in the design variables.
- **Rotor forces from hub motion** by quasi-steady blade-element derivatives at high inflow (Johnson's tiltrotor
  formulation, NASA TN D-7677), plus gyroscopic coupling; quasi-steady strip theory for the wing.
- **Constraint:** at 1.2 × dive speed for each rotor speed, the least-damped eigenvalue keeps a damping margin. The
  critical eigenpair (λ, v) is carried as optimizer variables with A v = λ v and a normalization, which is smooth.
- **Kept:** the frequency-placement margins remain as the simplest option.
- **Validation:** the rigid-wing limit (classical propeller-on-pylon whirl), the backward-whirl sign, published
  XV-15 and JVX whirl-flutter boundaries, and trends (a stiffer wing and a forward nacelle CG raise the flutter
  speed).
- **Outside the loop:** wing mode shapes from the CalculiX wing-box model as a check, the way the strain check runs
  today.

**Tools considered.** FENIAX (Imperial, JAX) is differentiable but has no rotor or flutter-eigenvalue capability and
would have to be wrapped as a CasADi callback. SHARPy (Imperial, UVLM plus nonlinear beams) can linearize and
check stability and has wind-turbine rotors, but is not differentiable and has no ready proprotor whirl model.
Either is useful as an independent check of the optimum, not inside the loop.

**Effort:** about 3-4 days for the beam-mode model, tests and the constraint; one more day for the finite-element
mode check.

## 2. Structures: close the loop on the wing

**Why.** The finite-element check of the wing box puts the jump take-off strain at 1.10 of the allowable after the
spar-cap correction, and the torsion mode is not cleanly identified.

**How.** Gauges sized in the finite-element model from every mission and failure load case, returned to the weight
model as calibration factors (outside the loop, like the existing checks); then fuselage and tail checks.
**Effort:** 2-3 days.

## 3. Calibrate drag and rotor efficiency against flight data

**Why.** Drag drives payload headroom at 210 kt more than any other input. It is calibrated to NASA NDARC's XV-15
components (an excrescence factor of 1.27 on AeroBuildup), not to flight. Airplane-mode rotor efficiency is
calibrated to the JVX test only.

**How.** XV-15 level-flight power required against airspeed (or a flight-test drag area) gives one factor on
parasite drag and one on airplane-mode rotor profile power, fitted in one small optimization like the existing
calibrations. **Effort:** a day, once the data source is in hand.

## 4. Weights: a second calibration aircraft

**Why.** Every group factor rests on the XV-15 weight statement; the fuselage (about 2×) and flight controls
(about 4×) need large factors, and the calibration factors are the largest sensitivities (about ±540 lb each for
±15 %). The V-22 and D266 wing checks exist but not full weight statements.

**How.** A second complete weight statement (V-22, AW609 or a published NDARC case), with the factors fitted to both.

## 5. Electrical layer on by default

**Why.** Inverters, cables, protection and partial-discharge insulation are modelled (about 330 kg and 3 % losses)
but off by default, because with every option on the problem converges only through the multistart strategy.

**How.** Better problem scaling and a continuation start (solve without the layer, then switch it on from that
point), so the full model converges from one start. **Effort:** 1-2 days.

## 6. Asymmetric and double failures

**Why.** Failures are single and applied to both rotors at once, so roll trim is not modelled. Double failures (a
battery string or a lane plus an engine) did not converge and are not proven infeasible.

**How.** One-sided degraded states with roll trim, and the double-failure cases solved from the single-failure
optimum; then the trade between an interconnect shaft and electrical cross-strapping. **Effort:** 2-3 days.

## 7. Uncertainty on the answer

**Why.** The take-off weight is a single number, but the calibration factors behind it are uncertain (the weight
factors alone move it about ±540 lb each for ±15 %).

**How.** Propagate the calibration factors (weights, drag, rotor efficiency) through the sizing to a take-off-weight
band, using the sensitivities CasADi already provides at the optimum, then a sampled check. **Effort:** about 1 day.

## 8. Conversion controls: from corridor to control-law design

**Why.** The corridor is computed from trim, but three things are missing before it supports control design:

- **Rotor in-plane (H) force in edgewise flow.** It would help cancel forward thrust, so the low-speed side at
  45-60° nacelle is conservative (89 kt at 60° against about 40-60 kt on the XV-15).
- **An airplane-mode rotor-speed schedule.** Hover tip speed is used throughout, which is why the 0° top speed
  (219 kt) falls below the 30° one (231 kt).
- **Cyclic in the trajectory model.** Without it the trajectory model is stricter than the trim: no level,
  constant-acceleration conversion fits the computed corridor, and the optimized conversion descends slightly
  through 15-90 kt.

**Then:** linearized pitch models at each trim point from CasADi Jacobians of the same equations (the plant for
control-law design and handling-qualities checks), lateral-directional trim, and later 6-DOF dynamics with control
allocation. **Effort:** H-force, rotor-speed schedule and cyclic about 2 days; linearized models about 1 day.

## 9. Configuration and layout

- **V-tail sizing** in place of the conventional tail (the V-tail is drawn only).
- **Symbolic layout (Tier 23):** packaging and clearance constraints from the drawn layout inside the sizing.
- **Machine and architecture enumeration:** other catalogue machines (magniX, Siemens, EMRAX, mixed units) and
  architecture options run as discrete cases around the continuous sizing.
- **Payload-range and endurance:** sweep payload at fixed design to draw the payload-range curve.

## 10. Engineering hygiene

- **Continuous integration:** the full test suite (about 560 tests) exceeds the 30-minute CI limit; split the slow
  integration tests into their own scheduled job.
- **Solver robustness:** each coupled problem starts from an explicit ordered list of starting points; some
  feature combinations need several failed starts first.
