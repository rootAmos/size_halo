# Plan 046: wing section thickness as a sizing trade (NeuralFoil)

## Goal and scope

The Halo wing is a 23 % thick section (NACA 2423, `thickness_to_chord_wing` 0.23, as on the XV-15). On a tiltrotor
the thickness is there for stiffness: torsion and beam stiffness for whirl flutter (plan 024 frequency placement,
plan 040 coupled flutter model) and box depth for the jump take-off bending (AFDD wing, plan 024/038). Thickness
also costs profile drag. Until now it was a fixed assumption, so the sizing never saw the trade.

In: an option that makes the wing thickness ratio (and camber) design variables, with NeuralFoil section polars
inside the AeroBuildup drag at every flight point (Reynolds number and Mach number from that point) and the same
thickness feeding the AFDD wing (box depth, cap area, stiffness) and the whirl frequency margins; a solve with
thickness free and solves at fixed thickness; tests; one figure. Out: a whirl-flutter constraint inside the
optimizer (plan 040 next step 2), a new airfoil shape optimization beyond thickness and camber, transonic design.

## References

- P. Sharpe, NeuralFoil (2023-2025), the AeroSandbox section-polar network, CST (Kulfan) inputs, Mach corrections
  inside `asb.KulfanAirfoil.get_aero_from_neuralfoil`.
- B. Kulfan, "Universal parametric geometry representation method", J. Aircraft 45(1), 2008 (CST class/shape
  functions: weights scale thickness and camber linearly).
- W. Johnson, NDARC Theory, NASA/TP-2009-215402, sec. 19-1.1 (AFDD tiltrotor wing; section form factors in
  thickness ratio).
- Acree, Peyran, Johnson, NASA/TP-2004-212262 (XV-15 0.23 and 0.15 t/c whirl-flutter study; plan 040).
- Abbott and von Doenhoff, Theory of Wing Sections (NACA 4-digit sections: maximum lift and drag against thickness).

## Model

**Where thickness entered before.** The wing airfoil was `asb.Airfoil("naca2423")` (coordinates). AeroBuildup turns
it into CST parameters (numerically) and calls NeuralFoil at every flight point with that point's Reynolds number,
Mach number and the plan 025 transition (10 % chord); the wing profile drag is that section drag times the
Scholz interference factor and the plan 034 excrescence factor. `thickness_to_chord_wing` (0.23) went only into the
AFDD wing (`TiltrotorWingMassModel`): box depth t c, NDARC section form factors, torque-box wall area for the torsion
frequency (GJ ~ G F_T A t^2), spar caps for the beam frequency and the jump take-off (lever arm ~ depth), and from
there the realized frequencies checked by the whirl frequency-placement margins (plan 024). The stall requirement
(120 kt) used a fixed aircraft CL_max = 1.5. So drag and weight used the same 23 % only by coincidence of two inputs.

**Section family** (`src/aircraft_closure/aerodynamics/wing_section.py`). The NACA 2423 CST fit, taken exactly as
`asb.Airfoil.get_aero_from_neuralfoil` builds it (normalized coordinates, 8 weights a side, leading-edge mode), is
split into camber and half-thickness weights; thickness ratio and camber scale them linearly (CST shape functions
are linear in the weights; the leading-edge radius scales with t^2 as in the NACA 4-digit family). The fit's chord
rotation (0.31 deg) is kept as an angle-of-attack offset, so at 23 % / 2 % the polar equals the coordinate
airfoil's to 1e-4 in CL and CD. The section is an `asb.KulfanAirfoil`, so NeuralFoil runs directly on the
(symbolic) CST weights: **NeuralFoil itself is inside the coupled Opti, no surrogate is needed for the polars**
(the network's cost does not change when its CST inputs become symbols; a single-section Opti solves in 0.04 s).
AeroBuildup is unchanged; `BuildupAerodynamics.to_asb` keeps the section's own transition wrapper
(`with_transition`) instead of rebuilding it from coordinates.

**Maximum lift.** A maximum over angle of attack is not smooth, so the section CL_max change is a fitted
surrogate: NeuralFoil (large model) section CL_max over the family at Re 8e6, Mach 0.18 (the 120 kt stall point on
the Halo chord), least-squares `ratio = 1 - 0.935 dt - 0.401 dt^2 + 2.97 dc` (dt = t - 0.23, dc = camber - 0.02;
fits the sweep to 1 %). The aircraft CL_max of the stall requirement and of the alpha-stall margins is
1.5 x ratio. Section values: 1.68 at 18 %, 1.61 at 23 %, 1.57 at 25 %.

**Sizing option** (`examples/halo_sizing.py`, all off by default):
- `wing_section_model="family"`: the CST section at `thickness_to_chord_wing`, `camber_wing` (or the design
  values); `halo_wing_section(design, assumptions)` gives the one thickness used by the airfoil and by the AFDD
  wing (`TiltrotorWingMassModel.thickness_to_chord`, unchanged interface).
- `free_thickness_to_chord_wing` (bounds 0.15-0.28) and `free_camber_wing` (0-0.04): design variables
  (`HaloDesign.thickness_to_chord_wing`, `camber_wing`).
- `stiffness_min_wing_Nm2 = (GJ, EI_beam)`: margins on the realized AFDD stiffness. The frequency-placement
  margins compare frequencies with a free wing design rotor speed and are met with the 23 % wing at 45-65 %
  margin, while the plan 040 coupled model finds that wing about 13 % short of 1.15 V_D. A stiffness floor at the
  23 % wing's values holds the coupled model's inputs (its flutter speed depends on GJ, EI and mass, not on
  thickness itself: 261 kt at 15 % and 264 kt at 23 % for the same GJ, checked with `examples/halo_whirl_flutter.py`).

`examples/halo_wing_section.py` runs the cases from the cached baseline and writes the table and figure.

## Verification

`tests/aerodynamics/test_wing_section.py`, `tests/integration/test_halo_wing_section.py` (10 tests):
- base member equals the coordinate NACA 2423 polar (CL to 2e-3, CD to 0.2 %); max thickness is exactly the
  parameter; zero camber is symmetric; section drag rises monotonically with thickness;
- NeuralFoil inside `asb.Opti`: thickness free with lift held drives to the lower bound; with thickness fixed by an
  equality the symbolic polar equals the numeric one; the same through AeroBuildup on the Halo (wing profile
  drag symbolic in thickness equals the numeric build-up; the family at 23 % matches the coordinate airfoil);
- CL_max surrogate is 1 at the base, falls with thickness, and matches NeuralFoil at 18 % and 25 % within 0.01;
- thickness reaches the weight model: `TiltrotorWingMassModel.thickness_to_chord` is the design value, a thinner
  wing has a larger torque box and a heavier wing at the same frequency targets, and the AFDD mass is a CasADi
  expression in the thickness;
- default unchanged: `wing_section_model="naca2423"`, coordinate airfoil, 0.23, no stiffness floor; a thickness
  design variable without the family raises. The family at 23 % gives the reference wing mass to 1e-9.

## Assumptions

- The family is NACA 2423 scaled in thickness (NACA 24xx-like), not a re-designed section; real thin tiltrotor
  sections (V-22 21-23 %, XV-15 64A223) would be designed for their own drag bucket.
- Fixed 10 % chord transition on both surfaces at every thickness (plan 025); NeuralFoil's thickness trend is that
  of a mostly turbulent section.
- The XV-15 wing calibration factors (AFDD torque-box efficiency, fittings) were fitted at 23 %; applied unchanged
  at other thicknesses. The NDARC form factors are regressions over typical tiltrotor sections (about 15-28 %),
  hence the 0.15-0.28 bounds.
- The aircraft CL_max scales with the section CL_max ratio (flaps, fuselage and 3-D effects unchanged).
- Machine unit counts held at the baseline's (2 motor, 3 generator units) so each case is one solve.

## Results (`examples/halo_wing_section.py`, 2026-10-10)

Each row is one coupled solve from the cached v3.6 baseline (15,179 lb), machine units fixed at 2 + 3. Margins are
the smallest over the airplane-mode points (frequency per rev against 1.087 torsion, 0.432 beam). The flutter
speed is the plan 040 coupled model (XV-15-calibrated, cantilever wing with tip nacelle) run on each sized wing
after the solve, minimum over the airplane-mode rotor speeds; required 1.15 V_D = 302 kt.

| Case | t/c | Take-off (lb) | Wing (kg) | Jump caps (kg) | Cruise L/D | Wing CD0 x1e4 | Torsion margin | Beam margin | GJ (MN m2) | EI beam (MN m2) | Flutter (kt) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fixed | 0.15 | 15,194 | 423 | 42 | 9.30 | 85 | +4 % | +62 % | 2.64 | 15.1 | 196 (wing beam) |
| fixed | 0.18 | 15,168 | 415 | 32 | 9.23 | 89 | +19 % | +61 % | 3.60 | 15.4 | 230 (wing beam) |
| fixed | 0.21 | 15,166 | 410 | 24 | 9.14 | 95 | +35 % | +63 % | 4.63 | 16.0 | 256 (wing beam) |
| **free, frequency margins** | **0.215** | 15,173 | 410 | 23 | 9.13 | 96 | +36 % | +59 % | 4.70 | 15.3 | 256 (wing beam) |
| fixed (reference; family = NACA 2423) | 0.23 | 15,179 | 408 | 18 | 9.06 | 100 | +45 % | +65 % | 5.34 | 16.4 | 264 (wing beam) |
| **free, GJ and EI floors at the 23 % wing** | **0.230** | 15,179 | 408 | 18 | 9.06 | 100 | +45 % | +65 % | 5.34 | 16.4 | 264 (wing beam) |
| fixed | 0.25 | 15,200 | 406 | 12 | 8.97 | 107 | +54 % | +67 % | 6.07 | 16.9 | 269 (wing beam) |
| fixed | 0.28 | 15,342 | 420 | 50 | 8.77 | 123 | +68 % | +74 % | 7.18 | 18.4 | 271 (gimbal) |

- The family at 23 % reproduces the reference exactly (15,179 lb, L/D 9.065): the section option changes nothing
  until thickness moves.
- **With the plan 024 frequency-placement margins only, the trade is flat:** 15,166-15,179 lb from 18 % to 23 %
  (0.1 %). No wing margin binds. The wing design rotor speed is a free variable, so as the section thins the
  optimizer lowers it and keeps the torque box at its 1 mm minimum gauge (85 kg in every case); torsion stiffness
  falls as t^2 (GJ 5.3 -> 3.6 MN m2 at 18 %) and the frequency margins absorb it (+45 % -> +19 %; at 15 % the
  torsion margin is down to +4 % at the maximum-speed rotor speed). What remains is jump take-off caps (lever arm
  ~ thickness: 18 -> 32 kg at 18 %) against section drag (wing CD0 -11 %, cruise L/D 9.06 -> 9.23). The free
  solve lands at 21.5 % (15,173 lb), 7 lb above the fixed 21 % solve; a second free solve from the 18 % result
  returns to the same point, so that 7 lb is local-optimum spread in the other variables, below what this trade
  can resolve. Thicker than 25 % loses on drag (28 %: +163 lb; the jump caps there also rise because the NDARC
  beam form factor is not monotone in t/c and the solve picks a different wing design rotor speed).
- **The coupled flutter check says thinning is the wrong direction:** the proxy-optimal 21.5 % wing flutters at
  256 kt (-8 kt from 23 %), 18 % at 230 kt, 15 % at 196 kt, against 302 kt required. Flutter speed follows GJ (at
  equal GJ, 15 % and 23 % give 261 and 264 kt), and the proxy lets GJ go.
- **With realized GJ and EI held at the 23 % wing's values** (the stiffness the coupled model already finds
  marginal), the optimum is 23.0 % with both floors binding: thinner needs a thicker box wall (GJ ~ G A t^2), which
  costs more than the drag saves; thicker adds drag for stiffness not needed. So the 23 % (XV-15) choice is the
  stiffness-driven optimum of this model, not a coincidence.
- The first stiffness-floor solve (from the baseline) stopped at a poorer local point, 15,269 lb at 22.9 % with
  the frequency caps traded for jump caps under a 31 rad/s wing design rotor speed; started from the 25 % result
  it reaches 15,179 lb at 23.0 %. The smoothed max(0, .) steps of the AFDD wing make that landscape piecewise;
  `examples/halo_wing_section.py report` keeps the lighter one.
- Thickening for flutter is weak: 25 % gives 269 kt (+5 kt) for +21 lb, 28 % 271 kt for +163 lb. Thickness is not
  the flutter lever; delta-3, nacelle CG and torsion (plan 040) are.

## Next

1. The coupled flutter constraint inside the sizing (plan 040 next step 2); until then use
   `stiffness_min_wing_Nm2` whenever thickness or the wing design rotor speed is free, because the
   frequency-placement margins do not protect the flutter speed.
2. Camber (`free_camber_wing`) is built and tested but not run in the sizing.
3. A real thin-section design (laminar-bucket 18-21 % sections) would change the drag side only modestly: the
   whole trade is under 15 lb.

## Progress log

- 2026-10-10: plan written; section family, sizing options, tests (10) built.
- 2026-10-10: ten solves (eight cases, two restarts), 1-3 min each from the cached baseline; coupled flutter check
  on every case; figure `docs/figures/neuralfoil_wing_section.png`.

## Slide summary

**Headline: the 23 % wing is the stiffness-driven optimum; the drag case for thinning is worth 13 lb and costs
flutter speed.**

- NeuralFoil section polars now run inside the coupled sizing with thickness as a design variable (no surrogate
  needed); the family at 23 % reproduces the 15,179 lb reference exactly.
- With only frequency-placement margins the optimizer thins to 21.5 % (15,173 lb; 18-23 % all within 13 lb) and
  GJ falls 12 %; the coupled flutter model then drops from 264 to 256 kt (18 %: 230 kt) against 302 kt required.
- Holding the 23 % wing's GJ and EI, thickness free returns 23.0 % at 15,179 lb with both stiffness floors binding.

Caveat: the flutter speeds come from the reduced-order cantilever model of plan 040 (XV-15-calibrated, about 10 %
conservative on the airframe), checked after the solve, not constrained in it; the section family is NACA 2423
scaled, not a designed thin tiltrotor section.
