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

(Filled in as built; see Progress log.)

## Verification

(Filled in as built.)

## Assumptions

(Filled in as built.)

## Progress log

- 2026-10-10: plan written.
