# Tiltrotor weight models and XV-15 mass validation

Status: COMPLETED 2026-10-02. Requested 2026-10-02: the author judged the Tier 9 result
(884 kg) too light for a Halo-class aircraft ("closer to 1x,xxx lbm") and named
the Bell XV-15 as the closest concept.

## Goal and scope

Tier 10a, the first of three plans that re-baseline the study on an XV-15-class
aircraft:

- 011 (this plan): rotorcraft-specific mass models plus a group-by-group
  validation against the published XV-15 weight statement, producing
  documented calibration factors.
- 012 (deferred): turboshaft altitude lapse, a mass-correlated turboshaft, and
  a hover-power check against the XV-15 hover ceiling.
- 013 (deferred): two-rotor, Halo-class series-hybrid sizing to XV-15-derived
  requirements using the 011 calibration.

In scope:

- AFDD weight equations, which AeroSandbox does not provide: AFDD82 rotor
  blades and hub, AFDD83 gearbox and rotor shaft, AFDD82 interconnect drive
  shaft, and the AFDD82 engine section (support, cowling, air induction).
- New physical mass items: `Nacelles`, `InterconnectShaft` and
  `FixedEquipment`. `LandingGear` gains an `is_retractable` field.
- A mechanical tiltrotor topology (turboshaft -> gearbox -> rotor, x2).
- The XV-15 reference aircraft (published geometry); a group comparison at the
  13,000 lb design gross weight; calibration factors; and a closure solve.

Out of scope: engine lapse, hover and cruise power validation, flight points
for the mechanical topology, and any sizing (plans 012 and 013).

## References

- XV-15 group weights (Nov 1974), geometry, tip speeds, load factors, engine
  ratings and drive layout: NASA TM X-62407, "NASA/Army XV-15 Tilt Rotor
  Research Aircraft Familiarization Document", 1975,
  https://ntrs.nasa.gov/citations/19750016648 (sec. 3.1.2, 3.3, 3.7, 3.8, 4.2,
  6.1, 6.2, fig. 7.1.1).
- XV-15 general characteristics: NASA SP-4517 (Maisel et al., 2000),
  appendix A, https://www.nasa.gov/wp-content/uploads/2023/04/sp-4517.pdf
- AFDD weight equations: W. Johnson, NDARC Theory, NASA/TP-2009-215402,
  ch. 19, https://ntrs.nasa.gov/citations/20100021405 (rotor sec. 19-2,
  engine section 19-6, drive system 19-7.4). Equations take lb, ft, ft/s, hp
  and rpm; wrappers convert from and to SI.
- Turboshaft mass: AeroSandbox `library.power_turboshaft.power_turboshaft`,
  a regression on historical engines.

## Published XV-15 data used

Weight statement in lb: rotor 1,070; wing 873; tail 209; fuselage 1,442;
alighting gear 508; hydraulics and flight controls 934; powerplant 1,754;
transmission/conversion 1,263; heating and air conditioning 100;
electrical 396; instrumentation 91; miscellaneous 436. Basic empty weight is
9,076 lb; design gross weight is 13,000 lb.

Geometry and ratings:

- Wing: 169 ft2, aspect ratio 6.12, span 32.17 ft between rotor centrelines,
  23 % thick (NACA 64A223).
- Horizontal tail: 50.25 ft2, aspect ratio 3.27.
- Vertical tails: two fins, 50.5 ft2 total.
- Tail length: 22.4 ft.
- Rotors: two of them, 25 ft diameter, 3 blades, 14 in chord, gimballed.
  Tip speed is 740 ft/s at 565 rpm in hover and 600 ft/s at 458 rpm in
  airplane mode.
- Limit load factor: 3.0 at design gross weight; factor of safety 1.5.
- Engines: two LTC1K-4K, 1,550 shp each for take-off (10 min), output about
  20,000 rpm. A cross-shaft interconnects the rotors.

## Assumptions (each stated in code where it is used)

1. **Coning frequency:** 1.55/rev, read from fig. 7.1.1 (lowest collective
   mode at about 880 cpm at 565 rpm). NDARC says to use the coning frequency
   for gimballed rotors. The blade weight goes as nu^2.53, so the result is
   reported at 1.0, 1.55 and 1.6.
2. **Drive system:** the rated power is total take-off power, 3,100 hp.
   Gearbox count is 3 (two proprotor gearboxes plus the interconnect set).
   f_Q = f_P = 60 %, NDARC's typical value for twin main rotors. There is one
   intermediate drive shaft per side (N_ds = 2), and the shaft length is the
   rotor spacing.
3. **Nacelles:** wetted area of 2 x 95 ft2, estimated from the planform as a
   cylinder about 9 ft long and 3.3 ft across. Air induction fraction is
   0.3 (NDARC typical). Pylon weight is zero.
4. **Engine mass:** from the AeroSandbox correlation at take-off power.
   Bracket: the T53-L-701 is 688 lb dry with its nose gearbox, which the
   LTC1K-4K removed.
5. **Fuselage:** 42.1 ft long overall (SP-4517). Equivalent diameter is
   5.5 ft, from the 5 ft cabin width plus structure.
6. **Tails and airframe groups:** twin vertical fins are modelled as one fin
   of equal area. The wing, tails, fuselage and gear use the existing Raymer
   GA correlations. The gear is retractable.
7. **Equipment not modelled:** heating, electrical, instrumentation and
   miscellaneous items (1,023 lb) are `FixedEquipment` taken from the
   statement. They are inputs, not predictions.
8. **Systems:** Raymer GA flight controls, with no avionics (the XV-15 books
   avionics under additional equipment), compared against the hydraulics and
   flight controls group.

## Interfaces

- `aircraft_closure.weights.afdd`: pure functions in SI that accept symbolic
  inputs, in the style of AeroSandbox's `library.weights`.
- `vehicle/items.py`: `Nacelles`, `InterconnectShaft`, `FixedEquipment`, and
  `LandingGear.is_retractable` (default False, so existing results don't
  change).
- `Aircraft` and `MassBreakdown` gain optional `nacelles`, `drive_shaft` and
  `equipment` fields. Each defaults to None, which contributes zero mass.
- `powertrain/topologies.py`: `build_mechanical_tiltrotor(turboshaft,
  gearbox, propulsor, count_rotors=2)`.
- `examples/xv15_reference.py`: `Xv15Reference` (published data in SI),
  `build_xv15_aircraft`, `compare_groups`, `calibration_factors` and
  `solve_xv15_closure`.

## Symbolic considerations

Power laws only, through `aerosandbox.numpy`, with no branching on values. The
closure keeps take-off mass as an Opti variable that drives every correlation.
The engine mass comes from an explicit Opti equality
`power_turboshaft(m) = P_TO`, so there is no hidden root-finding.

## Tests

- AFDD functions:
  - each matches the NDARC equation evaluated by hand in English units;
  - exponent identities (scale one input, check the ratio);
  - accepts CasADi MX inputs.
- New items: mass equals the AFDD sum; the None defaults leave Tier 4–9
  results unchanged (existing tests).
- Mechanical topology: it builds, its ports are compatible, and the instance
  counts are right.
- XV-15:
  - the group table sums to the predicted basic empty weight;
  - the calibration factors reproduce the statement exactly;
  - the calibrated closure returns 13,000 lb;
  - the uncalibrated closure is reported with a loose regression band;
  - raising the coning frequency raises rotor mass.

## Reference cases

1. Group-by-group predicted versus actual weight at 13,000 lb.
2. The uncalibrated closure: what take-off weight does the framework predict
   for the XV-15 useful load?
3. The calibrated closure, which must equal 13,000 lb.
4. Rotor-group sensitivity to the coning frequency.

## Implementation sequence

1. The AFDD library and its tests.
2. The items, the aircraft fields and the gear flag, with tests.
3. The mechanical topology.
4. The XV-15 example, its tests and the notebook.
5. Docs, then commit.

## Acceptance

All tests and notebooks pass. The Tier 10 notebook shows the group comparison
and the calibration factors, and explains which framework groups are weakest
at the 6 t scale.

## Progress and decisions

- 2026-10-02: Plan written. Calibration factors are reported, not applied by
  default; plan 013 decides which to carry.
- 2026-10-02: Implemented. Predicted vs actual (lb):
  - transmission 1,265 / 1,263;
  - powerplant 1,532 / 1,754 (regression engine 556 lb each);
  - tails 186 / 209;
  - gear 585 / 508;
  - rotor 1,545 / 1,070 at nu = 1.55 (nu = 1.35 matches);
  - wing 452 / 873;
  - fuselage 696 / 1,442;
  - hydraulics and flight controls 247 / 934.

  The uncalibrated closure gives 11,315 lb take-off and 7,391 lb empty. The
  calibrated closure gives 13,000 / 9,076 lb exactly. 201 tests and
  15 notebook checks pass, and all earlier notebooks were re-executed.
- Decision: the Raymer GA wing, fuselage and controls are the weak groups at
  this scale. The AFDD tiltrotor wing and rotor/conversion controls remain
  deferred; plan 013 carries calibration factors instead, with manned-research
  adjustments.

## Deferred

Plan 012 (engine lapse, correlated turboshaft, hover-power validation) and
plan 013 (Halo-class two-rotor sizing). The AFDD tiltrotor wing (stiffness
sizing), rotor and conversion flight controls, the hydraulics group, and the
AFDD fuselage and gear models are added only if 011 shows the Raymer groups
are the weak link.
