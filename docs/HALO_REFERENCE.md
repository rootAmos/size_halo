# Halo reference and assumptions

Archer's July 2026 announcement describes Halo as the commercial variant of
an autonomous tiltrotor platform with series hybrid-electric propulsion.
Source (accessed 2026-09-30):
https://www.investors.archer.com/news/news-details/2026/Introducing-Halo-Archers-Commercial-Variant-of-Dual-Use-Autonomous-VTOL-Aircraft-Platform-Developed-With-Anduril/default.aspx

This project is an independent conceptual-design framework inspired by that
architecture. It is not an Archer performance reconstruction. All numeric
component defaults and example values are illustrative engineering inputs.
No advertised range or payload is used to calibrate the present models.

Before aircraft closure, choose payload, rotor number/placement/diameter,
wing geometry, conventional empennage assumptions, hover duration and altitude,
cruise speed/range, climb/descent schedule, loiter/reserves, failure cases,
generator sizing, battery reserve/SOC, installation mass and thermal margins.
Maintain these as configurable research assumptions with provenance.

Native physics review: AeroSandbox 4.2.8 propulsion_propeller provides
propeller_shaft_power_from_thrust. Its airspeed-divided formulation is singular
at V=0; this project's equivalent momentum form covers hover and returns an
explicit power residual for inverse operation. Native motor and turboshaft
helpers were inspected; these dataclasses add the requested stable physical
interfaces, explicit ratings, replaceable losses and transparent assumptions.
https://aerosandbox.readthedocs.io/en/master/autoapi/aerosandbox/library/propulsion_propeller/index.html

## XV-15 class reference (from 2026-10-02)

The Tier 9 result (884 kg) was judged too light for a Halo-class aircraft,
which is a Group 5 platform. Public coverage gives no Halo weight, payload or
range. It does say Halo uses two large tiltrotors. The Bell XV-15 is the
closest published concept: two tiltrotors, cross-shafted turboshafts and a
13,000 lb design gross weight. It is used as follows:

1. **Mass-model validation and calibration** (Tier 10a). The XV-15 group
   weight statement in NASA TM X-62407 (1975), sec. 3.1.2,
   https://ntrs.nasa.gov/citations/19750016648
2. **Source of a Halo-class requirement set** (Tier 10c). Data from SP-4517,
   appendix A, https://www.nasa.gov/wp-content/uploads/2023/04/sp-4517.pdf
   - 13,000 lb design gross weight;
   - 2 x 1,550 shp take-off power;
   - 25 ft rotors;
   - 300 kt maximum speed;
   - 29,000 ft service ceiling;
   - 8,650 ft OGE hover ceiling;
   - 445 nm range.

The XV-15 is a manned research aircraft with ejection seats, crashworthy fuel
cells, redundant flight controls and an oxygen system. TM X-62407 warns that
these weigh more than the concept needs, so its calibration factors are applied
to an unmanned aircraft with stated adjustments, not wholesale.

## Drawn Halo layout and plan 037 reference (2026-10-04)

The Halo is uncrewed and unpressurized. Plan 037 (numbered 032 on its branch) uses these user statements and the
plan 031 drawing, made from public stills:

- turbines in the fuselage;
- whole tip nacelles that tilt;
- a V-tail (drawn only);
- a boxy fuselage, deeper than wide.

The sized aircraft is 11 m long, 1.68 m wide and 2.0 m deep (super-ellipse 3.2). Its fuselage mass is raw Raymer GA
x 1.70, anchored to a layout estimate of an uncrewed cargo fuselage. It weighs 12,821 lb at take-off with 900 kg
payload at 210 kt. These remain illustrative engineering inputs, not Archer data.

Plan 038 (numbered 035 on its branch) corrects the wing (spar caps at the real box depth, a 1 mm minimum gauge,
pylon inertia from the tip components) and places the turbogenerators by the layout: 13,038 lb.

## Combined reference (2026-10-05)

The layout line (plans 031, 037, 038) was merged with the main line (real machine units, redundancy, drag
corrections and tail-load trim drag, plans 033-036), taking the highest-fidelity version of each model. With every
model on, the reference weighs **6,885 kg (15,179 lb)** at 900 kg payload and 210 kt. The named sets
`assumptions_plan037` and `assumptions_plan038` reproduce the two layout-line references above.
