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
