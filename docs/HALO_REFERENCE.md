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
