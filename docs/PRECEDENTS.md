# What other VTOL programmes teach the Halo decision ledger

Researched 2026-10-10. Sources marked "as reported" could not be opened in full; their figures come from the
publisher's summary and should be checked before they are quoted outside the team.

The governing trades fall into four categories: design decisions, materials and technology, requirements and risk
tolerance. The comparable programmes say where each category has hurt before.

| Programme | What they chose | What it cost or taught | Halo trade |
| --- | --- | --- | --- |
| Bell Boeing V-22 | Whole nacelle tilts (engine, gearbox, rotor) | The conversion mechanism carries the engine as well as the rotor | Tilt mechanism |
| Bell V-280 | Engines fixed; only the proprotor gearbox tilts | Bell cites independent removal of engine, shaft and gearbox, and a lighter aircraft than the V-22 ([TWZ](https://www.twz.com/21162/we-talk-v-280-valor-versus-v-22-osprey-with-bells-head-of-tiltrotor-systems)) | Tilt mechanism |
| V-22 | Three-bar linkage, linear actuator, two hydraulic motors plus an electric third; nacelle locked horizontal ([US 7,871,033](https://patents.google.com/patent/US7871033), as reported) | Triple redundancy on the conversion drive | Conversion actuation |
| Joby JAS4-2 prototype | Linkage-driven tilt on inboard propellers | NTSB: the tilt actuator linkage let blades run steeper than commanded, a contributing factor in a cascading loss ([eVTOL Insights](https://evtolinsights.com/ntsb-finally-releases-report-on-joby-aviation-crash-jas4-2-prototype/)) | Conversion actuation |
| XV-15, V-22, V-280 | Cross-shaft: one engine drives both rotors ([NASA](https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf), as reported) | All three flying turboshaft tiltrotors use a shaft | Failure architecture |
| NASA X-57 | Battery and inverter-driven electric propulsion | Interference affected onboard systems and needed filters; battery redesigned for overheating; never flew ([NASA](https://www.nasa.gov/centers-and-facilities/armstrong/x-57-project-creates-paths-toward-electric-aviation-2/)) | HV voltage, tilt-joint crossings, cells |
| V-22 | Composites up to 57 % of the airframe | Empty weight 1,520 kg over specification at the end of full-scale development; composite frames (39 parts, 258 fasteners) replaced by one-piece aluminium, 6 lb lighter each; composite share cut to 43 % ([Flight International](https://flightglobal.com/tilting-in-favour/33528.article), as reported) | Fuselage material, weight margin |
| XV-15, V-22 | 23 % thick wings | Whirl flutter beyond dive speed at a drag and speed cost; a 15 % NASA wing lost stiffness, tailoring recovered about 18 % ([NASA TP-2004-212262](https://rotorcraft.arc.nasa.gov/Publications/files/AcreePeyranJohnson_TP2004-212262.pdf), as reported) | Whirl-flutter margin, wing |
| Leonardo AW609 | Civil tiltrotor | Applied 2012; special-class powered-lift basis from Parts 23, 25 and 29, effective 2 December 2024 ([Federal Register](https://www.govinfo.gov/content/pkg/FR-2024-10-31/html/2024-25238.htm)) | Certification and assurance level |

## What it means for Halo

1. **The tilt mechanism is the trade to get right first.** Halo already follows the V-280's logic by keeping the
   heavy turbines in the fuselage. What tilts, and how its actuator fails, has caused accidents and certification
   work on every programme.
2. **Electrical cross-strapping instead of a cross-shaft is a first.** It may well be lighter, but none of the
   tiltrotor that has flown has done it. The ledger records that as certification risk, not only mass.
3. **Weight grows in development unless it is allocated.** The V-22's 1,520 kg overrun was recovered only by giving
   each team a weight allocation to hold or trade. That is the discipline this ledger encodes.
4. **Integration, not components, stops electric aircraft.** X-57 had working motors and inverters and still never
   flew. The HV installation, at the top of Halo's ranking, is the same lesson.
5. **Certification basis is a schedule driver.** AW609 took twelve years from application to criteria.
