"""Halo trade register: the governing trades from concept to first article, and the decisions already taken.

Only decisions that shape the aircraft are listed, on two axes: the programme phase in which they lock (conceptual,
preliminary and detailed design, test and validation, after entry into service) and the decision type
(`decision_types`: requirement, architecture, technology and material, margin policy, increase maturity, verification). The
ledger's uncertain quantities are increase-maturity decisions; its binding limits are requirement, architecture, technology or
margin decisions.

Each open trade names its owner, the gate where it locks, the ledger items it moves and, where the sizing can already
price its options, the model hook. Trades dropped from an earlier register are marked "retired" in the ledger (kept
for the record, never ranked). Decided entries record the decision and its date.
"""
from aircraft_closure.ledger.model import Trade

# Decision types: the second axis of the ledger, beside the programme phase.
REQUIREMENT, ARCHITECTURE, TECHNOLOGY, MARGIN, MATURITY, VERIFICATION = (
    "Requirement", "Architecture", "Technology and material", "Margin policy", "Increase maturity", "Verification")
decision_types = {
    REQUIREMENT: "What it must do; negotiated with the customer",
    ARCHITECTURE: "How the parts fit together and how they fail",
    TECHNOLOGY: "What it is made of and with",
    MARGIN: "How much margin, redundancy and confidence the programme buys",
    MATURITY: "How far an estimate is firmed up (analysis, test, calibration, weighing) before committing",
    VERIFICATION: "How it will be proven: certification basis, means of compliance, test approach",
}

# Programme phases, each with the date its decisions lock (placeholder programme calendar).
CONCEPTUAL, PRELIMINARY, DETAILED, TEST, IN_SERVICE = (
    "Conceptual design", "Preliminary design", "Detailed design", "Test and validation", "After entry into service")
phases = {CONCEPTUAL: "2026-12-15", PRELIMINARY: "2027-06-30", DETAILED: "2028-03-31", TEST: "2029-09-30",
          IN_SERVICE: "2031-06-30"}
# Gate names used before the ledger was organised by phase.
renamed_gates = {"Concept freeze": CONCEPTUAL, "Preliminary design review": PRELIMINARY,
                 "Critical design review": DETAILED, "First article": TEST}

# (key, label, category, owner, gate, affects, model hook, note)
open_trades = (
    # ---- Architecture ----------------------------------------------------------------------------------------------
    ("trade.tilt_load_path", "Tilt mechanism: spindle station, bearings and fitting concept", ARCHITECTURE, "Structures",
     PRELIMINARY, ["gap.tilt_fittings", "mass.wing", "mass.nacelles", "gap.wing_strength"], "",
     "Sets whirl flutter, jump take-off root strain and nacelle CG against the elastic axis"),
    ("trade.conversion_actuator", "Conversion actuation: electromechanical or hydraulic, redundancy and stiffness",
     ARCHITECTURE, "Flight controls and systems", PRELIMINARY, ["gap.conversion_actuators"], "",
     "The actuator is the pylon pitch spring for whirl flutter; a jam is a critical failure"),
    ("trade.tilt_joint_crossings", "Power across the tilt joint: inverters in nacelle or fuselage, AC or DC, flex "
     "loops or slip rings, coolant swivel or nacelle-local cooling", ARCHITECTURE, "Electrical",
     PRELIMINARY, ["gap.tilt_joint_crossings", "gap.coolant_loop", "mass.motors"], "", ""),
    ("trade.failure_architecture", "Failure architecture: interconnect shaft or electrical cross-strapping", ARCHITECTURE,
     "Electrical", CONCEPTUAL, ["mass.motors", "mass.generators", "mass.battery"], "",
     "Needs asymmetric failure cases with roll trim"),
    ("trade.hv_voltage", "HV bus voltage and regulation (floating or DC-DC; 540 to 1,000 V)", ARCHITECTURE, "Electrical",
     CONCEPTUAL, ["gap.hv_installation", "mass.battery", "mass.motors"],
     "assumptions_for_bus_voltage at 540, 756, 800, 1,000 V, with and without DC-DC",
     "Voltage and cell series count are one decision; the engine-out battery voltage limit binds"),
    ("trade.thermal_architecture", "Thermal architecture: ram-air exchanger, liquid loop or nacelle-local", ARCHITECTURE,
     "Thermal", CONCEPTUAL, ["mass.heat_exchanger", "gap.coolant_loop"], "",
     "Hot-day hover heat rejection binds"),
    ("trade.rotor_span", "Rotor diameter against wing span", ARCHITECTURE, "Configuration", CONCEPTUAL,
     ["Rotor radius capped by the span (rotor diameter against span)", "mass.rotors", "mass.wing"],
     "clearance_rotor_fuselage_m and the span cap", "The span cap is the most expensive binding limit"),
    ("trade.rotor_hub", "Rotor hub: gimballed or hingeless", ARCHITECTURE, "Rotor and drive", PRELIMINARY,
     ["mass.rotors", "gap.tilt_fittings"], "", "Hub moments size the tilt fittings and the wing tip"),
    # ---- Technology and material -----------------------------------------------------------------------------------
    ("trade.wing_material", "Wing primary structure: material and cover concept", TECHNOLOGY, "Structures",
     CONCEPTUAL, ["mass.wing", "gap.wing_strength"], "wing_material graphite_epoxy and aluminium",
     "Covers (unstiffened, stiffened, sandwich) still to price: the FE shows a negative margin on 1 mm covers"),
    ("trade.fuselage_material", "Fuselage structure: material and construction", TECHNOLOGY, "Structures",
     PRELIMINARY, ["mass.fuselage"], "", "Composite monocoque or metallic semi-monocoque"),
    ("trade.fitting_material", "Tilt spindle and fittings: titanium, steel or composite", TECHNOLOGY, "Structures",
     DETAILED, ["gap.tilt_fittings"], "", "Fatigue under conversion cycles"),
    ("trade.rotor_construction", "Rotor blades and hub: material and construction", TECHNOLOGY, "Rotor and drive",
     PRELIMINARY, ["mass.rotors"], "", "Blade stiffness sets the rotor frequencies"),
    ("trade.cell_selection", "Electric powertrain technology: cells, pack and machines (50G, Evolito, Helix today)",
     TECHNOLOGY, "Electrical", CONCEPTUAL,
     ["mass.battery", "mass.motors", "mass.generators", "input.cell_to_pack", "input.capacity_end_of_life"],
     "machine database units (count_units_motor, count_units_generator)",
     "No further power scaling of the 50G (decided 2026-10-03)"),
    # ---- Requirements ----------------------------------------------------------------------------------------------
    ("trade.payload_range", "Mission: payload, range, cruise speed and altitude (900 kg, 445 nm, 210 kt, 10,000 ft)",
     REQUIREMENT, "Chief engineer", CONCEPTUAL, ["mass.battery"],
     "HaloRequirements mass_payload_kg, range_m, velocity_max_m_s, altitude_cruise_m",
     "Payload-range diagram not drawn yet"),
    ("trade.hover_cases", "Hover cases: hot and high (4,000 ft, 95 F) and engine-out duration", REQUIREMENT,
     "Chief engineer", CONCEPTUAL, ["mass.heat_exchanger", "mass.battery", "mass.generators"],
     "HaloRequirements altitude_hover_hot_m, temperature_hover_hot_K, duration_engine_out_hover_s",
     "Both bind at the optimum"),
    # ---- Margin policy and verification ----------------------------------------------------------------------------
    ("trade.redundancy_level", "Redundancy: lanes, buses and battery strings (2/2/2 today)", MARGIN, "Electrical",
     CONCEPTUAL, ["mass.motors", "mass.protection_and_bus_tie", "mass.battery"],
     "redundancy on/off, count_strings_battery 1 to 3", "Set by the safety assessment's failure rates"),
    ("trade.battery_reserve", "Battery reserve and end-of-life sizing (30 % reserve, 80 % capacity)", MARGIN,
     "Chief engineer", CONCEPTUAL, ["mass.battery", "input.capacity_end_of_life"],
     "free_soc_reserve with bounds_soc_reserve; factor_capacity_ageing_battery", ""),
    ("trade.weight_margin", "Weight margin policy: confidence level and management reserve", MARGIN,
     "Chief engineer", CONCEPTUAL, [], "", "99 % today; sets the NTE take-off weight"),
    ("trade.load_factors", "Design loads: ultimate and jump take-off load factors (4.5, 2.0)", MARGIN, "Structures",
     CONCEPTUAL, ["mass.wing", "gap.wing_strength"], "load_factor_ultimate, load_factor_jump", ""),
    ("trade.whirl_margin", "Whirl-flutter margin: frequency placement or flutter-speed margin", MARGIN, "Structures",
     PRELIMINARY, ["mass.wing"], "frequency_torsion_wing_per_rev, frequency_beam_wing_per_rev",
     "Frequency placement is a proxy for the flutter boundary"),
    ("trade.assurance_level", "Certification basis and design assurance levels", VERIFICATION, "Chief engineer",
     CONCEPTUAL, ["mass.systems", "mass.fixed_equipment"], "", "Drives redundancy and avionics weight"),
    # ---- Detailed design, test and validation, in service ---------------------------------------------------------
    ("trade.structure_joining", "Primary structure manufacturing and joining: integrated composite, bonded or "
     "fastened, machined metal", TECHNOLOGY, "Structures", DETAILED, ["mass.fuselage", "mass.wing"], "",
     "Part and fastener count is weight"),
    ("trade.hv_installation_standard", "HV installation standard: insulation, connectors, shielding and segregation",
     ARCHITECTURE, "Electrical", DETAILED, ["gap.hv_installation"], "", "Partial discharge at altitude and EMI"),
    ("trade.integration_rig", "Integration rig before first flight: HV powertrain and conversion on an iron bird",
     VERIFICATION, "Chief engineer", TEST, ["gap.hv_installation", "gap.conversion_actuators"], "",
     "Buys schedule certainty at the cost of a rig"),
    ("trade.envelope_expansion", "Envelope expansion: build-up flight test or model-based clearance of the "
     "conversion corridor", VERIFICATION, "Flight test", TEST, [], "", "The computed corridor is edgewise-limited above 45 deg"),
    ("trade.means_of_compliance", "Means of compliance: test or analysis credit for whirl flutter and failure cases",
     VERIFICATION, "Chief engineer", TEST, ["mass.wing"], "", ""),
    ("trade.maintenance_concept", "Maintenance concept: on-condition or scheduled for tilt mechanism, gearboxes and "
     "packs", ARCHITECTURE, "Chief engineer", IN_SERVICE, ["gap.tilt_fittings", "mass.rotor_gearboxes", "mass.battery"],
     "", "Access and removal paths are set in design"),
    ("trade.battery_life", "Battery replacement policy: pack life against end-of-life sizing", MARGIN, "Electrical",
     IN_SERVICE, ["mass.battery", "input.capacity_end_of_life"], "factor_capacity_ageing_battery", ""),
    ("trade.growth_reserve", "In-service growth reserve: weight and power held back for modifications", MARGIN,
     "Chief engineer", IN_SERVICE, [], "", "Fixed engines leave no power growth path"),
)

# (key, label, category, owner, decision, decided date)
decided_trades = (
    ("decided.engines", "Turboshafts: off-the-shelf or sized", ARCHITECTURE, "Chief engineer",
     "Fixed 2 x 1,120 hp off-the-shelf engines; never a design variable", "2026-10-03"),
    ("decided.max_speed", "Maximum speed: 250 or 210 kt", REQUIREMENT, "Chief engineer",
     "210 kt; 250 kt is infeasible with the fixed engines", "2026-10-03"),
    ("decided.payload", "Reference payload", REQUIREMENT, "Chief engineer", "900 kg", "2026-10-03"),
    ("decided.cell_scaling", "Battery: stretch the cell further or accept the pack", TECHNOLOGY, "Chief engineer",
     "Equivalent-circuit 50G pack; no further cell power scaling", "2026-10-03"),
    ("decided.turbines_fuselage", "Turbogenerator location", ARCHITECTURE, "Configuration",
     "In the fuselage, behind the rear spar", "2026-10-04"),
    ("decided.fuselage", "Fuselage shape and length", ARCHITECTURE, "Configuration",
     "Boxy unpressurized section, 1.68 m wide, 2.0 m deep, 11 m long", "2026-10-04"),
    ("decided.redundancy_default", "Redundancy default", MARGIN, "Electrical", "2 lanes, 2 buses, 2 strings, switchable",
     "2026-10-04"),
    ("decided.real_machines", "Machines: rubber scaling or catalogue units", TECHNOLOGY, "Electrical",
     "Best-in-class catalogue units, whole unit counts", "2026-10-04"),
    ("decided.nacelles_tilt", "What tilts", ARCHITECTURE, "Configuration", "Whole tip nacelles tilt", "2026-10-04"),
    ("decided.geared_rotors", "Drive: direct or geared", ARCHITECTURE, "Rotor and drive",
     "Geared rotors and a step-up gearbox to each generator", ""),
    ("decided.ruddervator_limit", "Ruddervator deflection limit", MARGIN, "Flight controls and systems", "±25 deg",
     "2026-10-05"),
)


def register(ledger, gates=None, today=""):
    """Bring the ledger in line with this register; returns (added, retired).

    New trades are added; existing ones get the register's label, category, links and hook (their options and
    decisions are kept); open trades no longer listed are retired, with the date, and never ranked again.
    """
    gates = phases
    ledger.gates = dict(phases)
    for item in list(ledger.quantities.values()) + list(ledger.trades.values()):
        if item.lock_gate in renamed_gates:
            item.lock_gate = renamed_gates[item.lock_gate]
            item.lock_date = phases[item.lock_gate]
    added = retired = 0
    listed = {t[0] for t in open_trades} | {t[0] for t in decided_trades}
    for key, label, category, owner, gate, affects, hook, note in open_trades:
        t = ledger.trades.get(key)
        if t is None:
            ledger.trades[key] = Trade(key, label, owner, lock_gate=gate, lock_date=gates[gate], note=note,
                                       affects=list(affects), model_hook=hook, category=category)
            added += 1
            continue
        t.label, t.category, t.owner, t.affects, t.model_hook = label, category, owner, list(affects), hook
        t.lock_gate, t.lock_date, t.note = gate, gates[gate], note
        if t.status == "retired":
            t.status = "open"
    for key, label, category, owner, decision, decided in decided_trades:
        t = ledger.trades.get(key)
        if t is None:
            ledger.trades[key] = Trade(key, label, owner, status="decided", decision=decision, decided_date=decided,
                                       category=category, lock_gate=CONCEPTUAL, lock_date=phases[CONCEPTUAL])
            added += 1
        else:
            t.category, t.lock_gate, t.lock_date = category, CONCEPTUAL, phases[CONCEPTUAL]
    for key, t in ledger.trades.items():
        if key not in listed and t.status == "open":
            t.status, t.note = "retired", f"Below the line: not a governing trade (retired {today})"
            retired += 1
        elif key not in listed and t.status == "decided":
            t.status, t.note = "retired", f"Analysis-method choice, not an aircraft decision (retired {today})"
            retired += 1
    apply_precedents(ledger)
    return added, retired


# Precedents from comparable programmes (researched 2026-10-10). "As reported" marks a source whose full text could not
# be opened; its figures come from the publisher's summary. Each entry: (precedent, [(label, url), ...]).
TWZ_V280 = ("The War Zone: V-280 against V-22, with Bell's head of tiltrotor systems",
            "https://www.twz.com/21162/we-talk-v-280-valor-versus-v-22-osprey-with-bells-head-of-tiltrotor-systems")
FLIGHT_V22 = ("Flight International, 'Tilting in favour' (2000), as reported",
              "https://flightglobal.com/tilting-in-favour/33528.article")
NASA_X57 = ("NASA: X-57 project creates paths toward electric aviation",
            "https://www.nasa.gov/centers-and-facilities/armstrong/x-57-project-creates-paths-toward-electric-aviation-2/")
NTSB_JOBY = ("eVTOL Insights: NTSB report on the Joby JAS4-2 prototype crash",
             "https://evtolinsights.com/ntsb-finally-releases-report-on-joby-aviation-crash-jas4-2-prototype/")
FR_AW609 = ("Federal Register 2024-25238: AW609 airworthiness criteria",
            "https://www.govinfo.gov/content/pkg/FR-2024-10-31/html/2024-25238.htm")
PATENT_TILT = ("US 7,871,033: Tilt actuation for a rotorcraft (background on the V-22), as reported",
               "https://patents.google.com/patent/US7871033")
NASA_THIN_WING = ("Acree, Peyran and Johnson, NASA/TP-2004-212262 (thin-wing whirl flutter), as reported",
                  "https://rotorcraft.arc.nasa.gov/Publications/files/AcreePeyranJohnson_TP2004-212262.pdf")
NASA_XV15 = ("NASA NTRS 19810001546 (XV-15 description), as reported",
             "https://ntrs.nasa.gov/api/citations/19810001546/downloads/19810001546.pdf")

precedents = {
    "trade.tilt_load_path": (
        "V-22 tilts the whole nacelle: engine, gearbox and rotor. V-280 keeps the engines fixed and tilts only the "
        "proprotor gearbox; Bell cites maintenance (engine, shaft and gearbox removable independently) and says the "
        "V-280 weighs less than a V-22. Halo already keeps the turbines in the fuselage and tilts only motors, gearbox "
        "and rotor, the V-280 logic carried further.", [TWZ_V280]),
    "trade.conversion_actuator": (
        "V-22: a three-bar linkage with a linear actuator, two hydraulic motors and an electric third for triple "
        "redundancy; the nacelle is locked horizontal to unload the actuator. Joby JAS4-2: the NTSB named the tilt "
        "actuator linkage at one station, which let blades run steeper than commanded, as a contributing factor in a "
        "cascading prototype loss.", [PATENT_TILT, NTSB_JOBY]),
    "trade.tilt_joint_crossings": (
        "X-57: electromagnetic interference from the electric propulsion affected onboard systems and needed filters "
        "designed and installed during integration; the aircraft never flew. Routing HV across a rotating joint adds "
        "this exposure where it is hardest to shield.", [NASA_X57]),
    "trade.failure_architecture": (
        "XV-15 and V-22 both carry a cross-shaft so one engine drives both rotors; V-280 kept it. No tiltrotor that "
        "has flown has replaced the shaft with electrical cross-strapping, so Halo's choice carries certification "
        "risk as well as mass.", [NASA_XV15, TWZ_V280]),
    "trade.hv_voltage": (
        "X-57 hit interference between its inverters and onboard systems and redesigned its battery for overheating; "
        "integration problems and component shortages ended the programme before first flight.", [NASA_X57]),
    "trade.wing_material": (
        "The sizing prices aluminium (the material of the XV-15 calibration) at +699 lb take-off against "
        "graphite-epoxy. The V-22 lesson is in the details: composite parts that need many fasteners lose to machined "
        "aluminium (see the fuselage trade).", [FLIGHT_V22]),
    "trade.fuselage_material": (
        "V-22: empty weight ran 1,520 kg over specification by the end of full-scale development, partly from "
        "over-ambitious new materials. Composite frames needing many parts and fasteners were replaced by machined "
        "aluminium (32.4 lb against 26.4 lb per frame, 39 parts against 1, 258 fasteners against none); composite share "
        "fell from 57 % to 43 % of the airframe.", [FLIGHT_V22]),
    "trade.weight_margin": (
        "V-22: 1,520 kg empty-weight overrun in full-scale development; the next phase gave each integrated product "
        "team a weight allocation to hold or trade, the discipline this ledger encodes.", [FLIGHT_V22]),
    "trade.whirl_margin": (
        "XV-15 and V-22 use 23 % thick wings to keep whirl flutter beyond dive speed, at a drag and speed cost; a 15 % "
        "NASA design lost stiffness, and tailoring recovered stability at about 18 %. Halo's NACA 2423 follows the "
        "same precedent.", [NASA_THIN_WING]),
    "trade.assurance_level": (
        "AW609: applied in 2012, certified as a special-class powered-lift on a basis drawn from Parts 23, 25 and 29 "
        "(17,500 lb maximum weight); the criteria took effect on 2 December 2024. A tiltrotor's certification basis "
        "is itself a long-lead risk.", [FR_AW609]),
    "trade.integration_rig": (
        "X-57's integration problems (interference between inverters and onboard systems, a battery redesign) and "
        "late component shortages ended the programme before first flight.", [NASA_X57]),
    "trade.maintenance_concept": (
        "V-280 fixed its engines partly so that engine, drive shaft and gearbox can be removed independently, which "
        "Bell says cuts maintenance time.", [TWZ_V280]),
    "trade.means_of_compliance": (
        "AW609 needed new criteria for the transition between modes; its special-class basis took effect in "
        "December 2024, twelve years after application.", [FR_AW609]),
    "trade.cell_selection": (
        "X-57's battery had to be redesigned for overheating before it was cleared; containment and cell spacing are "
        "installed mass the cell data does not show.", [NASA_X57]),
}


def apply_precedents(ledger):
    for key, (text, sources) in precedents.items():
        if key in ledger.trades:
            ledger.trades[key].precedent, ledger.trades[key].sources = text, [list(s) for s in sources]
