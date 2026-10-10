"""Twelve months to first flight: a simpler Block 0 flight article first, the final design (Block 1) in parallel.

First flight is a hover in helicopter mode, nacelles locked at 90 deg, on 2027-10-10 (twelve months from
2026-10-10); conversion is cleared afterwards by build-up flight test. Block 0 takes the low-risk, fast option on every
trade that gates first flight; the rest lock on the Block 1 calendar, informed by the integration rig, the Block 0
build and its flight data. Every date here is a proposal.
"""
first_flight = "2027-10-10"

# Phase lock dates. Block 0: the twelve-month plan. Block 1: the final design, which locks later on the same phases.
block0_phases = {"Conceptual design": "2026-12-15", "Preliminary design": "2027-03-15",
                 "Detailed design": "2027-06-15", "Test and validation": first_flight,
                 "After entry into service": "2030-12-31"}
block1_phases = {"Conceptual design": "2027-04-30", "Preliminary design": "2027-12-15",
                 "Detailed design": "2028-09-30", "Test and validation": "2029-09-30",
                 "After entry into service": "2030-12-31"}

# Open trade -> (block in which it must lock, the Block 0 choice for first flight).
blocks = {
    "trade.tilt_load_path": ("Block 0", "Conservative machined spindle and fittings with generous margin; nacelles "
                                        "locked at 90 deg for first flight"),
    "trade.conversion_actuator": ("Block 0", "Catalogue dual-motor electromechanical actuator; conversion cleared "
                                             "after first flight"),
    "trade.tilt_joint_crossings": ("Block 0", "Flex loops, no slip rings; air cooling local to the nacelle where it "
                                              "suffices"),
    "trade.failure_architecture": ("Block 0", "Electrical cross-strapping as designed; the uncrewed test article "
                                              "carries the risk with flight termination. Shaft question goes to the "
                                              "certification authority for Block 1"),
    "trade.hv_voltage": ("Block 0", "The bus voltage native to the selected machines and inverters, fixed by month 2"),
    "trade.thermal_architecture": ("Block 0", "Ram-air exchanger with ground fans; hot-day envelope deferred"),
    "trade.rotor_span": ("Block 0", "Today's span-capped rotor"),
    "trade.rotor_hub": ("Block 0", "Gimballed hub (XV-15 and V-22 heritage)"),
    "trade.wing_material": ("Block 0", "Whatever the structures supplier can deliver by month 9; aluminium costs "
                                       "+699 lb take-off (priced), taken from payload"),
    "trade.fuselage_material": ("Block 0", "Metallic semi-monocoque or simple composite panels; optimise in Block 1 "
                                           "(V-22 lesson)"),
    "trade.rotor_construction": ("Block 0", "The blade supplier's existing family, adapted"),
    "trade.cell_selection": ("Block 0", "50G cells in the supplier's modules; catalogue machines"),
    "trade.payload_range": ("Block 0", "Reduced payload accepted for Block 0; full mission in Block 1"),
    "trade.hover_cases": ("Block 0", "ISA sea-level hover only; hot and high in Block 1"),
    "trade.redundancy_level": ("Block 0", "2 lanes, 2 buses, 2 strings as designed"),
    "trade.battery_reserve": ("Block 0", "30 % reserve or more"),
    "trade.load_factors": ("Block 0", "Restricted, placarded envelope"),
    "trade.whirl_margin": ("Block 0", "Frequency placement with wide margin; airplane-mode speed limited until "
                                      "flutter is cleared"),
    "trade.integration_rig": ("Block 0", "Yes: HV powertrain and one tilting nacelle on an iron bird by month 9 "
                                         "(X-57 lesson)"),
    "trade.envelope_expansion": ("Block 0", "Build-up: hover, low speed, then conversion"),
    "trade.weight_margin": ("Block 1", "Track Block 0 against its predicted weight; set the NTE for Block 1"),
    "trade.fitting_material": ("Block 1", "Machined steel or titanium in Block 0"),
    "trade.structure_joining": ("Block 1", "Fastened in Block 0"),
    "trade.hv_installation_standard": ("Block 1", "Aerospace catalogue connectors with generous clearance in "
                                                  "Block 0"),
    "trade.assurance_level": ("Block 1", "Experimental permit to fly for Block 0"),
    "trade.means_of_compliance": ("Block 1", ""),
    "trade.maintenance_concept": ("Block 1", ""),
    "trade.battery_life": ("Block 1", ""),
    "trade.growth_reserve": ("Block 1", ""),
}

# Gantt: (lane, task, start, end); start == end is a milestone.
schedule = (
    ("Block 0 design", "Configuration freeze and supplier selection", "2026-10-10", "2026-12-15"),
    ("Block 0 design", "Configuration freeze", "2026-12-15", "2026-12-15"),
    ("Block 0 design", "Preliminary design", "2026-12-15", "2027-03-15"),
    ("Block 0 design", "Preliminary design review", "2027-03-15", "2027-03-15"),
    ("Block 0 design", "Detailed design", "2027-03-15", "2027-06-15"),
    ("Block 0 design", "Critical design review", "2027-06-15", "2027-06-15"),
    ("Long-lead parts", "Order machines, turbines, gearboxes, blades, cells, actuators", "2026-11-15", "2027-01-31"),
    ("Long-lead parts", "Deliveries", "2027-05-01", "2027-07-31"),
    ("Integration rig", "Build: HV powertrain and one tilting nacelle", "2027-01-15", "2027-05-31"),
    ("Integration rig", "Runs: power, EMI, failure cases, conversion cycles", "2027-06-01", "2027-08-31"),
    ("Airframe", "Tooling and long-lead structure", "2027-04-01", "2027-06-30"),
    ("Airframe", "Fabrication", "2027-06-15", "2027-08-15"),
    ("Airframe", "Assembly and systems integration", "2027-08-01", "2027-09-10"),
    ("Airframe", "Weigh the first article", "2027-09-10", "2027-09-10"),
    ("Test", "Ground runs and tethered hover", "2027-09-10", "2027-10-05"),
    ("Test", "Flight readiness review", "2027-10-05", "2027-10-05"),
    ("Test", "First flight: hover, helicopter mode", first_flight, first_flight),
    ("Test", "Envelope expansion to conversion", first_flight, "2028-03-31"),
    ("Block 1 (final design)", "Block 1 trades and design, from rig and Block 0 data", "2027-03-01", "2027-12-15"),
    ("Block 1 (final design)", "Block 1 preliminary design review", "2027-12-15", "2027-12-15"),
)


def apply_schedule(ledger):
    """Set the phase calendar, each trade's block and Block 0 choice, and the schedule."""
    ledger.gates = dict(block0_phases)
    ledger.baseline = dict(ledger.baseline, first_flight=first_flight)
    for q in ledger.quantities.values():
        if q.lock_gate in block0_phases:
            q.lock_date = block0_phases[q.lock_gate]
            for a in q.plan:
                phase = a.label.rsplit("(", 1)[-1].rstrip(")").capitalize()
                if phase in block0_phases:
                    a.due = block0_phases[phase]
    for key, t in ledger.trades.items():
        if key in blocks:
            t.block, t.block0_choice = blocks[key]
            if t.lock_gate:
                t.lock_date = (block0_phases if t.block == "Block 0" else block1_phases)[t.lock_gate]
        elif t.status == "decided":
            t.block = "Block 0"
    for limit in ledger.limits.values():
        if limit.lock_gate in block0_phases:
            limit.lock_date = block0_phases[limit.lock_gate]
    ledger.schedule = [dict(lane=lane, task=task, start=start, end=end) for lane, task, start, end in schedule]
