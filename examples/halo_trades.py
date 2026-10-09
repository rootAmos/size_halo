"""Halo trade register: every trade from concept to first article, and the decisions already taken.

Each open trade names its discipline, owner, the gate where it locks, the ledger items it moves and, where the sizing
can already price its options, the model hook (an existing `HaloAssumptions` switch or helper). The rest need an
owner's option estimates before the ledger can value them. Decided entries record the decision and its date, so the
ledger holds the decision history as well as the open work.
"""
from aircraft_closure.ledger.model import Trade

# (key, label, discipline, owner, gate, affects, model hook, note)
open_trades = (
    # ---- Concept freeze: configuration, architecture, requirements --------------------------------------
    ("trade.hv_voltage", "HV bus voltage and regulation (floating or DC-DC; 540 to 1,000 V)", "Electrical",
     "Electrical", "Concept freeze", ["gap.hv_installation", "mass.battery", "mass.motors"],
     "assumptions_for_bus_voltage at 540, 756, 800, 1,000 V, with and without DC-DC",
     "Voltage and cell series count are one decision; the engine-out battery voltage limit binds"),
    ("trade.failure_architecture", "Interconnect shaft or electrical cross-strapping", "Electrical",
     "Electrical", "Concept freeze", ["mass.motors", "mass.generators", "mass.battery"], "",
     "Needs asymmetric failure cases with roll trim"),
    ("trade.redundancy_level", "Redundancy level: lanes, buses and battery strings (2/2/2 today)", "Electrical",
     "Electrical", "Concept freeze", ["mass.motors", "mass.protection_and_bus_tie", "mass.battery"],
     "redundancy on/off, count_strings_battery 1 to 3", "Set by the safety assessment's failure rates"),
    ("trade.thermal_architecture", "Thermal: ram-air exchanger, liquid loop or nacelle-local cooling", "Thermal",
     "Thermal", "Concept freeze", ["mass.heat_exchanger", "gap.coolant_loop", "gap.tilt_joint_crossings"], "",
     "Hot-day hover heat rejection binds"),
    ("trade.rotor_span", "Rotor diameter against wing span", "Configuration", "Configuration", "Concept freeze",
     ["Rotor radius capped by the span (rotor diameter against span)", "mass.rotors", "mass.wing"],
     "clearance_rotor_fuselage_m and the span cap", "The span cap is the most expensive binding limit"),
    ("trade.blade_count", "Blade count: 3 or 4", "Rotor and drive", "Rotor and drive", "Concept freeze",
     ["mass.rotors"], "count_blades 3 and 4", "Noise and hub loads not modelled"),
    ("trade.tip_speed", "Hover tip speed and airplane-mode rotor speed schedule", "Rotor and drive",
     "Rotor and drive", "Concept freeze", ["mass.rotors", "mass.rotor_gearboxes", "mass.motors"],
     "speed_tip_m_s; a rotor-speed schedule is not modelled yet", "Hover tip speed is used in cruise today"),
    ("trade.battery_reserve", "Battery reserve state of charge (30 % today)", "Electrical", "Chief engineer",
     "Concept freeze", ["mass.battery"], "free_soc_reserve with bounds_soc_reserve", ""),
    ("trade.cell_selection", "Cell selection and chemistry (50G today)", "Electrical", "Electrical",
     "Concept freeze", ["mass.battery", "input.cell_to_pack", "input.capacity_end_of_life"], "",
     "No further power scaling of the 50G (decided 2026-10-03)"),
    ("trade.machine_selection", "Motor and generator units: Evolito, magniX, EMRAX, Siemens or mixed",
     "Electrical", "Electrical", "Concept freeze", ["mass.motors", "mass.generators"],
     "machine database units (count_units_motor, count_units_generator)", ""),
    ("trade.gearbox_stages", "Rotor and generator gearbox stage counts and ratios", "Rotor and drive",
     "Rotor and drive", "Concept freeze", ["mass.rotor_gearboxes", "mass.generator_gearboxes"],
     "gearbox_stages with the staircase stage model", "Generator step-up near 31:1 is the open refinement"),
    ("trade.tail_type", "V-tail or conventional tail", "Configuration", "Flight controls and systems",
     "Concept freeze", ["mass.tails", "static_margin"], "", "Drawn as a V-tail; sized as conventional"),
    ("trade.wing_section", "Wing thickness ratio and section (NACA 2423 today)", "Structures", "Structures",
     "Concept freeze", ["mass.wing", "gap.wing_strength"], "", "Whirl flutter against drag"),
    ("trade.wing_material", "Wing material: graphite-epoxy or aluminium", "Structures", "Structures",
     "Concept freeze", ["mass.wing"], "wing_material graphite_epoxy and aluminium", ""),
    ("trade.cruise_point", "Cruise speed and altitude requirement (210 kt, 10,000 ft)", "Requirements",
     "Chief engineer", "Concept freeze", ["mass.battery"], "HaloRequirements velocity_max_m_s, altitude_cruise_m",
     ""),
    ("trade.payload_range", "Payload against range (900 kg, 445 nm)", "Requirements", "Chief engineer",
     "Concept freeze", [], "HaloRequirements mass_payload_kg, range_m", "Payload-range diagram not drawn yet"),
    ("trade.hot_high", "Hot and high hover requirement (4,000 ft, 95 F)", "Requirements", "Chief engineer",
     "Concept freeze", ["mass.heat_exchanger"], "HaloRequirements altitude_hover_hot_m, temperature_hover_hot_K",
     ""),
    ("trade.engine_out_hover", "Engine-out hover duration and reserve", "Requirements", "Chief engineer",
     "Concept freeze", ["mass.battery", "mass.generators"], "HaloRequirements duration_engine_out_hover_s", ""),
    # ---- Preliminary design review: subsystem concepts ---------------------------------------------------
    ("trade.tilt_load_path", "Tilting load path: spindle station, tip rib and fitting concept", "Structures",
     "Structures", "Preliminary design review", ["gap.tilt_fittings", "mass.wing", "mass.nacelles"], "",
     "Whirl flutter and jump take-off root strain"),
    ("trade.conversion_actuator", "Conversion actuator: electromechanical or hydraulic, and its stiffness",
     "Flight controls", "Flight controls and systems", "Preliminary design review",
     ["gap.conversion_actuators"], "", "The actuator is the pylon pitch spring for whirl flutter"),
    ("trade.tilt_joint_crossings", "Tilt-joint crossings: flex loops or slip rings; coolant swivel or local HX",
     "Electrical", "Electrical", "Preliminary design review", ["gap.tilt_joint_crossings", "gap.coolant_loop"], "",
     ""),
    ("trade.wing_covers", "Wing covers: unstiffened, stringer-stiffened or sandwich", "Structures", "Structures",
     "Preliminary design review", ["gap.wing_strength", "mass.wing"], "",
     "FE shows a negative margin on 1 mm unstiffened covers; no buckling step yet"),
    ("trade.rotor_hub", "Rotor hub: gimballed or hingeless", "Rotor and drive", "Rotor and drive",
     "Preliminary design review", ["mass.rotors", "mass.nacelles"], "", "Hub moments drive the tilt fittings"),
    ("trade.landing_gear", "Landing gear: retractable or fixed, wheels or skids", "Structures", "Structures",
     "Preliminary design review", ["mass.landing_gear"], "", ""),
    ("trade.battery_location", "Battery location and containment", "Electrical", "Electrical",
     "Preliminary design review", ["mass.battery", "gap.hv_installation"], "", "CG and crash loads"),
    ("trade.conductor", "Cable conductor: aluminium or copper", "Electrical", "Electrical",
     "Preliminary design review", ["gap.hv_installation"], "conductor_cable aluminium_conductor, copper_conductor",
     ""),
    ("trade.inverter_location", "Inverter location: nacelle or fuselage", "Electrical", "Electrical",
     "Preliminary design review", ["mass.motors", "gap.tilt_joint_crossings"], "", "AC or DC across the tilt joint"),
    ("trade.partial_discharge", "Partial-discharge insulation strategy at altitude", "Electrical", "Electrical",
     "Preliminary design review", ["gap.hv_installation"], "", "Couples to the bus voltage"),
    ("trade.flight_control_actuation", "Flight control actuation: electromechanical or hydraulic, lane count",
     "Flight controls", "Flight controls and systems", "Preliminary design review", ["mass.systems"], "",
     "Flight controls carry an XV-15 factor of about 4"),
    ("trade.autonomy_avionics", "Autonomy and avionics architecture and lane count", "Systems", "Systems",
     "Preliminary design review", ["mass.fixed_equipment"], "", "100 lb allocation today"),
    ("trade.ice_protection", "Ice protection: none, electrothermal rotor and wing, or inlets only", "Systems",
     "Systems", "Preliminary design review", ["mass.fixed_equipment"], "", "Mission icing requirement open"),
    ("trade.lightning", "Lightning and HIRF protection on composite structure", "Structures", "Structures",
     "Preliminary design review", ["mass.wing", "mass.fuselage"], "", ""),
    # ---- Critical design review: detail design ------------------------------------------------------------
    ("trade.fitting_material", "Tilt and wing fittings: titanium, steel or composite", "Structures", "Structures",
     "Critical design review", ["gap.tilt_fittings"], "", ""),
    ("trade.skin_joints", "Skin joints: bonded or bolted", "Structures", "Structures", "Critical design review",
     ["mass.wing", "mass.fuselage"], "", "Repairability against mass"),
    ("trade.harness_standard", "Harness routing and connector standard", "Electrical", "Electrical",
     "Critical design review", ["gap.hv_installation", "mass.systems"], "", ""),
    ("trade.exchanger_core", "Heat exchanger core type", "Thermal", "Thermal", "Critical design review",
     ["mass.heat_exchanger"], "", ""),
    # ---- First article: manufacturing ---------------------------------------------------------------------
    ("trade.wing_skin_process", "Wing skin process: automated fibre placement or hand layup", "Manufacturing",
     "Manufacturing", "First article", ["mass.wing"], "", "Thickness tolerance is mass"),
    ("trade.fuselage_assembly", "Fuselage: monolithic or panelized assembly", "Manufacturing", "Manufacturing",
     "First article", ["mass.fuselage"], "", ""),
    ("trade.fitting_process", "Fittings: machined or additively manufactured", "Manufacturing", "Manufacturing",
     "First article", ["gap.tilt_fittings"], "", ""),
    ("trade.finish_allowances", "Paint, sealant and tolerance allowances", "Manufacturing", "Weights",
     "First article", ["mass.fuselage", "mass.wing"], "", ""),
)

# (key, label, discipline, owner, decision, decided date)
decided_trades = (
    ("decided.engines", "Turboshafts: off-the-shelf or sized", "Propulsion", "Chief engineer",
     "Fixed 2 x 1,120 hp off-the-shelf engines; never a design variable", "2026-10-03"),
    ("decided.max_speed", "Maximum speed: 250 or 210 kt", "Requirements", "Chief engineer",
     "210 kt; 250 kt is infeasible with the fixed engines", "2026-10-03"),
    ("decided.payload", "Reference payload", "Requirements", "Chief engineer", "900 kg", "2026-10-03"),
    ("decided.cell_scaling", "Battery: stretch the cell further or accept the pack", "Electrical",
     "Chief engineer", "Equivalent-circuit 50G pack; no further cell power scaling", "2026-10-03"),
    ("decided.wing_model", "Wing weight method", "Structures", "Weights", "NDARC tiltrotor wing (AFDD)",
     "2026-10-03"),
    ("decided.aero_model", "Aerodynamics method", "Aerodynamics", "Aerodynamics", "AeroBuildup by default",
     "2026-10-04"),
    ("decided.turbines_fuselage", "Turbogenerator location", "Configuration", "Configuration",
     "In the fuselage, behind the rear spar", "2026-10-04"),
    ("decided.fuselage", "Fuselage shape and length", "Configuration", "Configuration",
     "Boxy unpressurized section, 1.68 m wide, 2.0 m deep, 11 m long", "2026-10-04"),
    ("decided.redundancy_default", "Redundancy default", "Electrical", "Electrical",
     "2 lanes, 2 buses, 2 strings, switchable", "2026-10-04"),
    ("decided.real_machines", "Machines: rubber scaling or catalogue units", "Electrical", "Electrical",
     "Best-in-class catalogue units, whole unit counts", "2026-10-04"),
    ("decided.nacelles_tilt", "Tilt concept", "Configuration", "Configuration", "Whole tip nacelles tilt",
     "2026-10-04"),
    ("decided.geared_rotors", "Drive: direct or geared", "Rotor and drive", "Rotor and drive",
     "Geared rotors and a step-up gearbox to each generator", ""),
    ("decided.ruddervator_limit", "Ruddervator deflection limit", "Flight controls", "Flight controls and systems",
     "±25 deg", "2026-10-05"),
)


def register(ledger, gates):
    """Add every trade the ledger does not hold yet (existing entries, with their options, are kept)."""
    added = 0
    for key, label, discipline, owner, gate, affects, hook, note in open_trades:
        if key in ledger.trades:
            t = ledger.trades[key]
            t.discipline, t.affects, t.model_hook = discipline, list(affects), hook
            t.note = t.note if t.options else note
            continue
        ledger.trades[key] = Trade(key, label, owner, lock_gate=gate, lock_date=gates[gate], note=note,
                                   discipline=discipline, affects=list(affects), model_hook=hook)
        added += 1
    for key, label, discipline, owner, decision, decided in decided_trades:
        if key not in ledger.trades:
            ledger.trades[key] = Trade(key, label, owner, status="decided", decision=decision,
                                       discipline=discipline, decided_date=decided)
            added += 1
    return added
