"""Maturity paths: the concrete engineering steps that firm up each uncertain quantity, phase by phase.

Each step is (action, phase, spread after it). The spread is a fraction of the quantity's value for masses and an
absolute spread for model inputs. A step is done when the quantity's current spread is already at or below what the
step would leave, so a catalogue machine starts past "select supplier". The next undone step is the item's next
action; its phase lock date is the step's due date. Spreads follow the maturity categories of the weight prediction
(estimated 15 %, calibrated 10 %, layout 8 %, vendor data 5 %, vendor hardware 2 %) and are proposals for the owners.
"""
from aircraft_closure.ledger.model import Activity
from examples.halo_trades import CONCEPTUAL, DETAILED, PRELIMINARY, TEST, phases

BOUGHT = (
    ("Select supplier and candidate unit; request mass and ratings", CONCEPTUAL, 0.07),
    ("Verify the datasheet: mass, ratings and derating at altitude and hot day", PRELIMINARY, 0.05),
    ("Installed weight from layout: mounts, cooling, harness, protection", PRELIMINARY, 0.035),
    ("Weigh the delivered unit with its installation kit", TEST, 0.015),
)
STRUCTURE = (
    ("Size primary structure for every load case (strength, stiffness, whirl flutter)", CONCEPTUAL, 0.09),
    ("Structural layout: joints, fittings, cut-outs and access panels", PRELIMINARY, 0.07),
    ("FE sizing with buckling and fatigue; gauges back to the weight model", PRELIMINARY, 0.05),
    ("Detail drawings and parts list weight", DETAILED, 0.025),
    ("Weigh the first article", TEST, 0.01),
)
DRIVE = (
    ("Size gears, shafts and bearings to the load spectrum", CONCEPTUAL, 0.08),
    ("Select the gearbox supplier; confirm ratings and lubrication", PRELIMINARY, 0.06),
    ("Layout with housing, mounts and oil system", PRELIMINARY, 0.04),
    ("Weigh the first gearbox", TEST, 0.015),
)
ROTOR = (
    ("Size blades and hub to loads, frequencies and whirl stability", CONCEPTUAL, 0.06),
    ("Blade structural design: spar, skins, ballast", PRELIMINARY, 0.045),
    ("Hub and pitch-link layout", PRELIMINARY, 0.035),
    ("Weigh the first blade set and hub", TEST, 0.015),
)
SYSTEMS = (
    ("Define the equipment list and architecture (lanes, actuators, computers)", CONCEPTUAL, 0.12),
    ("Select suppliers for the main units; collect datasheet masses", PRELIMINARY, 0.08),
    ("Installation layout: brackets, harness, cooling", DETAILED, 0.04),
    ("Weigh installed systems on the first article", TEST, 0.015),
)
BATTERY = (
    ("Select the cell supplier and confirm the cell datasheet at end of life", CONCEPTUAL, 0.06),
    ("Module design: cell-to-pack mass from a real module", PRELIMINARY, 0.04),
    ("Thermal-runaway containment test; add what containment needs", PRELIMINARY, 0.03),
    ("Weigh the first pack", TEST, 0.01),
)

paths = {
    "mass.wing": STRUCTURE, "mass.tails": STRUCTURE, "mass.fuselage": STRUCTURE, "mass.nacelles": STRUCTURE,
    "mass.landing_gear": STRUCTURE,
    "mass.systems": SYSTEMS, "mass.fixed_equipment": SYSTEMS,
    "mass.rotors": ROTOR, "mass.rotor_gearboxes": DRIVE, "mass.generator_gearboxes": DRIVE,
    "mass.motors": BOUGHT, "mass.generators": BOUGHT, "mass.turboshafts": BOUGHT, "mass.heat_exchanger": BOUGHT,
    "mass.protection_and_bus_tie": BOUGHT, "mass.battery": BATTERY,
    "gap.hv_installation": (
        ("Fix bus voltage and architecture; write the electrical load list", CONCEPTUAL, 0.22),
        ("Wire list and harness routing from the layout", PRELIMINARY, 0.12),
        ("Select cables, connectors and contactors; partial-discharge rating at altitude", PRELIMINARY, 0.08),
        ("Harness mock-up weight", DETAILED, 0.04),
        ("Weigh the installed HV system", TEST, 0.015)),
    "gap.tilt_fittings": (
        ("Choose the spindle concept and bearing type", CONCEPTUAL, 0.45),
        ("Size spindle, bearings and fittings for conversion and jump take-off loads", PRELIMINARY, 0.2),
        ("Fitting layout and FE with fatigue", PRELIMINARY, 0.1),
        ("Detail drawings", DETAILED, 0.04),
        ("Weigh the first fittings", TEST, 0.015)),
    "gap.conversion_actuators": (
        ("Select actuator type (electromechanical or hydraulic) and redundancy", CONCEPTUAL, 0.45),
        ("Size for hinge moment, rate and stiffness; select supplier", PRELIMINARY, 0.15),
        ("Verify the supplier datasheet and installation", PRELIMINARY, 0.07),
        ("Weigh the delivered actuators", TEST, 0.015)),
    "gap.coolant_loop": (
        ("Define the loop: fluid, pumps, routing", CONCEPTUAL, 0.45),
        ("Size pumps, lines and reservoir to the heat loads", PRELIMINARY, 0.2),
        ("Select suppliers; datasheet masses", PRELIMINARY, 0.08),
        ("Weigh the installed loop with fluid", TEST, 0.02)),
    "gap.tilt_joint_crossings": (
        ("Select flex loop or slip ring; swivel or local cooling", CONCEPTUAL, 0.45),
        ("Supplier datasheet and bend-radius layout", PRELIMINARY, 0.15),
        ("Mock-up and conversion-cycle test", DETAILED, 0.06),
        ("Weigh the installed crossings", TEST, 0.02)),
    "gap.wing_strength": (
        ("FE with calibrated structure, root fittings and a buckling step", CONCEPTUAL, 0.6),
        ("Size stiffeners or sandwich covers to a non-negative margin", PRELIMINARY, 0.25),
        ("Detail design of the covers", DETAILED, 0.08),
        ("Static test of the wing box", TEST, 0.02)),
    "input.drag_excrescence": (
        ("Component drag build-up from the drawn layout", CONCEPTUAL, 0.07),
        ("CFD of the installed nacelle, wing and fuselage", PRELIMINARY, 0.05),
        ("Wind-tunnel test", DETAILED, 0.03),
        ("Flight-test drag polar", TEST, 0.015)),
    "input.cell_to_pack": (
        ("Pack concept with module count and structure", CONCEPTUAL, 0.02),
        ("Module design", PRELIMINARY, 0.012),
        ("Weigh the first module", DETAILED, 0.005)),
    "input.capacity_end_of_life": (
        ("Supplier ageing data at the mission duty cycle", CONCEPTUAL, 0.035),
        ("Cell ageing test at the mission duty cycle", PRELIMINARY, 0.02),
        ("Pack ageing data from test and validation", TEST, 0.01)),
}


def apply_paths(ledger):
    """Replace each quantity's plan with its maturity path; steps it has already reached are marked done."""
    for key, steps in paths.items():
        q = ledger.quantities.get(key)
        if q is None:
            continue
        mean, sigma = q.belief()
        relative = q.kind != "input"
        plan = []
        for action, phase, spread in steps:
            sigma_after = spread * mean if relative else spread
            plan.append(Activity(f"{action} ({phase.lower()})", phases[phase], sigma_after, q.owner,
                                 done=sigma_after >= sigma * (1 - 1e-9)))
        q.plan = plan
