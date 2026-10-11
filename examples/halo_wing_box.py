"""Size the Halo with the bottom-up wing box (plan 054) and write its sized layout for the structure and FE check.

Writes `output/structure/wing_box_layout.json`: take-off mass, the wing breakdown, realized stiffness and
frequencies, and per station the skin, stringer, web and spar-cap sizes, stringer pitch and height, rib pitch and
thickness. Run from the repo root: `python -m examples.halo_wing_box` (one coupled solve, several minutes).
"""
import json
from dataclasses import fields, replace
from pathlib import Path

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import HaloAssumptions, HaloRequirements, build_halo_aircraft, solve_halo_sizing

path_output = Path("output/structure/wing_box_layout.json")


def wing_box_layout(result, requirements, assumptions):
    """The sized wing box of a solved Halo (bottom-up wing) as plain numbers."""
    aircraft = build_halo_aircraft(result.design, requirements, assumptions)
    condition = StructuralDesignCondition(mass_design_kg=result.mass_takeoff_kg,
                                          load_factor_ultimate=assumptions.load_factor_ultimate,
                                          velocity_cruise_m_s=result.velocity_cruise_m_s,
                                          altitude_cruise_m=requirements.altitude_cruise_m,
                                          lift_to_drag_cruise=result.lift_to_drag_cruise)
    wing = aircraft.wing
    model = wing.mass_model
    sizing = model.masses(wing, condition)
    number = lambda v: [float(x) for x in v] if isinstance(v, tuple) else float(v)
    return dict(mass_takeoff_kg=float(result.mass_takeoff_kg), span_wing_m=float(wing.span_m()),
                chord_wing_m=float(wing.chord_mean_m()), x_le_wing_m=float(wing.x_le_root_m),
                thickness_to_chord_wing=float(model.thickness_to_chord), width_fuselage_m=float(model.width_fuselage_m),
                mass_tip_kg=float(model.mass_tip_kg), radius_gyration_pylon_m=float(model.radius_gyration_pylon_m),
                offset_cg_tip_m=float(model.offset_cg_tip_m), load_factor_ultimate=float(model.load_factor_ultimate),
                load_factor_jump=float(model.load_factor_jump),
                fraction_chord_front_spar=float(model.layout.fraction_chord_front_spar),
                fraction_chord_rear_spar=float(model.layout.fraction_chord_rear_spar),
                material={f.name: float(getattr(model.material, f.name)) for f in fields(model.material)},
                mass_wing_kg=float(sizing.total()), mass_primary_kg=float(sizing.mass_primary_kg()),
                **{f.name: number(getattr(sizing, f.name)) for f in fields(sizing)})


def compare(reference, result):
    """Group masses, binding constraints and key design values of two solved Halos (reference first)."""
    groups = dict(reference.component_masses_kg)
    for name, mass_kg in result.component_masses_kg:
        delta = mass_kg - groups.get(name, 0.0)
        if abs(delta) > 1.0:
            print(f"  {name:<34} {groups.get(name, 0.0):8.1f} -> {mass_kg:8.1f} kg ({delta:+.1f})")
    for label, ref, new in (("take-off", reference.mass_takeoff_kg, result.mass_takeoff_kg),
                            ("rotor radius m", reference.radius_rotor_m, result.radius_rotor_m),
                            ("span m", reference.span_m, result.span_m),
                            ("cruise speed m/s", reference.velocity_cruise_m_s, result.velocity_cruise_m_s),
                            ("wing design rotor speed rad/s", reference.design.speed_rotor_wing_design_rad_s,
                             result.design.speed_rotor_wing_design_rad_s)):
        print(f"  {label:<34} {float(ref):8.2f} -> {float(new):8.2f}")
    print("  binding (reference): " + ", ".join(str(b) for b in reference.binding))
    print("  binding (bottom-up): " + ", ".join(str(b) for b in result.binding))


if __name__ == "__main__":
    import sys
    requirements = HaloRequirements()
    assumptions = replace(HaloAssumptions(), wing_weight_model="bottom_up_tiltrotor",
                          free_pitch_rib_wing="--free-ribs" in sys.argv)
    result = solve_halo_sizing(requirements, assumptions)
    if "--compare" in sys.argv:
        print("AFDD default against the bottom-up wing:")
        compare(solve_halo_sizing(requirements, HaloAssumptions()), result)
    layout = wing_box_layout(result, requirements, assumptions)
    path_output.parent.mkdir(parents=True, exist_ok=True)
    path_output.write_text(json.dumps(layout, indent=1))
    lb = 0.45359237
    print(f"take-off {layout['mass_takeoff_kg']:.1f} kg ({layout['mass_takeoff_kg'] / lb:,.0f} lb); wing "
          f"{layout['mass_wing_kg']:.1f} kg; start {result.start}; rib pitch {layout["pitch_rib_m"]:.3f} m")
    print("wing: " + ", ".join(f"{k[5:-3]} {layout[k]:.1f}" for k in layout if k.startswith("mass_") and k not in ("mass_takeoff_kg", "mass_tip_kg")))
