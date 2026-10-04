"""Solve the Halo reference and save what the structural back-check needs (plan 031).

Writes `output/structure/reference.json`: the solved geometry, the structural
design condition (design mass, ultimate load factor, jump load factor), the
sizing's component masses (AFDD wing breakdown, Raymer tails and fuselage) and
the AFDD wing's required stiffnesses and wall areas. Run from the repo root:
`python -m examples.halo_structure_reference` (several minutes).
"""
import json
from dataclasses import fields
from pathlib import Path

from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import HaloAssumptions, HaloRequirements, build_halo_aircraft, solve_halo_sizing

path_output = Path("output/structure/reference.json")

if __name__ == "__main__":
    requirements, assumptions = HaloRequirements(), HaloAssumptions()
    result = solve_halo_sizing(requirements, assumptions)
    aircraft = build_halo_aircraft(result.design, requirements, assumptions)
    condition = StructuralDesignCondition(mass_design_kg=result.mass_takeoff_kg,
                                          load_factor_ultimate=assumptions.load_factor_ultimate,
                                          velocity_cruise_m_s=result.velocity_cruise_m_s,
                                          altitude_cruise_m=requirements.altitude_cruise_m,
                                          lift_to_drag_cruise=result.lift_to_drag_cruise)
    breakdown = aircraft.get_mass_breakdown(condition)
    wing = aircraft.wing
    model = wing.mass_model
    wing_masses = model.masses(wing, condition)
    material = model.material
    reference = dict(
        mass_takeoff_kg=float(result.mass_takeoff_kg),
        load_factor_ultimate=float(assumptions.load_factor_ultimate),
        load_factor_jump=float(assumptions.load_factor_jump),
        span_wing_m=float(wing.span_m()), area_wing_m2=float(wing.area_m2), chord_wing_m=float(wing.chord_mean_m()),
        thickness_to_chord_wing=float(model.thickness_to_chord),
        fraction_chord_torque_box=float(model.fraction_chord_torque_box),
        width_fuselage_m=float(model.width_fuselage_m),
        mass_tip_kg=float(model.mass_tip_kg),
        radius_rotor_m=float(result.radius_rotor_m),
        area_horizontal_tail_m2=float(result.design.area_horizontal_tail_m2),
        area_vertical_tail_m2=float(result.design.area_vertical_tail_m2),
        x_le_wing_m=float(result.design.x_le_wing_m),
        material_wing={f.name: float(getattr(material, f.name)) for f in fields(material)},
        wing_afdd={f.name: float(getattr(wing_masses, f.name)) for f in fields(wing_masses)},
        mass_wing_kg=float(breakdown.wing.mass), mass_horizontal_tail_kg=float(breakdown.horizontal_tail.mass),
        mass_vertical_tail_kg=float(breakdown.vertical_tail.mass), mass_fuselage_kg=float(breakdown.fuselage.mass),
        factor_wing=float(wing.mass_factor),
    )
    path_output.parent.mkdir(parents=True, exist_ok=True)
    path_output.write_text(json.dumps(reference, indent=1))
    print(json.dumps(reference, indent=1))
