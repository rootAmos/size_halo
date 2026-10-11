"""Plan 050: our offset-strip-fin core (plan 042) against OpenConcept's HXGroup, code to code.

Three steps, two environments (OpenConcept is not a dependency of this repository):

1. Repository environment, write the cases (every input as a plain number):
       python -m examples.ntu_openconcept_check cases <cases.json>
2. An environment with OpenConcept (run isolated, nothing from this repository is imported):
       python -I examples/ntu_openconcept_check.py openconcept <cases.json> <out.json>
   builds one `openconcept.thermal.heat_exchanger.HXGroup` per case and stores its outputs with the inputs; the
   stored file is tests/powertrain/data/compact_heat_exchanger_openconcept.json.
3. Repository environment, compare:
       python -m examples.ntu_openconcept_check compare [<out.json>]
   prints the relative difference per quantity per case.

Sign and scope conventions: OpenConcept's pressure drops are pressure changes (negative), ours are losses
(positive), so magnitudes are compared; OpenConcept's `component_weight` is material only, ours adds the coolant
held in the core, so our material mass is compared.
"""
import json
import sys
from datetime import date
from pathlib import Path

stored_path = Path(__file__).resolve().parents[1] / "tests" / "powertrain" / "data" / "compact_heat_exchanger_openconcept.json"

# Quantity name -> OpenConcept output name.
quantities = {
    "width_m": "width_overall", "height_m": "height_overall", "length_m": "length_overall",
    "diameter_hydraulic_air_m": "dh_cold", "diameter_hydraulic_coolant_m": "dh_hot",
    "area_flow_air_m2": "xs_area_cold", "area_flow_coolant_m2": "xs_area_hot",
    "area_heat_air_m2": "heat_transfer_area_cold", "area_heat_coolant_m2": "heat_transfer_area_hot",
    "reynolds_air": "Re_dh_cold", "reynolds_coolant": "Re_dh_hot",
    "colburn_air": "j_cold", "colburn_coolant": "j_hot", "fanning_air": "f_cold", "fanning_coolant": "f_hot",
    "coefficient_air_W_m2_K": "h_conv_cold", "coefficient_coolant_W_m2_K": "h_conv_hot",
    "efficiency_surface_air": "eta_overall_cold", "efficiency_surface_coolant": "eta_overall_hot",
    "conductance_W_K": "UA_overall", "ntu": "NTU", "effectiveness": "effectiveness",
    "power_heat_W": "heat_transfer", "temperature_air_out_K": "T_out_cold",
    "temperature_coolant_out_K": "T_out_hot", "pressure_drop_air_Pa": "delta_p_cold",
    "pressure_drop_coolant_Pa": "delta_p_hot", "mass_material_kg": "component_weight",
}


def build_cases():
    """The comparison cases, every input a float (repository environment: uses our air properties)."""
    import aerosandbox as asb
    from aircraft_closure.powertrain.components.compact_heat_exchanger import air_properties, water_glycol_properties

    def fluid(props):
        return {"cp": float(props.specific_heat_J_kg_K), "k": float(props.conductivity_W_m_K),
                "mu": float(props.viscosity_Pa_s), "rho": float(props.density_kg_m3)}

    geometry = {"channel_width_cold": 1.35e-3, "channel_height_cold": 14e-3, "fin_length_cold": 6e-3,
                "channel_width_hot": 1e-3, "channel_height_hot": 1e-3, "fin_length_hot": 6e-3,
                "fin_thickness": 0.102e-3, "plate_thickness": 0.2e-3, "case_thickness": 2e-3,
                "material_k": 190.0, "material_rho": 2700.0}
    # OpenConcept's HXGroup defaults: counts, air and coolant properties; operating point from its
    # simple_all_electric example (coolant 0.1 kg/s at 997 kg/m3), sea-level air.
    sea_level = asb.Atmosphere(altitude=0.0)
    cases = [{"name": "openconcept_defaults", **geometry, "n_wide_cold": 430.0, "n_long_cold": 3.0, "n_tall": 19.0,
              "air": {"cp": 1005.0, "k": 0.02596, "mu": 1.789e-5, "rho": float(sea_level.density())},
              "coolant": {"cp": 3801.0, "k": 0.405, "mu": 1.68e-3, "rho": 997.0},
              "mdot_cold": 1.0, "mdot_hot": 0.1, "T_in_cold": float(sea_level.temperature()), "T_in_hot": 330.0}]
    hot_day = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
    halo = {**geometry, "n_wide_cold": 600.0, "n_long_cold": 15.0, "n_tall": 40.0,
            "air": fluid(air_properties(hot_day)), "coolant": fluid(water_glycol_properties()),
            "mdot_cold": 8.0, "mdot_hot": 3.0, "T_in_cold": float(hot_day.temperature()), "T_in_hot": 273.15 + 60.0}
    cases.append({"name": "halo", **halo})
    cases += [{**halo, "name": f"halo_air_{m:g}kg_s", "mdot_cold": float(m)} for m in (2, 4, 6, 10, 12)]
    cases += [{**halo, "name": f"halo_long_{n}", "n_long_cold": float(n)} for n in (5, 10, 20, 25)]
    return cases


def run_openconcept(cases):
    """OpenConcept's HXGroup outputs for each case (OpenConcept environment only)."""
    from importlib.metadata import version
    import openmdao.api as om
    from openconcept.thermal.heat_exchanger import HXGroup

    units = {"channel_width_cold": "m", "channel_height_cold": "m", "fin_length_cold": "m", "channel_width_hot": "m",
             "channel_height_hot": "m", "fin_length_hot": "m", "fin_thickness": "m", "plate_thickness": "m",
             "case_thickness": "m", "material_k": "W/m/K", "material_rho": "kg/m**3"}
    results = []
    for case in cases:
        prob = om.Problem()
        prob.model.add_subsystem("hx", HXGroup(num_nodes=1), promotes=["*"])
        prob.setup()
        for name, unit in units.items():
            prob.set_val(name, case[name], units=unit)
        for count in ("n_wide_cold", "n_long_cold", "n_tall"):
            prob.set_val(f"ac|propulsion|thermal|hx|{count}", case[count])
        for side, fluid in (("cold", case["air"]), ("hot", case["coolant"])):
            prob.set_val(f"cp_{side}", fluid["cp"], units="J/kg/K")
            prob.set_val(f"k_{side}", fluid["k"], units="W/m/K")
            prob.set_val(f"mu_{side}", fluid["mu"], units="kg/m/s")
            prob.set_val(f"rho_{side}", fluid["rho"], units="kg/m**3")
            prob.set_val(f"mdot_{side}", case[f"mdot_{side}"], units="kg/s")
            prob.set_val(f"T_in_{side}", case[f"T_in_{side}"], units="K")
        prob.run_model()
        outputs = {name: float(prob.get_val(name)[0]) for name in quantities.values()}
        results.append({"inputs": case, "openconcept": outputs})
    return {"source": f"openconcept {version('openconcept')} (PyPI), openmdao {version('openmdao')}, "
                      "thermal/heat_exchanger.py HXGroup, num_nodes=1",
            "date": date.today().isoformat(), "plan": "docs/decisions/050-ntu-openconcept-check.md",
            "cases": results}


def ours(case, hydraulic_diameter_coolant="manglik_bergles"):
    """Our core's quantities for one stored case. `hydraulic_diameter_coolant="duct"` replaces the coolant-side
    hydraulic diameter by OpenConcept's 2 w h / (w + h), to isolate that one difference."""
    from dataclasses import replace

    from aircraft_closure.powertrain.components.compact_heat_exchanger import FluidProperties, OffsetStripFinCore

    class Core(OffsetStripFinCore):
        def passage_coolant(self):
            passage = super().passage_coolant()
            if hydraulic_diameter_coolant == "duct":
                w, h = self.width_channel_coolant_m, self.height_channel_coolant_m
                passage = replace(passage, diameter_hydraulic_m=2 * w * h / (w + h))
            return passage

    def fluid(props):
        return FluidProperties(specific_heat_J_kg_K=props["cp"], conductivity_W_m_K=props["k"],
                               viscosity_Pa_s=props["mu"], density_kg_m3=props["rho"])

    core = Core(count_wide=case["n_wide_cold"], count_long=case["n_long_cold"], count_tall=case["n_tall"],
                width_channel_air_m=case["channel_width_cold"], height_channel_air_m=case["channel_height_cold"],
                length_fin_air_m=case["fin_length_cold"], width_channel_coolant_m=case["channel_width_hot"],
                height_channel_coolant_m=case["channel_height_hot"], length_fin_coolant_m=case["fin_length_hot"],
                thickness_fin_m=case["fin_thickness"], thickness_plate_m=case["plate_thickness"],
                thickness_case_m=case["case_thickness"], density_material_kg_m3=case["material_rho"],
                conductivity_material_W_m_K=case["material_k"], density_coolant_kg_m3=case["coolant"]["rho"])
    air, coolant = core.passage_air(), core.passage_coolant()
    result = core.evaluate(case["mdot_cold"], case["mdot_hot"], case["T_in_cold"], case["T_in_hot"],
                           fluid(case["air"]), fluid(case["coolant"]))
    mass_coolant_kg = coolant.area_flow_m2 * core.width_m() * core.density_coolant_kg_m3
    values = {
        "width_m": core.width_m(), "height_m": core.height_m(), "length_m": core.length_m(),
        "diameter_hydraulic_air_m": air.diameter_hydraulic_m, "diameter_hydraulic_coolant_m": coolant.diameter_hydraulic_m,
        "area_flow_air_m2": air.area_flow_m2, "area_flow_coolant_m2": coolant.area_flow_m2,
        "area_heat_air_m2": air.area_heat_m2, "area_heat_coolant_m2": coolant.area_heat_m2,
        "reynolds_air": result.air.reynolds, "reynolds_coolant": result.coolant.reynolds,
        "colburn_air": result.air.colburn, "colburn_coolant": result.coolant.colburn,
        "fanning_air": result.air.fanning, "fanning_coolant": result.coolant.fanning,
        "coefficient_air_W_m2_K": result.air.coefficient_W_m2_K,
        "coefficient_coolant_W_m2_K": result.coolant.coefficient_W_m2_K,
        "efficiency_surface_air": result.air.efficiency_surface,
        "efficiency_surface_coolant": result.coolant.efficiency_surface,
        "conductance_W_K": result.conductance_W_K, "ntu": result.ntu, "effectiveness": result.effectiveness,
        "power_heat_W": result.power_heat_W, "temperature_air_out_K": result.temperature_air_out_K,
        "temperature_coolant_out_K": result.temperature_coolant_out_K,
        "pressure_drop_air_Pa": result.air.pressure_drop_Pa, "pressure_drop_coolant_Pa": result.coolant.pressure_drop_Pa,
        "mass_material_kg": core.get_mass() - mass_coolant_kg,
    }
    return {name: float(value) for name, value in values.items()}


def relative_differences(entry, hydraulic_diameter_coolant="manglik_bergles"):
    """(ours - OpenConcept) / |OpenConcept| per quantity; pressure drops by magnitude (sign conventions differ);
    outlet temperatures as the difference over the inlet-to-outlet change."""
    case, reference = entry["inputs"], entry["openconcept"]
    mine = ours(case, hydraulic_diameter_coolant)
    differences = {}
    for name, oc_name in quantities.items():
        value, ref = mine[name], reference[oc_name]
        if name.startswith("pressure_drop"):
            ref = -ref
        if name.startswith("temperature"):
            inlet = case["T_in_cold"] if "air" in name else case["T_in_hot"]
            differences[name] = (value - ref) / abs(ref - inlet)
        else:
            differences[name] = (value - ref) / abs(ref)
    return mine, differences


def compare(path=stored_path):
    data = json.loads(Path(path).read_text())
    print(f"OpenConcept reference: {data['source']}, {data['date']}")
    entries = data["cases"]
    names = [e["inputs"]["name"] for e in entries]
    tables = {variant: [relative_differences(e, variant)[1] for e in entries] for variant in ("manglik_bergles", "duct")}
    for variant, label in (("manglik_bergles", "our core as is"),
                           ("duct", "our core with OpenConcept's coolant-side d_h = 2wh/(w+h)")):
        print(f"\nRelative difference, % ({label}); temperatures: % of the temperature change")
        print(f"{'quantity':30s}" + "".join(f"{n[-12:]:>13s}" for n in names) + f"{'max |d|':>10s}")
        for name in quantities:
            row = [100 * t[name] for t in tables[variant]]
            print(f"{name:30s}" + "".join(f"{x:13.4f}" for x in row) + f"{max(abs(x) for x in row):10.4f}")
    print("\nKey values (ours / OpenConcept):")
    for entry in entries:
        mine, _ = relative_differences(entry)
        ref = entry["openconcept"]
        print(f"  {entry['inputs']['name']:22s} Q {mine['power_heat_W'] / 1e3:7.2f} / {ref['heat_transfer'] / 1e3:7.2f} kW, "
              f"eps {mine['effectiveness']:.4f} / {ref['effectiveness']:.4f}, "
              f"dp air {mine['pressure_drop_air_Pa']:7.1f} / {-ref['delta_p_cold']:7.1f} Pa, "
              f"dp coolant {mine['pressure_drop_coolant_Pa'] / 1e3:7.2f} / {-ref['delta_p_hot'] / 1e3:7.2f} kPa, "
              f"mass {mine['mass_material_kg']:6.2f} / {ref['component_weight']:6.2f} kg")


if __name__ == "__main__":
    command = sys.argv[1] if len(sys.argv) > 1 else "compare"
    if command == "cases":
        Path(sys.argv[2]).write_text(json.dumps(build_cases(), indent=1))
    elif command == "openconcept":
        try:
            import openconcept  # noqa: F401
        except ImportError:
            sys.exit("openconcept is not installed in this environment; skipping (stored results remain in "
                     f"{stored_path})")
        Path(sys.argv[3]).write_text(json.dumps(run_openconcept(json.loads(Path(sys.argv[2]).read_text())), indent=1))
    else:
        compare(sys.argv[2] if len(sys.argv) > 2 else stored_path)
