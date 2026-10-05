# Halo-inspired aircraft closure

[![tests](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml/badge.svg)](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml)

AeroSandbox/CasADi framework for sizing an unmanned series-hybrid-electric
tiltrotor together with its mission. Every discipline contributes equations to
one `asb.Opti` problem; there are no hidden convergence loops or second
solvers. The disciplines are configuration, powertrain, aerodynamics, mass,
stability, mission, energy management, thermal and redundancy. Fidelity is
added in tiers, each with a plan, tests and an executed verification notebook.
The [fidelity roadmap](docs/FIDELITY_ROADMAP.md) lists the tiers. Models are
validated against the Bell XV-15 and the full-scale JVX proprotor test.

**Current reference (plan 035):** a Halo-class two-rotor series hybrid.

- **Mission:** 900 kg payload, 445 nm, 210 kt at 10,000 ft, 13,000 ft
  ceiling, hot-day hover, engine-out and electrical failure hovers.
- **Engines:** two fixed off-the-shelf 1,120 hp turboshafts.
- **Battery:** Samsung 50G-shaped equivalent-circuit pack.
- **Wing:** NDARC tiltrotor wing with whirl-flutter margins.
- **Aerodynamics:** AeroSandbox AeroBuildup with Scholz drag corrections and trim drag from the tail load.
- **Thermal:** heat exchanger sized with the aircraft.
- **Machines and drive:**
  - redundant motors (2 lanes per rotor), 2 cross-strapped buses, 2 battery
    strings;
  - machines built from whole units of real products (Evolito-class motors,
    Helix-class generators);
  - a single-stage 5:1 rotor gearbox.
- **Result:** 7,395 kg (16,303 lb) take-off weight.

**For reviewers:**
- [docs/RESULTS.md](docs/RESULTS.md): what the framework concludes, how it is
  checked, and its limits.
- [docs/ARCHITECTURE_DIAGRAMS.md](docs/ARCHITECTURE_DIAGRAMS.md): how the
  framework is organized.

All numbers are illustrative engineering inputs, not Archer or Halo data
(see [reference assumptions](docs/HALO_REFERENCE.md)).

## Results at a glance

Figures from the executed tier notebooks (`notebooks/`) and the geometry
export, copied to `docs/figures/`.

**How the answer moved as fidelity was added.** Each bar is the reference
aircraft re-solved with that tier's models. The jump at plan 035 comes from
three changes: whole real machine units in place of idealized ("rubber")
scaling, electrical redundancy, and drag corrections for excrescence and
trim.

![Take-off weight by fidelity step](docs/figures/fidelity_progression.png)

**Geometry.** The OpenVSP outer mold line at three nacelle angles, and the
structural layout used for the mass check. These are rendered from an
earlier sized design (`examples/halo_openvsp.py`, `examples/halo_structure.py`).

![Halo-class 3-view and nacelle conversion](docs/figures/halo_views.png)
![Structural layout](docs/figures/halo_structure.png)

**Aerodynamics.**
- **Drag polar (Tier 21):** the simple model, AeroSandbox AeroBuildup and the
  Scholz hand build-up compared. The old guessed drag area was most of the
  drag.
- **Cross-check on the same geometry:** OpenVSP VSPAERO (VLM and panel)
  against AeroSandbox for lift, pitching moment, induced drag, and profile
  drag by component.

![Drag polar by model](docs/figures/drag_polar_models.png)
![OpenVSP vs AeroSandbox](docs/figures/aero_compare_openvsp.png)

**Validation against test data.**
- **Rotor power model (Tier 12):** against full-scale JVX hover and
  airplane-mode data. The D-2 hover data were held out of the fit.
- **XV-15 wing weight (Tier 20):** the NDARC tiltrotor wing by component,
  from the published wing modes.

![JVX proprotor calibration](docs/figures/rotor_jvx_calibration.png)
![XV-15 wing components](docs/figures/xv15_wing_components.png)

**Battery and thermal.**
- **Battery cell (Tier 17):** the 50G open-circuit voltage fit to the
  digitized cell data.
- **Thermal (Tier 19):** lumped machine temperature response, and the
  short-time rating it permits from a cold start.

![Battery OCV fit](docs/figures/battery_ocv_fit.png)
![Thermal short-time rating](docs/figures/thermal_short_time_rating.png)

**Trajectory optimization (Tier 14).** Minimum-energy transition and
minimum time to climb, flown inside the conversion corridor, against a
naive prescribed conversion. The minimum-energy transition uses about half
the energy of the naive schedule.

![Trajectories in the conversion corridor](docs/figures/trajectory_conversion_corridor.png)

**Sensitivities (Tier 22).** Take-off mass change for ±15 % on each
uncertain input. The XV-15 weight-calibration factors dominate.

![Sensitivity tornado](docs/figures/sensitivity_tornado.png)

## What is in the package

| Layer | Package | Contents |
|---|---|---|
| Core | `core/` | Typed ports, topology with buses and multiplicity, connection residuals, normalized margins |
| Powertrain | `powertrain/` | Motor and generator (McDonald AIAA 2015-1676 losses), battery, turboshaft, gearbox, actuator-disk rotor; port declarations, compatibility margins, series-hybrid builders including rubber sizing |
| Vehicle | `vehicle/` | Wing, tails, fuselage, gear, systems, payload, fuel, nacelles, interconnect shaft, fixed equipment, installed powertrain; Raymer GA and AFDD masses; mass and CG aggregation |
| Aerodynamics | `aerodynamics/` | Linear lift, parasite buildup, induced drag, drag-increment hook |
| Controls | `controls/` | Neutral point, static margin, elevator trim, Cn_beta, rudder for failed-rotor yaw |
| Performance | `performance/` | Quasi-steady flight points coupling aero and the whole powertrain |
| Requirements | `requirements/` | Hover, climb, speed and ceiling capability requirements |
| Mission | `mission/` | Hover, climb, cruise, loiter, descent segments; missions with fuel burn and SOC |
| Weights | `weights/` | AFDD rotorcraft weight equations (rotor, drive system, engine section) from NDARC |

Lower layers never import higher ones; components build equations and callers
own variables, constraints and objectives ([architecture](docs/ARCHITECTURE.md),
[interfaces](docs/MODEL_INTERFACES.md), [implementation notes](docs/IMPLEMENTATION_NOTES.md)).
Diagrams of the layering, fidelity scaling and sizing/trajectory/6-DOF levels are in [architecture diagrams](docs/ARCHITECTURE_DIAGRAMS.md).

## Environment and execution

With uv installed, `uv sync` creates `.venv` and installs the project from
`pyproject.toml` (Python 3.13 via `.python-version`; `uv sync --locked`
reproduces the lockfile). Examples import each other, so run them as modules
from the repository root.

```powershell
uv sync
uv run python -m unittest discover -s tests
uv run python -m examples.halo_sizing             # Tier 10c: Halo-class two-rotor series-hybrid sizing
uv run python -m examples.xv15_performance        # Tier 10b: XV-15 lapse, hover and sfc checks
uv run python -m examples.xv15_reference          # Tier 10a: XV-15 group-weight validation
uv run python -m examples.coupled_sizing          # Tier 9: sizing + mission + energy allocation
uv run python -m examples.mission_analysis        # Tier 8: prescribed and semi-free missions
uv run python -m examples.requirements_sizing     # Tier 7: powertrain sized to requirements
uv run python -m examples.tail_sizing             # Tier 6: stability-driven tail sizing
uv run python -m examples.cruise_closure          # Tier 5: cruise equilibrium in the closure
uv run python -m examples.aircraft_mass_closure   # Tier 4: mass and CG closure
uv run python -m examples.series_hybrid_point     # Tiers 2-3: topology-coupled hover point
uv run python -m examples.series_hybrid_point_explicit  # Tier 1: hand-coupled hover point
```

## Continuous integration

GitHub Actions runs the unit suite on every push and pull request
(`.github/workflows/tests.yml`: `uv sync --locked`, then `unittest`). The
notebooks workflow (`notebooks.yml`) executes every verification notebook on
demand and uploads the executed copies as an artifact. Tests that need
user-supplied local data in `data/` skip when it is absent.

## Verification notebooks

One executed notebook per tier (outputs kept so plots render on GitHub):

| Tier | Notebook |
|---|---|
| 0 | [Foundation](notebooks/tier0_foundation/foundation_verification.ipynb): environment, layout, governance, skills, conventions |
| 1 | [Powertrain components](notebooks/tier1_powertrain_components/powertrain_verification.ipynb) and [McDonald machine losses](notebooks/tier1_powertrain_components/motor_loss_model_verification.ipynb) |
| 2 | [Powertrain topology](notebooks/tier2_powertrain_topology/topology_verification.ipynb): ports, wiring rules, residuals, multiplicity |
| 3 | [Compatibility margins](notebooks/tier3_compatibility_margins/compatibility_verification.ipynb): envelopes, operating and design margins |
| 4 | [Vehicle mass closure](notebooks/tier4_vehicle_mass_closure/mass_closure_verification.ipynb): geometry, Raymer masses, CG, payload growth |
| 5 | [Aerodynamics](notebooks/tier5_aerodynamics/aerodynamics_verification.ipynb): lift, parasite buildup, polar, AeroBuildup comparison |
| 6 | [Stability and control](notebooks/tier6_stability_control/stability_control_verification.ipynb): neutral point, trim, Cn_beta, tail sizing |
| 7 | [Requirements](notebooks/tier7_requirements/requirements_verification.ipynb): flight points, requirement feasibility, powertrain sizing |
| 8 | [Missions](notebooks/tier8_missions/mission_verification.ipynb): segments, prescribed and semi-free missions |
| 9 | [Coupled sizing](notebooks/tier9_coupled_sizing/coupled_sizing_verification.ipynb): simultaneous sizing, mission optimization and energy allocation |
| 10a | [XV-15 mass validation](notebooks/tier10_xv15_reference/xv15_mass_validation.ipynb): AFDD weights, group-by-group comparison, calibration |
| 10b | [XV-15 power validation](notebooks/tier10_xv15_reference/xv15_power_validation.ipynb): engine lapse, hover figure of merit and download, part-power sfc |
| 10c | [Halo-class sizing](notebooks/tier10_halo_sizing/halo_sizing_verification.ipynb): two-rotor series hybrid, binding constraints, mission, sensitivities |

```powershell
uv sync --group notebooks
uv run jupyter lab notebooks
```

## Status and next step

All roadmap tiers are implemented (0–22). Open items, ranked by impact in
[docs/RESULTS.md](docs/RESULTS.md):

- drag calibration against XV-15 flight data;
- weight calibration rests on one complete aircraft;
- the electrical layer is implemented but not the default;
- the trajectory model has no thermal or electrical states, and 6-DOF is
  planned.
