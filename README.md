# Halo-inspired aircraft closure

[![tests](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml/badge.svg)](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml)

AeroSandbox/CasADi framework for sizing an unmanned series-hybrid-electric
tiltrotor together with its mission. Every discipline contributes equations to
one `asb.Opti` problem; there are no hidden convergence loops or second
solvers. Tiers 0–21 of the [fidelity roadmap](docs/FIDELITY_ROADMAP.md) are
implemented apart from Tiers 15, 18 and 22, and Tier 23 (geometry) is partial. Tier 9 is a coupled problem that sizes the aircraft,
optimizes its mission and allocates battery versus turbogenerator energy per
segment. Tiers 10a and 10b check the mass models and the engine and hover models against
the Bell XV-15. Tier 10c uses them to size a Halo-class two-rotor series
hybrid: 18,740 lb take-off for 900 kg payload, 445 nm, 250 kt and a 13,000 ft
ceiling (18,506 lb with the user-supplied turboshaft deck's part-power fuel curve,
Tier 11a).

**Current reference (plan 035): 13,038 lb at take-off** with 900 kg of payload at 210 kt.
- **Engines:** fixed at 2 x 1,120 hp, inside the fuselage.
- **Battery:** a Samsung 50G-shaped equivalent-circuit battery.
- **Wing:** the NDARC tiltrotor wing with whirl-flutter margins.
- **Aerodynamics:** AeroSandbox AeroBuildup, with Scholz miscellaneous items.
- **Thermal:** a thermal model with a ram-air heat exchanger.
- **Fuselage:** an unpressurized boxy fuselage (11 m long, 1.68 x 2.0 m). Its weight is raw Raymer GA x 1.70,
  anchored to a layout-based structure estimate.

The reference has moved as fidelity was added; see the [results](docs/RESULTS.md) for how and why.

All numbers are illustrative engineering inputs, not Archer or Halo data
(see [reference assumptions](docs/HALO_REFERENCE.md)).

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
| Export | `export/openvsp/` | Numeric geometry snapshot of a solved aircraft; OpenVSP outer mold line (tilting nacelles, Modes), STEP/STL export, PyVista renders. Optional; never imported by sizing code |

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
uv run python -m examples.halo_openvsp            # Plan 031: Halo in OpenVSP (needs OpenVSP, see below)
uv run python -m examples.halo_aero_compare       # Plan 031: VSPAERO and OpenVSP parasite drag vs AeroSandbox
```

### Optional: OpenVSP geometry export

The OpenVSP Python API ships with the OpenVSP release, not on PyPI. Install it
into `.venv` from a release built for Python 3.13 (3.53.1 is used here), and
the PyVista renderer from the `geometry` group:

```powershell
$vsp = "<OpenVSP-3.53.1-win64>\python"
uv pip install --system-certs "$vsp\openvsp_config" "$vsp\utilities" "$vsp\degen_geom" "$vsp\vsp_airfoils" "$vsp\openvsp"
uv sync --inexact --group geometry
```

These packages are outside the lockfile, so a plain `uv sync` removes them;
use `uv sync --inexact`. Tests that need OpenVSP skip when it is absent.
Outputs go to `output/` (not committed).

## Geometry, aero cross-check and structure tools (plan 031)

These are optional, they need OpenVSP, and nothing in the sizing depends on them. They take a solved aircraft (as
plain numbers) and check it from a different direction.

| Tool | What it does | Run |
|---|---|---|
| Outer mold line | The drawn Halo in OpenVSP: smooth bodies, a tapered wing, a V-tail, tip nacelles that tilt about the spindle, rotors. Writes `.vsp3` with hover, conversion and cruise Modes, STEP and STL per nacelle angle, and renders | `python -m examples.halo_openvsp` |
| Aero cross-check | VSPAERO (vortex lattice and panel) and the OpenVSP parasite-drag build-up against AeroSandbox VLM and AeroBuildup on the same geometry. Compares lift slope, neutral point, induced and profile drag | `python -m examples.halo_aero_compare` |
| Internal structure | Wing box (spars, ribs), fuselage (ring frames, bulkheads, floor) and V-tail as OpenVSP FEA structures. Writes CalculiX and Nastran decks, STL and a mass report, and renders the layout | `python -m examples.halo_structure` |
| Wing FE check | Runs CalculiX on the OpenVSP wing-box mesh with AFDD-mapped gauges and rigid nacelles: free-free beam, chord and torsion frequencies vs AFDD, and the ultimate jump take-off strain | `python -m examples.halo_wing_fe` (needs CalculiX, `CCX`) |
| Weight back-check | Sizes the primary-structure gauges on the drawn layout from simple ultimate loads and the AFDD stiffness requirements, then compares with the AFDD wing and Raymer tail and fuselage | `python -m examples.halo_structure_reference`, then `python -m examples.halo_structure_check` |

Main findings so far:
- The AFDD wing agrees with the layout to within about 10 %.
- AeroBuildup is conservative on stability and induced drag compared with VSPAERO.
- The XV-15 fuselage calibration overstated an uncrewed fuselage, which led to plan 032.
- **The CalculiX wing check finds the AFDD wing optimistic for this layout** (the caps work over a shorter lever
  arm in the real box):
  - beam frequency is 0.70x AFDD's and torsion 0.91x;
  - the jump take-off strain is 1.29x the allowable at the plan 032 reference;
  - the whirl-flutter margin is therefore probably not met.

Limits:
- Fuselage FE meshing takes minutes per file type.
- The structural STEP export is off.
- No FE solution is run yet.

See [implementation notes](docs/IMPLEMENTATION_NOTES.md) (plans 031 and 032) for numbers and OpenVSP quirks.

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

Open items, roughly by impact:

- **Airplane-mode rotor efficiency.** The cruise coefficient is the largest remaining assumption.
- **Plan 032 assumptions:**
  - the turbogenerator station inside the fuselage;
  - the XV-15 pylon radius of gyration, now without engines in the nacelle;
  - Raymer's flight-control mass scaling with fuselage length, which gives 104 kg of the 11 m saving and is weak
    for fly-by-wire.
- **V-tail.** The sizing still uses a conventional tail; the V-tail is drawn only.
- **Symbolic geometry layout (Tier 23).** Clearance and packaging constraints are still planned.
- **AFDD wing vs FE.** The CalculiX check found the AFDD cap lever arm optimistic for this box: about 29 % over
  strain in the jump take-off and torsion about 9 % low.
  - Plan 035 corrects the sizing: caps at the real box depth, a 1 mm minimum gauge, and pylon inertia from the
    tip components.
  - The FE re-check of the corrected wing: beam 0.85x AFDD, jump strain 1.10x the allowable (was 1.29x).
    Torsion identification is unresolved.
- **FE models.** Only the wing box is checked. The fuselage and tail decks carry placeholder gauges.

[Results](docs/RESULTS.md) has the full list and how the reference moved.
