# Halo-inspired aircraft closure

[![tests](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml/badge.svg)](https://github.com/rootAmos/size_halo/actions/workflows/tests.yml)

AeroSandbox/CasADi framework for sizing an unmanned series-hybrid-electric
tiltrotor together with its mission. Every discipline contributes equations to
one `asb.Opti` problem; there are no hidden convergence loops or second
solvers. Tiers 0–10c of the [fidelity roadmap](docs/FIDELITY_ROADMAP.md) are
implemented. Tier 9 is a coupled problem that sizes the aircraft,
optimizes its mission and allocates battery versus turbogenerator energy per
segment. Tiers 10a and 10b check the mass models and the engine and hover models against
the Bell XV-15. Tier 10c uses them to size a Halo-class two-rotor series
hybrid: 18,740 lb take-off for 900 kg payload, 445 nm, 250 kt and a 13,000 ft
ceiling (18,506 lb with the user-supplied turboshaft deck's part-power fuel curve,
Tier 11a). The current reference (plan 026) fixes the engines at 2 x 1,120 hp
and flies 210 kt with 900 kg of payload. It uses a Samsung 50G-shaped
equivalent-circuit battery and the NDARC tiltrotor wing with whirl-flutter
margins, and weighs 14,247 lb at take-off.

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

The largest remaining assumption is airplane-mode rotor efficiency (the cruise
coefficient). Compressibility drag, conversion flight and rotor-loss handling
are also open. These are Tier 11 candidates.

At XV-15 scale, the uncalibrated mass models close 13 % light. The Raymer GA
wing, fuselage and flight-control equations are the weak groups; the AFDD
rotorcraft equations track well. Per-group calibration factors are available,
and Tier 10c will document which of them it applies.
