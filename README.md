# Halo-inspired aircraft closure

AeroSandbox/CasADi framework for sizing an unmanned series-hybrid-electric
tiltrotor together with its mission. Every discipline contributes equations to
one `asb.Opti` problem; there are no hidden convergence loops or second
solvers. Tiers 0–9 and 10a of the [fidelity roadmap](docs/FIDELITY_ROADMAP.md)
are implemented. Tier 9 is a coupled problem that sizes the aircraft,
optimizes its mission and allocates battery versus turbogenerator energy per
segment. Tier 10a checks the mass models against the Bell XV-15, the published
twin tiltrotor now used as the Halo-class reference.

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

## Environment and execution

With uv installed, `uv sync` creates `.venv` and installs the project from
`pyproject.toml` (Python 3.13 via `.python-version`; `uv sync --locked`
reproduces the lockfile). Examples import each other, so run them as modules
from the repository root.

```powershell
uv sync
uv run python -m unittest discover -s tests
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

```powershell
uv sync --group notebooks
uv run jupyter lab notebooks
```

## Status and next step

Tier 10b is next: turboshaft altitude lapse and a hover and cruise power
check against the XV-15. Tier 10c follows: a two-rotor Halo-class aircraft
sized to XV-15-derived requirements.

At XV-15 scale, the uncalibrated mass models close 13 % light. The Raymer GA
wing, fuselage and flight-control equations are the weak groups; the AFDD
rotorcraft equations track well. Per-group calibration factors are available,
and Tier 10c will document which of them it applies.
