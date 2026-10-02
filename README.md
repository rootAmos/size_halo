# Halo-inspired aircraft closure

AeroSandbox/CasADi framework for an unmanned hybrid-electric tiltrotor sizing
and mission-performance project. Implements foundation, standalone
powertrain components and typed powertrain topology, compatibility margins and vehicle mass closure
and low-fidelity aerodynamics
(Tiers 0–5), following the
supplied bootstrap. Aircraft sizing and
mission performance are planned subsequent tiers, not implemented yet.

## Environment and execution

With uv installed, `uv sync` creates `.venv` and installs the project and its
dependencies from `pyproject.toml`. Python 3.13 is selected by `.python-version`.
Use `uv sync --locked` to reproduce the checked-in lockfile.
The default dev group includes PyYAML for validating local skill files;
`uv sync --no-dev` installs only runtime dependencies.

```powershell
uv sync
uv run python -m unittest discover -s tests -v
uv run python examples/series_hybrid_point.py
uv run python examples/series_hybrid_point_explicit.py
uv run python -m examples.aircraft_mass_closure
uv run python -m examples.cruise_closure
uv run python -m examples.tail_sizing
uv run python -m examples.requirements_sizing
uv run python -m aircraft_closure.powertrain.components.propulsor
```

Alternatively, use standard Python tooling:

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -e .
.venv/Scripts/python -m unittest discover -s tests -v
.venv/Scripts/python examples/series_hybrid_point.py
```

The six components are Motor, Generator, Battery, SimpleTurboshaft, Gearbox
and ActuatorDiskPropulsor. They return named results and expose limits; callers
own explicit Opti equations. A Tier 2 `Topology` describes how components are
wired (typed ports, an electrical bus, rotor multiplicity) and returns connection
residuals the caller constrains; it never creates variables or solves. Tier 3
margins express ratings and adjacent-component compatibility as normalized
quantities (>= 0 compatible) that callers constrain or report. Tier 4 vehicle
components own geometry and empirical masses; the take-off mass closes through
one explicit Opti equality, with the CG placed by solving for wing position. Tier 5
adds linear lift, parasite buildup and induced drag, and a cruise equilibrium
inside the closure. Both
examples couple a series-hybrid point at one illustrative hover condition with a
20% battery contribution: one by hand, one through the topology.

## Verification notebooks

Each tier has its own executed notebook (outputs kept so plots render remotely):

| Tier | Notebook |
|---|---|
| 0 | [Foundation](notebooks/tier0_foundation/foundation_verification.ipynb): environment, layout, governance, skills, conventions |
| 1 | [Powertrain components](notebooks/tier1_powertrain_components/powertrain_verification.ipynb): identities, limits, trends, symbolic use, coupled point; [McDonald machine losses](notebooks/tier1_powertrain_components/motor_loss_model_verification.ipynb) |
| 2 | [Powertrain topology](notebooks/tier2_powertrain_topology/topology_verification.ipynb): ports, wiring rules, residuals, Tier 1 reproduction, multiplicity |
| 3 | [Compatibility margins](notebooks/tier3_compatibility_margins/compatibility_verification.ipynb): envelopes, operating and design margins, binding limits, rating sizing |
| 4 | [Vehicle mass closure](notebooks/tier4_vehicle_mass_closure/mass_closure_verification.ipynb): geometry, Raymer masses, CG, one-solve closure, payload growth |
| 5 | [Aerodynamics](notebooks/tier5_aerodynamics/aerodynamics_verification.ipynb): lift, parasite buildup, polar, AeroBuildup comparison, cruise closure |
| 6 | [Stability and control](notebooks/tier6_stability_control/stability_control_verification.ipynb): neutral point, trim, Cn_beta, failed-rotor rudder, tail sizing |
| 7 | [Requirements](notebooks/tier7_requirements/requirements_verification.ipynb): flight points, power curves, requirement feasibility, powertrain sizing |

```powershell
uv sync --group notebooks
uv run jupyter lab notebooks
```

Read [architecture](docs/ARCHITECTURE.md), [interfaces](docs/MODEL_INTERFACES.md),
[roadmap](docs/FIDELITY_ROADMAP.md), and [reference assumptions](docs/HALO_REFERENCE.md).
All defaults are illustrative, not Archer specifications. Next: Tier 6 stability and
control, then vehicle/mass, aero, controls,
requirements, independently tested mission segments and coupled closure.
