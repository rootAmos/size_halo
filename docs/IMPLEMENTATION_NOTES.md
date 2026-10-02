# Foundation and simple powertrain implementation notes

## Delivered scope

Tier 0 supplies pyproject metadata, Python 3.13 selection, a uv dependency lock,
six repository-local skills, engineering governance, interfaces and a staged
roadmap. `uv sync` creates the editable local environment automatically. Runtime
dependency: AeroSandbox; dev dependency: PyYAML for skill validation. Tests use
standard-library unittest. The verified environment resolves AeroSandbox 4.2.10
and CasADi 3.8.1. Source compatibility is declared for Python >=3.10; execution
has been verified on Windows with Python 3.13.3 only.

Tier 1 supplies six independently usable components with immutable result
dataclasses, ratings, SI units, specific-power/energy mass estimates and manual
module examples. Motor and generator share a replaceable quadratic loss model.
Battery accounting conserves chemical and terminal energy including Joule losses.
The rotor accepts thrust directly or exposes inverse-operation power residuals
for caller-owned Opti coupling. No hidden optimizer or convergence loop exists.

## Physics decisions and limits

The native AeroSandbox propeller power helper inspected in version 4.2.8 divides
by axial airspeed, so it is singular at hover. The equivalent momentum equation
implemented here is valid at hover and checked against the native helper in
forward flight. It excludes vortex ring, windmilling and tilt-transition effects.
The positive induced-velocity root is rationalized to avoid cancellation at low
thrust and high axial speed. The exact zero-load point is handled explicitly.

Default numerical parameters illustrate engineering interfaces. They are not
measurements or inferred proprietary Halo specifications. Generator ratings are
shaft-input ratings; motor ratings are shaft-output ratings. All hardware limits
and valid operating domains remain caller-owned constraints. Engine efficiency
is constant and has no idle flow or atmospheric lapse. No thermal subsystem,
rotor blade mass correlation, installed-mass allowances or failure model exists.

## Verification

Thirty tests cover conservation identities, zero and ideal limits, mass scaling,
charge/discharge signs, physical trends, array evaluation, native AeroSandbox
agreement and symbolic component evaluations/size variables. Three integration
tests use actual Opti solves. The example specifies 5000 N hover thrust and a 20%
battery fraction of motor electrical input, obtaining approximately 89.286 kW
rotor shaft power, 18.653 kW battery terminal power and 0.005852 kg/s fuel flow.
Power residuals are reported and tested explicitly. All six manual examples
execute, and all six skill files pass the bundled skill validator.

CasADi 3.8.1 emits a NumPy compatibility FutureWarning through the installed
AeroSandbox stack during symbolic evaluation. Tests pass with its default mode;
the project does not change process-wide CasADi settings to silence it. Matplotlib
may try to cache fonts outside a sandbox; verification used MPLCONFIGDIR pointing
to the ignored workspace `.mpl-cache` directory when needed.

## Tier 2: typed ports and topology

`core/` holds generic ports, `Topology`, electrical buses and
`connection_residuals`; `powertrain/ports.py` declares component ports in a
dispatch table so the Tier 1 component interface is unchanged;
`powertrain/topologies.py` builds the series hybrid. The topology creates no
Opti variables, constraints or solves: the caller supplies port values and
applies the labelled residuals. `examples/series_hybrid_point.py` is the
topology-coupled point; the Tier 1 hand-written coupling is kept as
`examples/series_hybrid_point_explicit.py`.

The topology example adds explicit coupling variables at the connections
(turboshaft torque, rotor speed and torque, bus voltage) so shaft and bus
residuals are active rather than trivially satisfied. With one rotor it matches
the explicit Tier 1 result to better than 1e-6 relative (89.286 kW rotor shaft,
18.653 kW battery, 0.005852 kg/s fuel). With four symmetric rotor strings on one
bus, the battery is modeled as four reference packs in parallel (resistance / 4)
so bus voltage and per-motor current match the single case; total motor
current and battery power are exactly 4x, while fuel flow exceeds 4x because the
single generator's quadratic torque loss is unscaled. This is an illustrative
scaling choice, not a sizing rule.

Limits: no splitters, combiners, multi-input gearboxes, inverters, thermal or
fuel-tank ports; direct connections need equal counts; multiplicity is
symmetric, so failed-propulsor cases cannot yet be represented. Verification:
58 unittest cases (28 new) and the Tier 2 notebook (71 checks).

## Tier 3: compatibility margins

Operating margins replace the hand-written rating list in
`examples/series_hybrid_point.py`; the solution is unchanged (ratings are
inactive at 5000 N) and the report names motor shaft power as most critical
(7.95 % headroom). `build_point_problem` exposes the coupled point so callers
choose the objective: maximizing thrust shows the motor rating binding below
100 kW and the 100 kW gearbox input rating capping thrust above it (about
5284 N per rotor). Design margins of the default parts record the 150 kW
turboshaft overdriving the 100 kW generator as -0.5 without resizing; an Opti
using design margins as constraints sizes the minimum generator at 150 kW.
Verification: 77 unittest cases (19 new) and the Tier 3 notebook (47 checks).
Not modeled: current limits on machines, SOC and thermal margins, failure-case
margins, gear-transformed speed/torque envelopes.

## Verification notebooks

One executed notebook per tier under `notebooks/`: Tier 0 foundation checks,
Tier 1 component physics, Tier 2 topology and Tier 3 compatibility margins. Outputs are kept so plots render
remotely.

## Next stage

Tier 4: vehicle geometry, CG and empirical mass closure, then low-fidelity
aero, controls, requirements and mission segments.
