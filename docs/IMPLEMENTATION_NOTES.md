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

## Next stage

Create a separate Tier 2 plan for typed mechanical/electrical/fuel ports,
connections, multiplicity and a series-hybrid assembly. Keep that graph limited
to topology; expose cyclic voltage/current and shaft coupling through Opti.
Then add compatibility margins before vehicle/mass, low-fidelity aero, controls,
requirements and mission segments. The bootstrap explicitly defers complete
aircraft sizing and mission closure beyond this first implementation.
