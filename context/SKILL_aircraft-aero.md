---
name: aircraft-aero
description: >
  Use when implementing or modifying aerodynamic models, aerodynamic buildup,
  low-fidelity CL/CD models, control-effect hooks, parasite drag, propeller
  interference, AeroSandbox aero integration, or CFD-derived surrogate interfaces.
---

# Aircraft Aerodynamics

Start with the minimum useful model:

- linear lift
- parasite drag
- induced drag

Example conceptual form:

CL = CL0 + CL_alpha * alpha

CD = CD0 + CL^2 / (pi * e * AR)

Leave clear extension points for:

- horizontal-tail contribution
- vertical-tail contribution
- control-surface increments
- DATCOM-style parasite drag buildup
- trim drag
- propeller scrubbing/interference
- propwash effects

Before implementing higher fidelity, inspect AeroSandbox.

Prefer native AeroSandbox capability when it already solves the problem.

Possible aero sources:

1. Simple analytic model
2. AeroSandbox native analysis
3. Airfoil/polar-driven AeroSandbox analysis
4. CFD-derived polar / lookup / surrogate

Do not run expensive CFD directly inside the sizing optimization unless explicitly required.
