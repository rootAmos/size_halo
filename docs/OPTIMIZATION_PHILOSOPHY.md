# Optimization ownership

Components return equations. AeroSandbox Opti owns variables, constraints,
objectives and solving. Put coupling states such as induced velocity or bus
current explicitly in the caller. Equality residuals enforce physics; inequality
constraints enforce feasible operating domains and hardware limits. No model
starts its own solver. See examples/series_hybrid_point_explicit.py (every
equation by hand) and examples/series_hybrid_point.py (connection equations
from the Tier 2 topology). The topology describes connectivity and multiplicity
only; `connection_residuals` returns expressions the caller constrains to zero. Mechanical ports
carry speed/torque, electrical ports voltage/current and fuel ports fuel flow.
Cyclic networks are allowed and resolved through explicit Opti equalities.

## Starting points (Tier 22, plan 029)

A coupled sizing problem is solved once from one initial guess. When the guess
matters, the caller tries an explicit, finite, ordered list of candidate
starts, each an independent solve of the same problem, and records which one
converged (`solve_halo_sizing_multistart`, `StartRecord`). A candidate may
first solve a simpler precursor problem (other aerodynamics, other battery,
lower payload, a mass objective) only to obtain its guess. This is not a
convergence loop: no quantity is iterated to a fixed point, and every coupling
stays inside the one Opti problem. `select="best"` runs every candidate and
reports the spread between local optima.
