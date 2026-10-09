# Glossary

Terms and symbols used in the tutorials and the code. The tutorial that introduces each term is in brackets.

## Optimization and formulation

| Term | Meaning |
|---|---|
| **all-at-once / SAND** | simultaneous analysis and design: coupling and state variables are optimizer variables, discipline residuals are equality constraints, one NLP solves analysis and design together [00, 03] |
| **MDF** | multidisciplinary feasible: an inner MDA converges the disciplines at every optimizer iterate (a weight loop is a tiny MDA) [03] |
| **IDF** | individual discipline feasible: copies of coupling variables with consistency constraints [00] |
| **binding** | a margin that is zero at the optimum (the drivers use \|margin\| < 1e-5, or 1e-4 for the Halo); it sizes something [04] |
| **multiplier / dual** | Lagrange multiplier of a constraint: change of the optimal objective per unit relaxation; with objectives in tonnes and normalized margins, tonnes of MTOM per 100 % of a limit [02, 03] |
| **growth factor** | dMTOM/dpayload: take-off mass added per kilogram added, once everything resizes (about 2.5 for the Halo) [03, 20] |
| **margin** | (limit − value)/limit (`margin_below`) or (value − limit)/limit (`margin_above`); ≥ 0 is satisfied [04] |
| **scale** | `opti.variable(scale=s)` returns s × (the variable IPOPT sees); defaults to \|init_guess\|, or 1 for a zero guess [02] |
| **softmax** | smooth maximum s·ln(e^(a/s) + e^(b/s)); overestimates max(a, b) by at most s·ln 2 [01] |
| **precursor problem** | a simpler version of the sizing (other battery, Scholz aero, no thermal...) solved to get a starting point [20] |
| **warm start** | solving from an earlier solution (`initial=`) [02, 20] |
| **square system** | as many equations as unknowns: a point with a prescribed energy split [10] |
| **closure** | MTOM equals the sum of the parts, written `mass_takeoff_kg / total.mass == 1` [12] |

## Code vocabulary

| Term | Meaning |
|---|---|
| **component** | a frozen dataclass with `get_mass()`, `get_limits()`, `evaluate(...)`; never creates variables, constrains, iterates or clips [04] |
| **submodel** | a field filled by any object with the right method (`loss_model`, `mass_model`, `thermal_model`, `part_power_model`, `lapse_model`) [04] |
| **rubber machine** | a McDonald machine whose ratings follow its peak-efficiency point: Q_max = 2.5 Q̂, P_rated = 1.25 ω̂Q̂, ω_max = 2.5 ω̂ [05] |
| **port / port value** | typed connection point (domain, direction) / its state (speed and torque, voltage and current, fuel flow) [11] |
| **topology** | network description: instances with counts, buses, connections; no variables or solving [11] |
| **combiner / splitter** | a direct connection between different counts (`combine=True`): efforts equal, total flow conserved [11] |
| **degraded state** | active lanes, strings and failed buses at a flight point (a condition input, not a topology) [11] |
| **flight condition / flight point** | the description of an operating point / its variables, equations, results and margins (`build_flight_point`) [15] |
| **requirement** | a capability point at maximum weight (hover, climb, speed, ceiling, failure) [16] |
| **segment / sub-segment** | a mission leg mapped to a flight condition and a duration / one of its equal-duration points [17] |
| **reference / named set** | the current default design / an earlier reference kept reproducible (`assumptions_plan026`...) [19] |
| **plan NNN** | a decision record in `docs/decisions/` [04] |

## Physics symbols

| Symbol | Meaning |
|---|---|
| MTOM, m_TO | maximum (design) take-off mass |
| h, `hybridization_electric` | battery share of the bus demand at a point (negative: generators recharge the pack) [10, 15] |
| SOC, s | state of charge [07] |
| V_oc, OCV | open-circuit voltage; V* = OCV less the current-independent RC part; R_eff = effective resistance over an interval [07] |
| R0, R1, R2, τ1, τ2 | equivalent-circuit series resistance and RC branches [07] |
| ω̂, Q̂, η̂, k0 | McDonald peak-efficiency speed, torque, efficiency and parasite loss ratio [05] |
| C_T, C_P, σ, C_T/σ | rotor thrust and power coefficients, solidity, blade loading [08] |
| λ, `advance_ratio` | V / (ΩR), axial advance (inflow) ratio [08] |
| FM, η | figure of merit (hover), propulsive efficiency (airplane mode) [08] |
| DL/T | hover download fraction [13] |
| σ (engine) | density ratio ρ/ρ_SL in the turboshaft lapse σⁿ (T/T_ISA)^−m [06] |
| C_L, C_D, C_D0, e | lift, drag, parasite drag coefficients; Oswald factor [13] |
| SM, C_nβ | static margin (fraction of MAC), directional stability derivative [14] |
| PDIV | partial-discharge inception voltage [09] |
