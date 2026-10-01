# Optimization ownership

Components return equations. AeroSandbox Opti owns variables, constraints,
objectives and solving. Put coupling states such as induced velocity or bus
current explicitly in the caller. Equality residuals enforce physics; inequality
constraints enforce feasible operating domains and hardware limits. No model
starts its own solver. See examples/series_hybrid_point.py for a reference.
The eventual graph describes topology and multiplicity only. Mechanical ports
carry speed/torque, electrical ports voltage/current and fuel ports fuel flow.
Cyclic networks are allowed and resolved through explicit Opti equalities.
