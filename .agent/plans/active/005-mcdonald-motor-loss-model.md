# McDonald parametric electric-machine loss model

Status: COMPLETED 2026-10-01 — requested by the user on 2026-10-01 ("use this paper for the
loss model of the emotor"); user delegated review, so decisions below are
self-reviewed against AGENTS.md and the powertrain skill.

## Goal and scope

Powertrain fidelity change within Tier 1 components. Add the loss model of
McDonald, "Modeling of Electric Motor Driven Propellers for Conceptual Aircraft
Design", AIAA 2015-1676 (eqs. 1-4), as an interchangeable submodel behind the
unchanged machine interface `evaluate(speed_rad_s, torque_Nm, voltage_V)`, and
make it the default for `Motor` and `Generator`. Keep `SimpleMotorLossModel`
(AGENTS rule 10). Out of scope: inverter models, thermal derating, maps (Tier
10), voltage-dependent losses.

## References

Paper: `C:\Users\alexa\Documents\References\motors\6.2015-1676-propeller_motor_efficiency_scaling.pdf`
(local, not redistributed). Larminie and Lowry loss buildup (paper ref. 3).

## Physics

P_L = C0 + C1 w + C2 w^3 + C3 Q^2, with peak efficiency eta_hat at (w_hat, Q_hat)
and parasite loss ratio k0 in [0, 1]:

    C0 = k0 w_hat Q_hat (1 - eta_hat) / (6 eta_hat)
    C1 = -3 C0 / (2 w_hat) + Q_hat (1 - eta_hat) / (4 eta_hat)
    C2 = C0 / (2 w_hat^3) + Q_hat (1 - eta_hat) / (4 eta_hat w_hat^2)
    C3 = w_hat (1 - eta_hat) / (2 Q_hat eta_hat)

Ratings (eq. 4): Q_rated = kQ Q_hat, P_rated = kP w_hat Q_hat,
w_limit = kw w_hat. C1 simplifies to Q_hat (1 - eta_hat)(1 - k0) / (4 eta_hat)
>= 0 for k0 <= 1, so all coefficients are non-negative.

## Assumptions and decisions

- Motor and generator share the loss function (as in Tier 1); efficiency
  definitions differ by direction. C0 is a constant loss present even at
  standstill (documented domain: w >= 0, Q >= 0).
- Defaults: w_hat = 400 rad/s, Q_hat = 200 Nm (the reference hover operating
  point), eta_hat = 0.96, k0 = 0.5. Existing explicit ratings (100 kW, 500 Nm,
  1000 rad/s) stay constructor fields; `rubber_machine(...)` builds a machine
  from the seven paper parameters when ratings should follow eq. 4.
- Loss parameters may be symbolic, enabling rubber scaling as a design
  variable later (Tier 9).
- The reference topology scales the shared generator as a rubber machine
  (Q_hat x n), so generator losses, and fuel flow, scale exactly with n.

## Interfaces

```
powertrain/components/motor.py
    McDonaldLossCoefficients(constant_W, linear_W_s_rad, cubic_W_s3_rad3, torque_W_Nm2)
    McDonaldMotorLossModel(speed_peak_efficiency_rad_s, torque_peak_efficiency_Nm,
                           efficiency_peak, parasite_loss_ratio)
        .coefficients(), .evaluate(speed_rad_s, torque_Nm, voltage_V)
    rubber_machine(machine_type, speed_peak_efficiency_rad_s, torque_peak_efficiency_Nm,
                   efficiency_peak, parasite_loss_ratio, torque_ratio, power_ratio,
                   speed_ratio, **machine_kwargs)
```

## Tests

Peak efficiency equals eta_hat at (w_hat, Q_hat); the peak is stationary (zero
gradient of efficiency) and a maximum; k0 = 0 removes C0; non-negative
coefficients for k0 in [0, 1]; rating ratios; scale invariance (doubling Q_hat
doubles all losses at doubled torque); symbolic parameters; Simple model still
available and unchanged.

## Reference cases

Paper Figure 1 instance (eta_hat 95 %, k0 0.5, kQ = kP = kw = 2): efficiency at
the normalized optimum is 0.95 and drops toward the rated corners.

## Implementation sequence and acceptance

1. Loss model, coefficients, rubber_machine; tests.
2. Switch defaults; update reference numbers in tests, examples and docs.
3. Tier 1 notebook keeps verifying the simple model explicitly; new notebook
   `notebooks/tier1_powertrain_components/motor_loss_model_verification.ipynb`.
4. All tests and all notebooks pass; commit and push.

## Progress and decisions

- 2026-10-01: Plan written and self-reviewed.
- Figure 1 contour positions are not machine-readable from the PDF text, so the
  notebook reproduces the instance and checks model-guaranteed properties
  (peak value and location, corners below peak) without claiming a figure match.
- k0 has no effect at the peak speed (speed terms total 2 Q_hat w_hat (1 -
  eta)/(4 eta)); the parasite-ratio trend is checked off the peak speed.
- Tier 1 notebook keeps verifying the simple model by constructing it
  explicitly; its no-load generator check now works for either loss model.
- Reference values: battery 19.184 kW, fuel 0.006207 kg/s; 4-rotor fuel exactly
  4x. Verification: 112 tests; motor notebook 32/32; Tier 0-4 notebooks pass.

## Deferred

Loss maps from manufacturer data (Tier 10), inverter/voltage effects,
thermal limits, regeneration.
