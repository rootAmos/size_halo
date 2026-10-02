# Conventional-tail stability, trim and failed-propulsor yaw control

Status: COMPLETED 2026-10-01 — part of the user's "go all the way" request (2026-10-01);
review delegated, decisions self-reviewed against AGENTS.md and the
control-surface-sizing skill.

## Goal and scope

Tier 6 for a conventional horizontal and vertical tail: longitudinal static
stability (neutral point, static margin), pitch trim with elevator, tail trim
drag through the Tier 5 drag-increment hook, directional stability (Cn_beta)
and failed-propulsor asymmetric yaw with rudder sizing. A reference example
sizes both tail areas and the wing position inside the mass closure. Out of
scope: V-tail (deferred by architecture), dynamic stability, lateral (roll)
control, hover control by rotor thrust/tilt, CG envelopes over a mission.

## References

Raymer, Aircraft Design 5th ed., ch. 16 (eq. 16.25 fuselage pitching moment
and Fig. 16.14 K_fus; eq. 16.47 fuselage yawing moment); Etkin and Reid /
Nelson for neutral point, downwash gradient 2 CL_alpha / (pi AR) and the
thin-airfoil flap effectiveness tau = 1 - (theta - sin theta)/pi,
theta = arccos(2 c_f/c - 1); AeroSandbox `CL_over_Cl` for tail lift slopes.

## Physics (per radian unless stated)

- Surface aerodynamic centres at the quarter MAC (unswept surfaces).
- Downwash: eps = 2 CL_w / (pi AR); d eps / d alpha = 2 CL_alpha_w / (pi AR).
- Fuselage: Cm_alpha_fus = K_fus W_f^2 L_f / (c S) per degree.
- Neutral point: x_np = [CL_aw x_acw + eta_h V_h' CL_ah (1 - d eps/d alpha) x_ach]
  / CL_alpha_total - Cm_alpha_fus c / CL_alpha_total, V_h' = S_h / S.
- Pitching moment about the CG: Cm = Cm_ac_w + CL_w (x_cg - x_acw)/c
  - eta_h (S_h/S) CL_h (x_ach - x_cg)/c + Cm_alpha_fus alpha;
  CL_h = CL_ah (alpha + i_h - eps + tau_e delta_e); total CL = CL_w + eta_h (S_h/S) CL_h.
- Tail trim drag: CL_h^2 / (pi e_h AR_h) eta_h S_h/S as a DragIncrement.
- Directional: Cn_beta = eta_v CL_av S_v l_v / (S b) - 1.3 Vol_fus / (S b).
- Failed outboard propulsor: N = (T + D_failed) y; rudder:
  delta_r = N / (q S b eta_v CL_av tau_r S_v l_v / (S b)).

## Interfaces

```
vehicle/surfaces.py  HorizontalTail.elevator_chord_fraction (0.3),
                     VerticalTail.rudder_chord_fraction (0.3)
controls/stability.py
    flap_effectiveness(chord_fraction, correction=0.8)
    LongitudinalStability(tail_dynamic_pressure_ratio=0.9, cm_ac_wing=-0.09,
                          fuselage_pitch_factor_per_deg=0.012, tail_incidence_deg=0.0)
        .neutral_point_x_m(aircraft, aerodynamics, velocity_m_s, altitude_m)
        .static_margin(aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m)
        .evaluate(aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m,
                  alpha_deg, elevator_deg) -> LongitudinalResult
    DirectionalStability(vertical_tail_dynamic_pressure_ratio=0.9)
        .cn_beta_per_rad(aircraft, aerodynamics, x_cg_m, velocity_m_s, altitude_m)
        .rudder_for_yaw_moment_deg(aircraft, aerodynamics, x_cg_m, velocity_m_s,
                                   altitude_m, yaw_moment_Nm)
    failed_propulsor_yaw_moment_Nm(thrust_per_propulsor_N, lateral_arm_m,
                                   drag_failed_propulsor_N=0)
examples/tail_sizing.py
```

## Variables and constraints (reference example)

Variables: MTOM, wing LE position, S_h, S_v, cruise alpha and elevator. Equalities:
mass closure, trimmed lift = weight, Cm = 0. Inequalities: static margin >=
0.10, |elevator| <= 15 deg in cruise trim, Cn_beta >= 0.06 /rad, rudder for
outboard-rotor failure at 1.2 V_stall <= 20 deg, stall bound. Objective:
minimum MTOM. Margins expressed with Tier 3 `margin_above/below` and reported.

## Tests

Neutral point identities (no tail -> wing ac shifted by fuselage term; tail
moves it aft; SM sign); Cm_alpha = -SM CL_alpha_total (numeric derivative);
trim solution satisfies Cm = 0; flap effectiveness limits (0 -> 0, 1 -> 1);
trim drag positive; Cn_beta grows with S_v and arm; rudder deflection linear in
yaw moment, inverse in S_v and q; symbolic tail areas; example binding
constraints.

## Reference cases

1. Reference aircraft: neutral point, static margin at the Tier 4 CG.
2. Tail sizing: minimum-MTOM tails meeting SM, trim, Cn_beta and OEI rudder.

## Implementation sequence and acceptance

Module + tests; example + integration tests; Tier 6 notebook; docs; commit and
push. Acceptance: tests and all notebooks pass; constraints and binding margins
reported; no hidden solve in controls code.

## Progress and decisions

- 2026-10-01: Plan written. Rotor lateral positions are not represented by
  symmetric multiplicity, so the failed-propulsor arm is an explicit input
  (outboard rotor at the wing tip in the example). Flap effectiveness uses thin
  airfoil theory times a 0.8 viscous correction (stated, not calibrated).

## Deferred

V-tail mixing, roll control and aileron sizing, dynamic modes, hover and
transition control, CG travel envelopes, control-power margins for gusts.
