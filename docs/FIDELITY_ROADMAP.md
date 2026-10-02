# Fidelity roadmap

| Tier | Deliverable | Status |
|---|---|---|
| 0 | Governance, package, interfaces, tests | Implemented |
| 1 | Six standalone simple powertrain components | Implemented |
| 2 | Typed ports, connections, multiplicity, series-hybrid topology | Implemented |
| 3 | Speed/torque/voltage/current/power compatibility margins | Implemented |
| 4 | Vehicle geometry, CG and empirical mass closure | Implemented |
| 5 | Linear lift, parasite and induced drag, extension hooks | Implemented |
| 6 | Conventional tails, trim, stability, asymmetric thrust | Implemented |
| 7 | Payload, hover, climb, speed, ceiling constraints | Implemented |
| 8 | Operating point then isolated segment then prescribed mission | Implemented |
| 9 | Coupled aircraft closure and energy allocation | Implemented |
| 10a | AFDD tiltrotor weights, XV-15 group-weight validation and calibration | Implemented |
| 10b | Turboshaft lapse and part-power submodels, hover download, XV-15 hover-power check | Implemented |
| 10c | Two-rotor Halo-class series hybrid sized to XV-15-derived requirements (13,000 ft ceiling) | Next |
| 11 | Maps, engine decks, BEM, surrogates, advanced missions | Planned |

Keep simple implementations when higher fidelity is introduced. Use AeroSandbox
geometry, aero, weights and dynamics wherever suitable. Do not jump to a full
trajectory optimization before independent segment verification.
