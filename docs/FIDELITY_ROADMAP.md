# Fidelity roadmap

| Tier | Deliverable | Status |
|---|---|---|
| 0 | Governance, package, interfaces, tests | Implemented |
| 1 | Six standalone simple powertrain components | Implemented |
| 2 | Typed ports, connections, multiplicity, series-hybrid topology | Next |
| 3 | Speed/torque/voltage/current/power compatibility margins | Deferred |
| 4 | Vehicle geometry, CG and empirical mass closure | Deferred |
| 5 | Linear lift, parasite and induced drag, extension hooks | Deferred |
| 6 | Conventional tails, trim, stability, asymmetric thrust | Deferred |
| 7 | Payload, hover, climb, speed, ceiling constraints | Deferred |
| 8 | Operating point then isolated segment then prescribed mission | Deferred |
| 9 | Coupled aircraft closure and energy allocation | Deferred |
| 10 | Maps, engine decks, BEM, surrogates, advanced missions | Deferred |

Keep simple implementations when higher fidelity is introduced. Use AeroSandbox
geometry, aero, weights and dynamics wherever suitable. Do not jump to a full
trajectory optimization before independent segment verification.
