---
name: structural-mass-modeling
description: Implement empirical structural mass buildup for this aircraft closure repository.
---

Use AeroSandbox weight correlations where suitable. Return component masses
without mutating aircraft state. Document correlation scope, SI conversions,
and calibration factors. Structures estimates mass only; stress, deflection,
beam sizing and FEM require a scope change. Read docs/ARCHITECTURE.md and the
active plan; test scaling, units, and symbolic compatibility.
