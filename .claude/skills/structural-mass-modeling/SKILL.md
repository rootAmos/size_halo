---
name: structural-mass-modeling
description: Implement empirical structural mass buildup for this aircraft closure repository.
---

Use AeroSandbox weight correlations where suitable. Return component masses
without mutating aircraft state. Document correlation scope, SI conversions,
and calibration factors. Mass in the sizing is handbook-based; FE checks
(CalculiX/Nastran export), strength margins and whirl flutter each need their
own plan before they become sizing constraints. Read docs/ARCHITECTURE.md and
the active plan; test scaling, units, and symbolic compatibility.
