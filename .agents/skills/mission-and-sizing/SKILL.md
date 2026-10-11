---
name: mission-and-sizing
description: Build staged mission analysis and explicit AeroSandbox aircraft sizing formulations in this repository.
---

Read the active plan and docs/OPTIMIZATION_PHILOSOPHY.md. Keep capability
requirements separate from the flown mission. Advance through operating
point, isolated segment, prescribed mission, semi-free mission and optimized
trajectory. Test segments independently. Prefer AeroSandbox atmosphere,
OperatingPoint and dynamics. Keep Opti variables, mass/energy equalities,
component limits and objectives readable at orchestration level. Define
fuel/SOC reserve treatment and energy-management policy explicitly.
