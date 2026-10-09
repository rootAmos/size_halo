# Code walkthroughs

One notebook per module group. Every listing is printed from the repository with its real line numbers
(`show_source` in `tutorials/tutorial_helpers.py`), so the walkthroughs cannot drift from the code; every claim is
backed by a `check(...)` that runs on the sized v3.6 aircraft. All 16 notebooks execute; 730 checks pass.

| Notebook | Modules (every line) |
|---|---|
| [C01](C01_core.ipynb) | `core/ports.py`, `core/topology.py`, `core/margins.py` |
| [C02](C02_machines.ipynb) | `powertrain/components/motor.py`, `generator.py`; machine units in `halo_sizing.py` |
| [C03](C03_battery.ipynb) | `battery.py`, `battery_ecm.py`, `powertrain/cells.py`; how the flight point and mission call the pack |
| [C04](C04_turboshaft_gearbox.ipynb) | `turboshaft.py`, `powertrain/decks.py`, `gearbox.py`; XV-15 lapse fits; engine and gearbox masses |
| [C05](C05_rotor.ipynb) | `rotor.py`, `propulsor.py`, `examples/jvx_rotor_calibration.py` |
| [C06](C06_electrical_thermal_components.ipynb) | `converters.py`, `cable.py`, `protection.py`, `thermal.py`, `heat_exchanger.py`, `thermal/heat.py` |
| [C07](C07_powertrain_assembly.ipynb) | `powertrain/ports.py`, `topologies.py`, `compatibility.py`, `redundancy.py`, `machine_database.py`, `vehicle/powertrain_installation.py` |
| [C08](C08_aerodynamics.ipynb) | `aerodynamics/` (simple, Scholz, AeroBuildup, slipstream, trim, download) |
| [C09](C09_vehicle_weights.ipynb) | `vehicle/`, `weights/afdd.py`, `build_halo_aircraft`, XV-15 calibration |
| [C10](C10_controls.ipynb) | `controls/stability.py`, `examples/tail_sizing.py` |
| [C11](C11_flight_point.ipynb) | `performance/flight_point.py`, line by line |
| [C12](C12_mission_requirements.ipynb) | `mission/` (segments, mission, cost), `requirements/capability.py` |
| [C13](C13_halo_sizing.ipynb) | `examples/halo_sizing.py`, line by line, with an x-ray of the solved problem (sizes, bounds, multipliers) |
| [C14](C14_trajectory_corridor.ipynb) | `trajectory/tiltrotor.py`, `corridor.py`, and the trajectory examples |
| [C15](C15_export_openvsp.ipynb) | `export/openvsp/` and the structure and aero cross-check examples |
| [C16](C16_validation.ipynb) | XV-15, hot day, tiltrotor wing, OEW uncertainty and design-space examples |

## Findings from the walkthroughs

Found while verifying the code line by line. Nothing in the model was changed; each finding is documented with
evidence in the notebook named.

**Affect results**

| Finding | Where | Notebook |
|---|---|---|
| The battery end-of-interval voltage uses OCV and R0 at the *mid*-interval SOC. At the binding engine-out point it reads 525.0 V; at the end SOC it would be about 508 V, so the constraint that sizes the pack is about 3 % optimistic. | `battery_ecm.py:298` | C03 |
| Structural loads use the cruise dynamic pressure, not a dive speed. Faster cruise adds structure weight, which pulls the sized cruise speed down (185 kt fuel-optimal on the frozen aircraft vs 165 kt sized). | `vehicle/condition.py`, `halo_sizing.py:1252-1255` | C09, C12 |
| Adjacent-rating design margins (`compatibility.design_margins`) are never applied. On the sized aircraft, 2 lanes × 528 kW feed a 757 kW gearbox rating (margin −0.39). Operating margins hold at every flight point. | `halo_sizing.py:1269` | C07 |
| Pylon inertia counts one lane motor while the tip mass counts both, so the wing is about 8 kg light. | `halo_sizing.py:795-796` vs `:773` | C09 |
| With `BuildupAerodynamics`, `LongitudinalStability.evaluate` counts tail lift twice (whole-aircraft CL plus tail lift), about 10 % too much lift. This affects the conversion corridor trim. It does not affect the sizing's static margin or Cn_beta. | `controls/stability.py:91,102` | C10 |
| The cost block counts one motor per rotor instead of rotors × lanes: about +2.4 % acquisition and +$23 per mission. | `halo_sizing.py:1310,1316` | C12, C13 |
| The trajectory and corridor examples default to `HaloAssumptions()` with relaxed machine counts, so they fly 711 kW motors instead of the sized 528 kW. | `examples/trajectory_optimization.py:44`, `halo_conversion_corridor.py:29` | C14 |
| The OpenVSP export draws the v3.2 geometry, not v3.6. | `export/openvsp/snapshot.py:225-260` | C15 |
| The design-space sensitivity tool skips the whole-unit round-up, and two of its default inputs do nothing today. | `examples/halo_design_space.py` | C16 |

**Reporting and conditioning**

| Finding | Where | Notebook |
|---|---|---|
| `connection_residuals` is never applied in the sizing; connections hold by construction, and in failure states the residuals would be wrong (installed vs active counts). | `flight_point.py` | C01, C11 |
| Rotor limits (CT/σ, tip Mach, λ, tip speed) are plain constraints, not margins, so they never appear in `binding` although they bind. | `flight_point.py:221-225,395` | C11 |
| `SegmentResult.point` is the first sub-point, so the reported cruise battery share, L/D and CL are first-quarter values. | `mission/mission.py:137` | C12 |
| Duplicate constraints that would become degenerate if active: motor speed (also in `operating_margins`); `soc_end` and `soc_after_engine_out` (also in the mission). | `flight_point.py:392`, `halo_sizing.py:1272-1273` | C11, C13 |
| `ceil(n - 1e-3)` does not cover the smooth-max overshoot (up to 0.035 units). | `halo_sizing.py:1030` | C02 |
| The hot-day SOC is computed by energy, not coulomb counting (about 2 % off; slack today). | `halo_sizing.py:1245-1246` | C03, C13 |
| Requirement points are evaluated at a fixed SOC of 0.8, a default nobody sets. | `requirements/capability.py` | C12 |

**Documentation and dead code:** listed per notebook (stale docstrings, `unit_margins` that does not exist, unused
imports and functions, plan names that do not exist).
