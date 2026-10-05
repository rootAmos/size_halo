# Trim from the tail load, and trajectory states for the current reference

Status: COMPLETED 2026-10-04.

The user asked to "knock out" two open items, with the goal of a closed
architecture to explain to an external party:

- trim drag from the actual tail load;
- trajectory states, so trajectories fly the current reference.

Trim direction (user): "Just apply an effectiveness factor plus take the
cosine of the tail angle". References supplied:

- Scholz, *Aircraft Design* ch. 11 (empennage sizing);
- Scholz ch. 7 (wing design);
- Brelje et al. 2019 (OpenConcept thermal management).

## Goal and scope

1. **Trim drag (`aerodynamics/trim.py`, `TailTrim`).**
   - AeroBuildup computes moments about the CG (the wing quarter root
     chord, where the hover trim places it).
   - The tail-lift increment that zeroes the moment uses the effectiveness
     factor η_H = 0.9 and cos(tail dihedral).
   - Wing downwash at the tail, which AeroBuildup omits, is added as a
     moment correction: ε = 1.75 C_L / (π A) (Scholz eq. 11.29).
   - Trim drag = tail induced drag for the increment + the wing's
     induced-drag change. This replaces the flat 2 % on AeroBuildup; the
     Scholz model keeps the 2 %.
2. **Trajectory states (`trajectory/tiltrotor.py`).**
   - A temperature state for each thermal-modelled motor, generator and the
     pack: dT/dt = (loss − (T − T_c)/R)/C by collocation, using the same R
     and C as sizing, with the limit applied at every node.
   - Continuous power ratings are dropped for thermal machines, giving
     short-time ratings as in sizing.
   - Exchanger heat with a rating margin. Pumping power is paid as cooling
     drag in proportion w = q/(q + Δp) and by fans for the rest.
   - Lane motors share the rotor torque (Tier 18).
   - The generator step-up gearbox efficiency is on the engine side.
   - Unit-built machines need nothing new: their limits are the machine's
     own.

Not in scope:

- failure states along trajectories;
- the electrical layer in the trajectory model (it is not the default).

## References

- Scholz, ch. 11: eqs. 11.6, 11.12–11.29; η_H 0.85–0.95, typically 0.9.
- Brelje, Jasa, Martins, Gladin, "Development of a Conceptual-Level Thermal
  Management System Design Capability in OpenConcept", 2019. Ram-air duct
  and heat-exchanger modelling and the Meredith effect; the exchanger model
  is consistent, and thrust recovery is not credited.

## Assumptions

- Tail Oswald factor 0.8.
- Downwash bracket 1.75 (Scholz, typical geometry).
- CG at the wing quarter root chord.

## Interfaces

- `BuildupAerodynamics.trim`.
- `HaloAssumptions.efficiency_tail` and `angle_dihedral_tail_deg`.
- `build_tiltrotor_trajectory(..., temperature_start_C=None)`.
- `TiltrotorTrajectory.thermal` (`TrajectoryThermal`).
- `TrajectoryResult.temperatures_C`, `drag_cooling_N` and `power_fan_W`.

## Symbolic considerations

- Everything is explicit expressions or collocated states.
- The cooling blend is smooth.

## Tests

- `tests/aerodynamics/test_tail_trim.py`:
  - zero moment gives zero tail lift and zero drag;
  - the tail lift balances the moment;
  - cos Γ and η scaling;
  - the downwash relief sign.
- `tests/trajectory/test_trajectory_thermal.py` (fast settings: Scholz aero,
  thermal on, single lane):
  - temperatures are states starting at the coolant temperature and stay
    within limits;
  - fans in hover, ram drag at speed.

## Acceptance

- Targeted tests pass.
- No notebook re-runs (user).

## Progress and decisions

- **Trim drag on the reference geometry at cruise speed:**

| CL | Trim drag (% of CD) |
|---|---|
| 0.21 | −0.1 % |
| 0.40 | 0.5 % |
| 0.58 | 1.8 % |
| 0.76 | 3.4 % |

  Without the downwash correction it was about 7 % at CL 0.58, because the
  tail saw the free-stream angle of attack.
- **Trajectories on the 16,231 lb reference** (4 lane motors, thermal,
  cooling):
  - minimum-energy transition: 22.5 s, 10.5 kWh;
  - minimum time to climb: 225 s, SOC 0.95 → 0.73.

## Deferred

- Failure states and the electrical layer along trajectories.
- Elevator deflection and incidence (they do not change trim drag at this
  level).
