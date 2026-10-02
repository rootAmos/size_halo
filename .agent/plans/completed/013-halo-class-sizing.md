# Halo-class two-rotor series-hybrid sizing

Status: COMPLETED 2026-10-02. The user approved the requirement set on 2026-10-02 ("yes"):

- 13,000 ft ceiling (the user's number);
- about 900 kg payload;
- 445 nm range (the XV-15's);
- OGE hover at a modest altitude;
- about 250 kt cruise;
- XV-15 weight calibration, with the manned-research items removed.

## Goal and scope

Tier 10c. One explicit `asb.Opti`, in the Tier 9 style, sizes a two-rotor,
two-turbogenerator series-hybrid tiltrotor to an XV-15-class requirement set.
It optimizes the mission and the battery/generator energy split at the same
time, and uses:

- the Tier 10a calibrated weights;
- the Tier 10b engine lapse, part-power efficiency and hover download;
- the AeroSandbox turboshaft mass and efficiency regressions.

In scope:

1. **`build_series_hybrid(..., count_turbogenerators=1)`:** n identical
   turboshaft-generator pairs on the bus.
2. **Flight points:**
   - `FlightCondition.active_generator_count` (None means all);
   - the generator power balance and fuel flow scale by the active count.
3. **`ActuatorDiskPropulsor`:**
   - `coefficient_of_performance_airplane` (None means the same as hover),
     used by airplane-mode flight points;
   - `speed_tip_max_m_s` (None means no bound), which flight points enforce
     on rotor speed;
   - `radius_m()`.
4. **`examples/halo_sizing.py`:** the Halo-class reference aircraft builder
   and `solve_halo_sizing(...)`.
5. The Tier 10c notebook and a dashboard switched to the Halo-class aircraft.

Out of scope:

- conversion (transition) flight;
- airplane-mode power validation;
- compressibility above Mach 0.3, noted as a limitation (cruise is Mach
  0.39);
- cost;
- multiple missions.

## Requirements (SI in code)

| Item | Requirement |
|---|---|
| Payload | 900 kg |
| Hover | OGE at 4,000 ft ISA, T/W 1.05, 7 % download; battery may assist |
| Engine-out hover | 60 s at sea level, one turbogenerator plus battery, from the 0.30 SOC floor to at least 0.10 |
| Max speed | 250 kt (128.6 m/s) at 10,000 ft on turbogenerators only |
| Climb | 7.5 m/s at 80 m/s and 1,000 m, turbogenerators only |
| Ceiling | 13,000 ft, 0.5 m/s residual climb at 80 m/s, turbogenerators only |
| Stall | airplane-mode stall at or below 120 kt at sea level and MTOM, CLmax 1.5 |

The mission is 445 nm ground distance. In order:

1. 60 s take-off hover;
2. climb to 10,000 ft at 6 m/s and 80 m/s;
3. cruise (speed free, at most 128.6 m/s);
4. 20 min reserve loiter at 10,000 ft (speed free);
5. descent at 5 m/s and 80 m/s;
6. 60 s landing hover.

Fuel carried is 1.1 x burnt; landing SOC is at least 0.30.

## Configuration and models

- **Geometry:** two rotors at the wing tips, sharing one x station (hover
  trim puts the CG under it). The wing aspect ratio is fixed at 6.12 (XV-15;
  short, stiff tiltrotor wing). Wing area is variable, and the rotor radius
  is tied to span: R <= (b - fuselage width) / 2 - 0.3 m clearance (XV-15
  layout).
- **Fuselage:** 12.8 m by 1.68 m (XV-15).
- **Tails:** areas variable; static margin >= 0.10 and Cn_beta >= 0.06/rad,
  as in Tier 9.
- **Rotor masses:**
  - rotor group: AFDD82 blades plus hub at 740 ft/s tip speed, solidity
    0.089, 3 blades, nu 1.55, times the XV-15 calibration factor;
  - gearboxes: AFDD83, from motor rating and rotor speed, transmission
    factor;
  - engine section: AFDD82 nacelles for the turboshafts, powerplant factor.
- **Airframe masses:** Raymer GA with the XV-15 factors (wing, tails,
  fuselage, gear, flight controls); load factor 4.5 ultimate (XV-15).
- **Unmanned adjustment:** no crew items or ejection seats. Fixed equipment
  is the XV-15's electrical plus instrumentation (487 lb) plus 100 lb for
  autonomy avionics. Heating and air conditioning, and furnishings, are
  dropped. The airframe factors stay as calibrated: that is conservative,
  because the XV-15's crash load factors are inside them.
- **Turboshafts:** mass is a design variable with the explicit equality
  `power_turboshaft(m) = P_rated` (AeroSandbox regression). Full-power
  thermal efficiency comes from `thermal_efficiency_turboshaft(m)`. Lapse
  exponent 0.797 and the part-power knockdown are on (Tier 10b).
- **Rotors:**
  - hover figure of merit 0.67 (XV-15 calibrated, the baseline);
  - airplane-mode coefficient 0.87 (assumed; this gives about 0.85
    propulsive efficiency at cruise; sensitivity is reported);
  - tip speed at most 740 ft/s;
  - gearbox ratio 7 (keeps the motor near its peak-efficiency speed in
    hover).
- **Aerodynamics:** `SimpleAerodynamics` with miscellaneous drag area
  0.8 m2 (assumed: tip nacelles, spinners, gear fairings) and 7 % hover
  download.

## Design variables

- MTOM and fuel load;
- wing position and area;
- both tail areas;
- motor and generator peak torques;
- turboshaft rated power and mass (per engine);
- battery power and energy;
- rotor disk area per rotor.

Mission variables: cruise and loiter speed, the energy split of every
segment, and the per-point states.

## Constraints

- mass closure, with the structural condition fed by the solved cruise L/D
  and speed;
- hover trim;
- the requirement flight points, engine-out hover and SOC chain;
- the mission (fuel and SOC);
- stall wing loading;
- the span and rotor clearance geometry;
- static margin and Cn_beta;
- the turboshaft mass-power equality;
- every operating margin >= 0.

The failed-rotor rudder check is dropped: with two rotors, a rotor loss is
not survivable and motors are assumed dual-wound (stated, deferred).

## Objective

Minimum MTOM. Sensitivities: hover figure of merit 0.75, airplane-mode
coefficient 0.80, and payload 600 and 1,200 kg.

## Symbolic considerations

Rotor radius is sqrt(A / pi), so the AFDD masses are smooth in A. The
turboshaft mass-power regression uses `np.blend` (smooth). The active
generator count is a Python int set per point, not symbolic. Equalities are
normalized as in Tier 9.

## Tests

- Topology with two turbogenerators: counts, ports, mass doubles.
- Flight point:
  - with one of two generators active, the generator carries the whole
    turbogenerator share and fuel flow halves at the same power split;
  - the airplane-mode coefficient changes cruise power, not hover;
  - the tip-speed bound holds at the solution.
- Defaults unchanged (the existing suite).
- Halo sizing:
  - constraints satisfied;
  - closure fixed point;
  - turboshaft mass-power equality;
  - rotor radius within the span geometry;
  - MTOM in the 1x,xxx lb class (5,000–9,000 kg, a loose regression band);
  - a better hover figure of merit lowers MTOM;
  - more payload raises MTOM.

## Acceptance

Tests and notebooks pass. The Tier 10c notebook reports the sized aircraft,
the binding constraints, the weight breakdown, the mission and the
sensitivities. The dashboard shows the Halo-class aircraft. Docs, commit,
push.

## Progress and decisions

- 2026-10-02: Plan written.
- 2026-10-02: Implemented; it converged first time from the cold guess.
  - Take-off 8,500 kg (18,740 lb), empty 6,347 kg, fuel 1,253 kg.
  - Binding: max speed (turbogenerators), hover (motors, gearboxes, rotors),
    engine-out hover (battery power), stall (wing), static margin and
    Cn_beta (tails), SOC floors.
  - Sensitivities: FM 0.75 gives -6.9 %; cruise coefficient 0.80 gives
    +6.4 %; payload 600 / 1,200 kg gives 16,741 / 20,763 lb; uncalibrated
    weights give 13,537 lb.
  - Decision: the coefficient-0.80 and uncalibrated cases end in a local
    infeasibility from the cold guess, but nearby values solve. Added
    `initial=` (warm start) instead of retuning the defaults.
  - 230 tests and 14 notebook checks pass.
  - The dashboard artifact was switched to the Halo-class design.

## Deferred

- conversion corridor;
- compressibility drag;
- airplane-mode power validation;
- dual-wound motor and rotor-loss modelling;
- multiple missions;
- cost.
