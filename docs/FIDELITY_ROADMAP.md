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
| 10c | Two-rotor Halo-class series hybrid sized to XV-15-derived requirements (13,000 ft ceiling) | Implemented |
| 11a | Turboshaft deck part-power fuel curve (user-supplied GASP deck) | Implemented |
| 11b | Continuous integration: tests on every push; notebook execution on demand | Implemented |
| 12 | Rotor speed physics: induced plus profile power, propeller-mode efficiency in J and tip Mach | Implemented |
| 12b | Fixed off-the-shelf turboshafts (2 x 1,120 hp deck engine), battery-assisted hover, in-flight recharge | Implemented |
| 13 | Electric machines sized by torque; machine speed and gear ratio as design variables | Implemented |
| 14 | Trajectory optimization (AeroSandbox) on a sized aircraft: minimum-energy transition, time to climb | Implemented |
| 15 | Electrical layer: inverters, cables, protection, DC/DC; bus voltage as a discrete choice | Planned |
| 16 | Hot and high: ISA + delta-T atmosphere, temperature lapse, hover at destination after the mission | Implemented |
| 17 | Battery equivalent circuit with sag and ageing: OCV(SOC), R(SOC, C-rate, T), end-of-life, cycle cost | Implemented |
| 18 | Redundancy: lanes per rotor, cross-strapped buses, battery strings, multipoint failure cases | Planned |
| 19 | Thermal: losses to heat-exchanger mass and cooling drag; short-time ratings from thermal mass | Planned |
| 20 | Tiltrotor airframe weights: AFDD wing with torsional stiffness and whirl flutter; second calibration aircraft | Planned |
| 21 | Aero: compressibility drag rise, nacelle build-up, V-tail, download model, conversion segments | Implemented (partial): AeroBuildup model, Scholz hand check, blown wing, geometric download; V-tail and conversion deferred |
| 22 | Design-space practice: freed trades, multistart, cost objective, architecture enumeration, robustness | Planned |

Keep simple implementations when higher fidelity is introduced. Use AeroSandbox
geometry, aero, weights and dynamics wherever suitable. Do not jump to a full
trajectory optimization before independent segment verification.

## Tiers 11b–22 (from the 2026-10-02 external review; reordered 2026-10-03)

An external review of the Tier 10c Halo-class result found the framework
architecture sound. It ranked these gaps by how much they move the answer:

1. rotor speed has no physical effect;
2. machine mass ignores torque;
3. there is no electrical system;
4. there is only one failure case;
5. the battery and bus are idealised;
6. there is no hot day;
7. tiltrotor wing weights are thin;
8. the aero is used outside its validity;
9. the key trades are frozen as inputs;
10. optimization practice is narrow.

The tiers below address them in dependency order. Each tier gets its own
ExecPlan, verification notebook and dashboard refresh, and keeps its simpler
predecessor available.

### 11b Continuous integration

- **Deliverables:**
  - a GitHub Actions workflow that runs `uv sync --locked` and the unittest
    suite on every push;
  - notebook execution as a manual workflow;
  - tests that need local data (`data/engines/`) skip cleanly.
- **Why first:** it costs little, and every later tier changes shared
  interfaces.

### 12 Rotor speed physics (review items 1 and 9)

- **Rotor model:** a rotor class whose power depends on rotor speed. Hover
  power is induced (momentum with a k_i factor) plus profile:
  sigma Cd0 / 8 x (1 + 4.65 mu^2) in CP form, with a drag-divergence rise
  in tip Mach.
- **Propeller mode:** efficiency in advance ratio J and helical tip Mach,
  from the same blade-element-momentum terms (induced + profile + swirl),
  or a differentiable BEM if needed.
- **Design variables:** rotor speed per flight point, with a tip-Mach limit;
  solidity and tip speed become design variables.
- **Calibration and validation data:**
  - **Primary, hover and airplane mode:** C. W. Acree, "Assessment of JVX
    Proprotor Performance Data in Hover and Airplane-Mode Flight
    Conditions", NASA/TM-2016-219070 (user-supplied 2026-10-02),
    https://ntrs.nasa.gov/citations/20160004035. It contains full-scale
    V-22-class proprotor data: OARF hover tests and NFAC 40x80 airplane-mode
    tests at advance ratios up to about 0.56, with measured propulsive
    efficiency. It also gives polynomial hover regressions (Table 4) and the
    full test data in Appendix D (digital text, parseable).
  - **Secondary, hover:** full-scale XV-15 hover test, NASA TM 86833
    (user-supplied; scanned, rotated appendix tables).
  - **Cross-check:** XV-15 power required against airspeed, TM X-62407
    fig. A-12.
  - **Not used for proprotors:** the user's GASP general-aviation propeller
    map stops at J <= 1.6, so it is kept only as a generic propeller map.
- **Result:** "optimum-speed rotor" becomes a real trade, and the fixed
  0.67 / 0.87 coefficients are removed.

### 13 Electric machines sized by torque (review item 2)

- **Distinct power sizing (user, 2026-10-03):** motor power, generator power
  and turbine power are sized separately; they are linked through the
  efficiency chain and the battery share. The constraint diagram shows each
  requirement in one colour, with line style for the component it sizes:
  solid for motor shaft, dashed for generator output, dotted for turbine
  shaft (sea-level equivalent). Installed ratings appear as markers in the
  matching style.

- **Mass model:** machine mass from torque density (N·m/kg) plus a speed
  term, with McDonald losses unchanged. Peak torque and peak speed are both
  design variables.
- **Variables:** gear ratio, including a direct-drive option.
- **Gearbox:** AFDD83 mass in ratio and output torque.
- **Generator:** speed matched to the turboshaft output (or a reduction
  gearbox stated explicitly).
- **Result:** the cruise-torque trap and the direct-drive decision appear in
  the solution.

### 14 Trajectory optimization (user-requested 2026-10-02; moved up to Tier 14 on 2026-10-03)

**First problems:**

1. **Minimum-energy transition.** Hover to airplane mode (and back) at
   fixed altitude bands, with nacelle tilt as a control, through the
   conversion corridor.
2. **Time to climb.** Minimum time from take-off to a given altitude and
   speed.

Both run on the sized Tier 12/13 aircraft. Aerodynamics uses AeroSandbox's
`AeroBuildup` where it stays symbolic-friendly, and `SimpleAerodynamics`
otherwise; the full aero tier is 21. Battery and hot-day limits enter as
Tiers 16–17 land.

This is separate from sizing. It is a stand-alone optimal-control problem on
a fixed aircraft taken from a sizing result (`HaloSizingResult.design`), not
a new constraint inside the sizing Opti.

- **Principle (user, 2026-10-02): lean on AeroSandbox.**
  - Dynamics: `asb.DynamicsPointMass2DSpeedGamma` (longitudinal),
    `DynamicsPointMass3DSpeedGammaTrack` for ground tracks, and
    `DynamicsRigidBody2DBody` if pitch dynamics are needed. Each one's
    `add_force` and `state_derivatives` define the problem.
  - Collocation: `asb.Opti` with `opti.constrain_derivative` /
    `derivative_of` (trapezoidal integration).
  - Atmosphere: `asb.Atmosphere`.
  - Aerodynamics: `asb.AeroBuildup` evaluated per node (Tier 21).
  - This project adds only what AeroSandbox lacks: rotor and powertrain
    forces, the tilt kinematics, and the energy states.
- **Dynamics:** point-mass longitudinal dynamics with the AeroSandbox
  classes above, by direct collocation. Nacelle tilt is a control, so
  hover, conversion and airplane mode form one continuous trajectory, with
  rotor thrust and wing lift blended through the tilt angle.
- **States:** position, velocity, flight-path angle, mass, battery SOC (and
  battery temperature once Tier 19 exists).
- **Controls:**
  - nacelle tilt;
  - rotor thrust or collective;
  - rotor speed (Tier 12);
  - the electric power fraction;
  - pitch attitude.
- **Constraints:**
  - the conversion corridor;
  - power available against altitude and temperature (Tier 16);
  - battery current and voltage limits (Tier 17);
  - SOC reserves;
  - load factor and airspeed limits;
  - obstacle and terminal conditions.
- **Objectives:** minimum fuel, minimum time, or minimum energy cost, for a
  given mission or ground track.
- **Coupling to sizing:** one-way. Trajectory optimization checks and refines
  a sized design (for example the optimal climb, conversion and descent
  profiles, and the real energy split). Its results can feed back as better
  segment definitions or margins, but sizing stays on quasi-steady segments
  (the existing roadmap rule: no full trajectory optimization inside sizing).
- **Validation:** with tilt and speed frozen, it recovers the quasi-steady
  segment results; conversion power is checked against the XV-15 conversion
  corridor and power-against-airspeed data (TM X-62407 figs. 5.4.1 and
  A-12).

### 15 Electrical layer (review item 3)

- **Inverters:** mass and efficiency maps.
- **Cables:** mass and loss from layout length and current at the bus
  voltage.
- **Protection and contactors**, and an optional DC/DC converter.
- **Bus voltage:** a discrete design variable (enumerated, for example
  540 / 800 / 1,000 V), with derating and partial-discharge margins.
- **Installation factor:** replaced by explicit items; the installation
  factor of 1.0 goes away.

### 16 Hot and high (review item 6)

- **Atmosphere:** ISA plus a temperature offset.
- **Engine:** lapse in both density and temperature, fitted to the XV-15
  standard-day and 95 F curves (TM X-62407 figs. 6.2.2 and 5.1.2, already
  in hand).
- **Requirements:** hot-day hover at the destination after the mission, at
  that point's mass and state of charge.
- **Result:** exercises the hybrid's main advantage, the battery covering
  turbine lapse.

### 17 Battery with sag and ageing (review item 5)

- **Cell model:** an equivalent circuit, with OCV(SOC) and
  R(SOC, T, pulse duration), and series/parallel pack scaling.
- **Cell data:** S. Paudel et al., "Systematic Characterization of
  Lithium-Ion Cells for Electric Mobility and Grid Storage: A Case Study on
  Samsung INR21700-50G", *Batteries* 2025, 11, 313 (CC BY;
  user-supplied 2026-10-02), https://doi.org/10.3390/batteries11080313.
  - The cell: Samsung INR21700-50G, NCA, 4.9 Ah, 69 g, 2C continuous.
  - Fig. 7: OCV at 5 % SOC steps from -10 to 45 C (raster; digitize).
  - Fig. 8: DCIR and pulse power at 2, 10, 30 and 180 s, at SOC 0.2–0.8
    and six temperatures. This figure is vector graphics, so curve
    coordinates are recovered exactly.
  - Fig. 12: second-order ECM parameters (raster).
  - It is an energy cell: about 0.5 kW/kg continuous at cell level, and
    about 1.5–2 kW/kg for 10 s at 30 C. Against the Tier 10c assumption of
    3 kW/kg pack and roughly 12C for engine-out hover, the engine-out power
    reserve is expected to dominate battery mass. A power cell (or a
    cell-chemistry trade) becomes a design question for this tier.
- **Scaling decision (user, 2026-10-02):** take the *shape* of the discharge
  behaviour from the 50G data. That means OCV(SOC), and the trends of R
  with SOC and temperature. Scale the resistance (and with it the power
  capability) by an explicit factor so that the cell is as power-dense as
  the design needs. The factor is a named assumption, reported with its
  implied cell-level specific power, and swept in the sensitivities.
- **Bus:** bus voltage follows the battery, so the minimum voltage at the
  low-charge landing sizes the machines and inverters.
- **Ageing:** end-of-life capacity and resistance, and a cycle-life cost.
- **Reserves:** defined in both power and energy.
- **Mission:** segments sub-divided so OCV can vary within a long segment.
- **Algebraic loop (power demanded = power supplied):**
  - No iteration is needed. Battery current is already an Opti variable per
    flight point, and V = V* - I R0 with P = V I are equalities solved
    simultaneously with everything else. Here V* = OCV(SOC) - V_RC1 - V_RC2,
    so the loop is R0 I^2 - V* I + P = 0.
  - The physical low-current root (the closed form
    I = (V* - sqrt(V*^2 - 4 R0 P)) / (2 R0)) is selected explicitly with a
    branch constraint V >= V*/2. That also enforces the power ceiling
    P <= V*^2 / (4 R0).
  - Before this tier, the initial guess and the tiny pack resistance (under
    1 % sag) kept solutions on the low root, but nothing guaranteed it.
  - The RC states are propagated per sub-segment (the 2-RC ECM of
    Paudel et al.).

### 18 Redundancy and failure cases (review item 4)

- **Architecture:** N lanes per rotor (dual-wound machines, split
  inverters), cross-strapped buses and battery strings.
- **Failure cases** as multipoint constraints:
  - a lane out in hover, with roll trimmed by differential collective;
  - a genset out at landing;
  - a string out;
  - a bus fault.
- **Ratings:** short-time emergency ratings.
- **Removed:** the fixed-wing failed-propulsor yaw helper is dropped for
  tiltrotors.

### 19 Thermal (review item 3)

- **Losses to heat:** each source's losses go to a heat-exchanger mass and a
  cooling drag (Meredith-style), sized on the hot-day hover.
- **Short-time ratings:** set by the thermal mass of the machines and the
  battery.

### 20 Tiltrotor airframe weights (review item 7)

- **Wing:** the AFDD tiltrotor wing (NDARC sec. 19-1.1): torque box and
  spars sized by torsional and bending stiffness, plus a reduced-order
  whirl-flutter frequency constraint.
- **Second calibration aircraft:** V-22 or AW609 from public group weights.
- **Uncrewed adjustments:** explicit and itemised.

### 21 Aerodynamics (review item 8)

**Principle (user, 2026-10-02): lean heavily on AeroSandbox's built-in
aerodynamics.** `asb.AeroBuildup` on the `Aircraft.to_asb()` geometry is the
primary model: wing, tail and fuselage forces, its own compressibility and
form-factor treatment, and stall. That includes any nacelle or spinner
modelled as AeroSandbox fuselages. Scholz's level-0 method fills only what
AeroBuildup does not provide (interference factors, miscellaneous and
landing-gear drag, slipstream corrections) and serves as an independent
hand check. `SimpleAerodynamics` stays as the simplest model.

- **Level-0 drag build-up:** Scholz, *Aircraft Design*, ch. 13 "Drag
  Prediction" (HAW Hamburg lecture notes, user-suggested 2026-10-02),
  https://www.fzt.haw-hamburg.de/pers/Scholz/HOOU/AircraftDesign_13_Drag.pdf.
  - Component method: C_D0 = sum(C_f FF Q S_wet) / S_ref + C_D,misc +
    C_D,L+P.
  - Form factors: DATCOM / Raymer for wings and tails, the DATCOM fuselage
    form factor, and Raymer's nacelle FF = 1 + 0.35 / (l/d).
  - Interference factors from Table 13.4, and wetted areas per Torenbeek.
  - Wave drag from M_DD (Korn-type).
  - Oswald factor per the Nita-Scholz method.
  - Cross-checked against AeroSandbox's built-in methods (`AeroBuildup`,
    `Cf_flat_plate`, the fuselage form factor) that `SimpleAerodynamics`
    already uses. Nacelle and spinner drag areas replace the guessed
    0.8 m2.
- **Rotor-wing interaction:** correction factors for the slipstream-blown
  wing in airplane mode (dynamic-pressure and swirl increments on the
  immersed wing span: lift, drag and the change in Oswald factor).
- **Compressibility:** drag rise for thick sections (Korn equation).
- **Tails:** a V-tail option.
- **Download:** a function of wing and rotor geometry.
- **Conversion:** conversion segments with their power profile and the
  conversion corridor.

**Status (plan 025, 2026-10-03): implemented in part.** `BuildupAerodynamics`
(AeroBuildup plus Scholz interference, fittings drag area, fuselage Oswald
factor, transition, blown wing, geometric download), `ScholzAerodynamics`
(level-0 hand check with Korn-Lock wave drag and Nita-Scholz e) and
`HaloAssumptions.aerodynamics_model` ("simple" stays the default). Deferred:
V-tail, conversion segments, the blown wing in conversion (trajectories use
the unblown polar), trim drag. See IMPLEMENTATION_NOTES "Tier 21".

### 22 Design-space practice (review items 9 and 10)

- **Freed trades:** aspect ratio, fuselage size, solidity, tip speed, gear
  ratio, bus voltage and lane count, as variables or enumerated.
- **Multistart.**
- **Objectives:** a direct-operating-cost objective, with payload-range and
  sensitivity outputs.
- **Architectures:** enumeration of discrete powertrain architectures.
- **Robustness:** optimization over the dominant assumptions (figure of
  merit, cruise efficiency, battery specific energy, machine specific
  power).

