# Foundation and simple powertrain implementation notes

## Delivered scope

Tier 0 supplies pyproject metadata, Python 3.13 selection, a uv dependency lock,
six repository-local skills, engineering governance, interfaces and a staged
roadmap. `uv sync` creates the editable local environment automatically. Runtime
dependency: AeroSandbox; dev dependency: PyYAML for skill validation. Tests use
standard-library unittest. The verified environment resolves AeroSandbox 4.2.10
and CasADi 3.8.1. Source compatibility is declared for Python >=3.10; execution
has been verified on Windows with Python 3.13.3 only.

Tier 1 supplies six independently usable components with immutable result
dataclasses, ratings, SI units, specific-power/energy mass estimates and manual
module examples. Motor and generator share a replaceable quadratic loss model.
Battery accounting conserves chemical and terminal energy including Joule losses.
The rotor accepts thrust directly or exposes inverse-operation power residuals
for caller-owned Opti coupling. No hidden optimizer or convergence loop exists.

## Physics decisions and limits

The native AeroSandbox propeller power helper inspected in version 4.2.8 divides
by axial airspeed, so it is singular at hover. The equivalent momentum equation
implemented here is valid at hover and checked against the native helper in
forward flight. It excludes vortex ring, windmilling and tilt-transition effects.
The positive induced-velocity root is rationalized to avoid cancellation at low
thrust and high axial speed. The exact zero-load point is handled explicitly.

Default numerical parameters illustrate engineering interfaces. They are not
measurements or inferred proprietary Halo specifications. Generator ratings are
shaft-input ratings; motor ratings are shaft-output ratings. All hardware limits
and valid operating domains remain caller-owned constraints. Engine efficiency
is constant and has no idle flow or atmospheric lapse. No thermal subsystem,
rotor blade mass correlation, installed-mass allowances or failure model exists.

## Verification

Thirty tests cover conservation identities, zero and ideal limits, mass scaling,
charge/discharge signs, physical trends, array evaluation, native AeroSandbox
agreement and symbolic component evaluations/size variables. Three integration
tests use actual Opti solves. The example specifies 5000 N hover thrust and a 20%
battery fraction of motor electrical input, obtaining approximately 89.286 kW
rotor shaft power, 18.653 kW battery terminal power and 0.005852 kg/s fuel flow.
Power residuals are reported and tested explicitly. All six manual examples
execute, and all six skill files pass the bundled skill validator.

CasADi 3.8.1 emits a NumPy compatibility FutureWarning through the installed
AeroSandbox stack during symbolic evaluation. Tests pass with its default mode;
the project does not change process-wide CasADi settings to silence it. Matplotlib
may try to cache fonts outside a sandbox; verification used MPLCONFIGDIR pointing
to the ignored workspace `.mpl-cache` directory when needed.

## Tier 2: typed ports and topology

`core/` holds generic ports, `Topology`, electrical buses and
`connection_residuals`; `powertrain/ports.py` declares component ports in a
dispatch table so the Tier 1 component interface is unchanged;
`powertrain/topologies.py` builds the series hybrid. The topology creates no
Opti variables, constraints or solves: the caller supplies port values and
applies the labelled residuals. `examples/series_hybrid_point.py` is the
topology-coupled point; the Tier 1 hand-written coupling is kept as
`examples/series_hybrid_point_explicit.py`.

The topology example adds explicit coupling variables at the connections
(turboshaft torque, rotor speed and torque, bus voltage) so shaft and bus
residuals are active rather than trivially satisfied. With one rotor it matches
the explicit Tier 1 result to better than 1e-6 relative (89.286 kW rotor shaft,
18.653 kW battery, 0.005852 kg/s fuel). With four symmetric rotor strings on one
bus, the battery is modeled as four reference packs in parallel (resistance / 4)
so bus voltage and per-motor current match the single case; total motor
current and battery power are exactly 4x, while fuel flow exceeds 4x because the
single generator's quadratic torque loss is unscaled. This is an illustrative
scaling choice, not a sizing rule.

Limits: no splitters, combiners, multi-input gearboxes, inverters, thermal or
fuel-tank ports; direct connections need equal counts; multiplicity is
symmetric, so failed-propulsor cases cannot yet be represented. Verification:
58 unittest cases (28 new) and the Tier 2 notebook (71 checks).

## Tier 3: compatibility margins

Operating margins replace the hand-written rating list in
`examples/series_hybrid_point.py`; the solution is unchanged (ratings are
inactive at 5000 N) and the report names motor shaft power as most critical
(7.95 % headroom). `build_point_problem` exposes the coupled point so callers
choose the objective: maximizing thrust shows the motor rating binding below
100 kW and the 100 kW gearbox input rating capping thrust above it (about
5284 N per rotor). Design margins of the default parts record the 150 kW
turboshaft overdriving the 100 kW generator as -0.5 without resizing; an Opti
using design margins as constraints sizes the minimum generator at 150 kW.
Verification: 77 unittest cases (19 new) and the Tier 3 notebook (47 checks).
Not modeled: current limits on machines, SOC and thermal margins, failure-case
margins, gear-transformed speed/torque envelopes.

## Tier 4: vehicle geometry, CG and mass closure

`vehicle/` adds physical components that own geometry, hand it to `asb.Wing`
and `asb.Fuselage`, and call AeroSandbox's Raymer general-aviation correlations;
CG aggregation is `asb.MassProperties` addition, so no correlation or
mass-weighting formula is duplicated. `examples/aircraft_mass_closure.py`
closes the reference aircraft (12 m2 AR 9 wing, 7 m fuselage, 300 kg payload,
four rotor strings from the Tier 2/3 topology, 1.1 installation factor) in one
solve: MTOM 1552.4 kg, empty 1252.4 kg, powertrain 880 kg; the wing leading
edge at 3.233 m places the CG at 25 % MAC. Payload growth factor is about 1.11
with the powertrain fixed. Hover thrust per rotor is about 3806 N, below the
Tier 3 capability of about 5284 N; this is reported, not yet a requirement.

Raymer GA correlations are for light manned aircraft and are uncalibrated here;
`mass_factor` is the calibration hook. Powertrain ratings are not resized with
MTOM until coupled closure (Tier 9). Run the example as a module from the repo
root (`python -m examples.aircraft_mass_closure`) because it imports the
topology example. Verification: 102 unittest cases (25 new) and the Tier 4
notebook (62 checks).

## Plan 005: McDonald electric-machine losses

Motor and generator default losses now follow McDonald, "Modeling of Electric
Motor Driven Propellers for Conceptual Aircraft Design", AIAA 2015-1676, eqs.
1-4, through the unchanged machine interface. Defaults place the peak (0.96) at
the reference hover operating point. Rotor shaft power, margins and masses are
unchanged; the reference point now needs 19.184 kW battery power and
0.006207 kg/s fuel (Tier 1-2 sections above record the earlier simple-model
values, 18.653 kW and 0.005852 kg/s). The reference generator is rubber-scaled
with rotor count, so four-rotor fuel flow is exactly 4x. The paper's Figure 1
instance is reproduced in `notebooks/tier1_powertrain_components/
motor_loss_model_verification.ipynb` (32 checks); 112 unittest cases pass.

## Tier 5: low-fidelity aerodynamics

`SimpleAerodynamics` reuses AeroSandbox skin friction, lift-slope ratio, Oswald
factor and fuselage form factor; only Raymer's surface form factor is written
here. On the reference aircraft at 60 m/s and 1000 m: CD0 0.038 (the 0.25 m2
miscellaneous drag area for gear, nacelles and stowed rotors is 54 % of it),
e 0.76, (L/D)max 11.8 at CL 0.91, stall 13.1 deg at CLmax 1.5.
`examples/cruise_closure.py` solves mass closure, CG placement and lift =
weight together; cruise L/D 11.1, drag power 82 kW, MTOM 1552.9 kg (within
0.04 % of the assumed-L/D closure: the fuselage correlation is weakly coupled).
With this polar the minimum-power CL (1.57) exceeds CLmax, so drag power rises
monotonically from stall. AeroBuildup's slope is 20 % higher because it
includes tail lift (Tier 6). Verification: 126 unittest cases (14 new), Tier 5
notebook 28 checks.

## Tier 6: stability, trim and tail sizing

At the Tier 4 CG the reference aircraft has a 12.1 % static margin, trims at
60 m/s with -5.1 deg elevator (tail download) and has Cn_beta 0.040 /rad.
Because all rotors share one x station, hover pitch trim with equal thrust
requires the CG under that station; `examples/tail_sizing.py` uses this as an
equality and minimizes MTOM over wing position and both tail areas. Static
margin >= 10 % sizes the horizontal tail (2.16 m2) and Cn_beta >= 0.06 /rad the
fin (1.92 m2); the outboard-rotor failure at 1.2 V_stall needs 14.1 deg rudder
(limit 20). MTOM 1553.3 kg. Rotor lateral positions are explicit inputs (not
represented by symmetric multiplicity). Verification: 141 unittest cases (15
new), Tier 6 notebook 28 checks.

## Tier 7: requirements and flight points

The flight point couples aerodynamics and the full powertrain at one
quasi-steady condition; requirements are flight points with operating margins
>= 0. On the Tier 6 aircraft with reference ratings all four requirements are
feasible. `examples/requirements_sizing.py` resizes the rubber powertrain in
the closure at minimum MTOM: 1113 kg; max speed (80 m/s) on turbogenerator
power sizes turboshaft and generator (200 kW); hover T/W 1.1 at 1000 m sizes
motors (73 kW each) with the battery covering 67 % of hover power; disk area
falls to 3.8 m2 because point requirements carry no energy cost (missions fix
that in Tier 8/9). No turboshaft altitude lapse yet (Tier 10). Verification:
155 unittest cases (14 new), Tier 7 notebook 51 checks.

## Tier 8: missions

Segments (M1) chain into missions (M2) with fuel burn and SOC;
`examples/mission_analysis.py` closes MTOM, fuel load (1.1 x burnt) and wing
position with a 64-minute reference mission (two 60 s hovers, climb to 1000 m,
100 km cruise, 20 min loiter, descent) on the fixed Tier 6 aircraft.
Prescribed (hover 70 % battery, otherwise turbogenerator): MTOM 1588.4 kg, fuel
32.0 kg, landing SOC 0.80. Semi-free (M3: cruise speed and every h_e free,
minimum fuel): cruise 51.7 m/s, battery to the 0.30 floor mostly in loiter,
fuel 25.0 kg (-22 %), MTOM 1580.7 kg. With fixed speeds the minimum-fuel
cruise speed lies near 50-55 m/s. Fuel sits at the rotor station so hover
trim holds through the mission. Verification: 166 unittest cases (11 new),
Tier 8 notebook 50 checks.

## Tier 9: coupled sizing, mission optimization and energy allocation

One Opti sizes MTOM, fuel, wing position and area, tails, rubber motor and
generator, turboshaft, battery power and energy and rotor disk area, while
choosing cruise speed and every segment's electric power fraction, subject to
mass closure, hover trim, the Tier 7 requirements, the Tier 8 mission (fuel
reserve, SOC floor), an engine-out reserve (60 s battery-only hover from SOC
0.30 to >= 0.10), static margin, Cn_beta, failed-rotor rudder and all
operating margins.

Minimum MTOM: 883.6 kg (empty 561 kg, fuel 22.7 kg, battery 13.5 kWh), wing
5.05 m2, disk 4.6 m2, 46.8 kW motors, 175 kW turboshaft, cruise 45.3 m/s. Max
speed sizes the turbogenerator, hover the motors, the engine-out reserve the
battery, static margin and Cn_beta the tails; the optimizer spends the carried
battery in loiter and landing down to the SOC floor. Minimum fuel: 2123 kg,
1.3 kg fuel, 184 kWh battery (all-electric mission). Cruise 50/100/150 km gives
MTOM 873/884/894 kg.

Findings from making it converge: (1) raw-unit equalities (W, N) stalled IPOPT;
normalizing flight-point equalities and scaling variables fixed conditioning.
(2) The battery mass kink (energy- vs power-sized) sits at the optimum; an
optional smooth maximum (<= 2 kg x ln 2 overestimate) resolves it. (3) On mass
alone the battery never pays (0.9 MJ/kg versus 12.9 MJ/kg shaft energy from
fuel at 30 %), so without a redundancy requirement min-MTOM deletes it; the
engine-out reserve is the sizing reason, as in real series hybrids. Small
min-MTOM wings follow from having no field-length or gust requirement.
Verification: 174 unittest cases (8 new), Tier 9 notebook 26 checks.

## Tier 10a: tiltrotor weights and XV-15 validation

AeroSandbox has no rotorcraft weight equations. `weights/afdd.py` wraps NDARC's
AFDD82/83 rotor, drive-system and engine-section equations in SI. The XV-15 is
then built from framework components at its published geometry and compared,
group by group, with its Nov 1974 weight statement at 13,000 lb.

| Group | Predicted / actual (lb) | Ratio |
|---|---|---|
| Transmission | 1,265 / 1,263 | 1.00 |
| Powerplant | 1,532 / 1,754 | |
| Tails | 186 / 209 | |
| Gear | 585 / 508 | |
| Rotor | 1,545 / 1,070 | |
| Wing | 452 / 873 | 0.52 |
| Fuselage | 696 / 1,442 | 0.48 |
| Hydraulics and flight controls | 247 / 934 | 0.26 |

The powerplant combines AeroSandbox's turboshaft regression (556 lb per engine)
with the AFDD82 engine section. The rotor figure uses a coning frequency of
1.55/rev, read from TM X-62407 fig. 7.1.1; 1.35/rev reproduces 1,070 lb.

The rotorcraft-specific equations do well. The Raymer GA fixed-wing groups do
not: the tiltrotor wing is stiffness-sized, the fuselage is built for crash
loads, and the controls include rotor and conversion actuators.

With the XV-15 useful load, the uncalibrated framework closes at 11,315 lb and
an empty weight of 7,391 lb, 19 % light. Per-group calibration factors
(actual / predicted) reproduce 13,000 lb exactly. Verification: 201 unittest
cases (27 new) and the Tier 10 notebook's 15 checks.

## Tier 10b: engine lapse, part power and hover power

Fig. 6.2.2 of NASA TM X-62407 was digitized by pixel crossings:

| Altitude (ft) | 0 | 4,000 | 8,000 | 12,000 | 16,000 | 20,000 |
|---|---|---|---|---|---|---|
| XV-15 rotor shaft power available per engine (shp) | 1,374 | 1,285 | 1,181 | 1,061 | 942 | 788 |

A least-squares fit gives sigma^0.80, within 6 % to 20,000 ft. The real curve
is flatter low down and steeper higher up, as for a flat-rated engine.

The hover figure of merit was calibrated at sea level with the stated 7 %
download, giving 0.67. Predicted OGE hover weights are then within 2 % up to
10,000 ft and 3 % at 20,000 ft. The hover ceiling at 13,000 lb comes out at
7,140 ft. The figure's take-off line gives about 7,800 ft, and SP-4517 gives
8,650 ft at an unstated rating. The gap comes from the lapse under-prediction
between 4,000 and 12,000 ft.

The AeroSandbox / Geiss part-power knockdown matches the LTC1K-4K's sfc at its
four ratings within 1 %, taking contingency as maximum power.

Consequence for Tier 9: its hover power (FM 0.80, no download) was low by
33 %. At the 13,000 ft Halo-class ceiling, 73 % of sea-level power remains.
Verification: 217 unittest cases (16 new) and the Tier 10b notebook's
10 checks.

## Tier 10c: Halo-class sizing

The Tier 9 formulation, re-baselined on the XV-15:

- two tip rotors and two turbogenerators plus a battery on one bus;
- Tier 10a calibrated weights;
- AFDD rotors and gearboxes;
- AeroSandbox turboshaft mass and efficiency regressions, with the mass-power
  relation as an explicit equality;
- Tier 10b lapse, part-power and hover models.

Requirements (user-approved): 900 kg payload, 445 nm, 250 kt at 10,000 ft,
13,000 ft ceiling, OGE hover at 4,000 ft with T/W 1.05, engine-out hover on
one turbogenerator plus battery, stall at or below 120 kt.

**Result:**

- 8,500 kg (18,740 lb) take-off, 6,347 kg empty;
- powertrain 55 % of empty mass;
- 2 x 1,280 kW turboshafts and 2 x 1,439 kW motors;
- 140 kWh / 1,666 kW battery;
- 27.4 ft rotors at 78 kg/m2 disk loading;
- 23.8 m2 wing at 357 kg/m2;
- cruise 180 kt at L/D 8.3.

**What binds:** max speed sizes the turbogenerators. Hover sizes the motors,
gearboxes and rotors. Engine-out hover sizes battery power, and the 120 kt
stall limit sizes the wing.

**Sensitivities (warm-started):**

| Case | Take-off weight (lb) |
|---|---|
| Hover FM 0.75 | 17,444 |
| Cruise rotor coefficient 0.80 | 19,935 |
| Payload 600 / 1,200 kg | 16,741 / 20,763 |
| Uncalibrated weights | 13,537 |

From the cold default guess, the coefficient-0.80 and uncalibrated cases stop
at a local infeasibility in IPOPT. Nearby values solve cold, and both solve
warm-started, so `initial=` exists.

Verification: 230 unittest cases (13 new) and the Tier 10c notebook's
14 checks.

## Tier 11a: turboshaft deck part-power curve

The user supplied a GASP_TS 1,120 hp turboshaft deck: a full 13 Mach x
10 altitude x 16 throttle grid.

**The deck's altitude scaling is not physical.** Under the usual
correction, maximum power falls 20 % by 1,500 ft and stays flat at about
900 hp to 10,000 ft. It reads 953 hp at 15,000 ft and 612 hp at 17,500 ft.
This was reported to the user, so the XV-15-fitted lapse stays.

**The normalised part-power curve is sound.** sfc / sfc_max against power
fraction does not depend on the correction convention:

| Power fraction | 0.3 | 0.5 | 0.7 | 0.9 |
|---|---|---|---|---|
| Deck median sfc ratio | 1.54 | 1.22 | 1.09 | 1.02 |
| Geiss | 1.61 | 1.23 | 1.09 | 1.03 |

The deck's row-to-row spread is about +/-10-15 %. The deck also matches the
XV-15 ratings within 3 %.

**Why a cubic.** A B-spline through the table stalled IPOPT in the coupled
sizing (3,000 iterations, about 0.1 s each), because the low-power end is
steep. A cubic fit in Geiss's form fits the table within 1.1 % above 20 %
power and keeps the problem smooth and cheap.

**Result.** The Halo-class sizing with the deck curve closes at 8,394 kg
(18,506 lb), against 18,740 lb with Geiss.

**Data handling.** The raw deck stays local and gitignored at
`data/engines/`. A test re-derives the embedded table from it when it is
present.

Verification: 242 unittest cases (12 new) and Tier 0's 296 checks.

## Tier 12: rotor speed physics (JVX-calibrated)

`MomentumProfileRotor`: momentum induced power plus blade profile power over
sections at sqrt((Omega r)^2 + V^2), with one drag polar in loading referred
to the mean section dynamic pressure.

**Data.** NASA/TM-2016-219070 Appendix D, parsed by word coordinates (the
text layer wraps):

- hover Table D-1: 58 points, Mtip 0.67–0.68;
- hover Table D-2: 13 points, Mtip 0.73;
- Phase II airplane mode, Tables D-7a and D-7c: 42 points, lambda
  0.26–0.56.

The parsed airplane points reproduce eta = CT lambda / CP within 0.0004.

**Fit.** A free least-squares fit gives kappa < 1 (hover) and < 0
(airplane), because the induced and loading terms are collinear over the
data. So kappa is fixed at 1.15 and the fits are linear in the drag terms:

- hover polar: c_d = 0.0185 - 0.222 x + 1.066 x^2, with its bucket at
  CT/sigma ~ 0.10. Figure-of-merit RMS error 0.012; the Mtip 0.73 points
  (held out) are predicted within 2.2 % in power;
- airplane increment: 0.0017 + 0.0188 lambda^2. Efficiency RMS error 0.013,
  max 0.036, with no remaining trend in lambda (a constant increment left a
  -0.02 to +0.015 trend).

**Limitation found.** The model has no blade-stall physics. At large lambda
the (1 + 3 lambda^2) reference makes heavily loaded blades look light, so
cruise power keeps falling as the rotor slows. Unbounded, the Halo sizing
slowed cruise to lambda 0.83 (45 % of design tip speed), well outside the
data. `advance_ratio_max` = 0.60 is therefore a validity bound, and the
bound, not the physics, sets cruise rotor speed (about 61 % of design tip
speed against the XV-15's 81 %). BEM with stall is the deferred remedy.

**Halo-class result:**

- 7,814 kg (17,228 lb), against 18,506 lb with the actuator disk (that
  option reproduces it exactly);
- solidity 0.074 and design tip speed 238 m/s (at the hover tip-Mach
  bound);
- rotor radius at the span-clearance limit;
- motors 958 kW (was 1,419 kW) and battery 750 kW, because hover power falls
  with the JVX-class figure of merit of about 0.8.

Verification: 261 unittest cases (19 new).

## Tier 12b: fixed engines, battery-assisted hover, in-flight recharge

User rules (2026-10-03):

- turbine power required never exceeds power available;
- engines are fixed at the user's 1,120 hp deck engine (a non-OEM cannot
  raise power);
- the battery supplements hover and is recharged in flight;
- downsize the aircraft if needed.

**Implementation.**

- `HaloAssumptions.power_rated_turboshaft_fixed_W` = 1,120 hp.
- `build_flight_point` and `build_mission` take
  `hybridization_electric_min`: a negative battery share means recharge.
- `build_mission` keeps SOC inside the battery window at every segment and,
  with `soc_floor`, holds the 0.30 reserve throughout. Without that floor,
  cruise dipped to 0.20, below the engine-out reserve's assumption.
- `solve_halo_sizing(objective="payload")`.

**Result.** Maximum payload against sustained max speed:

| Max speed (kt) | 180–200 | 205 | 210 | 215 | 220 | 225 |
|---|---|---|---|---|---|---|
| Max payload (kg) | 1,240 | 1,186 | 935 | 683 | 424 | 163 |

At 180–200 kt the climb requirement on turbines alone limits payload. 250 kt
is infeasible at any size, partly because the fuselage stays at XV-15 size.

**Provisional reference:** 900 kg at 210 kt gives 6,748 kg (14,877 lb):

- 2 x 811 kW motors and 2 x 716 kW generators;
- a 67 kWh / 799 kW battery that covers about 27 % of take-off hover and
  49 % of landing hover, and is recharged in climb and descent.

The named legacy sets (`requirements_tier10c`, `assumptions_tier11a`,
`assumptions_tier12`) reproduce the earlier tiers exactly.

## Tier 13: machines sized by torque

Machine mass is now the smooth maximum of peak torque / torque density and
rated power / specific-power cap:

- torque density 15 N.m/kg, anchored on magniX magni650 (3,216 N.m in
  206 kg) and magni350 (1,608 N.m in 128 kg), both including inverters and
  cables;
- a 10 kW/kg cap at high speed.

Motor speed, rotor gear ratio and generator speed are design variables. The
rotor gearbox uses AFDD00, and an optional step-up gearbox sits between each
engine's 1,210 rpm output shaft and its generator.

**Architectures** (maximum payload at 210 kt, fixed 2 x 1,120 hp):

| Architecture | Max payload | Notes |
|---|---|---|
| Specific-power machines (Tier 12b) | 935 kg | |
| Torque-sized, geared rotors, step-up generators | 1,014 kg | |
| Generators on the 1,210 rpm shaft | 33 kg | 1,464 kg of generators |
| Direct-drive rotors | infeasible | about 30 kN.m per motor, about 2 t each |

**Reference** (900 kg at 210 kt): 13,760 lb, against 14,877 lb. The motor
runs at about 13,100 rpm behind a 30.6:1 gearbox (AFDD00's mild ratio
penalty; stage count is not modelled), and the generator runs at about
13,400 rpm.

**Torque trap:** climb, with the rotor held slow by the lambda <= 0.6 bound
at high power, sets peak motor torque (568 N.m, against 414 N.m in hover).

Verification: 276 unittest cases (12 new) and the Tier 13 notebook's
8 checks.

## Tier 14: trajectory optimization

A stand-alone optimal-control problem on the sized Halo reference
(`solve_halo_sizing()` -> `build_halo_aircraft(design)`). It is not part of
the sizing Opti (plan 019).

**What AeroSandbox supplies.**

- Dynamics: `asb.DynamicsPointMass2DSpeedGamma`.
- Collocation: trapezoidal `constrain_derivatives`.
- Atmosphere: `asb.Atmosphere`.

**What this project adds** (`aircraft_closure.trajectory.tiltrotor`): the
tilting rotor thrust, wing forces from `SimpleAerodynamics`, the rotor ->
gearbox -> motor -> bus chain, the battery/turbogenerator supply and the
mass, SOC and bus-energy states.

**Controls per node:** alpha, nacelle tilt (|rate| <= 8 deg/s), thrust and
speed per rotor, battery current and generator torque; also the final time.

**Constraints:**

- the Tier 12 rotor bounds;
- the motor and generator ratings;
- the lapsed turboshafts;
- the battery discharge rating and SOC >= 0.30;
- stall alpha and pitch attitude;
- the conversion corridor: V_stall cos(tilt) to an XV-15 fit, 115 kt at
  90 deg and 180 kt at 0 deg.

In-flight recharge is off.

**Validation.**

- Hover at tilt 90 deg recovers `build_flight_point` hover power to 2e-6.
- Airplane mode with thrust along the path is an exact identity.
- A collocated steady trajectory is within 1e-4.
- Tilt frozen at 0 deg is 1.4 % lower, because of the thrust's lift
  component.

**Results (Halo, 6,748 kg).**

| Problem | Time | Bus energy | Fuel | SOC |
|---|---|---|---|---|
| Minimum-energy transition, hover at 500 ft to 1.3 V_stall (144 kt) | 22 s | 10.4 kWh | 2.9 kg | 0.95 -> 0.92 |
| Prescribed transition (level, constant acceleration, 60 s) | 60 s | 20.4 kWh | 6.9 kg | 0.95 -> 0.94 |
| Minimum time, hover at sea level to 10,000 ft at 153 kt | 212 s | 101 kWh | 26.5 kg | 0.95 -> 0.62 |

**Transition.** The optimum:

- pitches nose-down to the -10 deg attitude limit;
- follows the corridor's low-speed boundary;
- trades about 30 m of altitude band for speed;
- finishes at the 8 deg/s nacelle rate.

The motor ratings bind (about 1.7 MW bus), not the engines.

**Climb.** It runs at about 125-130 kt with the nacelles at about 46 deg
and pitch at its 20 deg limit, and converts at the top.

- The generator rating caps the turbines, and above about 7,000 ft the
  turbine lapse does; the battery fills to the motor rating.
- **The conversion-mode climb is probably flattered** by the axial-only
  rotor model, which has no edgewise profile or hub-load penalty. Re-check
  it when edgewise rotor physics lands.

## Tier 16: hot and high

**Data.** These XV-15 curves were digitized from NASA TM X-62407 by pixel
analysis of the 200 dpi scan:

- fig. 6.2.2, take-off power, T = 95 F (dashed);
- fig. 5.1.2, twin-engine OGE hover, 95 F.

Axes were calibrated on gridlines found from pixel sums, and the
zero-altitude row was checked against the axis line. The same procedure
re-reads the Tier 10b standard-day data within 7 shp and 100 lb.

| Altitude (ft) | 0 | 2,000 | 4,000 | 6,000 | 8,000 | 10,000 | 12,000 |
|---|---|---|---|---|---|---|---|
| 95 F rotor shaft power per engine (shp, +/- 10) | 1,103 | 1,021 | 941 | 864 | 791 | 720 | 647 |
| ISA offset at 95 F (K) | 20.0 | 23.8 | 27.7 | 31.5 | 35.3 | 39.3 | 43.3 |
| 95 F OGE hover weight (lb, +/- 100) | 13,339 (229 ft) | 12,445 | 11,442 | 10,458 | 9,546 | 8,708 | 7,902 |

The hover line continues at 7,046 / 6,248 / 5,572 lb at 14,000 / 16,000 /
18,000 ft.

**Model.** `DensityTemperatureLapse` gives P/P_rated = sigma^n
(T/T_ISA)^-m.

- At a fixed pressure altitude, P_95F / P_std = (T/T_ISA)^-(n+m).
- A data-to-data fit of the paired curves at 0 / 4,000 / 8,000 / 12,000 ft
  gives n + m = 3.28, so m = 2.49 with Tier 10b's n = 0.797. The ratio
  residuals are within 0.3 %.
- Density accounts for only a quarter of the hot-day loss. The rest is the
  turbine's temperature limit, about 1.1 % per kelvin.
- Absolute 95 F power available is within 4.4 %. That error is the Tier 10b
  standard-day lapse error, which is flatter low down and steeper high up.

**Validation (prediction, not fit).** The model uses the Tier 10b figure of
merit (0.67, standard-day sea-level calibration), the hot density and the
hot-day lapse:

- the fig. 5.1.2 hover weights are predicted within -2.6 % to -1.5 % up to
  12,000 ft, the end of the 95 F power data;
- they are within +0.5 % to +4.4 % to 18,000 ft, where the lapse is
  extrapolated.

At 4,000 ft the 95 F day costs the XV-15 21 % of hover weight.

**Halo.** The hot-day hover is at 4,000 ft / 95 F (ISA + 27.66 K), T/W 1.05,
for 60 s, at the mission's end mass (5,820 kg) and SOC (0.30), down to the
0.10 floor. It is on by default.

- The two 1,120 hp turboshafts give 1,115 kW there, against 1,518 kW at
  4,000 ft on a standard day.
- The battery must carry at least 25 % of the hover, against 14 % in the
  standard-day 4,000 ft hover at take-off mass.
- It is not binding. The engine-out hover already sizes battery power, and
  the heavier standard-day hover sizes the motors.
- The reference stays at 6,747 kg (14,874 lb, against 14,877 lb without the
  hot hover). SOC is 0.16 after the hot hover.
- The hybrid covers the turbines' hot-day lapse at no mass cost.

**Hotter and higher destinations.** These were found by continuation in
altitude at 95 F:

| Hot-hover altitude (ft) | 4,000–5,250 | 5,500 | 5,750 | 5,900 | 6,000 |
|---|---|---|---|---|---|
| Take-off mass (kg) | 6,748 | 6,755 | 6,795 | 6,834 | no closed design |
| Hot-hover battery share (min) | 0.25–0.31 | 0.32 | 0.35 | 0.37 | – |

The limit is the rotor, not the battery:

- the hot hover's blade loading (CT/sigma = 0.14 at the Mach-limited design
  tip speed) binds;
- the optimizer adds solidity;
- the extra blade area costs cruise power against the fixed engines at
  210 kt.

**Defaults changed.** These two defaults change:

- `HaloRequirements()` now includes the hot-day hover;
- `HaloAssumptions()` uses the density-temperature lapse, which is identical
  on standard days.

Effects:

- `requirements_tier10c` sets `hover_hot_day=False`, so the Tier 10c / 11a /
  12 baselines reproduce exactly (18,506 / 17,228 lb).
- The new `requirements_tier12b` reproduces Tier 12b. The Tier 12b unit tests
  and notebook source now use it, because the landing-hover battery share is
  a free, non-binding split and is not unique once the hot point is added.
  The notebook's outputs are unchanged.

Verification: 285 unittest cases (21 new) and the Tier 16 notebook's 18
checks.

## Tier 17: battery equivalent circuit

**Data.** The Samsung INR21700-50G (Paudel et al. 2025, CC BY 4.0),
digitized into `data/batteries/`:

- **Fig. 8 (vector):** curve vertices via PyMuPDF `get_drawings()`. That gives
  1,232 points (DCIR and pulse power, 2/10/30/180 s, six temperatures),
  essentially exact. Collinear vertices that MATLAB merged were restored.
- **Fig. 7 OCV and Fig. 12 ECM (raster):** colour clusters at each 5 % SOC
  column, about +-5 mV at 30/45 C.
- **Cross-check:** eq. (12), P = 2.5 (OCV - 2.5) / DCIR, reproduces the
  separately digitized power curves to 1.3 % rms.

**Model.** `EquivalentCircuitBattery`:

- OCV: a degree-7 polynomial at 30 C.
- Resistance: R0 + 2 RC (8 s, 43 s), with smooth R(SOC, T) fitted by an
  `asb.Opti` least squares to all discharge DCIR, 4.7 % rms.
- A power-density factor F: resistance / F, current rating x F (user
  decision).
- End-of-life capacity 0.8 and resistance 1.5; cell/pack mass 0.7.

**Algebraic loop.** The loop V = V* - I R, P = V I stays as Opti equalities.
`build_flight_point` adds V >= V*/2, which keeps IPOPT on the physical
low-current root. From a high-root start IPOPT reports infeasibility; it
never returns that root.

**Missions.** `subsegments` lets OCV and resistance follow SOC through long
segments. RC states propagate exactly between points, and each point checks
its end-of-interval voltage against the cutoff. The default of 1 keeps
earlier results bit-for-bit.

**Halo result.** The reference keeps the constant battery (14,877 lb). With
the ECM pack (`assumptions_tier17`, F = 5, end of life, 25 C), 900 kg at
210 kt does not close on the fixed 2 x 1,120 hp engines.

- **The binding constraint:** the engine-out reserve (60 s at about
  590 kW between SOC 0.30 and 0.10). It needs about 60–70 kWh at
  147 Wh/kg, against the old 250 Wh/kg.
- **Maximum payload:**

  | F | 2 | 3 | 5 | 8 | 12 | 20 |
  |---|---|---|---|---|---|---|
  | Max payload (kg) | 62 | 384 | 638 | 687 | 707 | 722 |

  F = 1.5 does not close. Power binds up to F of about 5 (current rating,
  then the 525 V cutoff at SOC 0.11). From F = 8 the reserve energy binds.
- **At F = 5:**
  - Pack: 4,730 cells, 466 kg, 68.7 kWh.
  - Bus voltage: 860 V at take-off and 750 V at the floor. In the
    engine-out hover it falls to the 525 V cutoff, which is the
    minimum-voltage case.
  - Sensitivities: beginning of life 693 kg; 0 C cell 431 kg.
- **On the current aircraft** (Tier 13 torque-sized machines, Tier 16
  hot-day hover; merged 2026-10-03): max payload is 785 kg at F = 5,
  832 kg at F = 8 and 851 kg at F = 12. That is still short of 900 kg.
  At F = 5 the engine-out end voltage still binds; from F = 8 the reserve
  SOC binds. The table above and the Tier 17 notebook stay pinned to the
  Tier 12b aircraft.
- **User decision (2026-10-03):** take the lower payload. Plan 022 makes
  this pack the reference.

## Plan 022: equivalent-circuit battery as the reference

The user chose "take a lower payload", noting there is "not much more we
can do to stretch the cells".

- **Defaults:** `battery_model="ecm"` and `mass_payload_kg=780` (the maximum
  is 785 kg).
- **Reference result:** 6,548 kg (14,436 lb).
  - Battery: 440 kg, 64.9 kWh, 210s x 21.3p (about 4,470 cells).
  - Binding constraints:
    - the 210 kt turbine power;
    - the engine-out end voltage (525 V cutoff);
    - the hover motor, gearbox and rotor power;
    - static margin and Cn_beta;
    - mission end SOC.
  - This is 676 lb heavier than the 900 kg constant-battery aircraft
    (13,760 lb), even with 120 kg less payload.
- **Starting point:** with no `initial`, `solve_halo_sizing` first solves the
  constant-battery problem as the initial guess. From the generic guess,
  IPOPT reaches local infeasibility.
  - The max-payload solve likewise starts from a constant-battery design.
    From the equivalent-circuit minimum-mass design it reaches local
    infeasibility.
  - Multistart is Tier 22.
- **Legacy sets:**
  - `requirements_tier16` and `assumptions_tier16` keep the Tiers 13–16
    aircraft (900 kg, constant battery, 13,760 lb).
  - Every earlier named set pins `mass_payload_kg=900` and
    `battery_model="constant"`.
  - The Tier 10–16 notebooks pin their own sets in their first cell.
- **Trajectory:**
  - The motors and generators see the battery terminal voltage. Previously
    they saw the constant OCV, which moves Tier 14 results by less than
    0.1 %.
  - With the equivalent-circuit pack, SOC is coulomb-counted, and the limits
    are current, voltage and the low-current root.
  - RC polarization is steady at each node; there are no RC states.
  - On the reference, the minimum-energy transition takes 22.1 s and the
    climb to 10,000 ft takes 227 s (SOC 0.95 to 0.85).
- **Parallel runs:** run notebooks and test suites concurrently with
  `OMP_NUM_THREADS=1`. Several concurrent suites with threaded BLAS made
  IPOPT fail on otherwise reproducible solves.

## Tier 20: tiltrotor airframe weights

Plan 024. The AFDD tiltrotor wing (NDARC Theory sec. 19-1.1) sizes the wing in
four steps:

1. the torque box from the torsion frequency;
2. spar caps from the chord and beam bending frequencies;
3. extra caps from a 2-g jump take-off;
4. fairings, control surfaces and fittings by unit mass or fraction.

Frequencies are in per rev of a design rotor speed. The model is a `Wing.mass_model`
submodel; the Raymer wing (None) stays the default and the simplest model.

**XV-15 section calibration.** Acree et al. (1999) publish the XV-15 wing:

- components: torque box 567, spars 52, control surfaces 97, fairings 108 and
  fittings 122 lb;
- stiffness: GJ 2.80e9, EI beam 3.70e9 and EI chord 1.12e10 lb-in2;
- aluminium properties.

With a torque-box chord ratio of 0.45 (assumed), these give NDARC's box
efficiency 0.583 and spar-taper correction 0.526. The fairing (10.9 kg/m2) and
control-surface (15.2 kg/m2) unit masses and the fittings fraction (0.129)
follow, and are the model defaults.

**Frequencies to stiffness.** The XV-15 inputs come from NASA/TP-2004-212262:

- published symmetric modes at 458 rpm: torsion 1.09, beam 0.43 and chord
  0.83 per rev;
- stick-model tip mass: 2,142 lb per side;
- pylon radius of gyration: 2.77 ft, or 0.222 R.

With these, NDARC's single-mode relations give 0.57-0.62 of the published
stiffness. The wing comes to 658 lb against the 873 lb statement, so
`Xv15MassFactors.wing_tiltrotor` = 1.327. Raymer gives 452 lb and its factor
stays 1.930.

| XV-15 closure | Raymer wing | AFDD wing |
|---|---|---|
| Uncalibrated take-off (lb) | 11,315 | 11,536 |
| Calibrated take-off (lb) | 13,000 | 13,000 |

**Second calibration aircraft.** Public V-22 and AW609 group weight statements
were not found. The calibrated model was checked with no further fitting:

| Wing | Model x 1.327 (lb) | Actual (lb) | Error |
|---|---|---|---|
| V-22 FSD, composite (Popelka et al. 1995) | 2,023 | 2,470 | -18 % |
| Bell D266 (1968 MIL-STD-451 design statement, Harris vol. III) | 1,840 | 1,886 | -2 % |

- The V-22 tip mass is a framework model-chain estimate.
- The V-22 wing's fold/rotate scope is not stated by the source.
- The D266 rotor group (2,439 lb) needs solidity 0.062 with AFDD82 x the XV-15
  factor 0.69, below typical proprotors (0.08-0.11). The calibrated rotor model
  is therefore heavy for that design.
- Drive groups have no public split for either aircraft.

**Whirl flutter (reduced order).** In the Halo sizing the wing design rotor
speed is a design variable. Every airplane-mode point carries margins on the
torsion and beam frequency, in per rev of that point's rotor speed: climb,
cruise sub-points, loiter, descent, maximum speed and ceiling. Flutter speed
itself is not modelled.

**Halo with `wing_weight_model="afdd_tiltrotor"`** (780 kg payload,
equivalent-circuit battery):

| | Raymer (reference) | AFDD wing |
|---|---|---|
| Take-off mass | 6,548 kg (14,436 lb) | 6,144 kg (13,546 lb) |
| Wing | 547.5 kg | 372 kg |
| Wing area | 22.6 m2 | 21.3 m2 |
| Max payload | 785 kg | 959 kg |

- The torsion whirl-flutter margin binds at the 210 kt maximum-speed point:
  the wing is built for that rotor speed, 39.4 rad/s. Cruise runs at 27.7 rad/s
  (1.55 per rev).
- The Halo wing uses graphite epoxy. Its tips carry rotor, motor, gearbox and
  turbogenerator.

**Uncrewed equipment.** The Halo equipment is now itemised from the XV-15
groups:

- kept: electrical 396 and instrumentation 91 lb;
- removed: crew ECS (-100), ejection seats (-230) and other furnishings
  (-206 lb);
- added: autonomy and mission avionics allocation (+100 lb).

The total is unchanged at 587 lb. Crew items inside the fuselage and
flight-control groups are not split out (no public breakdown).

Verification: 392 unittest cases (39 new) and the Tier 20 notebook's checks.

## Plan 026: AFDD tiltrotor wing as the reference

The user approved the switch on 2026-10-03.

- **Defaults:** `wing_weight_model="afdd_tiltrotor"`, payload back to
  900 kg.
- **Reference:** 6,462 kg (14,247 lb).
  - Battery: 62.7 kWh, 426 kg.
  - Wing: 22.2 m².
  - Binding: whirl-flutter torsion at the 210 kt point, and the engine-out
    end voltage.
- **Starting point** (equivalent-circuit battery, no `initial`):
  - first, the constant-battery solve;
  - if that start fails, the equivalent-circuit solve at 85 % payload.
  - The 900 kg case needs the second start.
- **Legacy sets** pin `wing_weight_model="raymer"`.
  `requirements_plan022` / `assumptions_plan022` keep the 780 kg plan 022
  reference at 14,436 lb.

## Tier 21: aerodynamics

Plan 025. User direction: lean on AeroSandbox; Scholz fills the gaps and is
the hand check; `SimpleAerodynamics` stays (and stays the Halo default,
`HaloAssumptions.aerodynamics_model = "simple"`).

- **`BuildupAerodynamics`:** `asb.AeroBuildup` on `Aircraft.to_asb()`, which
  now includes the tip nacelles as two bodies of revolution (9 ft x 3.3 ft,
  spinner as the nose). Added on top: Scholz Table 13.4 interference
  (nacelles 1.5, tails 1.04), the Nita-Scholz fuselage factor k_e,F and Mach
  factor k_e,M on AeroBuildup's span efficiency (AeroBuildup takes d_F = 0),
  a fittings drag area (XV-15 "fittings and fixtures" 3.00 ft2 from NDARC,
  Johnson 2010, Table 1) replacing the guessed 0.8 m2, fixed landing gear when
  not retractable, and boundary-layer transition at 10 % chord (AeroBuildup
  does not pass `xtr` to NeuralFoil; `TransitionAirfoil` does).
- **`ScholzAerodynamics`:** Scholz ch. 13 component method with Torenbeek
  wetted areas, Cf with the Mach term, Raymer nacelle FF, Korn-Lock wave drag
  (AeroSandbox `Cd_wave_Korn`, kappa 0.87) and Nita-Scholz e with the
  turboprop k_e,D0 0.804 (AeroSandbox's `oswalds_efficiency` uses the mean of
  four classes). The 23 % NACA 2423 wing at CL 0.6 has M_crit 0.47, above the
  210 kt cruise Mach 0.33: no drag rise anywhere in the Halo envelope.
- **Cruise polar** (reference geometry, 150 kt, 10,000 ft, CL 0.6):

  | Model | CD0 | e | CD | L/D |
  |---|---|---|---|---|
  | Simple (0.8 m2 misc.) | 0.0559 | 0.765 | 0.0804 | 7.46 |
  | AeroBuildup | 0.0351 | 0.765 | 0.0597 | 10.05 |
  | Scholz | 0.0358 | 0.735 | 0.0614 | 9.77 |

  D/q breakdown (buildup / Scholz, ft2): wing 2.32 / 2.42, tails 0.51 / 0.59,
  fuselage 1.41 / 1.62, nacelles 1.14 / 0.94, fittings 3.00; total 8.4 / 8.6
  against 12.0 ft2 (0.0559 x S) for Simple. The guessed 0.8 m2 (8.6 ft2) was
  nearly the XV-15's whole drag area (9.25 ft2).
- **XV-15 check (NDARC):** modelled components 4.94 ft2 (buildup) and 5.34 ft2
  (Scholz) against 6.25 ft2; with the 3.00 ft2 fittings 7.9 / 8.3 against
  9.25 ft2. The build-up is 15–20 % low on the clean components (NeuralFoil
  wing with transition at 10 %: 1.60 against 2.18 ft2).
- **Blown wing** (`slipstream.py`): momentum-theory slipstream at the wing
  (x = 0.4 R), contracted radius, ideal swirl; half of each tip rotor's
  slipstream is on the wing (17.8 of 22.2 m2 on the Halo, the rotors nearly
  span the wing). Dynamic-pressure ratio on lift and profile drag; swirl raises
  the local angle (inboard-up) and the wing recovers half of the swirl energy
  flux. At the 150 kt cruise thrust (1.7 kN per rotor): q ratio 1.006, swirl
  0.23 deg, delta CL +0.018, net delta CD about zero. In sizing it is worth
  7 lb. A first form (lift tilted forward by the swirl angle) gave a swirl
  thrust eight times the swirl energy flux and was replaced by the
  energy-bounded form.
- **Coupling:** with a blown wing, each airplane-mode flight point gets a rotor
  thrust variable and the equality n T = D + W sin(gamma) (thrust depends on
  drag, drag on thrust), not an iteration.
- **Stall:** the flight point and trajectory pass the point's `aero` to
  `alpha_stall_deg`, which for the build-up is alpha + (cl_max - CL)/CL_alpha:
  the bound is exactly CL <= cl_max with no extra AeroBuildup runs (the first
  version linearized AeroBuildup through two extra runs per point; that tripled
  the graph and the solve time).
- **Download** (`download.py`): NDARC form DL/T = C_D,v S_immersed / A with
  the flap-projected chord; C_D,v = 0.846 reproduces the XV-15's 7 %. Halo
  reference geometry: 6.8 %. In sizing it couples wing chord and rotor radius.
- **Halo, plan 026 requirements (900 kg, 210 kt):**

  | Aerodynamics | MTOM | L/D cruise | cruise | wing | fuel | battery | solve |
  |---|---|---|---|---|---|---|---|
  | simple (reference) | 14,247 lb | 7.99 | 148 kt | 22.2 m2 | 984 kg | 62.7 kWh | 43 s |
  | buildup | 13,639 lb | 9.84 | 163 kt | 21.4 m2 | 846 kg | 55.8 kWh | 112–164 s |
  | buildup, no blowing | 13,646 lb | 9.82 | 163 kt | 21.4 m2 | 847 kg | 55.9 kWh | |
  | scholz | 13,702 lb | 9.55 | 161 kt | 21.5 m2 | 862 kg | 56.5 kWh | 8 s |

  The engine-out end voltage, whirl-flutter torsion at the 210 kt point and
  the hover motor power still bind; the 210 kt turbine power no longer does.
  - **Max payload at 210 kt (buildup):** 1,842 kg at 19,845 lb (959 kg with
    `SimpleAerodynamics`, plan 026). The 210 kt drag was what limited payload;
    with it relaxed, the climb and take-off-hover turbine power, the hover
    motor and whirl flutter bind instead. This large swing comes from the
    drag estimate (the 0.8 m2 guess against the 3.00 ft2 XV-15 value), so the
    drag allowance is now the most important aerodynamic input.
  - **Max speed at 900 kg (buildup):** 230 kt closes at 13,766 lb; 250 kt does
    not solve (IPOPT failure from the 230 kt start; not proven infeasible).
- **Solve time:** one AeroBuildup point is about 45 ms numerically; the Halo
  sizing grows from about 11–45 s per coupled solve to about 110–160 s
  (hessian evaluations dominate; `expand=True` and the smaller NeuralFoil
  models did not help). Without `initial`, the build-up first solves the
  Scholz problem as its starting point (from the generic guess the
  constant-battery build-up problem stopped at local infeasibility). A fitted
  surrogate was not needed.
- **Trajectories:** the Tier 14 problems solve with the build-up (unblown
  polar, vectorized over nodes): minimum-energy transition 21.5 s and
  8.5 kWh, time to climb 222 s, in 20–75 s instead of about 1 s.
- **Verification:** 431 unittest cases (37 new), Tier 21 notebook 26/26
  checks.
- **Deferred:** V-tail, conversion segments and the blown wing in conversion,
  trim drag, transition location as a calibration against XV-15 data.

## Tier 15: electrical layer

Plan 023. An optional layer between the machines, the battery and the bus:
`Inverter` (motor drive and generator active rectifier), `Cable` (DC
feeder), `ProtectionUnit` (contactors and fuses) and an optional
`DcDcConverter`. It is switched by `HaloAssumptions.electrical_layer`,
**off by default**, so the reference (900 kg, AFDD tiltrotor wing,
14,247 lb) is unchanged.

**Models.**

- **Converter losses:** fixed + switching (|P|) + conduction ((P/V)^2),
  98.5 % at rated power and nominal voltage, peak efficiency at 45 % load.
  Inverters 20 kW/kg (NASA EAP goal 19 kW/kg), DC link at most 0.75 x the
  semiconductor blocking voltage (1,200 V class: 900 V).
- **Cables:** aluminium at 3 A/mm2 of the design current (125 % of the
  source rating), length 1.25 x half span (bus in the fuselage, machines
  in the tip nacelles), battery feeder 3 m. Insulation sized so that
  Dakin's PD inception voltage at the 13,000 ft ceiling is 1.5 x the
  peak pack voltage (minimum wall 0.25 mm); a partial-discharge margin is
  checked at every flight point's pressure.
- **Protection:** two poles per feeder, 0.2 kg + 1.3 g/A per pole, 0.15 V
  per pole at rating.
- **Flight point:** the power balance and the battery share are on the bus
  side; the battery feeder's series drop is exact.
- **Machines:** split into bare machine + inverter so that the Tier 13
  integrated calibration is kept where it was anchored: 17.6 N.m/kg
  (magniX at 200 rad/s) and a 20 kW/kg bare high-speed cap (the old
  integrated 10 kW/kg). The implied bare cap is above NASA's HEMM
  16 kW/kg, so the Tier 13 cap was optimistic (sensitivity in the
  notebook).
- **Installation factor:** no longer passed by the Halo; the cabling and
  protection it stood for are explicit. Cooling stays for Tier 19.

**Halo with the layer** (756 V nominal, 210s pack, 1,200 V inverters):

- The minimum-mass problem at 900 kg is infeasible on the fixed
  2 x 1,120 hp engines.
- Maximum payload 567 kg (against 959 kg without the layer) at 6,304 kg
  (13,898 lb).
- Electrical items 320 kg: inverters 73 + 84 kg, cables 60 + 69 + 9 kg,
  protection 10 + 11 + 4 kg (motor, generator, battery feeders, both
  sides).
- Losses about 17 kW in cruise (inverters dominate; cables about 1/5).
- The payload loss is the 320 kg of items, their converter losses on
  the turbine-limited 210 kt and engine-out points, and the snowball.

**Bus voltage (maximum payload, discrete enumeration):**

| Nominal bus | Pack | Inverter class | Max payload | Electrical mass |
|---|---|---|---|---|
| 540 V | 150s | 1,200 V | 501 kg | 378 kg |
| 756 V | 210s | 1,200 V | 567 kg | 320 kg |
| 800 V | 222s | 1,700 V | 574 kg | 312 kg |
| 1,000 V | 278s | 1,700 V | 604 kg | 286 kg |
| 540 V DC/DC | 210s | 1,200 V | 484 kg | 375 kg |
| 756 V DC/DC | 210s | 1,200 V | 530 kg | 337 kg |
| 800 V DC/DC | 210s | 1,200 V | 536 kg | 332 kg |
| 1,000 V DC/DC | 210s | 1,700 V | 557 kg | 315 kg |

- Higher voltage wins: conductor mass falls as 1/V while the PD-sized
  insulation stays small. The model has no penalty for 1,700 V devices
  or higher-voltage contactors (deferred).
- 800 V nominal (932 V maximum) already exceeds the derated 1,200 V class.
- A DC/DC converter (64 kg, 2 % loss) never pays here: it buys a constant
  bus voltage, but the pack can set the bus directly.

**Solver practice.** The equivalent-circuit maximum-payload solve is
sensitive to its start. `solve_halo_max_payload` starts from a
constant-battery design and falls back to the constant-battery maximum
payload of the same assumptions; `enumerate_bus_voltage` retries a failed
option from its neighbours; from a constant-battery start the parallel
count is energy-matched (`count_parallel_guess`). Feeder ratings at 125 %
of their sources also keep the cable and contactor current limits from
duplicating the source's own limit (degenerate active constraints made
IPOPT report local infeasibility).

## Verification notebooks

One executed notebook per tier under `notebooks/`: Tier 0 foundation checks,
Tier 1 component physics, Tier 2 topology, Tier 3 compatibility margins, Tier 4 mass closure, Tier 5
aerodynamics, Tier 6 stability and control, Tier 7 requirements, Tier 8 missions, Tier 9 coupled sizing, Tier 10 XV-15 mass validation, Tier 20 tiltrotor wing weights and Tier 21 aerodynamics. Outputs are kept so plots render
remotely.

## Next stage

Tier 11 candidates:

- airplane-mode rotor efficiency and cruise-power validation (the largest
  remaining assumption);
- compressibility drag (cruise is about Mach 0.4);
- the conversion corridor;
- dual-wound motor and rotor-loss modelling;
- engine decks;
- BEM rotors.
