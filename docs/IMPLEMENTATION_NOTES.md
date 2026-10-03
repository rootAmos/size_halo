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

## Verification notebooks

One executed notebook per tier under `notebooks/`: Tier 0 foundation checks,
Tier 1 component physics, Tier 2 topology, Tier 3 compatibility margins, Tier 4 mass closure, Tier 5
aerodynamics, Tier 6 stability and control, Tier 7 requirements, Tier 8 missions, Tier 9 coupled sizing and Tier 10 XV-15 mass validation. Outputs are kept so plots render
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
