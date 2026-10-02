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

## Verification notebooks

One executed notebook per tier under `notebooks/`: Tier 0 foundation checks,
Tier 1 component physics, Tier 2 topology, Tier 3 compatibility margins, Tier 4 mass closure, Tier 5
aerodynamics, Tier 6 stability and control, Tier 7 requirements, Tier 8 missions and Tier 9 coupled sizing. Outputs are kept so plots render
remotely.

## Next stage

Tier 7 requirements (payload, hover, climb, speed, ceiling), Tier 8 mission
segments and Tier 9 coupled sizing with mission and energy allocation.
