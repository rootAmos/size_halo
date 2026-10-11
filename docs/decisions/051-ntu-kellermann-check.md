# Plan 051: check the ε-NTU cooler against Kellermann 2021, and published mass allowances

Status: COMPLETED 2026-10-11 (check, published-allowance option, tests). No sizing re-solve.

## Goal and scope

The plan 042 cooler (`NtuRamAirHeatExchanger`: an offset-strip-fin core, a ram-air duct and labelled mass
allowances) has been checked only against its own correlations. This plan:

1. sizes our core and duct at the one published system design point we have, Kellermann et al. 2021 (SP 30 %,
   T_cp 380 K, top of climb), and compares at system level: heat rejected, core volume and frontal area, air-side
   pressure ratio, drag, mass;
2. collects published thermal-management-system (TMS) mass breakdowns and, where they give a ratio, replaces the
   author-estimate allowances (ducts 12 kg/m² of core face, fans 2 kW/kg, pump, pipes and loop coolant 30 kg);
3. states the effect on the v3.7 cooler mass and, through the growth factor, on take-off weight.

Out: a full sizing solve (memory is tight; the effect is estimated with the growth factor 2.5), a different core
surface (Kellermann's rectangular microchannels), coldplates.

## References

- Kellermann, Lüdemann, Pohl, Hornung, "Design and Optimization of Ram Air-Based Thermal Management Systems for
  Hybrid-Electric Aircraft", *Aerospace* 8(1), 3, 2021. Design point and Table 2 defaults; Figure 10 (two TMS);
  Section 4.1 dimensions; Section 4.2 hot-day fan power.
- Chapman, Schnulo, Nitzsche, "Development of a Thermal Management System for Electrified Aircraft",
  NASA/TM-2020-220473 (AIAA 2020-0545), NTRS 20200001620: tiltwing TMS component masses (Table 4), pump and fan
  correlations (Eqs. 18 and 19), what is neglected (inlet, nozzle, headers, line walls, coldplates).
- Chapman, Hasseeb, Schnulo, "Thermal Management System Design for Electrified Aircraft Propulsion Concepts",
  NASA/TM-20205011477 (AIAA 2020-3571): loop masses for STARC-ABL and the RVLT tiltwing (Tables 6 and 8).
- Brelje, Jasa, Martins, Gray, NATO STO-MP-AVT-323-21, and OpenConcept (`openconcept/thermal/pump.py`,
  `hose.py`, `heat_exchanger.py`, MIT licence): pump mass 1/450 kg per W of rated power; hose wall and coolant mass
  per metre from diameter, design pressure, allowable stress and hose density; core mass as ours; no duct or fan mass.
  (The NATO PDF refused automated download; the OpenConcept source is the open record of the same models.)

## Method

**Design-point check (`examples/ntu_kellermann_check.py`).** Kellermann's inputs, as close as our model allows:

- flight: FL350 ISA, Mach 0.68 (V = 202.5 m/s); heat 96.3 kW per powertrain; one TMS;
- coolant temperatures from Table 2 defaults: coldplate coolant inlet T1 = 275 K (= exchanger coolant outlet),
  coldplate effectiveness 0.4 at T_cp 380 K, so the coolant enters the exchanger at 275 + 0.4 (380 - 275) = 317 K;
- capacity rates: C*_R = 1.0 (Table 2), so C_air = C_coolant = 96.3 kW / 42 K = 2.29 kW/K (2.28 kg/s of air);
- 50 % water-glycol at the loop mean (296 K): cp 3,300 J/(kg K), k 0.38 W/(m K), μ 3.8 mPa s, ρ 1,070 kg/m³
  (ASHRAE tables, rounded; Kellermann uses CoolProp);
- air-side core pressure ratio Π_c = 0.95 (Table 2 default), imposed as an equality on the core loss over the
  core-face total pressure; coolant-side loss at most 100 kPa (Kellermann's Π_h is optimized, its value not given);
- objective: core mass + duct allowance + 2.4 kg per N of duct drag (Kellermann's fuel-burn sensitivities,
  1.5 % per 1,000 kg and 3.6 % per 1,000 N, Figure 5);
- design variables: our three core counts (log-transformed) and the nozzle area, with the duct residual zero at the
  fixed air flow and the core-exit total pressure above ambient. A small `asb.Opti` problem.

Compared with Kellermann at system level: heat rejected, core L_c x L_h x H_stack (our air flow length x coolant
flow length x stack), volume and frontal area, Π_c, drag (Figure 10: two TMS), mass. Sensitivities: Π_c 0.98 and an
ideal inlet (recovery 1.0), because Kellermann's inlet and diffuser model (Mach-dependent c_p*, spillage) is not ours.

**Mass allowances.** A table of published breakdowns (in the example); where a published ratio exists, an
option `allowances="published"` on `NtuRamAirHeatExchanger` (default stays `"estimate"`, so plan 042 results are
unchanged), passed through `HaloAssumptions.allowances_heat_exchanger`.

## Results

**Design point** (`python -m examples.ntu_kellermann_check`; one TMS, 96.3 kW, FL350 Mach 0.68 = 202.5 m/s, air
at the core 241 K total, coolant 317 to 275 K, 2.28 kg/s of air):

| | Kellermann (rect. microchannels, 2-pass) | ours, Π_c 0.95 | ours, ideal inlet | ours, Π_c 0.98 |
|---|---|---|---|---|
| heat rejected | 96.3 kW | 96.3 kW | 96.3 kW | 96.3 kW |
| L_c (air) x L_h (coolant) x H_stack | 0.48 x 0.73 x 0.18 m | 0.118 x 0.66 x 0.25 m | 0.119 x 0.66 x 0.25 m | 0.089 x 0.97 x 0.26 m |
| core volume | 63 L | 19 L | 19 L | 22 L |
| frontal area | 0.131 m² | 0.164 m² | 0.162 m² | 0.252 m² |
| air-side Π_c | 0.95 (Table 2) | 0.95 (imposed) | 0.95 | 0.98 |
| coolant-side loss | not given | 33 kPa | 33 kPa | 66 kPa |
| duct drag | about 5 N (Fig. 10: 10 N for two) | 25.5 N | 14.0 N | -1.6 N |
| mass | about 55 kg whole TMS (Fig. 10: 110 kg for two) | core 8.0 kg + ducts 2.0 kg | 8.0 + 1.9 kg | 9.1 + 3.0 kg |

Effectiveness 0.554 in every case (the temperatures fix it); air-side Re 2,800 (1,800 at Π_c 0.98), coolant Re
105 to 135 (laminar); nozzle 0.035 to 0.038 m².

What differs because of the core surface:

- **Frontal area agrees within 25 %.** At a given air flow and Π_c the face velocity is set by the pressure
  budget, not the surface: both need about 0.13 to 0.16 m² for 2.3 kg/s.
- **Our core is 4 times shorter in the air direction and a third of the volume.** Offset strip fins (d_h 2.4 mm,
  the boundary layer restarting every 6 mm fin) have a much higher j per unit length than plain rectangular
  channels; Kellermann's optimum d_h,c is 5 to 10 mm (his Section 3.2), giving long channels (L_c 0.48 m) and a
  thin stack. The flow arrangement goes the other way: our single-pass cross flow needs NTU about 1.4 for ε 0.55 at
  C_r 1, against 1.24 in pure counterflow, which his two-pass overall counterflow approaches. The surface wins.
- **Drag is 5 times Kellermann's but the same order** (tens of N; 25 N x 202 m/s is 5 kW of propulsive power for
  96 kW rejected). Most of the gap is our inlet and diffuser model, not the core: with an ideal inlet (recovery 1.0
  instead of 0.95) our drag falls from 25.5 N to 14 N; the rest is our constant diffuser loss (0.3 face dynamic
  pressures) against his Mach-dependent c_p* and nozzle loss. At Π_c 0.98 the Meredith thrust cancels the loss
  (-1.6 N), as Kellermann reports near T_cp 400 K. His optimizer picks Π_c (about 0.94 to 0.96, Fig. 8); we
  impose 0.95.
- **Mass** cannot be compared like for like: Kellermann publishes only the total (about 55 kg per TMS, including
  coldplates, pipes, pump, coolant, diffuser and nozzle, with optimistic 0.5 mm plates and 0.1 mm fins). Our core
  is 8 kg (wet); core plus duct allowance 10 kg; with plan 042's 30 kg pump and pipes, 40 kg without coldplates:
  consistent.

**Published TMS masses** (printed by the example):

| source | system | heat | mass | kW/kg | breakdown |
|---|---|---|---|---|---|
| Kellermann 2021, Fig. 10 | one of two TMS, SP 30 %, T_cp 380 K, top of climb | 96.3 kW | about 55 kg | 1.75 | none published; excludes fan, redundancy (up to +100 % expected), cowlings |
| Chapman et al. TM-2020-220473, Table 4 | 15-pax tiltwing, hover, oil + 4 coolant loops | 741 kW | 176.2 kg (171.63 transient) | 4.2 | HEX (wet) 128.3 (73 %), puller fans 16.6 (9 %), pumps 5.2 (3 %), lines (fluid only) 26.1 (15 %) |
| same, coolant loops only | rectifier, inverters, motors | 488 kW | 127.5 kg | 3.8 | HEX 101.2, fans 12.9 (4 x 3.22 kg for 4 x 44 kW), pump 2.4 (1.18 kg/s), line 11.0 |
| Chapman et al. TM-20205011477, Table 8 | RVLT tiltwing baseline, hover | 361 kW | 54.0 kg | 6.7 | loops only: motor 4 x 7.11, oil 14.55, coolant 11.05 |
| same, Table 6 | STARC-ABL corrected baseline, rolling take-off | 340 kW | 109.3 kg | 3.1 | loops only: oil 2 x 10.08, coolant 2 x 15.30, BLI 58.57 |
| OpenConcept (Brelje et al.) | models, not a design | - | - | - | pump 1/450 kg per W rated; hose wall (D + t) π t ρ with t = D p / (2 σ), plus coolant; no duct or fan mass |
| Halo v3.7 (plan 042) | one cooler, hot-day hover sized | 121 kW peak | 47.6 kg | 2.5 | core 9.1, ducts 4.2, fans 4.3, pump and pipes 30 |

The Halo cooler's 2.5 kW/kg sits inside the published 1.75 to 6.7 kW/kg (Chapman's are optimistic: inlets,
nozzles, headers, line walls and coldplates neglected).

**Allowances.** What the sources support:

- Ducts (12 kg/m² of face): no published ratio (Kellermann's duct masses are not published, Chapman neglects inlet
  and nozzle, "assumed to scale as the core", OpenConcept carries none). Kept.
- Fans (2 kW/kg): Chapman's correlation (mass from air flow, fitted to small Lytron and Oriental Motor cooling fans)
  gives 3.22 kg for a 44 kW fan (13.7 kW/kg); at the Halo's 3.6 kg/s hover flow it gives 1.6 kg against our
  4.3 kg. A 44 kW electric fan with motor and controller at 3 kg is not credible: an extrapolation. Kept.
- Pump: OpenConcept 450 W/kg at a 1.7 kW rating (2 kg/s through 150 kPa of core and 154 kPa of line loss by
  Blasius, at OpenConcept's pump efficiency 0.35) = 3.8 kg; Chapman's 2.4 kg pump at 1.18 kg/s agrees.
- Lines: OpenConcept's hose gives 1.38 kg/m (wall 0.85, coolant 0.53), so 35.9 kg for 26 m, against plan 042's
  0.9 kg/m.

Option `allowances="published"` (`HaloAssumptions.allowances_heat_exchanger`): pump and pipes from OpenConcept's
pump and hose; ducts and fans keep the estimates. On the plan 042 / v3.7 cooler: pump and pipes 30.0 to 39.8 kg,
**cooler 47.6 to 57.4 kg (+9.8 kg)**; with the growth factor 2.5, take-off weight about **+24 kg (+54 lb)**
(14,306 lb to about 14,360 lb, not re-solved). The ε-NTU cooler's saving over mass-per-watt (123 kg) stands.

Not made the default: the difference is almost all hose wall, and OpenConcept's 1.03 MPa design pressure is a code
default, not a sized value. At 0.6 MPa (twice a 0.3 MPa loop) the same hose model gives 0.99 kg/m and pump and
pipes 29.4 kg, which is the plan 042 estimate. Line length (26 m, the author's) is the larger lever either way.
Recommendation: use `"published"` for the next reference re-solve as the conservative, sourced choice.

## Assumptions

- Kellermann's optimized T1, C*_R, Π_c and Π_h are not published; the Table 2 defaults stand in for them.
- Kellermann's Figure 10 is read off the plot: m_TMS about 110 kg and D_TMS about 10 N for two TMS at T_cp 380 K,
  SP 30 % (so about 55 kg and 5 N each); Figure 12 (multi-point, about 106 to 108 kg) agrees.
- Line length 26 m for the Halo (cooler in the fuselage at the wing, supply and return to both tips), as plan 042.

## Progress log

- 2026-10-11: plan written; Kellermann 2021, Chapman 2020 (TM-2020-220473) and 2021 (TM-20205011477) read; the
  NATO paper itself could not be downloaded (HTTP 403), OpenConcept's source read instead.
- 2026-10-11: `examples/ntu_kellermann_check.py` (check, published table, v3.7 allowance effect); option
  `allowances="published"` on `NtuRamAirHeatExchanger` and `HaloAssumptions.allowances_heat_exchanger` (default
  `"estimate"`, plan 042 results unchanged); 11 tests in `tests/powertrain/test_ntu_kellermann_check.py`, one
  assertion added to `tests/integration/test_halo_ntu_cooling.py`. The touched modules pass (43 tests in the new, NTU, core, duct, flight-point and Halo-option modules;
  11 in `test_halo_ntu_cooling` with `test_halo_thermal`). The v3.7 NTU pickle is not cached (only
  its JSON: cooler 48.1 kg), so the effect is evaluated at plan 042's reported geometry (counts 708 x 11.1 x 21.7,
  nozzle 0.082 m², fan 8.6 kW), which reproduces its 47.6 kg.

## Slide summary

**At Kellermann's 96 kW design point our cooler matches the frontal area at a third of the core volume, with
drag of the same order.**

- Same heat, inlet state and Π_c 0.95: face 0.16 against 0.13 m², core 19 against 63 L (strip fins against plain
  microchannels), drag 25 against about 5 N, of which 11 N is our inlet-recovery assumption.
- Published TMS run at 1.7 to 6.7 kW/kg (Halo 2.5); OpenConcept's pump and hose models put pump and pipes at 40 kg,
  not the 30 kg estimate.
- Published allowances: cooler +9.8 kg (57 kg), take-off weight about +24 kg (+54 lb) by the growth factor; an
  option now, recommended for the next re-solve.
