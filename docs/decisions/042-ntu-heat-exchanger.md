# Plan 042: compact heat exchanger by the ε-NTU method, with a ram-air duct

## Goal

Replace the mass-per-watt ram-air heat exchanger (plan 028: mass = rating / 1 kW/kg, fixed air-side
effectiveness 0.8, quadratic pressure-loss law, no heat-addition thrust) with a sized compact heat exchanger and
duct, so the sizing trades core size, intake and nozzle area, fan power in hover and cooling drag in cruise
directly. The heat exchanger is sized by the hot-day hover today, so the model sits on a binding constraint.

In: an offset-strip-fin plate-fin core (geometry, Manglik-Bergles j and f, fin efficiency, UA, ε-NTU, pressure
drop, mass); a compressible ram-air duct (diffuser, core loss and heat addition, nozzle, net force, so the
Meredith effect follows from the momentum balance); a hover fan; tests. Out (next): the core and duct as
design variables inside `solve_halo_sizing` behind `heat_exchanger_model="ntu"`, with the mass-per-watt model
kept as the default until the reference is re-solved; coldplates, pump and pipes (the coolant side stays lumped).

## References

- Kellermann, Lüdemann, Pohl, Hornung, "Design and Optimization of Ram Air-Based Thermal Management Systems for
  Hybrid-Electric Aircraft", *Aerospace* 8(1), 3, 2021: system architecture, multipass cross-flow core in
  overall counterflow, Mach-dependent diffuser loss, nozzle loss, sensitivities (about 1.5 % fuel burn per
  1,000 kg and 3.6 % per 1,000 N for their 180-seat aircraft), hot-day take-off as the hardest case, puller fan.
- Brelje, Jasa, Martins, Gray, "Development of a Conceptual-Level Thermal Management System Design Capability in
  OpenConcept", NATO STO-MP-AVT-323-21: the offset-strip-fin implementation followed here
  (`openconcept/thermal/heat_exchanger.py`, `ducts.py`, MIT licence); time-accurate temperatures matter (already
  carried here by the lumped machine and pack states of plan 028).
- Manglik and Bergles, "Heat transfer and pressure drop correlations for the rectangular offset strip fin compact
  heat exchanger", *Exp. Therm. Fluid Sci.* 10, 1995: j and f.
- Kays and London, *Compact Heat Exchangers*, 3rd ed.: ε-NTU for cross flow, entrance and exit loss coefficients.

## Model

**Core (`aircraft_closure.powertrain.components.compact_heat_exchanger`).** Offset strip fins on both sides, cross flow, both
fluids unmixed. Geometry from channel width, height and fin length per side, fin, plate and case thickness,
and the continuous counts n_wide (cold channels across), n_long (cold fin lengths in the flow direction), n_tall
(layers), so the core size is three smooth design variables:

- frontal height H = n_tall (2 (t_f + t_p) + h_c + h_h), width W = n_wide (t_f + w_c), cold flow length
  L = n_long l_c;
- hydraulic diameters, free-flow and heat-transfer areas, fin area ratio per side (OpenConcept's expressions);
- j = 0.6522 Re^-0.5403 α^-0.1541 δ^0.1499 γ^-0.0678 (1 + 5.269e-5 Re^1.34 α^0.504 δ^0.456 γ^-1.055)^0.1,
  f = 9.6243 Re^-0.7422 α^-0.1856 δ^0.3053 γ^-0.2659 (1 + 7.669e-8 Re^4.429 α^0.920 δ^3.767 γ^0.236)^0.1;
- h = j Re Pr^(1/3) k / d_h; fin efficiency tanh(m L)/(m L) with m = sqrt(2 h / (k_f t_f)) over the half
  channel height; overall surface efficiency η_o = 1 - (A_f/A)(1 - η_f);
- UA = 1 / (1/(η_o h A)_c + 1/(η_o h A)_h) (fouling optional, applied per side: OpenConcept's sum repeats the
  hot-side term, not copied);
- ε = 1 - exp(NTU^0.22 / C_r (exp(-C_r NTU^0.78) - 1)) (cross flow, both unmixed);
- core pressure drop per side Δp = G² / (2 ρ) (K_c + 4 f L / d_h + K_e) (K_c 0.3, K_e -0.1 as OpenConcept);
- mass = aluminium density x (plate + fin + case volume) plus the coolant held in the hot side.

Smoothness: C_min/C_max by a softmin over the two capacity rates (both fixed by the operating point and the
pump, so in practice the air side is C_min); no other branches.

**Duct (`aircraft_closure.powertrain.components.ram_air_duct`).** One-dimensional, steady, ideal gas:

- freestream total conditions; inlet total-pressure recovery η_inlet (input, default 0.98) and a diffuser loss
  coefficient on the core face dynamic pressure;
- core: total-pressure drop from the core model at the face mass flux; total temperature rise Q / (ṁ c_p)
  (the heat addition);
- nozzle: isentropic expansion from the core exit total state to ambient static pressure, with a loss
  coefficient; exit velocity from the total-to-static ratio; a smooth cap at sonic (Halo's speeds keep it
  subsonic);
- mass flow: an implicit unknown per point with the residual "nozzle area x exit mass flux = ṁ" (an Opti
  variable in the sizing, a 1-D root in the standalone check);
- net force F = ṁ (V_exit - V_∞) (nozzle exit at ambient pressure); drag = -F. Heat addition raises the exit
  velocity, which is the Meredith effect, with no separate term;
- hover (V_∞ = 0): a fan adds total pressure Δp_fan; fan electrical power ṁ Δp_fan / (ρ η_fan η_motor).

**Interface.** Same as `RamAirHeatExchanger`: `get_mass`, `get_limits`, `evaluate(power_heat_W, atmosphere,
velocity_m_s, fan)` returning `HeatExchangerResult` (drag, fan power, air mass flow, pressure drop), plus a
heat-rejection margin: the heat the core rejects at the point's flows and inlet temperatures must cover the heat
load.

## Sizing option (`HaloAssumptions.heat_exchanger_model = "ntu"`)

The mass-per-watt exchanger stays the default (`"mass_per_watt"`), so the reference is unchanged. With `"ntu"`
the installed cooler is `NtuRamAirHeatExchanger` (`powertrain/components/ntu_heat_exchanger.py`): the core and
duct above, a fan rating, and labelled mass allowances.

- **Design variables** (replace `power_rated_heat_exchanger_W`): the core counts `count_wide`, `count_long`,
  `count_tall`; the nozzle area (a fixed-geometry duct, one area for every point); the fan rating.
- **Per thermal point** (created by `build_flight_point`, which already owns the point's variables): an air mass
  flow with the duct residual (nozzle flow = assumed flow) constrained to zero; in hover a fan total-pressure rise
  (>= 0). Airplane-mode points have no fan: ram pressure alone drives the flow.
- **Heat rejection.** The coolant leaves the exchanger at the loop temperature (60 C, the temperature the lumped
  machine models reject to) and enters it at T_c + Q/(m_c c_c). The core must reject the end-of-interval heat Q
  (the peak within the interval, as for the old rating margin): eps C_min (T_c + Q/C_c - T_air) >= Q, written as
  (eps C_min (T_c - T_air) - Q (1 - eps C_min / C_c)) / 100 kW >= 0, with T_air the freestream total temperature.
  This is a capacity inequality: the loop's valve or bypass passes the actual heat. The coolant mass flow is a
  fixed assumption, 2.0 kg/s of 50 % water-glycol (a 16 K coolant rise at the 121 kW climb peak); the coolant-side
  pressure drop is reported, the pump's power is not charged (its mass is in the allowance).
- **Cooling drag** in airplane mode is the duct's net force (negative = thrust, the Meredith effect), so the
  point's cooling-drag variable loses its lower bound of zero under this option. Heat in the duct is the
  interval-mean heat, as the old drag law used.
- **Fan power** in hover: m dp_fan / (rho_face eta), eta = 0.6 fan and motor (the old model's value), added to
  the bus demand like the old fan power; it must stay below the fan rating.
- **Mass** = core (material + coolant in it) + allowances, each a separate item in the exchanger's mass breakdown:
  - ducts (inlet, diffuser, nozzle): 12 kg per m2 of core face. Basis: about 1.2 m of duct whose wetted area is
    about 6.7 x the face area for a 0.9 x 0.6 m face, at 1.8 kg/m2 (1.5 mm composite with stiffeners);
  - fans: rating / 2.0 kW/kg (electric fan, motor and controller; small ducted electric fans are 2-4 kW/kg);
  - pump, pipes and loop coolant outside the core: 30 kg fixed (a 6 kg pump; about 26 m of 25 mm line to the
    tips and back at 0.9 kg/m, tube plus glycol).
  These are the author's estimates, not from Kellermann 2021 (whose mass equations are not published) or
  OpenConcept (which carries no duct or fan mass); they are kept explicit so they can be replaced.
- **Duct margins** at every thermal point: the core-exit total pressure (after the diffuser and core losses, with
  the fan) at or above ambient, so the nozzle can expel the flow; and the nozzle no larger than the core face. The
  duct keeps its pressure finite for iterates whose losses exceed the inlet pressure (a smooth floor at 10 % of
  ambient); without the first margin that floor and the expansion floor let the optimizer "pass" the flow at a
  vanishing exit velocity through a huge nozzle (found in the second full solve, see the log).
- **Positivity.** The per-point air flow, the core counts, the nozzle area and the fan rating are log-transformed
  Opti variables: an Opti lower bound is a constraint that iterates may violate, and the core's fractional
  Reynolds powers return NaN for a negative flow (found in the third full solve).
- **Trajectory** (plan 036): only the mass-per-watt model is supported there for now (it raises otherwise).

## Verification (tests/thermal)

- ε-NTU: ε → 1 - exp(-NTU) as C_r → 0; ε → 0 as NTU → 0; ε increases with NTU; bounded by the counterflow value.
- j and f: Manglik and Bergles' published values for one of their reference cores at Re 500 and 3,000 (within
  a few per cent, since the correlation is the fit).
- Geometry and mass: a hand calculation for a small core.
- Energy balance: Q = C_air (T_air,out - T_air,in) = C_cool (T_cool,in - T_cool,out).
- Duct: no heat and no losses gives zero net force; losses give drag; adding heat at fixed losses reduces drag
  (Meredith); in hover the fan power equals the flow work over the efficiencies.
- CasADi: every output is an expression in the core counts, nozzle area and mass flow inside `asb.Opti`.
- Reference case: Kellermann's 30 % split design point (96.3 kW, top of climb) as an order-of-magnitude check on
  core size and mass.
- Sizing option (`tests/powertrain/test_ntu_heat_exchanger.py`, `tests/performance/test_flight_point_ntu_cooling.py`,
  `tests/integration/test_halo_ntu_cooling.py`): the mass items add up with the stated allowances; the heat Q* that
  zeroes the margin is what the core rejects with the coolant entering at T_c + Q*/C_c and leaving at T_c; more air
  rejects more and costs more pressure; the duct-pressure margin is negative in hover without the fan and
  positive with it; a small `asb.Opti` sizes core, nozzle and fan for a hot-day hover and a cruise point (hover
  heat and fan margins bind, both residuals zero, under the old 127 kg); in a flight point the duct residual is
  zero, the core covers the heat, the extra thrust equals the duct drag and the fan power joins the bus demand;
  the Halo build installs the option's cooler and keeps the mass-per-watt one by default.

## Assumptions

- Coolant 50 % water-glycol, properties constant at the mean temperature; air properties at the core inlet.
- Aluminium core (2,700 kg/m³, 190 W/(m K)); fin 0.102 mm, plate 0.2 mm, case 2 mm (OpenConcept defaults).
- Sizing option: duct, fan and pump/pipe allowances as above (author estimates); coolant flow fixed at 2.0 kg/s;
  pump power not charged (about 0.6 kW at the solved 153 kPa coolant-side loss, 50 % pump efficiency); a
  fixed-geometry duct (one nozzle area; no exit flap), no inlet spillage drag, fan efficiency 0.6 as before.

## Progress log

- 2026-10-10: plan written; references read (Kellermann 2021 full text, OpenConcept source). Kellermann's
  heat exchanger mass equations are not in the paper (it defers core sizing to Kays and London and Shah and
  Sekulić); the mass model follows OpenConcept's material volumes plus the coolant held in the core.
- 2026-10-10: core and duct implemented in `powertrain/components/compact_heat_exchanger.py` and
  `powertrain/components/ram_air_duct.py` (next to `RamAirHeatExchanger`, which stays the default); 18 tests in
  `tests/powertrain/test_compact_heat_exchanger.py` and `test_ram_air_duct.py` pass. The j and f check is
  against the correlation's laminar asymptote and its smoothness through transition, not tabulated test data.
  Halo-scale results: a 0.87 x 0.62 x 0.09 m core rejects about 114 kW on the hot day at 8 kg/s of air for
  about 19 kg of core and 0.7 kPa of air-side loss; in cruise (85 m/s, 10,000 ft) 80 kW costs about 51 N of
  drag. Next: the `heat_exchanger_model="ntu"` option in the sizing (core counts and nozzle area as design
  variables, one air-flow variable per thermal point, hover fan pressure rise per hover point), then a
  re-solve of the reference.
- 2026-10-10/11: sizing option `heat_exchanger_model="ntu"` built (above); 14 new tests pass, with the existing
  thermal, flight-point thermal, hot-day, duct and core modules (83 tests in the touched modules, including the
  trajectory thermal tests; full suite 700 tests OK, 8 skipped). Full solves from the cached reference (15,179 lb), one slot each:
  1. NTU: stopped on NaN (`Invalid_Number_Detected`, 283 s): iterates with a negative core-exit pressure.
  2. NTU with the pressure floor: converged to 14,292 lb, but on a loophole (a 1 MPa air-side loss, exit velocity
     0.8 m/s through a huge nozzle). Rejected; the duct margins were added.
  3. NTU with the duct margins: iteration limit (2,426 s) with thousands of NaN evaluations from negative flows.
  4. NTU with log-transformed variables: converged in 118 s (reproduced in 112 s by a second run): **14,297 lb,
     -882 lb**, every margin met.
  5. Default with the final code: 15,179.0 lb (unchanged, twice).
  6. Both with relaxed machine units (`integer_units=False`) to split the change: default 14,300 lb (generator
     2.004 units), NTU 13,853 lb (1.945 units): the cooler itself is worth **-447 lb**; the rest (-435 lb) is the
     generator falling from 3 whole units to 2, because the reference sits 0.2 % above the 2-unit threshold.

  NTU result (integer units): cooler 47.6 kg (core 9.2, ducts 4.2, fans 4.3, pump and pipes 30) against 126.9 kg;
  core 1.03 m wide x 0.34 m tall x 0.067 m deep (counts 708 x 11.1 x 21.7), nozzle 0.082 m2, fan rating 8.6 kW.
  Hot-day hover: 3.6 kg/s of air, effectiveness 0.59, fan 8.6 kW (5.4 kW before); the hot-day hover still binds
  (heat rejection and fan rating), and so do the cold-start take-off, landing and engine-out hovers' heat
  rejection. Cruise cooling drag 53 N (2.9 N before; 4.5 kW of propulsive power at 85 m/s): the one nozzle is
  sized for hover, so cruise runs 5.3 kg/s through it, twice what the heat needs; the Meredith recovery is in the
  net (exit 75 m/s). Coolant-side loss 153 kPa. Machine units: motor 2, generator 2 (3 before). Figure:
  `docs/figures/ntu_heat_exchanger.png` (`python -m examples.ntu_heat_exchanger_figure` on the four pickles).
  Not done: an exit flap (per-point nozzle area), which should cut most of the cruise drag; pump power; the
  trajectory with this cooler.

## Slide summary

**A sized ε-NTU cooler weighs 48 kg, not 127 kg, and the Halo closes at 14,297 lb (-882 lb).**

- Cooler 47.6 kg: core 9 kg (1.03 x 0.34 x 0.07 m), ducts 4, fans 4 (8.6 kW), pump and pipes 30 (estimate).
- Hot-day hover still sizes it: fan 8.6 kW (was 5.4 kW); cruise cooling drag 53 N (was 3 N) through a duct whose
  fixed nozzle is sized for hover.
- With machine units relaxed the cooler alone is worth -447 lb; the other -435 lb is the generator dropping from 3
  units to 2, since the reference sat 0.2 % over the threshold.

Caveat: half the saving is a whole-unit step on a knife edge, and the allowances (30 kg pump and pipes, 12 kg/m2
ducts, 2 kW/kg fans) are author estimates; the mass-per-watt model stays the default until they are checked.
