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

## Assumptions

- Coolant 50 % water-glycol, properties constant at the mean temperature; air properties at the core inlet.
- Aluminium core (2,700 kg/m³, 190 W/(m K)); fin 0.102 mm, plate 0.2 mm, case 2 mm (OpenConcept defaults).
- Diffuser, nozzle and fan masses are not yet in this plan; the coolant loop gap stays in the ledger.

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
