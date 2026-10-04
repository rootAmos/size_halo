# Thermal: heat loads, heat exchanger, cooling drag and short-time ratings

Status: COMPLETED 2026-10-04. Tier 19 (roadmap "19 Thermal", review item 3).

## Goal and scope

- **Heat loads per flight point** from the losses the models already compute:
  motors, rotor gearboxes, generators, generator step-up gearboxes and the
  battery. Each load is a named `HeatLoad(source, power_W, count)`, so Tier 15
  inverter and cable losses plug in by name without changing the thermal
  code.
- **Ram-air liquid heat exchanger** (`RamAirHeatExchanger`): mass per watt
  rejected at a reference coolant-to-air temperature difference; air mass
  flow from the heat and the point's temperature difference; air-side
  pumping power from a quadratic pressure-drop law. In airplane mode the ram
  pressure supplies the pumping power (cooling drag); in hover fans do (an
  electrical load on the bus). Its rating is a design variable and every
  point must stay within it (a margin, no loop).
- **Short-time ratings from thermal mass:** a lumped-capacitance temperature
  state for the motors, generators and battery pack, propagated in closed
  form over each segment or sub-segment and constrained at each end. The
  heat a component passes to the coolant is (T - T_c) / R, so thermal mass
  absorbs short peaks before the exchanger sees them. The continuous rating
  is the steady state at the machine's rated power and rated speed; hover and
  engine-out peaks may exceed it for their duration.
- **Switch:** `HaloAssumptions.thermal_model` (default False). The reference
  stays at 900 kg, 210 kt, 13,639 lb.
- **Not in scope:**
  - Tier 15 inverter and cable losses (only the plug-in point);
  - the battery's coupling of its temperature into resistance (see
    Deferred);
  - vapour-cycle cooling, coolant-loop dynamics, the Meredith thrust
    recovery, and engine (turboshaft) heat rejection.

## References

- F. W. Meredith, "Cooling of aircraft engines with special reference to
  ethylene glycol radiators enclosed in ducts", ARC R&M 1683, 1935: a ducted
  radiator's drag power equals the air-side pumping power (flow x total
  pressure loss / density), less any heat-addition thrust recovery.
- H. Kellermann, M. Lüdemann, M. Pohl, M. Hornung, "Design and optimization
  of ram air-based thermal management systems for hybrid-electric aircraft",
  *Aerospace* 2021, 8(1), 3, https://doi.org/10.3390/aerospace8010003: ram-air
  TMS with heat exchangers, ducts, pumps and fans; ram-air cooler sized at
  hot-day take-off; a wet air-to-liquid heat exchanger of about 0.62 kW/kg.
- A. Potamiti et al., "Thermal management system design for a series
  hybrid-electric propulsion architecture", *The Aeronautical Journal* 128
  (2024): a centralised TMS of 158 kg for about 250 kW (about 1.6 kW/kg),
  heat exchangers 80 % of it; effectiveness 0.95 assumed there.
- Incropera and DeWitt, *Fundamentals of Heat and Mass Transfer*, ch. 5:
  the lumped-capacitance solution
  T(t) = T_ss + (T_0 - T_ss) e^(-t/tau), T_ss = T_c + Q R, tau = R C.
- McDonald, AIAA 2015-1676 (machine losses, plan 005); magniX torque density
  (plan 018).

## Assumptions

- **Heat exchanger (assumed, cited ranges):**
  - 1.0 kW/kg at a 40 K coolant-to-air difference (between the 0.62 kW/kg
    wet heat exchanger of Kellermann et al. and the 1.6 kW/kg system of
    Potamiti et al.); this covers core, ducts, fans, pump and coolant.
  - Mass scales with the heat rejected at the reference difference; at a
    point the required rating is Q x 40 K / (T_coolant - T_ambient).
  - Coolant at 60 C into the heat exchanger (one water-glycol loop).
  - Effectiveness 0.8 on the air side, so air flow
    m = Q / (eps c_p (T_coolant - T_ambient)).
  - Air-side total-pressure loss 1,000 Pa at the rated flow and sea-level
    density, scaling as m^2 / rho.
  - Pumping power P = m dp / rho. Airplane mode: drag D = P / V (ram).
    Hover: fan power P / 0.6. The Meredith recovery is not credited
    (conservative).
- **Heat sources:** motors, generators and the battery (chemical minus
  terminal power) reject heat through the one exchanger. The gearboxes'
  heat is reported but goes to their own oil coolers, assumed inside the
  XV-15-calibrated AFDD drive weights (`sources_excluded`). The turboshafts
  reject their heat in the exhaust.
- **Machines (lumped capacitance):**
  - Effective specific heat 500 J/(kg K) on the whole machine mass.
  - Mean winding temperature limit 150 C (class H insulation is 180 C hot
    spot), coolant 60 C.
  - Thermal resistance from the continuous rating: the steady state at rated
    power and rated (peak-efficiency) speed is exactly the limit.
  - Losses do not depend on temperature.
- **Battery pack (lumped):** 1,000 J/(kg K) on the pack mass, 60 C limit,
  its loop at the 25 C managed cell temperature; R from the rated discharge
  current at SOC 0.5 with the polarization developed. Moving the pack's
  heat into the 60 C loop needs a chiller, whose power is not modelled. The
  current rating stays as a separate limit.
- **Thermal history in the Halo:** the mission starts at the coolant
  temperatures; temperatures propagate through every sub-segment. The
  engine-out hover continues from the end of the take-off hover (the engine
  fails at MTOM in the take-off hover). The hot-day hover continues from the
  end of the mission. The 4,000 ft hover requirement is a 60 s hover from
  the coolant temperature. Airplane-mode requirement points are steady
  (continuous).
- **Ratings with thermal on:** the machine power margins give way to the
  temperature limits; torque and speed limits stay. The rotor gearbox
  gets its own rating (design variable `power_rated_gearbox_W`), so the motor
  can be sized below the hover power while the drive is not.

## Interfaces

- **`powertrain/components/thermal.py`:** `LumpedThermalModel(
  specific_heat_J_kg_K, temperature_max_C, temperature_coolant_C)` with
  `temperature_steady_C`, `time_constant_s` and `temperature_end_C`.
- **`powertrain/components/heat_exchanger.py`:** `RamAirHeatExchanger(
  power_rated_W, specific_power_W_kg, temperature_coolant_C,
  delta_temperature_ref_C, effectiveness, pressure_drop_ref_Pa,
  efficiency_fan)` with `get_mass`, `get_limits` and `evaluate(power_heat_W,
  atmosphere, velocity_m_s, fan=False)` -> `HeatExchangerResult`.
- **`thermal_model` on `Motor`, `Generator`, `Battery` and
  `EquivalentCircuitBattery`:** None by default (no thermal state; the power
  margin stays). `LumpedThermalModel.temperature_mean_C` gives the
  interval-mean temperature.
- **`thermal/` package (discipline):** `HeatLoad`, `total_heat_W`,
  `thermal_parameters`, `PointThermal`, `evaluate_point_thermal` and
  `coolant_temperatures_C`.
- **`vehicle/powertrain_installation.py`:** `InstalledCooling(heat_exchanger,
  x_m, z_m, sources_excluded=())` and `PowertrainInstallation.cooling`
  (None).
- **`build_flight_point(..., temperature_start_C=None)`:** `FlightPoint`
  gains `heat_loads` and `thermal`. With a cooling installation, an
  airplane-mode point gets a cooling-drag variable and the fan power joins
  the bus demand.
- **`build_mission(..., thermal_start="steady")`:** "coolant" or a dict of
  start temperatures by instance name; `MissionResult.temperatures_end_C`.
- **`compatibility.operating_margins`:** a machine with a thermal model has no
  power-rating margin.
- **Halo:** `HaloAssumptions.thermal_model` and the thermal fields;
  `HaloDesign.power_rated_heat_exchanger_W`, `power_rated_gearbox_W`;
  `HaloSizingResult.thermal_trace`, `heat_exchanger`.

## Symbolic considerations

- The heat load is an explicit expression in the existing point variables.
- Cooling drag depends on the heat, the heat on thrust, thrust on drag: one
  Opti variable per airplane-mode point and an equality, no iteration.
- Fan power enters the existing bus equalities (battery current and
  generator torque are already variables).
- Temperature propagation is closed form; duration may be symbolic (cruise).
  Python branches only on None start temperatures and the numeric zero
  duration.

## Tests

- **`tests/powertrain/test_thermal.py`** (19): exponential response and
  steady state, one time constant, zero loss and zero duration, cooling from
  above, exact chaining, the mean-temperature energy balance, arrays,
  symbolic short-time rating through `asb.Opti`; machine and battery
  continuous ratings on the limit; the power margin replaced by temperature;
  heat exchanger zero-heat limit, rated-point identities, trends (Q^3, 1/V,
  1/rating^2, hot day), symbolic rating; heat loads by name.
- **`tests/performance/test_flight_point_thermal.py`** (12): loads equal the
  component losses; no cooling gives an unchanged point; cooling drag adds
  exactly to the thrust; hover fan power joins the bus demand; rating
  margin; excluded sources; exchanger mass; temperatures and margins;
  thermal mass absorbing a cold-start peak; mission chaining; start options.
- **`tests/integration/test_halo_thermal.py`** (8): off by default; the
  switch; the sizing closes; the hot-day hover sizes the exchanger; drag in
  flight and fans in hover; temperatures within limits; a hover peak above
  the continuous rating; the pack absorbing the engine-out heat.

## Acceptance

- Full unittest suite passes; the reference unchanged (13,639 lb).
- Notebook `notebooks/tier19_thermal/thermal_verification.ipynb` executed.

## Progress and decisions

- 2026-10-04: plan written; implemented.
- **First Halo result** (machines only, battery heat straight to the
  exchanger): 14,645 lb. The engine-out hover put up to 165 kW of battery
  loss into the exchanger (230 kg). Decision: give the pack a lumped state
  too and let every lumped component pass (T - T_c) / R to the coolant.
- **Gearbox heat:** excluded from the exchanger (oil coolers inside the AFDD
  drive weights).
- **Starting point:** with no `initial` the thermal problem first solves the
  thermal-off problem. With cooling installed the per-point battery current
  starts at 0 A (50 A drove the coulomb-counted SOC chain outside the cell
  data, and the battery loss through the cubic pumping law gave an initial
  infeasibility near 1e5; IPOPT stalled in restoration). The reference path
  keeps 50 A.
- **Result (thermal on, 900 kg, 210 kt):** 14,037 lb (+398 lb), from both
  the thermal-off and the thermal-Scholz starts. Exchanger 133 kg, sized by
  the hot-day hover (dT 25 K). Motors 570 kW continuous (hover requirement
  at 1.24 x), machines 63 kg lighter. Cooling drag 1–10 N; fan power up to
  5.6 kW. Maximum payload at 210 kt: 1,651 kg (1,842 kg off).
- The default stays off (the user decides at merge).
- **Verification:** 470 unittest cases pass (431 before, 39 new; 1 skipped
  as before); notebook 17/17 checks plus the full suite; the reference
  reproduces 13,639 lb.

## Deferred

- Pack temperature into the cell resistance (warmer cells sag less: the
  present 25 C resistance is conservative) and the chiller power for the
  battery loop.
- Vapour-cycle cooling for a battery loop below ambient on a hot day.
- Meredith thrust recovery; exit-flap drag; installation of the cooler in
  the nacelle.
- Inverter and cable heat (Tier 15 adds the loads by name).
