# Component interfaces

All components expose get_mass(), get_limits(), evaluate(). Results and limits
are dataclasses; values may be numeric scalars, vectors or CasADi expressions.
Models build equations and do not enforce limits, clip outputs or resize parts.

| Component | Operating inputs | Rating convention |
|---|---|---|
| Motor | speed_rad_s, torque_Nm, voltage_V | rated shaft output |
| Generator | speed_rad_s, torque_Nm, voltage_V | rated shaft input |
| Battery | current_A, soc, duration_s | terminal discharge/charge power |
| SimpleTurboshaft | shaft_power_W | shaft output |
| Gearbox | speed_input_rad_s, torque_input_Nm | shaft input |
| ActuatorDiskPropulsor | axial_velocity_m_s, atmosphere, thrust_N OR shaft_power_W and induced_velocity_m_s (speed_rad_s accepted, ignored) | shaft input |
| MomentumProfileRotor | axial_velocity_m_s, atmosphere, thrust_N, speed_rad_s | shaft input; limits blade_loading_max, mach_tip_helical_max, advance_ratio_max |

Motor/generator default losses follow McDonald, AIAA 2015-1676 (plan 005):
P_L = C0 + C1 w + C2 w^3 + C3 Q^2 with coefficients from the peak-efficiency
speed, torque and value and the parasite loss ratio k0 in [0, 1]
(`McDonaldMotorLossModel`; defaults 400 rad/s, 200 Nm, 0.96, 0.5).
`rubber_machine(...)` builds a machine whose ratings follow the paper's
ratios (Q_rated = kQ Q_hat, P_rated = kP w_hat Q_hat, w_limit = kw w_hat).
C0 is a constant loss present at standstill. The quadratic
`SimpleMotorLossModel` remains available. Neither is calibrated to hardware. Voltage
affects current only at this tier. Loss fidelity is interchangeable. Generator
operation requires shaft power >= losses. Neither machine models regeneration
or bidirectional operation.

Battery OCV is constant. Terminal voltage = OCV - I R, terminal power = V I,
chemical power = OCV I, and SOC_next = SOC - chemical_power dt / capacity.
Positive current discharges, negative charges. Joule losses are always positive.
Mass is the maximum of energy-driven and charge/discharge-power-driven mass.
No SOC saturation. Enforce positive terminal voltage, SOC bounds and terminal
power bounds externally. This is not a cell electrochemistry or thermal model.

Gear ratio = input speed/output speed; output torque = ratio * efficiency *
input torque. No reverse-power model. Engine fuel flow = shaft power /
(thermal efficiency * LHV), with no idle flow, lapse or transient model.

Rotor momentum: T = 2 rho A vi (V + vi), P = T (V + vi) / performance.
Thrust mode computes required power directly, including hover. Power mode
returns a power residual; the caller supplies vi and constrains residual = 0.
No RPM, vortex-ring state, descent, windmilling, installation losses, tip Mach,
blade loading or tilt-transition physics is modeled.

Positive sizes, capacities, voltage, density, specific powers and efficiencies
(0 < efficiency <= 1) are required. Operating domains are documented in the
modules. Invalid physical data is not silently repaired. Max torque and max
speed are independent bounds; caller also applies the rated-power bound.

## Topology (Tier 2)

`core/ports.py`: `Domain` (MECHANICAL, ELECTRICAL, FUEL), `Direction` (IN, OUT),
`PortSpec(name, domain, direction)` and the port values
`MechanicalPortValue(speed_rad_s, torque_Nm)`,
`ElectricalPortValue(voltage_V, current_A)` and `FuelPortValue(fuel_flow_kg_s)`.
Flow (torque, current, fuel flow) is positive in the port's nominal direction.

`core/topology.py`: `Topology.add(name, component, ports, count=1)`,
`add_bus(name)`, `connect(a, b)`, read-only `instances`, `buses`, `connections`.
Direct connections join one OUT and one IN port of equal domain and count; each
port connects once; electrical buses are the only junction. Wiring errors raise
at `connect`. `connection_residuals(topology, port_values)` maps
`"instance.port"` to port values and returns `Residual(label, value)` items:
field differences for direct connections; for each bus, voltage equality to the
first attached port and `sum(sign * count * current) = 0` (OUT +, IN -). It
creates no variables or constraints. Unconnected ports are boundaries.

| Component | Ports (direction) |
|---|---|
| Motor | electrical (IN), shaft (OUT) |
| Generator | shaft (IN), electrical (OUT) |
| Battery | electrical (OUT; positive current discharges) |
| SimpleTurboshaft | fuel (IN), shaft (OUT) |
| Gearbox | shaft_in (IN), shaft_out (OUT) |
| ActuatorDiskPropulsor | shaft (IN) |
| MomentumProfileRotor | shaft (IN) |

Declarations live in `powertrain/ports.py` (`port_specs_for`), not on the
component classes. `powertrain/topologies.py` provides
`build_series_hybrid(..., count_rotors=n)`. Multiplicity is symmetric: n copies
share one set of port values; asymmetric or failed instances are deferred.

## Compatibility margins (Tier 3)

`core/margins.py`: `Margin(label, value)`; `margin_below(label, value, limit)`
= (limit - value) / limit; `margin_above(label, value, limit)` = (value -
limit) / limit. `>= 0` is compatible; 0.1 is 10 % headroom. `margin_report(
margins, value_of)` returns `MarginReportEntry(label, value, compatible)`
sorted most-critical first; it requires numeric values (e.g. `solution.value`).

`powertrain/compatibility.py`:

- `operating_margins(topology, port_values)`: per instance, from the same port
  values used for connection residuals. Motor/generator: shaft power vs rating,
  torque, speed, voltage window. Battery: terminal discharge and charge power.
  Turboshaft shaft power, gearbox input power, rotor shaft power.
- `design_margins(topology)`: ratings only. Per direct shaft connection the
  downstream port must tolerate the upstream maximum, field by field (speed,
  torque, power) where both sides declare it. Per bus: the battery terminal
  range (`battery_voltage_range_V`, at max discharge and max charge power) must
  lie in each machine's voltage window, and loss-free supply must cover demand
  with instance counts. Electrical power bounds are shaft ratings, so the bus
  power margin is optimistic.
- `port_envelope(component, port)`: `MechanicalEnvelope(max_speed_rad_s,
  max_torque_Nm, max_power_W)` (None = no bound) or `ElectricalEnvelope(
  min_voltage_V, max_voltage_V, max_power_W, sets_voltage)`.

Margins never clip or resize. Callers constrain `margin.value >= 0` or report.
Speed/torque envelopes are not yet transformed through gearboxes because no
component downstream of a gearbox declares speed or torque limits.

## Vehicle and mass closure (Tier 4)

Physical components own geometry and return `asb.MassProperties` (point mass
and CG) from `get_mass_properties(...)`; masses are AeroSandbox's Raymer
general-aviation correlations on the component's own `asb.Wing`/`asb.Fuselage`
times a dimensionless `mass_factor` (default 1).

| Component | Geometry inputs | Mass source | CG |
|---|---|---|---|
| Wing, HorizontalTail | area_m2, aspect_ratio, taper_ratio, x_le_root_m, z_m, airfoil | raymer mass_wing / mass_hstab | 40 % MAC |
| VerticalTail | area_m2, aspect_ratio (h^2/S), taper_ratio, x_le_root_m, z_root_m | raymer mass_vstab | 40 % MAC, 40 % height |
| Fuselage | length_m, diameter_m, nose/tail fractions, x_nose_m | raymer mass_fuselage (needs wing-to-tail arm) | 45 % length |
| LandingGear | gear lengths, x_main_m, x_nose_m | raymer main + nose, fixed | mass-weighted |
| Systems | mass_avionics_uninstalled_kg, x_m | raymer flight controls + avionics | stated |
| Payload | mass_kg, x_m, z_m | given | stated |
| PowertrainInstallation | topology, InstalledInstance(name, x_m, z_m) per instance | installation_factor x count x get_mass() | stated |

`StructuralDesignCondition(mass_design_kg, load_factor_ultimate=5.7,
velocity_cruise_m_s, altitude_cruise_m, lift_to_drag_cruise)` carries the design
mass into every correlation. `Aircraft.get_mass_breakdown(condition)` returns
`MassBreakdown` (one `MassProperties` per item, `total()`, `mass_empty_kg()`);
`get_mass`, `get_cg_x_m` and `to_asb()` (an `asb.Airplane`) build on it.
Surfaces are unswept trapezoids; body axes are x aft from the nose, z up.

Mass closure is the caller's explicit equality `mass_takeoff_kg ==
aircraft.get_mass(StructuralDesignCondition(mass_takeoff_kg))`; see
`examples/aircraft_mass_closure.py`. No fixed-point iteration exists. Not
modeled: fuel, inertia tensors, nacelle/tilt-mechanism structure.

## Aerodynamics (Tier 5)

`aerodynamics/simple.py` `SimpleAerodynamics(alpha_zero_lift_deg, cl_max,
thickness_location_chordwise, interference_wing, interference_tail,
interference_fuselage, drag_area_misc_m2)` reads geometry from the physical
components (no vehicle import).

- `evaluate(aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=())`
  returns `AeroResult` (cl, cd, cd0, cdi, cl_alpha_per_rad, oswald_efficiency,
  dynamic_pressure_Pa, mach, lift_N, drag_N, lift_to_drag); reference area is
  the wing planform area.
- CL = 2 pi CL_over_Cl(AR, M) (alpha - alpha_0L) (AeroSandbox; DATCOM).
- `parasite_drag_breakdown(...)`: Cf (AeroSandbox `Cf_flat_plate`) x form
  factor (Raymer 12.30 for surfaces; AeroSandbox fuselage form factor) x
  interference x S_wet / S_ref per component, plus CDA_misc / S_ref.
- CDi = CL^2 / (pi e AR), e from AeroSandbox `oswalds_efficiency`.
- `DragIncrement(label, cd)` is the additive extension hook.
- `alpha_stall_deg(...)` gives the linear-lift angle at CLmax for callers to
  bound alpha. Wing lift only; tails, trim and propeller effects are Tier 6+.

## Stability and control (Tier 6)

`controls/stability.py` builds equations; callers own trim variables and
constraints. `flap_effectiveness(chord_fraction, correction=0.8)` is the
thin-airfoil plain-flap tau times a viscous correction. Tails own
`elevator_chord_fraction` / `rudder_chord_fraction` (0.3).

- `LongitudinalStability(tail_dynamic_pressure_ratio=0.9, cm_ac_wing=-0.09,
  fuselage_pitch_factor_per_deg=0.012, tail_incidence_deg=0)`:
  `neutral_point_x_m`, `static_margin`, `lift_curve_slope_total_per_rad`, and
  `evaluate(aircraft, aerodynamics, x_cg_m, V, h, alpha_deg, elevator_deg)` ->
  `LongitudinalResult(cl_total, cl_wing, cl_tail, cm, downwash_deg, lift_N,
  trim_drag)`; trim drag is a Tier 5 `DragIncrement` (tail induced drag).
  Downwash 2 CL_w/(pi AR); fuselage Cm_alpha from Raymer eq. 16.25.
- `DirectionalStability(...)`: `cn_beta_per_rad` (fin with 1.55 x geometric AR
  end-plate factor plus Raymer eq. 16.47 fuselage term) and
  `rudder_for_yaw_moment_deg`.
- `failed_propulsor_yaw_moment_Nm(thrust, lateral_arm_m, drag_failed=0)`.

Surfaces are unswept (ac at quarter MAC). V-tail, roll control, dynamic modes
and hover/transition control are deferred.

## Flight points and requirements (Tier 7)

`powertrain/topologies.py`: `SeriesHybridSizing` (motor/generator peak
torques, turboshaft rating, battery discharge power and energy, disk area,
rotor count; any field may be an Opti variable) and
`build_series_hybrid_from_sizing` (McDonald rubber machines with kQ 2.5,
kP 1.25, kw 2.5; gearbox rated to the motor; rotor rated to the gearbox output;
rotor mass 3 kg/m2; battery resistance x capacity constant).

`performance/flight_point.py` (orchestration; creates per-point variables):
`FlightCondition(mode "hover"|"airplane", velocity_m_s, altitude_m,
climb_rate_m_s, thrust_to_weight, active_rotor_count, soc,
hybridization_electric, label)` and `build_flight_point(opti, aircraft,
aerodynamics, condition, mass_kg, hybridization_electric=None,
drag_increments=())` -> `FlightPoint` with thrust, rotor power, rotor/motor
speed, motor torque, motor electrical demand, battery and generator results,
fuel flow and labelled Tier 3 operating margins. Hover: thrust = (T/W) W / n.
Airplane mode: lift = W cos gamma (alpha variable), thrust = D + W sin gamma.
Rotor speed is a variable (motor efficiency enters via McDonald); generator
runs at its peak-efficiency speed; h_e in [0, 1] is a variable unless fixed.
Requires the series-hybrid reference instance names.

`requirements/capability.py`: `HoverRequirement`, `ClimbRequirement`,
`SpeedRequirement`, `CeilingRequirement` (each `.flight_condition()`), and
`RequirementSet`. Sustained requirements default to h_e = 0; hover is free.

## Missions (Tier 8)

`vehicle/items.py` `FuelLoad(mass_kg, x_m, z_m)`; `Aircraft.fuel` is optional
and `MassBreakdown.fuel` is always present (zero without a load);
`mass_empty_kg()` excludes payload and fuel.

`mission/segments.py`: `HoverSegment(duration, altitude, h_e)`,
`ClimbSegment(h_start, h_end, rate, V, h_e)`, `CruiseSegment(distance, h, V,
h_e)`, `LoiterSegment(duration, h, V, h_e)`, `DescentSegment(h_start, h_end,
rate, V, h_e)`; each has `duration_s()` and `flight_condition(soc)`; any field
may be an Opti variable; `ground_distance_m(segment)`. Climb/descent use the
mean altitude; hover is T/W = 1.

`mission/mission.py`: `Mission(segments)`, `build_mission(opti, aircraft,
aerodynamics, mission, mass_start_kg, soc_start)` -> `MissionResult(segments,
mass_fuel_burnt_kg, energy_battery_chemical_J, soc_end, mass_end_kg,
duration_s, margins)`; each `SegmentResult` holds its flight point, duration,
start mass, fuel, chemical and terminal battery energy and SOC. Segments use
their start mass; SOC falls by chemical energy / capacity; thrust >= 0 is
enforced. No in-flight charging (h_e in [0, 1]).

## Coupled sizing (Tier 9)

`examples/coupled_sizing.py` `solve_coupled_sizing(objective="mass_takeoff" |
"fuel", requirements, mission, aerodynamics)` is the readable aircraft-level
formulation (deliberately an example script, not a library class): design and
mission variables, all constraints and the objective are written out in one
function. Supporting library changes: `Battery.mass_smoothing_kg` (optional
softmax of energy- and power-sized mass; default exact maximum) and
`SeriesHybridSizing.battery_mass_smoothing_kg`; flight-point equalities are
normalized (lift by weight, bus powers by installed motor power).

## Tiltrotor weights and XV-15 reference (Tier 10a)

`weights/afdd.py`: AFDD rotorcraft weight equations from NDARC Theory
(NASA/TP-2009-215402, ch. 19). SI in, kg out, with no unit conversion left to
the caller. Every function accepts symbols:

- `mass_blades_afdd82_kg`, `mass_hub_afdd82_kg`, `mass_rotor_group_afdd82_kg`
  (all rotors);
- `mass_gearbox_rotor_shaft_afdd83_kg` (whole drive system);
- `mass_drive_shaft_afdd82_kg`;
- `mass_engine_support_afdd82_kg`, `mass_air_induction_afdd82_kg`,
  `mass_engine_cowling_afdd82_kg`.

`vehicle/items.py`:

- `Nacelles(mass_engines_kg, count_engines, area_wetted_m2, ...)`: AFDD82
  engine section, without pylon.
- `InterconnectShaft(power_drive_limit_W, speed_rotor_rad_s, length_m, ...)`.
- `FixedEquipment(mass_kg, x_m, z_m)`.
- `LandingGear.is_retractable` (default False).

`Aircraft` and `MassBreakdown` gain optional `nacelles`, `drive_shaft` and
`equipment` fields. An absent item contributes zero mass, and all three count
as empty mass.

`powertrain/topologies.py` `build_mechanical_tiltrotor(turboshaft, gearbox,
propulsor, count_rotors=2)` builds n x (turboshaft -> gearbox -> rotor). The
interconnect is airframe mass, not a port connection.

`examples/xv15_reference.py` provides:

- `Xv15Reference`: published data in SI;
- `published_groups`;
- `build_xv15_aircraft(reference, factors, mass_engine_kg)`;
- `compare_groups`;
- `calibration_factors`, which returns actual / predicted per group;
- `solve_xv15_closure`;
- `mass_turboshaft_from_power_kg`, which inverts AeroSandbox's
  `power_turboshaft` with one explicit Opti equality.

## Engine lapse, part power and hover download (Tier 10b)

`SimpleTurboshaft` gains three submodels. Each default reproduces Tiers 1–9.

| Field | Effect | Default |
|---|---|---|
| `lapse_exponent` | power available = rated x sigma^n | 0 |
| `part_power_model` | multiplies efficiency by a part-power ratio (Tier 11a submodels) | None |
| `mass_kg` | explicit mass, overriding specific power | None |

For the knockdown, throttle is shaft power over power available.

New methods:

- `power_available_W(atmosphere=None)`;
- `get_limits(atmosphere=None)`;
- `thermal_efficiency_at(shaft_power_W, atmosphere=None)`;
- `evaluate(shaft_power_W, atmosphere=None)`.

None means sea level.

`operating_margins(topology, port_values, atmosphere=None)` checks the
turboshaft against power available at the point. Design margins stay on
sea-level ratings. `build_flight_point` passes the point's atmosphere.

`SimpleAerodynamics.download_fraction_hover` (default 0) sets hover thrust to
T/W x W / (1 - f).

`examples/xv15_performance.py` provides:

- `Xv15PowerData`: digitized TM X-62407 figs. 6.2.2 and 5.1.1 and the sec. 6.2
  sfc ratings;
- `fit_lapse_exponent`;
- `calibrate_figure_of_merit`;
- `hover_mass_kg` and `hover_ceiling_m`, each one explicit Opti equality;
- `part_power_sfc_ratios`;
- `tier9_hover_power_ratio`.

`CoupledSizingResult` also reports:

- `powertrain_masses_kg` per instance;
- the motor and generator peak torques;
- per-segment motor speed and torque, appended to each segment tuple.

## Halo-class sizing (Tier 10c)

Library changes, each with a default that reproduces earlier tiers:

- `build_series_hybrid(..., count_turbogenerators=1)`.
- `FlightCondition.active_generator_count` (None means all). Each active
  turbogenerator carries an equal share of the generator power, and fuel flow
  scales by the active count.
- `ActuatorDiskPropulsor` gains:
  - `coefficient_of_performance_airplane` (None means the same as hover),
    used by airplane-mode flight points through `in_airplane_mode()`;
  - `speed_tip_max_m_s` (None means no bound), which flight points enforce
    as rotor speed x radius <= the bound;
  - `radius_m()`.

`examples/halo_sizing.py` provides:

- `HaloRequirements`, `HaloAssumptions`, and `HaloDesign` (every sized
  quantity; numbers or Opti variables);
- `build_halo_aircraft(design, ...)`;
- `halo_mission(...)`;
- `solve_halo_sizing(requirements, assumptions, factors, initial=None)`,
  which returns `HaloSizingResult`. `initial` warm-starts from an earlier
  result, which sensitivity studies need.

## Turboshaft deck and part-power submodels (Tier 11a)

`SimpleTurboshaft.part_power_model` replaces the boolean
`part_power_knockdown`. It multiplies the full-power efficiency by
`efficiency_ratio(throttle)`, where throttle = shaft power / power available.

| Submodel | Behaviour |
|---|---|
| None | constant efficiency |
| `GeissPartPowerModel()` | AeroSandbox knockdown (the previous `True`) |
| `CubicPartPowerModel(coefficients)` | 1 + a x + b x^2 + c x^3 with x = t - 1; exactly 1 at full power |
| `TabulatedPartPowerModel(power_fraction, sfc_ratio)` | B-spline through the table, end values held outside it |

`deck_1120hp_part_power_model()` is the cubic fitted to the embedded
`deck_1120hp_power_fraction` / `deck_1120hp_sfc_ratio` table.

`powertrain/decks.py` provides:

- `load_gasp_turboshaft_deck(path)`, which returns `TurboshaftDeck` (SI
  arrays plus header metadata);
- `part_power_curve(deck, fractions)`, which returns the median, minimum and
  maximum of sfc / sfc_max over all Mach and altitude rows;
- `fit_cubic_part_power(fractions, sfc_ratio)`.

`HaloAssumptions.part_power_model` defaults to the deck cubic.

## Rotor speed physics (Tier 12)

`powertrain/components/rotor.py` `MomentumProfileRotor` computes

CP = CT lambda + kappa CT lambda_i + (sigma / 2) c_d I(lambda)

- c_d is a blade drag polar in loading referred to the mean section dynamic
  pressure, x = (CT / sigma) / (1 + 3 lambda^2). Airplane mode adds the
  increment a + b lambda^2.
- I(lambda) is the closed-form profile integral (`profile_integral`), with
  hover evaluated at exactly 1/4.
- Constants are the JVX least-squares calibration
  (`examples/jvx_rotor_calibration.py`, data in `data/rotors/`, from
  NASA/TM-2016-219070).
- `evaluate` returns `RotorResult`: thrust, shaft power, induced velocity,
  blade loading, helical tip Mach and advance ratio.
- `get_limits` returns `RotorLimits`.
- `in_airplane_mode()` selects the airplane-mode constants.

`build_flight_point` handles both rotor models:

- it creates rotor speed before evaluating the rotor and passes
  `speed_rad_s` to it;
- it enforces `blade_loading_max` and `mach_tip_helical_max`, and
  `advance_ratio_max` in airplane mode, whenever the rotor defines them.

The actuator disk defines none of these limits, so its behaviour is
unchanged.

`HaloAssumptions.rotor_speed_physics` (default True) selects the new rotor.
`HaloDesign.solidity` and `HaloDesign.speed_tip_m_s` become design variables
(sigma 0.06–0.14, hover tip Mach <= 0.70 at sea level); None keeps the fixed
assumption values.

## Hot and high (Tier 16)

**Atmosphere.**

- `FlightCondition.temperature_offset_K` (default 0) is the ambient
  temperature minus ISA at the pressure altitude.
- `build_flight_point` builds one
  `asb.Atmosphere(altitude=..., temperature_deviation=...)`. The rotor, the
  turboshaft limits and the operating margins all use it. Pressure follows
  the ISA, so a hot day is less dense.
- These `SimpleAerodynamics` methods take the keyword
  `temperature_offset_K=0.0`: `evaluate`, `alpha_stall_deg`,
  `parasite_drag_breakdown`, `lift_curve_slope_per_rad` and
  `surface_lift_curve_slope_per_rad`.
- The four capability requirements and the five mission segments end with a
  `temperature_offset_K = 0.0` field, passed to their flight condition.
  Positional construction is unchanged.

**Turboshaft lapse submodel.**

| `SimpleTurboshaft.lapse_model` | Power available / rated |
|---|---|
| None | sigma^`lapse_exponent` (Tier 10b) |
| `DensityTemperatureLapse(lapse_exponent, lapse_exponent_temperature)` | sigma^n (T / T_ISA(h))^-m; exactly sigma^n on a standard day |

T_ISA(h) = T - `atmosphere.temperature_deviation`. When `lapse_model` is set,
`lapse_exponent` on the turboshaft is unused.

**XV-15 hot day (`examples/xv15_hot_day.py`).**

- `Xv15HotDayData`: digitized 95 F power available and hover weights.
- `temperature_from_fahrenheit_K` and `temperature_offset_K(altitude_m,
  temperature_K)`. The offset is taken from AeroSandbox's standard
  atmosphere.
- `hot_atmosphere`.
- `fit_temperature_lapse_exponent(n)`, `xv15_lapse_model(n)` and
  `xv15_engine_hot`.
- `hover_mass_kg(atmosphere, figure_of_merit, lapse_model)`.

**Halo sizing.**

- `HaloRequirements` fields: `hover_hot_day` (default True),
  `altitude_hover_hot_m` (4,000 ft), `temperature_hover_hot_K` (95 F),
  `thrust_to_weight_hover_hot` (1.05) and `duration_hover_hot_s` (60 s).
- The point is flown at the mission's end mass and end SOC. Margin
  `soc_after_hot_day_hover` >= the emergency floor.
- `HaloAssumptions.temperature_lapse` (default True) selects the XV-15
  density-temperature lapse.
- `HaloSizingResult.hovers` is a tuple of `HoverSummary`, one per hover point.
  Each holds:
  - mass, SOC and offset;
  - the solved battery share and the turbine-limited `battery_share_min`;
  - rotor power, turbine power available and battery power;
  - blade loading.

  It also gives `soc_after_hover_hot`.
- `requirements_tier12b` is the Tier 12b reference without the hot-day
  hover.
