# Component interfaces

All components expose get_mass(), get_limits(), evaluate(). Results and limits
are dataclasses; values may be numeric scalars, vectors or CasADi expressions.
Models build equations and do not enforce limits, clip outputs or resize parts.

| Component | Operating inputs | Rating convention |
|---|---|---|
| Motor | speed_rad_s, torque_Nm, voltage_V | rated shaft output |
| Generator | speed_rad_s, torque_Nm, voltage_V | rated shaft input |
| Battery | current_A, soc, duration_s | terminal discharge/charge power |
| EquivalentCircuitBattery | current_A, soc, duration_s, voltage_rc_start_V, temperature_C | cell voltage window and C-rate x power-density factor (Tier 17) |
| SimpleTurboshaft | shaft_power_W | shaft output |
| Gearbox | speed_input_rad_s, torque_input_Nm | shaft input |
| ActuatorDiskPropulsor | axial_velocity_m_s, atmosphere, thrust_N OR shaft_power_W and induced_velocity_m_s (speed_rad_s accepted, ignored) | shaft input |
| MomentumProfileRotor | axial_velocity_m_s, atmosphere, thrust_N, speed_rad_s | shaft input; limits blade_loading_max, mach_tip_helical_max, advance_ratio_max |
| Inverter | power_ac_W (positive DC to AC; negative as a rectifier), voltage_dc_V | rated AC power; DC link <= blocking voltage x derating (Tier 15) |
| DcDcConverter | power_input_W (positive battery to bus), voltage_input_V | rated power; regulated output voltage (Tier 15) |
| Cable | current_A | design current (sets the conductor area); peak voltage (sets the insulation) (Tier 15) |
| ProtectionUnit | current_A | rated current per pole (Tier 15) |
| RamAirHeatExchanger | power_heat_W, atmosphere, velocity_m_s, fan | heat rejected at the reference coolant-to-air temperature difference (Tier 19) |

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
| EquivalentCircuitBattery | electrical (OUT; positive current discharges; sets bus voltage) |
| SimpleTurboshaft | fuel (IN), shaft (OUT) |
| Gearbox | shaft_in (IN), shaft_out (OUT) |
| ActuatorDiskPropulsor | shaft (IN) |
| MomentumProfileRotor | shaft (IN) |
| Inverter | dc (IN), ac (OUT); as a generator rectifier `rectifier_port_specs()`: ac (IN), dc (OUT) |
| Cable, ProtectionUnit, DcDcConverter | input (IN), output (OUT), in the nominal power-flow direction |

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
| Wing, HorizontalTail | area_m2, aspect_ratio, taper_ratio, x_le_root_m, z_m, airfoil | raymer mass_wing / mass_hstab (Wing: or `mass_model`, Tier 20) | 40 % MAC |
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

## Tiltrotor wing weights (Tier 20)

**`weights/afdd.py`:**

- `section_form_factors_tiltrotor_wing(thickness_to_chord,
  fraction_chord_torque_box)` returns (F_B, F_C, F_T, F_VH).
- `wing_tiltrotor_afdd_masses(...)` is NDARC sec. 19-1.1 in consistent SI. It
  returns `TiltrotorWingMasses`:
  - masses: torque box, stiffness spar caps, jump spar caps, fairings,
    control surfaces, fittings and fold;
  - realized stiffness and mode frequencies (rad/s) and the jump moment;
  - `mass_primary_kg()` and `total()`.
- Optional `smoothing` rounds the two max(0, .) steps. Nothing iterates.

**`vehicle/surfaces.py`:**

- `Wing.mass_model` selects the wing mass submodel; the default None is
  Raymer. `Wing.mass_factor` multiplies either model.
- `Wing.chord_mean_m()`.
- `TiltrotorWingMassModel(mass_tip_kg, radius_gyration_pylon_m,
  speed_rotor_design_rad_s, width_fuselage_m, ...)`:
  - methods `masses(wing, condition)`, `mass_kg(wing, condition)` and
    `frequency_per_rev(masses, speed_rotor_rad_s)` (for whirl-flutter
    margins);
  - frequencies are per rev of `speed_rotor_design_rad_s`;
  - defaults are XV-15-calibrated (plan 024).
- `WingMaterial`, `aluminium_wing_material()` and
  `graphite_epoxy_wing_material()`.

**`examples/xv15_reference.py`:**

- `Xv15MassFactors.wing_tiltrotor` (default 1);
- `xv15_wing_mass_model()`;
- `wing_weight_model="raymer" | "afdd_tiltrotor"` on `build_xv15_aircraft`,
  `compare_groups` and `solve_xv15_closure`;
- `calibration_factors()` also returns `wing_tiltrotor`.

**`examples/tiltrotor_wing_calibration.py`:** the XV-15 section calibration,
plus the V-22 and Bell D266 cross-checks, using `data/weights/`.

**`examples/halo_sizing.py`:**

- `HaloAssumptions.wing_weight_model` (default "raymer") and the wing
  frequency, material, pylon, tip and jump fields;
- `HaloDesign.speed_rotor_wing_design_rad_s` (an Opti variable with the
  AFDD wing);
- whirl-flutter margins at airplane-mode points;
- `HaloSizingResult.wing_masses_kg` and `whirl_flutter`;
- itemised equipment: `EquipmentItem` and `halo_equipment_items`;
- the named set `assumptions_tier20`.

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

## Machines sized by torque (Tier 13)

- **`motor.TorqueDensityMassModel(torque_density_Nm_kg=15, specific_power_max_W_kg=1e4, smoothing_kg=2)`:**
  `mass_kg(machine)` = softmax(max_torque / torque_density,
  power_rated / specific_power_max).
- **`Motor.mass_model` and `Generator.mass_model`:** None (the default)
  keeps power / specific power.
- **`afdd.mass_gearbox_rotor_shaft_afdd00_kg(count_rotors, power_drive_limit_W, speed_engine_rad_s, speed_rotor_rad_s)`.**
- **`build_series_hybrid(..., generator_gearbox=None)`:** an optional
  step-up gearbox (reduction_ratio < 1) between turboshaft and generator.
  The flight point handles it: engine shaft power = generator shaft power /
  gearbox efficiency.
- **`HaloAssumptions`:** `machine_mass_by_torque` (default True),
  `torque_density_Nm_kg`, `specific_power_max_machine_W_kg`,
  `generator_step_up`, `speed_output_turboshaft_rad_s` (1,210 rpm),
  `direct_drive_rotor`.
- **`HaloDesign`:** `speed_peak_motor_rad_s`, `reduction_ratio`,
  `speed_peak_generator_rad_s`.
- **Legacy set:** `assumptions_tier12b` reproduces Tier 12b.

## Trajectory optimization (Tier 14)

`aircraft_closure.trajectory.tiltrotor` flies a **fixed, already-sized** aircraft through a
direct-collocation trajectory. It is separate from sizing: each trajectory is its own `asb.Opti`
over states, controls and the final time (plan 019).

AeroSandbox supplies the dynamics (`DynamicsPointMass2DSpeedGamma`), the trapezoidal collocation
(`constrain_derivatives`, `Opti.constrain_derivative`) and `Atmosphere`.

| Class / function | Role | Inputs | Outputs |
|---|---|---|---|
| `TiltrotorPointMass` | Equation-only tiltrotor force and power model (series-hybrid topology, `MomentumProfileRotor`) | node arrays: `velocity_m_s`, `altitude_m`, `alpha_deg`, `tilt_deg`, `thrust_per_rotor_N`, `speed_rotor_rad_s` | `TiltrotorForces`: wind-axis forces without gravity, lift, drag, rotor result, motor speed, torque and power, `power_electric_motors_W` (bus) |
| `TiltrotorPointMass.evaluate_supply` | Battery plus turbogenerators feeding the bus | `altitude_m`, `current_battery_A`, `torque_generator_Nm`, `soc` | `PowerSupply`: battery, generator and engine results, generator shaft power, lapsed turboshaft power available, bus supply, fuel flow |
| `ConversionCorridor` | Airspeed band against nacelle tilt (XV-15 fig. 5.4.1 shape) | `velocity_stall_m_s`, high-speed boundary at 90 and 0 deg | `velocity_min_m_s(tilt_deg)`, `velocity_max_m_s(tilt_deg)` |
| `build_tiltrotor_trajectory` | Orchestration: node variables, AeroSandbox dynamics, mass/SOC/energy states, operating limits | `opti`, model, initial mass and SOC, node count, duration bounds, `TrajectoryGuess`, `TrajectoryLimits`, corridor | `TiltrotorTrajectory` (node arrays, `acceleration_m_s2`, `rate_gamma_rad_s`, `tilt_rate_deg_s`) |

**Conventions.**

- Tilt is 90 deg in hover and 0 deg in airplane mode, measured from the fuselage axis.
- The rotor-axis angle to the flight path is alpha + tilt.
- Wind-axis z points down (AeroSandbox).
- The rotor's axial velocity is V cos(alpha + tilt). This is an axial-flow approximation; edgewise
  physics are deferred.
- The airplane-mode rotor coefficients and the hover download are blended with tilt.

**Caller's responsibilities.** The caller adds boundary conditions and the objective. The
examples are `examples/trajectory_optimization.py`:

- `solve_min_energy_transition`;
- `solve_prescribed_transition`;
- `solve_min_time_climb`.

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

## Equivalent-circuit battery (Tier 17)

**`EquivalentCircuitBattery`** (`powertrain/components/battery_ecm.py`,
plan 021) is a distinct class. `Battery` stays the simplest model.

- **Pack:** `count_series` x `count_parallel` cells (`count_parallel`
  continuous). Voltage = n_s x cell voltage; resistance = cell resistance x
  n_s / n_p.
- **Mass:** cells x cell mass / `fraction_mass_cells` (0.7).
- **Energy:** `energy_capacity_J` = Q x n_s x mean OCV(0..1) x
  `factor_capacity_ageing`.
- **Cell:** `LithiumIonCell`, i.e. ratings plus `ocv_model` and
  `resistance_model`.
  - Default: `inr21700_50g_cell()` (Samsung INR21700-50G, Paudel et al.,
    *Batteries* 2025, 11, 313, CC BY 4.0).
  - Data: `data/batteries/`. Loaders and fits: `powertrain/cells.py`.
- **OCV submodels** (interchangeable):
  - `PolynomialOcvModel`: degree 7, 30 C data, rms 9 mV.
  - `TabulatedOcvModel`: B-spline through the nodes, holding the end values.
- **Resistance:** R0 + two RC branches (tau 8 s and 43 s).
  - Form: R_k(SOC, T) = R_ref exp(a x + b x^2)
    (1 + c e^(-(s-0.2)/w_lo) + d e^((s-0.8)/w_hi)), with x = T_ref/T - 1.
  - Fitted to all 308 discharge DCIR points: rms 4.7 %.
  - Below SOC 0.2 the rise continues; above 0.8 the high term is held.
- **`factor_power_density`:** divides the resistances and multiplies the
  current rating (the user's explicit assumption). Mass is unchanged.
- **Ageing:** `factor_resistance_ageing` and `factor_capacity_ageing`.

`evaluate(current_A, soc, duration_s=0, voltage_rc_start_V=None,
temperature_C=None)` assumes a constant current over dt:

- SOC_next = SOC - I dt / Q. OCV and R are taken at the mid-interval SOC.
- RC branch k: v_k(dt) = I R_k + (v_k0 - I R_k) e^(-dt/tau_k), exact. The
  interval mean uses g_k = (tau_k/dt)(1 - e^(-dt/tau_k)).
- With `voltage_rc_start_V=None`, the steady state v_k = I R_k applies.
- `voltage_V` is the interval-mean terminal voltage (the bus voltage):
  V = V* - I R_eff.
  - `voltage_driving_V` V* = OCV - sum(v_k0 g_k).
  - `resistance_effective_ohm` R_eff = R0 + sum R_k (1 - g_k).
- `voltage_end_V`; `power_chemical_W` = OCV I; `power_loss_W` = chemical -
  terminal; `power_max_W` = V*^2 / (4 R_eff).

Limits are the voltage window `count_series` x (2.5, 4.2) V, the
discharge/charge current `count_parallel` x (9.8, 4.9) A x F, and the SOC
window. The compatibility envelope's power is the discharge current rating x
nominal voltage. Operating margins cover discharge and charge current and
min and max terminal voltage.

**Flight point.**

- `build_flight_point(..., duration_s=0.0, voltage_rc_start_V=None)` passes
  the RC state to the ECM.
- It also adds the branch constraint `voltage_V / voltage_driving_V >= 0.5`.
  This selects the low-current root of R_eff I^2 - V* I + P = 0, with no
  iteration, and enforces P <= P_max.

**Mission.** `build_mission(..., subsegments=1, polarization_start="rest")`:

- **Splitting:** each segment becomes equal-duration points (an int, or one
  count per segment). `SegmentResult.subsegments` holds the per-point
  results, and the top-level result aggregates them.
- **ECM bookkeeping:** the battery's `soc_next` chains SOC. RC voltages
  propagate from rest, or start "steady". Each point adds a margin
  "<label>: battery end voltage_V" against the pack minimum voltage.
- **Default:** 1 reproduces the earlier results exactly; the `Battery`
  bookkeeping is unchanged.
- **`HoverSegment.active_generator_count`:** models an engine-out hover.

**Halo** (`examples/halo_sizing.py`):

- **Selector:** `HaloAssumptions.battery_model` is "constant" (the reference)
  or "ecm".
- **ECM fields:** `factor_power_density_battery` 5, `count_series_battery`
  210, end-of-life capacity 0.8 / resistance 1.5, 25 C, cell/pack 0.7.
- **Splits:** mission (1, 1, 4, 1, 1, 1); engine-out hover 3 points, steady
  start.
- **Design variable:** `HaloDesign.count_parallel_battery`. Energy and power
  are then derived from the pack.
- **Outputs:** `battery_trace` and `engine_out_trace`.
- **Named sets:**
  - `assumptions_tier12b` (constant battery);
  - `assumptions_tier17` (ECM);
  - `requirements_tier16` with `assumptions_tier16` (the 900 kg
    constant-battery aircraft of Tiers 13–16).
  - `requirements_plan022` with `assumptions_plan022` (the 780 kg ECM
    aircraft with the Raymer wing).
  - `requirements_plan026` with `assumptions_plan026` (SimpleAerodynamics,
    AFDD wing, 900 kg).
  - Since plan 027 the defaults are the ECM pack, the AFDD tiltrotor wing
    and AeroBuildup aerodynamics, at 900 kg.
- **Trajectory** (`build_tiltrotor_trajectory`):
  - accepts either battery;
  - the motors see the terminal voltage;
  - with the ECM pack, SOC is coulomb-counted with current and voltage
    limits.

## Electrical layer (Tier 15)

Plan 023. Components in `powertrain/components/converters.py`, `cable.py` and `protection.py`; the
whole layer is optional (`build_series_hybrid(..., electrical=None)`; Halo
`HaloAssumptions.electrical_layer`, default False).

| Class | Mass | Losses | Limits |
|---|---|---|---|
| `Inverter` | rated power / specific power (20 kW/kg) | `ConverterLossModel` | rated AC power, DC window (min, blocking x derating) |
| `DcDcConverter` | rated power / specific power (12 kW/kg) | `ConverterLossModel` (98 %) | rated power, input window, output voltage |
| `Cable` | conductor + PD-sized insulation, x (1 + accessories) | R I^2, drop R I | design current, peak voltage, PDIV at the design altitude |
| `ProtectionUnit` | poles x (0.2 kg + 1.3 g/A x rated current) | poles x rated drop x I^2 / I_rated | rated current and voltage |

- **`ConverterLossModel(efficiency_rated, fraction_loss_conduction, fraction_loss_switching, voltage_rated_V)`:**
  P_loss = L_r [f_0 + f_s |P|/P_r + f_c (P/P_r)^2 (V_r/V)^2], L_r = P_r (1 - eta_r)/eta_r; |P| smoothed.
- **`PartialDischargeModel`:** PDIV = 163 (t/eps_r)^0.46 V (t in micrometres; Dakin) x (p/p0)^0.5;
  `thickness_required_m` inverts it for PDIV = 1.5 x peak voltage. `Cable.thickness_insulation_m()` is the
  smooth maximum of that and the minimum wall.
- **Materials:** `aluminium_conductor()` (default) and `copper_conductor()`; `InsulationMaterial`.
- **`ElectricalLayer`** (`topologies.py`): one component per feeder type; per feeder bus - protection -
  cable - inverter - motor, generator - rectifier - cable - protection - bus, battery - protection - cable -
  [DC/DC] - bus.
- **Flight point:** `FlightPoint.electrical` (`ElectricalLayerResult`: bus and battery voltages, per-instance
  results, bus-side powers, loss totals, and `heat_loads`: one Tier 19 `HeatLoad(instance_name, per-unit
  loss, active count)` per electrical instance, appended to `FlightPoint.heat_loads`). The hybridization share is on
  the bus side.
- **Operating margins:** inverter AC power and DC window; cable and protection current (squared, either
  sign); cable partial discharge at the point's pressure; DC/DC power and input window.
- **Halo:** `electrical_layer`, bare-machine figures (17.6 N.m/kg, 20 kW/kg cap), inverter (20 kW/kg,
  98.5 %, 1,200 V x 0.75), feeders (1.25 x half span, battery 3 m, 125 % current rating), optional DC/DC.
  `bus_voltage_window`, `build_halo_electrical`, `assumptions_for_bus_voltage(assumptions, V, dcdc=False)`,
  `enumerate_bus_voltage(...)`, `solve_halo_max_payload(...)`, `assumptions_tier15`.

## Aerodynamics build-up (Tier 21)

Plan 025. Three interchangeable models share the operating interface
`evaluate(aircraft, velocity_m_s, altitude_m, alpha_deg, drag_increments=(),
temperature_offset_K=0.0, rotor_state=None) -> AeroResult`,
`alpha_stall_deg(aircraft, velocity_m_s, altitude_m, temperature_offset_K=0.0,
aero=None)`, `lift_curve_slope_per_rad`, `surface_lift_curve_slope_per_rad`,
`cl_max` and `hover_download_fraction(aircraft)`.

- **`SimpleAerodynamics`** (Tier 5): unchanged physics; it accepts and ignores
  `rotor_state` and `aero`; `hover_download_fraction` returns the constant
  `download_fraction_hover`.
- **`BuildupAerodynamics`** (`aerodynamics/buildup.py`): `asb.AeroBuildup`
  on `aircraft.to_asb()` (wings, fuselage, nacelle bodies) with the airfoils
  wrapped in `TransitionAirfoil` (NeuralFoil `xtr_upper`, `xtr_lower`,
  `n_crit`). Fields: `cl_max`, `interference` (`InterferenceFactors`),
  `drag_area_misc_m2`, `drag_area_landing_gear_fixed_m2`, `xtr_upper`,
  `xtr_lower`, `n_crit`, `model_size`, `blown_wing` (`BlownWing` or None),
  `download_fraction_hover` (None: geometry), `download` (`HoverDownload`).
  - CD = sum_c Q_c D_c / (q S) + CDA_misc / S [+ CDA_gear / S if the gear is
    fixed] + CL^2 / (pi AR e) + blown increments + `drag_increments`.
  - e = AeroBuildup span efficiency (s_eff / b)^2 x k_e,F x k_e,M.
  - `cd0` is the parasite drag at the operating point (NeuralFoil profile drag
    varies with alpha), not a zero-lift value.
  - Lift is the whole aircraft's (tails at zero incidence, no downwash). The
    slope methods stay the Tier 6 analytic isolated-surface slopes, which
    `cl_alpha_per_rad` also reports.
  - `alpha_stall_deg(..., aero=point)` = alpha + (cl_max - CL)/CL_alpha, so
    alpha <= alpha_stall is exactly CL <= cl_max at the point; without `aero`
    it linearizes AeroBuildup through 0 and 8 deg (two extra runs).
  - `evaluate_buildup(...)` returns `BuildupResult(aero, breakdown,
    cl_aerobuildup, oswald_span, blown)` for reports.
- **`ScholzAerodynamics`** (`aerodynamics/scholz.py`, extends
  `SimpleAerodynamics`): the hand check. Functions
  `skin_friction_turbulent(Re, M)`, `form_factor_surface`,
  `form_factor_fuselage`, `form_factor_nacelle`, `area_wetted_fuselage_m2`,
  `area_wetted_surface_m2`, `oswald_nita_scholz(...)`; `wave_drag_coefficient`
  uses AeroSandbox `Cd_wave_Korn` (kappa_A 0.87).
- **Blown wing** (`aerodynamics/slipstream.py`): `RotorState(thrust_per_rotor_N,
  speed_rotor_rad_s)`; `BlownWing(fraction_slipstream_on_wing=0.5,
  ratio_distance_disk_to_radius=0.4, sign_swirl=+1, factor_swirl=1,
  efficiency_swirl_recovery=0.5).evaluate(...) -> BlownWingIncrement`
  (velocity_induced_m_s, velocity_increment_wing_m_s, ratio_dynamic_pressure,
  angle_swirl_deg, area_immersed_m2, delta_cl, delta_cd_profile,
  delta_cd_swirl). All increments vanish at zero thrust.
- **Download** (`aerodynamics/download.py`): `HoverDownload(
  drag_coefficient_vertical=0.846, chord_fraction_flap=0.25,
  deflection_flap_hover_deg=60, interference=1)`;
  `download_fraction(chord_wing_m, radius_rotor_m, count_rotors, download)`;
  `hover_download_fraction(aircraft, download)` reads the mean wing chord and
  the propulsor instance.
- **Flight point:** hover uses `aerodynamics.hover_download_fraction(aircraft)`.
  In airplane mode, a model with a `blown_wing` gets a rotor-thrust Opti
  variable per point and the equality n T = D + W sin(gamma); the stall bound
  passes the point's `aero`.
- **Trajectory:** download via `hover_download_fraction`; the stall bound
  passes `aero`; no rotor state (unblown polar).
- **Vehicle:** `Nacelles(length_m=None, diameter_m=None, y_m=0.0)`,
  `Nacelles.to_asb()` (two bodies of revolution, spinner as the nose);
  `Aircraft.to_asb()` appends them.
- **Halo:** `HaloAssumptions.aerodynamics_model` ("simple" default, "buildup",
  "scholz"), `length_nacelle_m` (9 ft), `diameter_nacelle_m` (3.3 ft),
  `drag_area_misc_buildup_m2` (3.00 ft2), `blown_wing` (True);
  `build_halo_aerodynamics(requirements, assumptions)`.

## Thermal (Tier 19)

Plan 028. Every value may be an Opti expression; nothing iterates.

- **`powertrain/components/thermal.py` `LumpedThermalModel(specific_heat_J_kg_K=500,
  temperature_max_C=150, temperature_coolant_C=60)`:** a submodel owned by a
  component (`thermal_model` on `Motor`, `Generator`, `Battery` and
  `EquivalentCircuitBattery`, default None).
  `temperature_end_C(power_loss_W, duration_s, capacity_J_K, resistance_K_W,
  temperature_start_C=None)` is the exact constant-heat solution
  T_ss + (T_0 - T_ss) e^(-dt/tau), T_ss = T_c + Q R, tau = R C; a None start
  is the steady state, a numeric zero duration keeps the start.
  `temperature_mean_C(...)` is the interval mean T_ss + (T_0 - T_ss) g with
  g = (tau/dt)(1 - e^(-dt/tau)); the mean heat to the coolant is
  (T_mean - T_c) / R = loss - C (T_end - T_0) / dt.
- **`powertrain/components/heat_exchanger.py` `RamAirHeatExchanger(power_rated_W,
  specific_power_W_kg, temperature_coolant_C, delta_temperature_ref_C,
  effectiveness, pressure_drop_ref_Pa, efficiency_fan)`:** mass = rating /
  specific power; `evaluate(power_heat_W, atmosphere, velocity_m_s=0, fan=False)`
  returns `HeatExchangerResult(mass_flow_air_kg_s, pressure_drop_Pa,
  power_pumping_W, drag_N, power_fan_W, power_heat_equivalent_W,
  delta_temperature_C)`. Air flow Q / (eps cp dT), pressure loss
  dp_ref (m/m_ref)^2 rho_ref/rho, pumping power m dp / rho; ram drag = pumping
  power / V, or fan power = pumping power / eta_fan with `fan=True`; required
  rating Q dT_ref / dT.
- **`thermal/heat.py`:** `HeatLoad(source, power_W, count)` (per unit, by
  instance name), `total_heat_W(loads, sources_excluded=())`,
  `thermal_parameters(component)` (C = c x mass; R = (T_max - T_c) / loss
  at the continuous rating: rated power at rated speed for machines, rated
  discharge current at SOC 0.5 for batteries, so the continuous rating's
  steady state is the limit), `evaluate_point_thermal(...)` returning
  `PointThermal(heat_loads, power_heat_W, power_heat_end_W,
  power_heat_equivalent_W, cooling, temperatures_end_C, temperatures_mean_C,
  power_to_coolant_W, margins)`, and `coolant_temperatures_C(powertrain)`.
  A thermal-modelled source passes (T - T_c) / R to the cooler (interval
  mean for drag and fan power, end of interval for the rating margin);
  other sources pass their loss.
- **`vehicle/powertrain_installation.py`:** `InstalledCooling(heat_exchanger,
  x_m, z_m=0, sources_excluded=())`; `PowertrainInstallation.cooling`
  (None) adds the item "heat_exchanger" to the powertrain mass.
- **Flight point:** `build_flight_point(..., temperature_start_C=None)`.
  `FlightPoint.heat_loads` (motor, gearbox, generator, battery,
  generator_gearbox) and `FlightPoint.thermal`. With cooling installed, the
  fan power joins the bus demand that the battery share and the generators
  supply; in airplane mode a cooling-drag variable joins the thrust, with the
  equality D_cool = exchanger drag. Margins "<label>: heat_exchanger
  power_heat_W" and "<label>: <instance> temperature_C" (normalized by the
  allowed rise T_max - T_c).
- **Mission:** `build_mission(..., thermal_start="steady" | "coolant" | dict)`;
  `MissionResult.temperatures_end_C`.
- **Flight point start:** with cooling installed the battery current starts
  at 0 A (50 A otherwise).
- **Compatibility:** a machine with a thermal model has no power-rating
  operating margin (torque, speed and voltage margins stay).
- **Halo:** `HaloAssumptions.thermal_model` (False) and its fields;
  `HaloDesign.power_rated_heat_exchanger_W` and `power_rated_gearbox_W`;
  `HaloSizingResult.thermal_trace` and `heat_exchanger`.
