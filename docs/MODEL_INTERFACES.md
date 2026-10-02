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
| ActuatorDiskPropulsor | axial_velocity_m_s, atmosphere, thrust_N OR shaft_power_W and induced_velocity_m_s | shaft input |

Motor/generator loss coefficients are nonnegative; the default quadratic model
is an illustrative loss proxy, not a calibrated electromagnetic model. Voltage
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
