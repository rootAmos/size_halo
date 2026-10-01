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
