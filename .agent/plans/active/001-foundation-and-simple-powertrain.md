# Foundation and simple powertrain

## Goal and scope

Implement bootstrap Tiers 0 and 1 for a Halo-inspired unmanned series-hybrid
tiltrotor research repository. Read context guidance in full. References:
`docs/ARCHITECTURE.md`, `docs/MODEL_INTERFACES.md`, and the context bootstrap.
Aircraft sizing and missions remain subsequent tiers, as explicitly instructed.

## Physics and interfaces

SI throughout. Six standalone components: Motor, Generator, Battery,
SimpleTurboshaft, Gearbox, ActuatorDiskPropulsor. Named results and limits.
Positive motor power consumes electrical energy; positive generator power
supplies it. Battery current is positive on discharge. Gear ratio is input
speed/output speed. Engine efficiency and component specific powers are
illustrative constructor inputs, not Archer data.

Inputs: shaft speed/torque, bus voltage, battery current/SOC/time step, engine
shaft demand, rotor axial velocity/atmosphere/shaft power/induced velocity.
Outputs: physical power, losses, current, SOC, fuel flow, thrust, and residuals.
Parameters: ratings, specific power/energy, loss coefficients, resistance,
efficiency, rotor area/mass. No component owns design variables or controls.
Battery SOC is an externally supplied state. Induced velocity is an externally
supplied coupling variable for power-driven rotor evaluations.
Equality: rotor shaft-power residual = 0 in caller's Opti. Inequalities: caller
enforces power/speed/torque/current/voltage/SOC ratings and positive operating
domains. No clipping or hidden iteration. Symbolic math uses AeroSandbox.

## Sequence and acceptance

1. Install governance, six local skills, package metadata and docs.
2. Implement components and each module's manual sanity case.
3. Test identities, limits, trends, signs, numeric vectors and Opti compatibility.
4. Run an explicit coupled Opti reference case and all sanity examples.
5. Record results and Tier 2 recommendation; move completed plan.

Reference cases: ideal hover momentum theory; forward-flight AeroSandbox
actuator disk; ideal motor/gearbox; constant-current battery discharge/charge;
constant-power fuel flow; series-hybrid power balance at one operating point.
Acceptance: all tests pass; no second solver; explicit documented domains;
README has runnable setup and honest implementation status.

## Deferred

Generic graph, compatibility reports, aircraft geometry/mass closure, aero,
controls, requirements, missions, thermal management, engine decks, maps,
BEM, CFD, V-tail and calibrated Halo performance. Tier 2 should add typed
mechanical/electrical/fuel ports, connections, multiplicity and series-hybrid
topology, with cycles expressed as caller-owned Opti equalities.

## Progress and decisions

- Guidance read; AeroSandbox 4.2.8 is installed.
- Native actuator-disk helper divides by airspeed at hover. Use algebraically
  equivalent momentum equations; test agreement in positive forward flight.
- Use standard-library unittest, avoiding an unnecessary test dependency.
- Implementation and verification in progress.
