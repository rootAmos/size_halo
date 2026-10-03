# Electrical layer: inverters, cables, protection, DC/DC, bus voltage

Status: ACTIVE 2026-10-03. Tier 15 (roadmap "15 Electrical layer", review
item 3). Orchestrator direction: the layer is a `HaloAssumptions` switch,
off by default, so the plan 022 reference (780 kg, 14,436 lb) is unchanged.

## Goal and scope

- **Components** (equation-only, `get_mass` / `get_limits` / `evaluate`):
  - `Inverter`: DC/AC converter between the bus and each motor, and the
    same hardware as an active rectifier behind each generator. Mass from
    specific power; losses from `ConverterLossModel` in power and DC
    voltage (fixed, switching and conduction terms).
  - `Cable`: a DC feeder (two conductors). Conductor area from a current
    density at the design current; insulation thickness from a
    partial-discharge rule at the design altitude, so insulation mass grows
    with voltage. Loss I^2 R; voltage drop I R.
  - `ProtectionUnit`: contactors and fuses per feeder; mass from the rated
    current; loss from the rated contact drop.
  - `DcDcConverter` (optional, off by default): regulates the bus voltage
    and decouples it from the pack series count.
- **Topology:** `build_series_hybrid(..., electrical=None)` inserts, per
  feeder, bus - protection - cable - inverter - machine (motors), machine -
  inverter - cable - protection - bus (generators) and battery - protection
  - cable - [DC/DC] - bus.
- **Flight point:** the power chain through the new instances; the power
  balance is on the bus side. Operating margins for every new instance.
- **Machine mass split:** with the layer on, machines use bare-machine
  figures and the inverter is a separate mass (see Assumptions).
- **Halo:** `HaloAssumptions.electrical_layer` (default False) and the
  layer's parameters; the installation factor is no longer passed (the
  cabling it stood for is explicit). Bus voltage as a discrete choice by
  explicit enumeration (example function and notebook).
- **Not in scope:** thermal management and cooling mass (Tier 19);
  redundancy, cross-strapping and nacelle-local buses (Tier 18); the
  trajectory model (Tier 14 code keeps the bare bus); machine insulation and
  reflected-wave overvoltage at the motor terminals.

## References

- R. H. Jansen et al., "Overview of NASA Electrified Aircraft Propulsion
  (EAP) Research for Large Subsonic Transports", AIAA 2017-4701: power
  converter goals of about 19 kW/kg at 99 % efficiency.
- magniX magni350 / magni650 (machine plus inverter masses, as already used
  in plan 018 for the 15 N.m/kg integrated torque density).
- P. Dakin et al.'s empirical partial-discharge inception voltage of an
  insulated conductor in air, PDIV = 163 (t / eps_r)^0.46 V peak with t in
  micrometres (sea level), as commonly quoted in the inverter-fed machine
  insulation literature.
- Paschen-law pressure dependence of air breakdown (PDIV falls with
  pressure): represented by (p / p0)^0.5 (assumed exponent).
- TE Connectivity (Kilovac) EV200 contactor: about 500 A continuous,
  12-900 V DC, about 0.43 kg; the scale for protection mass.
- Semiconductor voltage derating for cosmic-ray single-event burnout: DC
  link at most about 0.6-0.75 of the blocking voltage (industry practice).

## Assumptions

- Inverter: 20 kW/kg (between the NASA 19 kW/kg goal and recent SiC
  demonstrators; assumed), 98.5 % at rated power and the rated (nominal
  pack) voltage. Loss split at rated: conduction 0.5, switching 0.4, fixed
  0.1, so peak efficiency is near 45 % load.
- Inverter voltage: 1,200 V devices derated to 0.75 (900 V DC link max);
  the bus enumeration picks the smallest standard class (650 / 1,200 /
  1,700 / 3,300 V) that covers the pack's maximum voltage.
- Machine split (keeps the Tier 13 calibration where it was anchored):
  - torque density 15 N.m/kg integrated (magniX at about 200 rad/s) gives
    1/15 = 1/tau_bare + 200/20,000, so tau_bare = 17.6 N.m/kg;
  - the 10 kW/kg high-speed cap is treated as integrated as well, so the
    bare cap is 1/(1/10 - 1/20) = 20 kW/kg. This is above NASA's HEMM
    16 kW/kg electromagnetic target, i.e. the Tier 13 cap was optimistic;
    a sensitivity case with a 10 kW/kg bare cap is reported.
- Cables: aluminium, 3.4e-8 ohm m (about 80 C), 2,700 kg/m3, 3 A/mm2
  design current density (assumed; free-air rating of large aerospace
  feeders with altitude derating). Insulation eps_r 2.5, 1,700 kg/m3
  (fluoropolymer), minimum wall 0.25 mm. Accessories (terminations,
  shield, clamps) +20 % of the cable mass.
- Feeder lengths: 1.25 x half span (bus in the fuselage, machines in the
  tip nacelles); battery feeder 3 m.
- Design current: rated power over the minimum bus voltage (motor and
  generator feeders), the pack's discharge current rating (battery).
- PD design: PDIV(ceiling) >= 1.5 x peak pack voltage at 13,000 ft.
- Protection: two poles per feeder; 0.2 kg + 1.3 g/A per pole; 0.15 V per
  pole at rated current.
- DC/DC: 12 kW/kg, 98 % at rated (assumed); rated at the pack discharge
  power.
- Machine and inverter voltage window with the layer: 0.9 x pack minimum
  to the derated blocking voltage (machines wound for the bus).
- Cable and protection losses use the bus voltage for the feeder current
  (first order; drops are below 1 %). The battery feeder is exact: bus
  voltage = terminal voltage - I (R_cable + R_protection).

## Interfaces

- `powertrain/components/converters.py`: `ConverterLossModel`,
  `Inverter` (`evaluate(power_ac_W, voltage_dc_V)`; positive power flows
  DC to AC, so a rectifier sees negative power), `DcDcConverter`
  (`evaluate(power_input_W, voltage_input_V)`), results and limits.
- `powertrain/components/cable.py`: `ConductorMaterial`,
  `InsulationMaterial`, `PartialDischargeModel`, `Cable`
  (`evaluate(current_A)`), `aluminium_conductor()`, `copper_conductor()`.
- `powertrain/components/protection.py`: `ProtectionUnit`
  (`evaluate(current_A)`).
- `powertrain/topologies.py`: `ElectricalLayer` (the per-feeder
  components) and `build_series_hybrid(..., electrical=None)`.
- `ports.py` / `compatibility.py`: ports, envelopes and operating margins
  (inverter AC power and DC voltage window; cable current, voltage and
  partial-discharge margin at the point's pressure; protection current;
  DC/DC power and input window).
- `performance/flight_point.py`: `FlightPoint.electrical`
  (`ElectricalLayerResult`, None without the layer).
- `examples/halo_sizing.py`: `HaloAssumptions.electrical_layer` and its
  parameters; `assumptions_for_bus_voltage(assumptions, voltage_nominal_V,
  dcdc=False)`; `enumerate_bus_voltage(...)`; `build_halo_electrical(...)`.

## Symbolic considerations

- All component expressions accept Opti variables (machine ratings, span,
  pack parallel count). The voltage enumeration is an explicit loop of
  independent solves over a discrete choice, not a hidden convergence loop.
- The converter switching term uses a smooth |P| (sqrt(P^2 + (eps P_r)^2)),
  so charging and discharging are differentiable.
- Insulation thickness is a smooth maximum of the minimum wall and the
  closed-form PD thickness (no iteration).
- Branching only on Python flags and instance names, never on symbols.

## Tests

- `tests/powertrain/test_electrical.py`: converter identities (rated
  efficiency, loss split, peak-efficiency load, sign symmetry), limits;
  cable resistance/mass identities, PD inversion round trip, insulation
  growth with voltage and altitude, loss sign; protection identities;
  DC/DC; ports/topology insertion; symbolic `asb.Opti` solves with MX.
- `tests/integration/test_halo_electrical.py`: flag off reproduces the
  reference; flag on closes (or max payload), electrical masses present,
  losses positive, bus voltage enumeration ordering.

## Acceptance

- Full suite passes; Tier 0 governance notebook passes with the new rows.
- `notebooks/tier15_electrical/electrical_verification.ipynb` executed.
- Docs updated (roadmap, interfaces, implementation notes).

## Progress and decisions

- 2026-10-03: plan written.

## Deferred

- Thermal (cooling mass and drag of converter losses): Tier 19.
- Nacelle-local buses, cross-strapping and redundancy: Tier 18.
- Device-class penalties on converter mass/efficiency; DC arc interruption
  mass at higher voltage; reflected-wave overvoltage at machine terminals.
- Electrical layer in the Tier 14 trajectory model.
