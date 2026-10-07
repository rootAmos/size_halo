# Redundancy: lanes per rotor, cross-strapped buses, battery strings, failure cases

Status: COMPLETED 2026-10-04. Tier 18 (roadmap "18 Redundancy and failure cases", review item 4).
Plan numbers 029 (Tier 22) and 031 (OpenVSP) are taken by other sessions.

## Goal and scope

- **Architecture parameters** (a topology description, no solving):
  - motor lanes per rotor: N lane motors (each with its own inverter, cable
    and contactor when the Tier 15 layer is on) driving one rotor gearbox
    through a combining input;
  - B cross-strapped DC buses, joined by bus-tie contactors and tie cables
    (`ProtectionUnit`, `Cable`), normally open;
  - S parallel battery strings, each behind its own isolation contactor
    (`ProtectionUnit`).
  - Masses are the existing components times their multiplicity plus the
    bus-tie and string-isolation hardware.
- **Failure cases** as extra flight points in the sizing problem (multipoint),
  each a degraded topology state built from reduced active counts:
  - lane out in hover: N - 1 lanes carry the rotor torque, within their torque
    limit and (thermal on) their temperature limit over the hover;
  - bus out in hover: the N/B lanes per rotor on the failed bus are lost and
    the sources on that bus feed the survivors through the bus tie
    (tie current, tie losses and its rating margin);
  - string out at the SOC floor in hover (all engines): S - 1 strings carry
    the battery share;
  - optional double failures: string out with one engine out; lane out with
    one engine out.
  - The existing one-engine-out hover stays.
- **Switch:** `HaloAssumptions.redundancy` (default False). With it off the
  topology, problem and reference are unchanged (900 kg, 210 kt, 14,037 lb,
  thermal on, AeroBuildup). With it on and all counts 1 and no failure cases
  the problem is identical (limiting case).
- **Not in scope:** asymmetric (one-rotor) degraded states and roll trim by
  differential collective; inverter short-time ratings (the inverter has no
  thermal model); fault transients and protection coordination; the
  trajectory model; the generator interconnect (gensets stay one per bus side
  in the count sense only).

## References

- Roadmap section "### 18".
- Plan 023 (Tier 15 components: `ProtectionUnit`, `Cable`; feeder rating
  125 %), plan 028/030 (thermal short-time ratings).
- SAE ARP4761 / ARP4754A practice: single failures must not be catastrophic;
  degraded modes evaluated as separate operating points.
- Dual-wound / multi-lane fault-tolerant machines (e.g. Mecrow et al.,
  "Design and testing of a four-phase fault-tolerant permanent-magnet
  machine for an engine fuel pump", IEEE Trans. Energy Conv. 2004): lanes
  isolated so one lane's failure leaves the others able to carry an overload.

## Assumptions

- **Lanes:** N identical lane motors per rotor (rubber machines at 1/N of the
  torque), on a combining gearbox input (equal speed, torques add). With the
  torque-density mass model (linear in torque) the healthy-state machine mass
  and losses are those of one machine; the cost of lanes is the failure-case
  sizing and, with the layer, per-lane feeder overheads. No combining-gearbox
  or clutch mass increment (assumed inside the AFDD drive weight).
- **Lane out:** the failed lane is declutched (a freewheel per lane,
  assumed), so its spinning losses vanish.
- **Lane out:** applied to every rotor at once (symmetric multiplicity). This
  is conservative for power and heat (both rotors run the degraded lane state)
  and needs no roll trim.
- **Buses:** B identical buses, each feeding N/B lanes per rotor and 1/B of
  the sources (requires N divisible by B). Lumped as one bus in the topology
  under symmetric operation (`Bus.count`). B - 1 ties, each a contactor and a
  2 m cable rated at 125 % of the rated current of one bus's sources (its
  generators' electrical rating and its strings' discharge rating) at the
  minimum bus voltage.
- **Bus out:** the failed bus's lanes are lost; its sources transfer through
  one tie. Tie current = (motor and fan demand) / B / bus voltage (first
  order, the tie loss is well below 1 %); the loss joins the bus demand.
- **Strings:** the pack (its parallel count is still the design variable) is
  split into S equal strings; each string contactor is rated at 125 % of the
  string's discharge current rating. String contactor drop is in series with
  the battery (exact, like the Tier 15 battery feeder).
- **String out:** S - 1 strings at the pack SOC: capacity, current rating,
  conductance and thermal mass scale by (S - 1) / S.
- **Failure hovers (Halo):** 60 s at MTOM, sea level, from the SOC floor
  (0.30) to the emergency floor (0.10), thermal history from the end of the
  take-off hover (as the engine-out hover), polarization developed.
- **Sensible set (reported):** N = 2, B = 2, S = 2; lane out, bus out, string
  out; double failures as sensitivities.

## Interfaces

- `core/topology.py`: `Topology.add_bus(name, domain, count=1)` (`Bus.count`);
  `Topology.connect(a, b, combine=False)`: with `combine=True` a direct
  connection may join counts where one is an integer multiple of the other
  (combiner / splitter): efforts equal, total flow conserved
  (`count x flow`). `connection_residuals` and `design_margins` honour it.
- `core/ports.py`: `port_flow_fields` (torque, current, fuel flow per domain).
- `powertrain/topologies.py`: `RedundancyLayer(count_lanes, count_buses,
  count_strings_battery, protection_string, protection_bus_tie,
  cable_bus_tie)`; `build_series_hybrid(..., redundancy=None)`.
- `powertrain/redundancy.py`: `redundancy_counts(topology)`,
  `degraded_state(topology, condition)`, `battery_with_strings(battery,
  fraction_active)`, `battery_for_condition(topology, condition)`.
- `performance/flight_point.py`: `FlightCondition.active_lane_count` (per
  rotor), `active_battery_string_count`, `count_buses_failed`; `FlightPoint`
  gains `redundancy` (`RedundancyPointResult`).
- `compatibility.operating_margins(..., components=None)` and
  `thermal.heat.evaluate_point_thermal(..., components=None)`: per-point
  component overrides (the degraded pack).
- `mission/segments.HoverSegment`: the three new counts.
- Halo: `HaloAssumptions.redundancy` and its parameters,
  `build_halo_redundancy(...)`, `failure_hover_cases`,
  `halo_failure_hovers`, `HaloSizingResult.failure_cases`,
  `count_lanes_motor`; `assumptions_tier18`; `solve_halo_sizing(...,
  staged_start=True)`.

## Symbolic considerations

- Counts are Python integers (topology multiplicity); ratings may be Opti
  variables (pack parallel count, machine torque).
- Degraded points are ordinary flight points with fewer active units: no
  loop. The tie current is an explicit expression of the point's demand.
- Branching only on integers, flags and names.

## Tests

- `tests/core/test_topology.py`: combiner/splitter residuals and validation.
- `tests/powertrain/test_redundancy.py`: multiplicity mass identities
  (lanes, ties, string contactors), the string-scaled pack (capacity, current,
  resistance, thermal time constant), N = 1 / B = 1 / S = 1 builds the plain
  topology, validation (N divisible by B), design margins with a combiner.
- `tests/performance/test_flight_point_redundancy.py`: one lane equals the
  plain point; lane torque = rotor torque / active lanes; lane out raises the
  per-lane torque by N / (N - 1); bus out drops N / B lanes and adds tie
  current and loss; string out scales the pack limit and voltage drop;
  symbolic `asb.Opti` solve with lanes.
- `tests/integration/test_halo_redundancy.py`: flag off unchanged topology;
  flag on with ones and no cases equals the reference problem; hardware and
  masses; trends (more lanes: lower per-lane rating, more total motor rating);
  the sized aircraft closes with every failure case.

## Implementation sequence

1. Topology combiner and bus count; tests.
2. Redundancy layer in `build_series_hybrid`; helpers; tests.
3. Flight point and mission degraded states; tests.
4. Halo switch, hardware, failure cases, warm start; tests.
5. Solves: reference unchanged; redundancy on (sensible set); sensitivities.
6. Notebook, docs, plan to completed, commit.

## Acceptance

- Full unittest suite passes; reference unchanged with defaults.
- `notebooks/tier18_redundancy/redundancy_verification.ipynb` executed.
- Roadmap row 18 `Implemented`; implementation notes section.

## Progress and decisions

- 2026-10-04: plan written; topology combiner/splitter and bus count,
  `RedundancyLayer`, degraded flight points, Halo option implemented.
- **Reference unchanged:** defaults 14,037 lb; redundancy on with all counts
  1 and no cases also 14,037 lb.
- **Coverage rule:** with one lane per rotor on each bus (N = B) the bus
  out is the lane-out state plus the tie, so the lane-out point is dropped;
  the duplicated binding motor-temperature constraints made IPOPT fail in
  restoration for 2 lanes / 2 buses.
- **Staged start:** bus out + string out from the single-lane design reached
  local infeasibility; with strings a failed solve restarts from the same
  problem without strings (`staged_start`).
- **Result (2/2/2, AeroBuildup, thermal on):** 14,342 lb (+139 kg); the
  bus-out (lane-out) motor temperature binds; string out does not bind
  (margin 0.12). Mass by architecture in the notebook and implementation
  notes.
- **Double failures** (string out + engine out, lane out + engine out): no
  converged solution from the 2/2/2 solution (IPOPT: infeasible). Open
  whether physically infeasible with the fixed engines.

## Deferred

- Double failures with an engine out: find a start or show infeasibility.
- Lane counts not a multiple of the bus count (cross-strapped motor feeders).

- Asymmetric degraded states (one rotor) with roll trim.
- Inverter short-time (thermal) ratings.
- Nacelle-local buses, generator cross-shafting, fault transients.
