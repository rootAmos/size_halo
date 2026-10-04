"""Tier 18 redundancy: architecture counts and degraded states read from a topology (no variables, no solving).

The architecture is described by `topologies.RedundancyLayer` and lives in the topology as multiplicity: lane
motors (`motor` count / `propulsor` count), bus copies (`Bus.count`) and string contactors
(`protection_string` count). A failure case is an ordinary flight point with fewer active units:

- lane out: fewer active lanes per rotor (the rest carry the rotor torque);
- bus out: the failed bus's share of the lanes is lost and its sources feed the others through a tie;
- string out: the pack runs on fewer strings at the same SOC, so capacity, current rating, conductance and
  thermal mass scale with the active fraction.
"""
from dataclasses import dataclass, replace

from .components.battery import Battery
from .components.battery_ecm import EquivalentCircuitBattery


@dataclass(frozen=True)
class RedundancyCounts:
    count_lanes: int                # lane motors per rotor
    count_buses: int
    count_strings_battery: int
    count_ties: int                 # bus ties (count_buses - 1 when the buses are cross-strapped)


def redundancy_counts(topology):
    instances = topology.instances
    count_lanes = instances["motor"].count // instances["propulsor"].count
    count_buses = topology.buses["bus"].count if "bus" in topology.buses else 1
    count_strings = instances["protection_string"].count if "protection_string" in instances else 1
    count_ties = instances["protection_bus_tie"].count if "protection_bus_tie" in instances else 0
    return RedundancyCounts(count_lanes, count_buses, count_strings, count_ties)


@dataclass(frozen=True)
class DegradedState:
    """Active units at one flight point (all integers)."""
    count_lanes_active: int          # per rotor
    count_strings_active: int
    count_buses_failed: int
    counts: RedundancyCounts

    @property
    def fraction_strings_active(self):
        return self.count_strings_active / self.counts.count_strings_battery


def degraded_state(topology, condition):
    """Active lanes per rotor, active strings and failed buses for a flight condition (validated).

    `condition.active_lane_count` (None: all) is per rotor before any bus failure; each failed bus removes
    count_lanes / count_buses lanes per rotor more.
    """
    counts = redundancy_counts(topology)
    lanes = getattr(condition, "active_lane_count", None)
    lanes = counts.count_lanes if lanes is None else lanes
    strings = getattr(condition, "active_battery_string_count", None)
    strings = counts.count_strings_battery if strings is None else strings
    buses_failed = getattr(condition, "count_buses_failed", 0) or 0
    if not 1 <= lanes <= counts.count_lanes:
        raise ValueError(f"active_lane_count must be in 1..{counts.count_lanes}, got {lanes}.")
    if not 1 <= strings <= counts.count_strings_battery:
        raise ValueError(f"active_battery_string_count must be in 1..{counts.count_strings_battery}, got {strings}.")
    if buses_failed and buses_failed > counts.count_ties:
        raise ValueError(f"{buses_failed} failed bus(es) need at least as many bus ties; the topology has "
                         f"{counts.count_ties}.")
    lanes = lanes - buses_failed * counts.count_lanes // counts.count_buses
    if lanes < 1:
        raise ValueError("The failure state leaves no lane on a rotor.")
    return DegradedState(lanes, strings, buses_failed, counts)


def battery_with_strings(battery, fraction_active):
    """The pack with only `fraction_active` of its parallel strings connected (same SOC, cells and voltage).

    Equivalent circuit: the parallel count scales. Constant battery: capacity and power ratings scale and the
    resistance divides. A fraction of one returns the pack itself.
    """
    if fraction_active == 1:
        return battery
    if isinstance(battery, EquivalentCircuitBattery):
        return replace(battery, count_parallel=fraction_active * battery.count_parallel)
    if isinstance(battery, Battery):
        return replace(battery, energy_capacity_J=fraction_active * battery.energy_capacity_J,
                       resistance_ohm=battery.resistance_ohm / fraction_active,
                       max_discharge_power_W=fraction_active * battery.max_discharge_power_W,
                       max_charge_power_W=fraction_active * battery.max_charge_power_W)
    raise TypeError(f"No string model for {type(battery).__name__}.")


def battery_for_condition(topology, condition):
    """The pack as connected at this condition (strings isolated by a failure removed)."""
    battery = topology.instances["battery"].component
    return battery_with_strings(battery, degraded_state(topology, condition).fraction_strings_active)
