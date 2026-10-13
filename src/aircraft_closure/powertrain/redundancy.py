"""Tier 18 redundancy: architecture counts and degraded states read from a topology (no variables, no solving).

The architecture is described by `topologies.RedundancyLayer` and lives in the topology as multiplicity: lane
motors (`motor` count / `propulsor` count), bus copies (`Bus.count`) and string contactors
(`protection_string` count). A failure case is an ordinary flight point with fewer active units:

- lane out: fewer active lanes per rotor (the rest carry the rotor torque), on every rotor or, with
  `count_rotors_lane_out` (plan 066), on that many rotors only (one channel lost on one rotor);
- bus out: the failed bus's share of the lanes is lost and its sources feed the others through a tie;
- string out: the pack runs on fewer strings at the same SOC, so capacity, current rating, conductance and
  thermal mass scale with the active fraction.
"""
from dataclasses import dataclass, replace
from typing import Any

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
    count_lanes_active: int          # per degraded rotor (every rotor unless count_rotors_degraded says otherwise)
    count_strings_active: int
    count_buses_failed: int
    counts: RedundancyCounts
    count_rotors_degraded: Any = None   # plan 066: rotors with lanes out; None: every rotor (symmetric)

    @property
    def fraction_strings_active(self):
        return self.count_strings_active / self.counts.count_strings_battery

    @property
    def count_lanes_active_healthy(self):
        """Active lanes on a rotor without a lane failure (only failed buses remove lanes there)."""
        if self.count_rotors_degraded is None:
            return self.count_lanes_active
        return self.counts.count_lanes - self.count_buses_failed * self.counts.count_lanes // self.counts.count_buses

    def power_shaft_lane_W(self, power_shaft_rotor_W, degraded=True):
        """Shaft power per active lane: the rotor's shaft power shared equally by its active lanes.

        On a degraded rotor of n lanes with one out this is P_rotor / (n - 1), the channel sizing case.
        """
        lanes = self.count_lanes_active if degraded else self.count_lanes_active_healthy
        return power_shaft_rotor_W / lanes


def degraded_state(topology, condition):
    """Active lanes per rotor, active strings and failed buses for a flight condition (validated).

    `condition.active_lane_count` (None: all) is per rotor before any bus failure; each failed bus removes
    count_lanes / count_buses lanes per rotor more. `condition.count_rotors_lane_out` (plan 066; None: every
    rotor) limits the lane failure to that many rotors, e.g. one channel lost on one rotor.
    """
    counts = redundancy_counts(topology)
    lanes = getattr(condition, "active_lane_count", None)
    lanes = counts.count_lanes if lanes is None else lanes
    strings = getattr(condition, "active_battery_string_count", None)
    strings = counts.count_strings_battery if strings is None else strings
    buses_failed = getattr(condition, "count_buses_failed", 0) or 0
    rotors_degraded = getattr(condition, "count_rotors_lane_out", None)
    if rotors_degraded is not None:
        count_rotors = topology.instances["propulsor"].count
        if not isinstance(rotors_degraded, int) or not 1 <= rotors_degraded <= count_rotors:
            raise ValueError(f"count_rotors_lane_out must be an integer in 1..{count_rotors}, "
                             f"got {rotors_degraded!r}.")
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
    return DegradedState(lanes, strings, buses_failed, counts, rotors_degraded)


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
