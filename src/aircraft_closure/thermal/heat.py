"""Heat loads, heat rejection and lumped component temperatures at one flight point (Tier 19, plan 028).

Discipline layer: it reads losses that the component models already return
and builds expressions only (no variables, no constraints; margins are
returned for the caller to constrain).

* `HeatLoad(source, power_W, count)`: the loss of one unit of an instance
  (`source` is the topology instance name, e.g. "motor", "generator_gearbox",
  and later Tier 15's "inverter" or "cable") and the number of units active.
  A new loss source plugs in by adding a `HeatLoad`; nothing here lists the
  sources.
* `thermal_parameters(component)`: thermal capacitance C = c x component
  mass and resistance R = (T_max - T_coolant) / loss at the continuous
  rating, so the steady state at the continuous rating sits exactly on the
  temperature limit and the component's existing rating keeps its meaning.
  Machines: rated power at the rated speed (the loss model's peak-efficiency
  speed when it has one, else the maximum speed). Batteries: the rated
  discharge current at mid SOC with the polarization developed (the
  equivalent-circuit pack) or the rated power at the open-circuit voltage
  (the constant battery).
* `evaluate_point_thermal(...)`: each thermal-modelled instance's end and
  interval-mean temperatures and the heat it passes to the coolant,
  (T - T_c) / R, which is less than its loss while its thermal mass warms
  (a cold-start hover) and more while it cools; other sources pass their loss
  straight through. The installed heat exchanger rejects every source except
  `InstalledCooling.sources_excluded` (so a new source is rejected there by
  default): its drag or fan power follows the interval-mean heat, and its
  rating margin the end-of-interval heat (the peak within the interval).
  Margins: temperature at the end of the interval against the limit
  (normalized by the allowed rise), and the exchanger rating.
"""
from dataclasses import dataclass
from typing import Any

from aircraft_closure.core.margins import Margin, margin_below
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery


@dataclass(frozen=True)
class HeatLoad:
    source: str
    power_W: Any            # per unit
    count: Any = 1

    def total_W(self):
        return self.count * self.power_W


def total_heat_W(heat_loads, sources_excluded=()):
    """Sum of count x power over every load whose source is not in `sources_excluded`."""
    return sum((load.total_W() for load in heat_loads if load.source not in sources_excluded), 0.0)


@dataclass(frozen=True)
class ThermalParameters:
    capacity_J_K: Any
    resistance_K_W: Any
    power_loss_continuous_W: Any


def power_loss_continuous_W(component):
    """Loss at the component's continuous rating (see the module docstring)."""
    if isinstance(component, EquivalentCircuitBattery):
        current_A = component.get_limits().max_discharge_current_A
        return current_A**2 * sum(component.resistances_ohm(0.5))
    if isinstance(component, Battery):
        return (component.max_discharge_power_W / component.voltage_open_circuit_V)**2 * component.resistance_ohm
    speed_rated_rad_s = getattr(component.loss_model, "speed_peak_efficiency_rad_s", component.max_speed_rad_s)
    return component.loss_model.evaluate(speed_rated_rad_s, component.power_rated_W / speed_rated_rad_s,
                                         component.max_voltage_V)


def thermal_parameters(component):
    thermal = component.thermal_model
    loss_W = power_loss_continuous_W(component)
    return ThermalParameters(
        capacity_J_K=thermal.specific_heat_J_kg_K * component.get_mass(),
        resistance_K_W=(thermal.temperature_max_C - thermal.temperature_coolant_C) / loss_W,
        power_loss_continuous_W=loss_W)


@dataclass(frozen=True)
class PointThermal:
    heat_loads: tuple
    power_heat_W: Any                   # interval-mean heat into the heat exchanger (0 without one)
    power_heat_end_W: Any               # at the end of the interval
    power_heat_equivalent_W: Any        # end heat at the exchanger's reference temperature difference (rating demand)
    cooling: Any                        # HeatExchangerResult at the mean heat, or None
    temperatures_end_C: dict            # instance name -> temperature at the end of the interval
    temperatures_mean_C: dict           # instance name -> interval-mean temperature
    power_to_coolant_W: dict            # instance name -> interval-mean heat to the coolant, per unit
    margins: tuple


def evaluate_point_thermal(powertrain, heat_loads, atmosphere, velocity_m_s, mode, duration_s=0.0,
                           temperature_start_C=None, label="point"):
    """`temperature_start_C`: None (steady state, no history) or a dict of start temperatures by instance name."""
    margins, temperatures_end_C, temperatures_mean_C, mean_W, end_W = [], {}, {}, {}, {}
    instances = powertrain.topology.instances
    for load in heat_loads:
        component = instances[load.source].component if load.source in instances else None
        thermal = getattr(component, "thermal_model", None)
        if thermal is None:
            mean_W[load.source] = end_W[load.source] = load.power_W
            continue
        p = thermal_parameters(component)
        start_C = None if temperature_start_C is None else temperature_start_C[load.source]
        end_C = thermal.temperature_end_C(load.power_W, duration_s, p.capacity_J_K, p.resistance_K_W, start_C)
        average_C = thermal.temperature_mean_C(load.power_W, duration_s, p.capacity_J_K, p.resistance_K_W, start_C)
        temperatures_end_C[load.source], temperatures_mean_C[load.source] = end_C, average_C
        mean_W[load.source] = (average_C - thermal.temperature_coolant_C) / p.resistance_K_W
        end_W[load.source] = (end_C - thermal.temperature_coolant_C) / p.resistance_K_W
        margins.append(Margin(f"{label}: {load.source} temperature_C", (thermal.temperature_max_C - end_C)
                              / (thermal.temperature_max_C - thermal.temperature_coolant_C)))
    cooling = getattr(powertrain, "cooling", None)
    power_heat_W = power_heat_end_W = power_heat_equivalent_W = 0.0
    cooling_result = None
    if cooling is not None:
        exchanger = cooling.heat_exchanger
        rejected = [load for load in heat_loads if load.source not in cooling.sources_excluded]
        power_heat_W = sum((load.count * mean_W[load.source] for load in rejected), 0.0)
        power_heat_end_W = sum((load.count * end_W[load.source] for load in rejected), 0.0)
        cooling_result = exchanger.evaluate(power_heat_W, atmosphere, velocity_m_s, fan=(mode == "hover"))
        power_heat_equivalent_W = power_heat_end_W * exchanger.delta_temperature_ref_C / cooling_result.delta_temperature_C
        margins.append(margin_below(f"{label}: heat_exchanger power_heat_W", power_heat_equivalent_W,
                                    exchanger.power_rated_W))
    return PointThermal(heat_loads=tuple(heat_loads), power_heat_W=power_heat_W, power_heat_end_W=power_heat_end_W,
                        power_heat_equivalent_W=power_heat_equivalent_W, cooling=cooling_result,
                        temperatures_end_C=temperatures_end_C, temperatures_mean_C=temperatures_mean_C,
                        power_to_coolant_W=mean_W, margins=tuple(margins))


def coolant_temperatures_C(powertrain):
    """Start temperatures at the coolant temperature for every thermal-modelled instance (a cold start)."""
    return {name: instance.component.thermal_model.temperature_coolant_C
            for name, instance in powertrain.topology.instances.items()
            if getattr(instance.component, "thermal_model", None) is not None}
