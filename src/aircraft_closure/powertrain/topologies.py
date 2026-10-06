"""Reference powertrain topologies; descriptions only, never solved here."""
from dataclasses import dataclass
from typing import Any

from aircraft_closure.core.topology import Topology
from .components.battery import Battery
from .components.gearbox import Gearbox
from .components.generator import Generator
from .components.motor import Motor, rubber_machine
from .components.propulsor import ActuatorDiskPropulsor
from .components.turboshaft import SimpleTurboshaft
from .ports import port_specs_for, rectifier_port_specs


@dataclass(frozen=True)
class ElectricalLayer:
    """Tier 15 feeder components, one set per feeder type (copied by the machine counts).

    Motor feeder: bus -> protection_motor -> cable_motor -> inverter_motor -> motor.
    Generator feeder: generator -> inverter_generator (active rectifier) -> cable_generator ->
    protection_generator -> bus. Battery feeder: battery -> protection_battery -> cable_battery ->
    [dcdc ->] bus. `dcdc` None (the default) lets the pack set the bus voltage.
    """
    inverter_motor: Any
    cable_motor: Any
    protection_motor: Any
    inverter_generator: Any
    cable_generator: Any
    protection_generator: Any
    cable_battery: Any
    protection_battery: Any
    dcdc: Any = None


@dataclass(frozen=True)
class RedundancyLayer:
    """Tier 18 redundancy architecture (a description; failure states are flight-point inputs).

    - `count_lanes`: lane motors per rotor (each with its own feeder when the Tier 15 layer is on), on a
      combining gearbox input: equal speed, torques add.
    - `count_buses`: identical cross-strapped DC buses, each feeding count_lanes / count_buses lanes per rotor
      and an equal share of the sources; `count_buses - 1` normally open ties, each `protection_bus_tie` +
      `cable_bus_tie` (its ports are boundaries: no current in normal operation).
    - `count_strings_battery`: the pack split into isolated parallel strings, each behind its own
      `protection_string` contactor (a splitter from the pack).
    Hardware is only added for counts above one, so all ones build the plain topology.
    """
    count_lanes: int = 1
    count_buses: int = 1
    count_strings_battery: int = 1
    protection_string: Any = None
    protection_bus_tie: Any = None
    cable_bus_tie: Any = None

    def __post_init__(self):
        for name in ("count_lanes", "count_buses", "count_strings_battery"):
            value = getattr(self, name)
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer, got {value!r}.")
        if self.count_lanes % self.count_buses != 0:
            raise ValueError(f"Each bus feeds an equal share of the lanes: count_lanes {self.count_lanes} must be a "
                             f"multiple of count_buses {self.count_buses}.")
        if self.count_strings_battery > 1 and self.protection_string is None:
            raise ValueError("Battery strings need a protection_string contactor.")
        if self.count_buses > 1 and (self.protection_bus_tie is None or self.cable_bus_tie is None):
            raise ValueError("Cross-strapped buses need a protection_bus_tie and a cable_bus_tie.")


def build_series_hybrid(motor, generator, battery, turboshaft, gearbox, propulsor, count_rotors=1,
                        count_turbogenerators=1, generator_gearbox=None, electrical=None, redundancy=None):
    """m x (turboshaft [-> generator_gearbox] -> generator) -> bus <- battery; bus -> n x (motor -> gearbox -> rotor).

    The turboshaft fuel port is left unconnected as a boundary port. `generator_gearbox` (optional) is a
    step-up gearbox between the engine output shaft and the generator (reduction_ratio < 1). `electrical`
    (optional `ElectricalLayer`, Tier 15) inserts inverters, cables, protection and an optional DC/DC
    converter between the machines, the battery and the bus; None connects them to the bus directly.
    `redundancy` (optional `RedundancyLayer`, Tier 18): `motor` is then one lane motor (count_rotors x
    count_lanes copies), the bus has `count_buses` copies with their ties, and the pack feeds the bus
    through its string contactors.
    """
    r = redundancy if redundancy is not None else RedundancyLayer()
    count_motors = count_rotors * r.count_lanes
    topology = Topology()
    topology.add("turboshaft", turboshaft, port_specs_for(turboshaft), count=count_turbogenerators)
    topology.add("generator", generator, port_specs_for(generator), count=count_turbogenerators)
    topology.add("battery", battery, port_specs_for(battery))
    topology.add_bus("bus", count=r.count_buses)
    topology.add("motor", motor, port_specs_for(motor), count=count_motors)
    topology.add("gearbox", gearbox, port_specs_for(gearbox), count=count_rotors)
    topology.add("propulsor", propulsor, port_specs_for(propulsor), count=count_rotors)
    if generator_gearbox is None:
        topology.connect("turboshaft.shaft", "generator.shaft")
    else:
        topology.add("generator_gearbox", generator_gearbox, port_specs_for(generator_gearbox), count=count_turbogenerators)
        topology.connect("turboshaft.shaft", "generator_gearbox.shaft_in")
        topology.connect("generator_gearbox.shaft_out", "generator.shaft")
    battery_port, battery_combine = "battery.electrical", False
    if r.count_strings_battery > 1:
        topology.add("protection_string", r.protection_string, port_specs_for(r.protection_string),
                     count=r.count_strings_battery)
        topology.connect("battery.electrical", "protection_string.input", combine=True)
        battery_port, battery_combine = "protection_string.output", True
    if r.count_buses > 1:
        # Normally open ties: the outer ports stay unconnected (boundaries) in normal operation.
        topology.add("protection_bus_tie", r.protection_bus_tie, port_specs_for(r.protection_bus_tie),
                     count=r.count_buses - 1)
        topology.add("cable_bus_tie", r.cable_bus_tie, port_specs_for(r.cable_bus_tie), count=r.count_buses - 1)
        topology.connect("protection_bus_tie.output", "cable_bus_tie.input")
    if electrical is None:
        topology.connect("generator.electrical", "bus")
        topology.connect(battery_port, "bus")
        topology.connect("motor.electrical", "bus")
    else:
        _add_electrical_layer(topology, electrical, count_motors, count_turbogenerators, battery_port,
                              battery_combine)
    topology.connect("motor.shaft", "gearbox.shaft_in", combine=r.count_lanes > 1)
    topology.connect("gearbox.shaft_out", "propulsor.shaft")
    return topology


def _add_electrical_layer(topology, e, count_motors, count_turbogenerators, battery_port="battery.electrical",
                          battery_combine=False):
    for name, count in (("protection_motor", count_motors), ("cable_motor", count_motors),
                        ("inverter_motor", count_motors), ("cable_generator", count_turbogenerators),
                        ("protection_generator", count_turbogenerators), ("protection_battery", 1),
                        ("cable_battery", 1)):
        component = getattr(e, name)
        topology.add(name, component, port_specs_for(component), count=count)
    topology.add("inverter_generator", e.inverter_generator, rectifier_port_specs(), count=count_turbogenerators)
    topology.connect("protection_motor.input", "bus")
    topology.connect("protection_motor.output", "cable_motor.input")
    topology.connect("cable_motor.output", "inverter_motor.dc")
    topology.connect("inverter_motor.ac", "motor.electrical")
    topology.connect("generator.electrical", "inverter_generator.ac")
    topology.connect("inverter_generator.dc", "cable_generator.input")
    topology.connect("cable_generator.output", "protection_generator.input")
    topology.connect("protection_generator.output", "bus")
    topology.connect(battery_port, "protection_battery.input", combine=battery_combine)
    topology.connect("protection_battery.output", "cable_battery.input")
    if e.dcdc is None:
        topology.connect("cable_battery.output", "bus")
    else:
        topology.add("dcdc", e.dcdc, port_specs_for(e.dcdc))
        topology.connect("cable_battery.output", "dcdc.input")
        topology.connect("dcdc.output", "bus")


def build_mechanical_tiltrotor(turboshaft, gearbox, propulsor, count_rotors=2):
    """n x (turboshaft -> gearbox -> rotor), as on the XV-15.

    The rotor interconnect (cross-shaft) carries power only after an engine
    failure; it is a mass item on the airframe, not a port connection here.
    """
    topology = Topology()
    topology.add("turboshaft", turboshaft, port_specs_for(turboshaft), count=count_rotors)
    topology.add("gearbox", gearbox, port_specs_for(gearbox), count=count_rotors)
    topology.add("propulsor", propulsor, port_specs_for(propulsor), count=count_rotors)
    topology.connect("turboshaft.shaft", "gearbox.shaft_in")
    topology.connect("gearbox.shaft_out", "propulsor.shaft")
    return topology


@dataclass(frozen=True)
class SeriesHybridSizing:
    """Rubber-scaled series-hybrid ratings; every field may be an Opti variable.

    Defaults reproduce the four-rotor reference (100 kW motors, 400 kW
    generator, 600 kW turboshaft, 144 MJ / 400 kW battery, 10 m2 rotors).
    Machines follow McDonald's eq. 4 ratios (kQ 2.5, kP 1.25, kw 2.5 about the
    peak point); the battery is reference packs in parallel, so resistance
    scales inversely with capacity; rotor mass scales with disk area; the
    gearbox is rated to the motor and the rotor to the gearbox output.
    """
    torque_peak_motor_Nm: Any = 200.0
    torque_peak_generator_Nm: Any = 800.0
    power_rated_turboshaft_W: Any = 600000.0
    power_max_discharge_battery_W: Any = 400000.0
    energy_capacity_battery_J: Any = 144000000.0
    area_disk_m2: Any = 10.0
    count_rotors: int = 4
    speed_peak_motor_rad_s: Any = 400.0
    speed_peak_generator_rad_s: Any = 400.0
    reduction_ratio: Any = 4.0
    mass_per_disk_area_kg_m2: Any = 3.0
    resistance_energy_product_ohm_J: Any = 1800000.0
    battery_mass_smoothing_kg: Any = None


def build_series_hybrid_from_sizing(sizing):
    ratios = dict(torque_ratio=2.5, power_ratio=1.25, speed_ratio=2.5)
    motor = rubber_machine(Motor, sizing.speed_peak_motor_rad_s, sizing.torque_peak_motor_Nm, **ratios)
    generator = rubber_machine(Generator, sizing.speed_peak_generator_rad_s, sizing.torque_peak_generator_Nm, **ratios)
    gearbox = Gearbox(reduction_ratio=sizing.reduction_ratio, power_rated_W=motor.power_rated_W)
    propulsor = ActuatorDiskPropulsor(area_disk_m2=sizing.area_disk_m2,
                                      mass_kg=sizing.mass_per_disk_area_kg_m2 * sizing.area_disk_m2,
                                      max_shaft_power_W=gearbox.power_rated_W * gearbox.efficiency)
    battery = Battery(energy_capacity_J=sizing.energy_capacity_battery_J,
                      resistance_ohm=sizing.resistance_energy_product_ohm_J / sizing.energy_capacity_battery_J,
                      max_discharge_power_W=sizing.power_max_discharge_battery_W,
                      max_charge_power_W=0.5 * sizing.power_max_discharge_battery_W,
                      mass_smoothing_kg=sizing.battery_mass_smoothing_kg)
    turboshaft = SimpleTurboshaft(power_rated_W=sizing.power_rated_turboshaft_W)
    return build_series_hybrid(motor, generator, battery, turboshaft, gearbox, propulsor,
                               count_rotors=sizing.count_rotors)
