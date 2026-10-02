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
from .ports import port_specs_for


def build_series_hybrid(motor, generator, battery, turboshaft, gearbox, propulsor, count_rotors=1,
                        count_turbogenerators=1):
    """m x (turboshaft -> generator) -> bus <- battery; bus -> n x (motor -> gearbox -> rotor).

    The turboshaft fuel port is left unconnected as a boundary port.
    """
    topology = Topology()
    topology.add("turboshaft", turboshaft, port_specs_for(turboshaft), count=count_turbogenerators)
    topology.add("generator", generator, port_specs_for(generator), count=count_turbogenerators)
    topology.add("battery", battery, port_specs_for(battery))
    topology.add_bus("bus")
    topology.add("motor", motor, port_specs_for(motor), count=count_rotors)
    topology.add("gearbox", gearbox, port_specs_for(gearbox), count=count_rotors)
    topology.add("propulsor", propulsor, port_specs_for(propulsor), count=count_rotors)
    topology.connect("turboshaft.shaft", "generator.shaft")
    topology.connect("generator.electrical", "bus")
    topology.connect("battery.electrical", "bus")
    topology.connect("motor.electrical", "bus")
    topology.connect("motor.shaft", "gearbox.shaft_in")
    topology.connect("gearbox.shaft_out", "propulsor.shaft")
    return topology


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
