"""A notional all-electric tiltrotor powertrain (plan 066), kept for future evaluations.

Two proprotors on a fixed wing, each driven by `count_channels_per_rotor` direct-drive motor channels (feeder
protection, cable, inverter, motor) from one battery on one DC bus. Every channel is rated so that the remaining
n - 1 channels of a rotor carry its hover shaft power after one channel (motor or inverter) fails.

Illustrative numbers only, not a closed design; each comes from the literature cited on its field:

- Fredericks, Sripad, Bower, Viswanathan, ACS Energy Letters 3 (2018) 2989-2994: battery metrics for electric
  VTOL (pack specific energy, hover discharge rates of several C).
- Siemens SP260D (2016): 260 kW continuous, 50 kg, direct drive; 5.2 kW/kg.
- Plan 023: inverter 20 kW/kg (between the NASA 19 kW/kg goal and SiC demonstrators).
- Maisel et al., NASA SP-2000-4517 (XV-15 history): tiltrotor disk loading about 13 lb/ft2 (620 N/m2).
- Johnson, Rotorcraft Aeromechanics (2013), ch. 3 and 7: hover figure of merit 0.7-0.8 for proprotors.

Run: `python examples/all_electric_tiltrotor.py` (prints the topology, masses, the one-channel-out hover and a
consistent hover operating point solved for the inverter DC voltage).
"""
from dataclasses import dataclass

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.core.ports import ElectricalPortValue, MechanicalPortValue
from aircraft_closure.core.topology import connection_residuals
from aircraft_closure.powertrain.compatibility import design_margins
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.cable import Cable
from aircraft_closure.powertrain.components.converters import Inverter
from aircraft_closure.powertrain.components.motor import Motor, rubber_machine
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.redundancy import degraded_state
from aircraft_closure.powertrain.topologies import ElectricalLayer, build_all_electric

acceleration_gravity_m_s2 = 9.80665


@dataclass(frozen=True)
class NotionalAllElectricTiltrotor:
    """Literature-sourced inputs of the notional aircraft (SI)."""
    mass_takeoff_kg: float = 1600.0                 # notional light cargo tiltrotor
    count_rotors: int = 2
    diameter_rotor_m: float = 4.5                   # disk loading 490 N/m2, below the XV-15's 620 (NASA SP-2000-4517)
    figure_of_merit_hover: float = 0.75             # proprotor hover FM 0.7-0.8 (Johnson, Rotorcraft Aeromechanics)
    speed_tip_hover_m_s: float = 200.0              # XV-15 hover tip speed about 235 m/s; lower for noise (assumed)
    mass_per_disk_area_rotor_kg_m2: float = 3.0     # framework rubber-rotor default (topologies.SeriesHybridSizing)
    # Battery (Fredericks et al. 2018): pack 200 Wh/kg, about 5C peak (1 kW/kg), SOC window 0.10-0.95.
    energy_capacity_battery_J: float = 90.0 * 3.6e6
    specific_energy_pack_J_kg: float = 200.0 * 3600.0
    specific_power_pack_W_kg: float = 1000.0
    min_soc: float = 0.10
    max_soc: float = 0.95
    voltage_open_circuit_V: float = 800.0           # 800 V class bus (plan 023 defaults)
    resistance_energy_product_ohm_J: float = 1.8e6  # framework reference pack (topologies.SeriesHybridSizing)
    # Channel: direct-drive motor (Siemens SP260D, 5.2 kW/kg continuous) and SiC inverter (20 kW/kg, plan 023).
    specific_power_motor_W_kg: float = 5200.0
    efficiency_peak_motor: float = 0.95
    specific_power_inverter_W_kg: float = 20000.0
    length_cable_motor_m: float = 6.0               # fuselage bay to wingtip nacelle (assumed)
    length_cable_battery_m: float = 2.0
    voltage_min_bus_V: float = 600.0                # sizing voltage for feeder currents (assumed)

    def area_disk_m2(self):
        return np.pi * (self.diameter_rotor_m / 2) ** 2

    def speed_rotor_hover_rad_s(self):
        return self.speed_tip_hover_m_s / (self.diameter_rotor_m / 2)

    def thrust_hover_rotor_N(self):
        """Hover thrust per rotor at weight (download neglected in this notional case)."""
        return self.mass_takeoff_kg * acceleration_gravity_m_s2 / self.count_rotors


def notional_propulsor(data, max_shaft_power_W=1e6):
    area_m2 = data.area_disk_m2()
    return ActuatorDiskPropulsor(area_disk_m2=area_m2, mass_kg=data.mass_per_disk_area_rotor_kg_m2 * area_m2,
                                 coefficient_of_performance=data.figure_of_merit_hover,
                                 max_shaft_power_W=max_shaft_power_W)


def power_shaft_hover_rotor_W(data, atmosphere=None):
    """Actuator-disk hover shaft power per rotor (closed form, no solve)."""
    atmosphere = atmosphere or asb.Atmosphere(altitude=0.0)
    return notional_propulsor(data).evaluate(0.0, atmosphere, thrust_N=data.thrust_hover_rotor_N()).shaft_power_W


def build_notional_all_electric_tiltrotor(count_channels_per_rotor=3, data=None):
    """The notional aircraft's topology, each channel rated for its rotor's hover power with one channel out."""
    if count_channels_per_rotor < 2:
        raise ValueError("One channel out needs at least two channels per rotor.")
    data = data or NotionalAllElectricTiltrotor()
    power_rotor_W = power_shaft_hover_rotor_W(data)
    power_rated_channel_W = power_rotor_W / (count_channels_per_rotor - 1)
    # The rotor (drive) is rated to all its channels together, as the series hybrid rates the rotor to its gearbox.
    propulsor = notional_propulsor(data, count_channels_per_rotor * power_rated_channel_W)
    speed_rad_s = data.speed_rotor_hover_rad_s()
    # McDonald machine about the hover point: rated power 1.25 x the peak-efficiency point, 1.5 x speed margin.
    motor = rubber_machine(Motor, speed_rad_s, power_rated_channel_W / (1.25 * speed_rad_s),
                           efficiency_peak=data.efficiency_peak_motor, torque_ratio=2.0, power_ratio=1.25,
                           speed_ratio=1.5, specific_power_W_kg=data.specific_power_motor_W_kg)
    power_rated_inverter_W = power_rated_channel_W / data.efficiency_peak_motor
    current_feeder_A = power_rated_inverter_W / data.voltage_min_bus_V
    power_max_battery_W = (data.energy_capacity_battery_J / data.specific_energy_pack_J_kg
                           * data.specific_power_pack_W_kg)
    current_battery_A = power_max_battery_W / data.voltage_min_bus_V
    battery = Battery(energy_capacity_J=data.energy_capacity_battery_J,
                      voltage_open_circuit_V=data.voltage_open_circuit_V,
                      resistance_ohm=data.resistance_energy_product_ohm_J / data.energy_capacity_battery_J,
                      specific_energy_J_kg=data.specific_energy_pack_J_kg,
                      specific_power_W_kg=data.specific_power_pack_W_kg,
                      max_discharge_power_W=power_max_battery_W, max_charge_power_W=0.5 * power_max_battery_W,
                      min_soc=data.min_soc, max_soc=data.max_soc)
    electrical = ElectricalLayer(
        inverter_motor=Inverter(power_rated_W=power_rated_inverter_W,
                                specific_power_W_kg=data.specific_power_inverter_W_kg),
        cable_motor=Cable(length_m=data.length_cable_motor_m, max_current_A=current_feeder_A),
        protection_motor=ProtectionUnit(max_current_A=current_feeder_A),
        inverter_generator=None, cable_generator=None, protection_generator=None,
        cable_battery=Cable(length_m=data.length_cable_battery_m, max_current_A=current_battery_A),
        protection_battery=ProtectionUnit(max_current_A=current_battery_A))
    return build_all_electric(motor, battery, propulsor, count_rotors=data.count_rotors,
                              count_channels_per_rotor=count_channels_per_rotor, electrical=electrical)


def hover_port_values(topology, data, voltage_dc_inverter_V, current_battery_A, soc=0.9, atmosphere=None):
    """Port values of an all-channels hover at weight, given the inverter DC voltage and the battery current.

    Both may be symbolic. Walks each motor feeder from the motor back to the bus and the battery feeder from the
    bus to the pack; two residuals stay open, the bus current balance and the battery terminal voltage against
    its feeder, and fix the two inputs when a caller constrains `connection_residuals` to zero.
    """
    instances = topology.instances
    count_lanes = instances["motor"].count // instances["propulsor"].count
    speed_rad_s = data.speed_rotor_hover_rad_s()
    torque_rotor_Nm = power_shaft_hover_rotor_W(data, atmosphere) / speed_rad_s
    torque_motor_Nm = torque_rotor_Nm / count_lanes
    motor = instances["motor"].component.evaluate(speed_rad_s, torque_motor_Nm, voltage_dc_inverter_V)
    inverter = instances["inverter_motor"].component.evaluate(motor.power_electric_W, voltage_dc_inverter_V)
    current_feeder_A = inverter.current_dc_A
    voltage_cable_in_V = voltage_dc_inverter_V + instances["cable_motor"].component.evaluate(
        current_feeder_A).voltage_drop_V
    voltage_bus_V = voltage_cable_in_V + instances["protection_motor"].component.evaluate(
        current_feeder_A).voltage_drop_V
    voltage_battery_cable_in_V = voltage_bus_V + instances["cable_battery"].component.evaluate(
        current_battery_A).voltage_drop_V
    voltage_protection_in_V = voltage_battery_cable_in_V + instances["protection_battery"].component.evaluate(
        current_battery_A).voltage_drop_V
    battery = instances["battery"].component.evaluate(current_battery_A, soc)
    motor_ac = ElectricalPortValue(voltage_dc_inverter_V, motor.current_A)
    return {
        "motor.shaft": MechanicalPortValue(speed_rad_s, torque_motor_Nm),
        "propulsor.shaft": MechanicalPortValue(speed_rad_s, torque_rotor_Nm),
        "motor.electrical": motor_ac,
        "inverter_motor.ac": motor_ac,
        "inverter_motor.dc": ElectricalPortValue(voltage_dc_inverter_V, current_feeder_A),
        "cable_motor.output": ElectricalPortValue(voltage_dc_inverter_V, current_feeder_A),
        "cable_motor.input": ElectricalPortValue(voltage_cable_in_V, current_feeder_A),
        "protection_motor.output": ElectricalPortValue(voltage_cable_in_V, current_feeder_A),
        "protection_motor.input": ElectricalPortValue(voltage_bus_V, current_feeder_A),
        "cable_battery.output": ElectricalPortValue(voltage_bus_V, current_battery_A),
        "cable_battery.input": ElectricalPortValue(voltage_battery_cable_in_V, current_battery_A),
        "protection_battery.output": ElectricalPortValue(voltage_battery_cable_in_V, current_battery_A),
        "protection_battery.input": ElectricalPortValue(voltage_protection_in_V, current_battery_A),
        "battery.electrical": ElectricalPortValue(battery.voltage_V, current_battery_A),
    }


def solve_hover_voltage(topology, data, soc=0.9):
    """Inverter DC voltage of the all-channels hover from `asb.Opti`, with every connection residual at zero."""
    opti = asb.Opti()
    voltage_dc_inverter_V = opti.variable(init_guess=data.voltage_open_circuit_V, scale=100.0, lower_bound=100.0)
    current_battery_A = opti.variable(init_guess=100.0, scale=100.0)
    residuals = connection_residuals(topology, hover_port_values(topology, data, voltage_dc_inverter_V,
                                                                 current_battery_A, soc))
    # Most residuals are identities of the feeder walk; only the bus current and battery voltage carry unknowns.
    opti.subject_to([residual.value / 100.0 == 0 for residual in residuals if not _is_zero(residual.value)])
    solution = opti.solve(verbose=False)
    voltage_V, current_A = float(solution(voltage_dc_inverter_V)), float(solution(current_battery_A))
    values = connection_residuals(topology, hover_port_values(topology, data, voltage_V, current_A, soc))
    return voltage_V, current_A, max(abs(float(residual.value)) for residual in values)


def _is_zero(value):
    """True for a residual that is identically zero (a number, or a CasADi expression simplified to 0)."""
    if isinstance(value, (int, float)):
        return value == 0
    return bool(value.is_zero()) if hasattr(value, "is_zero") else False


@dataclass(frozen=True)
class OneChannelOut:
    """Condition with one channel (motor or inverter) lost on one rotor."""
    active_lane_count: int
    count_rotors_lane_out: int = 1


def main():
    data = NotionalAllElectricTiltrotor()
    power_rotor_W = power_shaft_hover_rotor_W(data)
    print(f"Notional all-electric tiltrotor: {data.mass_takeoff_kg:.0f} kg, hover shaft power "
          f"{power_rotor_W / 1e3:.1f} kW per rotor (ISA sea level)")
    for count_channels in (2, 3, 4, 5, 6):
        topology = build_notional_all_electric_tiltrotor(count_channels, data)
        motor = topology.instances["motor"].component
        inverter = topology.instances["inverter_motor"].component
        count_motors = topology.instances["motor"].count
        state = degraded_state(topology, OneChannelOut(active_lane_count=count_channels - 1))
        mass_kg = count_motors * (motor.get_mass() + inverter.get_mass())
        print(f"  {count_channels} channels/rotor: {motor.power_rated_W / 1e3:6.1f} kW per channel, installed "
              f"{count_motors * motor.power_rated_W / 1e3:6.1f} kW, motors+inverters {mass_kg:5.1f} kg; one out: "
              f"{state.power_shaft_lane_W(power_rotor_W) / 1e3:5.1f} kW per channel on the failed rotor, "
              f"{state.power_shaft_lane_W(power_rotor_W, degraded=False) / 1e3:5.1f} kW on the healthy one")
    topology = build_notional_all_electric_tiltrotor(3, data)
    voltage_V, current_A, residual_max = solve_hover_voltage(topology, data)
    print(f"Hover, 3 channels/rotor: inverter DC voltage {voltage_V:.1f} V, battery current {current_A:.1f} A, "
          f"largest connection residual {residual_max:.2e}")
    battery = topology.instances["battery"].component
    power_battery_W = battery.evaluate(current_A, 0.9).power_electric_W
    print(f"Battery output in hover {power_battery_W / 1e3:.1f} kW of {battery.max_discharge_power_W / 1e3:.0f} kW")
    margins = {margin.label: float(margin.value) for margin in design_margins(topology)}
    label = "motor.shaft->propulsor.shaft max_power_W"
    print(f"Design margin {label}: {margins[label]:+.3f}")


if __name__ == "__main__":
    main()
