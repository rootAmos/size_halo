"""Per-bus electrical state of a hover with failed turbogenerators (plan 040): the asymmetric engine-out.

The lumped flight point treats all buses as one junction: one power balance, the battery current shared equally by
the strings. That is exact while every bus has the same sources. A failed turbogenerator breaks the symmetry: with
two cross-strapped buses, each with an equal share of the generators, the battery strings and the lane motors, the
bus that lost its generator ("bus A") and the healthy one ("bus B") operate differently. This module writes them as
two buses:

* each bus b has its own battery string group (the strings are isolated: `count_strings / count_buses` per bus), its
  own terminal voltage V_b (the string group's terminal voltage less its contactor drop), its own generators (bus A
  keeps any it did not lose) and its own lane motors (`count_lanes / count_buses` per rotor);
* the lanes of a rotor share its gearbox input torque unequally: the bus A lane torque is a variable and the bus B
  lane carries the rest (a combining gearbox; the controller may shift torque to the healthy bus within the lane
  limits); the rotors stay symmetric (each bus feeds a lane on every rotor), so there is no roll or yaw imbalance;
* the bus tie (`bus_tie`): "open" carries no current, so each bus balances on its own sources; "closed" (protection
  logic closes the normally open tie after the failure) carries current I_t from bus B to bus A with
  V_B - V_A = I_t R_tie, its loss I_t^2 R_tie, and margins on its current rating and partial discharge;
* power balances per bus (normalized as in the lumped point), with the heat-exchanger fan load shared equally:
  bus A: V_A I_A + G_A P_gen,A + V_A I_t = M_A P_motor,A + P_fan / 2,
  bus B: V_B I_B + G_B P_gen,B          = M_B P_motor,B + P_fan / 2 + V_B I_t.

Each bus's string group has its own SOC and RC history (`condition.soc` and `voltage_rc_start_V` may be dicts keyed
"A" and "B"; a plain value applies to both), carried by the mission chain. Thermal: bus B's machines and pack keep
the instance names ("motor", "generator", "battery"); bus A's are "motor@A", "generator@A", "battery@A", with the same
thermal models, so each group has its own temperature history and margins.

Scope: hover, two buses, battery strings divisible by the bus count, the equivalent-circuit pack, no electrical layer,
no other failure combined (lanes, strings and buses all active). Margins are written here explicitly (the topology's
`operating_margins` assumes one state per instance). Components remain equation-only; this is orchestration, like
`build_flight_point`, which calls it.
"""
from dataclasses import dataclass
from typing import Any

from aircraft_closure.core.margins import margin_above, margin_below
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.redundancy import battery_with_strings, redundancy_counts
from aircraft_closure.thermal.heat import HeatLoad, evaluate_point_thermal

buses = ("A", "B")


@dataclass(frozen=True)
class BusState:
    """One bus at the point (expressions); powers are totals over the bus's active units."""
    label: str
    voltage_V: Any
    battery: Any                    # EquivalentCircuitBatteryResult of the bus's string group
    current_battery_A: Any
    count_motors: int
    torque_motor_Nm: Any            # per lane motor on this bus
    motor: Any                      # MotorResult per lane motor
    count_generators: int
    generator: Any                  # GeneratorResult per generator (None without generators)
    engine: Any                     # TurboshaftResult per engine (None without generators)
    power_battery_bus_W: Any
    power_generators_bus_W: Any
    power_motors_W: Any


@dataclass(frozen=True)
class BusSplitResult:
    bus_tie: str
    buses: dict                     # "A" / "B" -> BusState
    current_tie_A: Any
    power_loss_tie_W: Any
    share_torque_A: Any             # bus A lanes' share of each rotor's gearbox input torque (0.5: equal)
    soc_next: dict                  # "A" / "B" -> SOC at the end of the interval
    voltage_rc_end_V: dict          # "A" / "B" -> RC branch voltages at the end of the interval
    voltage_end_V: dict             # "A" / "B" -> terminal voltage at the end of the interval


def _per_bus(value, label):
    return value[label] if isinstance(value, dict) else value


def check_split_supported(topology, condition, battery_model, has_layer):
    """Raise for configurations outside the module's scope (see the module docstring)."""
    counts = redundancy_counts(topology)
    if condition.mode != "hover":
        raise ValueError("The per-bus engine-out point is a hover.")
    if counts.count_buses != 2:
        raise ValueError(f"The per-bus engine-out point needs two buses, the topology has {counts.count_buses}.")
    if counts.count_strings_battery % counts.count_buses or counts.count_lanes % counts.count_buses:
        raise ValueError("Battery strings and lanes must divide equally between the buses.")
    if not isinstance(battery_model, EquivalentCircuitBattery):
        raise ValueError("The per-bus engine-out point needs the equivalent-circuit battery.")
    if has_layer:
        raise ValueError("The per-bus engine-out point does not support the electrical layer yet.")
    if condition.active_lane_count or condition.active_battery_string_count or condition.count_buses_failed:
        raise ValueError("The per-bus engine-out point does not combine with lane, string or bus failures.")
    if condition.bus_tie not in ("open", "closed"):
        raise ValueError(f"Unknown bus_tie '{condition.bus_tie}' (use 'open' or 'closed').")


def build_split_bus(opti, aircraft, condition, *, heat_loads_shared, atmosphere, active_rotor_count, active_generator_count,
                    torque_gearbox_input_Nm, speed_motor_rad_s, duration_s, voltage_rc_start_V, temperature_start_C,
                    power_scale_W, hybridization_electric_min=0.0):
    """Add the per-bus electrical state of a hover point to `opti`; returns (BusSplitResult, heat loads, thermal,
    margins without the condition label, fuel flow, total battery terminal power, total electrical demand).

    `heat_loads_shared`: loads that do not depend on the bus (the rotor gearboxes), passed to the thermal model.
    `hybridization_electric_min` bounds each bus's battery share of its own demand from below, as the lumped point's
    share (0: the generators do not recharge the strings)."""
    topology = aircraft.powertrain.topology
    instances = topology.instances
    counts = redundancy_counts(topology)
    motor_model = instances["motor"].component
    generator_model = instances["generator"].component
    battery_model = instances["battery"].component
    turboshaft_model = instances["turboshaft"].component
    contactor = instances["protection_string"].component
    count_generators_installed = instances["generator"].count
    generators_per_bus = count_generators_installed // counts.count_buses
    count_failed = count_generators_installed - active_generator_count
    if count_failed < 1 or count_failed > generators_per_bus:
        raise ValueError(f"The per-bus engine-out point needs 1..{generators_per_bus} failed generators on one bus, "
                         f"got {count_failed}.")
    count_generators = dict(A=generators_per_bus - count_failed, B=generators_per_bus)
    lanes_per_bus = counts.count_lanes // counts.count_buses
    strings_per_bus = counts.count_strings_battery // counts.count_buses
    pack = battery_with_strings(battery_model, strings_per_bus / counts.count_strings_battery)

    # Lane torque split: bus A's lanes carry a variable share of each rotor's gearbox input, bus B's the rest.
    share_A = opti.variable(init_guess=0.5, lower_bound=0.0, upper_bound=1.0)
    torque_lane_Nm = dict(A=share_A * torque_gearbox_input_Nm / lanes_per_bus,
                          B=(1 - share_A) * torque_gearbox_input_Nm / lanes_per_bus)

    speed_generator_rad_s = generator_model.loss_model.speed_peak_efficiency_rad_s
    gearbox_generator = instances["generator_gearbox"].component if "generator_gearbox" in instances else None
    states, margins, heat_loads = {}, [], list(heat_loads_shared)
    fuel_flow_kg_s, power_battery_W, power_motors_W = 0.0, 0.0, 0.0
    for b in buses:
        current_A = opti.variable(init_guess=0.0, scale=100.0)
        battery = pack.evaluate(current_A, _per_bus(condition.soc, b), duration_s,
                                None if voltage_rc_start_V is None else _per_bus(voltage_rc_start_V, b))
        # Low-current root of the pack's power equation (as the lumped point).
        opti.subject_to(battery.voltage_V / battery.voltage_driving_V >= 0.5)
        current_string_A = current_A / strings_per_bus
        voltage_bus_V = battery.voltage_V - current_A * contactor.resistance_ohm() / strings_per_bus
        count_motors = active_rotor_count * lanes_per_bus
        motor = motor_model.evaluate(speed_motor_rad_s, torque_lane_Nm[b], voltage_bus_V)
        generator = engine = None
        power_generators_bus_W = 0.0
        if count_generators[b]:
            torque_generator_Nm = opti.variable(init_guess=500.0, scale=500.0, lower_bound=0.0)
            generator = generator_model.evaluate(speed_generator_rad_s, torque_generator_Nm, voltage_bus_V)
            speed_engine_rad_s, torque_engine_Nm = speed_generator_rad_s, torque_generator_Nm
            if gearbox_generator is not None:
                speed_engine_rad_s = speed_generator_rad_s * gearbox_generator.reduction_ratio
                torque_engine_Nm = torque_generator_Nm / (gearbox_generator.reduction_ratio * gearbox_generator.efficiency)
                gear_generator = gearbox_generator.evaluate(speed_engine_rad_s, torque_engine_Nm)
                margins.append(margin_below(f"generator_gearbox {b} power_input_W", gear_generator.power_input_W,
                                            gearbox_generator.power_rated_W))
                heat_loads.append(HeatLoad("generator_gearbox" + ("" if b == "B" else "@A"),
                                           gear_generator.power_loss_W, count_generators[b]))
            engine = turboshaft_model.evaluate(speed_engine_rad_s * torque_engine_Nm, atmosphere)
            fuel_flow_kg_s = fuel_flow_kg_s + count_generators[b] * engine.fuel_flow_kg_s
            power_generators_bus_W = count_generators[b] * generator.power_electric_W
            margins += _machine_margins(f"generator {b}", generator_model, speed_generator_rad_s, torque_generator_Nm,
                                        voltage_bus_V)
            margins.append(margin_below(f"turboshaft {b} power_shaft_W", speed_engine_rad_s * torque_engine_Nm,
                                        turboshaft_model.power_available_W(atmosphere)))
            heat_loads.append(HeatLoad("generator" + ("" if b == "B" else "@A"), generator.power_loss_W,
                                       count_generators[b]))
        margins += _machine_margins(f"motor {b}", motor_model, speed_motor_rad_s, torque_lane_Nm[b], voltage_bus_V)
        limits = pack.get_limits()
        margins += [
            margin_below(f"battery {b} discharge_current_A", current_A, limits.max_discharge_current_A),
            margin_below(f"battery {b} charge_current_A", -current_A, limits.max_charge_current_A),
            margin_above(f"battery {b} min_voltage_V", battery.voltage_V, limits.min_voltage_V),
            margin_below(f"battery {b} max_voltage_V", battery.voltage_V, limits.max_voltage_V),
            margin_below(f"protection_string {b} current_A (squared)", current_string_A**2, contactor.max_current_A**2),
        ]
        heat_loads += [HeatLoad("motor" + ("" if b == "B" else "@A"), motor.power_loss_W, count_motors),
                       HeatLoad("battery" + ("" if b == "B" else "@A"), battery.power_loss_W, 1),
                       HeatLoad("protection_string" + ("" if b == "B" else "@A"),
                                contactor.evaluate(current_string_A).power_loss_W, strings_per_bus)]
        power_battery_bus_W = voltage_bus_V * current_A
        power_motors_bus_W = count_motors * motor.power_electric_W
        power_battery_W = power_battery_W + battery.power_electric_W
        power_motors_W = power_motors_W + power_motors_bus_W
        states[b] = BusState(b, voltage_bus_V, battery, current_A, count_motors, torque_lane_Nm[b], motor,
                             count_generators[b], generator, engine, power_battery_bus_W, power_generators_bus_W,
                             power_motors_bus_W)

    # The tie: open (no current) or closed (B -> A through the tie contactor and cable).
    current_tie_A, power_loss_tie_W = 0.0, 0.0
    if condition.bus_tie == "closed":
        tie_contactor, tie_cable = instances["protection_bus_tie"].component, instances["cable_bus_tie"].component
        resistance_tie_ohm = tie_contactor.resistance_ohm() + tie_cable.resistance_ohm()
        current_tie_A = opti.variable(init_guess=0.0, scale=100.0)
        opti.subject_to((states["B"].voltage_V - states["A"].voltage_V - current_tie_A * resistance_tie_ohm)
                        / states["B"].voltage_V == 0)
        power_loss_tie_W = current_tie_A**2 * resistance_tie_ohm
        margins += [
            margin_below("protection_bus_tie current_A (squared)", current_tie_A**2, tie_contactor.max_current_A**2),
            margin_below("cable_bus_tie current_A (squared)", current_tie_A**2, tie_cable.max_current_A**2),
            margin_below("cable_bus_tie partial_discharge_V",
                         tie_cable.partial_discharge.factor_safety * states["B"].voltage_V,
                         tie_cable.voltage_inception_V(atmosphere.pressure() / 101325.0)),
        ]
        heat_loads += [HeatLoad("protection_bus_tie", tie_contactor.evaluate(current_tie_A).power_loss_W, 1),
                       HeatLoad("cable_bus_tie", tie_cable.evaluate(current_tie_A).power_loss_W, 1)]

    # Thermal: bus B's groups keep the instance names, bus A's carry "@A" (same components, own histories).
    components = {"battery": pack, "battery@A": pack, "motor@A": motor_model, "generator@A": generator_model}
    start_C = None
    if temperature_start_C is not None:
        start_C = dict(temperature_start_C)
        for name in ("motor", "generator", "battery"):
            if name in start_C:
                start_C.setdefault(name + "@A", start_C[name])
    thermal = evaluate_point_thermal(aircraft.powertrain, heat_loads, atmosphere, 0.0, "hover", duration_s, start_C,
                                     condition.label, components=components)
    power_fan_W = thermal.cooling.power_fan_W if thermal.cooling is not None else 0.0

    # Per-bus power balances, normalized as the lumped point's.
    opti.subject_to([
        (states["A"].power_battery_bus_W + states["A"].power_generators_bus_W + states["A"].voltage_V * current_tie_A
         - states["A"].power_motors_W - power_fan_W / 2) / power_scale_W == 0,
        (states["B"].power_battery_bus_W + states["B"].power_generators_bus_W
         - states["B"].power_motors_W - power_fan_W / 2 - states["B"].voltage_V * current_tie_A) / power_scale_W == 0,
    ])
    # Battery share of each bus's own demand (motors, half the fan, the tie) at or above the minimum.
    demand_W = dict(A=states["A"].power_motors_W + power_fan_W / 2,
                    B=states["B"].power_motors_W + power_fan_W / 2 + states["B"].voltage_V * current_tie_A)
    opti.subject_to([(states[b].power_battery_bus_W - hybridization_electric_min * demand_W[b]) / power_scale_W >= 0
                     for b in buses])
    result = BusSplitResult(
        bus_tie=condition.bus_tie, buses=states, current_tie_A=current_tie_A, power_loss_tie_W=power_loss_tie_W,
        share_torque_A=share_A, soc_next={b: states[b].battery.soc_next for b in buses},
        voltage_rc_end_V={b: states[b].battery.voltage_rc_end_V for b in buses},
        voltage_end_V={b: states[b].battery.voltage_end_V for b in buses})
    return result, tuple(heat_loads), thermal, tuple(margins), fuel_flow_kg_s, power_battery_W, power_motors_W + power_fan_W


def _machine_margins(name, machine, speed_rad_s, torque_Nm, voltage_V):
    """Operating margins of one machine group (as `operating_margins`, with the group name in the label)."""
    margins = []
    if machine.thermal_model is None:
        margins.append(margin_below(f"{name} power_shaft_W", speed_rad_s * torque_Nm, machine.power_rated_W))
    return margins + [
        margin_below(f"{name} torque_Nm", torque_Nm, machine.max_torque_Nm),
        margin_below(f"{name} speed_rad_s", speed_rad_s, machine.max_speed_rad_s),
        margin_above(f"{name} min_voltage_V", voltage_V, machine.min_voltage_V),
        margin_below(f"{name} max_voltage_V", voltage_V, machine.max_voltage_V),
    ]
