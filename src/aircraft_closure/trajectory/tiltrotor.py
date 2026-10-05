"""Tiltrotor point-mass model and direct-collocation builder for trajectory optimization (Tier 14, plan 019).

Trajectory optimization flies an already-sized aircraft: every design quantity is a number, the
trajectory Opti owns only states, controls and the final time. AeroSandbox supplies the dynamics
(`asb.DynamicsPointMass2DSpeedGamma`, wind axes), the collocation (`Opti.constrain_derivative`,
trapezoidal) and the atmosphere; this module adds what AeroSandbox lacks: the tilting rotor thrust,
the rotor and powertrain power chain and the energy states.

Geometry (longitudinal plane, all angles positive nose-up / rotor-axis-up):
    alpha        wing (= fuselage, zero incidence) angle of attack to the flight path
    tilt         nacelle angle from the fuselage x-axis: 90 deg hover (helicopter), 0 deg airplane
    phi          = alpha + tilt, rotor-axis angle to the velocity vector
    pitch        = gamma + alpha, fuselage attitude
Wind-axis forces (x along velocity, z down-normal, ASB convention), gravity added by AeroSandbox:
    F_x = n T (1 - f_dl) cos(phi) - D,    F_z = -(n T (1 - f_dl) sin(phi) + L)

Approximations (plan 019):
* Rotor power uses the axial-flow `MomentumProfileRotor` with the velocity component along the rotor
  axis, V cos(phi), as the axial velocity; the in-plane (edgewise) component is ignored, so there is
  no translational-lift benefit or edgewise profile-power rise in conversion. The airplane-mode
  profile-drag increment and induced-power factor are phased in with cos^2(tilt) (hover values at
  90 deg, airplane values at 0 deg).
  Negative axial inflow (descent through the disk, vortex-ring state) is outside momentum theory; the
  builder constrains V cos(phi) >= 0.
* Download: the hover fraction of the aerodynamics model, faded with sin^2(tilt) and with the wake
  skew factor v_h^2 / (v_h^2 + V^2), v_h the hover induced velocity; exactly the hover value at
  V = 0, tilt = 90 deg (the quasi-steady hover flight point).
* Wing lift and drag: the aerodynamics model's coefficients evaluated at max(V, a floor) for the
  Reynolds and Mach numbers only; the dynamic pressure uses the true speed, so forces vanish at V = 0.
  Attached flow only: alpha is bounded by the model's stall angle at every node.
* Battery (plan 022): the motors and generators see the battery terminal voltage. With the
  equivalent-circuit battery, SOC is coulomb-counted and the limits are current, voltage and the
  low-current root. The RC polarization is at its steady value at each node; there are no RC states.
  That is conservative for manoeuvres shorter than the 43 s time constant.
"""
from dataclasses import dataclass, replace
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.performance.flight_point import acceleration_gravity_m_s2
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor
from aircraft_closure.thermal.heat import thermal_parameters


@dataclass(frozen=True)
class ConversionCorridor:
    """Airspeed band against nacelle tilt, after the XV-15 corridor (NASA TM X-62407 fig. 5.4.1).

    Low-speed boundary (wing stall / attitude limit): V >= V_stall cos(tilt); the XV-15 13,000 lb
    boundary is within about 10 kt of this with V_stall = 100 kt at 0, 30 and 60 deg.
    High-speed boundary (rotor torque and loads): V <= V_90 + (V_0 - V_90) (1 - (tilt / 90 deg)^2),
    a fit to the XV-15 boundary (115 kt at 90 deg, about 150, 173 and 180 kt at 60, 30 and 0 deg).
    The high-speed boundary is taken in absolute terms (a rotor of similar tip speed and size);
    the low-speed boundary scales with the aircraft's own stall speed.
    """
    velocity_stall_m_s: Any
    velocity_max_helicopter_m_s: Any = 115 * u.knot
    velocity_max_airplane_m_s: Any = 180 * u.knot

    def velocity_min_m_s(self, tilt_deg):
        return self.velocity_stall_m_s * np.cosd(tilt_deg)

    def velocity_max_m_s(self, tilt_deg):
        return self.velocity_max_helicopter_m_s + (self.velocity_max_airplane_m_s - self.velocity_max_helicopter_m_s) \
            * (1 - (tilt_deg / 90) ** 2)


@dataclass(frozen=True)
class TiltrotorForces:
    """Per-node force and power expressions (arrays over the nodes)."""
    thrust_per_rotor_N: Any
    angle_thrust_velocity_deg: Any
    velocity_axial_m_s: Any
    download_fraction: Any
    lift_N: Any
    drag_N: Any
    force_x_wind_N: Any
    force_z_wind_N: Any
    cl: Any
    cd: Any
    alpha_stall_deg: Any
    rotor: Any
    speed_motor_rad_s: Any
    torque_motor_Nm: Any
    power_shaft_motor_W: Any
    power_electric_motors_W: Any


@dataclass(frozen=True)
class PowerSupply:
    """Bus supply at each node: battery plus the active turbogenerators."""
    battery: Any
    generator: Any
    engine: Any
    power_shaft_generator_W: Any
    power_available_turboshaft_W: Any
    power_bus_W: Any
    fuel_flow_kg_s: Any


@dataclass(frozen=True)
class TiltrotorPointMass:
    """Equation-only force and power model of a sized series-hybrid tiltrotor.

    `aircraft` is a numeric `Aircraft` whose powertrain is the series-hybrid reference topology with a
    `MomentumProfileRotor` propulsor; `aerodynamics` is a `SimpleAerodynamics`-like model.
    """
    aircraft: Any
    aerodynamics: Any
    velocity_coefficient_min_m_s: float = 10.0       # floor for Reynolds/Mach in the force coefficients
    velocity_axial_smoothing_m_s: float = 1e-4       # keeps the profile integral finite at zero inflow

    def __post_init__(self):
        if not isinstance(self.instance("propulsor"), MomentumProfileRotor):
            raise TypeError("TiltrotorPointMass needs a MomentumProfileRotor propulsor (rotor speed physics).")

    def instance(self, name):
        return self.aircraft.powertrain.topology.instances[name].component

    def count(self, name):
        return self.aircraft.powertrain.topology.instances[name].count

    def velocity_stall_m_s(self, mass_kg, altitude_m=0.0):
        """Airplane-mode (wing-borne) stall speed at the aerodynamics model's CLmax."""
        density_kg_m3 = asb.Atmosphere(altitude=altitude_m).density()
        return np.sqrt(2 * mass_kg * acceleration_gravity_m_s2
                       / (density_kg_m3 * self.aircraft.wing.area_m2 * self.aerodynamics.cl_max))

    def evaluate(self, velocity_m_s, altitude_m, alpha_deg, tilt_deg, *, thrust_per_rotor_N, speed_rotor_rad_s,
                 voltage_bus_V=None):
        rotor_base = self.instance("propulsor")
        gearbox = self.instance("gearbox")
        motor = self.instance("motor")
        count_rotors = self.count("propulsor")
        atmosphere = asb.Atmosphere(altitude=altitude_m)
        density_kg_m3 = atmosphere.density()

        # Wing: coefficients at a floored speed (Reynolds, Mach), dynamic pressure at the true speed.
        velocity_coefficient_m_s = np.fmax(velocity_m_s, self.velocity_coefficient_min_m_s)
        aero = self.aerodynamics.evaluate(self.aircraft, velocity_coefficient_m_s, altitude_m, alpha_deg)
        force_scale_N = 0.5 * density_kg_m3 * velocity_m_s ** 2 * self.aircraft.wing.area_m2
        lift_N = force_scale_N * aero.cl
        drag_N = force_scale_N * aero.cd
        alpha_stall_deg = self.aerodynamics.alpha_stall_deg(self.aircraft, velocity_coefficient_m_s, altitude_m,
                                                            aero=aero)

        # Rotor: axial-flow model on the velocity component along the rotor axis.
        angle_thrust_velocity_deg = alpha_deg + tilt_deg
        velocity_axial_m_s = velocity_m_s * np.cosd(angle_thrust_velocity_deg)
        # Hover-to-airplane blend weight cos^2(tilt), with tilt clipped at 0 deg (nacelles past airplane mode
        # count as airplane mode) so tilt <= 0 recovers the airplane-mode rotor exactly.
        tilt_clipped_deg = np.fmax(tilt_deg, 0.0)
        weight_airplane = np.cosd(tilt_clipped_deg) ** 2
        increment_a, increment_b = rotor_base.drag_increment_airplane
        rotor_model = replace(rotor_base, airplane_mode=True,
                              kappa_airplane=rotor_base.kappa_hover + (rotor_base.kappa_airplane - rotor_base.kappa_hover)
                              * weight_airplane,
                              drag_increment_airplane=(increment_a * weight_airplane, increment_b * weight_airplane))
        rotor = rotor_model.evaluate(np.sqrt(velocity_axial_m_s ** 2 + self.velocity_axial_smoothing_m_s ** 2),
                                     atmosphere, thrust_N=thrust_per_rotor_N, speed_rad_s=speed_rotor_rad_s)

        # Download on the wing under the rotor wake, faded with tilt and wake skew.
        velocity_induced_hover_sq_m2_s2 = thrust_per_rotor_N / (2 * density_kg_m3 * rotor_base.area_disk_m2)
        download_fraction = (self.aerodynamics.hover_download_fraction(self.aircraft) * np.sind(tilt_clipped_deg) ** 2
                             * velocity_induced_hover_sq_m2_s2 / (velocity_induced_hover_sq_m2_s2 + velocity_m_s ** 2))
        thrust_net_N = count_rotors * thrust_per_rotor_N * (1 - download_fraction)
        force_x_wind_N = thrust_net_N * np.cosd(angle_thrust_velocity_deg) - drag_N
        force_z_wind_N = -(thrust_net_N * np.sind(angle_thrust_velocity_deg) + lift_N)

        # Drive chain: gearbox loss on torque (as the flight point), motor losses from its own model.
        speed_motor_rad_s = gearbox.reduction_ratio * speed_rotor_rad_s
        # Tier 18: a rotor's lane motors share its gearbox input torque equally.
        lanes = self.count("motor") / count_rotors
        torque_motor_Nm = rotor.shaft_power_W / (gearbox.efficiency * speed_motor_rad_s) / lanes
        if voltage_bus_V is None:
            battery = self.instance("battery")
            voltage_bus_V = (battery.voltage_open_circuit_V(battery.max_soc)
                             if isinstance(battery, EquivalentCircuitBattery) else battery.voltage_open_circuit_V)
        motor_result = motor.evaluate(speed_motor_rad_s, torque_motor_Nm, voltage_bus_V)
        return TiltrotorForces(
            thrust_per_rotor_N=thrust_per_rotor_N, angle_thrust_velocity_deg=angle_thrust_velocity_deg,
            velocity_axial_m_s=velocity_axial_m_s, download_fraction=download_fraction, lift_N=lift_N, drag_N=drag_N,
            force_x_wind_N=force_x_wind_N, force_z_wind_N=force_z_wind_N, cl=aero.cl, cd=aero.cd,
            alpha_stall_deg=alpha_stall_deg, rotor=rotor, speed_motor_rad_s=speed_motor_rad_s,
            torque_motor_Nm=torque_motor_Nm, power_shaft_motor_W=motor_result.power_shaft_W,
            power_electric_motors_W=self.count("motor") * motor_result.power_electric_W)

    def evaluate_supply(self, altitude_m, *, current_battery_A, torque_generator_Nm, soc):
        """Battery and turbogenerators (all active, equal shares) feeding the bus."""
        battery_model = self.instance("battery")
        generator_model = self.instance("generator")
        turboshaft = self.instance("turboshaft")
        count_generators = self.count("generator")
        atmosphere = asb.Atmosphere(altitude=altitude_m)
        battery = battery_model.evaluate(current_battery_A, soc)
        speed_generator_rad_s = generator_model.loss_model.speed_peak_efficiency_rad_s
        generator = generator_model.evaluate(speed_generator_rad_s, torque_generator_Nm, battery.voltage_V)
        power_shaft_generator_W = speed_generator_rad_s * torque_generator_Nm
        instances = self.aircraft.powertrain.topology.instances
        # Tier 13: a step-up gearbox between turboshaft and generator takes its loss from the engine side.
        efficiency_step_up = instances["generator_gearbox"].component.efficiency if "generator_gearbox" in instances else 1.0
        engine = turboshaft.evaluate(power_shaft_generator_W / efficiency_step_up, atmosphere)
        return PowerSupply(battery=battery, generator=generator, engine=engine,
                           power_shaft_generator_W=power_shaft_generator_W / efficiency_step_up,
                           power_available_turboshaft_W=turboshaft.power_available_W(atmosphere),
                           power_bus_W=count_generators * generator.power_electric_W + battery.power_electric_W,
                           fuel_flow_kg_s=count_generators * engine.fuel_flow_kg_s)


@dataclass(frozen=True)
class TrajectoryLimits:
    """Operating bounds of the trajectory; hardware limits come from the components themselves."""
    tilt_rate_max_deg_s: float = 8.0       # XV-15-like nacelle conversion rate
    tilt_min_deg: float = 0.0
    tilt_max_deg: float = 95.0             # XV-15 conversion range (TM X-62407 sec. 8.1.4)
    alpha_min_deg: float = -10.0
    pitch_min_deg: float = -10.0
    pitch_max_deg: float = 20.0
    gamma_max_deg: float = 30.0
    velocity_min_m_s: float = 0.5          # speed-gamma states are singular at zero speed
    velocity_max_m_s: Any = None
    soc_min: float = 0.3
    # Battery assists only (no in-flight recharge) unless allowed: with recharging, a minimum-energy or
    # minimum-time objective would burn extra fuel to fly lighter, a spurious split of no interest here.
    allow_battery_charging: bool = False


@dataclass(frozen=True)
class TrajectoryGuess:
    """Initial guesses; each node quantity may be a scalar or an array of `count_nodes` values."""
    duration_s: float
    velocity_m_s: Any
    altitude_m: Any
    tilt_deg: Any
    alpha_deg: Any = 4.0
    gamma_deg: Any = 0.0
    thrust_per_rotor_N: Any = 3.0e4
    speed_rotor_rad_s: Any = 45.0
    current_battery_A: Any = 0.0
    torque_generator_Nm: Any = 1000.0


@dataclass(frozen=True)
class TiltrotorTrajectory:
    time_s: Any
    duration_s: Any
    dynamics: Any
    x_m: Any
    altitude_m: Any
    velocity_m_s: Any
    gamma_rad: Any
    mass_kg: Any
    soc: Any
    energy_bus_J: Any
    alpha_deg: Any
    tilt_deg: Any
    tilt_rate_deg_s: Any
    pitch_deg: Any
    thrust_per_rotor_N: Any
    speed_rotor_rad_s: Any
    current_battery_A: Any
    torque_generator_Nm: Any
    forces: Any
    supply: Any
    corridor: Any
    acceleration_m_s2: Any
    rate_gamma_rad_s: Any
    thermal: Any = None


@dataclass(frozen=True)
class TrajectoryThermal:
    """Temperature states and cooling along a trajectory (zeros and no states without thermal models)."""
    temperatures_C: dict                 # instance name -> node temperatures
    power_heat_W: Any                    # heat into the exchanger at each node
    drag_cooling_N: Any
    power_fan_W: Any
    constraints: tuple


def _thermal_states(opti, model, forces, supply, altitude_m, velocity_m_s, time_s, count_nodes, temperature_start_C):
    """Tier 19 along a trajectory (plan 036): a lumped temperature state per thermal-modelled machine and the pack,
    dT/dt = (loss - (T - T_coolant) / R) / C by collocation, the same R and C as sizing (`thermal_parameters`); the
    temperature limit at every node. Heat to the coolant goes to the ram-air exchanger: its pumping power is paid as
    cooling drag in proportion w = q / (q + dp) and by the fans for the rest, with q the dynamic pressure and dp the
    exchanger pressure drop (a smooth blend of the sizing's airplane-mode drag and hover fan cases).
    `temperature_start_C`: dict by instance name; None starts at the coolant temperature (a cold start)."""
    instances = model.aircraft.powertrain.topology.instances
    losses = {
        "motor": forces.power_electric_motors_W / model.count("motor") - forces.power_shaft_motor_W,
        "generator": supply.generator.power_shaft_W - supply.generator.power_electric_W,
        "battery": supply.battery.power_loss_W,
    }
    temperatures, constraints, heat_W = {}, [], 0.0
    for name, loss_W in losses.items():
        component = instances[name].component
        thermal = getattr(component, "thermal_model", None)
        if thermal is None:
            continue
        p = thermal_parameters(component)
        start_C = thermal.temperature_coolant_C if temperature_start_C is None else temperature_start_C[name]
        temperature_C = opti.variable(init_guess=start_C * np.ones(count_nodes), scale=50.0)
        opti.constrain_derivative((loss_W - (temperature_C - thermal.temperature_coolant_C) / p.resistance_K_W)
                                  / p.capacity_J_K, temperature_C, time_s)
        constraints += [temperature_C[0] == start_C, temperature_C <= thermal.temperature_max_C]
        temperatures[name] = temperature_C
        cooling = getattr(model.aircraft.powertrain, "cooling", None)
        if cooling is not None and name not in cooling.sources_excluded:
            heat_W = heat_W + instances[name].count * (temperature_C - thermal.temperature_coolant_C) / p.resistance_K_W
    cooling = getattr(model.aircraft.powertrain, "cooling", None)
    zero = 0 * velocity_m_s
    if cooling is None or not temperatures:
        return TrajectoryThermal(temperatures, zero, zero, zero, tuple(constraints))
    exchanger = cooling.heat_exchanger
    atmosphere = asb.Atmosphere(altitude=altitude_m)
    result = exchanger.evaluate(heat_W, atmosphere, velocity_m_s, fan=True)
    pressure_dynamic_Pa = 0.5 * atmosphere.density() * velocity_m_s**2
    ram = pressure_dynamic_Pa / (pressure_dynamic_Pa + result.pressure_drop_Pa + 1.0)
    drag_N = ram * result.power_pumping_W / np.fmax(velocity_m_s, 1.0)
    power_fan_W = (1 - ram) * result.power_pumping_W / exchanger.efficiency_fan
    constraints.append(result.power_heat_equivalent_W / exchanger.power_rated_W <= 1)
    return TrajectoryThermal(temperatures, heat_W, drag_N, power_fan_W, tuple(constraints))


def _node_guess(value, count_nodes):
    return value * np.ones(count_nodes) if np.isscalar(value) else np.array(value, dtype=float)


def build_tiltrotor_trajectory(opti, model, *, mass_initial_kg, soc_initial, count_nodes, duration_bounds_s, guess,
                               limits=TrajectoryLimits(), corridor=None, temperature_start_C=None):
    """Direct-collocation tiltrotor trajectory on a fixed aircraft (orchestration: creates Opti variables).

    States per node: x, altitude, speed, flight-path angle (AeroSandbox point mass), mass (fuel burn),
    battery SOC and bus energy. Controls per node: alpha, nacelle tilt (with its rate bound), thrust
    and speed per rotor, battery current and generator torque. The final time is a variable within
    `duration_bounds_s`. Boundary conditions and the objective belong to the caller.
    """
    n = count_nodes
    g = lambda value: _node_guess(value, n)  # noqa: E731
    duration_s = opti.variable(init_guess=guess.duration_s, scale=guess.duration_s,
                               lower_bound=duration_bounds_s[0], upper_bound=duration_bounds_s[1])
    time_s = np.linspace(0, duration_s, n)
    velocity_guess_m_s = g(guess.velocity_m_s)
    x_guess_m = np.concatenate([[0.0], np.cumsum(0.5 * (velocity_guess_m_s[1:] + velocity_guess_m_s[:-1])
                                                 * guess.duration_s / (n - 1))])
    count_rotors = model.count("propulsor")
    rotor = model.instance("propulsor")
    motor = model.instance("motor")
    generator_model = model.instance("generator")
    battery_model = model.instance("battery")
    scale_power_W = count_rotors * motor.power_rated_W

    x_m = opti.variable(init_guess=x_guess_m, scale=max(np.max(np.abs(x_guess_m)), 100.0))
    altitude_m = opti.variable(init_guess=g(guess.altitude_m), scale=max(np.max(np.abs(g(guess.altitude_m))), 100.0))
    velocity_m_s = opti.variable(init_guess=velocity_guess_m_s, scale=50.0, lower_bound=limits.velocity_min_m_s,
                                 upper_bound=limits.velocity_max_m_s)
    gamma_rad = opti.variable(init_guess=np.radians(g(guess.gamma_deg)), scale=0.2,
                              lower_bound=-np.radians(limits.gamma_max_deg), upper_bound=np.radians(limits.gamma_max_deg))
    mass_kg = opti.variable(init_guess=mass_initial_kg * np.ones(n), scale=mass_initial_kg)
    soc = opti.variable(init_guess=soc_initial * np.ones(n), scale=0.1, lower_bound=limits.soc_min,
                        upper_bound=battery_model.max_soc)
    energy_bus_J = opti.variable(init_guess=np.linspace(0, 1.5e6 * guess.duration_s, n), scale=1e6 * guess.duration_s)
    alpha_deg = opti.variable(init_guess=g(guess.alpha_deg), scale=5.0, lower_bound=limits.alpha_min_deg)
    tilt_deg = opti.variable(init_guess=g(guess.tilt_deg), scale=30.0, lower_bound=limits.tilt_min_deg,
                             upper_bound=limits.tilt_max_deg)
    thrust_per_rotor_N = opti.variable(init_guess=g(guess.thrust_per_rotor_N), scale=1e4, lower_bound=0.0)
    speed_rotor_rad_s = opti.variable(init_guess=g(guess.speed_rotor_rad_s), scale=50.0, lower_bound=10.0)
    current_battery_A = opti.variable(init_guess=g(guess.current_battery_A), scale=300.0)
    torque_generator_Nm = opti.variable(init_guess=g(guess.torque_generator_Nm), scale=1000.0, lower_bound=0.0)
    # Nacelle rate per interval (n - 1 values): an exact bound on the piecewise-linear tilt history. (A
    # trapezoidal `derivative_of` variable only bounds the mean of adjacent node rates, which lets them
    # alternate.)
    tilt_rate_deg_s = np.diff(tilt_deg) / np.diff(time_s)

    supply = model.evaluate_supply(altitude_m, current_battery_A=current_battery_A,
                                   torque_generator_Nm=torque_generator_Nm, soc=soc)
    # The motors see the battery terminal (bus) voltage.
    forces = model.evaluate(velocity_m_s, altitude_m, alpha_deg, tilt_deg, thrust_per_rotor_N=thrust_per_rotor_N,
                            speed_rotor_rad_s=speed_rotor_rad_s, voltage_bus_V=supply.battery.voltage_V)

    thermal = _thermal_states(opti, model, forces, supply, altitude_m, velocity_m_s, time_s, n, temperature_start_C)

    dynamics = asb.DynamicsPointMass2DSpeedGamma(mass_props=asb.MassProperties(mass=mass_kg), x_e=x_m, z_e=-altitude_m,
                                                 speed=velocity_m_s, gamma=gamma_rad, alpha=alpha_deg)
    dynamics.add_force(Fx=forces.force_x_wind_N - thermal.drag_cooling_N, Fz=forces.force_z_wind_N, axes="wind")
    dynamics.add_gravity_force(g=acceleration_gravity_m_s2)
    dynamics.constrain_derivatives(opti, time_s)
    derivatives = dynamics.state_derivatives()
    opti.constrain_derivative(-supply.fuel_flow_kg_s, mass_kg, time_s)
    is_ecm = isinstance(battery_model, EquivalentCircuitBattery)
    if is_ecm:
        # Coulomb counting; polarization at its steady value at each node (no RC states, plan 022).
        opti.constrain_derivative(-current_battery_A / battery_model.capacity_As, soc, time_s)
        battery_limits = battery_model.get_limits()
        opti.subject_to([
            current_battery_A <= battery_limits.max_discharge_current_A,
            current_battery_A >= -battery_limits.max_charge_current_A,
            supply.battery.voltage_V >= battery_limits.min_voltage_V,
            supply.battery.voltage_V <= battery_limits.max_voltage_V,
            # Low-current root of R_eff I^2 - V* I + P = 0 (as the flight point).
            supply.battery.voltage_V / supply.battery.voltage_driving_V >= 0.5,
        ])
    else:
        opti.constrain_derivative(-supply.battery.power_chemical_W / battery_model.energy_capacity_J, soc, time_s)
        opti.subject_to([
            supply.battery.power_electric_W / scale_power_W <= battery_model.max_discharge_power_W / scale_power_W,
            supply.battery.power_electric_W / scale_power_W >= -battery_model.max_charge_power_W / scale_power_W,
        ])
    power_demand_bus_W = forces.power_electric_motors_W + thermal.power_fan_W
    opti.constrain_derivative(power_demand_bus_W, energy_bus_J, time_s)

    pitch_deg = np.degrees(gamma_rad) + alpha_deg
    corridor_constraints = [] if corridor is None else [
        velocity_m_s >= corridor.velocity_min_m_s(tilt_deg), velocity_m_s <= corridor.velocity_max_m_s(tilt_deg)]
    opti.subject_to([
        # Bus balance and sources (normalized by the installed motor rating).
        (supply.power_bus_W - power_demand_bus_W) / scale_power_W == 0,
        supply.power_shaft_generator_W <= supply.power_available_turboshaft_W,      # turboshaft shaft power
        torque_generator_Nm <= generator_model.max_torque_Nm,
        # Rotor (Tier 12 validity and hardware bounds) and drive.
        forces.rotor.blade_loading <= rotor.blade_loading_max,
        forces.rotor.mach_tip_helical <= rotor.mach_tip_helical_max,
        forces.rotor.advance_ratio <= rotor.advance_ratio_max,
        forces.velocity_axial_m_s >= 0,
        forces.rotor.shaft_power_W / scale_power_W <= rotor.max_shaft_power_W / scale_power_W,
        forces.speed_motor_rad_s <= motor.max_speed_rad_s,
        forces.torque_motor_Nm <= motor.max_torque_Nm,
        # Wing and attitude.
        alpha_deg <= forces.alpha_stall_deg,
        pitch_deg >= limits.pitch_min_deg,
        pitch_deg <= limits.pitch_max_deg,
        tilt_rate_deg_s <= limits.tilt_rate_max_deg_s,
        tilt_rate_deg_s >= -limits.tilt_rate_max_deg_s,
    ] + corridor_constraints)
    # Continuous power ratings, unless the machine has a thermal model (then its temperature limit applies, as in
    # sizing: short-time ratings from thermal mass).
    if motor.thermal_model is None:
        opti.subject_to(forces.power_shaft_motor_W / scale_power_W <= motor.power_rated_W / scale_power_W)
    if generator_model.thermal_model is None:
        opti.subject_to(generator_model.loss_model.speed_peak_efficiency_rad_s * torque_generator_Nm / scale_power_W
                        <= generator_model.power_rated_W / scale_power_W)
    opti.subject_to(list(thermal.constraints))
    if not limits.allow_battery_charging:
        opti.subject_to(supply.battery.power_electric_W / scale_power_W >= 0)
    if rotor.speed_tip_max_m_s is not None:
        opti.subject_to(speed_rotor_rad_s * rotor.radius_m() <= rotor.speed_tip_max_m_s)
    opti.subject_to([x_m[0] == 0, mass_kg[0] == mass_initial_kg, soc[0] == soc_initial, energy_bus_J[0] == 0])
    return TiltrotorTrajectory(
        time_s=time_s, duration_s=duration_s, dynamics=dynamics, x_m=x_m, altitude_m=altitude_m,
        velocity_m_s=velocity_m_s, gamma_rad=gamma_rad, mass_kg=mass_kg, soc=soc, energy_bus_J=energy_bus_J,
        alpha_deg=alpha_deg, tilt_deg=tilt_deg, tilt_rate_deg_s=tilt_rate_deg_s, pitch_deg=pitch_deg,
        thrust_per_rotor_N=thrust_per_rotor_N, speed_rotor_rad_s=speed_rotor_rad_s,
        current_battery_A=current_battery_A, torque_generator_Nm=torque_generator_Nm, forces=forces, supply=supply,
        corridor=corridor, acceleration_m_s2=derivatives["speed"], rate_gamma_rad_s=derivatives["gamma"],
        thermal=thermal)
