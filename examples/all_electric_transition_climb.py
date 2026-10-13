"""Optimized take-off, conversion and climb of the notional all-electric tiltrotor (plan 068).

The notional aircraft of plan 066 (`examples/all_electric_tiltrotor.py`: 1,600 kg, two 4.5 m proprotors, three
direct-drive motor channels per rotor, a 90 kWh pack) gets an airframe and the rotor-speed rotor model, so the
tiltrotor trajectory model (plan 019) can fly it battery only. The segment: trimmed hover at sea level ->
conversion -> climb to 1,500 m at 75 m/s in level airplane-mode flight, with the least battery energy. It is
compared with a fixed-power rule of thumb, 180 s at 1.5 x the cruise power (an illustrative generic heuristic).

Airframe (illustrative numbers, not a closed design):
- Wing 7.5 m2, aspect ratio 6 (the XV-15's 6.12, NASA SP-2000-4517), span 6.7 m: the rotor hubs near the tips
  clear the fuselage with 4.5 m rotors. Wing loading 2.1 kN/m2 (XV-15 about 3.7 kN/m2).
- Fuselage 7 m x 1.3 m, tails sized as a light aircraft (Raymer tail volume coefficients); `SimpleAerodynamics`
  with CLmax 1.5, the XV-15 hover download fraction 0.07 (NASA TM X-62407 sec. 5.1) and 0.10 m2 of miscellaneous
  drag area (assumed).

Run: `python examples/all_electric_transition_climb.py` (writes `output/all_electric_transition_climb.png`).
"""
from pathlib import Path

import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.trajectory.segments import (energy_fixed_power_rule_J, solve_steady_level_power,
                                                  solve_transition_climb)
from aircraft_closure.trajectory.tiltrotor import TiltrotorPointMass
from aircraft_closure.vehicle.aircraft import Aircraft
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation
from aircraft_closure.vehicle.surfaces import HorizontalTail, VerticalTail, Wing
from examples.all_electric_tiltrotor import NotionalAllElectricTiltrotor, build_notional_all_electric_tiltrotor

altitude_cruise_m = 1500.0
velocity_cruise_m_s = 75.0
soc_take_off = 0.95
# Rule of thumb compared against (illustrative generic heuristic, not a sourced method).
duration_rule_s = 180.0
factor_rule = 1.5


def build_notional_trajectory_model(count_channels_per_rotor=3, data=None):
    """`TiltrotorPointMass` of the notional all-electric tiltrotor: airframe, installed battery-only powertrain
    with the `MomentumProfileRotor`, and `SimpleAerodynamics`."""
    data = data or NotionalAllElectricTiltrotor()
    topology = build_notional_all_electric_tiltrotor(count_channels_per_rotor, data, rotor_model="momentum_profile")
    x_wing_m = 2.8
    # Every instance at the wing (the CG is not used by the point-mass trajectory).
    installation = PowertrainInstallation(topology, tuple(InstalledInstance(name, x_wing_m)
                                                          for name in topology.instances))
    aircraft = Aircraft(
        wing=Wing(area_m2=7.5, aspect_ratio=6.0, taper_ratio=1.0, x_le_root_m=x_wing_m, z_m=0.9),
        horizontal_tail=HorizontalTail(area_m2=1.8, aspect_ratio=4.0, x_le_root_m=6.0, z_m=0.6),
        vertical_tail=VerticalTail(area_m2=1.2, aspect_ratio=1.5, x_le_root_m=5.9, z_root_m=0.6),
        fuselage=Fuselage(length_m=7.0, diameter_m=1.3),
        landing_gear=None, systems=None, powertrain=installation, payload=None)
    # Miscellaneous drag area 0.10 m2 (hubs, nacelles, gear fairings; assumed: the framework default 0.25 m2 is for
    # a 6 t aircraft).
    aerodynamics = SimpleAerodynamics(cl_max=1.5, download_fraction_hover=0.07, drag_area_misc_m2=0.10)
    return TiltrotorPointMass(aircraft, aerodynamics)


def plot(result, power_cruise_W, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = result.time_s
    panels = [
        ("Altitude (m)", result.altitude_m),
        ("Airspeed (m/s)", result.velocity_m_s),
        ("Nacelle tilt (deg)", result.tilt_deg),
        ("Battery power (kW)", result.power_battery_W / 1e3),
        ("State of charge", result.soc),
    ]
    figure, axes = plt.subplots(len(panels), 1, figsize=(7, 11), sharex=True, facecolor="#FFFFFF")
    for axis, (label, values) in zip(axes, panels):
        axis.set_facecolor("#FFFFFF")
        axis.plot(t, values, color="#1f4e79", linewidth=1.8, marker="o", markersize=2.5)
        axis.set_ylabel(label)
        axis.grid(True, color="#dddddd", linewidth=0.6)
    axes[3].axhline(factor_rule * power_cruise_W / 1e3, color="#c0504d", linestyle="--", linewidth=1.2,
                    label=f"{factor_rule:g} x cruise power")
    axes[3].axhline(power_cruise_W / 1e3, color="#7f7f7f", linestyle=":", linewidth=1.2, label="Cruise power")
    axes[3].legend(loc="center right", frameon=False)
    axes[-1].set_xlabel("Time (s)")
    figure.suptitle("Notional all-electric tiltrotor: minimum-energy take-off, conversion and climb", fontsize=11)
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150, facecolor="#FFFFFF")
    plt.close(figure)


def main(count_nodes=41):
    data = NotionalAllElectricTiltrotor()
    model = build_notional_trajectory_model(data=data)
    mass_kg = data.mass_takeoff_kg
    power_cruise_W = solve_steady_level_power(model, mass_kg=mass_kg, soc=soc_take_off, altitude_m=altitude_cruise_m,
                                              velocity_m_s=velocity_cruise_m_s)
    result = solve_transition_climb(model, mass_kg=mass_kg, soc_initial=soc_take_off,
                                    altitude_final_m=altitude_cruise_m, velocity_final_m_s=velocity_cruise_m_s,
                                    count_nodes=count_nodes)
    energy_rule_J = energy_fixed_power_rule_J(power_cruise_W, factor_rule, duration_rule_s)
    print(f"Cruise ({altitude_cruise_m:.0f} m, {velocity_cruise_m_s:.0f} m/s) battery power "
          f"{power_cruise_W / 1e3:.1f} kW")
    print(f"Optimized take-off, conversion and climb: {result.duration_s:.0f} s, {result.distance_m / 1e3:.2f} km, "
          f"battery energy {result.energy_electric_J / 3.6e6:.2f} kWh (chemical {result.energy_battery_chemical_J / 3.6e6:.2f}"
          f" kWh), peak battery power {result.power_battery_max_W / 1e3:.0f} kW, SOC {result.soc[0]:.3f} -> "
          f"{result.soc[-1]:.3f}")
    print(f"Rule of thumb ({duration_rule_s:.0f} s at {factor_rule:g} x cruise power): "
          f"{energy_rule_J / 3.6e6:.2f} kWh; optimized / rule = {result.energy_electric_J / energy_rule_J:.2f}")
    path = Path(__file__).resolve().parents[1] / "output" / "all_electric_transition_climb.png"
    plot(result, power_cruise_W, path)
    print(f"Figure: {path}")
    return result, power_cruise_W, energy_rule_J


if __name__ == "__main__":
    main()
