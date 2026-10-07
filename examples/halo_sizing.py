"""Halo-class two-rotor series-hybrid tiltrotor: simultaneous sizing, mission and energy allocation.

The Tier 9 formulation, re-baselined on the Bell XV-15 (plans 011-013):
two tip-mounted rotors, two turbogenerators on one bus with a battery, and the
validated Tier 10 models.

Requirements (adopted 2026-10-02, revised 2026-10-03): 900 kg payload (780 kg in plan 022, back to
900 kg with the AFDD tiltrotor wing in plan 026), 445 nm mission, 210 kt at 10,000 ft, 13,000 ft ceiling, OGE hover at 4,000 ft, engine-out
hover on one turbogenerator plus battery, stall at most 120 kt.

Models carried from Tier 10:
  * weights: Raymer GA airframe x XV-15 calibration factors; AFDD82 rotor group
    and AFDD83 gearboxes; AFDD82 engine section; AeroSandbox turboshaft
    mass-power regression as an explicit Opti equality;
  * engine: lapse sigma^0.797 and part-power knockdown (XV-15 validated);
    Tier 16 adds the 95 F temperature lapse (T / T_ISA)^-2.49 (XV-15 fit);
  * hover: figure of merit 0.67 with 7 % download (XV-15 calibrated).
Tier 16 adds an OGE hover at a hot/high destination (default 4,000 ft / 95 F)
at the mission's end mass and SOC (`HaloRequirements.hover_hot_day`).
Tier 20 (plan 024) adds the AFDD tiltrotor wing as an option
(`HaloAssumptions.wing_weight_model = "afdd_tiltrotor"`) with whirl-flutter
frequency margins at every airplane-mode point, and itemises the uncrewed
equipment changes from the XV-15 statement.
Tier 19 (plan 028) adds a thermal option (`HaloAssumptions.thermal_model`): heat
loads from every loss, a ram-air heat exchanger (mass, cooling drag, hover fan
power) sized by a design-variable rating, and lumped motor and generator
temperatures that let hover and engine-out peaks exceed the continuous rating.
Tier 18 (plan 032) adds a redundancy option (`HaloAssumptions.redundancy`): lane motors
per rotor, cross-strapped buses with bus ties, isolated battery strings, and failure
hovers (lane out, bus out, string out; optional double failures with an engine out)
as extra points of the one sizing problem.
Assumptions specific to this case are fields of `HaloAssumptions` and are
listed in plan 013. Illustrative study, not Archer data.
"""
import time
from dataclasses import dataclass, field, fields, replace
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np
import aerosandbox.tools.units as u
from aerosandbox.library.power_turboshaft import power_turboshaft, thermal_efficiency_turboshaft

from aircraft_closure.aerodynamics.trim import TailTrim
from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.scholz import ScholzAerodynamics
from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.aerodynamics.slipstream import BlownWing
from aircraft_closure.controls.stability import DirectionalStability, LongitudinalStability
from aircraft_closure.core.margins import margin_above, margin_below, margin_report
from aircraft_closure.mission.cost import CostBreakdown, CostModel
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import (ClimbSegment, CruiseSegment, DescentSegment, HoverSegment,
                                               LoiterSegment, ground_distance_m)
from aircraft_closure.performance.flight_point import FlightCondition, acceleration_gravity_m_s2, build_flight_point
from aircraft_closure.powertrain.components.battery import Battery
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery, inr21700_50g_cell
from aircraft_closure.powertrain.components.cable import Cable, aluminium_conductor
from aircraft_closure.powertrain.components.converters import ConverterLossModel, DcDcConverter, Inverter
from aircraft_closure.powertrain.components.gearbox import Gearbox, GearStageModel
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.heat_exchanger import RamAirHeatExchanger
from aircraft_closure.powertrain.components.motor import (DatabaseMassModel, Motor, TorqueDensityMassModel,
                                                          UnitMachineMassModel,
                                                          rubber_machine)
from aircraft_closure.powertrain.components.protection import ProtectionUnit
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.rotor import MomentumProfileRotor
from aircraft_closure.powertrain.components.thermal import LumpedThermalModel
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft, deck_1120hp_part_power_model
from aircraft_closure.powertrain.topologies import ElectricalLayer, RedundancyLayer, build_series_hybrid
from aircraft_closure.requirements.capability import (CeilingRequirement, ClimbRequirement, HoverRequirement,
                                                      RequirementSet, SpeedRequirement)
from aircraft_closure.vehicle.aircraft import Aircraft, MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.fuselage import Fuselage
from aircraft_closure.vehicle.items import FixedEquipment, FuelLoad, LandingGear, Nacelles, Payload, Systems
from aircraft_closure.thermal.heat import coolant_temperatures_C, thermal_parameters
from aircraft_closure.vehicle.powertrain_installation import (InstalledCooling, InstalledInstance,
                                                              PowertrainInstallation)
from aircraft_closure.vehicle.surfaces import (HorizontalTail, TiltrotorWingMassModel, VerticalTail, Wing,
                                               aluminium_wing_material, graphite_epoxy_wing_material)
from aircraft_closure.weights import afdd
from examples.xv15_hot_day import temperature_from_fahrenheit_K, temperature_offset_K, xv15_lapse_model
from examples.xv15_performance import fit_lapse_exponent
from examples.xv15_reference import Xv15MassFactors, Xv15Reference, calibration_factors

fuel_factor = 1.1
soc_take_off = 0.95
soc_minimum = 0.3
soc_emergency_floor = 0.1
scale_objective_cost_usd = 100.0       # objective="cost": IPOPT converged from the mass optimum with 100, not 1,000


@dataclass(frozen=True)
class EquipmentItem:
    """One line of the fixed-equipment build-up; negative masses are removals."""
    label: str
    mass_kg: float
    source: str


# XV-15 fixed-equipment groups (TM X-62407 sec. 3.1.2), the starting point for the Halo equipment.
xv15_equipment_items = (
    EquipmentItem("electrical", 396 * u.lbm, "TM X-62407 sec. 3.1.2"),
    EquipmentItem("instrumentation", 91 * u.lbm, "TM X-62407 sec. 3.1.2"),
    EquipmentItem("heating and air conditioning", 100 * u.lbm, "TM X-62407 sec. 3.1.2"),
    EquipmentItem("miscellaneous (furnishings, equipment)", 436 * u.lbm, "TM X-62407 sec. 3.1.2"),
)
# Uncrewed changes, itemised (plan 024). Until Tier 20 the same 587 lb was carried as one number, with the
# heating group relabelled as autonomy; the total is unchanged.
uncrewed_equipment_adjustments = (
    EquipmentItem("remove crew environmental control (heating, ventilation, air conditioning)", -100 * u.lbm,
                  "TM X-62407 sec. 3.1.2 (the ECS serves the crew station)"),
    EquipmentItem("remove ejection seats", -230 * u.lbm,
                  "Harris, NASA/SP-2015-215959 vol. III p. 312 (from the XV-15 weight statement)"),
    EquipmentItem("remove other furnishings and miscellaneous crew equipment", -(436 - 230) * u.lbm,
                  "TM X-62407 sec. 3.1.2 miscellaneous less ejection seats"),
    EquipmentItem("add autonomy and mission avionics (flight computers, sensors, datalinks)", 100 * u.lbm,
                  "assumed allocation (the XV-15 carried 144 lb of avionics as useful load, not empty weight)"),
    # Crew items inside other groups are not split out (no public breakdown): cockpit controls in flight
    # controls, crew-station and crash structure in the fuselage. They remain inside those calibration factors.
)
halo_equipment_items = xv15_equipment_items + uncrewed_equipment_adjustments


def mass_equipment_from_items_kg(items):
    return sum(item.mass_kg for item in items)


@dataclass(frozen=True)
class HaloRequirements:
    # 900 kg (plan 026): with the AFDD tiltrotor wing the equivalent-circuit pack carries up to 959 kg at 210 kt.
    # 780 kg with the Raymer wing (plan 022, requirements_plan022); 900 kg before that (requirements_tier16).
    mass_payload_kg: float = 900.0
    range_m: float = 445 * 1852.0
    altitude_cruise_m: float = 10000 * u.foot
    # 210 kt (plan 017): the most the fixed 2 x 1,120 hp turboshafts sustain with 900 kg over 445 nm (max
    # payload 935 kg); 250 kt is infeasible at any size with these engines. Pending the author's choice.
    velocity_max_m_s: float = 210 * u.knot
    altitude_hover_m: float = 4000 * u.foot
    thrust_to_weight_hover: float = 1.05
    altitude_ceiling_m: float = 13000 * u.foot
    climb_rate_ceiling_m_s: float = 0.5
    climb_rate_m_s: float = 7.5
    velocity_climb_m_s: float = 80.0
    velocity_stall_max_m_s: float = 120 * u.knot
    cl_max: float = 1.5
    duration_loiter_s: float = 1200.0
    duration_hover_s: float = 60.0
    duration_engine_out_hover_s: float = 60.0
    # Tier 16: OGE hover at a hot/high destination after the mission, at its end mass and end SOC, for
    # `duration_hover_hot_s` down to the emergency SOC floor. Default 4,000 ft / 95 F ("4k/95", ISA + 27.7 K).
    hover_hot_day: bool = True
    altitude_hover_hot_m: float = 4000 * u.foot
    temperature_hover_hot_K: float = temperature_from_fahrenheit_K(95.0)
    thrust_to_weight_hover_hot: float = 1.05
    duration_hover_hot_s: float = 60.0

    def requirement_set(self):
        return RequirementSet(
            mass_payload_kg=self.mass_payload_kg,
            hover=HoverRequirement(altitude_m=self.altitude_hover_m, thrust_to_weight=self.thrust_to_weight_hover),
            climb=ClimbRequirement(climb_rate_m_s=self.climb_rate_m_s, velocity_m_s=self.velocity_climb_m_s,
                                   altitude_m=1000.0),
            speed=SpeedRequirement(velocity_m_s=self.velocity_max_m_s, altitude_m=self.altitude_cruise_m),
            ceiling=CeilingRequirement(altitude_m=self.altitude_ceiling_m, climb_rate_m_s=self.climb_rate_ceiling_m_s,
                                       velocity_m_s=self.velocity_climb_m_s),
        )


@dataclass(frozen=True)
class HaloAssumptions:
    count_rotors: int = 2
    count_turbogenerators: int = 2
    count_blades: int = 3
    solidity: float = 0.089                       # XV-15
    speed_tip_m_s: float = 740 * u.foot           # XV-15 hover tip speed, also the bound
    frequency_coning_per_rev: float = 1.55        # XV-15 reading; the rotor factor calibrates it
    figure_of_merit: float = 0.67                 # XV-15 calibrated (Tier 10b); actuator-disk rotor only
    coefficient_airplane: float = 0.87            # assumed; actuator-disk rotor only
    # Tier 12: momentum + profile rotor calibrated on JVX (rotor speed, solidity and tip speed matter); False
    # restores the Tier 10c actuator disk with the two constant coefficients above.
    rotor_speed_physics: bool = True
    # Plan 017 (decided 2026-10-03): off-the-shelf turboshafts, power fixed by the engine deck (1,120 hp SLS, the
    # user-supplied GASP_TS deck); a non-OEM cannot raise power available. None restores a sized (rubber) engine.
    power_rated_turboshaft_fixed_W: Any = 1120 * u.hp
    # The battery supplements the turbines (hover) and is recharged by the generators on free-split segments
    # (negative battery share), within its charge rating and SOC window. 0 forbids in-flight charging.
    hybridization_electric_min: float = -1.0
    # Hold the mission SOC floor at every segment end (the engine-out reserve is assessed from it); False keeps
    # only the battery's own window, as in Tiers 10c-12.
    soc_floor_every_segment: bool = True
    # Tier 13 (plan 018): machines sized by torque (TorqueDensityMassModel); motor speed, rotor gear ratio and
    # generator speed become design variables. False keeps power / specific power (5 / 4 kW/kg) and fixed speeds.
    machine_mass_by_torque: bool = True
    torque_density_Nm_kg: float = 15.0                 # magniX class, including inverters and cables
    specific_power_max_machine_W_kg: float = 10000.0   # high-speed cap
    generator_step_up: bool = True                     # step-up gearbox from the engine's output shaft
    speed_output_turboshaft_rad_s: float = 1210 * u.rpm  # deck propeller_rpm: the engine as delivered (non-OEM)
    direct_drive_rotor: bool = False                   # True: no rotor gearbox, motor turns at rotor speed
    mach_tip_hover_max: float = 0.70              # hover tip Mach bound on the design tip speed (JVX tested 0.68-0.73)
    download_fraction_hover: float = 0.07         # XV-15 TM X-62407 sec. 5.1
    reduction_ratio: float = 7.0                  # motor near peak-efficiency speed in hover
    speed_peak_motor_rad_s: float = 400.0
    speed_peak_generator_rad_s: float = 400.0
    aspect_ratio_wing: float = 6.12               # XV-15
    # Plan 037 (2026-10-04: "11 m, as drawn"): the Halo drawing's length; with the boxy section it has the
    # wetted area of the round XV-15 tube (42.1 ft), on which the 1.70 fuselage factor was anchored.
    length_fuselage_m: float = 11.0
    diameter_fuselage_m: float = 5.5 * u.foot     # XV-15 (Tier 10a assumption); the width of a boxy section
    # Plan 037 (author: "archer halo is not pressurized. that's why it's so boxy"): a super-ellipse section this deep
    # and with this exponent (plan 031 drawing). None: the round XV-15 section, the reference until plan 037.
    height_fuselage_m: Any = 2.0
    shape_fuselage: float = 3.2
    clearance_rotor_fuselage_m: float = 0.3       # XV-15 ~1 ft
    x_horizontal_tail_m: float = 9.8              # tail root leading edge; ends inside the 11 m fuselage (11.4 before plan 037)
    drag_area_misc_m2: float = 0.8                # assumed: tip nacelles, spinners, gear fairings
    load_factor_ultimate: float = 4.5             # XV-15
    mass_equipment_kg: float = mass_equipment_from_items_kg(halo_equipment_items)  # 587 lb; itemised above
    area_wetted_nacelles_m2: float = 2 * 95 * u.foot**2
    resistance_energy_product_ohm_J: float = 1.8e6
    battery_mass_smoothing_kg: float = 10.0
    # Part-power fuel curve: the author-supplied 1,120 hp GASP deck (plan 014); GeissPartPowerModel() is the
    # XV-15-validated alternative (within 1 % of it above 50 % power).
    part_power_model: Any = field(default_factory=deck_1120hp_part_power_model)
    # Tier 16: turboshaft lapse in density and temperature (XV-15 95 F fit); identical to sigma^n on a standard
    # day. False keeps the density-only lapse at any temperature.
    temperature_lapse: bool = True
    # ---- Tier 17 battery (plan 021) ----
    # "ecm": EquivalentCircuitBattery, Samsung INR21700-50G OCV and resistance shape (Paudel et al. 2025);
    # "constant": the Tier 1 constant-OCV Battery sized by energy and power (the reference for Tiers 10c-16).
    # The ECM pack is the reference from plan 022 (it cannot carry 900 kg at 210 kt on the fixed engines).
    battery_model: str = "ecm"
    # Decision 2026-10-02: resistance / factor and current rating x factor (more power-dense 50G-shaped cell).
    # 5: 10C continuous; below about 5 battery power sizes the pack, above about 8 the reserve energy does (plan 021).
    factor_power_density_battery: float = 5.0
    count_series_battery: int = 210               # 756 V nominal; 525-882 V window inside the 400-900 V machines
    factor_capacity_ageing_battery: float = 0.8   # end of life: 80 % of rated capacity (assumed)
    factor_resistance_ageing_battery: float = 1.5  # end of life: +50 % resistance (assumed)
    temperature_cell_battery_C: float = 25.0      # thermally managed pack (assumed)
    fraction_mass_cells_battery: float = 0.7      # cell / pack mass (assumed; cylindrical-cell packs ~0.65-0.75)
    # Points per mission segment (take-off hover, climb, cruise, loiter, descent, landing hover) so OCV and
    # resistance follow SOC through long segments; and points in the 60 s engine-out hover (SOC 0.30 -> 0.10).
    subsegments_mission: tuple = (1, 1, 4, 1, 1, 1)
    subsegments_engine_out: int = 3
    # ---- Tier 15 electrical layer (plan 023) ----
    # True inserts inverters (motors) and active rectifiers (generators), DC feeder cables and protection
    # between the machines, the battery and the bus, and sizes the machines as bare machines. False keeps the
    # machines connected to the bus directly, with the integrated Tier 13 machine figures above.
    electrical_layer: bool = False
    # Bare-machine figures that keep the Tier 13 integrated calibration: 1/15 = 1/17.6 + 200 rad/s / 20 kW/kg
    # (magniX at its speed) and 1/10 kW/kg = 1/20 + 1/20 (high-speed cap). 20 kW/kg is above NASA's HEMM
    # 16 kW/kg electromagnetic target, i.e. the Tier 13 cap was optimistic (plan 023 sensitivity).
    torque_density_machine_bare_Nm_kg: float = 17.6
    specific_power_max_machine_bare_W_kg: float = 20000.0
    specific_power_inverter_W_kg: float = 20000.0      # NASA EAP goal ~19 kW/kg (Jansen et al. 2017); assumed
    efficiency_inverter: float = 0.985                 # at rated power and nominal bus voltage (SiC; assumed)
    voltage_blocking_inverter_V: float = 1200.0        # semiconductor class; DC link <= blocking x derating
    factor_derating_voltage_inverter: float = 0.75     # cosmic-ray single-event-burnout margin (assumed)
    factor_voltage_min_machine: float = 0.9            # machines and inverters work down to 0.9 x pack minimum
    factor_routing_cable: float = 1.25                 # feeder length = factor x half span (bus in the fuselage)
    length_cable_battery_m: float = 3.0
    factor_rating_feeder: float = 1.25                 # feeder current rating / source rating (NEC-style 125 %)
    conductor_cable: Any = field(default_factory=aluminium_conductor)
    # Optional DC/DC converter between the battery feeder and the bus, regulating the bus at voltage_bus_dcdc_V.
    dcdc_converter: bool = False
    voltage_bus_dcdc_V: float = 800.0
    specific_power_dcdc_W_kg: float = 12000.0          # assumed
    efficiency_dcdc: float = 0.98                      # assumed
    # ---- Tier 20 wing weight (plan 024) ----
    # "afdd_tiltrotor" (the reference from plan 026, adopted 2026-10-03): the AFDD tiltrotor wing
    # (NDARC 19-1.1) x its own XV-15 factor, built for the wing design rotor speed (a design variable) and checked
    # by whirl-flutter frequency margins at every airplane-mode point.
    # "raymer": Raymer GA wing x the XV-15 wing factor (the reference until plan 026).
    wing_weight_model: str = "afdd_tiltrotor"
    # XV-15 symmetric wing modes in per rev of its airplane-mode rotor speed (torsion 1.09, beam 0.43, chord 0.83).
    frequency_torsion_wing_per_rev: float = Xv15Reference().frequency_torsion_wing_per_rev
    frequency_beam_wing_per_rev: float = Xv15Reference().frequency_beam_wing_per_rev
    frequency_chord_wing_per_rev: float = Xv15Reference().frequency_chord_wing_per_rev
    thickness_to_chord_wing: float = 0.23                # NACA 2423 airfoil above (XV-15 64A223, V-22 23 %)
    wing_material: str = "graphite_epoxy"                # or "aluminium" (the XV-15's)
    # Plan 038: None builds the pylon pitch inertia about the spindle from the tip components (rotor at the hub,
    # motor and gearbox near it, cowling spread along the nacelle); a number is the pylon radius of gyration / rotor
    # radius (0.222, the XV-15 with its engines in the nacelles: the reference until plan 038).
    ratio_radius_gyration_pylon: Any = None
    length_mast_m: float = 1.0                           # spindle to rotor hub (the plan 031 layout)
    offset_drive_pylon_m: float = 0.6                    # spindle to the motor and rotor gearbox (assumed)
    radius_gyration_cowling_m: float = 0.8               # cowling about the spindle (3.4 m nacelle, assumed)
    # Plan 038 (the CalculiX check of plan 031): spar-cap separation / wing thickness. None: the wing airfoil's depth at
    # the spars (AeroSandbox `local_thickness`, mean of the two stations); 1.0: NDARC.
    ratio_depth_spar_cap: Any = None
    fraction_chord_front_spar: float = 0.15
    fraction_chord_rear_spar: float = 0.60
    thickness_min_torque_box_m: float = 0.001            # minimum torque-box wall gauge (composite, assumed); 0: NDARC
    # Plan 037 (2026-10-04: "turbines are likely inside the fuselage"): False puts the turbogenerators, their
    # gearboxes and the engine support and air induction in the fuselage behind the wing box; the tip nacelles keep
    # the rotor, motor, rotor gearbox and cowling. True: the XV-15 arrangement, the reference until plan 037.
    turbogenerators_on_wing_tips: bool = False
    # Plan 038: None places the turbogenerators by the layout, their centre half an engine length (0.5 m, assumed)
    # behind the wing rear spar and 0.35 m below the fuselage top; numbers are the plan 037 placement (1.0 m behind
    # the wing quarter chord, z 0.5 m).
    offset_x_turbogenerators_m: Any = None
    z_turbogenerators_m: Any = None
    length_half_turbogenerator_m: float = 0.5
    depth_turbogenerator_below_top_m: float = 0.35
    load_factor_jump: float = 2.0
    smoothing_wing_tiltrotor: float = 0.01               # rounds the AFDD max(0, .) steps for IPOPT
    # ---- Tier 21 aerodynamics (plan 025) ----
    # "simple": SimpleAerodynamics with `drag_area_misc_m2` and the constant download (the reference until plan 027);
    # "buildup": AeroSandbox AeroBuildup plus Scholz interference, the misc. drag area below, blown wing and the
    # geometric hover download; "scholz": the Scholz level-0 hand check (linear lift, no blown wing).
    aerodynamics_model: str = "buildup"   # plan 027 (decided 2026-10-04); "simple" until then
    length_nacelle_m: float = 9.0 * u.foot        # assumed; with the diameter, ~95 ft2 wetted (cowling mass)
    diameter_nacelle_m: float = 3.3 * u.foot
    drag_area_misc_buildup_m2: float = 3.00 * u.foot**2  # XV-15 "fuselage fittings & fixtures" (NDARC, Johnson 2010)
    blown_wing: bool = True                       # "buildup": rotor slipstream increments in airplane mode
    # ---- Tier 22 design-space practice (plan 029) ----
    # Freed trades: each flag turns a fixed assumption above into a design variable within the bounds.
    # Mission cruise and loiter altitude: drag against turboshaft lapse and climb energy. The 210 kt speed
    # requirement stays at `HaloRequirements.altitude_cruise_m`; the upper bound is the 13,000 ft ceiling.
    free_altitude_cruise: bool = False
    bounds_altitude_cruise_m: tuple = (2000 * u.foot, 13000 * u.foot)
    # Wing aspect ratio: induced drag against wing weight, whirl-flutter stiffness and rotor-span clearance.
    free_aspect_ratio_wing: bool = False
    bounds_aspect_ratio_wing: tuple = (4.0, 12.0)
    # Reserve SOC: the mission SOC floor, which is also where the engine-out hover starts (0.30 when fixed).
    free_soc_reserve: bool = False
    bounds_soc_reserve: tuple = (0.20, 0.90)
    # Cost per mission for objective="cost" and for every result's `cost` (src/aircraft_closure/mission/cost.py).
    cost_model: Any = field(default_factory=CostModel)
    # Plan 034: drag AeroBuildup and the Scholz build-up leave out. Excrescence, leakage and protuberance as a factor
    # on component drag, calibrated so the XV-15 components match NDARC's 6.25 ft2 (Johnson 2010, Table 1); trim
    # drag as a fraction of parasite + induced drag (assumed 2 %, within the 1-5 % usual for an aft tail at cruise).
    drag_corrections: bool = True   # plan 035 default (decided 2026-10-04); False before
    factor_excrescence_buildup: float = 1.27
    factor_excrescence_scholz: float = 1.17
    fraction_trim_drag: float = 0.02                   # Scholz aero only; AeroBuildup trims from the tail load
    efficiency_tail: float = 0.9                       # tail dynamic-pressure ratio eta_H (Scholz ch. 11)
    angle_dihedral_tail_deg: float = 0.0               # 0: conventional horizontal tail
    # ---- Fuselage mass (plan 037) ----
    # Raw Raymer GA (unpressurized) times this factor. 1.70 anchors it to the plan 031 layout estimate of the
    # uncrewed cargo fuselage (primary 310 + secondary 220 kg at the plan 030 reference, 1.73x raw Raymer; author
    # chose "layout-anchored ~1.7" on 2026-10-04). None: the XV-15 group calibration (2.07, a crewed fuselage),
    # the reference until plan 037.
    mass_factor_fuselage: Any = 1.70
    # ---- Tier 19 thermal (plan 028) ----
    # True: heat loads from every loss go to a ram-air heat exchanger (mass from its rating, a design variable;
    # cooling drag in airplane mode, fan power in hover), and lumped motor and generator temperatures replace
    # their power ratings (hover and engine-out peaks may exceed the continuous rating for their duration). The
    # rotor gearbox is then rated separately (`HaloDesign.power_rated_gearbox_W`). True is the reference from plan 030
    # (adopted 2026-10-04); False: the reference until then (`assumptions_plan027`).
    thermal_model: bool = True
    specific_power_heat_exchanger_W_kg: float = 1000.0   # at the 40 K reference (Kellermann 2021, Potamiti 2024)
    delta_temperature_ref_heat_exchanger_C: float = 40.0
    temperature_coolant_C: float = 60.0                  # water-glycol loop into the heat exchanger and machines
    effectiveness_heat_exchanger: float = 0.8
    pressure_drop_ref_heat_exchanger_Pa: float = 1000.0  # air side, at the rated flow and sea-level density
    efficiency_fan_heat_exchanger: float = 0.6
    specific_heat_machine_J_kg_K: float = 500.0          # effective, whole machine mass
    temperature_max_machine_C: float = 150.0             # mean winding (class H: 180 C hot spot)
    # The pack: lumped on its whole mass, its loop held at `temperature_cell_battery_C` (25 C; moving that heat
    # into the 60 C loop needs a chiller, not modelled). Its thermal mass absorbs the engine-out peak.
    specific_heat_battery_J_kg_K: float = 1000.0         # Li-ion cells ~1,000-1,100 J/(kg K), pack structure less
    temperature_max_battery_C: float = 60.0
    # Heat loads cooled elsewhere: the gearboxes' own oil coolers, assumed inside the XV-15-calibrated AFDD drive
    # system weights (they include the lubrication systems). Their heat is still reported.
    sources_excluded_heat_exchanger: tuple = ("gearbox", "generator_gearbox")
    # ---- Tier 18 redundancy (plan 032) ----
    # True: `count_lanes_motor` lane motors per rotor (each 1/N of the torque, on a combining gearbox input),
    # `count_buses` cross-strapped buses joined by normally open ties (contactor + cable), `count_strings_battery`
    # isolated pack strings (a contactor each), and the failure hovers below as extra points of the sizing problem.
    # True is the default from plan 035. False: the single-lane aircraft. With True, all counts 1 and no failure cases the
    # problem is the same as with False.
    redundancy: bool = True   # plan 035 default (decided 2026-10-04); False before
    count_lanes_motor: int = 2
    count_buses: int = 2
    count_strings_battery: int = 2
    length_cable_bus_tie_m: float = 2.0              # bus to bus in the fuselage (assumed)
    # Failure hovers: 60 s at MTOM from the SOC floor down to the emergency floor, thermal history from the end of
    # the take-off hover (as the engine-out hover). A lane out is applied to both rotors (conservative).
    failure_lane_out: bool = True                    # one lane per rotor lost, all engines
    failure_bus_out: bool = True                     # one bus lost: its lanes lost, its sources through a tie
    failure_string_out: bool = True                  # one battery string isolated, all engines
    failure_string_out_engine_out: bool = False      # double failure: a string and an engine
    failure_lane_out_engine_out: bool = False        # double failure: a lane per rotor and an engine
    # ---- Plan 033 machine database and gearbox stages (refines Tier 13) ----
    # Machine mass: "units" (whole units of real machines, plan 035, the default), "torque_density"
    # (TorqueDensityMassModel above, the rubber reference until plan 035) or "database"
    # (DatabaseMassModel: continuous torque density falling with base speed, fitted to
    # data/machines/aerospace_motors.csv). The database model is a bare-machine fit: with the electrical layer
    # the inverter is the separate component; without it the model adds rated power / specific_power_inverter_W_kg.
    # The cap is specific_power_max_machine_bare_W_kg in both modes.
    machine_mass_model: str = "units"   # plan 035 default (decided 2026-10-04); "torque_density" before
    torque_density_database_Nm_kg: float = 11.79       # at speed_ref_database_rad_s (fit, plan 033)
    speed_ref_database_rad_s: float = 500.0
    exponent_speed_database: float = 0.271
    # True: rotor and generator step-up gearboxes get an explicit stage count from their ratio, with mass factor
    # and efficiency per stage (GearStageModel). The AFDD drive mass is evaluated at the XV-15 calibration ratio
    # (20,000 / 565 rpm) and scaled by mass_factor(ratio) / mass_factor(XV-15 ratio). False: AFDD with its own
    # mild ratio exponent and a constant 0.97 efficiency (the reference until plan 035).
    gearbox_stages: bool = True   # plan 035 default (decided 2026-10-04); False before
    # Relaxed (softplus) stage count: the smooth staircase (GearStageModel(staircase=True)) solves on the fast set
    # but not through the AeroBuildup and thermal starts with the database machines (plan 033).
    gear_stage_model: Any = field(default_factory=lambda: GearStageModel(staircase=False))
    reduction_ratio_max: float = 40.0                  # upper bound on the rotor gear ratio (slow-motor trades)
    # ---- Plan 035: machines from whole units of real best-in-class products ("no rubber motors", author 2026-10-04) --
    # machine_mass_model "units": each lane motor is `count_units_motor` stacked units of a low-speed axial-flux
    # machine, each generator `count_units_generator` units of a high-speed radial machine. None: the relaxed count,
    # then (`integer_units`) rounded up and re-solved. Unit data from data/machines/aerospace_motors.csv.
    # Motor unit: Evolito D1500 2x3 (40 kg, 1,500 N m peak, rated 35 N m/kg -> 1,400 N m continuous, 2,500 rpm max;
    # continuous power at the 1,800 rpm peak-power speed: 1,400 x 188.5 rad/s = 264 kW).
    mass_unit_motor_kg: float = 40.0
    torque_peak_unit_motor_Nm: float = 1500.0
    power_continuous_unit_motor_W: float = 1400.0 * 1800 * u.rpm
    speed_max_unit_motor_rad_s: float = 2500 * u.rpm
    # Generator unit: Helix SPX242 demonstrator (31.2 kg, 470 N m peak, 315 kW continuous, 17,000 rpm max).
    mass_unit_generator_kg: float = 31.2
    torque_peak_unit_generator_Nm: float = 470.0
    power_continuous_unit_generator_W: float = 315e3
    speed_max_unit_generator_rad_s: float = 17000 * u.rpm
    count_units_motor: Any = None
    count_units_generator: Any = None
    integer_units: bool = True


@dataclass(frozen=True)
class HaloDesign:
    """Every sized quantity; fields may be Opti variables or numbers."""
    x_le_wing_m: Any
    area_wing_m2: Any
    area_horizontal_tail_m2: Any
    area_vertical_tail_m2: Any
    torque_peak_motor_Nm: Any
    torque_peak_generator_Nm: Any
    power_rated_turboshaft_W: Any
    mass_turboshaft_bare_kg: Any
    power_max_discharge_battery_W: Any
    energy_capacity_battery_J: Any
    area_disk_m2: Any
    mass_fuel_kg: Any
    solidity: Any = None                          # None: the assumption value (fixed)
    speed_tip_m_s: Any = None                     # design (hover) tip speed; None: the assumption value
    speed_peak_motor_rad_s: Any = None            # Tier 13 variables; None: the assumption values
    reduction_ratio: Any = None
    speed_peak_generator_rad_s: Any = None
    count_parallel_battery: Any = None            # Tier 17 "ecm": parallel strings (continuous); energy and power
                                                  # above are then derived from the pack
    speed_rotor_wing_design_rad_s: Any = None     # Tier 20 "afdd_tiltrotor": rotor speed the wing frequencies are
                                                  # placed against; None: the design (hover) rotor speed
    altitude_cruise_m: Any = None                 # Tier 22 freed trades; None: the fixed values
    aspect_ratio_wing: Any = None
    soc_reserve: Any = None
    power_rated_heat_exchanger_W: Any = None      # Tier 19 thermal: heat rejected at the reference difference
    power_rated_gearbox_W: Any = None             # Tier 19 thermal: rotor gearbox rating; None: the motor rating


def battery_thermal(assumptions):
    """Tier 19: the pack's lumped thermal model (its own loop at the managed cell temperature)."""
    a = assumptions
    if not a.thermal_model:
        return {}
    return dict(thermal_model=LumpedThermalModel(a.specific_heat_battery_J_kg_K, a.temperature_max_battery_C,
                                                 a.temperature_cell_battery_C))


def build_halo_battery(design, assumptions=HaloAssumptions()):
    a, d = assumptions, design
    if a.battery_model == "ecm":
        return EquivalentCircuitBattery(count_series=a.count_series_battery, count_parallel=d.count_parallel_battery,
                                        factor_power_density=a.factor_power_density_battery,
                                        temperature_cell_C=a.temperature_cell_battery_C,
                                        factor_capacity_ageing=a.factor_capacity_ageing_battery,
                                        factor_resistance_ageing=a.factor_resistance_ageing_battery,
                                        fraction_mass_cells=a.fraction_mass_cells_battery,
                                        min_soc=soc_emergency_floor, max_soc=soc_take_off, **battery_thermal(a))
    if a.battery_model == "constant":
        return Battery(energy_capacity_J=d.energy_capacity_battery_J,
                       resistance_ohm=a.resistance_energy_product_ohm_J / d.energy_capacity_battery_J,
                       max_discharge_power_W=d.power_max_discharge_battery_W,
                       max_charge_power_W=0.5 * d.power_max_discharge_battery_W,
                       mass_smoothing_kg=a.battery_mass_smoothing_kg, **battery_thermal(a))
    raise ValueError(f"Unknown battery model '{a.battery_model}'.")


@dataclass(frozen=True)
class BusVoltageWindow:
    """Tier 15 bus voltages implied by the pack (or the DC/DC converter) and the inverter class."""
    voltage_nominal_V: float
    voltage_min_V: float           # lowest bus voltage the machines and feeders are designed for
    voltage_max_V: float           # highest bus voltage (insulation design)
    voltage_max_inverter_V: float  # derated semiconductor limit


def bus_voltage_window(assumptions=HaloAssumptions()):
    a = assumptions
    cell = inr21700_50g_cell()
    if a.dcdc_converter:
        nominal_V, min_V, max_V = a.voltage_bus_dcdc_V, a.voltage_bus_dcdc_V, a.voltage_bus_dcdc_V
    else:
        nominal_V = a.count_series_battery * cell.voltage_nominal_V
        min_V = a.count_series_battery * cell.voltage_min_V
        max_V = a.count_series_battery * cell.voltage_max_V
    return BusVoltageWindow(nominal_V, min_V, max_V, a.voltage_blocking_inverter_V * a.factor_derating_voltage_inverter)


def power_electric_rated_W(machine):
    """Electrical rating of a machine's converter and feeder: shaft rating / peak machine efficiency."""
    return machine.power_rated_W / machine.loss_model.efficiency_peak


def build_halo_electrical(motor, generator, battery, span_m, requirements=HaloRequirements(),
                          assumptions=HaloAssumptions()):
    """Tier 15 feeders: inverters at machine rating, cables and protection sized at rated power and the
    lowest bus voltage (battery: its discharge current rating), insulation for the highest bus voltage at the
    ceiling."""
    a, window = assumptions, bus_voltage_window(assumptions)
    cell = inr21700_50g_cell()
    pack_min_V = a.count_series_battery * cell.voltage_min_V
    pack_max_V = a.count_series_battery * cell.voltage_max_V
    if isinstance(battery, EquivalentCircuitBattery):
        current_battery_max_A = battery.get_limits().max_discharge_current_A
        power_battery_max_W = battery.power_max_discharge_W
    else:
        current_battery_max_A = battery.max_discharge_power_W / pack_min_V
        power_battery_max_W = battery.max_discharge_power_W
    loss_model = ConverterLossModel(efficiency_rated=a.efficiency_inverter, voltage_rated_V=window.voltage_nominal_V)
    min_voltage_machine_V = a.factor_voltage_min_machine * window.voltage_min_V

    def inverter(machine):
        # Rated at the machine's electrical power: shaft rating / McDonald peak efficiency (motor input).
        return Inverter(power_rated_W=power_electric_rated_W(machine), specific_power_W_kg=a.specific_power_inverter_W_kg,
                        voltage_blocking_V=a.voltage_blocking_inverter_V,
                        factor_derating_voltage=a.factor_derating_voltage_inverter,
                        min_voltage_V=min_voltage_machine_V, loss_model=loss_model)

    def feeder(length_m, current_A, voltage_max_V):
        # Feeders are rated above their source's continuous current (NEC-style 125 % rule); this also keeps the
        # cable and contactor limits from duplicating the source's own current limit (degenerate constraints).
        current_A = a.factor_rating_feeder * current_A
        return (Cable(length_m=length_m, max_current_A=current_A, max_voltage_V=voltage_max_V,
                      altitude_design_m=requirements.altitude_ceiling_m, conductor=a.conductor_cable),
                ProtectionUnit(max_current_A=current_A, max_voltage_V=voltage_max_V))

    length_nacelle_m = a.factor_routing_cable * span_m / 2
    cable_motor, protection_motor = feeder(length_nacelle_m, power_electric_rated_W(motor) / window.voltage_min_V,
                                           window.voltage_max_V)
    # Generators at the tips feed along the wing; in the fuselage (plan 037) they sit next to the bus.
    length_generator_m = length_nacelle_m if a.turbogenerators_on_wing_tips else a.length_cable_battery_m
    cable_generator, protection_generator = feeder(length_generator_m,
                                                   power_electric_rated_W(generator) / window.voltage_min_V,
                                                   window.voltage_max_V)
    cable_battery, protection_battery = feeder(a.length_cable_battery_m, current_battery_max_A, pack_max_V)
    dcdc = None
    if a.dcdc_converter:
        dcdc = DcDcConverter(power_rated_W=power_battery_max_W, specific_power_W_kg=a.specific_power_dcdc_W_kg,
                             voltage_output_V=a.voltage_bus_dcdc_V, min_voltage_input_V=a.factor_voltage_min_machine
                             * pack_min_V, max_voltage_input_V=pack_max_V,
                             loss_model=ConverterLossModel(efficiency_rated=a.efficiency_dcdc,
                                                           voltage_rated_V=a.count_series_battery
                                                           * cell.voltage_nominal_V))
    return ElectricalLayer(inverter_motor=inverter(motor), cable_motor=cable_motor, protection_motor=protection_motor,
                           inverter_generator=inverter(generator), cable_generator=cable_generator,
                           protection_generator=protection_generator, cable_battery=cable_battery,
                           protection_battery=protection_battery, dcdc=dcdc)


def count_lanes_motor(assumptions):
    """Tier 18: lane motors per rotor (1 without the redundancy option)."""
    return assumptions.count_lanes_motor if assumptions.redundancy else 1


def build_halo_redundancy(generator, battery, requirements=HaloRequirements(), assumptions=HaloAssumptions()):
    """Tier 18 architecture: string contactors at 125 % of a string's discharge current rating; each bus tie
    (contactor + cable) at 125 % of the rated current of one bus's sources (its share of the generators'
    electrical rating and of the pack's discharge rating) at the lowest bus voltage. None without the option."""
    a = assumptions
    if not a.redundancy:
        return None
    window = bus_voltage_window(a)
    cell = inr21700_50g_cell()
    pack_max_V = a.count_series_battery * cell.voltage_max_V
    if isinstance(battery, EquivalentCircuitBattery):
        current_battery_max_A = battery.get_limits().max_discharge_current_A
        power_battery_max_W = battery.power_max_discharge_W
    else:
        current_battery_max_A = battery.max_discharge_power_W / (a.count_series_battery * cell.voltage_min_V)
        power_battery_max_W = battery.max_discharge_power_W
    protection_string = None
    if a.count_strings_battery > 1:
        protection_string = ProtectionUnit(max_current_A=a.factor_rating_feeder * current_battery_max_A
                                           / a.count_strings_battery, max_voltage_V=pack_max_V)
    protection_bus_tie = cable_bus_tie = None
    if a.count_buses > 1:
        current_tie_A = a.factor_rating_feeder * (a.count_turbogenerators * power_electric_rated_W(generator)
                                                  + power_battery_max_W) / (a.count_buses * window.voltage_min_V)
        protection_bus_tie = ProtectionUnit(max_current_A=current_tie_A, max_voltage_V=window.voltage_max_V)
        cable_bus_tie = Cable(length_m=a.length_cable_bus_tie_m, max_current_A=current_tie_A,
                              max_voltage_V=window.voltage_max_V, altitude_design_m=requirements.altitude_ceiling_m,
                              conductor=a.conductor_cable)
    return RedundancyLayer(count_lanes=a.count_lanes_motor, count_buses=a.count_buses,
                           count_strings_battery=a.count_strings_battery, protection_string=protection_string,
                           protection_bus_tie=protection_bus_tie, cable_bus_tie=cable_bus_tie)


def build_halo_aerodynamics(requirements=HaloRequirements(), assumptions=HaloAssumptions()):
    a, r = assumptions, requirements
    if a.aerodynamics_model == "simple":
        return SimpleAerodynamics(drag_area_misc_m2=a.drag_area_misc_m2,
                                  download_fraction_hover=a.download_fraction_hover, cl_max=r.cl_max)
    if a.aerodynamics_model == "buildup":
        # Plan 036: trim drag from the tail load about the CG (Scholz ch. 11) replaces the flat fraction.
        corrections = (dict(factor_excrescence=a.factor_excrescence_buildup,
                            trim=TailTrim(a.efficiency_tail, a.angle_dihedral_tail_deg)) if a.drag_corrections else {})
        return BuildupAerodynamics(cl_max=r.cl_max, drag_area_misc_m2=a.drag_area_misc_buildup_m2,
                                   blown_wing=BlownWing() if a.blown_wing else None, **corrections)
    if a.aerodynamics_model == "scholz":
        corrections = (dict(factor_excrescence=a.factor_excrescence_scholz, fraction_trim_drag=a.fraction_trim_drag)
                       if a.drag_corrections else {})
        return ScholzAerodynamics(cl_max=r.cl_max, drag_area_misc_m2=a.drag_area_misc_buildup_m2, **corrections)
    raise ValueError(f"Unknown aerodynamics model '{a.aerodynamics_model}'.")


def ratio_reference_gear_stages():
    """The XV-15 engine-to-rotor ratio (20,000 / 565 rpm, 35.4:1), where the AFDD transmission factor is calibrated."""
    reference = Xv15Reference()
    return reference.speed_engine_rad_s / reference.speed_rotor_hover_rad_s


def stage_mass_ratio(stage_model, ratio):
    """Plan 033: drive mass relative to the AFDD drive at the XV-15 calibration ratio; `ratio` is fast/slow (>= 1)."""
    return stage_model.mass_factor(ratio) / stage_model.mass_factor(ratio_reference_gear_stages())


def build_halo_aircraft(design, requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                        lapse_exponent=None):
    a, d = assumptions, design
    factors = factors if factors is not None else calibration_factors()
    lapse_exponent = lapse_exponent if lapse_exponent is not None else fit_lapse_exponent()
    ratios = dict(torque_ratio=2.5, power_ratio=1.25, speed_ratio=2.5)
    by_torque = a.machine_mass_by_torque
    if a.machine_mass_model not in ("torque_density", "database", "units"):
        raise ValueError(f"Unknown machine mass model '{a.machine_mass_model}'.")
    is_database = a.machine_mass_model == "database"
    if is_database and not (by_torque or a.electrical_layer):
        raise ValueError("The database machine mass model needs machine_mass_by_torque (speed design variables).")
    if a.electrical_layer:
        # Tier 15: the inverter is a separate component, so the machines are bare machines wound for the bus.
        mass_model = TorqueDensityMassModel(a.torque_density_machine_bare_Nm_kg, a.specific_power_max_machine_bare_W_kg)
        window = bus_voltage_window(a)
        ratios = dict(ratios, min_voltage_V=a.factor_voltage_min_machine * window.voltage_min_V,
                      max_voltage_V=window.voltage_max_inverter_V)
    else:
        mass_model = (TorqueDensityMassModel(a.torque_density_Nm_kg, a.specific_power_max_machine_W_kg)
                      if by_torque else None)
    if is_database:
        # Plan 033: bare machines from the database fit; the inverter is added unless it is a Tier 15 component.
        mass_model = DatabaseMassModel(
            torque_density_ref_Nm_kg=a.torque_density_database_Nm_kg, speed_ref_rad_s=a.speed_ref_database_rad_s,
            exponent_speed=a.exponent_speed_database, specific_power_max_W_kg=a.specific_power_max_machine_bare_W_kg,
            specific_power_inverter_W_kg=None if a.electrical_layer else a.specific_power_inverter_W_kg)
    mass_model_generator = mass_model
    speed_peak_motor_rad_s = d.speed_peak_motor_rad_s if d.speed_peak_motor_rad_s is not None else a.speed_peak_motor_rad_s
    if d.speed_peak_generator_rad_s is not None:
        speed_peak_generator_rad_s = d.speed_peak_generator_rad_s
    elif by_torque and not a.generator_step_up:
        speed_peak_generator_rad_s = a.speed_output_turboshaft_rad_s    # generator on the engine output shaft
    else:
        speed_peak_generator_rad_s = a.speed_peak_generator_rad_s
    reduction_ratio = 1.0 if a.direct_drive_rotor else (d.reduction_ratio if d.reduction_ratio is not None
                                                        else a.reduction_ratio)
    thermal = dict(thermal_model=LumpedThermalModel(a.specific_heat_machine_J_kg_K, a.temperature_max_machine_C,
                                                    a.temperature_coolant_C)) if a.thermal_model else {}
    motor = rubber_machine(Motor, speed_peak_motor_rad_s, d.torque_peak_motor_Nm, **ratios, mass_model=mass_model,
                           **thermal)
    generator = rubber_machine(Generator, speed_peak_generator_rad_s, d.torque_peak_generator_Nm, **ratios,
                               mass_model=mass_model_generator, **thermal)
    if a.machine_mass_model == "units":
        # Plan 035: whole units of real machines. The machine's limits are n x the unit's published ratings (max
        # torque, continuous power) and the unit's max speed; the peak-efficiency point (w_hat, Q_hat, design
        # variables) only places the efficiency map. n: `count_units_*`, or the relaxed count the rubber ratings
        # would need (used only to find n). The inverter is added unless it is a Tier 15 component.
        inverter = None if a.electrical_layer else a.specific_power_inverter_W_kg
        motor = unit_machine(motor, a.count_units_motor, UnitMachineMassModel(
            a.mass_unit_motor_kg, a.torque_peak_unit_motor_Nm, a.power_continuous_unit_motor_W,
            a.speed_max_unit_motor_rad_s, specific_power_inverter_W_kg=inverter))
        generator = unit_machine(generator, a.count_units_generator, UnitMachineMassModel(
            a.mass_unit_generator_kg, a.torque_peak_unit_generator_Nm, a.power_continuous_unit_generator_W,
            a.speed_max_unit_generator_rad_s, specific_power_inverter_W_kg=inverter))
    # Tier 19: with thermal machines the drive is rated on its own (the motor's rating is continuous).
    # Tier 18: `motor` is one lane motor; the drive takes all of a rotor's lanes.
    lanes = count_lanes_motor(a)
    power_rated_gearbox_W = (d.power_rated_gearbox_W if a.thermal_model and d.power_rated_gearbox_W is not None
                             else (motor.power_rated_W if lanes == 1 else lanes * motor.power_rated_W))
    radius_m = np.sqrt(d.area_disk_m2 / np.pi)
    solidity = d.solidity if d.solidity is not None else a.solidity
    speed_tip_m_s = d.speed_tip_m_s if d.speed_tip_m_s is not None else a.speed_tip_m_s
    chord_m = solidity * np.pi * radius_m / a.count_blades
    speed_rotor_design_rad_s = speed_tip_m_s / radius_m
    mass_rotors_kg = factors.rotor * afdd.mass_rotor_group_afdd82_kg(a.count_rotors, a.count_blades, radius_m, chord_m,
                                                                     speed_tip_m_s, a.frequency_coning_per_rev)
    if a.direct_drive_rotor:
        # No rotor gearbox: a lossless, massless pass-through keeps the topology uniform.
        gearbox = Gearbox(reduction_ratio=1.0, efficiency=1.0, power_rated_W=power_rated_gearbox_W,
                          specific_power_W_kg=1e12)
    else:
        stages = a.gear_stage_model if a.gearbox_stages else None
        # Plan 033: with explicit stages the AFDD input speed is held at the XV-15 calibration ratio, so its own
        # ratio exponent is not counted twice; the stage mass factor carries the ratio dependence.
        speed_input_rad_s = (speed_rotor_design_rad_s * ratio_reference_gear_stages() if stages is not None
                             else (speed_peak_motor_rad_s if by_torque else a.speed_peak_motor_rad_s))
        if by_torque:
            # AFDD00: mild penalty for higher reduction ratio (input-speed exponent 0.099).
            mass_gearboxes_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd00_kg(
                a.count_rotors, a.count_rotors * power_rated_gearbox_W, speed_input_rad_s, speed_rotor_design_rad_s)
        else:
            mass_gearboxes_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd83_kg(
                a.count_rotors * power_rated_gearbox_W, speed_rotor_design_rad_s, speed_input_rad_s, a.count_rotors,
                0.6)
        efficiency = {}
        if stages is not None:
            mass_gearboxes_kg = mass_gearboxes_kg * stage_mass_ratio(stages, reduction_ratio)
            efficiency = dict(efficiency=stages.efficiency(reduction_ratio))
        gearbox = Gearbox(reduction_ratio=reduction_ratio, power_rated_W=power_rated_gearbox_W,
                          specific_power_W_kg=power_rated_gearbox_W / (mass_gearboxes_kg / a.count_rotors), **efficiency)
    generator_gearbox = None
    if by_torque and a.generator_step_up and d.speed_peak_generator_rad_s is not None:
        # Step-up from the engine's output shaft; AFDD00 with the slow (engine) side as the "rotor" speed.
        ratio_step_up = speed_peak_generator_rad_s / a.speed_output_turboshaft_rad_s
        efficiency = {}
        if a.gearbox_stages:
            mass_generator_gearbox_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd00_kg(
                1, d.power_rated_turboshaft_W, a.speed_output_turboshaft_rad_s * ratio_reference_gear_stages(),
                a.speed_output_turboshaft_rad_s) * stage_mass_ratio(a.gear_stage_model, ratio_step_up)
            efficiency = dict(efficiency=a.gear_stage_model.efficiency(ratio_step_up))
        else:
            mass_generator_gearbox_kg = factors.transmission * afdd.mass_gearbox_rotor_shaft_afdd00_kg(
                1, d.power_rated_turboshaft_W, speed_peak_generator_rad_s, a.speed_output_turboshaft_rad_s)
        generator_gearbox = Gearbox(reduction_ratio=a.speed_output_turboshaft_rad_s / speed_peak_generator_rad_s,
                                    power_rated_W=d.power_rated_turboshaft_W,
                                    specific_power_W_kg=d.power_rated_turboshaft_W / mass_generator_gearbox_kg,
                                    **efficiency)
    if a.rotor_speed_physics:
        rotor = MomentumProfileRotor(area_disk_m2=d.area_disk_m2, solidity=solidity, mass_kg=mass_rotors_kg / a.count_rotors,
                                     max_shaft_power_W=gearbox.power_rated_W * gearbox.efficiency,
                                     speed_tip_max_m_s=speed_tip_m_s)
    else:
        rotor = ActuatorDiskPropulsor(area_disk_m2=d.area_disk_m2, mass_kg=mass_rotors_kg / a.count_rotors,
                                      coefficient_of_performance=a.figure_of_merit,
                                      coefficient_of_performance_airplane=a.coefficient_airplane,
                                      max_shaft_power_W=gearbox.power_rated_W * gearbox.efficiency,
                                      speed_tip_max_m_s=speed_tip_m_s)
    battery = build_halo_battery(design, assumptions)
    turboshaft = SimpleTurboshaft(power_rated_W=d.power_rated_turboshaft_W,
                                  mass_kg=factors.powerplant * d.mass_turboshaft_bare_kg,
                                  thermal_efficiency=thermal_efficiency_turboshaft(d.mass_turboshaft_bare_kg),
                                  lapse_exponent=lapse_exponent, part_power_model=a.part_power_model,
                                  lapse_model=xv15_lapse_model(lapse_exponent) if a.temperature_lapse else None)
    electrical = (build_halo_electrical(motor, generator, battery, np.sqrt(d.area_wing_m2 * a.aspect_ratio_wing),
                                        requirements, a) if a.electrical_layer else None)
    redundancy = build_halo_redundancy(generator, battery, requirements, a)
    topology = build_series_hybrid(motor, generator, battery, turboshaft, gearbox, rotor, count_rotors=a.count_rotors,
                                   count_turbogenerators=a.count_turbogenerators, generator_gearbox=generator_gearbox,
                                   electrical=electrical, redundancy=redundancy)

    nacelles = Nacelles(mass_engines_kg=a.count_turbogenerators * d.mass_turboshaft_bare_kg,
                        count_engines=a.count_turbogenerators, area_wetted_m2=a.area_wetted_nacelles_m2,
                        x_m=0.0, z_m=1.2, mass_factor=factors.powerplant)
    if a.wing_weight_model == "raymer":
        wing_mass = dict(mass_factor=factors.wing)
    elif a.wing_weight_model == "afdd_tiltrotor":
        # Mass on one wing tip: rotor, motor and rotor gearbox, plus the turbogenerator and its nacelle section.
        mass_motors_tip_kg = motor.get_mass() if lanes == 1 else lanes * motor.get_mass()    # Tier 18 lanes
        mass_tip_kg = mass_rotors_kg / a.count_rotors + mass_motors_tip_kg + gearbox.get_mass()
        if electrical is not None:
            # Tier 15: inverters in the nacelle (one per lane with Tier 18 lanes).
            mass_tip_kg = mass_tip_kg + (electrical.inverter_motor.get_mass() if lanes == 1
                                         else lanes * electrical.inverter_motor.get_mass())
        if a.turbogenerators_on_wing_tips:
            mass_tip_kg = mass_tip_kg + (a.count_turbogenerators / a.count_rotors) * (
                factors.powerplant * d.mass_turboshaft_bare_kg + generator.get_mass()
                + (electrical.inverter_generator.get_mass() if electrical is not None else 0.0)
                + (generator_gearbox.get_mass() if generator_gearbox is not None else 0.0)) \
                + nacelles.get_mass_properties().mass / a.count_rotors
        else:
            # Plan 037: only the cowling of the AFDD engine section stays at the tips.
            mass_tip_kg = mass_tip_kg + factors.powerplant * afdd.mass_engine_cowling_afdd82_kg(
                a.area_wetted_nacelles_m2) / a.count_rotors
        if a.ratio_radius_gyration_pylon is not None:
            radius_gyration_pylon_m = a.ratio_radius_gyration_pylon * radius_m
        elif a.turbogenerators_on_wing_tips:
            raise ValueError("Give `ratio_radius_gyration_pylon` with the turbogenerators at the tips.")
        else:
            # Plan 038: pitch inertia of the tilting nacelle about the spindle from its components.
            mass_drive_kg = motor.get_mass() + gearbox.get_mass() + (
                electrical.inverter_motor.get_mass() if electrical is not None else 0.0)
            mass_cowling_kg = factors.powerplant * afdd.mass_engine_cowling_afdd82_kg(a.area_wetted_nacelles_m2) \
                / a.count_rotors
            inertia_pylon_kg_m2 = (mass_rotors_kg / a.count_rotors * a.length_mast_m**2
                                   + mass_drive_kg * a.offset_drive_pylon_m**2
                                   + mass_cowling_kg * a.radius_gyration_cowling_m**2)
            radius_gyration_pylon_m = np.sqrt(inertia_pylon_kg_m2 / mass_tip_kg)
        if a.ratio_depth_spar_cap is not None:
            ratio_depth_spar_cap = a.ratio_depth_spar_cap
        else:
            depths = asb.Airfoil("naca2423").local_thickness(np.array([a.fraction_chord_front_spar,
                                                                       a.fraction_chord_rear_spar]))
            ratio_depth_spar_cap = float(np.mean(depths)) / a.thickness_to_chord_wing
        materials = dict(graphite_epoxy=graphite_epoxy_wing_material, aluminium=aluminium_wing_material)
        wing_mass = dict(mass_factor=factors.wing_tiltrotor, mass_model=TiltrotorWingMassModel(
            mass_tip_kg=mass_tip_kg, radius_gyration_pylon_m=radius_gyration_pylon_m,
            ratio_depth_spar_cap=ratio_depth_spar_cap, thickness_min_torque_box_m=a.thickness_min_torque_box_m,
            speed_rotor_design_rad_s=(d.speed_rotor_wing_design_rad_s if d.speed_rotor_wing_design_rad_s is not None
                                      else speed_rotor_design_rad_s),
            width_fuselage_m=a.diameter_fuselage_m, frequency_torsion_per_rev=a.frequency_torsion_wing_per_rev,
            frequency_beam_per_rev=a.frequency_beam_wing_per_rev, frequency_chord_per_rev=a.frequency_chord_wing_per_rev,
            thickness_to_chord=a.thickness_to_chord_wing, material=materials[a.wing_material](),
            count_rotors=a.count_rotors, load_factor_jump=a.load_factor_jump, smoothing=a.smoothing_wing_tiltrotor))
    else:
        raise ValueError(f"Unknown wing weight model '{a.wing_weight_model}'.")
    aspect_ratio_wing = d.aspect_ratio_wing if d.aspect_ratio_wing is not None else a.aspect_ratio_wing
    wing = Wing(area_m2=d.area_wing_m2, aspect_ratio=aspect_ratio_wing, taper_ratio=1.0, x_le_root_m=d.x_le_wing_m,
                z_m=1.2, airfoil=asb.Airfoil("naca2423"), **wing_mass)
    x_rotor_m = d.x_le_wing_m + 0.25 * wing.chord_root_m()
    z_rotor_m = wing.z_m + a.length_mast_m
    if a.turbogenerators_on_wing_tips:
        x_turbogenerator_m, z_turbogenerator_m = x_rotor_m, wing.z_m
    else:
        x_turbogenerator_m = (x_rotor_m + a.offset_x_turbogenerators_m if a.offset_x_turbogenerators_m is not None
                              else d.x_le_wing_m + a.fraction_chord_rear_spar * wing.chord_root_m()
                              + a.length_half_turbogenerator_m)
        height_fuselage_m = a.height_fuselage_m if a.height_fuselage_m is not None else a.diameter_fuselage_m
        z_turbogenerator_m = (a.z_turbogenerators_m if a.z_turbogenerators_m is not None
                              else height_fuselage_m / 2 - a.depth_turbogenerator_below_top_m)
    locations = (InstalledInstance("turboshaft", x_m=x_turbogenerator_m, z_m=z_turbogenerator_m),
                 InstalledInstance("generator", x_m=x_turbogenerator_m, z_m=z_turbogenerator_m),
                 InstalledInstance("battery", x_m=x_rotor_m - 0.8, z_m=-0.2),
                 InstalledInstance("motor", x_m=x_rotor_m, z_m=z_rotor_m),
                 InstalledInstance("gearbox", x_m=x_rotor_m, z_m=z_rotor_m),
                 InstalledInstance("propulsor", x_m=x_rotor_m, z_m=z_rotor_m + 0.5))
    if generator_gearbox is not None:
        locations = locations + (InstalledInstance("generator_gearbox", x_m=x_turbogenerator_m,
                                                   z_m=z_turbogenerator_m),)
    if electrical is not None:
        # Inverters in the tip nacelles with their machines; nacelle feeders along the wing quarter chord;
        # protection at the bus next to the battery.
        x_bus_m, z_bus_m = x_rotor_m - 0.8, -0.2
        locations = locations + (
            InstalledInstance("inverter_motor", x_m=x_rotor_m, z_m=z_rotor_m),
            InstalledInstance("inverter_generator", x_m=x_turbogenerator_m, z_m=z_turbogenerator_m),
            InstalledInstance("cable_motor", x_m=x_rotor_m, z_m=wing.z_m),
            InstalledInstance("cable_generator", x_m=x_rotor_m, z_m=wing.z_m),
            InstalledInstance("cable_battery", x_m=x_bus_m, z_m=z_bus_m),
            InstalledInstance("protection_motor", x_m=x_bus_m, z_m=z_bus_m),
            InstalledInstance("protection_generator", x_m=x_bus_m, z_m=z_bus_m),
            InstalledInstance("protection_battery", x_m=x_bus_m, z_m=z_bus_m))
        if electrical.dcdc is not None:
            locations = locations + (InstalledInstance("dcdc", x_m=x_bus_m, z_m=z_bus_m),)
    if redundancy is not None:
        # Tier 18: string contactors with the pack; bus ties between the buses next to it.
        x_bus_m, z_bus_m = x_rotor_m - 0.8, -0.2
        if redundancy.protection_string is not None:
            locations = locations + (InstalledInstance("protection_string", x_m=x_bus_m, z_m=z_bus_m),)
        if redundancy.protection_bus_tie is not None:
            locations = locations + (InstalledInstance("protection_bus_tie", x_m=x_bus_m, z_m=z_bus_m),
                                     InstalledInstance("cable_bus_tie", x_m=x_bus_m, z_m=z_bus_m))
    cooling = None
    if a.thermal_model:
        # One ram-air cooler for every heat load, in the fuselage at the wing (the loop runs to the tips).
        cooling = InstalledCooling(RamAirHeatExchanger(
            power_rated_W=d.power_rated_heat_exchanger_W, specific_power_W_kg=a.specific_power_heat_exchanger_W_kg,
            temperature_coolant_C=a.temperature_coolant_C,
            delta_temperature_ref_C=a.delta_temperature_ref_heat_exchanger_C,
            effectiveness=a.effectiveness_heat_exchanger, pressure_drop_ref_Pa=a.pressure_drop_ref_heat_exchanger_Pa,
            efficiency_fan=a.efficiency_fan_heat_exchanger), x_m=x_rotor_m, z_m=0.0,
            sources_excluded=a.sources_excluded_heat_exchanger)
    return Aircraft(
        wing=wing,
        horizontal_tail=HorizontalTail(area_m2=d.area_horizontal_tail_m2, aspect_ratio=3.27, taper_ratio=1.0,
                                       x_le_root_m=a.x_horizontal_tail_m, z_m=0.8, airfoil=asb.Airfoil("naca0015"),
                                       mass_factor=factors.tail),
        vertical_tail=VerticalTail(area_m2=d.area_vertical_tail_m2, aspect_ratio=2.33, taper_ratio=0.6,
                                   x_le_root_m=a.x_horizontal_tail_m, z_root_m=0.8, airfoil=asb.Airfoil("naca0009"),
                                   mass_factor=factors.tail),
        fuselage=Fuselage(length_m=a.length_fuselage_m, diameter_m=a.diameter_fuselage_m,
                          height_m=a.height_fuselage_m, shape=a.shape_fuselage,
                          mass_factor=(a.mass_factor_fuselage if a.mass_factor_fuselage is not None
                                       else factors.fuselage)),
        landing_gear=LandingGear(length_main_m=3.0 * u.foot, length_nose_m=3.0 * u.foot, x_main_m=x_rotor_m + 0.6,
                                 x_nose_m=1.5, z_m=-0.9, is_retractable=True, mass_factor=factors.alighting_gear),
        systems=Systems(mass_avionics_uninstalled_kg=0.0, x_m=3.0, mass_factor=factors.flight_controls),
        powertrain=PowertrainInstallation(topology, locations, cooling=cooling),
        payload=Payload(mass_kg=requirements.mass_payload_kg, x_m=x_rotor_m),
        fuel=FuelLoad(mass_kg=d.mass_fuel_kg, x_m=x_rotor_m, z_m=wing.z_m),
        nacelles=replace(nacelles, x_m=x_turbogenerator_m, z_m=z_turbogenerator_m, length_m=a.length_nacelle_m,
                         diameter_m=a.diameter_nacelle_m, y_m=wing.span_m() / 2),
        equipment=FixedEquipment(mass_kg=a.mass_equipment_kg, x_m=3.5),
    )


def halo_mission(requirements=HaloRequirements(), velocity_cruise_m_s=None, velocity_loiter_m_s=None):
    r = requirements
    climb = ClimbSegment(0.0, r.altitude_cruise_m, 6.0, 80.0, None, "climb")
    descent = DescentSegment(r.altitude_cruise_m, 0.0, 5.0, 80.0, None, "descent")
    distance_cruise_m = r.range_m - ground_distance_m(climb) - ground_distance_m(descent)
    return Mission((
        HoverSegment(r.duration_hover_s, 0.0, None, "take-off hover"),
        climb,
        CruiseSegment(distance_cruise_m, r.altitude_cruise_m, velocity_cruise_m_s, None, "cruise"),
        LoiterSegment(r.duration_loiter_s, r.altitude_cruise_m, velocity_loiter_m_s, None, "reserve loiter"),
        descent,
        HoverSegment(r.duration_hover_s, 0.0, None, "landing hover"),
    ))


@dataclass(frozen=True)
class HaloSizingResult:
    mass_takeoff_kg: float
    mass_payload_kg: float
    mass_empty_kg: float
    mass_fuel_kg: float
    mass_fuel_burnt_kg: float
    design: HaloDesign
    energy_battery_kWh: float
    radius_rotor_m: float
    span_m: float
    disk_loading_kg_m2: float
    wing_loading_kg_m2: float
    power_rated_motor_W: float
    power_rated_generator_W: float
    velocity_cruise_m_s: float
    velocity_loiter_m_s: float
    lift_to_drag_cruise: float
    cl_cruise: float
    static_margin: float
    cn_beta_per_rad: float
    soc_end: float
    soc_after_engine_out: float
    closure_residual_kg: float
    turboshaft_mass_power_residual: float
    thermal_efficiency_turboshaft: float
    segments: tuple
    binding: tuple
    min_margin: float
    component_masses_kg: tuple
    powertrain_masses_kg: tuple
    hovers: tuple = ()                    # HoverSummary per hover point (requirement, mission, hot day)
    soc_after_hover_hot: Any = None
    # Tier 17 ("ecm" battery): per mission point and per engine-out point, time_start_s, soc_start, soc_end,
    # voltage_bus_V (interval mean), voltage_end_V, voltage_ocv_V, current_A, power_W (terminal).
    battery_trace: tuple = ()
    engine_out_trace: tuple = ()
    # Tier 20 ("afdd_tiltrotor"): wing breakdown (name, kg) and per airplane-mode point the realized wing torsion
    # and beam frequencies in per rev of that point's rotor speed.
    wing_masses_kg: tuple = ()
    whirl_flutter: tuple = ()
    # Tier 22 (plan 029): the starting point that produced this result (StartRecord; None when the caller passed
    # `initial`) and the cost per mission of `HaloAssumptions.cost_model` (CostBreakdown, numeric).
    start: Any = None
    cost: Any = None
    # Plan 035 ("units" machines): (motor, generator) unit counts, relaxed (what the units would need) and fixed.
    machine_units: Any = None
    # Tier 19 (`thermal_model`): per point (mission, engine-out, requirement and hot-day points) the heat by
    # source, heat rejected, cooling drag, fan power, required exchanger rating and machine end temperatures;
    # and the exchanger (rating, mass) plus the continuous machine losses and thermal time constants.
    thermal_trace: tuple = ()
    heat_exchanger: Any = None
    # Tier 18 (`redundancy`): lane motors per rotor (the motor figures above are per lane) and, per failure hover,
    # its label, SOC at the end, worst per-lane torque / max torque, motor end temperature, battery current, tie
    # current, its smallest margin and its binding margins.
    count_lanes_motor: int = 1
    failure_cases: tuple = ()


def count_parallel_guess(guess, assumptions):
    """Initial parallel-string count: the guess's own count, else 30. With the Tier 15 electrical layer a start
    from a constant-battery design is energy-matched instead, so packs of any series count (the bus-voltage
    options) start with the guess's energy; without the layer the earlier 30 is kept (results unchanged)."""
    if guess.count_parallel_battery is not None:
        return guess.count_parallel_battery
    if guess.energy_capacity_battery_J is None or not assumptions.electrical_layer:
        return 30.0
    string = build_halo_battery(replace(guess, count_parallel_battery=1.0), replace(assumptions, battery_model="ecm"))
    return guess.energy_capacity_battery_J / string.energy_capacity_J


def unit_machine(rubber, count_units, units):
    """Plan 035: `rubber` (a McDonald machine, which places the efficiency map) rebuilt from `count_units` whole
    units (None: the relaxed count the rubber ratings need): limits and mass are those of the units."""
    count = count_units if count_units is not None else units.count_units_relaxed(rubber)
    return replace(rubber, max_torque_Nm=count * units.torque_peak_unit_Nm,
                   power_rated_W=count * units.power_continuous_unit_W, max_speed_rad_s=units.speed_max_unit_rad_s,
                   mass_model=replace(units, count_units=count))


def machine_units_record(instances, value, assumptions):
    """Plan 035: {machine: (unit count in this solve, whole count or None if relaxed)}; None for other models."""
    if assumptions.machine_mass_model != "units":
        return None
    a = assumptions
    fixed = dict(motor=a.count_units_motor, generator=a.count_units_generator)
    return {name: (value(instances[name].component.mass_model.count_units), fixed[name])
            for name in ("motor", "generator")}


def solve_halo_sizing(requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None, verbose=False,
                      max_iter=3000, initial=None, objective="mass_takeoff", staged_start=True, stage_fallback=True):
    """Size the Halo: one coupled AeroSandbox solve from `initial`, or the Tier 22 starting-point strategy.

    `initial`: an earlier `HaloSizingResult` used as the initial guess (sensitivity studies); the problem is then
    solved once from it. `staged_start` (Tier 18): False drops the "strings_off" start for redundant aircraft.

    objective "mass_takeoff" minimizes take-off mass at the required payload; "payload" makes payload a
    variable and maximizes it (the aircraft is sized around fixed engines, plan 017); "cost" minimizes the
    cost per mission of `HaloAssumptions.cost_model` at the required payload (Tier 22).

    With no `initial`, `solve_halo_sizing_multistart` tries an ordered list of starting points, each one
    independent coupled solve, and returns the first that converges; `result.start` records which start
    succeeded and every attempt (plan 029). The order reproduces the earlier rules first (thermal-off start for
    the thermal model, plan 028; Scholz-aero start for the AeroBuildup model, plan 025; constant-battery start for the equivalent-circuit pack, plan 022; payload
    continuation, plan 026), then perturbed designs, then the generic guess.
    """
    a = assumptions
    if (a.machine_mass_model == "units" and a.integer_units
            and (a.count_units_motor is None or a.count_units_generator is None)):
        # Plan 035: whole units. Solve with the relaxed unit counts, round each up, re-solve with the counts fixed
        # (two explicit solves; the relaxed one is the start of the integer one).
        relaxed = solve_halo_sizing(requirements, replace(a, integer_units=False), factors, verbose, max_iter,
                                    initial, objective, staged_start, stage_fallback)
        counts = {name: int(np.ceil(n - 1e-3)) for name, (n, _) in relaxed.machine_units.items()}
        fixed = replace(a, count_units_motor=a.count_units_motor or counts["motor"],
                        count_units_generator=a.count_units_generator or counts["generator"])
        return solve_halo_sizing_once(requirements, fixed, factors, verbose, max_iter, relaxed, objective)
    if assumptions.redundancy:
        failure_hover_cases(assumptions)              # Tier 18: reject impossible failure cases before any solve
    stages = assumptions.gear_stage_model
    if initial is None and stage_fallback and assumptions.gearbox_stages and stages.staircase:
        # Plan 033: the staircase stage count is a landscape of plateaus and steep steps, with a local optimum per
        # stage count. Two starting points, the strategy's own and the relaxed (softplus) stage-count solution,
        # each one complete solve; the better result is kept.
        candidates = []
        try:
            candidates.append(solve_halo_sizing(requirements, assumptions, factors, verbose, max_iter, None,
                                                objective, staged_start, stage_fallback=False))
        except RuntimeError:
            pass
        try:
            relaxed = solve_halo_sizing(requirements, replace(assumptions, gear_stage_model=replace(
                stages, staircase=False)), factors, max_iter=max_iter, objective=objective)
            candidates.append(solve_halo_sizing_once(requirements, assumptions, factors, verbose, max_iter, relaxed,
                                                     objective))
        except RuntimeError:
            pass
        if not candidates:
            raise RuntimeError("Halo sizing with gearbox stages failed from both starting points.")
        return min(candidates, key=lambda c: objective_value(c, objective))
    if initial is None:
        starts = None
        if assumptions.redundancy and not staged_start:
            starts = tuple(s for s in default_starts(requirements, assumptions, objective) if s != "strings_off")
        return solve_halo_sizing_multistart(requirements, assumptions, factors, verbose, max_iter, objective, starts)
    if assumptions.redundancy and staged_start and assumptions.count_strings_battery > 1:
        # Tier 18 (plan 032) with a caller's start: if the redundant problem with battery strings fails from it,
        # the same problem without the strings and their cases is solved from it and used as the start instead.
        try:
            return solve_halo_sizing_once(requirements, assumptions, factors, verbose, max_iter, initial, objective)
        except RuntimeError:
            pass
        without_strings = replace(assumptions, count_strings_battery=1, failure_string_out=False,
                                  failure_string_out_engine_out=False)
        initial = solve_halo_sizing_once(requirements, without_strings, factors, False, max_iter, initial, objective)
    return solve_halo_sizing_once(requirements, assumptions, factors, verbose, max_iter, initial, objective)


def solve_halo_sizing_once(requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                           verbose=False, max_iter=3000, initial=None, objective="mass_takeoff"):
    """One coupled solve from `initial` (None: the generic guess). Raises RuntimeError if IPOPT fails."""
    if objective not in ("mass_takeoff", "payload", "cost"):
        raise ValueError(f"Unknown objective '{objective}'.")
    factors = factors if factors is not None else calibration_factors()
    lapse_exponent = fit_lapse_exponent()
    a, r = assumptions, requirements
    aerodynamics = build_halo_aerodynamics(r, a)
    longitudinal, directional = LongitudinalStability(), DirectionalStability()
    opti = asb.Opti()

    # ---- Design variables -------------------------------------------------------------
    guess = initial.design if initial is not None else HaloDesign(
        x_le_wing_m=5.5, area_wing_m2=18.0, area_horizontal_tail_m2=4.7, area_vertical_tail_m2=4.7,
        torque_peak_motor_Nm=2500.0, torque_peak_generator_Nm=2000.0, power_rated_turboshaft_W=1.0e6,
        mass_turboshaft_bare_kg=230.0, power_max_discharge_battery_W=1.0e6, energy_capacity_battery_J=1.8e8,
        area_disk_m2=40.0, mass_fuel_kg=1000.0)
    mass_takeoff_kg = opti.variable(init_guess=initial.mass_takeoff_kg if initial else 6000.0, scale=1000.0,
                                    lower_bound=1000.0)
    lanes = count_lanes_motor(assumptions)
    if initial is not None and initial.count_lanes_motor != lanes:
        # Tier 18: a start with a different lane count keeps its total motor torque per rotor.
        guess = replace(guess, torque_peak_motor_Nm=guess.torque_peak_motor_Nm * initial.count_lanes_motor / lanes)
    is_ecm = a.battery_model == "ecm"
    design = HaloDesign(
        x_le_wing_m=opti.variable(init_guess=guess.x_le_wing_m, lower_bound=3.0, upper_bound=8.5),
        area_wing_m2=opti.variable(init_guess=guess.area_wing_m2, scale=10.0, lower_bound=5.0, upper_bound=60.0),
        area_horizontal_tail_m2=opti.variable(init_guess=guess.area_horizontal_tail_m2, lower_bound=0.5),
        area_vertical_tail_m2=opti.variable(init_guess=guess.area_vertical_tail_m2, lower_bound=0.5),
        torque_peak_motor_Nm=opti.variable(init_guess=guess.torque_peak_motor_Nm, scale=1000.0, lower_bound=100.0),
        torque_peak_generator_Nm=opti.variable(init_guess=guess.torque_peak_generator_Nm, scale=1000.0,
                                               lower_bound=100.0),
        # Tier 19: the cooler's rating and the drive's own rating (the motor's becomes continuous).
        power_rated_heat_exchanger_W=opti.variable(
            init_guess=guess.power_rated_heat_exchanger_W if guess.power_rated_heat_exchanger_W is not None
            else 3e5, scale=1e5, lower_bound=1e3) if a.thermal_model else None,
        power_rated_gearbox_W=opti.variable(
            init_guess=guess.power_rated_gearbox_W if guess.power_rated_gearbox_W is not None
            else (initial.power_rated_motor_W if initial is not None else 7e5), scale=1e6,
            lower_bound=1e4) if a.thermal_model else None,
        power_rated_turboshaft_W=(a.power_rated_turboshaft_fixed_W if a.power_rated_turboshaft_fixed_W is not None
                                  else opti.variable(init_guess=guess.power_rated_turboshaft_W, scale=1e6,
                                                     lower_bound=5e4)),
        mass_turboshaft_bare_kg=opti.variable(init_guess=guess.mass_turboshaft_bare_kg, scale=100.0, lower_bound=10.0),
        # Tier 17 "ecm": the pack (count_parallel_battery) sets energy and power; they are filled in below.
        power_max_discharge_battery_W=None if is_ecm else opti.variable(
            init_guess=guess.power_max_discharge_battery_W, scale=1e6, lower_bound=1e4),
        energy_capacity_battery_J=None if is_ecm else opti.variable(
            init_guess=guess.energy_capacity_battery_J, scale=1e8, lower_bound=1e6),
        count_parallel_battery=opti.variable(
            init_guess=count_parallel_guess(guess, a), scale=10.0, lower_bound=1.0) if is_ecm else None,
        area_disk_m2=opti.variable(init_guess=guess.area_disk_m2, scale=10.0, lower_bound=5.0, upper_bound=120.0),
        mass_fuel_kg=opti.variable(init_guess=guess.mass_fuel_kg, scale=500.0, lower_bound=0.0),
    )
    if a.machine_mass_by_torque:
        # Tier 13: motor speed, rotor gear ratio and (with a step-up gearbox) generator speed are trades.
        design = replace(design, speed_peak_motor_rad_s=opti.variable(
            init_guess=guess.speed_peak_motor_rad_s if guess.speed_peak_motor_rad_s is not None
            else (60.0 if a.direct_drive_rotor else 400.0), scale=100.0, lower_bound=20.0, upper_bound=2000.0))
        if not a.direct_drive_rotor:
            design = replace(design, reduction_ratio=opti.variable(
                init_guess=min(guess.reduction_ratio if guess.reduction_ratio is not None else 7.0,
                               a.reduction_ratio_max),
                lower_bound=1.5, upper_bound=a.reduction_ratio_max))
        if a.generator_step_up:
            design = replace(design, speed_peak_generator_rad_s=opti.variable(
                init_guess=guess.speed_peak_generator_rad_s if guess.speed_peak_generator_rad_s is not None else 600.0,
                scale=100.0, lower_bound=a.speed_output_turboshaft_rad_s, upper_bound=2500.0))
    if a.rotor_speed_physics:
        # Tier 12: solidity and design tip speed are trades; hover tip Mach bounded at sea level.
        speed_sound_sea_level_m_s = asb.Atmosphere(altitude=0).speed_of_sound()
        design = replace(design,
                         solidity=opti.variable(init_guess=guess.solidity if guess.solidity is not None else 0.10,
                                                lower_bound=0.06, upper_bound=0.14),
                         speed_tip_m_s=opti.variable(
                             init_guess=guess.speed_tip_m_s if guess.speed_tip_m_s is not None else 220.0, scale=100.0,
                             lower_bound=150.0, upper_bound=a.mach_tip_hover_max * speed_sound_sea_level_m_s))
    is_afdd_wing = a.wing_weight_model == "afdd_tiltrotor"
    if is_afdd_wing:
        # Tier 20: the rotor speed the wing's frequencies are placed against; airplane-mode points may not exceed it.
        design = replace(design, speed_rotor_wing_design_rad_s=opti.variable(
            init_guess=guess.speed_rotor_wing_design_rad_s if guess.speed_rotor_wing_design_rad_s is not None
            else 40.0, scale=10.0, lower_bound=5.0, upper_bound=200.0))
    # Tier 22 freed trades (plan 029).
    if a.free_altitude_cruise:
        low, high = a.bounds_altitude_cruise_m
        design = replace(design, altitude_cruise_m=opti.variable(
            init_guess=guess.altitude_cruise_m if guess.altitude_cruise_m is not None else r.altitude_cruise_m,
            scale=1000.0, lower_bound=low, upper_bound=high))
    if a.free_aspect_ratio_wing:
        low, high = a.bounds_aspect_ratio_wing
        design = replace(design, aspect_ratio_wing=opti.variable(
            init_guess=guess.aspect_ratio_wing if guess.aspect_ratio_wing is not None else a.aspect_ratio_wing,
            lower_bound=low, upper_bound=high))
    if a.free_soc_reserve:
        low, high = a.bounds_soc_reserve
        design = replace(design, soc_reserve=opti.variable(
            init_guess=guess.soc_reserve if guess.soc_reserve is not None else soc_minimum,
            lower_bound=low, upper_bound=high))
    soc_reserve = design.soc_reserve if design.soc_reserve is not None else soc_minimum
    altitude_cruise_m = design.altitude_cruise_m if design.altitude_cruise_m is not None else r.altitude_cruise_m
    mission = halo_mission(replace(r, altitude_cruise_m=altitude_cruise_m), velocity_cruise_m_s=opti.variable(
        init_guess=initial.velocity_cruise_m_s if initial else 110.0, scale=50.0, lower_bound=60.0,
        upper_bound=r.velocity_max_m_s), velocity_loiter_m_s=opti.variable(
        init_guess=initial.velocity_loiter_m_s if initial else 80.0, scale=50.0, lower_bound=55.0,
        upper_bound=r.velocity_max_m_s))

    # ---- Aircraft, mission, requirements ----------------------------------------------
    if objective == "payload":
        mass_payload_kg = opti.variable(init_guess=initial.mass_payload_kg if initial else 600.0, scale=100.0,
                                        lower_bound=0.0)
        r = replace(r, mass_payload_kg=mass_payload_kg)
    aircraft = build_halo_aircraft(design, r, a, factors, lapse_exponent)
    instances = aircraft.powertrain.topology.instances
    battery = instances["battery"].component
    if is_ecm:
        design = replace(design, energy_capacity_battery_J=battery.energy_capacity_J,
                         power_max_discharge_battery_W=battery.power_max_discharge_W)
    thermal = a.thermal_model
    flown = build_mission(opti, aircraft, aerodynamics, mission, mass_takeoff_kg, soc_take_off,
                          hybridization_electric_min=a.hybridization_electric_min,
                          # The engine-out reserve is assessed from the SOC floor, so hold it at every segment.
                          soc_floor=soc_reserve if a.soc_floor_every_segment else None,
                          **(dict(subsegments=a.subsegments_mission) if is_ecm else {}),
                          # Tier 19: machines start at the coolant temperature and carry their history.
                          **(dict(thermal_start="coolant") if thermal else {}))
    # Tier 19: the hover requirement is a 60 s hover from a cold start; airplane-mode requirements are steady.
    cold_C = coolant_temperatures_C(aircraft.powertrain) if thermal else None
    requirement_points = [build_flight_point(opti, aircraft, aerodynamics, c, mass_takeoff_kg,
                                             **(dict(duration_s=r.duration_hover_s, temperature_start_C=cold_C)
                                                if thermal and c.mode == "hover" else {}))
                          for c in r.requirement_set().flight_conditions()]
    take_off = flown.segments[0]
    # Tier 19: the engine fails at the end of the take-off hover (at MTOM); the machines are that warm.
    take_off_end_C = (take_off.subsegments or (take_off,))[-1].point.thermal.temperatures_end_C if thermal else None
    cruise = next(s for s in flown.segments if isinstance(s.segment, CruiseSegment))
    loiter = next(s for s in flown.segments if isinstance(s.segment, LoiterSegment))
    # Engine-out hover: one turbogenerator plus the battery, at MTOM, from the SOC floor.
    if is_ecm:
        # Tier 17: sub-divided so the sag at the low-SOC end is seen; polarization already developed (steady)
        # when the engine fails; SOC may fall to the emergency floor.
        reserve = build_mission(opti, aircraft, aerodynamics, Mission((HoverSegment(
            r.duration_engine_out_hover_s, 0.0, None, "engine-out hover",
            active_generator_count=a.count_turbogenerators - 1),)), mass_takeoff_kg, soc_reserve,
            soc_floor=soc_emergency_floor, subsegments=a.subsegments_engine_out, polarization_start="steady",
            **(dict(thermal_start=take_off_end_C) if thermal else {}))
        engine_out_points = reserve.segments[0].subsegments or reserve.segments
        engine_out = engine_out_points[0].point
        engine_out_margins = reserve.margins
        soc_after_reserve = reserve.soc_end
    else:
        engine_out = build_flight_point(opti, aircraft, aerodynamics, FlightCondition(
            mode="hover", altitude_m=0.0, soc=soc_reserve, active_generator_count=a.count_turbogenerators - 1,
            label="engine-out hover"), mass_takeoff_kg,
            **(dict(duration_s=r.duration_engine_out_hover_s, temperature_start_C=take_off_end_C) if thermal else {}))
        engine_out_points = ()
        engine_out_margins = engine_out.margins
        soc_after_reserve = soc_reserve - (engine_out.battery.power_chemical_W * r.duration_engine_out_hover_s
                                           / design.energy_capacity_battery_J)
    # Tier 16: hot/high OGE hover at the destination, at the mission's end mass and end SOC.
    hover_hot = None
    if r.hover_hot_day:
        hover_hot = build_flight_point(opti, aircraft, aerodynamics, FlightCondition(
            mode="hover", altitude_m=r.altitude_hover_hot_m, thrust_to_weight=r.thrust_to_weight_hover_hot,
            soc=flown.soc_end, temperature_offset_K=temperature_offset_K(r.altitude_hover_hot_m,
                                                                         r.temperature_hover_hot_K),
            label="hot-day hover"), flown.mass_end_kg,
            **(dict(duration_s=r.duration_hover_hot_s, temperature_start_C=flown.temperatures_end_C) if thermal
               else {}))
        soc_after_hover_hot = flown.soc_end - (hover_hot.battery.power_chemical_W * r.duration_hover_hot_s
                                               / design.energy_capacity_battery_J)
    # Tier 18: failure hovers, each a degraded state of the same aircraft (one more coupled point, no loop).
    failures = halo_failure_hovers(opti, aircraft, aerodynamics, mass_takeoff_kg, r, a,
                                   take_off_end_C if thermal else None) if a.redundancy else ()

    # ---- Mass closure ------------------------------------------------------------------
    condition = StructuralDesignCondition(mass_design_kg=mass_takeoff_kg, load_factor_ultimate=a.load_factor_ultimate,
                                          velocity_cruise_m_s=cruise.segment.velocity_m_s,
                                          altitude_cruise_m=altitude_cruise_m,
                                          lift_to_drag_cruise=cruise.point.aero.lift_to_drag)
    breakdown = aircraft.get_mass_breakdown(condition)
    total = breakdown.total()
    x_cg_m = total.x_cg

    # ---- Stability, geometry, stall -----------------------------------------------------
    static_margin = longitudinal.static_margin(aircraft, aerodynamics, x_cg_m, 80.0, 1000.0)
    cn_beta_per_rad = directional.cn_beta_per_rad(aircraft, aerodynamics, x_cg_m, 80.0, 1000.0)
    radius_m = instances["propulsor"].component.radius_m()
    span_m = aircraft.wing.span_m()
    density_sea_level_kg_m3 = asb.Atmosphere(altitude=0).density()
    lift_stall_N = 0.5 * density_sea_level_kg_m3 * r.velocity_stall_max_m_s**2 * design.area_wing_m2 * r.cl_max
    weight_N = mass_takeoff_kg * acceleration_gravity_m_s2

    design_margins = (
        margin_above("static_margin", static_margin, 0.10),
        margin_above("cn_beta_per_rad", cn_beta_per_rad, 0.06),
        margin_above("soc_end", flown.soc_end, soc_reserve),
        margin_above("soc_after_engine_out_hover", soc_after_reserve, soc_emergency_floor),
        margin_above("stall_lift_at_120kt", lift_stall_N, weight_N),
        margin_below("rotor_radius_m", radius_m, (span_m - a.diameter_fuselage_m) / 2 - a.clearance_rotor_fuselage_m),
    )
    all_margins = (design_margins + flown.margins + engine_out_margins
                   + tuple(m for p in requirement_points for m in p.margins)
                   + tuple(m for _, failure in failures for m in failure.margins))
    whirl_points, wing_masses = [], None
    if is_afdd_wing:
        # Tier 20 reduced-order whirl flutter: at every airplane-mode point the realized wing torsion and beam
        # frequencies, in per rev of that point's rotor speed, stay at or above the required placement.
        wing_masses = aircraft.wing.mass_model.masses(aircraft.wing, condition)
        whirl_points = [p for p in ([s.point for seg in flown.segments for s in (seg.subsegments or (seg,))]
                                    + list(requirement_points)) if p.condition.mode == "airplane"]
        for p in whirl_points:
            torsion_per_rev, beam_per_rev, _ = TiltrotorWingMassModel.frequency_per_rev(wing_masses,
                                                                                         p.speed_rotor_rad_s)
            all_margins += (margin_above(f"whirl flutter torsion per rev ({p.condition.label})", torsion_per_rev,
                                         a.frequency_torsion_wing_per_rev),
                            margin_above(f"whirl flutter beam per rev ({p.condition.label})", beam_per_rev,
                                         a.frequency_beam_wing_per_rev))
    if hover_hot is not None:
        # An alternative contingency to the engine-out reserve: from the end SOC down to the emergency floor.
        all_margins += hover_hot.margins + (
            margin_above("soc_after_hot_day_hover", soc_after_hover_hot, soc_emergency_floor),)

    opti.subject_to([
        mass_takeoff_kg / total.mass == 1,                                           # mass closure
        x_cg_m == aircraft.wing.x_le_root_m + 0.25 * aircraft.wing.chord_root_m(),   # hover pitch trim
        design.mass_fuel_kg == fuel_factor * flown.mass_fuel_burnt_kg,               # fuel with reserve
        power_turboshaft(design.mass_turboshaft_bare_kg) / design.power_rated_turboshaft_W == 1,  # engine regression
    ])
    opti.subject_to([m.value >= 0 for m in all_margins])
    # ---- Cost per mission (Tier 22) -----------------------------------------------------------
    motor_component, generator_component = instances["motor"].component, instances["generator"].component
    turboshaft_component = instances["turboshaft"].component
    mass_airframe_kg = (breakdown.mass_empty_kg() - battery.get_mass()
                        - a.count_rotors * motor_component.get_mass()
                        - a.count_turbogenerators * (generator_component.get_mass() + turboshaft_component.get_mass()))
    energies_point_J = [p.energy_battery_chemical_J for s in flown.segments for p in (s.subsegments or (s,))]
    softness_energy_J = 1e-3 * design.energy_capacity_battery_J   # smooth max(E, 0): charge is not throughput
    cost = a.cost_model.evaluate(
        mass_airframe_kg=mass_airframe_kg,
        power_rated_machines_W=(a.count_rotors * motor_component.power_rated_W
                                + a.count_turbogenerators * generator_component.power_rated_W),
        power_rated_turboshafts_W=a.count_turbogenerators * design.power_rated_turboshaft_W,
        energy_capacity_battery_J=design.energy_capacity_battery_J,
        mass_fuel_burnt_kg=flown.mass_fuel_burnt_kg,
        energy_discharge_battery_J=sum(np.softmax(e, 0.0, softness=softness_energy_J) for e in energies_point_J),
        energy_recharge_ground_J=(soc_take_off - flown.soc_end) * design.energy_capacity_battery_J,
        duration_mission_s=flown.duration_s)
    if objective == "payload":
        opti.minimize(-r.mass_payload_kg / 100)
    elif objective == "cost":
        opti.minimize(cost.per_mission_usd / scale_objective_cost_usd)
    else:
        opti.minimize(mass_takeoff_kg / 1000)

    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    value = lambda expression: float(solution.value(expression))
    report = margin_report(all_margins, solution.value)
    altitude_cruise_value_m = value(altitude_cruise_m)
    altitudes = [(0, 0), (0, altitude_cruise_value_m), (altitude_cruise_value_m,) * 2, (altitude_cruise_value_m,) * 2,
                 (altitude_cruise_value_m, 0), (0, 0)]
    return HaloSizingResult(
        mass_takeoff_kg=value(mass_takeoff_kg), mass_payload_kg=value(r.mass_payload_kg),
        mass_empty_kg=value(breakdown.mass_empty_kg()),
        mass_fuel_kg=value(design.mass_fuel_kg), mass_fuel_burnt_kg=value(flown.mass_fuel_burnt_kg),
        design=HaloDesign(**{f.name: value(getattr(design, f.name)) if getattr(design, f.name) is not None else None
                             for f in fields(HaloDesign)}),
        energy_battery_kWh=value(design.energy_capacity_battery_J) / 3.6e6,
        radius_rotor_m=value(radius_m), span_m=value(span_m),
        disk_loading_kg_m2=value(mass_takeoff_kg / (a.count_rotors * design.area_disk_m2)),
        wing_loading_kg_m2=value(mass_takeoff_kg / design.area_wing_m2),
        power_rated_motor_W=value(instances["motor"].component.power_rated_W),
        power_rated_generator_W=value(instances["generator"].component.power_rated_W),
        velocity_cruise_m_s=value(cruise.segment.velocity_m_s), velocity_loiter_m_s=value(loiter.segment.velocity_m_s),
        lift_to_drag_cruise=value(cruise.point.aero.lift_to_drag), cl_cruise=value(cruise.point.aero.cl),
        static_margin=value(static_margin), cn_beta_per_rad=value(cn_beta_per_rad), soc_end=value(flown.soc_end),
        soc_after_engine_out=value(soc_after_reserve), closure_residual_kg=value(mass_takeoff_kg - total.mass),
        turboshaft_mass_power_residual=value(power_turboshaft(design.mass_turboshaft_bare_kg)
                                             / design.power_rated_turboshaft_W - 1),
        thermal_efficiency_turboshaft=value(thermal_efficiency_turboshaft(design.mass_turboshaft_bare_kg)),
        segments=tuple(dict(label=s.segment.label, duration_s=value(s.duration_s), fuel_kg=value(s.mass_fuel_burnt_kg),
                            battery_kWh=value(s.energy_battery_chemical_J) / 3.6e6, soc_end=value(s.soc_end),
                            hybridization=value(s.point.hybridization_electric),
                            power_rotors_W=value(a.count_rotors * s.point.power_shaft_rotor_W),
                            speed_motor_rad_s=value(s.point.speed_motor_rad_s),
                            torque_motor_Nm=value(s.point.torque_motor_Nm),
                            altitude_start_m=h[0], altitude_end_m=h[1],
                            **(electrical_summary(s.point.electrical, value) if s.point.electrical is not None
                               else {}))
                       for s, h in zip(flown.segments, altitudes)),
        binding=tuple(e.label for e in report if abs(float(e.value)) < 1e-4),
        min_margin=float(report[0].value),
        component_masses_kg=tuple((f.name, value(getattr(breakdown, f.name).mass)) for f in fields(MassBreakdown)),
        powertrain_masses_kg=tuple((item.instance_name, value(item.mass_properties.mass))
                                   for item in aircraft.powertrain.get_instance_mass_properties()),
        hovers=tuple(hover_summary(point, value, instances, a) for point in
                     [p for p in requirement_points if p.condition.mode == "hover"]
                     + [s.point for s in flown.segments if isinstance(s.segment, HoverSegment)]
                     + ([hover_hot] if hover_hot is not None else [])),
        soc_after_hover_hot=value(soc_after_hover_hot) if hover_hot is not None else None,
        battery_trace=battery_trace(tuple(p for s in flown.segments for p in (s.subsegments or (s,))), value)
        if is_ecm else (),
        engine_out_trace=battery_trace(engine_out_points, value),
        wing_masses_kg=tuple((f.name, value(getattr(wing_masses, f.name))) for f in fields(wing_masses)
                             if f.name.startswith("mass_")) if wing_masses is not None else (),
        whirl_flutter=tuple(dict(label=p.condition.label, speed_rotor_rad_s=value(p.speed_rotor_rad_s),
                                 torsion_per_rev=value(wing_masses.frequency_torsion_rad_s / p.speed_rotor_rad_s),
                                 beam_per_rev=value(wing_masses.frequency_beam_rad_s / p.speed_rotor_rad_s))
                            for p in whirl_points),
        cost=CostBreakdown(**{f.name: value(getattr(cost, f.name)) for f in fields(CostBreakdown)}),
        machine_units=machine_units_record(instances, value, a),
        thermal_trace=thermal_trace(
            tuple(p for s in flown.segments for p in (s.subsegments or (s,))) + tuple(engine_out_points)
            + tuple(p for _, failure in failures for s in failure.segments for p in (s.subsegments or (s,))),
            requirement_points + ([] if is_ecm else [engine_out]) + ([hover_hot] if hover_hot is not None else []),
            value) if thermal else (),
        heat_exchanger=heat_exchanger_summary(aircraft.powertrain, value) if thermal else None,
        count_lanes_motor=lanes,
        failure_cases=tuple(failure_summary(label, failure, instances, value) for label, failure in failures),
    )


def failure_hover_cases(assumptions):
    """Tier 18: ((label, HoverSegment counts), ...) for the failure hovers switched on; a case with nothing to
    fail onto (e.g. a lane out with one lane per rotor) raises."""
    a = assumptions
    lanes, engines = a.count_lanes_motor, a.count_turbogenerators
    cases = []
    # With one lane per rotor on each bus, a bus out is the lane-out state plus the tie (more demand, more heat), so
    # it covers the lane out; the duplicate point would only repeat its binding constraints (degenerate for IPOPT).
    lane_out_covered = a.failure_bus_out and a.count_buses > 1 and lanes // a.count_buses == 1
    if a.failure_lane_out and not (lane_out_covered and lanes > 1):
        cases.append(("lane-out hover", dict(active_lane_count=lanes - 1)))
    if a.failure_bus_out:
        cases.append(("bus-out hover", dict(count_buses_failed=1)))
    if a.failure_string_out:
        cases.append(("string-out hover", dict(active_battery_string_count=a.count_strings_battery - 1)))
    if a.failure_string_out_engine_out:
        cases.append(("string-out engine-out hover", dict(active_battery_string_count=a.count_strings_battery - 1,
                                                          active_generator_count=engines - 1)))
    if a.failure_lane_out_engine_out:
        cases.append(("lane-out engine-out hover", dict(active_lane_count=lanes - 1,
                                                        active_generator_count=engines - 1)))
    needs = {"lane-out": lanes > 1, "bus-out": a.count_buses > 1, "string-out": a.count_strings_battery > 1}
    for label, _ in cases:
        for prefix, available in needs.items():
            if label.startswith(prefix) and not available:
                raise ValueError(f"The '{label}' case needs more than one unit to fail onto; set it False or raise "
                                 f"the count.")
    return tuple(cases)


def halo_failure_hovers(opti, aircraft, aerodynamics, mass_takeoff_kg, requirements, assumptions,
                        temperature_start_C=None):
    """Tier 18 failure hovers: ((label, MissionResult), ...), each a 60 s hover at MTOM from the SOC floor to the
    emergency floor in a degraded state (fewer lanes, a bus out, a string isolated, optionally an engine out too).
    The thermal history starts at `temperature_start_C` (the end of the take-off hover) when given."""
    r, a = requirements, assumptions
    cases = failure_hover_cases(a)
    is_ecm = a.battery_model == "ecm"
    failures = []
    for label, counts in cases:
        failure = build_mission(opti, aircraft, aerodynamics, Mission((HoverSegment(
            r.duration_engine_out_hover_s, 0.0, None, label, **counts),)), mass_takeoff_kg, soc_minimum,
            soc_floor=soc_emergency_floor, polarization_start="steady",
            **(dict(subsegments=a.subsegments_engine_out) if is_ecm else {}),
            **(dict(thermal_start=temperature_start_C) if temperature_start_C is not None else {}))
        failures.append((label, failure))
    return tuple(failures)


def failure_summary(label, failure, instances, value):
    """Numeric summary of one Tier 18 failure hover (see `HaloSizingResult.failure_cases`)."""
    points = [s.point for seg in failure.segments for s in (seg.subsegments or (seg,))]
    motor = instances["motor"].component
    margins = [(m.label, value(m.value)) for m in failure.margins]
    temperatures = [p.thermal.temperatures_end_C.get("motor") for p in points if p.thermal is not None]
    return dict(
        label=label, soc_end=value(failure.soc_end),
        count_lanes_active=points[0].redundancy.count_lanes_active,
        count_strings_active=points[0].redundancy.count_strings_active,
        count_buses_failed=points[0].redundancy.count_buses_failed,
        count_generators_active=points[0].condition.active_generator_count or instances["generator"].count,
        torque_lane_to_max=max(value(p.torque_motor_Nm / motor.max_torque_Nm) for p in points),
        temperature_end_motor_C=(max(value(t) for t in temperatures) if temperatures and temperatures[0] is not None
                                 else None),
        current_battery_max_A=max(value(p.battery.current_A) for p in points),
        current_tie_A=max(value(p.redundancy.current_tie_A) for p in points),
        power_loss_tie_W=max(value(p.redundancy.power_loss_tie_W) for p in points),
        min_margin=min(v for _, v in margins),
        binding=tuple(name for name, v in margins if abs(v) < 1e-4))


@dataclass(frozen=True)
class HoverSummary:
    """A solved hover point. `battery_share_min` is the battery share needed with the turbines at full power
    available (the turbine-limited share, generators at the point's efficiency); the solved `hybridization` may
    be higher wherever the split is free and not binding."""
    label: str
    altitude_m: float
    temperature_offset_K: float
    mass_kg: float
    soc: float
    hybridization: float
    battery_share_min: float
    power_rotors_W: float
    power_available_turboshafts_W: float
    power_battery_W: float
    blade_loading: Any = None             # CT / sigma (momentum + profile rotor only)


def hover_summary(point, value, instances, assumptions):
    c = point.condition
    atmosphere = asb.Atmosphere(altitude=c.altitude_m, temperature_deviation=c.temperature_offset_K)
    count_generators = c.active_generator_count or assumptions.count_turbogenerators
    available_W = count_generators * instances["turboshaft"].component.power_available_W(atmosphere)
    power_generator_bus_W = (point.electrical.power_generator_bus_W if point.electrical is not None
                             else point.generator.power_electric_W)
    efficiency_generator = power_generator_bus_W / point.engine.power_shaft_W
    rotor = instances["propulsor"].component
    blade_loading = None
    if getattr(rotor, "solidity", None) is not None:
        blade_loading = value(point.thrust_per_rotor_N / (atmosphere.density() * rotor.area_disk_m2 * rotor.solidity
                                                          * (point.speed_rotor_rad_s * rotor.radius_m())**2))
    return HoverSummary(
        label=c.label, altitude_m=value(c.altitude_m), temperature_offset_K=value(c.temperature_offset_K),
        mass_kg=value(point.weight_N / acceleration_gravity_m_s2), soc=value(c.soc),
        hybridization=value(point.hybridization_electric),
        battery_share_min=value(1 - available_W * efficiency_generator / point.power_electric_motors_W),
        power_rotors_W=value(assumptions.count_rotors * point.power_shaft_rotor_W),
        power_available_turboshafts_W=value(available_W), power_battery_W=value(point.power_battery_W),
        blade_loading=blade_loading)


def electrical_summary(electrical, value):
    """Tier 15 per-point numbers: bus voltage and the electrical-layer losses (W)."""
    return dict(voltage_bus_V=value(electrical.voltage_bus_V),
                power_loss_inverters_W=value(electrical.power_loss_inverters_W),
                power_loss_cables_W=value(electrical.power_loss_cables_W),
                power_loss_protection_W=value(electrical.power_loss_protection_W),
                power_loss_dcdc_W=value(electrical.power_loss_dcdc_W),
                power_loss_electrical_W=value(electrical.power_loss_total_W))


def thermal_trace(segment_points, other_points, value):
    """Numeric thermal state per point (Tier 19): segment results (with durations) then single flight points."""
    rows = []
    for duration_s, p in ([(s.duration_s, s.point) for s in segment_points] + [(None, p) for p in other_points]):
        t = p.thermal
        rows.append(dict(
            label=p.condition.label, mode=p.condition.mode,
            duration_s=value(duration_s) if duration_s is not None else None,
            heat_W={load.source: value(load.total_W()) for load in t.heat_loads},
            power_heat_W=value(t.power_heat_W), drag_cooling_N=value(t.cooling.drag_N),
            power_fan_W=value(t.cooling.power_fan_W), power_heat_equivalent_W=value(t.power_heat_equivalent_W),
            mass_flow_air_kg_s=value(t.cooling.mass_flow_air_kg_s),
            delta_temperature_C=value(t.cooling.delta_temperature_C),
            power_heat_end_W=value(t.power_heat_end_W),
            power_to_coolant_W={name: value(power_W) for name, power_W in t.power_to_coolant_W.items()},
            temperatures_end_C={name: value(temperature) for name, temperature in t.temperatures_end_C.items()},
            power_shaft_motor_W=value(p.speed_motor_rad_s * p.torque_motor_Nm)))
    return tuple(rows)


def heat_exchanger_summary(powertrain, value):
    exchanger = powertrain.cooling.heat_exchanger
    machines = {}
    for name in ("motor", "generator", "battery"):
        component = powertrain.topology.instances[name].component
        parameters = thermal_parameters(component)
        if name != "battery":
            rating_W = component.power_rated_W
        elif isinstance(component, EquivalentCircuitBattery):
            rating_W = component.power_max_discharge_W
        else:
            rating_W = component.max_discharge_power_W
        machines[name] = dict(power_rated_W=value(rating_W), mass_kg=value(component.get_mass()),
                              power_loss_continuous_W=value(parameters.power_loss_continuous_W),
                              time_constant_s=value(parameters.capacity_J_K * parameters.resistance_K_W))
    return dict(power_rated_W=value(exchanger.power_rated_W), mass_kg=value(exchanger.get_mass()),
                power_rated_gearbox_W=value(powertrain.topology.instances["gearbox"].component.power_rated_W),
                machines=machines)


def battery_trace(points, value):
    """Numeric battery state per mission point (Tier 17 dashboards and notebooks)."""
    rows, time_s = [], 0.0
    for p in points:
        b = p.point.battery
        rows.append(dict(label=p.point.condition.label, time_start_s=time_s, duration_s=value(p.duration_s),
                         soc_start=value(p.soc_start), soc_end=value(p.soc_end), voltage_bus_V=value(b.voltage_V),
                         voltage_end_V=value(b.voltage_end_V), voltage_ocv_V=value(b.voltage_open_circuit_V),
                         current_A=value(b.current_A), power_W=value(b.power_electric_W)))
        time_s += rows[-1]["duration_s"]
    return tuple(rows)


# ---- Tier 22 (plan 029): starting-point strategy ---------------------------------------------------------------
# Every candidate start is one independent coupled solve of the target problem from an initial guess; a candidate
# may first solve a simpler precursor problem (other aerodynamics, other battery, lower payload) to obtain that
# guess. Nothing iterates to a fixed point: the list is finite and ordered, and the record says what happened.
start_labels = ("caller", "redundancy_off", "strings_off", "electrical_off", "thermal_off", "mass_objective", "scholz_aero", "constant_battery",
                "payload_continuation", "perturbed_low", "perturbed_high", "generic")
fraction_payload_continuation = 0.85      # plan 026
factor_perturbation_low = 0.85
factor_perturbation_high = 1.15


@dataclass(frozen=True)
class StartAttempt:
    """One candidate start: whether the target solve converged, its objective value and its wall time."""
    label: str
    success: bool
    mass_takeoff_kg: Any = None
    objective_value: Any = None
    time_s: float = 0.0
    message: str = ""


@dataclass(frozen=True)
class StartRecord:
    """`label`: the start whose solution was returned; `attempts`: every candidate tried, in order."""
    label: str
    attempts: tuple

    def successes(self):
        return tuple(a for a in self.attempts if a.success)

    def spread_mass_takeoff_kg(self):
        """Largest minus smallest take-off mass over the successful starts (local-optimum spread)."""
        masses = [a.mass_takeoff_kg for a in self.successes()]
        return max(masses) - min(masses) if masses else None


def objective_value(result, objective):
    """The minimized quantity of `objective` on a numeric result (lower is better)."""
    if objective == "mass_takeoff":
        return result.mass_takeoff_kg
    if objective == "payload":
        return -result.mass_payload_kg
    if objective == "cost":
        return result.cost.per_mission_usd
    raise ValueError(f"Unknown objective '{objective}'.")


def default_starts(requirements, assumptions, objective):
    """The ordered candidate starts that apply to this problem (plan 029).

    0. "redundancy_off" / "strings_off" (Tier 18 redundancy): the same aircraft without redundancy, then (with
       battery strings) without the strings and their failure cases (plan 032).
    1. "electrical_off" (Tier 15 electrical layer with the thermal model): the same problem without the layer
       (plan 023: from the layer design without the thermal model IPOPT reaches local infeasibility). Without
       the thermal model it is a later fallback, after payload continuation.
       "thermal_off" (thermal model without the layer): the same problem without the thermal model (plan 028);
       from the generic guess, and with AeroBuildup from the thermal Scholz design, IPOPT can fail.
    2. "mass_objective" (objective "cost" only): the same problem at minimum take-off mass.
    3. "scholz_aero" (AeroBuildup only): the same problem with the Scholz hand-check aerodynamics (plan 025).
    4. "constant_battery" (equivalent-circuit pack only): the same requirements with the constant-OCV battery,
       without the electrical layer, at minimum take-off mass (plans 022 and 023).
    5. "payload_continuation" (equivalent-circuit pack, fixed payload of at least 300 kg): the same problem at
       85 % payload (plan 026).
    6. "perturbed_low" / "perturbed_high": the first precursor (or caller) design scaled by 0.85 / 1.15.
    7. "generic": the built-in generic guess.
    """
    a, is_ecm = assumptions, assumptions.battery_model == "ecm"
    starts = []
    if a.redundancy:
        # Tier 18 (plan 032): the same aircraft without redundancy; with battery strings, then the redundant
        # aircraft without the strings and their failure cases (which itself starts from "redundancy_off").
        starts.append("redundancy_off")
        if a.count_strings_battery > 1:
            starts.append("strings_off")
    if a.thermal_model:
        starts.append("electrical_off" if a.electrical_layer else "thermal_off")
    if objective == "cost":
        starts.append("mass_objective")
    if a.aerodynamics_model == "buildup":
        starts.append("scholz_aero")
    if is_ecm:
        starts.append("constant_battery")
        if objective != "payload" and requirements.mass_payload_kg >= 300.0:
            starts.append("payload_continuation")
    if a.electrical_layer and not a.thermal_model:
        starts.append("electrical_off")
    if starts:
        starts += ["perturbed_low", "perturbed_high"]
    return tuple(starts + ["generic"])


def _precursor_problem(label, requirements, assumptions, objective):
    """The simpler problem a start label solves first, as (requirements, assumptions, objective); None: none."""
    if label == "redundancy_off":
        return requirements, replace(assumptions, redundancy=False), objective
    if label == "strings_off":
        return requirements, replace(assumptions, count_strings_battery=1, failure_string_out=False,
                                     failure_string_out_engine_out=False), objective
    if label == "electrical_off":
        return requirements, replace(assumptions, electrical_layer=False), objective
    if label == "thermal_off":
        return requirements, replace(assumptions, thermal_model=False), objective
    if label == "mass_objective":
        return requirements, assumptions, "mass_takeoff"
    if label == "scholz_aero":
        return requirements, replace(assumptions, aerodynamics_model="scholz"), objective
    if label == "constant_battery":
        return requirements, replace(assumptions, battery_model="constant", electrical_layer=False), "mass_takeoff"
    if label == "payload_continuation":
        return (replace(requirements, mass_payload_kg=fraction_payload_continuation * requirements.mass_payload_kg),
                assumptions, "mass_takeoff")
    return None


def perturbed_start(result, factor):
    """`result` with every design quantity, take-off mass and speeds scaled by `factor` (an initial guess only)."""
    design = HaloDesign(**{f.name: (getattr(result.design, f.name) * factor
                                    if getattr(result.design, f.name) is not None else None)
                           for f in fields(HaloDesign)})
    return replace(result, design=design, mass_takeoff_kg=result.mass_takeoff_kg * factor,
                   velocity_cruise_m_s=result.velocity_cruise_m_s * factor,
                   velocity_loiter_m_s=result.velocity_loiter_m_s * factor)


def solve_halo_sizing_multistart(requirements=HaloRequirements(), assumptions=HaloAssumptions(), factors=None,
                                 verbose=False, max_iter=3000, objective="mass_takeoff", starts=None, select="first",
                                 cache=None, initial=None):
    """Tier 22 starting-point strategy (plan 029): try `starts` in order, each one coupled solve of this problem.

    `starts`: start labels (`start_labels`); None uses `default_starts`. `initial`: a caller-supplied design (an
    earlier `HaloSizingResult`, e.g. a neighbouring problem in a sweep) tried first as start "caller" and used as
    the design the perturbed starts scale. `select` "first" returns the first converged solution (the default,
    as cheap as the earlier rules); "best" tries every start and returns the lowest objective among those that
    converged, so `result.start` also shows whether different starts reach
    different local optima (`StartRecord.spread_mass_takeoff_kg`).

    Precursor problems are solved by this same strategy with the default starts, but without perturbation, and
    without payload continuation inside a continuation precursor, so a failing problem costs a bounded number of
    solves. `cache` (a dict) shares precursor solutions between calls. Raises RuntimeError when no start converges.
    """
    if select not in ("first", "best"):
        raise ValueError(f"Unknown select '{select}'.")
    starts = default_starts(requirements, assumptions, objective) if starts is None else tuple(starts)
    if initial is not None:
        starts = ("caller",) + tuple(s for s in starts if s != "caller")
    return _multistart(requirements, assumptions, factors, verbose, max_iter, objective, starts, select,
                       {} if cache is None else cache, initial)


def _multistart(requirements, assumptions, factors, verbose, max_iter, objective, starts, select, cache,
                initial_caller=None):
    unknown = [s for s in starts if s not in start_labels]
    if unknown:
        raise ValueError(f"Unknown start labels {unknown}.")
    if "caller" in starts and initial_caller is None:
        raise ValueError("Start 'caller' needs `initial`.")
    attempts, solutions, precursors = [], [], []
    for label in starts:
        time_start_s = time.perf_counter()
        initial, message = None, ""
        if label == "caller":
            initial = initial_caller
            precursors.append(initial)
        elif label in ("perturbed_low", "perturbed_high"):
            if not precursors:
                attempts.append(StartAttempt(label, False, message="no precursor design to perturb"))
                continue
            initial = perturbed_start(precursors[0], factor_perturbation_low if label == "perturbed_low"
                                      else factor_perturbation_high)
        elif label != "generic":
            problem = _precursor_problem(label, requirements, assumptions, objective)
            initial = _precursor(problem, factors, max_iter, cache, allow_continuation=label != "payload_continuation")
            if initial is None:
                attempts.append(StartAttempt(label, False, time_s=time.perf_counter() - time_start_s,
                                             message="precursor problem did not converge"))
                continue
            precursors.append(initial)
        try:
            result = solve_halo_sizing_once(requirements, assumptions, factors, verbose, max_iter, initial, objective)
        except RuntimeError as error:
            lines = [line.strip() for line in str(error).splitlines() if line.strip()]
            message = lines[-1][:200] if lines else "solver failed"
            attempts.append(StartAttempt(label, False, time_s=time.perf_counter() - time_start_s, message=message))
            continue
        attempts.append(StartAttempt(label, True, result.mass_takeoff_kg, objective_value(result, objective),
                                     time.perf_counter() - time_start_s))
        solutions.append((label, result))
        if select == "first":
            break
    if not solutions:
        raise RuntimeError("Halo sizing: no starting point converged; attempts: "
                           + "; ".join(f"{a.label}: {a.message}" for a in attempts))
    # Ties (within 1e-6 relative) go to the earlier start, so "best" keeps the "first" answer when they agree.
    lowest = min(objective_value(r, objective) for _, r in solutions)
    label, result = next((l, r) for l, r in solutions
                         if objective_value(r, objective) <= lowest + 1e-6 * max(abs(lowest), 1.0))
    return replace(result, start=StartRecord(label, tuple(attempts)))


def _precursor(problem, factors, max_iter, cache, allow_continuation):
    """Solve a precursor problem (cached); None if it does not converge. No perturbed starts below the first
    level, and no payload continuation inside a continuation precursor, so the number of solves stays bounded."""
    requirements, assumptions, objective = problem
    excluded = ("perturbed_low", "perturbed_high") + (() if allow_continuation else ("payload_continuation",))
    key = (repr(requirements), repr(assumptions), objective, id(factors), excluded)
    if key not in cache:
        starts = tuple(s for s in default_starts(requirements, assumptions, objective) if s not in excluded)
        try:
            cache[key] = _multistart(requirements, assumptions, factors, False, max_iter, objective, starts, "first",
                                     cache)
        except RuntimeError:
            cache[key] = None
    return cache[key]



# Named earlier baselines, so each tier's notebook keeps reproducing its own result. `pre_plan035` pins the
# settings plan 035 changed (drag corrections, redundancy, unit-built machines, gearbox stages) to their earlier
# values for every set below.
pre_plan035 = dict(drag_corrections=False, redundancy=False, machine_mass_model="torque_density", gearbox_stages=False)
# Plans 037 and 038 (the drawn layout and the wing check): XV-15 fuselage calibration, turbogenerators at the tips,
# round 12.8 m fuselage with the tail at 11.4 m, NDARC spar caps without a minimum gauge, XV-15 pylon ratio.
pre_layout = dict(mass_factor_fuselage=None, turbogenerators_on_wing_tips=True, height_fuselage_m=None,
                  length_fuselage_m=42.1 * u.foot, x_horizontal_tail_m=11.4, ratio_radius_gyration_pylon=0.222,
                  ratio_depth_spar_cap=1.0, thickness_min_torque_box_m=0.0, offset_x_turbogenerators_m=1.0,
                  z_turbogenerators_m=0.5)
requirements_tier10c = HaloRequirements(mass_payload_kg=900.0, velocity_max_m_s=250 * u.knot, hover_hot_day=False)
requirements_tier12b = HaloRequirements(mass_payload_kg=900.0, hover_hot_day=False)
# Tiers 13-16 reference (plans 018-020): 900 kg with the constant-OCV battery (13,760 lb).
requirements_tier16 = HaloRequirements(mass_payload_kg=900.0)
assumptions_tier16 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, battery_model="constant",
                                     wing_weight_model="raymer",
                                     aerodynamics_model="simple")
# Plan 022 reference: 780 kg with the equivalent-circuit battery and the Raymer wing (14,436 lb).
requirements_plan022 = HaloRequirements(mass_payload_kg=780.0)
assumptions_plan022 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, wing_weight_model="raymer",
                                      aerodynamics_model="simple")
# Plan 026 reference: 900 kg, AFDD wing, SimpleAerodynamics (14,247 lb).
requirements_plan026 = HaloRequirements(mass_payload_kg=900.0)
assumptions_plan026 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, aerodynamics_model="simple")
assumptions_tier12 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, battery_model="constant",
                                     wing_weight_model="raymer",
                                     aerodynamics_model="simple", power_rated_turboshaft_fixed_W=None,
                                     hybridization_electric_min=0.0, soc_floor_every_segment=False,
                                     machine_mass_by_torque=False)
# Tier 12b reference (plan 017): fixed engines with the constant-OCV battery and Tier 12b machines.
assumptions_tier12b = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, battery_model="constant",
                                      wing_weight_model="raymer",
                                      aerodynamics_model="simple", machine_mass_by_torque=False)
# Tier 17 (plan 021): the equivalent-circuit 50G-shaped pack at end of life, on the Raymer-wing aircraft.
assumptions_tier17 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, battery_model="ecm",
                                     wing_weight_model="raymer",
                                     aerodynamics_model="simple")
assumptions_tier11a = replace(assumptions_tier12, rotor_speed_physics=False)
# Tier 15 (plan 023): the reference with the electrical layer (756 V nominal pack, 1,200 V inverters).
assumptions_tier15 = HaloAssumptions(**pre_plan035, **pre_layout, electrical_layer=True, thermal_model=False)
standard_blocking_voltages_V = (650.0, 1200.0, 1700.0, 3300.0)


def solve_halo_max_payload(requirements=HaloRequirements(), assumptions=HaloAssumptions(), initial=None, **kwargs):
    """Maximum payload from `initial` (with the equivalent-circuit pack, a constant-battery design: plan 022).

    If that start fails and the pack is the equivalent circuit, the constant-battery maximum payload without the
    electrical layer (itself from `initial`) is tried as a second starting point. Starting points only: each attempt
    is one complete coupled solve.
    """
    try:
        return solve_halo_sizing(requirements, assumptions, objective="payload", initial=initial, **kwargs)
    except RuntimeError:
        if assumptions.battery_model != "ecm":
            raise
    start = solve_halo_sizing(requirements, replace(assumptions, battery_model="constant", electrical_layer=False),
                              objective="payload", initial=initial, **kwargs)
    return solve_halo_sizing(requirements, assumptions, objective="payload", initial=start, **kwargs)


def assumptions_for_bus_voltage(assumptions, voltage_nominal_V, dcdc=False):
    """Tier 15 discrete bus-voltage option, with the explicit pack coupling.

    Without a DC/DC converter the pack sets the bus: count_series = round(V_nominal / 3.6 V) (50G cells). With
    one, the pack keeps its series count and the converter regulates the bus at `voltage_nominal_V`. The
    inverter class is the smallest standard blocking voltage whose derated limit covers the highest bus
    voltage (650 / 1,200 / 1,700 / 3,300 V).
    """
    a = replace(assumptions, electrical_layer=True, dcdc_converter=dcdc)
    if dcdc:
        a = replace(a, voltage_bus_dcdc_V=voltage_nominal_V)
    else:
        a = replace(a, count_series_battery=int(round(voltage_nominal_V / inr21700_50g_cell().voltage_nominal_V)))
    voltage_max_V = bus_voltage_window(a).voltage_max_V
    blocking_V = next(v for v in standard_blocking_voltages_V if v * a.factor_derating_voltage_inverter >= voltage_max_V)
    return replace(a, voltage_blocking_inverter_V=blocking_V)


def enumerate_bus_voltage(voltages_nominal_V=(540.0, 756.0, 800.0, 1000.0), requirements=HaloRequirements(),
                          assumptions=HaloAssumptions(), objective="mass_takeoff", dcdc=False, initial=None):
    """Tier 15 discrete trade: one independent sizing per bus-voltage option (an explicit enumeration of a
    discrete choice, not a convergence loop). Returns ((option assumptions, result or None), ...); None marks
    an option that failed to solve.

    Every option starts from the same `initial` design; None uses the constant-battery aircraft without the
    electrical layer (the max-payload solve needs a constant-battery start, plan 022; see
    `solve_halo_max_payload`). An option that fails from it is retried from the previous option's solution and
    then from the nearest option that solved (different starting points only; each is one coupled solve).
    """
    if initial is None:
        initial = solve_halo_sizing(requirements, replace(assumptions, battery_model="constant", electrical_layer=False))
    rows, previous = [], None
    for voltage_V in voltages_nominal_V:
        option = assumptions_for_bus_voltage(assumptions, voltage_V, dcdc=dcdc)
        result = None
        for start in (initial, previous):
            if start is None:
                continue
            try:
                result = (solve_halo_max_payload(requirements, option, initial=start) if objective == "payload"
                          else solve_halo_sizing(requirements, option, initial=start))
                break
            except RuntimeError:
                pass
        previous = result or previous
        rows.append((option, result))
    # Options that failed from both are retried once from the nearest option that solved.
    for index, (option, result) in enumerate(rows):
        solved = [(abs(j - index), r) for j, (_, r) in enumerate(rows) if r is not None]
        if result is None and solved:
            try:
                start = min(solved, key=lambda item: item[0])[1]
                rows[index] = (option, solve_halo_max_payload(requirements, option, initial=start)
                               if objective == "payload" else solve_halo_sizing(requirements, option, initial=start))
            except RuntimeError:
                pass
    return tuple(rows)
# Tier 20 (plan 024): the AFDD tiltrotor wing (the default from plan 026).
assumptions_tier20 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False, wing_weight_model="afdd_tiltrotor",
                                     aerodynamics_model="simple")
# Plan 027 reference: 900 kg, AFDD wing, AeroBuildup, no thermal model (13,639 lb).
requirements_plan027 = HaloRequirements(mass_payload_kg=900.0)
assumptions_plan027 = HaloAssumptions(**pre_plan035, **pre_layout, thermal_model=False)
# Tier 18 (plan 032): the reference with the sensible redundancy set (2 lanes per rotor, 2 cross-strapped buses,
# 2 battery strings; lane-out, bus-out and string-out hovers).
assumptions_tier18 = HaloAssumptions(**dict(pre_plan035, redundancy=True), **pre_layout)
# Plan 030 reference: 900 kg, AFDD wing, AeroBuildup, thermal model, rubber machines (14,037 lb).
requirements_plan030 = HaloRequirements(mass_payload_kg=900.0)
assumptions_plan030 = HaloAssumptions(**pre_plan035, **pre_layout)
# Plan 037 reference (the drawn layout, plan 032 on its branch): layout-anchored fuselage, turbogenerators in the
# fuselage, boxy 11 m fuselage; NDARC spar caps, no minimum gauge, XV-15 pylon ratio (12,821 lb).
requirements_plan037 = HaloRequirements(mass_payload_kg=900.0)
assumptions_plan037 = HaloAssumptions(**pre_plan035, ratio_radius_gyration_pylon=0.222, ratio_depth_spar_cap=1.0,
                                      thickness_min_torque_box_m=0.0, offset_x_turbogenerators_m=1.0,
                                      z_turbogenerators_m=0.5)
# Plan 038 reference (plan 035 on its branch): plan 037 plus spar caps at the box depth, 1 mm minimum gauge, pylon
# inertia from the tip components, turbogenerators by the layout (13,038 lb).
requirements_plan038 = HaloRequirements(mass_payload_kg=900.0)
assumptions_plan038 = HaloAssumptions(**pre_plan035)


if __name__ == "__main__":
    result = solve_halo_sizing(verbose=False)
    d = result.design
    print(f"MTOM {result.mass_takeoff_kg:,.0f} kg ({result.mass_takeoff_kg / u.lbm:,.0f} lb), empty "
          f"{result.mass_empty_kg:,.0f} kg, fuel {result.mass_fuel_kg:,.0f} kg, battery {result.energy_battery_kWh:.1f} kWh")
    print(f"wing {d.area_wing_m2:.1f} m2 (span {result.span_m:.2f} m), rotor R {result.radius_rotor_m:.2f} m, "
          f"DL {result.disk_loading_kg_m2:.0f} kg/m2, W/S {result.wing_loading_kg_m2:.0f} kg/m2")
    print(f"motors {result.power_rated_motor_W / 1e3:,.0f} kW x2, turboshafts {d.power_rated_turboshaft_W / 1e3:,.0f} kW x2 "
          f"({d.mass_turboshaft_bare_kg:.0f} kg each, eta {result.thermal_efficiency_turboshaft:.3f}), battery "
          f"{d.power_max_discharge_battery_W / 1e3:,.0f} kW")
    print(f"cruise {result.velocity_cruise_m_s / u.knot:.0f} kt, L/D {result.lift_to_drag_cruise:.1f}, CL {result.cl_cruise:.2f}")
    print("binding:", ", ".join(result.binding))
    for name, mass_kg in result.component_masses_kg:
        print(f"  {name:<16}{mass_kg:9.1f} kg")
    for s in result.segments:
        print(f"  {s['label']:<15}{s['duration_s']:7.0f} s {s['fuel_kg']:7.1f} kg SOC {s['soc_end']:.3f} "
              f"h_e {s['hybridization']:.2f} {s['power_rotors_W'] / 1e3:7.0f} kW")
    for h in result.hovers:
        print(f"  {h.label:<15}{h.altitude_m / u.foot:6.0f} ft ISA{h.temperature_offset_K:+5.1f} K "
              f"rotors {h.power_rotors_W / 1e3:5.0f} kW, turbines available {h.power_available_turboshafts_W / 1e3:5.0f} kW, "
              f"battery share >= {h.battery_share_min:+.2f}")
