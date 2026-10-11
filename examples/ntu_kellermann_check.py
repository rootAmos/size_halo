"""Plan 051: the ε-NTU cooler at Kellermann et al.'s 2021 design point, and published TMS mass breakdowns.

    python -m examples.ntu_kellermann_check

Kellermann, Lüdemann, Pohl, Hornung, "Design and Optimization of Ram Air-Based Thermal Management Systems for
Hybrid-Electric Aircraft", Aerospace 8(1), 3, 2021: SP 30 %, T_cp 380 K, top of climb (FL350, Mach 0.68),
96.3 kW per powertrain, 50 % water-glycol. Their core is rectangular microchannels (two-pass cross flow in overall
counterflow); ours is offset strip fins in single-pass cross flow, so the comparison is at system level: the same
heat, inlet state, coolant temperatures, capacity-rate ratio and core pressure-ratio budget, then core size, drag
and mass.

Our core and duct are sized by a small `asb.Opti` problem: the three core counts and the nozzle area, minimizing
core mass + duct allowance + 2.4 kg per N of drag (Kellermann's Figure 5 fuel-burn sensitivities), subject to the
heat, Π_c, a coolant-side loss cap, the duct residual at the fixed air flow and the core-exit pressure margin.
"""
from dataclasses import dataclass, field, replace
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.powertrain.components.compact_heat_exchanger import FluidProperties, OffsetStripFinCore
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct


def water_glycol_cold_properties():
    """50 % ethylene glycol near 296 K, the mean of Kellermann's loop (275-317 K): ASHRAE tables, rounded."""
    return FluidProperties(specific_heat_J_kg_K=3300.0, conductivity_W_m_K=0.38, viscosity_Pa_s=3.8e-3,
                           density_kg_m3=1070.0)


@dataclass(frozen=True)
class KellermannDesignPoint:
    """Kellermann 2021 inputs (Section 2.5, Figure 6, Table 2 defaults) for one TMS at SP 30 %, T_cp 380 K."""
    altitude_m: float = 35000 * 0.3048
    mach: float = 0.68
    power_heat_W: float = 96.3e3
    temperature_coldplate_K: float = 380.0
    temperature_coolant_outlet_K: float = 275.0     # T1: coldplate coolant inlet = exchanger coolant outlet
    effectiveness_coldplate: float = 0.4
    ratio_capacity: float = 1.0                     # C*_R, cold (air) to hot (coolant)
    ratio_pressure_core: float = 0.95               # Π_c, air side
    pressure_drop_coolant_max_Pa: float = 100e3     # Π_h optimized and not given: a cap
    mass_per_drag_kg_N: float = 3.6 / 1.5           # Figure 5: 1.5 % fuel per 1,000 kg, 3.6 % per 1,000 N
    coolant: Any = field(default_factory=water_glycol_cold_properties)

    def atmosphere(self):
        return asb.Atmosphere(altitude=self.altitude_m)

    def velocity_m_s(self):
        return self.mach * self.atmosphere().speed_of_sound()

    def temperature_coolant_inlet_K(self):
        """Exchanger coolant inlet = coldplate outlet, T1 + ε_cp (T_cp - T1) (Kellermann Eq. A2)."""
        return (self.temperature_coolant_outlet_K
                + self.effectiveness_coldplate * (self.temperature_coldplate_K - self.temperature_coolant_outlet_K))

    def capacity_coolant_W_K(self):
        return self.power_heat_W / (self.temperature_coolant_inlet_K() - self.temperature_coolant_outlet_K)


@dataclass(frozen=True)
class KellermannReference:
    """Published results for one TMS (Figure 10 read off the plot, halved: it shows two TMS)."""
    length_air_m: float = 0.48          # L_c
    length_coolant_m: float = 0.73      # L_h
    height_stack_m: float = 0.18        # H_stack
    length_diffuser_m: float = 2.2
    length_nozzle_m: float = 0.9
    mass_tms_kg: float = 110.0 / 2      # coldplates, exchanger, diffuser, nozzle, pipes, pump, coolant (no fan)
    drag_tms_N: float = 10.0 / 2
    power_fan_hot_day_W: float = 60e3 / 2   # Section 4.2: up to 60 kW for both, hot-day take-off

    def volume_core_m3(self):
        return self.length_air_m * self.length_coolant_m * self.height_stack_m

    def area_frontal_m2(self):
        return self.length_coolant_m * self.height_stack_m


@dataclass(frozen=True)
class KellermannCheck:
    point: KellermannDesignPoint
    core: OffsetStripFinCore
    area_nozzle_m2: float
    mass_flow_air_kg_s: float
    mass_flow_coolant_kg_s: float
    temperature_air_inlet_K: float
    power_heat_W: float
    effectiveness: float
    ratio_pressure_core: float
    pressure_drop_air_Pa: float
    pressure_drop_coolant_Pa: float
    drag_N: float
    velocity_exit_m_s: float
    mass_core_kg: float
    mass_ducts_kg: float
    reynolds_air: float
    reynolds_coolant: float

    def volume_core_m3(self):
        return self.core.length_m() * self.core.width_m() * self.core.height_m()


def size_at_kellermann(point=KellermannDesignPoint(), duct=RamAirDuct(), mass_duct_per_area_kg_m2=12.0):
    """Our core and duct sized at Kellermann's design point (a small Opti problem; see the module docstring)."""
    atmosphere, velocity_m_s = point.atmosphere(), point.velocity_m_s()
    temperature_air_K, pressure_face_Pa, density_face_kg_m3 = duct.inlet_state(atmosphere, velocity_m_s)
    capacity_W_K = point.capacity_coolant_W_K()
    mass_flow_coolant_kg_s = capacity_W_K / point.coolant.specific_heat_J_kg_K
    mass_flow_air_kg_s = point.ratio_capacity * capacity_W_K / duct.specific_heat_J_kg_K
    air = FluidProperties(specific_heat_J_kg_K=duct.specific_heat_J_kg_K, conductivity_W_m_K=0.0215,  # air near 241 K
                          viscosity_Pa_s=atmosphere.dynamic_viscosity(), density_kg_m3=density_face_kg_m3)
    opti = asb.Opti()
    log_wide, log_long, log_tall, log_nozzle = (opti.variable(init_guess=np.log(v)) for v in (500.0, 15.0, 20.0, 0.05))
    core = OffsetStripFinCore(count_wide=np.exp(log_wide), count_long=np.exp(log_long), count_tall=np.exp(log_tall),
                              density_coolant_kg_m3=point.coolant.density_kg_m3)
    sized_duct = replace(duct, area_nozzle_m2=np.exp(log_nozzle))
    result = core.evaluate(mass_flow_air_kg_s, mass_flow_coolant_kg_s, temperature_air_K,
                           point.temperature_coolant_inlet_K(), air, point.coolant)
    flow = sized_duct.evaluate(mass_flow_air_kg_s, point.power_heat_W, result.air.pressure_drop_Pa,
                               core.area_frontal_m2(), atmosphere, velocity_m_s)
    mass_ducts_kg = mass_duct_per_area_kg_m2 * core.area_frontal_m2()
    opti.subject_to([
        result.power_heat_W / point.power_heat_W >= 1,
        result.air.pressure_drop_Pa / pressure_face_Pa == 1 - point.ratio_pressure_core,
        result.coolant.pressure_drop_Pa / point.pressure_drop_coolant_max_Pa <= 1,
        flow.residual_mass_flow_kg_s == 0,
        (flow.pressure_total_core_exit_Pa - atmosphere.pressure()) / 1e3 >= 0])
    opti.minimize(core.get_mass() + mass_ducts_kg + point.mass_per_drag_kg_N * flow.drag_N)
    s = opti.solve(verbose=False)
    sized_core = replace(core, count_wide=s(core.count_wide), count_long=s(core.count_long),
                         count_tall=s(core.count_tall))
    return KellermannCheck(
        point=point, core=sized_core, area_nozzle_m2=s(sized_duct.area_nozzle_m2),
        mass_flow_air_kg_s=mass_flow_air_kg_s, mass_flow_coolant_kg_s=mass_flow_coolant_kg_s,
        temperature_air_inlet_K=temperature_air_K, power_heat_W=s(result.power_heat_W),
        effectiveness=s(result.effectiveness),
        ratio_pressure_core=1 - s(result.air.pressure_drop_Pa) / pressure_face_Pa,
        pressure_drop_air_Pa=s(result.air.pressure_drop_Pa), pressure_drop_coolant_Pa=s(result.coolant.pressure_drop_Pa),
        drag_N=s(flow.drag_N), velocity_exit_m_s=s(flow.velocity_exit_m_s), mass_core_kg=s(core.get_mass()),
        mass_ducts_kg=s(mass_ducts_kg), reynolds_air=s(result.air.reynolds), reynolds_coolant=s(result.coolant.reynolds))


@dataclass(frozen=True)
class PublishedTmsMass:
    """One published TMS mass breakdown. `items_kg`: component masses as published (totals over quantities)."""
    source: str
    system: str
    power_heat_W: float
    items_kg: dict
    excluded: str

    def total_kg(self):
        return sum(self.items_kg.values())

    def specific_power_W_kg(self):
        return self.power_heat_W / self.total_kg()


def published_breakdowns():
    """Published TMS masses (numbers as printed; see plan 051 for page references)."""
    return (
        PublishedTmsMass(
            source="Kellermann et al. 2021, Fig. 10 (read off the plot) and Sec. 4.1",
            system="one of two ram-air TMS, SP 30 %, T_cp 380 K, top of climb",
            power_heat_W=96.3e3, items_kg={"whole TMS (no breakdown published)": 55.0},
            excluded="puller fan, redundancy (authors expect up to +100 %), integration (cowlings)"),
        PublishedTmsMass(
            source="Chapman, Schnulo, Nitzsche, NASA/TM-2020-220473, Table 4 (steady state)",
            system="15-passenger tiltwing, hover at 5,000 ft, oil loop + 4 coolant loops",
            # Table 2 hover heat: generator 181.15, rectifier 172.1, gearbox 72.5, 4 x (inverter 40.44, motor 38.42).
            power_heat_W=(181.15 + 172.1 + 72.5 + 4 * (40.44 + 38.42)) * 1e3,
            items_kg={"heat exchangers (wet): 1 x 27.13 + 4 x 25.3": 27.13 + 4 * 25.3,
                      "puller fans: 1 x 3.69 + 4 x 3.22": 3.69 + 4 * 3.22,
                      "pumps: 2.8 + 2.4": 2.8 + 2.4,
                      "lines (fluid only): 15.1 + 11": 15.1 + 11.0},
            excluded="inlet, nozzle, headers, line walls, coldplates (171.63 kg in the transient design)"),
        PublishedTmsMass(
            source="Chapman, Schnulo, Nitzsche, NASA/TM-2020-220473, Table 4 (coolant loops only)",
            system="the 4 air-coolant loops: rectifier, inverters, motors",
            power_heat_W=(172.1 + 4 * (40.44 + 38.42)) * 1e3,
            items_kg={"heat exchangers (wet)": 4 * 25.3, "puller fans": 4 * 3.22, "pump": 2.4,
                      "line (fluid only)": 11.0},
            excluded="inlet, nozzle, headers, line walls, coldplates"),
        PublishedTmsMass(
            source="Chapman, Hasseeb, Schnulo, NASA/TM-20205011477, Tables 7 and 8 (baseline)",
            system="RVLT tiltwing, hover: motor loops, engine oil loop, engine coolant loop",
            power_heat_W=361e3, items_kg={"motor loops 4 x 7.11": 4 * 7.11, "engine oil loop": 14.55,
                                          "engine coolant loop": 11.05},
            excluded="components treated as ideal; no component split published"),
        PublishedTmsMass(
            source="Chapman, Hasseeb, Schnulo, NASA/TM-20205011477, Tables 5 and 6 (corrected baseline)",
            system="STARC-ABL, rolling take-off, electric system only",
            power_heat_W=(118 + 58 + 55 + 109) * 1e3,
            items_kg={"engine oil loops 2 x 10.08": 2 * 10.08, "engine coolant loops 2 x 15.30": 2 * 15.30,
                      "BLI loop": 58.57},
            excluded="components treated as ideal; no component split published"),
    )


def plan042_cooler(allowances="estimate"):
    """The v3.7 / plan 042 sized cooler: counts 708 x 11.1 x 21.7 (1.03 x 0.34 x 0.067 m), nozzle 0.082 m2,
    fan rating 8.6 kW (the v3.7 pickle with the NTU cooler is not cached; plan 042 reports this geometry)."""
    return NtuRamAirHeatExchanger(core=OffsetStripFinCore(count_wide=708.0, count_long=11.1, count_tall=21.7),
                                  duct=RamAirDuct(area_nozzle_m2=0.082), power_rated_fan_W=8.6e3,
                                  allowances=allowances)


def pressure_drop_lines_Pa(cooler):
    """Darcy loss in the cooler's lines with Blasius' friction factor (supports the pump rating)."""
    c = cooler.coolant
    area_m2 = np.pi * cooler.diameter_line_m**2 / 4
    velocity_m_s = cooler.mass_flow_coolant_kg_s / (c.density_kg_m3 * area_m2)
    reynolds = c.density_kg_m3 * velocity_m_s * cooler.diameter_line_m / c.viscosity_Pa_s
    friction = 0.316 * reynolds**-0.25
    return friction * cooler.length_lines_m / cooler.diameter_line_m * c.density_kg_m3 * velocity_m_s**2 / 2


def main():
    point, reference = KellermannDesignPoint(), KellermannReference()
    print(f"Kellermann 2021 design point: FL350 Mach {point.mach} ({point.velocity_m_s():.1f} m/s), "
          f"{point.power_heat_W / 1e3:.1f} kW, coolant {point.temperature_coolant_inlet_K():.0f} -> "
          f"{point.temperature_coolant_outlet_K:.0f} K, C*_R {point.ratio_capacity}, Pi_c {point.ratio_pressure_core}")
    cases = (("Pi_c 0.95, our duct (inlet recovery 0.95)", point, RamAirDuct()),
             ("Pi_c 0.95, ideal inlet (recovery 1.0)", point, RamAirDuct(recovery_inlet=1.0)),
             ("Pi_c 0.98, our duct", replace(point, ratio_pressure_core=0.98), RamAirDuct()))
    print(f"\n{'case':44s} {'Q kW':>6s} {'Lc x Lh x H m':>18s} {'V L':>5s} {'A m2':>5s} {'Pi_c':>6s} "
          f"{'dp_h kPa':>8s} {'D N':>6s} {'core kg':>7s} {'duct kg':>7s}")
    print(f"{'Kellermann (microchannels, 2-pass)':44s} {96.3:6.1f} "
          f"{reference.length_air_m:5.2f} x {reference.length_coolant_m:4.2f} x {reference.height_stack_m:4.2f} "
          f"{reference.volume_core_m3() * 1e3:5.1f} {reference.area_frontal_m2():5.3f} {0.95:6.3f} {'-':>8s} "
          f"{reference.drag_tms_N:6.1f} {'-':>7s} {'-':>7s}   whole TMS ~{reference.mass_tms_kg:.0f} kg")
    for label, p, duct in cases:
        c = size_at_kellermann(p, duct)
        print(f"{label:44s} {c.power_heat_W / 1e3:6.1f} {c.core.length_m():5.3f} x {c.core.width_m():4.2f} x "
              f"{c.core.height_m():4.2f} {c.volume_core_m3() * 1e3:5.1f} {c.core.area_frontal_m2():5.3f} "
              f"{c.ratio_pressure_core:6.3f} {c.pressure_drop_coolant_Pa / 1e3:8.1f} {c.drag_N:6.1f} "
              f"{c.mass_core_kg:7.1f} {c.mass_ducts_kg:7.1f}   eps {c.effectiveness:.3f}, Re air {c.reynolds_air:.0f}"
              f", nozzle {c.area_nozzle_m2:.3f} m2")

    print("\nPublished TMS masses:")
    for b in published_breakdowns():
        print(f"- {b.source}: {b.system}; heat {b.power_heat_W / 1e3:.0f} kW, total {b.total_kg():.1f} kg "
              f"({b.specific_power_W_kg() / 1e3:.2f} kW/kg); excludes {b.excluded}")
        for name, mass_kg in b.items_kg.items():
            print(f"    {name}: {mass_kg:.2f} kg ({mass_kg / b.total_kg():.0%})")

    estimate, published = plan042_cooler("estimate"), plan042_cooler("published")
    print(f"\nHalo v3.7 cooler (plan 042 geometry): lines {published.mass_lines_per_length_kg_m():.2f} kg/m, line loss "
          f"{pressure_drop_lines_Pa(published) / 1e3:.0f} kPa at {published.mass_flow_coolant_kg_s} kg/s")
    for label, cooler in (("estimate", estimate), ("published", published)):
        b = cooler.mass_breakdown()
        print(f"  {label:9s}: core {b.mass_core_kg:.1f}, ducts {b.mass_ducts_kg:.1f}, fans {b.mass_fans_kg:.1f}, "
              f"pump and pipes {b.mass_pump_pipes_kg:.1f}, total {b.total_kg():.1f} kg")
    delta_kg = published.get_mass() - estimate.get_mass()
    print(f"  change {delta_kg:+.1f} kg; take-off weight about {2.5 * delta_kg:+.0f} kg "
          f"({2.5 * delta_kg / 0.45359237:+.0f} lb) with the growth factor 2.5")


if __name__ == "__main__":
    main()
