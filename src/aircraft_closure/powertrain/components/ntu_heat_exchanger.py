"""Installed ram-air cooler sized by the ε-NTU core and the duct (plan 042, the sizing option "ntu").

The same interface as `RamAirHeatExchanger` (`get_mass`, `get_limits`, `evaluate`, `margins`), but the operating
point needs two more unknowns that the caller's `asb.Opti` owns: the air mass flow (with the duct residual
`residual_mass_flow_kg_s` constrained to zero) and, in hover, the fan total-pressure rise.

* Coolant loop: the coolant leaves the exchanger at `temperature_coolant_C` (the loop temperature the lumped
  machine models reject to) and enters it at T_c + Q / C_coolant, C_coolant = ṁ_coolant c_coolant (a fixed
  coolant flow). Air enters the core at the freestream total temperature.
* Heat-rejection margin: the core must reject the heat Q at that inlet temperature,
  ε C_min (T_c + Q / C_coolant - T_air) >= Q, i.e. ε C_min (T_c - T_air) - Q (1 - ε C_min / C_coolant) >= 0,
  normalized by `power_heat_scale_W`. A capacity inequality: a valve or bypass passes the actual heat.
* Duct: heat addition Q_mean (the interval-mean heat, as the old drag law), core air-side pressure drop at the
  flow, diffuser and nozzle losses, fan pressure rise; net force F and drag -F (negative drag is the Meredith
  thrust); fan electrical power ṁ Δp_fan / (ρ_face η_fan), below the fan rating.
* Mass: core (material and the coolant in it) plus explicit allowances, `mass_breakdown()`:
  ducts `mass_duct_per_area_kg_m2` x core face area; fans rating / `specific_power_fan_W_kg`; pump, pipes and
  loop coolant `mass_pump_pipes_kg` (fixed). The allowances are the author's estimates (plan 042).
"""
from dataclasses import dataclass, field, replace
from typing import Any

from aircraft_closure.core.margins import Margin, margin_below
from aircraft_closure.powertrain.components.compact_heat_exchanger import (
    FluidProperties, OffsetStripFinCore, water_glycol_properties)
from aircraft_closure.powertrain.components.ram_air_duct import RamAirDuct

kelvin_offset_K = 273.15


@dataclass(frozen=True)
class NtuHeatExchangerResult:
    mass_flow_air_kg_s: Any
    pressure_drop_Pa: Any               # core, air side
    power_pumping_W: Any                # flow work lost in the core, ṁ Δp_core / ρ_face (reported)
    drag_N: Any                         # duct drag, -net force (negative: Meredith thrust)
    power_fan_W: Any                    # electrical
    delta_temperature_C: Any            # coolant (exchanger outlet) minus air at the core inlet
    residual_mass_flow_kg_s: Any        # constrain to zero
    conductance_effective_W_K: Any      # ε C_min at this flow
    capacity_coolant_W_K: Any
    pressure_drop_coolant_Pa: Any
    effectiveness: Any
    velocity_exit_m_s: Any


@dataclass(frozen=True)
class NtuHeatExchangerLimits:
    power_rated_fan_W: Any


@dataclass(frozen=True)
class NtuMassBreakdown:
    mass_core_kg: Any
    mass_ducts_kg: Any
    mass_fans_kg: Any
    mass_pump_pipes_kg: Any

    def total_kg(self):
        return self.mass_core_kg + self.mass_ducts_kg + self.mass_fans_kg + self.mass_pump_pipes_kg


@dataclass(frozen=True)
class NtuRamAirHeatExchanger:
    core: Any = field(default_factory=OffsetStripFinCore)
    duct: Any = field(default_factory=RamAirDuct)
    power_rated_fan_W: Any = 10e3
    temperature_coolant_C: Any = 60.0
    mass_flow_coolant_kg_s: Any = 2.0
    specific_power_fan_W_kg: Any = 2000.0         # electric fan, motor and controller
    mass_duct_per_area_kg_m2: Any = 12.0          # inlet, diffuser and nozzle per m2 of core face
    mass_pump_pipes_kg: Any = 30.0                # pump, lines to the tips and back, loop coolant outside the core
    power_heat_scale_W: Any = 1e5                 # normalizes the heat-rejection margin
    coolant: Any = field(default_factory=water_glycol_properties)

    def mass_breakdown(self):
        return NtuMassBreakdown(mass_core_kg=self.core.get_mass(),
                                mass_ducts_kg=self.mass_duct_per_area_kg_m2 * self.core.area_frontal_m2(),
                                mass_fans_kg=self.power_rated_fan_W / self.specific_power_fan_W_kg,
                                mass_pump_pipes_kg=self.mass_pump_pipes_kg)

    def get_mass(self):
        return self.mass_breakdown().total_kg()

    def get_limits(self):
        return NtuHeatExchangerLimits(self.power_rated_fan_W)

    def evaluate(self, power_heat_W, atmosphere, velocity_m_s=0.0, fan=False, mass_flow_air_kg_s=None,
                 pressure_rise_fan_Pa=0.0):
        """`mass_flow_air_kg_s` (required) and `pressure_rise_fan_Pa` are the caller's per-point unknowns; `fan`
        is accepted for the common interface (the fan acts through `pressure_rise_fan_Pa`)."""
        if mass_flow_air_kg_s is None:
            raise ValueError("The ε-NTU exchanger needs the point's air mass flow (an Opti variable).")
        temperature_air_K, _, density_face_kg_m3 = self.duct.inlet_state(atmosphere, velocity_m_s)
        air = FluidProperties(specific_heat_J_kg_K=self.duct.specific_heat_J_kg_K, conductivity_W_m_K=0.0259,
                              viscosity_Pa_s=atmosphere.dynamic_viscosity(), density_kg_m3=density_face_kg_m3)
        temperature_coolant_K = self.temperature_coolant_C + kelvin_offset_K
        # ε C_min (= ε UA / NTU) does not depend on the inlet temperatures; the margin sets the coolant inlet.
        core = self.core.evaluate(mass_flow_air_kg_s, self.mass_flow_coolant_kg_s, temperature_air_K,
                                  temperature_coolant_K, air, self.coolant)
        conductance_effective_W_K = core.effectiveness * core.conductance_W_K / core.ntu
        duct = self.duct.evaluate(mass_flow_air_kg_s, power_heat_W, core.air.pressure_drop_Pa,
                                  self.core.area_frontal_m2(), atmosphere, velocity_m_s, pressure_rise_fan_Pa)
        return NtuHeatExchangerResult(
            mass_flow_air_kg_s=mass_flow_air_kg_s, pressure_drop_Pa=core.air.pressure_drop_Pa,
            power_pumping_W=mass_flow_air_kg_s * core.air.pressure_drop_Pa / density_face_kg_m3,
            drag_N=duct.drag_N, power_fan_W=duct.power_fan_W,
            delta_temperature_C=temperature_coolant_K - temperature_air_K,
            residual_mass_flow_kg_s=duct.residual_mass_flow_kg_s,
            conductance_effective_W_K=conductance_effective_W_K,
            capacity_coolant_W_K=self.mass_flow_coolant_kg_s * self.coolant.specific_heat_J_kg_K,
            pressure_drop_coolant_Pa=core.coolant.pressure_drop_Pa, effectiveness=core.effectiveness,
            velocity_exit_m_s=duct.velocity_exit_m_s)

    def power_heat_capacity_W(self, power_heat_W, result):
        """Heat the core rejects with the coolant entering at T_c + Q / C_coolant (Q = `power_heat_W`)."""
        return result.conductance_effective_W_K * (result.delta_temperature_C
                                                   + power_heat_W / result.capacity_coolant_W_K)

    def margins(self, power_heat_end_W, result, label):
        """(heat demand reported as `power_heat_equivalent_W`, margins): heat rejection and fan rating."""
        margin_heat = (result.conductance_effective_W_K * result.delta_temperature_C
                       - power_heat_end_W * (1 - result.conductance_effective_W_K / result.capacity_coolant_W_K))
        return power_heat_end_W, (
            Margin(f"{label}: heat_exchanger heat_rejection", margin_heat / self.power_heat_scale_W),
            margin_below(f"{label}: heat_exchanger power_fan_W", result.power_fan_W, self.power_rated_fan_W))


def with_core_counts(exchanger, count_wide, count_long, count_tall, area_nozzle_m2, power_rated_fan_W):
    """The exchanger with the sizing's design variables in place."""
    return replace(exchanger, core=replace(exchanger.core, count_wide=count_wide, count_long=count_long,
                                           count_tall=count_tall),
                   duct=replace(exchanger.duct, area_nozzle_m2=area_nozzle_m2), power_rated_fan_W=power_rated_fan_W)


if __name__ == "__main__":
    import aerosandbox as asb
    exchanger = NtuRamAirHeatExchanger(core=OffsetStripFinCore(count_wide=600.0, count_long=15.0, count_tall=40.0),
                                       duct=RamAirDuct(area_nozzle_m2=0.15))
    hot_day = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
    opti = asb.Opti()
    mass_flow_kg_s = opti.variable(init_guess=4.0, lower_bound=0.05)
    pressure_rise_fan_Pa = opti.variable(init_guess=500.0, scale=500.0, lower_bound=0.0)
    result = exchanger.evaluate(80e3, hot_day, 0.0, True, mass_flow_kg_s, pressure_rise_fan_Pa)
    _, margins = exchanger.margins(80e3, result, "hot-day hover")
    opti.subject_to([result.residual_mass_flow_kg_s == 0, margins[0].value >= 0])
    opti.minimize(result.power_fan_W / 1e3)
    s = opti.solve(verbose=False)
    print(f"mass {exchanger.get_mass():.1f} kg {exchanger.mass_breakdown()}; hot-day 80 kW: flow "
          f"{s(mass_flow_kg_s):.2f} kg/s, fan {s(result.power_fan_W) / 1e3:.2f} kW, dp fan {s(pressure_rise_fan_Pa):.0f} Pa")
