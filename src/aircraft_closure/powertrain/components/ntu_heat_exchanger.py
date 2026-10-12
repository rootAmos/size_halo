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
* Duct margins: the core-exit total pressure must exceed ambient (otherwise the duct's smooth floors would let
  a vanishing exit velocity "pass" the flow through an unbounded nozzle), and the nozzle is no larger than the
  core face.
* Mass: core (material and the coolant in it) plus explicit allowances, `mass_breakdown()`:
  ducts `mass_duct_per_area_kg_m2` x core face area; fans rating / `specific_power_fan_W_kg`; pump, pipes and
  loop coolant `mass_pump_pipes_kg` (fixed). The allowances are the author's estimates (plan 042).
* `allowances="published"` (plan 051) replaces the pump-and-pipes estimate with OpenConcept's published models
  (Brelje et al., NATO STO-MP-AVT-323-21; `openconcept/thermal/pump.py` and `hose.py`):
  - pump: `power_rated_pump_W` / `specific_power_pump_W_kg` (OpenConcept: 1/450 kg per W of rated electrical
    power). Default rating 1.7 kW: 2.0 kg/s of water-glycol through about 150 kPa in the core (plan 042's solved
    153 kPa) and 154 kPa in 26 m of 25.4 mm line (Darcy with Blasius' friction factor, 3.8 m/s, Re 6e4), at
    OpenConcept's pump efficiency 0.35. Chapman et al. (NASA/TM-2020-220473, Table 4) give 2.4 kg for a 1.18 kg/s
    coolant pump, the same order;
  - lines: `length_lines_m` of hose with wall thickness t = D p_design / (2 σ) (thin-wall hoop stress), wall mass
    (D + t) π t ρ_hose plus the coolant π D² / 4 ρ_coolant per metre (OpenConcept defaults: D 25.4 mm, p_design
    1.03 MPa, σ 2.07 MPa, ρ_hose 1,356 kg/m³: 0.85 kg/m wall and 0.53 kg/m coolant). Chapman et al. count only the
    coolant in their lines (wall mass neglected), so OpenConcept's is the more complete published model.
  Ducts and fans keep the estimates: Kellermann et al. 2021 do not publish their duct or fan masses, Chapman et al.
  neglect inlet and nozzle masses (they assume these scale with the core), OpenConcept carries none, and Chapman's
  fan correlation (mass from air flow, fitted to small cooling fans) is not credible at tens of kW (plan 051).
"""
import math
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
    pressure_total_exit_Pa: Any         # core exit total pressure (with the fan), before the nozzle
    pressure_ambient_Pa: Any


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
    allowances: str = "estimate"                  # "estimate" (plan 042) or "published" (plan 051: pump and lines)
    power_rated_pump_W: Any = 1700.0              # "published": electrical rating
    specific_power_pump_W_kg: Any = 450.0         # "published": OpenConcept SimplePump, 1/450 kg/W
    length_lines_m: Any = 26.0                    # "published": supply and return to both tips
    diameter_line_m: Any = 0.0254                 # "published": OpenConcept SimpleHose defaults from here on
    pressure_design_line_Pa: Any = 1.03e6
    stress_allowable_line_Pa: Any = 2.07e6
    density_line_kg_m3: Any = 1356.3
    power_heat_scale_W: Any = 1e5                 # normalizes the heat-rejection margin
    coolant: Any = field(default_factory=water_glycol_properties)

    def mass_lines_per_length_kg_m(self):
        """Hose wall plus the coolant in it, per metre (OpenConcept `SimpleHose`)."""
        diameter_m = self.diameter_line_m
        thickness_m = diameter_m * self.pressure_design_line_Pa / (2 * self.stress_allowable_line_Pa)
        return ((diameter_m + thickness_m) * math.pi * thickness_m * self.density_line_kg_m3
                + math.pi * diameter_m**2 / 4 * self.coolant.density_kg_m3)

    def mass_pump_pipes_published_kg(self):
        return (self.power_rated_pump_W / self.specific_power_pump_W_kg
                + self.length_lines_m * self.mass_lines_per_length_kg_m())

    def mass_breakdown(self):
        if self.allowances == "estimate":
            mass_pump_pipes_kg = self.mass_pump_pipes_kg
        elif self.allowances == "published":
            mass_pump_pipes_kg = self.mass_pump_pipes_published_kg()
        else:
            raise ValueError(f"Unknown allowances '{self.allowances}' (\"estimate\" or \"published\").")
        return NtuMassBreakdown(mass_core_kg=self.core.get_mass(),
                                mass_ducts_kg=self.mass_duct_per_area_kg_m2 * self.core.area_frontal_m2(),
                                mass_fans_kg=self.power_rated_fan_W / self.specific_power_fan_W_kg,
                                mass_pump_pipes_kg=mass_pump_pipes_kg)

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
            velocity_exit_m_s=duct.velocity_exit_m_s, pressure_total_exit_Pa=duct.pressure_total_core_exit_Pa,
            pressure_ambient_Pa=atmosphere.pressure())

    def power_heat_capacity_W(self, power_heat_W, result):
        """Heat the core rejects with the coolant entering at T_c + Q / C_coolant (Q = `power_heat_W`)."""
        return result.conductance_effective_W_K * (result.delta_temperature_C
                                                   + power_heat_W / result.capacity_coolant_W_K)

    def margins(self, power_heat_end_W, result, label):
        """(heat demand reported as `power_heat_equivalent_W`, margins): heat rejection, fan rating, the duct
        exit total pressure above ambient (the nozzle can expel the flow: no reverse expansion) and the nozzle no
        larger than the core face."""
        margin_heat = (result.conductance_effective_W_K * result.delta_temperature_C
                       - power_heat_end_W * (1 - result.conductance_effective_W_K / result.capacity_coolant_W_K))
        return power_heat_end_W, (
            Margin(f"{label}: heat_exchanger heat_rejection", margin_heat / self.power_heat_scale_W),
            margin_below(f"{label}: heat_exchanger power_fan_W", result.power_fan_W, self.power_rated_fan_W),
            Margin(f"{label}: heat_exchanger pressure_exit_Pa",
                   (result.pressure_total_exit_Pa - result.pressure_ambient_Pa) / 1e3),
            margin_below(f"{label}: heat_exchanger area_nozzle_m2", self.duct.area_nozzle_m2,
                         self.core.area_frontal_m2()))


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
