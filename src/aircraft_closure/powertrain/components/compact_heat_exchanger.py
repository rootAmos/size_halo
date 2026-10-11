"""Offset-strip-fin plate-fin heat exchanger by the ε-NTU method (plan 042).

Cross flow, both fluids unmixed: air (cold side) through the frontal face, coolant (hot side) across the
width. Geometry and correlations follow Brelje et al. (OpenConcept `thermal/heat_exchanger.py`, NATO
STO-MP-AVT-323-21); j and f are Manglik and Bergles (Exp. Therm. Fluid Sci. 10, 1995); ε-NTU and the
entrance and exit loss coefficients are Kays and London.

The core size is set by three continuous counts, so it is a set of smooth design variables:
`count_wide` (air channels across the face), `count_long` (fin lengths along the air flow), `count_tall`
(layers, each one air and one coolant passage).

* face height H = count_tall (2 (t_fin + t_plate) + h_air + h_coolant), width W = count_wide (t_fin + w_air),
  air flow length L = count_long l_air; the coolant flows across W;
* j = 0.6522 Re^-0.5403 α^-0.1541 δ^0.1499 γ^-0.0678 (1 + 5.269e-5 Re^1.34 α^0.504 δ^0.456 γ^-1.055)^0.1,
  f = 9.6243 Re^-0.7422 α^-0.1856 δ^0.3053 γ^-0.2659 (1 + 7.669e-8 Re^4.429 α^0.920 δ^3.767 γ^0.236)^0.1,
  with α = w/h, δ = t_fin/l, γ = t_fin/w (valid for Re 120 to 10^4);
* h = j Re Pr^(1/3) k / d_h; fin efficiency η_f = tanh(m h/2) / (m h/2), m = sqrt(2 h_conv / (k_fin t_fin));
  surface efficiency η_o = 1 - (A_fin/A)(1 - η_f);
* UA = 1 / (1/(η_o h A)_air + R_foul,air + 1/(η_o h A)_coolant + R_foul,coolant);
* ε = 1 - exp(NTU^0.22 / C_r (exp(-C_r NTU^0.78) - 1)), NTU = UA / C_min (C_min, C_max by a smooth min and max);
* heat Q = ε C_min (T_coolant,in - T_air,in);
* pressure drop per side Δp = G² / (2 ρ) (K_c + 4 f L_flow / d_h + K_e);
* mass: core material (plates, fins, case) plus the coolant held in the core.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class FluidProperties:
    specific_heat_J_kg_K: Any
    conductivity_W_m_K: Any
    viscosity_Pa_s: Any
    density_kg_m3: Any

    def prandtl(self):
        return self.viscosity_Pa_s * self.specific_heat_J_kg_K / self.conductivity_W_m_K


def air_properties(atmosphere, density_kg_m3=None):
    """Air at the atmosphere's static temperature (cp and k constant; viscosity from AeroSandbox)."""
    return FluidProperties(specific_heat_J_kg_K=1005.0, conductivity_W_m_K=0.0259,
                           viscosity_Pa_s=atmosphere.dynamic_viscosity(),
                           density_kg_m3=atmosphere.density() if density_kg_m3 is None else density_kg_m3)


def water_glycol_properties():
    """50 % ethylene glycol near 60 C (OpenConcept's values; density from ASHRAE tables)."""
    return FluidProperties(specific_heat_J_kg_K=3801.0, conductivity_W_m_K=0.405, viscosity_Pa_s=1.68e-3,
                           density_kg_m3=1045.0)


def offset_strip_fin_factors(reynolds, ratio_aspect, ratio_fin_length, ratio_fin_spacing):
    """Manglik and Bergles j and f for rectangular offset strip fins."""
    a, d, g = ratio_aspect, ratio_fin_length, ratio_fin_spacing
    colburn = (0.6522 * reynolds**-0.5403 * a**-0.1541 * d**0.1499 * g**-0.0678
               * (1 + 5.269e-5 * reynolds**1.340 * a**0.504 * d**0.456 * g**-1.055)**0.1)
    fanning = (9.6243 * reynolds**-0.7422 * a**-0.1856 * d**0.3053 * g**-0.2659
               * (1 + 7.669e-8 * reynolds**4.429 * a**0.920 * d**3.767 * g**0.236)**0.1)
    return colburn, fanning


def effectiveness_cross_flow_unmixed(ntu, ratio_capacity):
    """Kays and London's approximation for cross flow with both fluids unmixed."""
    return 1 - np.exp(ntu**0.22 / ratio_capacity * (np.exp(-ratio_capacity * ntu**0.78) - 1))


@dataclass(frozen=True)
class CorePassage:
    """One side's passages: flow area, heat-transfer area, hydraulic diameter and fin ratios."""
    area_flow_m2: Any
    area_heat_m2: Any
    diameter_hydraulic_m: Any
    length_flow_m: Any
    fraction_area_fin: Any
    ratio_aspect: Any
    ratio_fin_length: Any
    ratio_fin_spacing: Any
    height_fin_m: Any


@dataclass(frozen=True)
class SideResult:
    reynolds: Any
    colburn: Any
    fanning: Any
    coefficient_W_m2_K: Any
    efficiency_surface: Any
    pressure_drop_Pa: Any


@dataclass(frozen=True)
class CoreResult:
    power_heat_W: Any
    effectiveness: Any
    ntu: Any
    conductance_W_K: Any
    temperature_air_out_K: Any
    temperature_coolant_out_K: Any
    air: SideResult
    coolant: SideResult


@dataclass(frozen=True)
class OffsetStripFinCore:
    count_wide: Any = 400.0
    count_long: Any = 3.0
    count_tall: Any = 20.0
    width_channel_air_m: Any = 1.35e-3
    height_channel_air_m: Any = 14e-3
    length_fin_air_m: Any = 6e-3
    width_channel_coolant_m: Any = 1e-3
    height_channel_coolant_m: Any = 1e-3
    length_fin_coolant_m: Any = 6e-3
    thickness_fin_m: Any = 0.102e-3
    thickness_plate_m: Any = 0.2e-3
    thickness_case_m: Any = 2e-3
    density_material_kg_m3: Any = 2700.0      # aluminium
    conductivity_material_W_m_K: Any = 190.0
    resistance_fouling_air_m2_K_W: Any = 0.0
    resistance_fouling_coolant_m2_K_W: Any = 0.0
    loss_entrance: Any = 0.3                  # K_c
    loss_exit: Any = -0.1                     # K_e (pressure recovery)
    density_coolant_kg_m3: Any = 1045.0

    def height_m(self):
        return self.count_tall * (2 * (self.thickness_fin_m + self.thickness_plate_m)
                                  + self.height_channel_air_m + self.height_channel_coolant_m)

    def width_m(self):
        return self.count_wide * (self.thickness_fin_m + self.width_channel_air_m)

    def length_m(self):
        return self.count_long * self.length_fin_air_m

    def area_frontal_m2(self):
        return self.width_m() * self.height_m()

    def passage_air(self):
        w, h, l, t = (self.width_channel_air_m, self.height_channel_air_m, self.length_fin_air_m,
                      self.thickness_fin_m)
        return CorePassage(
            area_flow_m2=w * h * self.count_wide * self.count_tall,
            area_heat_m2=2 * (w + h) * l * self.count_long * self.count_wide * self.count_tall,
            diameter_hydraulic_m=4 * w * h * l / (2 * (w * l + h * l + t * h) + t * w),
            length_flow_m=self.length_m(), fraction_area_fin=h / (h + w), ratio_aspect=w / h,
            ratio_fin_length=t / l, ratio_fin_spacing=t / w, height_fin_m=h)

    def passage_coolant(self):
        w, h, l, t = (self.width_channel_coolant_m, self.height_channel_coolant_m, self.length_fin_coolant_m,
                      self.thickness_fin_m)
        count_across = self.length_m() / (w + t)      # coolant channels side by side along the air flow
        count_along = self.width_m() / l               # fin lengths along the coolant flow
        return CorePassage(
            area_flow_m2=w * h * count_across * self.count_tall,
            area_heat_m2=2 * (w + h) * l * count_along * count_across * self.count_tall,
            diameter_hydraulic_m=4 * w * h * l / (2 * (w * l + h * l + t * h) + t * w),
            length_flow_m=self.width_m(), fraction_area_fin=h / (h + w), ratio_aspect=w / h,
            ratio_fin_length=t / l, ratio_fin_spacing=t / w, height_fin_m=h)

    def get_mass(self):
        """Plates, fins and case in the core material, plus the coolant in the core."""
        length_m, width_m, height_m = self.length_m(), self.width_m(), self.height_m()
        t = self.thickness_fin_m
        coolant = self.passage_coolant()
        volume_plate_m3 = length_m * width_m * 2 * self.thickness_plate_m * self.count_tall
        # Fin material: each passage's wetted perimeter (one wall per channel) x fin thickness x flow length.
        volume_fin_m3 = ((self.width_channel_air_m + self.height_channel_air_m + t) * t * length_m
                         * self.count_wide * self.count_tall
                         + (self.width_channel_coolant_m + self.height_channel_coolant_m + t) * t * width_m
                         * coolant.area_flow_m2 / (self.width_channel_coolant_m * self.height_channel_coolant_m))
        volume_case_m3 = self.thickness_case_m * length_m * 2 * (height_m + width_m)
        mass_coolant_kg = coolant.area_flow_m2 * width_m * self.density_coolant_kg_m3
        return self.density_material_kg_m3 * (volume_plate_m3 + volume_fin_m3 + volume_case_m3) + mass_coolant_kg

    def side(self, passage, mass_flow_kg_s, fluid):
        flux_kg_m2_s = mass_flow_kg_s / passage.area_flow_m2
        reynolds = flux_kg_m2_s * passage.diameter_hydraulic_m / fluid.viscosity_Pa_s
        colburn, fanning = offset_strip_fin_factors(reynolds, passage.ratio_aspect, passage.ratio_fin_length,
                                                    passage.ratio_fin_spacing)
        coefficient_W_m2_K = (colburn * reynolds * fluid.prandtl()**(1 / 3) * fluid.conductivity_W_m_K
                              / passage.diameter_hydraulic_m)
        m_per_m = np.sqrt(2 * coefficient_W_m2_K / (self.conductivity_material_W_m_K * self.thickness_fin_m))
        x = m_per_m * passage.height_fin_m / 2
        efficiency_fin = np.tanh(x) / x
        efficiency_surface = 1 - passage.fraction_area_fin * (1 - efficiency_fin)
        pressure_drop_Pa = (flux_kg_m2_s**2 / (2 * fluid.density_kg_m3)
                            * (self.loss_entrance + 4 * fanning * passage.length_flow_m / passage.diameter_hydraulic_m
                               + self.loss_exit))
        return SideResult(reynolds=reynolds, colburn=colburn, fanning=fanning, coefficient_W_m2_K=coefficient_W_m2_K,
                          efficiency_surface=efficiency_surface, pressure_drop_Pa=pressure_drop_Pa)

    def evaluate(self, mass_flow_air_kg_s, mass_flow_coolant_kg_s, temperature_air_in_K, temperature_coolant_in_K,
                 air, coolant=None):
        """Heat rejected and both sides' pressure drops at the given flows and inlet temperatures."""
        coolant = water_glycol_properties() if coolant is None else coolant
        passage_air, passage_coolant = self.passage_air(), self.passage_coolant()
        side_air = self.side(passage_air, mass_flow_air_kg_s, air)
        side_coolant = self.side(passage_coolant, mass_flow_coolant_kg_s, coolant)
        conductance_W_K = 1 / (1 / (side_air.efficiency_surface * side_air.coefficient_W_m2_K * passage_air.area_heat_m2)
                               + self.resistance_fouling_air_m2_K_W / (side_air.efficiency_surface
                                                                       * passage_air.area_heat_m2)
                               + 1 / (side_coolant.efficiency_surface * side_coolant.coefficient_W_m2_K
                                      * passage_coolant.area_heat_m2)
                               + self.resistance_fouling_coolant_m2_K_W / (side_coolant.efficiency_surface
                                                                           * passage_coolant.area_heat_m2))
        capacity_air_W_K = mass_flow_air_kg_s * air.specific_heat_J_kg_K
        capacity_coolant_W_K = mass_flow_coolant_kg_s * coolant.specific_heat_J_kg_K
        capacity_min_W_K = np.softmin(capacity_air_W_K, capacity_coolant_W_K, softness=1e-3 * capacity_air_W_K)
        capacity_max_W_K = np.softmax(capacity_air_W_K, capacity_coolant_W_K, softness=1e-3 * capacity_air_W_K)
        ntu = conductance_W_K / capacity_min_W_K
        effectiveness = effectiveness_cross_flow_unmixed(ntu, capacity_min_W_K / capacity_max_W_K)
        power_heat_W = effectiveness * capacity_min_W_K * (temperature_coolant_in_K - temperature_air_in_K)
        return CoreResult(power_heat_W=power_heat_W, effectiveness=effectiveness, ntu=ntu,
                          conductance_W_K=conductance_W_K,
                          temperature_air_out_K=temperature_air_in_K + power_heat_W / capacity_air_W_K,
                          temperature_coolant_out_K=temperature_coolant_in_K - power_heat_W / capacity_coolant_W_K,
                          air=side_air, coolant=side_coolant)


if __name__ == "__main__":
    import aerosandbox as asb
    core = OffsetStripFinCore()
    atmosphere = asb.Atmosphere(altitude=1219.2, temperature_deviation=27.7)
    result = core.evaluate(4.0, 1.5, atmosphere.temperature(), 273.15 + 60.0, air_properties(atmosphere))
    print(f"core {core.width_m():.3f} x {core.height_m():.3f} x {core.length_m():.3f} m, {core.get_mass():.1f} kg; "
          f"Q {result.power_heat_W / 1e3:.1f} kW, eps {result.effectiveness:.3f}, NTU {result.ntu:.2f}, "
          f"dp air {result.air.pressure_drop_Pa:.0f} Pa, dp coolant {result.coolant.pressure_drop_Pa / 1e3:.1f} kPa, "
          f"Re air {result.air.reynolds:.0f}, Re coolant {result.coolant.reynolds:.0f}")
