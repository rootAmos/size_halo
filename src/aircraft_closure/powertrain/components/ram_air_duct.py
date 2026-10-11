"""Ram-air cooling duct: inlet, diffuser, heat exchanger core, optional fan, nozzle (plan 042).

One-dimensional, steady, ideal gas; stations follow OpenConcept's compressible duct (Brelje et al.) with the
loss terms of Kellermann et al. (Aerospace 8(1), 3, 2021) reduced to coefficients:

* freestream total state Tt0 = T + V^2 / (2 cp), pt0 = p (Tt0 / T)^(γ/(γ-1));
* inlet: the ram rise pt0 - p is recovered with `recovery_inlet`;
* diffuser to the core face: loss `loss_diffuser` x face dynamic pressure (G_face^2 / (2 ρ_face));
* core: total-pressure drop from the core model at this mass flow; heat Q raises Tt by Q / (ṁ cp);
* fan (hover): total-pressure rise `pressure_rise_fan_Pa`; electrical power ṁ Δp_fan / (ρ_face η_fan);
* nozzle: expansion from the core-exit total state to ambient static pressure, velocity coefficient
  `coefficient_velocity_nozzle`; flow ρ_e V_e A_nozzle;
* net force F = ṁ (V_e - V) (exit at ambient pressure, subsonic); drag = -F.

Heat addition raises V_e at a given flow, so it recovers thrust (the Meredith effect) with no separate term.

The mass flow is not solved here: `evaluate` returns the residual (nozzle flow - ṁ) for the caller's
`asb.Opti` to drive to zero, one variable per operating point. The expansion term is kept positive with a
smooth floor, so a point whose duct cannot pass the flow shows up as a large residual, not a NaN.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox.numpy as np


@dataclass(frozen=True)
class DuctResult:
    residual_mass_flow_kg_s: Any        # nozzle flow minus the assumed flow; constrain to zero
    force_net_N: Any                    # forward positive
    drag_N: Any
    power_fan_W: Any                    # electrical
    velocity_exit_m_s: Any
    mach_exit: Any
    pressure_total_core_exit_Pa: Any
    temperature_total_exit_K: Any


@dataclass(frozen=True)
class RamAirDuct:
    area_nozzle_m2: Any = 0.1
    recovery_inlet: Any = 0.95          # fraction of the ram rise kept (flush or scoop inlet)
    loss_diffuser: Any = 0.3            # x core-face dynamic pressure
    coefficient_velocity_nozzle: Any = 0.98
    efficiency_fan: Any = 0.55          # fan and motor, electrical to flow power
    ratio_specific_heats: Any = 1.4
    specific_heat_J_kg_K: Any = 1005.0
    gas_constant_J_kg_K: Any = 287.05

    def inlet_state(self, atmosphere, velocity_m_s=0.0):
        """Air at the core face before the diffuser loss: (total temperature K, total pressure Pa, density kg/m3)."""
        g, cp, r = self.ratio_specific_heats, self.specific_heat_J_kg_K, self.gas_constant_J_kg_K
        temperature_K, pressure_Pa = atmosphere.temperature(), atmosphere.pressure()
        temperature_total_K = temperature_K + velocity_m_s**2 / (2 * cp)
        pressure_total_Pa = pressure_Pa * (temperature_total_K / temperature_K)**(g / (g - 1))
        pressure_inlet_Pa = pressure_Pa + self.recovery_inlet * (pressure_total_Pa - pressure_Pa)
        return temperature_total_K, pressure_inlet_Pa, pressure_inlet_Pa / (r * temperature_total_K)

    def evaluate(self, mass_flow_kg_s, power_heat_W, pressure_drop_core_Pa, area_face_m2, atmosphere,
                 velocity_m_s=0.0, pressure_rise_fan_Pa=0.0):
        g, cp, r = self.ratio_specific_heats, self.specific_heat_J_kg_K, self.gas_constant_J_kg_K
        exponent = (g - 1) / g
        pressure_Pa = atmosphere.pressure()
        temperature_total_K, pressure_inlet_Pa, density_face_kg_m3 = self.inlet_state(atmosphere, velocity_m_s)
        pressure_dynamic_face_Pa = (mass_flow_kg_s / area_face_m2)**2 / (2 * density_face_kg_m3)
        pressure_exit_total_Pa = (pressure_inlet_Pa - self.loss_diffuser * pressure_dynamic_face_Pa
                                  - pressure_drop_core_Pa + pressure_rise_fan_Pa)
        temperature_exit_total_K = temperature_total_K + power_heat_W / (mass_flow_kg_s * cp)
        expansion = 1 - (pressure_Pa / pressure_exit_total_Pa)**exponent
        expansion = np.softmax(expansion, 1e-6, softness=1e-6)
        velocity_exit_m_s = (self.coefficient_velocity_nozzle
                             * np.sqrt(2 * cp * temperature_exit_total_K * expansion))
        temperature_exit_K = temperature_exit_total_K - velocity_exit_m_s**2 / (2 * cp)
        density_exit_kg_m3 = pressure_Pa / (r * temperature_exit_K)
        mass_flow_nozzle_kg_s = density_exit_kg_m3 * velocity_exit_m_s * self.area_nozzle_m2
        force_net_N = mass_flow_kg_s * (velocity_exit_m_s - velocity_m_s)
        return DuctResult(
            residual_mass_flow_kg_s=mass_flow_nozzle_kg_s - mass_flow_kg_s, force_net_N=force_net_N,
            drag_N=-force_net_N,
            power_fan_W=mass_flow_kg_s * pressure_rise_fan_Pa / (density_face_kg_m3 * self.efficiency_fan),
            velocity_exit_m_s=velocity_exit_m_s,
            mach_exit=velocity_exit_m_s / np.sqrt(g * r * temperature_exit_K),
            pressure_total_core_exit_Pa=pressure_exit_total_Pa, temperature_total_exit_K=temperature_exit_total_K)


if __name__ == "__main__":
    import aerosandbox as asb
    duct = RamAirDuct(area_nozzle_m2=0.08)
    cruise = asb.Atmosphere(altitude=3048.0)
    opti = asb.Opti()
    mass_flow_kg_s = opti.variable(init_guess=5.0, lower_bound=0.1)
    result = duct.evaluate(mass_flow_kg_s, 80e3, 800.0 * (mass_flow_kg_s / 6.0)**1.8, 0.45, cruise, 85.0)
    opti.subject_to(result.residual_mass_flow_kg_s == 0)
    solution = opti.solve(verbose=False)
    print(f"cruise 85 m/s, 80 kW: flow {solution(mass_flow_kg_s):.2f} kg/s, exit {solution(result.velocity_exit_m_s):.1f}"
          f" m/s, drag {solution(result.drag_N):.0f} N")
