"""Plan 049: the trajectory flies the plan 042 ε-NTU cooler (per-node air flow, duct residual, fan, duct force)."""
import unittest
from types import SimpleNamespace

import aerosandbox as asb
import numpy as np

from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger
from aircraft_closure.thermal.heat import thermal_parameters
from examples.halo_sizing import HaloAssumptions, HaloDesign, HaloRequirements
from examples.trajectory_optimization import halo_trajectory_case, solve_min_energy_transition

# Plan 042's converged NTU reference (14,297 lb, `heat_exchanger_model="ntu"`, other assumptions default), so the
# test needs no sizing solve.
design_ntu_reference = HaloDesign(
    x_le_wing_m=4.597078, area_wing_m2=22.207214, area_horizontal_tail_m2=5.029656, area_vertical_tail_m2=2.643105,
    torque_peak_motor_Nm=1594.9527, torque_peak_generator_Nm=614.45409, power_rated_turboshaft_W=835183.69,
    mass_turboshaft_bare_kg=184.20688, power_max_discharge_battery_W=739340.36, energy_capacity_battery_J=219106810.6,
    area_disk_m2=69.125849, mass_fuel_kg=908.69788, solidity=0.06, speed_tip_m_s=238.20588,
    speed_peak_motor_rad_s=176.64856, reduction_ratio=4.1417839, speed_peak_generator_rad_s=869.08655,
    count_parallel_battery=19.958438, speed_rotor_wing_design_rad_s=56.660545, power_rated_heat_exchanger_W=None,
    power_rated_gearbox_W=713717.81, count_wide_heat_exchanger=707.91197, count_long_heat_exchanger=11.141349,
    count_tall_heat_exchanger=21.697078, area_nozzle_heat_exchanger_m2=0.082174126,
    power_rated_fan_heat_exchanger_W=8552.3309)
mass_takeoff_ntu_reference_kg = 6484.9864


class TrajectoryNtuCoolingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assumptions = HaloAssumptions(heat_exchanger_model="ntu")
        sizing = SimpleNamespace(design=design_ntu_reference, mass_takeoff_kg=mass_takeoff_ntu_reference_kg)
        cls.case = halo_trajectory_case(sizing, HaloRequirements(), cls.assumptions)
        cls.exchanger = cls.case.model.aircraft.powertrain.cooling.heat_exchanger
        cls.transition = solve_min_energy_transition(cls.case, count_nodes=21)

    def test_installed_cooler_is_the_sized_ntu_one(self):
        self.assertIsInstance(self.exchanger, NtuRamAirHeatExchanger)
        self.assertAlmostEqual(self.exchanger.duct.area_nozzle_m2, 0.082174126)
        self.assertAlmostEqual(self.exchanger.power_rated_fan_W, 8552.3309)

    def _exchanger_at_nodes(self):
        """The cooler re-evaluated numerically at the solved nodes, with the heat from the temperature states."""
        r = self.transition
        instances = self.case.model.aircraft.powertrain.topology.instances
        excluded = self.case.model.aircraft.powertrain.cooling.sources_excluded
        heat_W = 0.0
        for name, temperature in r.temperatures_C.items():
            if name not in excluded:
                p = thermal_parameters(instances[name].component)
                coolant_C = instances[name].component.thermal_model.temperature_coolant_C
                heat_W = heat_W + instances[name].count * (temperature - coolant_C) / p.resistance_K_W
        result = self.exchanger.evaluate(heat_W, asb.Atmosphere(altitude=r.altitude_m), r.velocity_m_s, True,
                                         r.mass_flow_air_kg_s, r.pressure_rise_fan_Pa)
        return heat_W, result

    def test_duct_flow_and_heat_rejection_at_every_node(self):
        r = self.transition
        heat_W, result = self._exchanger_at_nodes()
        self.assertTrue(np.all(r.mass_flow_air_kg_s > 0))
        np.testing.assert_allclose(result.residual_mass_flow_kg_s / r.mass_flow_air_kg_s, 0.0, atol=1e-6)
        _, margins = self.exchanger.margins(heat_W, result, "node")
        for margin in margins[:3]:                      # heat rejection, fan rating, core-exit pressure
            self.assertTrue(np.all(margin.value >= -1e-6), margin.label)
        self.assertGreater(np.max(heat_W), 10e3)        # the machines warm from the cold start
        np.testing.assert_allclose(r.power_fan_W, result.power_fan_W, rtol=1e-6, atol=1e-6)
        for name, temperature in r.temperatures_C.items():
            thermal = self.case.model.instance(name).thermal_model
            self.assertAlmostEqual(temperature[0], thermal.temperature_coolant_C, places=6)
            self.assertTrue(np.all(temperature <= thermal.temperature_max_C + 1e-6))

    def test_fan_at_low_speed_ram_air_wing_borne(self):
        r = self.transition
        pressure_dynamic_Pa = 0.5 * asb.Atmosphere(altitude=r.altitude_m).density() * r.velocity_m_s**2
        slow = (r.velocity_m_s < 6.0) & (np.arange(len(r.velocity_m_s)) > 0)
        # Near hover the ram rise cannot drive the flow: the fan supplies more pressure than the ram.
        self.assertTrue(np.all(r.pressure_rise_fan_Pa[slow] > pressure_dynamic_Pa[slow]))
        # Wing-borne the ram drives it and the fan (which costs bus energy) is off.
        self.assertLess(r.power_fan_W[-1], 1e-3 * self.exchanger.power_rated_fan_W)

    def test_duct_force(self):
        r = self.transition
        _, result = self._exchanger_at_nodes()
        # Wing-borne, fan off: the full momentum-balance drag along the fuselage axis (plan 042 cruise: about 53 N).
        self.assertAlmostEqual(r.drag_cooling_N[-1], result.drag_N[-1] * np.cos(np.radians(r.alpha_deg[-1])),
                               delta=1e-3 * abs(result.drag_N[-1]))
        self.assertGreater(r.drag_cooling_N[-1], 20.0)
        self.assertLess(r.drag_cooling_N[-1], 200.0)
        # Hover: the duct force is not counted (as the sizing's hover points).
        self.assertLess(abs(r.drag_cooling_N[0]), 1.0)

    def test_bus_energy_includes_the_fan(self):
        r = self.transition
        power_W = r.power_bus_W + r.power_fan_W
        energy_J = np.sum(0.5 * (power_W[1:] + power_W[:-1]) * np.diff(r.time_s))
        self.assertAlmostEqual(r.energy_bus_J[-1] / energy_J, 1.0, places=6)

    def test_transition_flies(self):
        r = self.transition
        self.assertAlmostEqual(r.tilt_deg[0], 90.0, places=4)
        self.assertAlmostEqual(r.tilt_deg[-1], 0.0, places=4)
        self.assertGreater(r.energy_bus_J[-1], 0.0)
        self.assertGreater(r.duration_s, 5.0)


if __name__ == "__main__":
    unittest.main()
