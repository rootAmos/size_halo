"""Plan 055: the Halo whirl-flutter model built from a sized design, numeric (fast; the coupled solve is in
docs/decisions/055-whirl-flutter-in-the-loop.md)."""
import unittest
from dataclasses import replace

import aerosandbox.numpy as np
import numpy as onp

from aircraft_closure.aeroelasticity.whirl_flutter import (least_damped_eigenpairs, state_labels,
                                                           whirl_flutter_matrices)
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from examples.halo_sizing import (HaloAssumptions, HaloDesign, HaloRequirements, build_halo_aircraft,
                                  halo_whirl_check, halo_whirl_inputs, halo_whirl_system)

design = HaloDesign(
    x_le_wing_m=4.62, area_wing_m2=23.3, area_horizontal_tail_m2=5.25, area_vertical_tail_m2=2.73,
    torque_peak_motor_Nm=1617.0, torque_peak_generator_Nm=891.0, power_rated_turboshaft_W=835183.7,
    mass_turboshaft_bare_kg=184.2, power_max_discharge_battery_W=None, energy_capacity_battery_J=None,
    area_disk_m2=73.4, mass_fuel_kg=936.0, solidity=0.06, speed_tip_m_s=238.2, speed_peak_motor_rad_s=204.5,
    reduction_ratio=4.53, speed_peak_generator_rad_s=580.5, count_parallel_battery=22.95,
    speed_rotor_wing_design_rad_s=56.3, power_rated_heat_exchanger_W=1.27e5, power_rated_gearbox_W=7.57e5)
assumptions = replace(HaloAssumptions(), count_units_motor=2, count_units_generator=3)
condition = StructuralDesignCondition(mass_design_kg=6885.0, load_factor_ultimate=4.5, velocity_cruise_m_s=85.0,
                                      altitude_cruise_m=3000.0, lift_to_drag_cruise=9.0)


def inputs_for(design_):
    aircraft = build_halo_aircraft(design_, HaloRequirements(), assumptions)
    wing = aircraft.wing
    return halo_whirl_inputs(wing, wing.mass_model.masses(wing, condition),
                             wing.get_mass_properties(condition).mass,
                             float(np.sqrt(design_.area_disk_m2 / np.pi)), design_.solidity)


class HaloWhirlLoopTests(unittest.TestCase):
    def test_reference_class_wing_flutters_near_plan_040(self):
        """A v3.6-class wing: calibrated flutter speed in the plan 040 band (262-275 kt), wing beam critical."""
        (row,) = halo_whirl_check(inputs_for(design), [29.4], -18.0, assumptions)
        self.assertEqual(row["label"], "wing beam")
        self.assertGreater(row["velocity_flutter_kt"], 240.0)
        self.assertLess(row["velocity_flutter_kt"], 300.0)

    def test_stiffer_wing_design_speed_raises_flutter_speed(self):
        """The wing design rotor speed is the optimizer's stiffness lever."""
        low = halo_whirl_check(inputs_for(design), [29.4], -18.0, assumptions)[0]["velocity_flutter_kt"]
        stiff = replace(design, speed_rotor_wing_design_rad_s=70.0)
        high = halo_whirl_check(inputs_for(stiff), [29.4], -18.0, assumptions)[0]["velocity_flutter_kt"]
        self.assertGreater(high, low + 10.0)

    def test_least_damped_mode_at_the_check_speed_is_the_wing_beam(self):
        airframe, rotor = halo_whirl_system(inputs_for(design), 29.4, -18.0, assumptions)
        velocity_m_s = 1.15 * 1.25 * HaloRequirements().velocity_max_m_s / 0.84
        pairs = least_damped_eigenpairs(*whirl_flutter_matrices(airframe, rotor, velocity_m_s, 29.4, 1.225), count=2,
                                        labels=state_labels(airframe, rotor))
        self.assertLess(pairs[0].damping_ratio, 0.0)          # this wing flutters below 1.15 V_D
        self.assertTrue(onp.isfinite(pairs[0].omega))


if __name__ == "__main__":
    unittest.main()


class WarmStartRotorSpeedTests(unittest.TestCase):
    def test_node_labels_fall_back_to_their_segment(self):
        """Plan 060 nodes ("cruise 0/4") take the starting rotor speed of the earlier result's segment; an exact
        label wins, and a label with no match keeps its own guess."""
        from types import SimpleNamespace

        import aerosandbox as asb

        from examples.halo_sizing import warm_start_rotor_speeds
        opti = asb.Opti()
        points = []
        for label in ("cruise 0/4", "cruise 1/4", "climb", "ferry 0/2"):
            x = opti.variable(init_guess=50.0)
            points.append(SimpleNamespace(speed_rotor_rad_s=2.0 * x, condition=SimpleNamespace(label=label)))
        warm_start_rotor_speeds(opti, points, {"cruise 1/4": 30.0, "cruise 2/4": 31.0, "climb": 28.0})
        speeds = [float(opti.value(p.speed_rotor_rad_s, opti.initial())) for p in points]
        onp.testing.assert_allclose(speeds, [30.0, 30.0, 28.0, 100.0], rtol=1e-12)
