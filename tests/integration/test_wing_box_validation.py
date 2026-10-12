"""Plan 054: the bottom-up wing box with no calibration factor against the V-22 FSD and Bell D266 wings, with the
same stiffness requirements and tip masses as the AFDD cross-checks (`examples.tiltrotor_wing_calibration`)."""
import unittest

from aircraft_closure.weights import wing_box as wb
from examples.tiltrotor_wing_calibration import d266_wing_check, v22_wing_check

lb, ft = 0.45359237, 0.3048


def bottom_up(check, material, span_ft, area_ft2, mass_lb, width_ft):
    m = check.masses
    return wb.size_wing_box(
        span_m=span_ft * ft, chord_m=area_ft2 / span_ft * ft, thickness_to_chord=0.23, mass_design_kg=mass_lb * lb,
        mass_tip_kg=check.mass_tip_kg, radius_gyration_pylon_m=1.0, offset_cg_tip_m=0.4,
        speed_rotor_design_rad_s=40.0, frequency_torsion_per_rev=1.0, frequency_beam_per_rev=1.0,
        frequency_chord_per_rev=1.0, width_attachment_m=width_ft * ft, material=material,
        stiffness_required_Nm2=(float(m.stiffness_torsion_Nm2), float(m.stiffness_beam_Nm2),
                                float(m.stiffness_chord_Nm2)))


class TestWingBoxValidation(unittest.TestCase):
    def test_v22_composite_wing_without_calibration(self):
        check = v22_wing_check()
        sizing = bottom_up(check, wb.graphite_epoxy_wing_box_material(), 45.8, 382.0, 47500, 7.9)
        error = sizing.total() / check.mass_actual_kg - 1
        self.assertLess(abs(error), 0.15)           # -12 % with the box's mean depth (plan 054)
        # Closer than the AFDD wing even with its XV-15 factor (-18 %).
        self.assertLess(abs(error), abs(check.mass_predicted_calibrated_kg / check.mass_actual_kg - 1))

    def test_d266_aluminium_wing_without_calibration(self):
        check = d266_wing_check()
        sizing = bottom_up(check, wb.aluminium_wing_box_material(), 46.8, 330.5, 23000, 6.5)
        self.assertLess(abs(sizing.total() / check.mass_actual_kg - 1), 0.10)


if __name__ == "__main__":
    unittest.main()
