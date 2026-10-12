"""Plan 034: excrescence and trim drag on the AeroBuildup and Scholz models."""
import unittest
from dataclasses import replace

import aerosandbox.numpy as np
import aerosandbox.tools.units as u

from aircraft_closure.aerodynamics.buildup import BuildupAerodynamics
from aircraft_closure.aerodynamics.scholz import InterferenceFactors, ScholzAerodynamics
from examples.halo_sizing import HaloAssumptions, assumptions_plan030, assumptions_v3_7, build_halo_aerodynamics
from examples.xv15_reference import Xv15Reference, build_xv15_aircraft, build_xv15_drag_aircraft


def scalar(x):
    return float(np.asarray(x).reshape(-1)[0])


reference = Xv15Reference()
xv15 = build_xv15_drag_aircraft(reference)          # plan 054: published sections, drawn bodies
# Plan 034 calibration geometry: NACA 2423 / 0015 sections, round fuselage, 9 x 3.3 ft nacelles.
xv15_plan034 = build_xv15_aircraft(reference, mass_engine_kg=300.0)
xv15_plan034 = replace(xv15_plan034, nacelles=replace(xv15_plan034.nacelles, length_m=9 * u.foot,
                                                      diameter_m=3.3 * u.foot, y_m=xv15_plan034.wing.span_m() / 2))
tails = InterferenceFactors(horizontal_tail=1.08, vertical_tail=1.08)


def drag_area_ft2(model, aircraft=xv15):
    result = model.evaluate(aircraft, reference.velocity_cruise_m_s, reference.altitude_cruise_m, 0.0)
    return scalar(result.cd0) * aircraft.wing.area_m2 / u.foot**2


class DragCorrectionTests(unittest.TestCase):
    def test_defaults_are_clean_components(self):
        for model in (BuildupAerodynamics(), ScholzAerodynamics()):
            self.assertEqual(model.factor_excrescence, 1.0)
            self.assertEqual(model.fraction_trim_drag, 0.0)

    def test_halo_factors_calibrate_xv15_components_to_ndarc(self):
        """NDARC XV-15 cruise: wing, tails, fuselage and pylons 6.25 ft2 (Johnson 2010, Table 1)."""
        a = HaloAssumptions()
        trip = dict(xtr_upper=a.fraction_chord_transition, xtr_lower=a.fraction_chord_transition)
        for model in (BuildupAerodynamics(interference=tails, factor_excrescence=a.factor_excrescence_buildup, **trip),
                      ScholzAerodynamics(interference=tails, factor_excrescence=a.factor_excrescence_scholz)):
            self.assertAlmostEqual(drag_area_ft2(model) / 6.25, 1.0, delta=0.01, msg=type(model).__name__)

    def test_v3_7_factors_calibrate_the_plan034_geometry(self):
        """The v3.7 set keeps the plan 034 calibration: 1.27 / 1.17 on the 2423 wing, 10 % laminar, 9 ft nacelles."""
        a = assumptions_v3_7
        trip = dict(xtr_upper=a.fraction_chord_transition, xtr_lower=a.fraction_chord_transition)
        for model in (BuildupAerodynamics(interference=tails, factor_excrescence=a.factor_excrescence_buildup, **trip),
                      ScholzAerodynamics(interference=tails, factor_excrescence=a.factor_excrescence_scholz)):
            self.assertAlmostEqual(drag_area_ft2(model, xv15_plan034) / 6.25, 1.0, delta=0.01,
                                   msg=type(model).__name__)

    def test_excrescence_is_a_separate_item_proportional_to_components(self):
        clean = ScholzAerodynamics(interference=tails)
        dirty = ScholzAerodynamics(interference=tails, factor_excrescence=1.2)
        args = (xv15, reference.velocity_cruise_m_s, reference.altitude_cruise_m)
        items_clean = {i.label: scalar(i.cd0) for i in clean.parasite_drag_breakdown(*args)}
        items_dirty = {i.label: scalar(i.cd0) for i in dirty.parasite_drag_breakdown(*args)}
        components = sum(v for k, v in items_clean.items() if k not in ("excrescence", "miscellaneous", "landing_gear"))
        self.assertNotIn("excrescence", items_clean)          # default: no item, the plan 025 expression graph
        self.assertAlmostEqual(items_dirty["excrescence"], 0.2 * components, places=12)

    def test_trim_drag_scales_total_drag_not_cd0(self):
        for cls in (ScholzAerodynamics, BuildupAerodynamics):
            base = cls().evaluate(xv15, 100.0, 3000.0, 4.0)
            trimmed = cls(fraction_trim_drag=0.02).evaluate(xv15, 100.0, 3000.0, 4.0)
            self.assertAlmostEqual(scalar(trimmed.cd0), scalar(base.cd0), places=12)
            self.assertAlmostEqual(scalar(trimmed.cd) - scalar(base.cd),
                                   0.02 * (scalar(base.cd0) + scalar(base.cdi)), places=10)

    def test_halo_switch(self):
        off = build_halo_aerodynamics(assumptions=assumptions_plan030)
        on = build_halo_aerodynamics(assumptions=HaloAssumptions())       # on by default since plan 035
        self.assertEqual(off.factor_excrescence, 1.0)
        self.assertEqual(on.factor_excrescence, HaloAssumptions().factor_excrescence_buildup)
        self.assertIsNotNone(on.trim)                                  # plan 036: trim from the tail load
        self.assertIsNone(off.trim)


if __name__ == "__main__":
    unittest.main()
