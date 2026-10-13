"""Plan 067: the turbine maximum-continuous-power cap at airplane-mode mission points."""
import unittest
from types import SimpleNamespace

import aerosandbox as asb

from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from examples.halo_sizing import HaloAssumptions, turboshaft_mcp_margins


def point(mode, altitude_m, power_W, label):
    condition = SimpleNamespace(mode=mode, altitude_m=altitude_m, temperature_offset_K=0.0, label=label)
    return SimpleNamespace(condition=condition, engine=SimpleNamespace(power_shaft_W=power_W))


class TurboshaftMcpTests(unittest.TestCase):
    def test_cap_is_the_fraction_of_power_available_at_altitude(self):
        engine = SimpleTurboshaft(power_rated_W=835e3, lapse_exponent=0.797)
        points = [point("hover", 0.0, 800e3, "take-off hover 0/1"), point("airplane", 3048.0, 500e3, "cruise 0/4")]
        margins = turboshaft_mcp_margins(points, engine, 0.90)
        self.assertEqual([m.label for m in margins], ["cruise 0/4: turboshaft MCP power_W"])
        mcp_W = 0.90 * engine.power_available_W(asb.Atmosphere(altitude=3048.0))
        self.assertAlmostEqual(float(margins[0].value), (mcp_W - 500e3) / mcp_W, places=12)
        self.assertLess(mcp_W, 0.90 * 835e3)       # lapse at 10,000 ft

    def test_off_by_default(self):
        self.assertIsNone(HaloAssumptions().fraction_mcp_turboshaft)


if __name__ == "__main__":
    unittest.main()
