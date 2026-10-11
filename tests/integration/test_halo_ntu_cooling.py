"""Plan 042: the Halo sizing's heat-exchanger option (builds only; the full solve is in the plan's progress log)."""
import unittest
from dataclasses import replace

from examples.halo_sizing import HaloAssumptions, build_halo_aircraft
from aircraft_closure.powertrain.components.heat_exchanger import RamAirHeatExchanger
from aircraft_closure.powertrain.components.ntu_heat_exchanger import NtuRamAirHeatExchanger
from tests.integration.test_halo_thermal import numeric_design


class HaloHeatExchangerOptionTests(unittest.TestCase):
    def test_default_is_ntu_and_v3_6_is_mass_per_watt(self):
        # Plan 048: the epsilon-NTU cooler is the v3.7 default; v3.6 and earlier keep the mass-per-watt cooler.
        from examples.halo_sizing import assumptions_v3_6
        self.assertEqual(HaloAssumptions().heat_exchanger_model, "ntu")
        aircraft = build_halo_aircraft(numeric_design(), assumptions=assumptions_v3_6)
        self.assertIsInstance(aircraft.powertrain.cooling.heat_exchanger, RamAirHeatExchanger)

    def test_ntu_option_installs_the_sized_cooler(self):
        design = replace(numeric_design(), power_rated_heat_exchanger_W=None, count_wide_heat_exchanger=600.0,
                         count_long_heat_exchanger=15.0, count_tall_heat_exchanger=40.0,
                         area_nozzle_heat_exchanger_m2=0.12, power_rated_fan_heat_exchanger_W=12e3)
        a = HaloAssumptions(heat_exchanger_model="ntu")
        aircraft = build_halo_aircraft(design, assumptions=a)
        cooler = aircraft.powertrain.cooling.heat_exchanger
        self.assertIsInstance(cooler, NtuRamAirHeatExchanger)
        self.assertEqual((cooler.core.count_wide, cooler.core.count_long, cooler.core.count_tall), (600.0, 15.0, 40.0))
        self.assertEqual(cooler.duct.area_nozzle_m2, 0.12)
        self.assertEqual(cooler.duct.efficiency_fan, a.efficiency_fan_heat_exchanger)
        self.assertEqual(cooler.mass_flow_coolant_kg_s, a.mass_flow_coolant_heat_exchanger_kg_s)
        masses = dict((i.instance_name, i.mass_properties.mass) for i in aircraft.powertrain.get_instance_mass_properties())
        self.assertAlmostEqual(masses["heat_exchanger"], cooler.get_mass())
        self.assertAlmostEqual(cooler.mass_breakdown().mass_fans_kg, 12e3 / a.specific_power_fan_heat_exchanger_W_kg)
        # Plans 051 and 048: "published" is the v3.7 default; "estimate" still reaches the installed cooler.
        self.assertEqual(cooler.allowances, "published")
        estimate = build_halo_aircraft(design, assumptions=replace(a, allowances_heat_exchanger="estimate"))
        self.assertEqual(estimate.powertrain.cooling.heat_exchanger.allowances, "estimate")

    def test_unknown_model_rejected(self):
        with self.assertRaises(ValueError):
            build_halo_aircraft(numeric_design(), assumptions=HaloAssumptions(heat_exchanger_model="nusselt"))


if __name__ == "__main__":
    unittest.main()
