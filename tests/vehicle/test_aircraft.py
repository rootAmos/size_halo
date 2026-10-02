import unittest
from dataclasses import fields, replace

from aircraft_closure.vehicle.aircraft import MassBreakdown
from aircraft_closure.vehicle.condition import StructuralDesignCondition
from aircraft_closure.vehicle.powertrain_installation import InstalledInstance, PowertrainInstallation
from examples.aircraft_mass_closure import build_reference_aircraft
from examples.series_hybrid_point import build_reference_topology

condition = StructuralDesignCondition(mass_design_kg=1500.0)


class PowertrainInstallationTests(unittest.TestCase):
    def locations(self, topology):
        return tuple(InstalledInstance(name, x_m=float(i), z_m=0.0) for i, name in enumerate(topology.instances))

    def test_mass_counts_multiplicity_and_installation_factor(self):
        topology = build_reference_topology(count_rotors=4)
        installation = PowertrainInstallation(topology, self.locations(topology), installation_factor=1.1)
        expected_kg = 1.1 * sum(i.count * i.component.get_mass() for i in topology.instances.values())
        self.assertAlmostEqual(float(installation.get_mass_properties().mass), float(expected_kg), places=9)
        motor = next(m for m in installation.get_instance_mass_properties() if m.instance_name == "motor")
        self.assertAlmostEqual(float(motor.mass_properties.mass), 1.1 * 4 * 20.0, places=9)

    def test_locations_must_cover_every_instance_once(self):
        topology = build_reference_topology()
        complete = self.locations(topology)
        with self.assertRaisesRegex(ValueError, "missing"):
            PowertrainInstallation(topology, complete[:-1])
        with self.assertRaisesRegex(ValueError, "more than once"):
            PowertrainInstallation(topology, complete + complete[:1])
        with self.assertRaisesRegex(ValueError, "unknown"):
            PowertrainInstallation(topology, complete + (InstalledInstance("ghost", 0.0),))


class AircraftAggregationTests(unittest.TestCase):
    def test_total_is_sum_of_items_and_cg_is_mass_weighted(self):
        breakdown = build_reference_aircraft().get_mass_breakdown(condition)
        items = [getattr(breakdown, f.name) for f in fields(MassBreakdown)]
        total = breakdown.total()
        mass_kg = sum(float(item.mass) for item in items)
        x_cg_m = sum(float(item.mass) * float(item.x_cg) for item in items) / mass_kg
        z_cg_m = sum(float(item.mass) * float(item.z_cg) for item in items) / mass_kg
        self.assertAlmostEqual(float(total.mass), mass_kg, places=9)
        self.assertAlmostEqual(float(total.x_cg), x_cg_m, places=9)
        self.assertAlmostEqual(float(total.z_cg), z_cg_m, places=9)
        self.assertAlmostEqual(float(breakdown.mass_empty_kg()), mass_kg - 300.0, places=9)

    def test_payload_shift_moves_cg_by_mass_ratio(self):
        aircraft = build_reference_aircraft()
        base = aircraft.get_mass_breakdown(condition)
        shifted = replace(aircraft, payload=replace(aircraft.payload, x_m=4.0))
        delta_x_cg_m = float(shifted.get_cg_x_m(condition)) - float(base.total().x_cg)
        self.assertAlmostEqual(delta_x_cg_m, 300.0 * (4.0 - 3.0) / float(base.total().mass), places=9)

    def test_moving_the_wing_moves_rotors_with_it(self):
        forward = build_reference_aircraft(x_le_wing_m=2.0).get_mass_breakdown(condition)
        aft = build_reference_aircraft(x_le_wing_m=3.0).get_mass_breakdown(condition)
        self.assertGreater(float(aft.powertrain.x_cg), float(forward.powertrain.x_cg))
        self.assertAlmostEqual(float(aft.wing.x_cg) - float(forward.wing.x_cg), 1.0, places=9)

    def test_wing_to_tail_distance_uses_root_quarter_chords(self):
        aircraft = build_reference_aircraft()
        expected_m = ((aircraft.horizontal_tail.x_le_root_m + 0.25 * aircraft.horizontal_tail.chord_root_m())
                      - (aircraft.wing.x_le_root_m + 0.25 * aircraft.wing.chord_root_m()))
        self.assertAlmostEqual(float(aircraft.distance_wing_to_tail_m()), float(expected_m), places=12)

    def test_to_asb_airplane(self):
        airplane = build_reference_aircraft().to_asb()
        self.assertEqual([w.name for w in airplane.wings], ["wing", "horizontal_tail", "vertical_tail"])
        self.assertEqual(len(airplane.fuselages), 1)


if __name__ == "__main__":
    unittest.main()
