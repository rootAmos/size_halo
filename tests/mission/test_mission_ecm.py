"""Tier 17: equivalent-circuit battery in flight points and sub-segmented missions."""
import unittest
from dataclasses import replace

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.aerodynamics.simple import SimpleAerodynamics
from aircraft_closure.mission.mission import Mission, build_mission
from aircraft_closure.mission.segments import CruiseSegment, HoverSegment
from aircraft_closure.performance.flight_point import FlightCondition, build_flight_point
from aircraft_closure.powertrain.components.battery_ecm import EquivalentCircuitBattery
from aircraft_closure.powertrain.components.gearbox import Gearbox
from aircraft_closure.powertrain.components.generator import Generator
from aircraft_closure.powertrain.components.motor import Motor
from aircraft_closure.powertrain.components.propulsor import ActuatorDiskPropulsor
from aircraft_closure.powertrain.components.turboshaft import SimpleTurboshaft
from aircraft_closure.powertrain.topologies import build_series_hybrid
from examples.aircraft_mass_closure import build_reference_aircraft

count_rotors = 4
pack = EquivalentCircuitBattery(count_series=210, count_parallel=12.0, factor_power_density=5.0)


def ecm_aircraft(battery=pack):
    """The four-rotor reference (examples/series_hybrid_point.py) with the equivalent-circuit pack."""
    generator = Generator()
    losses = generator.loss_model
    generator = Generator(power_rated_W=generator.power_rated_W * count_rotors,
                          max_torque_Nm=generator.max_torque_Nm * count_rotors,
                          loss_model=replace(losses, torque_peak_efficiency_Nm=losses.torque_peak_efficiency_Nm
                                             * count_rotors))
    topology = build_series_hybrid(Motor(), generator, battery,
                                   SimpleTurboshaft(power_rated_W=SimpleTurboshaft().power_rated_W * count_rotors),
                                   Gearbox(), ActuatorDiskPropulsor(), count_rotors=count_rotors)
    return build_reference_aircraft(3.23, area_horizontal_tail_m2=2.16, area_vertical_tail_m2=1.92, topology=topology)


aircraft = ecm_aircraft()
aero = SimpleAerodynamics()
mission = Mission((HoverSegment(60.0, 0.0, 0.6, "take-off hover"), CruiseSegment(60000.0, 1000.0, 60.0, 0.3)))


def fly(subsegments=1, soc=0.9, battery_mission=mission):
    opti = asb.Opti()
    flown = build_mission(opti, aircraft, aero, battery_mission, 1550.0, soc, subsegments=subsegments)
    opti.minimize(flown.mass_fuel_burnt_kg)
    return flown, opti.solve(verbose=False)


def points(flown):
    return [p for s in flown.segments for p in (s.subsegments or (s,))]


class FlightPointBranchTests(unittest.TestCase):
    def test_flight_point_is_on_the_low_current_root(self):
        """The power balance R_eff I^2 - V* I + P = 0 is solved as an equality; the solution is its low root."""
        opti = asb.Opti()
        point = build_flight_point(opti, aircraft, aero, FlightCondition(mode="hover", altitude_m=0.0, soc=0.3,
                                                                         hybridization_electric=0.8), 1550.0)
        solution = opti.solve(verbose=False)
        b = point.battery
        v_star_V, r_ohm = solution.value(b.voltage_driving_V), solution.value(b.resistance_effective_ohm)
        power_W = solution.value(b.power_electric_W)
        low_A = (v_star_V - np.sqrt(v_star_V**2 - 4 * r_ohm * power_W)) / (2 * r_ohm)
        self.assertAlmostEqual(solution.value(b.current_A) / low_A, 1.0, places=6)
        self.assertGreater(solution.value(b.voltage_V), v_star_V / 2)

    def test_branch_constraint_excludes_the_high_current_root(self):
        """Without the branch an Opti started near the high root converges there (V < V*/2); with it, that
        root is never returned (IPOPT reports local infeasibility from there) and a normal start gives the low root."""
        result = pack.evaluate(0.0, 0.3)
        r_ohm, v_star_V = float(result.resistance_effective_ohm), float(result.voltage_driving_V)

        def solve(branch, guess_A):
            opti = asb.Opti()
            current_A = opti.variable(init_guess=guess_A)
            battery = pack.evaluate(current_A, 0.3)
            opti.subject_to(battery.power_electric_W / 1e5 == 2.0)
            if branch:
                opti.subject_to(battery.voltage_V / battery.voltage_driving_V >= 0.5)
            return opti.solve(verbose=False).value(battery.voltage_V / battery.voltage_driving_V)

        high_guess_A = 0.95 * v_star_V / r_ohm
        self.assertLess(solve(False, high_guess_A), 0.5)
        try:
            self.assertGreaterEqual(solve(True, high_guess_A), 0.5 - 1e-9)
        except RuntimeError:
            pass                                      # infeasibility reported, never the high root
        self.assertGreater(solve(True, 50.0), 0.5)
        self.assertGreater(solve(False, 50.0), 0.5)   # the usual guess was already on the low root


class SubsegmentMissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.one, cls.one_solution = fly(1)
        cls.four, cls.four_solution = fly((2, 4))

    def test_point_count_and_labels(self):
        self.assertEqual(len(self.four.segments), 2)
        self.assertEqual(len(points(self.four)), 6)
        self.assertEqual(self.four.segments[1].subsegments[2].point.condition.label, "cruise 3/4")
        self.assertIn("cruise 3/4: battery end voltage_V", [m.label for m in self.four.margins])

    def test_soc_chain_is_coulomb_counting(self):
        s = self.four_solution
        soc = 0.9
        for p in points(self.four):
            self.assertAlmostEqual(s.value(p.soc_start), soc, places=9)
            soc -= s.value(p.point.battery.current_A) * s.value(p.duration_s) / pack.capacity_As
        self.assertAlmostEqual(s.value(self.four.soc_end), soc, places=9)
        self.assertAlmostEqual(s.value(self.four.segments[1].soc_end), soc, places=9)

    def test_energy_is_conserved(self):
        """Chemical energy = integral of OCV dQ (exact for the polynomial) to second order in the step;
        terminal energy + losses = chemical energy at every point."""
        for flown, s, tolerance in ((self.one, self.one_solution, 2e-3), (self.four, self.four_solution, 2e-4)):
            chemical_J = 0.0
            for p in points(flown):
                b = p.point.battery
                self.assertAlmostEqual(s.value(b.power_electric_W + b.power_loss_W) / s.value(b.power_chemical_W),
                                       1.0, places=9)
                soc_start, soc_end = s.value(p.soc_start), s.value(p.soc_end)
                exact_J = (pack.capacity_As * pack.count_series * (soc_start - soc_end)
                           * pack.cell.ocv_model.mean_voltage_V(soc_end, soc_start))
                chemical_J += s.value(p.energy_battery_chemical_J)
                self.assertAlmostEqual(s.value(p.energy_battery_chemical_J) / exact_J, 1.0, delta=tolerance)
            self.assertAlmostEqual(s.value(flown.energy_battery_chemical_J) / chemical_J, 1.0, places=9)

    def test_rc_state_propagates_from_rest(self):
        s = self.four_solution
        sequence = points(self.four)
        first = sequence[0].point.battery
        steady = pack.evaluate(s.value(first.current_A), 0.9)
        self.assertGreater(s.value(first.voltage_V), float(steady.voltage_V))   # polarization still building
        for previous, current in zip(sequence, sequence[1:]):
            end_V = s.value(previous.point.battery.voltage_rc_end_V[1])
            # Continuity: the next interval's driving voltage uses the previous end state.
            b = current.point.battery
            duration_s = s.value(current.duration_s)
            tau_s = pack.cell.resistance_model.time_constants_s()[1]
            offset_V = end_V * tau_s / duration_s * (1 - np.exp(-duration_s / tau_s))
            self.assertLess(abs(s.value(b.voltage_open_circuit_V - b.voltage_driving_V)
                                - (offset_V + s.value(previous.point.battery.voltage_rc_end_V[0])
                                   * 8.0 / duration_s * (1 - np.exp(-duration_s / 8.0)))), 1e-6)

    def test_voltage_falls_through_the_cruise(self):
        s = self.four_solution
        cruise = self.four.segments[1].subsegments
        voltages = [s.value(p.point.battery.voltage_open_circuit_V) for p in cruise]
        self.assertTrue(all(a > b for a, b in zip(voltages, voltages[1:])))

    def test_one_subsegment_matches_the_split_closely(self):
        self.assertAlmostEqual(self.one_solution.value(self.one.soc_end) / self.four_solution.value(self.four.soc_end),
                               1.0, delta=0.01)

    def test_steady_polarization_start(self):
        opti = asb.Opti()
        flown = build_mission(opti, aircraft, aero, Mission((HoverSegment(60.0, 0.0, 0.6),)), 1550.0, 0.9,
                              polarization_start="steady")
        s = opti.solve(verbose=False)
        b = flown.segments[0].point.battery
        self.assertAlmostEqual(s.value(b.voltage_V), float(pack.evaluate(s.value(b.current_A), 0.9, 60.0).voltage_V),
                               places=6)

    def test_bad_arguments(self):
        with self.assertRaises(ValueError):
            build_mission(asb.Opti(), aircraft, aero, mission, 1550.0, 0.9, subsegments=(1,))
        with self.assertRaises(ValueError):
            build_mission(asb.Opti(), aircraft, aero, mission, 1550.0, 0.9, polarization_start="warm")


if __name__ == "__main__":
    unittest.main()
