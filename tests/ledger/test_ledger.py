import math
import os
import tempfile
import unittest
from datetime import date
from types import SimpleNamespace

from aircraft_closure.ledger.ingest import ingest_evidence, ingest_sizing
from aircraft_closure.ledger.model import Activity, Evidence, Ledger, Quantity, Trade, TradeOption
from aircraft_closure.ledger.rank import portfolio, priorities, z_99
from aircraft_closure.ledger.store import load, save, view

today = date(2026, 10, 9)


def mass(key, value, sigma, **kwargs):
    return Quantity(key, key, "mass", "kg", "Structures", evidence=[Evidence("2026-10-01", value, sigma, "sizing")],
                    sensitivity=1.5, **kwargs)


def two_item_ledger():
    ledger = Ledger("test", growth_factor=1.5)
    ledger.quantities = {"a": mass("a", 400.0, 40.0), "b": mass("b", 300.0, 10.0)}
    return ledger


class BeliefTest(unittest.TestCase):
    def test_rerun_of_same_source_replaces(self):
        q = mass("a", 400.0, 40.0)
        q.evidence.append(Evidence("2026-10-05", 420.0, 42.0, "sizing"))
        self.assertEqual(q.belief(), (420.0, 42.0))

    def test_independent_sources_at_top_tier_fuse(self):
        q = mass("a", 400.0, 40.0)
        q.evidence += [Evidence("2026-10-05", 450.0, 30.0, "FE", 2), Evidence("2026-10-06", 470.0, 30.0, "test", 2)]
        mean, sigma = q.belief()
        self.assertAlmostEqual(mean, 460.0)                    # the handbook tier no longer counts
        self.assertAlmostEqual(sigma, 30.0 / math.sqrt(2))


class RankingTest(unittest.TestCase):
    def test_portfolio_is_root_sum_of_squares(self):
        p = portfolio(two_item_ledger())
        self.assertAlmostEqual(p["sigma_takeoff_kg"], 1.5 * math.hypot(40.0, 10.0))

    def test_full_resolution_frees_its_share_of_the_99_margin(self):
        ranked = priorities(two_item_ledger(), today)
        top = ranked[0]
        self.assertEqual(top.key, "a")
        sigma_total = 1.5 * math.hypot(40.0, 10.0)
        self.assertAlmostEqual(top.value_high_kg, z_99 * (sigma_total - 15.0))
        self.assertEqual(top.schedule, "no plan")

    def test_planned_activity_sets_the_expected_value(self):
        ledger = two_item_ledger()
        ledger.quantities["a"].plan = [Activity("FE wing box", "2026-10-01", 20.0)]
        ledger.quantities["a"].lock_date = "2026-10-20"
        top = priorities(ledger, today)[0]
        sigma_total = 1.5 * math.hypot(40.0, 10.0)
        self.assertAlmostEqual(top.value_kg, z_99 * (sigma_total - 1.5 * math.hypot(20.0, 10.0)))
        self.assertEqual(top.schedule, "overdue")

    def test_quantity_without_sensitivity_is_a_blind_spot_not_zero(self):
        ledger = two_item_ledger()
        ledger.quantities["c"] = Quantity("c", "drag factor", "input", "-", "Aero",
                                          evidence=[Evidence("2026-10-01", 1.27, 0.1, "NDARC")])
        blind = [r for r in priorities(ledger, today) if r.key == "c"][0]
        self.assertEqual(blind.kind, "blind spot")
        self.assertIsNone(blind.value_kg)

    def test_trade_probability_and_range(self):
        ledger = two_item_ledger()
        ledger.trades["t"] = Trade("t", "wing covers", "Structures", options=[
            TradeOption("unstiffened", "a", 0.0, 0.0), TradeOption("stiffened", "a", -20.0, 10.0)])
        trade = [r for r in priorities(ledger, today) if r.key == "t"][0]
        self.assertEqual(trade.kind, "decide trade")
        self.assertAlmostEqual(trade.value_kg, 30.0)           # 1.5 x 20 kg
        self.assertIn("still open", trade.action)               # P = 97.7 %


class IngestTest(unittest.TestCase):
    def test_sizing_updates_masses_limits_and_history(self):
        ledger = two_item_ledger()
        ledger.quantities["a"].sizing_keys = ("wing",)
        result = SimpleNamespace(
            mass_takeoff_kg=6885.0, mass_empty_kg=5049.0, mass_payload_kg=900.0,
            component_masses_kg=(("wing", 410.0),), powertrain_masses_kg=(),
            sensitivity=SimpleNamespace(objective="mass_takeoff", growth_factor=1.8,
                                        margins=(("hover T/W", 0.0, 2500.0), ("stall", 0.3, 0.0))))
        ingest_sizing(ledger, result, "sizing", today)
        self.assertEqual(ledger.quantities["a"].belief(), (410.0, 41.0))
        self.assertEqual(ledger.quantities["b"].sensitivity, 1.8)
        self.assertEqual(list(ledger.limits), ["hover T/W"])
        self.assertEqual(len(ledger.history), 1)

    def test_evidence_reranks_and_round_trips(self):
        ledger = two_item_ledger()
        ingest_evidence(ledger, "a", 380.0, 5.0, "vendor weight", 3, today)
        self.assertEqual(priorities(ledger, today)[0].key, "b")
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "ledger.json")
            save(ledger, path)
            again = load(path)
        self.assertEqual(again.quantities["a"].belief(), (380.0, 5.0))
        self.assertEqual(view(again, today)["priorities"][0]["key"], "b")


if __name__ == "__main__":
    unittest.main()
