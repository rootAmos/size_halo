# Trade study: Redundancy: lanes, buses and battery strings (2/2/2 today)

Run 2026-10-11 at commit [`3d990f9`](https://github.com/rootAmos/size_halo/commit/3d990f991377c3cf8e573e6866bde2a0fef10710). Ledger key `trade.redundancy_level`, owner Electrical, locks at Conceptual design (2026-12-15).

**Result.** No redundancy layer (single lane, bus, string) saves 1,223 lb of take-off mass (±38 lb, 1σ) against 2 lanes, 2 buses, 2 strings (today). Margin policy: P(better than 2 lanes, 2 buses, 2 strings (today)) = 100 %. The data has decided it. It ranks #1 of 71 in the ledger.

| Option | Take-off (lb) | Change (lb) | 1σ (lb) | Empty (lb) | Battery (lb) | Binding limits |
|---|---|---|---|---|---|---|
| 2 lanes, 2 buses, 2 strings (today) | 15,179 | +0 | 0 | 11,130 | 1,047 | 18 |
| 3 battery strings | 15,181 | +2 | 0 | 11,132 | 1,048 | 18 |
| 4 lanes per rotor | 17,521 | +2,342 | 71 | 13,289 | 1,416 | 21 |
| No redundancy layer (single lane, bus, string) | 13,957 | -1,223 | 38 | 9,992 | 857 | 20 |

**What changes, item by item** (empty weight, lb, against the reference):

| Item | 3 battery strings | 4 lanes per rotor | No redundancy layer (single lane, bus, string) |
|---|---|---|---|
| Wing | +0 | +105 | -56 |
| Tails | +0 | +22 | -11 |
| Fuselage | +0 | +38 | -21 |
| Landing gear | +0 | +64 | -34 |
| Systems | +0 | +131 | -69 |
| Rotors | +0 | +299 | -151 |
| Rotor gearboxes | +0 | +110 | -56 |
| Motors (with inverters) | +0 | +938 | -469 |
| Battery | +0 | +368 | -190 |
| Heat exchanger | +0 | +74 | -19 |
| Protection and bus tie | +1 | +8 | -63 |

**Method.** Each option is one coupled sizing, warm-started from the baseline (15,179 lb) at its machine unit counts. The 1σ is the relative model error of the empty-weight change (each changed item at its maturity spread), applied to the take-off change; errors common to all options cancel.

**Reproduce.**

```
git checkout 3d990f991377c3cf8e573e6866bde2a0fef10710
python -m examples.halo_trade_study trade.redundancy_level --cache output/ledger/baseline.pkl
```
