# Trade study: Wing material: graphite-epoxy or aluminium

Run 2026-10-09 at commit [`a6998ae`](https://github.com/rootAmos/size_halo/commit/a6998aec87b74bf1ac1583b5a09096dbb7bf832e). Ledger key `trade.wing_material`, owner Structures, locks at Concept freeze (2026-12-15).

**Result.** The reference holds: Graphite-epoxy (baseline) is the lightest option. The best alternative, Aluminium (the XV-15's), adds 699 lb of take-off mass (±35 lb, 1σ). The data has decided it: the trade can be closed.

| Option | Take-off (lb) | Change (lb) | 1σ (lb) | Empty (lb) | Battery (lb) | Binding limits |
|---|---|---|---|---|---|---|
| Graphite-epoxy (baseline) | 15,179 | +0 | 0 | 11,130 | 1,047 | 18 |
| Aluminium (the XV-15's) | 15,878 | +699 | 35 | 11,767 | 1,189 | 18 |

**What changes, item by item** (empty weight, lb, against the reference):

| Item | Aluminium (the XV-15's) |
|---|---|
| Wing | +291 |
| Tails | +5 |
| Fuselage | +16 |
| Landing gear | +19 |
| Systems | +35 |
| Rotors | +67 |
| Rotor gearboxes | +36 |
| Battery | +141 |
| Heat exchanger | +22 |
| Protection and bus tie | +3 |

**Method.** Each option is one coupled sizing, warm-started from the baseline (15,179 lb) at its machine unit counts. The 1σ is the relative model error of the empty-weight change (each changed item at its maturity spread), applied to the take-off change; errors common to all options cancel.

**Reproduce.**

```
git checkout a6998aec87b74bf1ac1583b5a09096dbb7bf832e
python -m examples.halo_trade_study trade.wing_material --cache output/ledger/baseline.pkl
```
