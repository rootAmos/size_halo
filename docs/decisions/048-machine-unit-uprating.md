# Plan 048: baseline v3.7, machine units uprated 10 % in continuous power and the epsilon-NTU cooler

## Goal and scope

Plan 035 builds each motor and generator from whole units of a real product at its catalogue ratings. The
v3.6 generator needs 2.004 units of continuous power, so it rounds up to 3 units: about 400 lb of take-off weight
for 0.2 % of one unit's rating (the plan 044 battery study and the plan 042 cooler study both found the effect).
A unit that close to its rating can be run slightly harder (thermal margin, supplier uprating of the same
hardware). The author's decision (2026-10-11): assume a 10 % boost in continuous power is available, and make it
the default, aircraft version v3.7.

In: `HaloAssumptions.factor_uprating_power_units` (default 1.10) on the unit continuous power; the integrated
inverter kept at its catalogue mass; `assumptions_v3_6` (factor 1.0) so v3.6 stays reproducible; the v3.7 solve,
with and without the plan 042 cooler. Out: torque uprating (peak torque stays at catalogue), a per-point inverter
rating, a thermal derating map.

## References

- Plan 035 (`docs/decisions/035-final-reference.md`): whole real machine units, the relaxed-then-integer solve.
- Plan 042 (NTU cooler) and plan 044 (battery by objective): the 435-880 lb whole-unit rounding finding.
- Unit data: Evolito D1500 (motor), Helix SPX242 (generator), `data/machines/aerospace_motors.csv`.

## Model

- Unit continuous power x `factor_uprating_power_units`; unit mass, peak torque and maximum speed unchanged. The
  relaxed count, the rounding and the integer limits all see the uprated unit, so a machine within 10 % of a
  whole number of units rounds down.
- `UnitMachineMassModel.factor_uprating_power`: the integrated inverter mass uses the catalogue rating (rated
  power / factor / specific power): the uprating runs the same drive harder. Without this the inverter grew with
  the uprated rating on every machine, about +95 lb even where the extra power was not used.
- The thermal model is unchanged: machine resistance follows the (uprated) continuous rating, and every machine
  temperature limit stays a constraint.

## Verification

- `tests/integration/test_halo_unit_uprating.py`: the v3.7 default and the v3.6 set; uprating raises continuous
  power by exactly the factor with peak torque, speed and mass (unit and inverter) unchanged; two uprated
  generator units cover the 2.004-unit demand.
- `assumptions_v3_6` must re-solve to 15,179 lb; the default to the v3.7 number below.

## Assumptions

- 10 % continuous-power margin on the same hardware is available from the suppliers (author's assumption, to be
  confirmed with them: it is a programme decision, not a physics result).
- Inverters share the margin (same drive, run harder).

## Results (cached-start solves, `examples/halo_unit_uprating.py`)

| Case | Take-off weight | Units (motor / generator) |
|---|---|---|
| v3.6 re-solved on the merged main (factor 1.0) | 15,179.0 lb | 2 / 3 |
| Factor 1.10, inverter growing with the rating (first version) | 14,777 lb | 2 / 2 |
| Factor 1.10 with the plan 042 cooler, same | 14,392 lb | 2 / 2 |
| Factor 1.10, inverter at catalogue mass, mass-per-watt cooler | 14,689 lb (-490) | 2 / 2 |
| **v3.7: factor 1.10, inverter at catalogue mass, plan 042 epsilon-NTU cooler** | **14,306 lb (-873)** | 2 / 2 |

v3.7 against v3.6: empty weight 10,317 lb (-813), fuel 2,005 lb, battery 61.0 kWh (912 lb, -9 kWh), wing 239 ft²
(span 38.3 ft, 59.8 lb/ft²), rotors 30.8 ft (disk loading 9.6 lb/ft²), cruise 165 kt at L/D 8.96, cooler 48 kg
(was 127 kg), cost per mission USD 3,237 (was 3,308). Binding: the span-capped rotor radius, the engine-out battery
end voltage and turbogenerator power, the 4,000 ft hover drive, static margin and the hot-day cooler heat
rejection.

The 873 lb splits into the cooler (about -450 lb at fixed units, plan 042) and the generator dropping from 3 to 2
uprated units (about -420 lb); with whole units the two are not additive.
| Factor 1.05 | did not converge (NaN from the 3-generator start, iteration limit from the 2-generator start) | - |

## Progress log

- 2026-10-11: option added; first solves with the inverter scaling with the rating (14,777 lb; 14,392 lb with the
  cooler, heavier than the cooler alone at 14,297 lb because every inverter grew 10 %). Inverter fixed to its
  catalogue mass; 10 % made the default as v3.7 (author's decision). The 1.05 case did not converge from two
  starts and is left unresolved (sensitivity only).
- 2026-10-11: the author made the plan 042 epsilon-NTU cooler part of the v3.7 baseline as well. Default cooler
  "ntu"; `pre_plan035` (every named set of v3.3 and earlier) and the new `assumptions_v3_6` / `assumptions_v3_5`
  pin the mass-per-watt cooler and the catalogue rating so earlier versions stay reproducible. v3.7 solved from the
  cached start at 14,306 lb; the from-scratch default solve is checked by `tests/integration/test_halo_plan035.py`.
  The trajectory model accepts only the mass-per-watt cooler until plan 049 (separate branch) ports the NTU cooler.

## Slide summary

- Headline: v3.7 is 14,306 lb, 873 lb lighter than v3.6: a physics-sized cooler and machine units uprated 10 %.
- The generator needs 2.004 catalogue units, which rounded up to 3; uprated 10 % in continuous power, 2 carry it.
- Caveats: the 10 % is an assumed supplier margin, to be confirmed; the cooler's ducts, fans and pump are estimated
  allowances, and redundancy can raise cooling mass.
