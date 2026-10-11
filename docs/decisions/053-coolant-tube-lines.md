# Plan 053: metal-tube coolant lines (baseline v3.8)

## Goal and scope

v3.7 (plan 048) took the cooler's pump and lines from OpenConcept's published models (plan 051): 26 m of
reinforced flexible hose, 25.4 mm bore, wall sized by hoop stress at 1.03 MPa design pressure over 2.07 MPa
allowable (about 6 mm), 0.85 kg/m of wall plus 0.53 kg/m of coolant: 39.8 kg of pump and lines, 69 % of the
58 kg cooler. The whole cooling system came to 1.9 kW of heat per kg, against the author's rule of thumb of about
3 kW/kg and NASA's published systems (3.1-6.7 kW/kg, plan 051). The author (2026-10-11): "the coolant lines were
never going to be hose"; "metal makes far more sense".

In: `allowances="tube"` on `NtuRamAirHeatExchanger` and `HaloAssumptions.allowances_heat_exchanger` (the v3.8
default); `assumptions_v3_7` keeps v3.7 reproducible. Out: routing lengths from a layout, fittings and clamps,
two pumps' installation, a per-point line pressure drop in the pump rating.

## References

- Plan 051 (`docs/decisions/051-ntu-kellermann-check.md`): OpenConcept's `SimplePump` and `SimpleHose` models,
  the published TMS specific powers (Chapman et al., NASA/TM-2020-220473 and NASA/TM-20205011477).
- Aluminium 6061-T6: yield about 276 MPa (allowable taken as about a third).

## Model

- `count_loops = 2`: one coolant loop per wing (the motors and inverters are at each tip); each loop carries half
  the 2 kg/s, so its bore is 25.4 / sqrt(2) = 18.0 mm for the same velocity (3.8 m/s).
- Aluminium tube, wall max(1 mm minimum gauge, D p / (2 sigma)) with p 1.03 MPa and sigma 90 MPa: the minimum
  gauge governs (hoop stress needs 0.1 mm). 0.161 kg/m of wall plus 0.265 kg/m of coolant: 0.426 kg/m.
- Flexible hose only across each tilting nacelle joint: 1.5 m per loop (supply and return), OpenConcept's hose
  model at the loop bore: 0.692 kg/m.
- Same 26 m of total line (supply and return to both tips) and the same 1.7 kW pump (3.8 kg).
- Pump and lines: 3.8 + 23 x 0.426 + 3 x 0.692 = 15.6 kg (was 39.8 kg).

## Verification

- `tests/powertrain/test_ntu_tube_lines.py` (6 tests): loop bore keeps the velocity; minimum gauge governs at loop
  pressure; hand calculation of the 15.6 kg; tube well under half the hose; the v3.7 core with tube lines lands at
  2.8-4.0 kW/kg (3.31 kW/kg: 33.8 kg for 112 kW); unknown option rejected.
- `tests/integration/test_halo_ntu_cooling.py`: "tube" is the default, `assumptions_v3_7` keeps "published".
- v3.8 re-solved from the v3.7 result; `tests/integration/test_halo_plan035.py` pins the new baseline.

## Assumptions

- 1 mm minimum gauge and 90 MPa allowable for aluminium tube; 1.5 m of hose per loop at the tilt joint.
- 26 m of total line is unchanged from plan 051 (supply and return to both tips); real routing will set it.
- The pump stays one 1.7 kW rating (two loops would have two smaller pumps; the mass is small either way).
- The 3 kW/kg rule of thumb is a check on the result, not an input.

## Results

Re-solved from the v3.7 result (`examples/halo_unit_uprating.py v3_8`, 264 s):

| | v3.6 | v3.7 (hose) | **v3.8 (tube)** |
|---|---|---|---|
| Take-off | 15,179 lb | 14,357 lb | **14,231 lb** (6,455 kg) |
| Empty | 11,130 lb | 10,365 lb | 10,248 lb |
| Cooler | 127 kg (mass per watt) | 58.0 kg | 33.6 kg |
| Cooling system heat per mass (112 kW peak) | - | 1.9 kW/kg | 3.3 kW/kg |
| Battery | 70.0 kWh | 61.5 kWh | 60.2 kWh |
| Fuel | 2,065 lb | 2,008 lb | 1,999 lb |
| Wing, span | 251 ft², 39.2 ft | 240 ft², 38.3 ft | 238 ft², 38.2 ft |
| Rotors | 31.7 ft | 30.8 ft | 30.7 ft |
| Cost per mission | USD 3,308 | 3,242 | 3,230 |

The binding constraints are unchanged from v3.7 (span-capped rotor radius, engine-out battery end voltage and
turbogenerator power, the 4,000 ft hover drive, static margin, hot-day cooler heat rejection). -24 kg of cooler
became -126 lb (-57 kg) of take-off: the growth factor, 2.4.

## Progress log

- 2026-10-11: plan and model; 30 tests pass in the touched modules (tube, Kellermann check, NTU cooler, Halo NTU
  option, unit uprating). With the v3.7 core the cooler is 33.8 kg (was 57.95 kg): core 9.5, ducts 4.3, fan 4.4,
  pump and lines 15.6; 3.31 kW/kg.
- 2026-10-11: v3.8 re-solved at 14,231 lb (cooler 33.6 kg); `tests/integration/test_halo_plan035.py` now pins it.

## Slide summary

- Headline: metal-tube coolant lines put the cooling system at about 3.3 kW/kg, in line with published systems.
- Pump and lines 15.6 kg instead of 39.8 kg: aluminium tube, one loop per wing, hose only at the tilt joints.
- Caveat: line lengths are still an estimate until a layout routes them.
