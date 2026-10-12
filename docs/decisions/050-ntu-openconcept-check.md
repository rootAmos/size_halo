# Plan 050: the ε-NTU heat exchanger checked against OpenConcept, code to code

## Goal

Check `OffsetStripFinCore` (plan 042, `powertrain/components/compact_heat_exchanger.py`) against the
implementation it follows, OpenConcept's `HXGroup` (`openconcept/thermal/heat_exchanger.py`), with identical
inputs: geometry, material, fluid properties, flows and inlet temperatures. Every quantity both codes produce is
compared and every difference above 0.5 % is explained. A real error in our model is reported, not fixed here.

In: the core alone (geometry, Reynolds numbers, j and f, h, surface efficiency, UA, NTU, effectiveness, heat,
outlet temperatures, both pressure drops, mass). Out: the duct (`ram_air_duct.py` against OpenConcept's
`ducts.py`), the sizing option.

## References

- Brelje, Jasa, Martins, Gray, "Development of a Conceptual-Level Thermal Management System Design Capability in
  OpenConcept", NATO STO-MP-AVT-323-21; OpenConcept 1.2.6 (PyPI, MIT licence, installed 2026-10-11 in an
  isolated environment outside the repository), `openconcept/thermal/heat_exchanger.py`.
- Manglik and Bergles, *Exp. Therm. Fluid Sci.* 10, 1995: j, f, and the offset-strip-fin hydraulic diameter
  d_h = 4 A_c l / A = 4 w h l / (2 (w l + h l + t h) + t w).
- Kays and London, *Compact Heat Exchangers*, 3rd ed.

## Method

- `examples/ntu_openconcept_check.py` has three steps. In the repository environment it writes the cases (every
  input as a plain number; the Halo air properties come from our `air_properties` on the hot-day atmosphere, so
  both codes see the same numbers). In OpenConcept's own environment (run with `python -I`, OpenConcept treated
  as untrusted code) it builds an `HXGroup` per case, sets every input and stores the outputs. In the repository
  environment again it evaluates our core on the stored inputs and prints the per-quantity relative differences.
- Stored OpenConcept outputs: `tests/powertrain/data/compact_heat_exchanger_openconcept.json` (version and date
  in the file). `tests/powertrain/test_compact_heat_exchanger_openconcept.py` compares our core with them, so
  the test does not need OpenConcept.
- Cases: OpenConcept's `HXGroup` defaults (430 x 3 x 19, its property defaults; operating point from its
  `simple_all_electric` example: coolant 0.1 kg/s at 997 kg/m3, so the coolant is C_min); the Halo-scale core
  (600 x 15 x 40, air 8 kg/s on the hot day at 4,000 ft ISA+27.7 K, coolant 3 kg/s at 60 C); air flow 2-12 kg/s
  at count_long 15 (the C_min side swaps near 11.3 kg/s); count_long 5-25 at 8 kg/s.
- Our sign convention: pressure drops are positive losses; OpenConcept's are pressure changes (negative). Magnitudes
  are compared. Our mass includes the coolant held in the core; OpenConcept's `component_weight` is material only,
  so our material mass (total minus coolant) is compared.

## Results

Eleven cases, OpenConcept 1.2.6 (`HXGroup`, `num_nodes=1`) against `OffsetStripFinCore`; largest |relative
difference| over the cases (temperatures: the outlet-temperature difference over the inlet-to-outlet change):

| Quantity | As is | With OpenConcept's coolant d_h |
|---|---|---|
| Width, height, length | 0 | 0 |
| Air d_h, flow area, heat-transfer area | 0 | 0 |
| Coolant flow area, heat-transfer area | 0 | 0 |
| **Coolant d_h** | **-1.26 %** | 0 |
| Air Re, j, f, h, surface efficiency | < 1e-15 | < 1e-15 |
| **Coolant Re** | **-1.26 %** | < 1e-15 |
| **Coolant j, h** | **+0.58 to +0.67 %** | < 1e-15 |
| **Coolant f** | **+0.59 to +0.94 %** | < 1e-15 |
| Coolant surface efficiency | -0.03 % | < 1e-15 |
| UA, NTU | +0.04 to +0.09 % | < 1e-15 |
| Effectiveness, heat, outlet temperatures | +0.01 to +0.04 % | < 1e-15 |
| Air pressure drop | < 1e-15 | < 1e-15 |
| **Coolant pressure drop** | **+1.9 to +2.2 %** | < 1e-15 |
| Material mass | 0 | 0 |

Key values (ours / OpenConcept): OpenConcept defaults (430 x 3 x 19, air 1 kg/s, coolant 0.1 kg/s) 9.74 / 9.74 kW,
ε 0.6125 / 0.6123, air loss 31.8 / 31.8 Pa, coolant loss 13.7 / 13.4 kPa, 1.15 / 1.15 kg. Halo core (600 x 15 x 40,
air 8 kg/s, coolant 3 kg/s, hot day) 113.78 / 113.75 kW, ε 0.5670 / 0.5668, air loss 711.3 / 711.3 Pa, coolant loss
68.2 / 66.7 kPa, material 15.68 / 15.68 kg. Air sweep 2-12 kg/s: heat 45.6-136.1 kW, within 0.04 %; length sweep
count_long 5-25: heat 60.8-137.5 kW, within 0.03 %.

**The one difference is the coolant-side hydraulic diameter.** OpenConcept uses Manglik and Bergles' offset-strip-fin
d_h on the cold side but the plain-duct 2 w h / (w + h) on the hot side, while feeding both into the Manglik and
Bergles j and f; ours uses the Manglik and Bergles definition on both sides, which is the one the correlation was
fitted with. For the 1 x 1 x 6 mm coolant fins (t 0.102 mm) ours is 0.9874 mm against 1 mm. That lowers the coolant
Reynolds number by 1.26 %, raises j and f (their Re exponents are negative), and raises the coolant-side loss most,
since it scales as f / d_h (+2.2 %; +1.9 % at count_long 5, where the coolant Re is 1,640 and f's transition term
weakens the Re dependence). Heat moves by 0.04 % at most because the air side carries most of the thermal
resistance. Substituting OpenConcept's d_h makes every quantity agree to round-off (< 4e-16) in every case, so
there is no other difference: not in geometry, j and f, h, fin efficiency, UA, ε-NTU (including the C_min branch,
covered both ways), outlet temperatures, pressure-loss coefficients or mass.

Other checks: no fouling, so OpenConcept's repeated hot-side fouling term has no effect. Our smooth C_min/C_max
(softness 1e-3 C_air) is invisible here (the closest case, 12 kg/s, has the capacities 6 % apart); with equal
capacity rates it would shift C_min by at most 0.07 %. Our K_e of -0.1 enters as `+ K_e` in a loss, OpenConcept's as
`- K_e` in a pressure change: the same thing. Not an error in our model; plan 042's statement that the hydraulic
diameters are "OpenConcept's expressions" holds for the air side only.

## Assumptions

- Zero fouling on both sides (OpenConcept's `UAOverall` assigns the hot-side fouling term to the cold-side
  variable and adds it twice; with zero fouling the bug has no effect).
- Constant properties per case, the same numbers given to both codes.

## Progress log

- 2026-10-11: plan written; OpenConcept 1.2.6 installed in an isolated environment and its heat exchanger read.
- 2026-10-11: eleven cases run through OpenConcept and stored; `tests/powertrain/test_compact_heat_exchanger_openconcept.py`
  (4 tests) passes with the existing core tests (17 in the two modules). Result above: identical except the
  coolant-side hydraulic diameter (ours is Manglik and Bergles', OpenConcept's is the plain duct); heat within
  0.04 %, coolant loss +2.2 %. No change to the model.
- 2026-10-11: figure `docs/figures/ntu_validation.png` (`python -m examples.ntu_validation_figure`), left panel: heat
  parity with OpenConcept over the eleven stored cases; it also carries the plan 051 checks.

## Slide summary

**Our ε-NTU core reproduces OpenConcept's heat exchanger: heat within 0.04 % over 11 cases, 10 to 137 kW.**

- Identical geometry, air side, UA, ε-NTU and mass; exact to round-off once one input definition is matched.
- The one difference: coolant-side hydraulic diameter (Manglik-Bergles' strip-fin form vs OpenConcept's plain duct),
  1.3 %, giving +2.2 % coolant pressure drop.
