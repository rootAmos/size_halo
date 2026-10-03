# Electric machines sized by torque; machine speed and gear ratio as design variables

Status: COMPLETED 2026-10-03. Tier 13 of the 2026-10-02 roadmap: external review item 2
("motor and generator mass is just power divided by a constant ... the model
can't trade direct drive against geared drive, gear ratio, or RPM schedule").
The user added on 2026-10-03 that motor, generator and turbine power must be
sized distinctly and shown with distinct line styles on the constraint
diagram (implemented in the dashboard during plan 017).

## Goal and scope

- **`TorqueDensityMassModel`** for `Motor` and `Generator`, an
  interchangeable mass submodel (default None keeps power / specific
  power). mass = softmax(max_torque / torque_density,
  power_rated / specific_power_max).
- **AFDD00 gearbox and rotor-shaft equation** (NDARC ch. 19). Its
  input-speed exponent of 0.099 gives a mild reduction-ratio penalty.
- **Optional generator step-up gearbox** in `build_series_hybrid`, handled
  by the flight point: engine shaft power = generator shaft power /
  gearbox efficiency.
- **Halo-class sizing:**
  - motor peak speed, rotor reduction ratio and generator speed become
    design variables;
  - two discrete alternatives: direct-drive rotor, and generator on the
    engine's 1,210 rpm output shaft (the engine as delivered);
  - the named legacy set `assumptions_tier12b` reproduces Tier 12b.

## References

- magniX magni650 (700 kW continuous, 3,216 N.m peak torque, 2,300 rpm,
  206 kg including inverters and HV cables) and magni350 (1,608 N.m,
  128 kg). Public specifications, e.g.
  https://www.emobility-engineering.com/magnix-magni350-650-magnidrive-100/
- NASA High Efficiency Megawatt Motor (1.4 MW, 6,800 rpm, 16 kW/kg
  electromagnetic; partially superconducting).
  https://ntrs.nasa.gov/citations/20190030477
- NDARC Theory, NASA/TP-2009-215402, sec. 19-7.4 (AFDD00 gearbox).
- The user's GASP_TS deck: `propeller_rpm` 1,210 (engine output shaft) and
  free-turbine speed 24,539 rpm.

## Assumptions

- **Torque density:** 15 N.m/kg, magniX class, including inverters and
  cables. Tier 15 (electrical layer) must split the inverter out to avoid
  double counting.
- **Specific-power cap:** 10 kW/kg at high speed. This is a conventional
  machine's cap; HEMM's 16 kW/kg needs superconductors.
- **Gear ratio bounds:** 1.5–40. Multi-stage gearbox mass beyond AFDD00's
  mild ratio penalty is not modelled.
- **Generator step-up gearbox:** AFDD00, with the engine output speed as
  the slow side.

## Interfaces

- `motor.TorqueDensityMassModel`;
- `Motor.mass_model` and `Generator.mass_model`;
- `afdd.mass_gearbox_rotor_shaft_afdd00_kg`;
- `build_series_hybrid(..., generator_gearbox=None)`;
- `HaloAssumptions`: `machine_mass_by_torque`, `torque_density_Nm_kg`,
  `specific_power_max_machine_W_kg`, `generator_step_up`,
  `speed_output_turboshaft_rad_s`, `direct_drive_rotor`;
- `HaloDesign`: `speed_peak_motor_rad_s`, `reduction_ratio`,
  `speed_peak_generator_rad_s`.

## Symbolic considerations

A smooth maximum (softmax) in the mass model keeps IPOPT differentiable.
The gearbox ratio is symbolic. The generator gearbox exists only when
step-up is chosen, a Python-level architecture switch. Discrete
architectures are separate solves, per the review's "enumeration" item.

## Tests

- Defaults unchanged.
- Mass model:
  - torque-limited and power-limited limits;
  - the smooth-maximum bound;
  - a faster machine is lighter at equal power;
  - symbolic.
- AFDD00: the published equation, and the ratio exponent.
- Topology: the generator gearbox wiring.
- Halo:
  - machines sized by torque, with the step-up gearbox present;
  - a generator on the 1,210 rpm shaft carries less than a quarter of the
    stepped payload;
  - the legacy Tier 12b reference reproduces 14,877 lb.

## Acceptance

- Tests and notebooks pass.
- The Tier 13 notebook shows:
  - the architecture comparison;
  - the machine speeds and ratio chosen;
  - motor, generator and turbine power sized distinctly;
  - the cruise-torque check.
- The dashboard and docs are updated.

## Progress and decisions

- 2026-10-03: Implemented.
  - Geared rotors with step-up generators: 900 kg at 210 kt closes at
    13,760 lb, against 14,877 lb with the specific-power machines. Max
    payload at 210 kt rises to 1,014 kg. The optimizer runs fast machines
    (motor about 1,377 rad/s, ratio about 31; generator stepped up to about
    1,405 rad/s).
  - Generators on the engine's 1,210 rpm shaft: 1,464 kg of generators, so
    max payload at 210 kt is only 33 kg.
  - Direct-drive rotors: no feasible design. At about 52 rad/s each motor
    needs about 31 kN.m peak torque, roughly 2 t at 15 N.m/kg.
  - Conclusion: geared rotors and a step-up generator gearbox are required.
  - Torque trap: climb (slow rotor at the lambda bound, high power) sets
    peak motor torque at 568 N.m, against 414 N.m in hover.

## Deferred

- multi-stage gearbox mass and loss growth with ratio;
- machine thermal limits (Tier 19);
- inverter split (Tier 15);
- an explicit stage-count discrete choice.
