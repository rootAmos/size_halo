# AFDD wing cap depth and minimum gauge; pylon inertia and turbogenerator station (plan 035)

Status: implemented 2026-10-04. The user asked to "tackle 1 and 2" after the plan 031 CalculiX check.
- **Item 1** is the AFDD wing, found optimistic for the drawn box.
- **Item 2** is the open plan 032 assumptions.

## Changes

1. **AFDD tiltrotor wing options** (`weights/afdd.py`, `TiltrotorWingMassModel`). Both defaults are NDARC.
   - `ratio_depth_spar_cap` (default 1.0): the spar-cap separation / wing thickness. It scales the caps' bending
     lever arm: stiffness with its square, jump-take-off moment capacity linearly.
   - `thickness_min_torque_box_m` (default 0): the torque-box wall area is at least that gauge over the box
     perimeter. The realized torsion stiffness follows from the final area.
   - Halo:
     - κ from AeroSandbox `Airfoil.local_thickness` at the layout spars (0.15 and 0.60 chord), giving 0.826
       for NACA 2423;
     - minimum gauge 1.0 mm.
2. **Pylon pitch inertia from the tip components** (`ratio_radius_gyration_pylon=None`):
   - rotor at the hub, 1.0 m from the spindle;
   - motor and gearbox at 0.6 m;
   - cowling at a 0.8 m radius of gyration.
   - The 0.6 and 0.8 m values are assumed. The XV-15 ratio 0.222 R assumed engines in the nacelle.
3. **Turbogenerator station from the layout:** half an engine length (0.5 m) behind the rear spar, and 0.35 m
   below the fuselage top.

All earlier named sets pin NDARC κ = 1, no minimum gauge, the 0.222 ratio and the plan 032 station. The new
`assumptions_plan032` reproduces 12,821 lb.

## Results

**Reference: 5,913.9 kg (13,038 lb)**, +217 lb on plan 032.

| | Plan 032 | Plan 035 |
|---|---|---|
| Wing | 319.0 kg | 358.5 kg |
| Torque box | 50.1 kg | 75.4 kg (minimum gauge) |
| Stiffness caps | 4.0 kg | 15.7 kg |
| Jump caps | 35.2 kg | 22.4 kg |
| Whirl-flutter torsion at max speed | 1.09 per rev (binding) | 1.49 per rev (no longer binding) |

The pylon radius of gyration is 0.842 m, against 0.995 m with the XV-15 ratio.

## Tests

- **Unit tests:** AFDD defaults are NDARC; shallower caps need more cap mass and still meet the beam frequency;
  the minimum gauge floors the box and raises torsion.
- **Integration:** `test_halo_plan035`.
- **CalculiX re-check:** see progress.

## Progress

- 2026-10-04: implemented and solved. CalculiX re-check running.
