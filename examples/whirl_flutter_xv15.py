"""XV-15 whirl flutter: the reduced-order model against CAMRAD II (Acree, Peyran and Johnson, NASA/TP-2004-212262).

The airframe is the published NASTRAN stick-model hub modes (CAMRAD II deck, appendix A; tables C2 and C4):
three symmetric and three antisymmetric wing modes, each analysed with the right rotor. The rotor is the XV-15
steel-blade gimballed rotor from the same deck (radius 12.5 ft, 3 blades, 14 in chord, gimbal spring 12,587
ft-lb/rad, delta-3 -15 deg, blade mass distribution for the flap inertia). Conditions as the report: 458 rpm, sea
level, limited-power trim (130,000 in-lb torque limit, reached at 275 kt on the thin wing).

CAMRAD II boundaries (flexible blades, lag, rigid drive train): thick 0.23 t/c wing 335 kt, thin 0.15 t/c wing
275 kt, both antisymmetric beam. Run from the repo root: `python -m examples.whirl_flutter_xv15`.

Result (2026-10-10): thick wing 399 kt, thin wing 342 kt, both symmetric beam (CAMRAD: antisymmetric beam).
The thick-to-thin drop is 57 kt against CAMRAD's 60 kt. One airspeed knockdown, 0.84, set on the thick wing
predicts the thin wing at 287 kt against 275 kt. Rigid in-plane blades (no lag, no compressibility, fixed delta-3)
stay stable to 500 kt, as the report's rigid-blade case does; the boundary comes from the pitched lag and coning
bending, the collective-dependent delta-3, Prandtl-Glauert lift slope and the pitch-lag coupling.
"""
from dataclasses import replace

import numpy as onp

from aircraft_closure.aeroelasticity.whirl_flutter import (GimballedProprotor, airframe_from_hub_modes,
                                                           flutter_speed, whirl_modes)

foot_m, slug_kg, knot_m_s, lbf_N = 0.3048, 14.5939, 0.514444, 4.44822

# Blade mass per length (slug/ft) at r/R = 0, 0.02, ..., 1.0 (deck RPROP/MASS); flap inertia about the gimbal.
mass_blade_slug_ft = [4.513, 0.638, 0.807, 1.032, 1.192, 1.156, 1.120, 0.853, 0.508, 0.246, 0.231, 0.216, 0.208,
                      0.202, 0.195, 0.184, 0.174, 0.164, 0.153, 0.146, 0.152, 0.157, 0.171, 0.189, 0.201, 0.197,
                      0.193, 0.189, 0.186, 0.182, 0.179, 0.175, 0.171, 0.168, 0.164, 0.161, 0.157, 0.154, 0.150,
                      0.147, 0.143, 0.140, 0.136, 0.132, 0.129, 0.125, 0.122, 0.133, 0.149, 0.161, 0.161]
radius_m = 12.5 * foot_m
stations_m = onp.linspace(0, 1, 51) * radius_m
mass_blade_kg_m = onp.array(mass_blade_slug_ft) * slug_kg / foot_m
inertia_flap_blade_kg_m2 = float(onp.trapezoid(mass_blade_kg_m * stations_m**2, stations_m))
moment_first_blade_kg_m = float(onp.trapezoid(mass_blade_kg_m * stations_m, stations_m))
# Chordwise stiffness (lb-ft2, deck EILAG) at the same stations.
stiffness_lag_lb_ft2 = [1111000.0] * 4 + [
    1259000, 1851000, 2440000, 3070000, 3720000, 4250000, 4470000, 4680000, 5060000, 5480000, 5660000, 5100000,
    4530000, 3980000, 3440000, 3060000, 3160000, 3250000, 3490000, 3770000, 3960000, 3870000, 3790000, 3690000,
    3600000, 3500000, 3410000, 3320000, 3230000, 3150000, 3060000, 2990000, 2910000, 2850000, 2780000, 2710000,
    2650000, 2590000, 2530000, 2470000, 2420000, 2370000, 2320000, 2280000, 2240000, 2210000, 2210000]


# Flapwise stiffness (lb-ft2, deck EIFLAP) at the same stations.
stiffness_flap_lb_ft2 = [1111000.0] * 4 + [
    1215000, 1632000, 2049000, 1911000, 1589000, 1287000, 1045000, 803000, 674000, 582000, 499000, 445000, 392000,
    355000, 323000, 294000, 278000, 261000, 246000, 232000, 217000, 202000, 188000, 175000, 163000, 151000, 140000,
    129000, 120000, 110000, 101000, 93400, 85300, 78100, 71200, 64400, 58300, 52200, 47200, 42400, 37800, 33700,
    29500, 26100, 22800, 20400, 20400]


def frequency_bending_per_rev(stiffness_lb_ft2, speed_rotor_rad_s, inplane, count_terms=6):
    """First bending frequency of the blade cantilevered at the hub, in per rev: Ritz on eta^2 ... eta^(n+1) with
    centrifugal stiffening (and the in-plane -Omega^2 softening for lag)."""
    eta = stations_m / radius_m
    powers = range(2, count_terms + 2)
    phi = [eta**k for k in powers]
    dphi = [k * eta**(k - 1) / radius_m for k in powers]
    ddphi = [k * (k - 1) * eta**(k - 2) / radius_m**2 for k in powers]
    stiffness_Nm2 = onp.array(stiffness_lb_ft2, dtype=float) * lbf_N * foot_m**2
    tension_per_omega2 = onp.array([onp.trapezoid((mass_blade_kg_m * stations_m)[i:], stations_m[i:])
                                    for i in range(len(stations_m))])
    mass = onp.array([[onp.trapezoid(mass_blade_kg_m * a * b, stations_m) for b in phi] for a in phi])
    stiffness = onp.array([[onp.trapezoid(stiffness_Nm2 * a * b, stations_m)
                            + speed_rotor_rad_s**2 * onp.trapezoid(tension_per_omega2 * da * db, stations_m)
                            for b, db in zip(ddphi, dphi)] for a, da in zip(ddphi, dphi)])
    if inplane:
        stiffness = stiffness - speed_rotor_rad_s**2 * mass
    eigenvalues = onp.linalg.eigvals(onp.linalg.solve(mass, stiffness))
    return float(onp.sqrt(onp.min(eigenvalues.real)) / speed_rotor_rad_s)


rotor = GimballedProprotor(radius_m=radius_m, count_blades=3, chord_m=14 * 0.0254,
                           inertia_flap_blade_kg_m2=inertia_flap_blade_kg_m2,
                           stiffness_gimbal_Nm_rad=12587 * lbf_N * foot_m,
                           pitch_flap_coupling=onp.tan(onp.radians(-18.0)),
                           collective_pitch_horn_level_rad=onp.radians(20.5), lift_slope=5.7, drag_coefficient=0.01,
                           fraction_root_cutout=0.12,
                           # Deck drive train reflected to rotor speed: engine 0.098 slug-ft2 x 35.2^2, gearbox
                           # 15.9 and rotor shaft 2 slug-ft2; governor omitted (about 1 Hz, below the wing modes).
                           inertia_drive_kg_m2=(0.098 * 35.2**2 + 15.9 + 2.0) * slug_kg * foot_m**2,
                           moment_first_blade_kg_m=moment_first_blade_kg_m, damping_blade_ratio=0.005,
                           # Prandtl-Glauert on the helical Mach number (Johnson, TM X-73213, a = 5.7); pitch-lag
                           # coupling (lag back / pitch up) inside the range of Johnson's figure 7, assumed.
                           speed_sound_m_s=340.3, pitch_lag_coupling=-0.2)
speed_rotor_rad_s = 458 * 2 * onp.pi / 60
rotor_rigid_blades = rotor
# Stiff in-plane (lag above 1/rev, 1.41/rev at the 589 rpm hover speed); coning from the flapwise stiffness.
rotor = replace(rotor, frequency_lag_per_rev=frequency_bending_per_rev(stiffness_lag_lb_ft2, speed_rotor_rad_s, True),
                frequency_coning_per_rev=frequency_bending_per_rev(stiffness_flap_lb_ft2, speed_rotor_rad_s, False))
# Symmetric modes: rotor speed free on the drive inertia; antisymmetric: held by the interconnect shaft.
rotors = dict(symmetric=replace(rotor, drive="free"), antisymmetric=replace(rotor, drive="locked"))
density_kg_m3 = 1.225

# Limited-power trim: level flight (drag ~ V^2) up to the torque limit, constant power above it.
power_limit_W = 130000 * lbf_N * 0.0254 * speed_rotor_rad_s
efficiency_propulsive = 0.8                                   # assumed
velocity_torque_limit_m_s = 275 * knot_m_s
thrust_limit_N = efficiency_propulsive * power_limit_W / velocity_torque_limit_m_s


def thrust_limited_power_N(velocity_m_s):
    ratio = velocity_m_s / velocity_torque_limit_m_s
    return thrust_limit_N * min(ratio**2, 1 / ratio)


def hub_modes(frequency_hz, mass_slug, damping, displacement_ft, rotation_rad_ft, labels):
    """CAMRAD hub modes (ft-normalized shapes, slug modal mass) in SI with the coordinate in metres."""
    return airframe_from_hub_modes([2 * onp.pi * f for f in frequency_hz], [m * slug_kg for m in mass_slug], damping,
                                   displacement_ft, [[r / foot_m for r in row] for row in rotation_rad_ft], labels)


# Thick (0.23 t/c) stick model, deck order (appendix A): sym beam, chord, torsion; antisym beam, torsion, chord.
thick = dict(
    frequency_hz=[3.293, 6.320, 8.340, 6.267, 7.075, 8.703],
    mass_slug=[201.611, 319.965, 50.109, 231.637, 242.992, 57.764],
    damping=[0.03, 0.07, 0.08, 0.05, 0.05, 0.06],
    displacement_ft=[[-0.0274534, 0.0641842, 1.0], [1.0, -0.928248, -0.275460], [-0.184042, 0.123503, -0.552480],
                     [0.270239, -0.304366, 0.943833], [0.707744, -0.398595, -0.576665],
                     [-0.249960, 0.0802357, -0.451574]],
    rotation_rad_ft=[[0.138543, -0.0250751, 0.00251207], [0.0321882, 0.112465, -0.190338],
                     [0.0405354, 0.202698, 0.0219114], [0.100003, -0.156089, -0.0304170],
                     [0.0212266, 0.195098, -0.124386], [0.0651168, 0.189523, 0.0308434]],
    labels=["sym beam", "sym chord", "sym torsion", "antisym beam", "antisym torsion", "antisym chord"])
# Thin (0.15 t/c) stick model, table C4 order: sym beam, chord, torsion; antisym beam, chord, torsion.
thin = dict(
    frequency_hz=[2.424, 5.104, 5.974, 4.570, 6.522, 5.484],
    mass_slug=[194.819, 154.612, 66.537, 199.105, 117.104, 86.614],
    damping=[0.03, 0.07, 0.08, 0.05, 0.05, 0.06],
    displacement_ft=[[-0.0247409, 0.0619576, 1.0], [0.540728, -0.533949, -0.507410], [-0.359734, 0.281981, -0.544684],
                     [0.152875, -0.223944, 1.0], [0.550376, -0.231849, 0.417617], [-0.267452, 0.200962, 0.431073]],
    rotation_rad_ft=[[0.136220, -0.0288973, 0.00223698], [0.0480945, 0.196812, -0.111702],
                     [0.0406928, 0.201679, 0.0539970], [0.0796, -0.1953, -0.0095], [-0.0754, -0.1851, -0.0791],
                     [-0.0594, -0.1869, 0.0522]],
    labels=["sym beam", "sym chord", "sym torsion", "antisym beam", "antisym chord", "antisym torsion"])


def symmetry_airframes(data):
    """Symmetric (modes 1-3) and antisymmetric (modes 4-6) airframes, each analysed with the right rotor."""
    out = {}
    for name, index in (("symmetric", slice(0, 3)), ("antisymmetric", slice(3, 6))):
        out[name] = hub_modes(*(data[k][index] for k in ("frequency_hz", "mass_slug", "damping", "displacement_ft",
                                                          "rotation_rad_ft", "labels")))
    return out


def boundary_kt(data, rotor_models=None):
    """Lowest boundary over the symmetric and antisymmetric sets: (knots, mode label)."""
    rotor_models = rotor_models or rotors
    velocities_m_s = onp.arange(150, 501, 5) * knot_m_s
    results = []
    for symmetry, airframe in symmetry_airframes(data).items():
        v, label = flutter_speed(airframe, rotor_models[symmetry], speed_rotor_rad_s, density_kg_m3, velocities_m_s,
                                 thrust_limited_power_N)
        results.append((v / knot_m_s if v is not None else onp.inf, label))
    return min(results, key=lambda r: r[0])


# Calibration: one airspeed knockdown on the thick wing (CAMRAD II 335 kt), checked on the thin wing (275 kt).
boundary_camrad_kt = dict(thick=335.0, thin=275.0)


def calibration():
    """(knockdown, model thick kt, model thin kt, calibrated thin kt)."""
    thick_kt, thin_kt = boundary_kt(thick)[0], boundary_kt(thin)[0]
    knockdown = boundary_camrad_kt["thick"] / thick_kt
    return knockdown, thick_kt, thin_kt, knockdown * thin_kt


if __name__ == "__main__":
    print(f"XV-15 rotor: flap inertia {inertia_flap_blade_kg_m2:.0f} kg m2, Lock number "
          f"{rotor.lock_number(density_kg_m3):.2f}, gimbal {rotor.frequency_flap_per_rev(speed_rotor_rad_s):.3f}/rev, "
          f"lag {rotor.frequency_lag_per_rev:.3f}/rev, coning {rotor.frequency_coning_per_rev:.3f}/rev, "
          f"thrust at the torque limit {thrust_limit_N / 1e3:.1f} kN per rotor")
    for name, data, camrad_kt in (("thick 0.23 t/c", thick, 335), ("thin 0.15 t/c", thin, 275)):
        v_kt, label = boundary_kt(data)
        print(f"\n{name}: boundary {v_kt:.0f} kt ({label}); CAMRAD II {camrad_kt} kt (antisym beam)")
        for symmetry, airframe in symmetry_airframes(data).items():
            for v in (200, 250, 300, 350, 400):
                modes = whirl_modes(airframe, rotors[symmetry], v * knot_m_s, speed_rotor_rad_s, density_kg_m3,
                                    thrust_limited_power_N(v * knot_m_s))
                wing = [(l, z, f) for f, z, l in zip(modes.frequency_rad_s, modes.damping_ratio, modes.labels)
                        if l in thick["labels"] or l in thin["labels"]]
                print(f"  {symmetry:13s} {v} kt: " + ", ".join(f"{l} {100 * z:5.2f}% @ {f / 2 / onp.pi:4.1f} Hz"
                                                               for l, z, f in wing))
    knockdown, thick_kt, thin_kt, thin_calibrated_kt = calibration()
    print(f"\nKnockdown {knockdown:.3f} (thick: model {thick_kt:.0f} kt -> CAMRAD 335 kt); thin wing: model "
          f"{thin_kt:.0f} kt x knockdown = {thin_calibrated_kt:.0f} kt against CAMRAD 275 kt; drop thick to thin "
          f"{thick_kt - thin_kt:.0f} kt (CAMRAD 60 kt)")
