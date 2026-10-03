"""Landing gear, systems, payload, nacelle, drive-shaft and fixed-equipment mass items."""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer

from aircraft_closure.weights import afdd


@dataclass(frozen=True)
class LandingGear:
    """Tricycle gear; Raymer GA main and nose correlations (fixed unless `is_retractable`)."""
    length_main_m: Any = 0.6
    length_nose_m: Any = 0.5
    x_main_m: Any = 3.6
    x_nose_m: Any = 1.0
    z_m: Any = -0.6
    mass_factor: Any = 1.0
    is_retractable: bool = False

    def get_mass_properties(self, condition):
        mass_main_kg = raymer.mass_main_landing_gear(self.length_main_m, condition.mass_design_kg,
                                                     is_retractable=self.is_retractable)
        mass_nose_kg = raymer.mass_nose_landing_gear(self.length_nose_m, condition.mass_design_kg,
                                                     is_retractable=self.is_retractable)
        return (asb.MassProperties(mass=self.mass_factor * mass_main_kg, x_cg=self.x_main_m, z_cg=self.z_m)
                + asb.MassProperties(mass=self.mass_factor * mass_nose_kg, x_cg=self.x_nose_m, z_cg=self.z_m))


@dataclass(frozen=True)
class Systems:
    """Flight controls (Raymer GA) plus installed avionics (Raymer GA).

    Raymer's flight-control correlation is for manned aircraft; `mass_factor`
    is the place to calibrate it for an unmanned fly-by-wire system.
    """
    mass_avionics_uninstalled_kg: Any = 30.0
    x_m: Any = 1.5
    z_m: Any = 0.0
    mass_factor: Any = 1.0

    def get_mass_properties(self, condition, wing, fuselage):
        asb_wing = wing.to_asb()
        asb_fuselage = fuselage.to_asb()
        mass_flight_controls_kg = raymer.mass_flight_controls(
            asb.Airplane(wings=[asb_wing], fuselages=[asb_fuselage]),
            design_mass_TOGW=condition.mass_design_kg, ultimate_load_factor=condition.load_factor_ultimate,
            fuselage=asb_fuselage, main_wing=asb_wing)
        mass_avionics_kg = raymer.mass_avionics(self.mass_avionics_uninstalled_kg)
        return asb.MassProperties(mass=self.mass_factor * (mass_flight_controls_kg + mass_avionics_kg),
                                  x_cg=self.x_m, z_cg=self.z_m)


@dataclass(frozen=True)
class Payload:
    mass_kg: Any = 300.0
    x_m: Any = 3.0
    z_m: Any = -0.1

    def get_mass_properties(self):
        return asb.MassProperties(mass=self.mass_kg, x_cg=self.x_m, z_cg=self.z_m)


@dataclass(frozen=True)
class FuelLoad:
    """Usable fuel at take-off as a point mass at the tank position."""
    mass_kg: Any = 0.0
    x_m: Any = 3.5
    z_m: Any = 0.0

    def get_mass_properties(self):
        return asb.MassProperties(mass=self.mass_kg, x_cg=self.x_m, z_cg=self.z_m)


@dataclass(frozen=True)
class Nacelles:
    """Engine section of all nacelles: support, cowling and air induction (AFDD82).

    `mass_engines_kg` is the dry mass of all engines the nacelles carry, usually
    the installed turboshaft mass expression. Pylon structure is not included.

    Optional geometry (Tier 21): a symmetric pair of bodies of revolution of
    `length_m` x `diameter_m` centred at (`x_m`, +/-`y_m`, `z_m`); the spinner is
    the body's nose. Without it the nacelles have no aerodynamic shape.
    """
    mass_engines_kg: Any
    count_engines: Any = 2
    area_wetted_m2: Any = 17.7
    fraction_air_induction: Any = 0.3
    x_m: Any = 3.0
    z_m: Any = 0.6
    mass_factor: Any = 1.0
    length_m: Any = None
    diameter_m: Any = None
    y_m: Any = 0.0

    def to_asb(self):
        """The nacelle bodies as AeroSandbox fuselages (empty without geometry)."""
        if self.length_m is None:
            return []
        radius_m = self.diameter_m / 2
        # Spinner nose to 25 % length, full section to 70 %, tapering to a 30 % radius at the tail.
        stations = [(0.0, 0.05), (0.25, 1.0), (0.7, 1.0), (1.0, 0.3)]
        return [asb.Fuselage(name=f"nacelle_{side}", xsecs=[
            asb.FuselageXSec(xyz_c=[self.x_m + (fraction - 0.5) * self.length_m, sign * self.y_m, self.z_m],
                             radius=radius_fraction * radius_m)
            for fraction, radius_fraction in stations]) for side, sign in (("right", 1), ("left", -1))]

    def get_mass_properties(self):
        mass_kg = (afdd.mass_engine_support_afdd82_kg(self.mass_engines_kg, self.count_engines,
                                                      self.fraction_air_induction)
                   + afdd.mass_air_induction_afdd82_kg(self.mass_engines_kg, self.count_engines,
                                                       self.fraction_air_induction)
                   + afdd.mass_engine_cowling_afdd82_kg(self.area_wetted_m2))
        return asb.MassProperties(mass=self.mass_factor * mass_kg, x_cg=self.x_m, z_cg=self.z_m)


@dataclass(frozen=True)
class InterconnectShaft:
    """Cross-shaft between rotors (AFDD82 drive shaft), sized by drive-system power and rotor speed."""
    power_drive_limit_W: Any
    speed_rotor_rad_s: Any
    length_m: Any
    count_drive_shafts: Any = 2
    fraction_power_second_rotor: Any = 0.6
    x_m: Any = 3.0
    z_m: Any = 0.6
    mass_factor: Any = 1.0

    def get_mass_properties(self):
        mass_kg = afdd.mass_drive_shaft_afdd82_kg(self.power_drive_limit_W, self.speed_rotor_rad_s, self.length_m,
                                                  self.count_drive_shafts, self.fraction_power_second_rotor)
        return asb.MassProperties(mass=self.mass_factor * mass_kg, x_cg=self.x_m, z_cg=self.z_m)


@dataclass(frozen=True)
class FixedEquipment:
    """Equipment carried at a given mass rather than predicted (electrical, environmental, furnishings)."""
    mass_kg: Any = 0.0
    x_m: Any = 3.0
    z_m: Any = 0.0

    def get_mass_properties(self):
        return asb.MassProperties(mass=self.mass_kg, x_cg=self.x_m, z_cg=self.z_m)
