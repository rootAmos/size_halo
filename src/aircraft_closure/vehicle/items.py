"""Landing gear, systems and payload mass items."""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb
import aerosandbox.library.weights.raymer_general_aviation_weights as raymer


@dataclass(frozen=True)
class LandingGear:
    """Fixed tricycle gear; Raymer GA main and nose correlations (non-retractable)."""
    length_main_m: Any = 0.6
    length_nose_m: Any = 0.5
    x_main_m: Any = 3.6
    x_nose_m: Any = 1.0
    z_m: Any = -0.6
    mass_factor: Any = 1.0

    def get_mass_properties(self, condition):
        mass_main_kg = raymer.mass_main_landing_gear(self.length_main_m, condition.mass_design_kg,
                                                     is_retractable=False)
        mass_nose_kg = raymer.mass_nose_landing_gear(self.length_nose_m, condition.mass_design_kg,
                                                     is_retractable=False)
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
