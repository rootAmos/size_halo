"""Aircraft assembly: composes physical components and aggregates mass and CG.

The assembly never solves. Mass closure is the caller's explicit equality
`mass_takeoff_kg == aircraft.get_mass(StructuralDesignCondition(mass_takeoff_kg))`.
CG aggregation is AeroSandbox's `MassProperties` addition.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb


def _optional_mass_properties(item):
    """Items without a design condition; an absent item weighs nothing."""
    return item.get_mass_properties() if item is not None else asb.MassProperties(mass=0)


@dataclass(frozen=True)
class MassBreakdown:
    wing: Any
    horizontal_tail: Any
    vertical_tail: Any
    fuselage: Any
    landing_gear: Any
    systems: Any
    powertrain: Any
    payload: Any
    fuel: Any
    nacelles: Any
    drive_shaft: Any
    equipment: Any

    def total(self):
        return (self.wing + self.horizontal_tail + self.vertical_tail + self.fuselage + self.landing_gear
                + self.systems + self.powertrain + self.payload + self.fuel + self.nacelles + self.drive_shaft
                + self.equipment)

    def mass_empty_kg(self):
        """Everything except payload and fuel."""
        return self.total().mass - self.payload.mass - self.fuel.mass


@dataclass(frozen=True)
class Aircraft:
    wing: Any
    horizontal_tail: Any
    vertical_tail: Any
    fuselage: Any
    landing_gear: Any
    systems: Any
    powertrain: Any
    payload: Any
    fuel: Any = None
    nacelles: Any = None
    drive_shaft: Any = None
    equipment: Any = None

    def distance_wing_to_tail_m(self):
        """Root quarter-chord of the wing to that of the horizontal tail (Raymer)."""
        return ((self.horizontal_tail.x_le_root_m + 0.25 * self.horizontal_tail.chord_root_m())
                - (self.wing.x_le_root_m + 0.25 * self.wing.chord_root_m()))

    def get_mass_breakdown(self, condition):
        return MassBreakdown(
            wing=self.wing.get_mass_properties(condition),
            horizontal_tail=self.horizontal_tail.get_mass_properties(condition),
            vertical_tail=self.vertical_tail.get_mass_properties(condition),
            fuselage=self.fuselage.get_mass_properties(condition, self.distance_wing_to_tail_m()),
            landing_gear=self.landing_gear.get_mass_properties(condition),
            systems=self.systems.get_mass_properties(condition, self.wing, self.fuselage),
            powertrain=self.powertrain.get_mass_properties(),
            payload=self.payload.get_mass_properties(),
            fuel=_optional_mass_properties(self.fuel),
            nacelles=_optional_mass_properties(self.nacelles),
            drive_shaft=_optional_mass_properties(self.drive_shaft),
            equipment=_optional_mass_properties(self.equipment),
        )

    def get_mass(self, condition):
        return self.get_mass_breakdown(condition).total().mass

    def get_cg_x_m(self, condition):
        return self.get_mass_breakdown(condition).total().x_cg

    def to_asb(self):
        return asb.Airplane(name="aircraft",
                            wings=[self.wing.to_asb(), self.horizontal_tail.to_asb(), self.vertical_tail.to_asb()],
                            fuselages=[self.fuselage.to_asb()]
                            + (self.nacelles.to_asb() if self.nacelles is not None else []))
