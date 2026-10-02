"""Installed powertrain: topology instances placed on the airframe.

Mass of each instance is `count * component.get_mass()`; the installation
factor covers mounts, cabling and cooling that the component models omit.
Symmetric multiplicity places all copies of an instance at one x/z position,
which is exact for CG when copies are mirrored about the symmetry plane.
"""
from dataclasses import dataclass
from typing import Any

import aerosandbox as asb


@dataclass(frozen=True)
class InstalledInstance:
    instance_name: str
    x_m: Any
    z_m: Any = 0.0


@dataclass(frozen=True)
class InstalledMass:
    instance_name: str
    mass_properties: Any


@dataclass(frozen=True)
class PowertrainInstallation:
    topology: Any
    locations: tuple
    installation_factor: Any = 1.0

    def __post_init__(self):
        located = [location.instance_name for location in self.locations]
        if len(set(located)) != len(located):
            raise ValueError(f"Instances located more than once: {sorted(located)}.")
        missing = set(self.topology.instances) - set(located)
        unknown = set(located) - set(self.topology.instances)
        if missing or unknown:
            raise ValueError(f"Locations must cover every instance exactly; missing {sorted(missing)}, "
                             f"unknown {sorted(unknown)}.")

    def get_instance_mass_properties(self):
        installed = []
        for location in self.locations:
            instance = self.topology.instances[location.instance_name]
            mass_kg = self.installation_factor * instance.count * instance.component.get_mass()
            installed.append(InstalledMass(location.instance_name,
                                           asb.MassProperties(mass=mass_kg, x_cg=location.x_m, z_cg=location.z_m)))
        return tuple(installed)

    def get_mass_properties(self):
        return sum((item.mass_properties for item in self.get_instance_mass_properties()), asb.MassProperties(mass=0))
