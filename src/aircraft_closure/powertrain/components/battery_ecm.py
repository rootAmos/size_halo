"""Equivalent-circuit battery pack (Tier 17): OCV(SOC), R0 + two RC branches, series/parallel cells.

Circuit per cell: OCV(SOC) in series with R0 and two RC branches (R1 || C1,
R2 || C2; time constants tau_k = R_k C_k), as in Paudel et al., Batteries
2025, 11, 313 (Samsung INR21700-50G). The pack is `count_series` cells in
series and `count_parallel` (continuous in sizing) strings in parallel, so pack
voltage = count_series x cell voltage and pack resistance = cell resistance x
count_series / count_parallel. Positive current discharges.

Operating relations for a constant current I over `duration_s` (dt), starting
from SOC s0 and RC voltages v_k0 (pack level):

* SOC (coulomb counting): s1 = s0 - I dt / Q_pack.
* OCV and resistances are evaluated at the mid-interval SOC (s0 + s1) / 2, so
  sub-segmented missions integrate OCV dQ to second order.
* RC branch k (exact for constant I): v_k(dt) = I R_k + (v_k0 - I R_k) e^(-dt/tau_k);
  its interval mean is I R_k + (v_k0 - I R_k) g_k with g_k = (tau_k/dt)(1 - e^(-dt/tau_k)).
* Terminal (interval-mean) voltage V = OCV - sum(v_k mean) - I R0 = V* - I R_eff,
  with V* = OCV - sum(v_k0 g_k) (current-independent part) and
  R_eff = R0 + sum R_k (1 - g_k).
* Chemical power = OCV I; terminal power = V I; loss = chemical - terminal.
* Power demanded = V I gives R_eff I^2 - V* I + P = 0. The physical root is the
  low-current one; the caller's Opti selects it with V >= V*/2, which also
  enforces P <= P_max = V*^2 / (4 R_eff) (`voltage_driving_V`, `power_max_W`).

`voltage_rc_start_V=None` (no RC history) uses the steady state v_k = I R_k:
the polarization fully developed, i.e. a constant load held for longer than
about 3 tau_2 (~2 min). That is conservative for short pulses. Missions pass
the RC state explicitly from point to point (starting from rest).

Cell resistance: R_k(SOC, T) = R_k,ref x exp(a x + b x^2) x (1 + c e^(-(s - s_lo)/w_lo)
+ d e^((s - s_hi)/w_hi)), x = T_ref / T - 1 (temperature in kelvin). Pack
resistance is divided by `factor_power_density` (the user's explicit scaling
assumption: shape from the 50G, magnitude as power-dense as the design needs)
and multiplied by `factor_resistance_ageing` (end of life). The current
rating scales with `factor_power_density` too.

Outside the fitted data (SOC 0.2-0.8 for resistance; 0-1 for OCV;
-10..45 C): the resistance SOC terms continue exponentially (resistance keeps
rising towards empty, conservative); the OCV polynomial is not clamped and
must stay inside 0..1 (missions bound SOC by `min_soc`/`max_soc`). The
tabulated OCV holds its end values. The model does not clip or enforce any
limit; the caller constrains current, voltage and SOC.
"""
from dataclasses import dataclass, field
from functools import cached_property
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

kelvin_offset_K = 273.15


@dataclass(frozen=True)
class PolynomialOcvModel:
    """Cell OCV [V] = polynomial in SOC (coefficients highest power first); smooth and cheap for IPOPT."""
    coefficients: tuple
    source: str = ""

    def voltage_V(self, soc):
        voltage_V = 0 * soc + self.coefficients[0]
        for coefficient in self.coefficients[1:]:
            voltage_V = voltage_V * soc + coefficient
        return voltage_V

    def mean_voltage_V(self, soc_low=0.0, soc_high=1.0):
        """Mean OCV over an SOC interval (exact polynomial integral): energy = Q x mean OCV x delta SOC."""
        degree = len(self.coefficients) - 1
        antiderivative = lambda s: sum(c * s**(degree - i + 1) / (degree - i + 1)
                                       for i, c in enumerate(self.coefficients))
        return (antiderivative(soc_high) - antiderivative(soc_low)) / (soc_high - soc_low)


@dataclass(frozen=True)
class TabulatedOcvModel:
    """Cell OCV interpolated on the digitized nodes (AeroSandbox B-spline); end values held outside."""
    soc: tuple
    voltage_open_circuit_V: tuple
    source: str = ""

    @cached_property
    def _table(self):
        return asb.InterpolatedModel(x_data_coordinates=np.array(self.soc),
                                     y_data_structured=np.array(self.voltage_open_circuit_V), method="bspline",
                                     fill_value=None)  # None: hold end values

    def voltage_V(self, soc):
        return self._table(soc)

    def mean_voltage_V(self, soc_low=0.0, soc_high=1.0):
        """Trapezoidal mean of the B-spline over the interval (numeric bounds only)."""
        grid = np.linspace(soc_low, soc_high, 401)
        values = self.voltage_V(grid)
        return np.sum((values[1:] + values[:-1]) / 2 * np.diff(grid)) / (soc_high - soc_low)


@dataclass(frozen=True)
class ResistanceBranch:
    """One series element: R0 (time_constant_s None) or an RC branch; cell-level values at the reference."""
    resistance_ref_ohm: Any
    time_constant_s: Any = None
    exponent_temperature_linear: float = 0.0
    exponent_temperature_quadratic: float = 0.0
    coefficient_soc_low: float = 0.0
    coefficient_soc_high: float = 0.0


@dataclass(frozen=True)
class CellResistanceModel:
    """Smooth R(SOC, T) for R0 and the RC branches (see module docstring for the form)."""
    branches: tuple
    temperature_reference_C: float = 25.0
    soc_low: float = 0.2
    soc_high: float = 0.8
    width_soc_low: float = 0.1
    width_soc_high: float = 0.05
    softness_soc_high: float = 0.01
    source: str = ""

    def resistances_ohm(self, soc, temperature_C):
        """Cell resistances (R0, R1, R2, ...) at the operating point."""
        x = (self.temperature_reference_C + kelvin_offset_K) / (temperature_C + kelvin_offset_K) - 1
        low = np.exp(-(soc - self.soc_low) / self.width_soc_low)
        # Above the data (SOC > 0.8) the high-SOC term is held (smooth minimum), not extrapolated.
        high = np.exp((np.softmin(soc, self.soc_high, softness=self.softness_soc_high) - self.soc_high)
                      / self.width_soc_high)
        return tuple(b.resistance_ref_ohm
                     * np.exp(b.exponent_temperature_linear * x + b.exponent_temperature_quadratic * x**2)
                     * (1 + b.coefficient_soc_low * low + b.coefficient_soc_high * high)
                     for b in self.branches)

    def time_constants_s(self):
        return tuple(b.time_constant_s for b in self.branches[1:])

    def resistance_dc_ohm(self, soc, temperature_C, duration_pulse_s):
        """DCIR of a constant-current pulse from rest (OCV drift excluded): R0 + sum R_k (1 - e^(-t/tau_k))."""
        resistances_ohm = self.resistances_ohm(soc, temperature_C)
        return resistances_ohm[0] + sum(r * (1 - np.exp(-duration_pulse_s / tau))
                                        for r, tau in zip(resistances_ohm[1:], self.time_constants_s()))


@dataclass(frozen=True)
class LithiumIonCell:
    """Cell data (rated values from the datasheet table) plus OCV and resistance submodels."""
    capacity_As: float
    mass_kg: float
    voltage_nominal_V: float
    voltage_max_V: float
    voltage_min_V: float
    current_max_discharge_A: float
    current_max_charge_A: float
    ocv_model: Any
    resistance_model: Any
    source: str = ""


def inr21700_50g_ocv_model():
    """Degree-7 least-squares polynomial through the 21 digitized 30 C OCV points (Fig. 7, Paudel et al. 2025);
    rms 9 mV, max 21 mV, monotonic on 0..1 (`cells.fit_ocv_polynomial`)."""
    return PolynomialOcvModel(coefficients=(
        49.61180607503143, -166.02992448421182, 226.02487919114515, -167.11922685349367, 77.10718563734386,
        -23.702004925522193, 5.388723669513503, 2.9040578016187197),
        source="Paudel et al. 2025 Fig. 7, 30 C, degree-7 fit")


def inr21700_50g_resistance_model():
    """Global least-squares fit (`cells.fit_resistance_model`) to all 308 discharge DCIR points of Fig. 8
    (vector data; 2/10/30/180 s pulses, SOC 0.2-0.8, -10..45 C), after removing the OCV drift during the
    pulse. Time constants fixed at the Fig. 12 25 C values (8 s, 43 s); R1 shares R0's temperature factor
    and has no SOC term. rms 4.7 %, max 21 % (relative)."""
    return CellResistanceModel(branches=(
        ResistanceBranch(0.02392439884083178, None, 3.727089416999324, 38.829103847261536, 0.041208063365828655,
                         0.03928296686137162),
        ResistanceBranch(0.001788124149185901, 8.0, 3.727089416999324, 38.829103847261536, 0.0, 0.0),
        ResistanceBranch(0.015751230453774544, 43.0, 6.537132280562984, -22.035358590567608, 0.9475754210356837,
                         0.2670367872558247),
    ), temperature_reference_C=25.0, width_soc_low=0.09439550548038171, width_soc_high=0.040069900449445306,
        source="Paudel et al. 2025 Fig. 8 discharge DCIR, 2-RC fit")


def inr21700_50g_cell():
    """Samsung INR21700-50G (NCA, Table 1 of Paudel et al. 2025): 4.9 Ah, 69 g, 3.6 V nominal, 2.5-4.2 V,
    9.8 A continuous discharge. Charge rating 4.9 A (1C) is an assumption (half the discharge rating, the
    legacy `Battery` convention)."""
    return LithiumIonCell(capacity_As=4.9 * 3600, mass_kg=0.069, voltage_nominal_V=3.6, voltage_max_V=4.2,
                          voltage_min_V=2.5, current_max_discharge_A=9.8, current_max_charge_A=4.9,
                          ocv_model=inr21700_50g_ocv_model(), resistance_model=inr21700_50g_resistance_model(),
                          source="Samsung INR21700-50G, Paudel et al., Batteries 2025, 11, 313 (CC BY 4.0)")


@dataclass(frozen=True)
class EquivalentCircuitBatteryResult:
    voltage_V: Any                      # terminal voltage, interval mean (the bus voltage)
    current_A: Any
    power_electric_W: Any               # terminal power V I
    power_loss_W: Any                   # chemical - terminal (R0 and RC branches)
    power_chemical_W: Any               # OCV I
    soc_next: Any
    voltage_open_circuit_V: Any         # at the mid-interval SOC
    voltage_rc_V: tuple                 # interval-mean RC branch voltages
    voltage_rc_end_V: tuple             # RC branch voltages at the end of the interval (next point's start)
    voltage_end_V: Any                  # terminal voltage at the end of the interval
    voltage_driving_V: Any              # V* = OCV - current-independent RC part; branch: voltage_V >= V*/2
    resistance_effective_ohm: Any       # R_eff: V = V* - I R_eff
    power_max_W: Any                    # V*^2 / (4 R_eff), the matched-load ceiling


@dataclass(frozen=True)
class EquivalentCircuitBatteryLimits:
    min_voltage_V: Any
    max_voltage_V: Any
    max_discharge_current_A: Any
    max_charge_current_A: Any
    min_soc: float
    max_soc: float


@dataclass(frozen=True)
class EquivalentCircuitBattery:
    count_series: Any = 210
    count_parallel: Any = 20.0
    cell: Any = field(default_factory=inr21700_50g_cell)
    factor_power_density: Any = 1.0         # resistances / factor, current rating x factor (1 = the 50G)
    temperature_cell_C: Any = 25.0
    factor_capacity_ageing: Any = 1.0       # remaining capacity fraction (end of life e.g. 0.8)
    factor_resistance_ageing: Any = 1.0     # resistance growth (end of life e.g. 1.5)
    fraction_mass_cells: float = 0.7        # cell mass / pack mass (assumed, cylindrical-cell packs ~0.65-0.75)
    min_soc: float = 0.1
    max_soc: float = 0.95
    # Tier 19: LumpedThermalModel or None. The pack temperature is reported and limited; it does not yet feed
    # back into `temperature_cell_C` (the resistance stays at the managed temperature, plan 028).
    thermal_model: Any = None

    @property
    def count_cells(self):
        return self.count_series * self.count_parallel

    @property
    def capacity_As(self):
        return self.count_parallel * self.cell.capacity_As * self.factor_capacity_ageing

    @property
    def energy_capacity_J(self):
        """Energy from full to empty at open circuit: Q x mean OCV(0..1) x count_series."""
        return self.capacity_As * self.count_series * self.cell.ocv_model.mean_voltage_V(0.0, 1.0)

    @property
    def power_max_discharge_W(self):
        """Rated discharge power: current rating at nominal pack voltage."""
        return self.get_limits().max_discharge_current_A * self.count_series * self.cell.voltage_nominal_V

    def voltage_open_circuit_V(self, soc):
        return self.count_series * self.cell.ocv_model.voltage_V(soc)

    def resistances_ohm(self, soc, temperature_C=None):
        """Pack resistances (R0, R1, R2) including the power-density and ageing factors."""
        temperature_C = self.temperature_cell_C if temperature_C is None else temperature_C
        scale = self.count_series / self.count_parallel * self.factor_resistance_ageing / self.factor_power_density
        return tuple(r * scale for r in self.cell.resistance_model.resistances_ohm(soc, temperature_C))

    def get_mass(self):
        return self.count_cells * self.cell.mass_kg / self.fraction_mass_cells

    def get_limits(self):
        c = self.cell
        return EquivalentCircuitBatteryLimits(
            min_voltage_V=self.count_series * c.voltage_min_V, max_voltage_V=self.count_series * c.voltage_max_V,
            max_discharge_current_A=self.count_parallel * c.current_max_discharge_A * self.factor_power_density,
            max_charge_current_A=self.count_parallel * c.current_max_charge_A * self.factor_power_density,
            min_soc=self.min_soc, max_soc=self.max_soc)

    def evaluate(self, current_A, soc, duration_s=0.0, voltage_rc_start_V=None, temperature_C=None):
        soc_next = soc - current_A * duration_s / self.capacity_As
        soc_mid = (soc + soc_next) / 2
        voltage_open_circuit_V = self.voltage_open_circuit_V(soc_mid)
        resistance_0_ohm, *resistances_rc_ohm = self.resistances_ohm(soc_mid, temperature_C)
        time_constants_s = self.cell.resistance_model.time_constants_s()
        is_instant = isinstance(duration_s, (int, float)) and duration_s == 0
        voltage_rc_V, voltage_rc_end_V, offsets_V, slopes_ohm = [], [], [], []
        for k, (r_ohm, tau_s) in enumerate(zip(resistances_rc_ohm, time_constants_s)):
            steady_V = current_A * r_ohm
            if voltage_rc_start_V is None:
                # No history: polarization fully developed (steady state).
                mean_fraction, end_fraction, start_V = 0.0, 0.0, 0.0
            elif is_instant:
                mean_fraction, end_fraction, start_V = 1.0, 1.0, voltage_rc_start_V[k]
            else:
                end_fraction = np.exp(-duration_s / tau_s)
                mean_fraction, start_V = tau_s / duration_s * (1 - end_fraction), voltage_rc_start_V[k]
            voltage_rc_V.append(steady_V + (start_V - steady_V) * mean_fraction)
            voltage_rc_end_V.append(steady_V + (start_V - steady_V) * end_fraction)
            offsets_V.append(start_V * mean_fraction)
            slopes_ohm.append(r_ohm * (1 - mean_fraction))
        voltage_driving_V = voltage_open_circuit_V - sum(offsets_V)
        resistance_effective_ohm = resistance_0_ohm + sum(slopes_ohm)
        voltage_V = voltage_driving_V - current_A * resistance_effective_ohm
        voltage_end_V = voltage_open_circuit_V - sum(voltage_rc_end_V) - current_A * resistance_0_ohm
        power_chemical_W = voltage_open_circuit_V * current_A
        power_electric_W = voltage_V * current_A
        return EquivalentCircuitBatteryResult(
            voltage_V=voltage_V, current_A=current_A, power_electric_W=power_electric_W,
            power_loss_W=power_chemical_W - power_electric_W, power_chemical_W=power_chemical_W, soc_next=soc_next,
            voltage_open_circuit_V=voltage_open_circuit_V, voltage_rc_V=tuple(voltage_rc_V),
            voltage_rc_end_V=tuple(voltage_rc_end_V), voltage_end_V=voltage_end_V,
            voltage_driving_V=voltage_driving_V, resistance_effective_ohm=resistance_effective_ohm,
            power_max_W=voltage_driving_V**2 / (4 * resistance_effective_ohm))


if __name__ == "__main__":
    pack = EquivalentCircuitBattery()
    print(pack.get_mass(), pack.energy_capacity_J / 3.6e6, pack.evaluate(100.0, 0.5, 60.0, (0.0, 0.0)))
