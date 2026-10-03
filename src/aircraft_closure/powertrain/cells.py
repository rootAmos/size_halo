"""Battery cell data: digitized tables and the fits behind the Tier 17 cell models (data layer).

The CSV files in `data/batteries/` are digitized from S. Paudel, J. Zhang,
B. Ayalew, R. Singh, "Systematic Characterization of Lithium-Ion Cells for
Electric Mobility and Grid Storage: A Case Study on Samsung INR21700-50G",
Batteries 2025, 11, 313 (CC BY 4.0, https://doi.org/10.3390/batteries11080313);
each file's header records the figure, the method and the accuracy.

The fits reproduce the coefficients hard-coded in
`components/battery_ecm.py` (components cannot read files); tests check that
they agree. The resistance fit is a least-squares problem owned by an
`asb.Opti`, like any other optimization in this repository.
"""
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import aerosandbox as asb
import aerosandbox.numpy as np

from .components.battery_ecm import (CellResistanceModel, PolynomialOcvModel, ResistanceBranch, TabulatedOcvModel,
                                     inr21700_50g_cell)

data_directory = Path(__file__).resolve().parents[3] / "data" / "batteries"


def _read_rows(path):
    with open(path, encoding="utf-8") as file:
        return list(csv.DictReader(line for line in file if not line.startswith("#")))


@dataclass(frozen=True)
class OcvTable:
    temperature_C: Any
    soc: Any
    voltage_open_circuit_V: Any
    method: tuple

    def at_temperature(self, temperature_C):
        mask = self.temperature_C == temperature_C
        order = np.argsort(self.soc[mask])
        return self.soc[mask][order], self.voltage_open_circuit_V[mask][order]


@dataclass(frozen=True)
class DcirTable:
    temperature_C: Any
    soc: Any
    duration_pulse_s: Any
    discharge: Any                      # True: discharge pulse, False: charge (regen) pulse
    resistance_dc_ohm: Any
    power_pulse_W: Any

    def select(self, discharge=True):
        mask = self.discharge == discharge
        return DcirTable(self.temperature_C[mask], self.soc[mask], self.duration_pulse_s[mask], self.discharge[mask],
                         self.resistance_dc_ohm[mask], self.power_pulse_W[mask])


@dataclass(frozen=True)
class EcmTable:
    temperature_C: Any
    soc: Any
    resistance_0_ohm: Any
    resistance_1_ohm: Any
    resistance_2_ohm: Any
    time_constant_1_s: Any
    time_constant_2_s: Any


def _column(rows, name):
    return np.array([r[name] if r[name] != "" else "nan" for r in rows], dtype=float)


def load_ocv_table(path=data_directory / "inr21700_50g_ocv.csv"):
    rows = _read_rows(path)
    return OcvTable(_column(rows, "temperature_C"), _column(rows, "soc"), _column(rows, "voltage_open_circuit_V"),
                    tuple(r["method"] for r in rows))


def load_dcir_table(path=data_directory / "inr21700_50g_dcir.csv"):
    rows = _read_rows(path)
    return DcirTable(_column(rows, "temperature_C"), _column(rows, "soc"), _column(rows, "duration_pulse_s"),
                     np.array([r["direction"] == "discharge" for r in rows]), _column(rows, "resistance_dc_ohm"),
                     _column(rows, "power_pulse_W"))


def load_ecm_table(path=data_directory / "inr21700_50g_ecm_discharge.csv"):
    rows = _read_rows(path)
    return EcmTable(*(_column(rows, name) for name in ("temperature_C", "soc", "resistance_0_ohm", "resistance_1_ohm",
                                                       "resistance_2_ohm", "time_constant_1_s", "time_constant_2_s")))


def fit_ocv_polynomial(soc, voltage_V, degree=7):
    """Least-squares polynomial coefficients (highest power first)."""
    design = np.stack([np.array(soc)**(degree - i) for i in range(degree + 1)], axis=1)
    return tuple(np.linalg.lstsq(design, np.array(voltage_V), rcond=None)[0])


def tabulated_ocv_model(temperature_C=30.0, table=None):
    """B-spline through the digitized OCV nodes at one temperature (30 C: complete 0-1 range)."""
    table = table if table is not None else load_ocv_table()
    soc, voltage_V = table.at_temperature(temperature_C)
    return TabulatedOcvModel(tuple(soc), tuple(voltage_V), source=f"Paudel et al. 2025 Fig. 7, {temperature_C:g} C")


def resistance_dc_corrected_ohm(dcir, ocv_model, capacity_cell_As):
    """DCIR minus the OCV drift during the pulse: dOCV/dSOC x t / Q (current-independent for a constant pulse)."""
    step = 1e-4
    slope_V = (ocv_model.voltage_V(dcir.soc + step) - ocv_model.voltage_V(dcir.soc - step)) / (2 * step)
    return dcir.resistance_dc_ohm - slope_V * dcir.duration_pulse_s / capacity_cell_As


def fit_resistance_model(dcir=None, ocv_model=None, capacity_cell_As=4.9 * 3600, time_constants_s=(8.0, 43.0),
                         temperature_reference_C=25.0, initial=None):
    """Global least squares (relative residuals) of `CellResistanceModel.resistance_dc_ohm` to discharge DCIR.

    R1 shares R0's temperature exponents and has no SOC term (the 2-180 s pulses do not resolve more).
    Returns (model, relative residuals).
    """
    dcir = (dcir if dcir is not None else load_dcir_table()).select(discharge=True)
    ocv_model = ocv_model if ocv_model is not None else inr21700_50g_cell().ocv_model
    target_ohm = resistance_dc_corrected_ohm(dcir, ocv_model, capacity_cell_As)
    start = initial if initial is not None else inr21700_50g_cell().resistance_model
    b0, b1, b2 = start.branches
    opti = asb.Opti()
    r0 = opti.variable(init_guess=b0.resistance_ref_ohm, scale=0.01, lower_bound=0.0)
    r1 = opti.variable(init_guess=b1.resistance_ref_ohm, scale=0.001, lower_bound=0.0)
    r2 = opti.variable(init_guess=b2.resistance_ref_ohm, scale=0.01, lower_bound=0.0)
    a0, q0, a2, q2 = (opti.variable(init_guess=v) for v in (b0.exponent_temperature_linear,
                                                             b0.exponent_temperature_quadratic,
                                                             b2.exponent_temperature_linear,
                                                             b2.exponent_temperature_quadratic))
    c0, d0, c2, d2 = (opti.variable(init_guess=v, lower_bound=-1.0, upper_bound=10.0)
                      for v in (b0.coefficient_soc_low, b0.coefficient_soc_high, b2.coefficient_soc_low,
                                b2.coefficient_soc_high))
    width_low = opti.variable(init_guess=start.width_soc_low, lower_bound=0.01, upper_bound=0.5)
    width_high = opti.variable(init_guess=start.width_soc_high, lower_bound=0.01, upper_bound=0.5)
    model = CellResistanceModel(branches=(
        ResistanceBranch(r0, None, a0, q0, c0, d0),
        ResistanceBranch(r1, time_constants_s[0], a0, q0, 0.0, 0.0),
        ResistanceBranch(r2, time_constants_s[1], a2, q2, c2, d2),
    ), temperature_reference_C=temperature_reference_C, width_soc_low=width_low, width_soc_high=width_high)
    residual = model.resistance_dc_ohm(dcir.soc, dcir.temperature_C, dcir.duration_pulse_s) / target_ohm - 1
    opti.minimize(np.sum(residual**2))
    solution = opti.solve(verbose=False)
    value = solution.value
    fitted = CellResistanceModel(branches=(
        ResistanceBranch(value(r0), None, value(a0), value(q0), value(c0), value(d0)),
        ResistanceBranch(value(r1), time_constants_s[0], value(a0), value(q0), 0.0, 0.0),
        ResistanceBranch(value(r2), time_constants_s[1], value(a2), value(q2), value(c2), value(d2)),
    ), temperature_reference_C=temperature_reference_C, width_soc_low=value(width_low),
        width_soc_high=value(width_high), source="refit of Paudel et al. 2025 Fig. 8 discharge DCIR")
    return fitted, value(residual)


def pulse_power_W(voltage_open_circuit_V, resistance_dc_ohm, voltage_min_pulse_V=2.5):
    """Paudel et al. eq. (12) / US DOE HPPC: discharge pulse power capability at the minimum pulse voltage."""
    return voltage_min_pulse_V * (voltage_open_circuit_V - voltage_min_pulse_V) / resistance_dc_ohm


if __name__ == "__main__":
    table = load_ocv_table()
    soc, voltage_V = table.at_temperature(30.0)
    print("OCV coefficients", fit_ocv_polynomial(soc, voltage_V))
    model, residual = fit_resistance_model()
    print(model)
    print("rms", np.sqrt(np.mean(residual**2)), "max", np.max(np.abs(residual)))
