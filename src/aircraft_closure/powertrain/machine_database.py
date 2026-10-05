"""Public aerospace electric-machine data and the torque-density fit behind `DatabaseMassModel` (plan 033).

The data live in `data/machines/aerospace_motors.csv`, one row per machine. Every number there is cited by URL.
Ratings are SI here: torques in N m, powers in W, speeds in rad/s. Missing values are None.

Fit:
- continuous torque density T_cont / m against base speed w_base = P_cont / T_cont;
- the form is the power law tau(w) = tau_ref (w / w_ref)^(-a);
- it is ordinary least squares in log space;
- it uses only the rows marked `fit` (bare machines, or unknown inverter status, with published continuous
  torque and power).

Rows with an inverter included, and peak-only rows, are compared with the model and not fitted.
"""
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as onp

default_database_path = Path(__file__).resolve().parents[3] / "data" / "machines" / "aerospace_motors.csv"
speed_ref_default_rad_s = 500.0


@dataclass(frozen=True)
class MachineRecord:
    name: str
    manufacturer: str
    topology: str                  # "axial", "radial" or "unknown"
    mass_kg: float
    torque_continuous_Nm: object   # float or None (not published)
    torque_peak_Nm: object
    power_continuous_W: object
    power_peak_W: object
    speed_rated_rad_s: object
    speed_max_rad_s: object
    inverter_included: str         # "y", "n" or "unknown"
    stackable: str                 # "y", "n" or "unknown"
    fit: bool
    source_url: str
    notes: str

    def speed_base_rad_s(self):
        """P_cont / T_cont where both are published; else the rated speed (peak-only rows); else None."""
        if self.power_continuous_W is not None and self.torque_continuous_Nm is not None:
            return self.power_continuous_W / self.torque_continuous_Nm
        return self.speed_rated_rad_s

    def torque_density_continuous_Nm_kg(self):
        return None if self.torque_continuous_Nm is None else self.torque_continuous_Nm / self.mass_kg


def _number(text, scale=1.0):
    text = text.strip()
    return float(text) * scale if text else None


def load_machine_database(path=default_database_path):
    rpm_rad_s = 2 * onp.pi / 60
    with open(path, encoding="utf-8", newline="") as handle:
        return tuple(MachineRecord(
            name=row["name"], manufacturer=row["manufacturer"], topology=row["topology"],
            mass_kg=float(row["mass_kg"]), torque_continuous_Nm=_number(row["torque_continuous_Nm"]),
            torque_peak_Nm=_number(row["torque_peak_Nm"]), power_continuous_W=_number(row["power_continuous_kW"], 1e3),
            power_peak_W=_number(row["power_peak_kW"], 1e3), speed_rated_rad_s=_number(row["speed_rated_rpm"], rpm_rad_s),
            speed_max_rad_s=_number(row["speed_max_rpm"], rpm_rad_s), inverter_included=row["inverter_included"],
            stackable=row["stackable"], fit=row["fit"] == "y", source_url=row["source_url"], notes=row["notes"])
            for row in csv.DictReader(handle))


@dataclass(frozen=True)
class TorqueDensityFit:
    """tau(w) = torque_density_ref_Nm_kg * (w / speed_ref_rad_s)^(-exponent_speed); `residuals` is
    ((name, model mass / database mass), ...) over the fitted rows."""
    torque_density_ref_Nm_kg: float
    speed_ref_rad_s: float
    exponent_speed: float
    residuals: tuple
    rms_log_residual: float

    def torque_density_Nm_kg(self, speed_base_rad_s):
        return self.torque_density_ref_Nm_kg * (speed_base_rad_s / self.speed_ref_rad_s) ** (-self.exponent_speed)


def fit_torque_density(records, speed_ref_rad_s=speed_ref_default_rad_s):
    """Least squares on ln(tau) = ln(tau_ref) - a ln(w / w_ref) over the rows marked `fit`."""
    rows = [r for r in records if r.fit]
    log_speed = onp.log([r.speed_base_rad_s() / speed_ref_rad_s for r in rows])
    log_density = onp.log([r.torque_density_continuous_Nm_kg() for r in rows])
    matrix = onp.column_stack([onp.ones_like(log_speed), -log_speed])
    (log_tau_ref, exponent), *_ = onp.linalg.lstsq(matrix, log_density, rcond=None)
    fitted = log_tau_ref - exponent * log_speed
    # Model mass / database mass = database torque density / model torque density.
    ratios = onp.exp(log_density - fitted)
    return TorqueDensityFit(float(onp.exp(log_tau_ref)), speed_ref_rad_s, float(exponent),
                            tuple((r.name, float(x)) for r, x in zip(rows, ratios)),
                            float(onp.sqrt(onp.mean((log_density - fitted) ** 2))))


if __name__ == "__main__":
    fit = fit_torque_density(load_machine_database())
    print(f"tau = {fit.torque_density_ref_Nm_kg:.3f} N m/kg x (w / {fit.speed_ref_rad_s:.0f} rad/s)^-{fit.exponent_speed:.4f}"
          f", rms log residual {fit.rms_log_residual:.3f}")
    for name, ratio in fit.residuals:
        print(f"  {name:22s} model/database mass {ratio:.2f}")
