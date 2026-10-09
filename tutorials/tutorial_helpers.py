"""Shared helpers for the tutorials: paths, plot style, checks, Opti inspection and a cached baseline solve.

Nothing here changes the model. Import it at the top of every tutorial:

    from tutorial_helpers import *

What it gives you:
- `repo_root`, with `src/` (the package `aircraft_closure`), the repo root (for `examples/`) and `tutorials/` on
  `sys.path`;
- `asb`, `np` (AeroSandbox's symbolic NumPy), `u` (AeroSandbox units) and `plt`;
- a plot style and colour names (`blue`, `orange`, `aqua`, `yellow`, `red`, `ink`, `ink_muted`);
- `check(name, passed)` and `check_summary()`: each tutorial verifies the claims it makes;
- `lb(kg)` and `scalar(x)`;
- `describe_opti(opti)`: size of an `asb.Opti` problem (variables, constraints) before or after a solve;
- `baseline()`: the full v3.6 reference design (about 10-15 minutes the first time, then cached), used only by
  the deep-dive tutorials.
"""
import contextlib
import hashlib
import pickle
import sys
import time
import warnings
from pathlib import Path

repo_root = next(p for p in [Path.cwd(), *Path.cwd().parents] if (p / "pyproject.toml").exists())
for _path in (repo_root / "src", repo_root, repo_root / "tutorials"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore", message=".*legacy numpy.*")

import aerosandbox as asb                     # noqa: E402
import aerosandbox.numpy as np                # noqa: E402
import aerosandbox.tools.units as u           # noqa: E402
import matplotlib.pyplot as plt               # noqa: E402

# ---- plot style (the palette of the discipline notebooks) -------------------------------------------------------
ink, ink_muted, grid_color = "#1a1a19", "#5e5d58", "#e4e3dc"
blue, orange, aqua, yellow, red = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#c8372d"
_style = {"axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "axes.facecolor": "white",
          "figure.facecolor": "white", "grid.color": grid_color, "grid.linewidth": 0.6, "axes.edgecolor": ink_muted,
          "axes.labelcolor": ink, "xtick.color": ink_muted, "ytick.color": ink_muted, "axes.titlesize": 11,
          "axes.labelsize": 10, "xtick.labelsize": 9, "ytick.labelsize": 9, "font.size": 10, "figure.dpi": 110,
          "legend.frameon": False, "legend.fontsize": 9}


def apply_style():
    """(Re)apply the tutorial plot style; AeroSandbox's own drawing functions change matplotlib's global style."""
    plt.rcParams.update(_style)


apply_style()

# ---- checks -----------------------------------------------------------------------------------------------------
check_log = []


def check(name, passed):
    """Record and print one named check, so each notebook verifies the claims it teaches."""
    passed = bool(passed)
    check_log.append((name, passed))
    print(("PASS  " if passed else "FAIL  ") + name)


def check_summary():
    failed = [name for name, ok in check_log if not ok]
    print(f"{len(check_log) - len(failed)} of {len(check_log)} checks passed"
          + ("" if not failed else ": FAILED " + "; ".join(failed)))


def scalar(x):
    """A float from a numeric result that may come back as a 0-d or 1-element array (AeroSandbox models)."""
    return float(np.asarray(x, dtype=float).reshape(-1)[0])


def lb(kg):
    """Kilograms to pounds (mass)."""
    return kg / u.lbm


# ---- looking at an Opti problem -----------------------------------------------------------------------------------
def describe_opti(opti):
    """Number of scalar decision variables, constraints and equality constraints in an `asb.Opti`."""
    # CasADi stores each constraint row with lower and upper bounds; equal bounds mean an equality. AeroSandbox
    # writes variable bounds (lower_bound=, upper_bound=) as constraint rows too, so they are counted here.
    import casadi as cas
    import numpy as onp
    lower = onp.array(cas.evalf(opti.lbg)).ravel()
    upper = onp.array(cas.evalf(opti.ubg)).ravel()
    return dict(variables=opti.nx, constraints=opti.ng, equalities=int(sum(lower == upper)),
                inequalities=int(sum(lower != upper)))


@contextlib.contextmanager
def quiet_solver():
    """Silence C-level stderr (CasADi's repeated "NaN detected" warnings from starts that fail) inside the block.

    The warnings are harmless (IPOPT backs off from the bad step) but can fill a notebook; results are unchanged.
    """
    import io
    import os
    # CasADi writes through Python's sys.stderr when run from Python; IPOPT's C code writes to file descriptor 2.
    sys.stderr.flush()
    saved_fd = os.dup(2)
    with open(os.devnull, "w") as devnull, contextlib.redirect_stderr(io.StringIO()):
        os.dup2(devnull.fileno(), 2)
        try:
            yield
        finally:
            os.dup2(saved_fd, 2)
            os.close(saved_fd)


# ---- the cached baseline (deep dives only) ----------------------------------------------------------------------
cache_dir = repo_root / "tutorials" / ".cache"


def source_fingerprint():
    """A hash of everything the sizing depends on (package, examples, data)."""
    digest = hashlib.sha256()
    for folder, pattern in ((repo_root / "src", "*.py"), (repo_root / "examples", "*.py"), (repo_root / "data", "*")):
        for path in sorted(folder.rglob(pattern)):
            if path.is_file():
                digest.update(str(path.relative_to(repo_root)).encode())
                digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def cached(name, compute):
    """`compute()` once per source version; later calls (and other notebooks) load the pickle."""
    cache_dir.mkdir(exist_ok=True)
    path = cache_dir / f"{name}-{source_fingerprint()}.pkl"
    if path.exists():
        with open(path, "rb") as file:
            return pickle.load(file)
    time_start_s = time.perf_counter()
    value = compute()
    print(f"[{name}: computed in {(time.perf_counter() - time_start_s) / 60:.1f} min and cached]")
    with open(path, "wb") as file:
        pickle.dump(value, file)
    return value


def baseline():
    """The v3.6 baseline design (`solve_halo_sizing()` with every default), solved once and cached."""
    from examples.halo_sizing import solve_halo_sizing
    return cached("baseline", solve_halo_sizing)


def baseline_assumptions():
    """The assumptions of the final (integer-unit) solve of the baseline: defaults with the unit counts fixed."""
    from dataclasses import replace
    from examples.halo_sizing import HaloAssumptions
    counts = {name: fixed or int(np.ceil(n - 1e-3)) for name, (n, fixed) in baseline().machine_units.items()}
    return replace(HaloAssumptions(), count_units_motor=counts["motor"], count_units_generator=counts["generator"])


# ---- Opti x-ray: look inside a solve that happens deep in library code --------------------------------------------
class OptiXray:
    """What `capture_opti()` saw: the last Opti solved, its solution, every `subject_to` call with the duals it
    returned, and selected local variables of the function that called `solve()`."""

    def __init__(self):
        self.opti = None
        self.solution = None
        self.calls = []            # (constraints passed, duals returned) per subject_to call, in order
        self.locals = {}
        self.time_s = None

    def dual_values(self, duals):
        return [float(self.solution.value(d)) for d in duals]


@contextlib.contextmanager
def capture_opti(keep_locals=("all_margins", "mass_takeoff_kg", "total", "design", "flown", "breakdown")):
    """Patch `asb.Opti.subject_to` and `asb.Opti.solve` for the duration of the block.

    The patch only records; it never changes a constraint or the solve. Use it to read the size of the problem,
    the number of iterations and the Lagrange multipliers of named constraints of a solve made by library code.
    """
    xray = OptiXray()
    subject_to, solve = asb.Opti.subject_to, asb.Opti.solve

    def recording_subject_to(self, constraint, _stacklevel=1):
        duals = subject_to(self, constraint, _stacklevel=_stacklevel + 1)
        xray.calls.append((constraint, duals, self))
        return duals

    def recording_solve(self, *args, **kwargs):
        caller = sys._getframe(1).f_locals
        time_start_s = time.perf_counter()
        solution = solve(self, *args, **kwargs)
        xray.time_s = time.perf_counter() - time_start_s
        xray.opti, xray.solution = self, solution
        xray.locals = {name: caller[name] for name in keep_locals if name in caller}
        return solution

    asb.Opti.subject_to, asb.Opti.solve = recording_subject_to, recording_solve
    try:
        yield xray
    finally:
        asb.Opti.subject_to, asb.Opti.solve = subject_to, solve
        # keep only the calls made on the Opti that was solved last
        xray.calls = [(c, d) for c, d, owner in xray.calls if owner is xray.opti]
