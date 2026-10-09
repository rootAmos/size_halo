"""Shared helpers for the tutorials: paths, plot style, checks, a cached baseline solve and an Opti "x-ray".

Nothing here changes the model. The baseline cache only saves the ~10 minute solve between notebooks; it is keyed
on the source code, so any change to `src/`, `examples/` or `data/` triggers a fresh solve.
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

check_log = []


def check(name, passed):
    """Record and print one named check, so each notebook verifies the claims it teaches."""
    passed = bool(passed)
    check_log.append((name, passed))
    print(("PASS  " if passed else "FAIL  ") + name)


def scalar(x):
    """A number from a numeric result that may come back as a 0-d or 1-element array (AeroSandbox models)."""
    return float(np.asarray(x, dtype=float).reshape(-1)[0])


def show_source(target, start=None, end=None):
    """Print source with the file's real line numbers, straight from the repo (so it can never drift from the code).

    `target`: a function, class or module object, or a path relative to the repo root
    ("src/aircraft_closure/core/margins.py"). `start`/`end`: 1-based file line range to print (inclusive); by default
    the whole object (or file).
    """
    import inspect
    if isinstance(target, str):
        path = repo_root / target
        lines = path.read_text().splitlines()
        first = 1
    else:
        path = Path(inspect.getsourcefile(target))
        lines, first = inspect.getsourcelines(target)
        lines = [line.rstrip("\n") for line in lines]
        first = max(first, 1)
        if start is None and end is None:
            start, end = first, first + len(lines) - 1
        lines = path.read_text().splitlines()
        first = 1
    start = start or first
    end = end or len(lines)
    print(f"# {path.relative_to(repo_root)}  lines {start}-{end}")
    for number in range(start, end + 1):
        print(f"{number:4d}  {lines[number - 1]}")


def lb(kg):
    """Kilograms to pounds (mass)."""
    return kg / u.lbm


def check_summary():
    failed = [name for name, ok in check_log if not ok]
    print(f"{len(check_log) - len(failed)} of {len(check_log)} checks passed"
          + ("" if not failed else ": FAILED " + "; ".join(failed)))


# ---- the cached baseline ----------------------------------------------------------------------------------------
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
