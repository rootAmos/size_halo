"""Run manifests: one JSON file per solve with the code version, every input it read (with its certainty), the
outputs, and the sensitivity matrix when computed. The ledger reads evidence from these."""
import datetime
import json
import subprocess
from dataclasses import asdict
from pathlib import Path

import numpy as np


def git_state(directory="."):
    """(commit SHA, True when tracked files have uncommitted changes); (None, None) outside git."""
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=directory, capture_output=True, text=True,
                             check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=directory,
                                    capture_output=True, text=True, check=True).stdout.strip())
        return sha, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return [_jsonable(v) for v in value.tolist()]
    if isinstance(value, (np.floating, np.integer)):
        value = value.item()
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def build_manifest(source, inputs, outputs, sensitivity=None, note="", now=None):
    """`inputs`: list of UncertainInput; `outputs`: {key: value}; `sensitivity`: optional dict with "mode"
    ("resized" or "fixed"), "outputs" and "inputs" (keys) and "jacobian" (outputs x inputs)."""
    sha, dirty = git_state()
    now = now or datetime.datetime.now().replace(microsecond=0)
    return _jsonable(dict(
        source=source, time=now.isoformat(), commit=sha, dirty=dirty, note=note,
        inputs=[asdict(i) for i in inputs], outputs=dict(outputs), sensitivity=sensitivity))


def write_manifest(directory, manifest):
    """Write `manifest` as <directory>/<time>_<source>.json and return the path."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = manifest["time"].replace(":", "").replace("-", "")
    slug = "".join(c if c.isalnum() else "_" for c in manifest["source"]).strip("_")[:60]
    path = directory / f"{stamp}_{slug}.json"
    path.write_text(json.dumps(manifest, indent=1, ensure_ascii=False), encoding="utf-8")
    return path


def read_manifest(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))
