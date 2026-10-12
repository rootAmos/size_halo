"""The input registry: every field of the dataclasses a solve reads, with certainty overlaid from a JSON file.

`inputs_from_dataclass` lists every field (nothing is silently left out); `apply_overrides` sets kind, spread,
source and owner from a mapping {key: {field: value}} that owners edit (e.g. `ledger/inputs.json`).
"""
import json
from dataclasses import fields, is_dataclass
from pathlib import Path

from aircraft_closure.uncertainty.model import UncertainInput


def _plain(value):
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if is_dataclass(value):
        return {f.name: _plain(getattr(value, f.name)) for f in fields(value)}
    return repr(value)


def inputs_from_dataclass(instance, prefix, default_kind="setting"):
    """One `UncertainInput` per field of `instance`, keyed "prefix.field"; numbers default to `default_kind`."""
    result = []
    for f in fields(instance):
        value = _plain(getattr(instance, f.name))
        numeric = isinstance(value, (int, float)) and not isinstance(value, bool)
        result.append(UncertainInput(key=f"{prefix}.{f.name}", value=value,
                                     kind=default_kind if numeric else "setting"))
    return result


def apply_overrides(inputs, overrides):
    """Set certainty fields from {key: {field: value}}; unknown keys raise (a typo must not pass as certainty)."""
    by_key = {i.key: i for i in inputs}
    unknown = sorted(set(overrides) - set(by_key))
    if unknown:
        raise KeyError(f"overrides for inputs the solve does not read: {unknown}")
    for key, values in overrides.items():
        for name, value in values.items():
            if name in ("key", "value"):
                raise ValueError(f"{key}: the value comes from the solve, not the overrides")
            setattr(by_key[key], name, value)
        by_key[key].__post_init__()
    return inputs


def load_overrides(path):
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
