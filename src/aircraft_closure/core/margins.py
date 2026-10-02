"""Normalized margins: >= 0 is compatible, 0.1 means 10 % headroom.

Margins report; they never clip, resize or solve. Callers either constrain
`margin.value >= 0` in their own `asb.Opti` or report margins after a solve.
"""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Margin:
    label: str
    value: Any


@dataclass(frozen=True)
class MarginReportEntry:
    label: str
    value: Any
    compatible: bool


def margin_below(label, value, limit):
    """Margin for `value <= limit`, normalized by the (positive) limit."""
    return Margin(label, (limit - value) / limit)


def margin_above(label, value, limit):
    """Margin for `value >= limit`, normalized by the (positive) limit."""
    return Margin(label, (value - limit) / limit)


def margin_report(margins, value_of=lambda value: value):
    """Numeric margins sorted from most to least critical.

    `value_of` maps a margin expression to a number, e.g. `solution.value`
    after an Opti solve. Symbolic values without `value_of` are rejected by
    the comparison, never silently coerced.
    """
    entries = []
    for margin in margins:
        value = value_of(margin.value)
        entries.append(MarginReportEntry(margin.label, value, bool(value >= 0)))
    return tuple(sorted(entries, key=lambda entry: entry.value))
