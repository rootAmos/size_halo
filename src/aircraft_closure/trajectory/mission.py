"""A mission flown as linked trajectory segments in one `asb.Opti` (plan 069).

Orchestration: `solve_linked_mission` builds each segment with `build_tiltrotor_trajectory` on the same `asb.Opti`,
links the state at every join (altitude, airspeed, flight-path angle, nacelle angle; SOC and mass through
`initial_from`), sets the boundary conditions and one objective, solves and returns numbers. The multi-phase
formulation of Betts (2010, ch. 4.7): phases joined by linkage constraints in one NLP.

Each segment may fly its own model (a hot hover day and a standard cruise day, a degraded powertrain), provided
the battery capacity is the same. The battery is quasi-steady at the joins (no RC state carried).
"""
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

import aerosandbox as asb
import aerosandbox.numpy as np

from aircraft_closure.trajectory.segments import (TransitionClimbResult, _hover_estimates, _result, _smoothing,
                                                  _trimmed)
from aircraft_closure.trajectory.tiltrotor import TrajectoryGuess, TrajectoryLimits, build_tiltrotor_trajectory


@dataclass(frozen=True)
class SegmentEnd:
    """Boundary conditions at one end of a segment; None leaves a quantity free."""
    altitude_m: Any = None
    velocity_m_s: Any = None
    gamma_deg: Any = None
    tilt_deg: Any = None
    trimmed: bool = False            # zero acceleration and flight-path rate


@dataclass(frozen=True)
class MissionSegment:
    """One collocated segment of a linked mission."""
    name: str
    guess: TrajectoryGuess
    count_nodes: int = 5
    duration_bounds_s: tuple = (1.0, 7200.0)      # equal bounds fix the duration
    distance_m: Any = None                        # horizontal distance flown in the segment; None: free
    end: SegmentEnd = field(default_factory=SegmentEnd)
    altitude_min_m: Any = None                    # floor at every node; None: none
    model: Any = None                             # None: the mission's model
    limits: TrajectoryLimits = field(default_factory=TrajectoryLimits)
    corridor: Any = None
    configure: Optional[Callable] = None          # configure(opti, trajectory): extra constraints
    counted: bool = True                          # in the mission objective


@dataclass(frozen=True)
class LinkedMissionResult:
    """Per-segment numbers (in segment order) and the totals of the counted segments."""
    objective: str
    names: tuple
    segments: tuple                               # TransitionClimbResult per segment
    counted: tuple
    energy_electric_J: float                      # counted segments' bus energy
    energy_battery_chemical_J: float              # counted segments' chemical energy
    duration_s: float                             # counted segments
    distance_m: float                             # counted segments

    def segment(self, name) -> TransitionClimbResult:
        return self.segments[self.names.index(name)]


def _boundary(trajectory, index, end):
    t, out = trajectory, []
    if end.altitude_m is not None:
        out.append(t.altitude_m[index] == end.altitude_m)
    if end.velocity_m_s is not None:
        out.append(t.velocity_m_s[index] == end.velocity_m_s)
    if end.gamma_deg is not None:
        out.append(t.gamma_rad[index] == np.radians(end.gamma_deg))
    if end.tilt_deg is not None:
        out.append(t.tilt_deg[index] == end.tilt_deg)
    return out


def solve_linked_mission(model, segments, *, mass_kg, soc_initial, start=SegmentEnd(), objective="energy",
                         configure=None, verbose=False, max_iter=3000) -> LinkedMissionResult:
    """Fly `segments` (a sequence of `MissionSegment`) as one trajectory from `start` (boundary conditions of the
    first node) with `mass_kg` and `soc_initial`; least counted bus energy ("energy") or time ("time").

    `configure(opti, trajectories)` (trajectories in segment order) may add mission-level constraints that span
    segments, e.g. a total ground distance."""
    if objective not in ("energy", "time"):
        raise ValueError(f"Unknown objective '{objective}'.")
    if len({s.name for s in segments}) != len(segments):
        raise ValueError("Segment names must be unique.")
    opti = asb.Opti()
    built, previous = [], None
    for s in segments:
        m = s.model if s.model is not None else model
        t = build_tiltrotor_trajectory(opti, m, mass_initial_kg=mass_kg, soc_initial=soc_initial,
                                       count_nodes=s.count_nodes, duration_bounds_s=s.duration_bounds_s,
                                       guess=s.guess, limits=s.limits, corridor=s.corridor, initial_from=previous)
        if previous is None:
            opti.subject_to(_boundary(t, 0, start))
            if start.trimmed:
                _trimmed(opti, t, 0)
        else:
            opti.subject_to([t.altitude_m[0] == previous.altitude_m[-1],
                             t.velocity_m_s[0] == previous.velocity_m_s[-1],
                             t.gamma_rad[0] == previous.gamma_rad[-1],
                             t.tilt_deg[0] == previous.tilt_deg[-1]])
        opti.subject_to(_boundary(t, -1, s.end))
        if s.end.trimmed:
            _trimmed(opti, t, -1)
        if s.distance_m is not None:
            opti.subject_to(t.x_m[-1] == s.distance_m)
        if s.altitude_min_m is not None:
            opti.subject_to(t.altitude_m >= s.altitude_min_m)
        if s.configure is not None:
            s.configure(opti, t)
        built.append((s, m, t))
        previous = t

    if configure is not None:
        configure(opti, tuple(t for _, _, t in built))
    counted = [(s, t) for s, _, t in built if s.counted]
    smoothing = sum(_smoothing(t) for _, _, t in built)
    if objective == "energy":
        # Scale: momentum hover power over the guessed counted duration (plan 068's estimate), so the objective is O(1).
        scale_J = _hover_estimates(model, mass_kg, 0.0)[2] * sum(float(s.guess.duration_s) for s, _ in counted)
        opti.minimize(sum(t.energy_bus_J[-1] for _, t in counted) / scale_J + 1e-4 * smoothing)
    else:
        scale_s = sum(float(s.guess.duration_s) for s, _ in counted)
        opti.minimize(sum(t.duration_s for _, t in counted) / scale_s + 1e-4 * smoothing)
    solution = opti.solve(verbose=verbose, max_iter=max_iter)
    results = tuple(_result(objective, solution, m, t) for _, m, t in built)
    flags = tuple(s.counted for s, _, _ in built)
    pick = [r for r, c in zip(results, flags) if c]
    return LinkedMissionResult(
        objective=objective, names=tuple(s.name for s, _, _ in built), segments=results, counted=flags,
        energy_electric_J=float(sum(r.energy_electric_J for r in pick)),
        energy_battery_chemical_J=float(sum(r.energy_battery_chemical_J for r in pick)),
        duration_s=float(sum(r.duration_s for r in pick)), distance_m=float(sum(r.distance_m for r in pick)))
