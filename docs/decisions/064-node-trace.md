# Plan 064: per-node mission trace

## Goal and scope

Store the state of every mission node in the sizing result (`HaloSizingResult.node_trace`), so mission plots
draw what the sizing solved instead of rebuilding it: the author asked for "everything per node, flight dynamics,
powertrain data, everything" and a profile of altitude, speed, range, fuel and flight path angle with angle of
attack and pitch.

In: one row per node (per segment point without plan 060 nodes) with time, flight path, attitude, aero, rotor,
powertrain powers, fuel flow, mass, battery and temperatures. Output only: no model, constraint or default changes.
Out: per-node altitude in climb and descent (every node of a segment flies the segment's mean altitude; changing
that is a model change), temperature sub-stepping, the battery heat sign.

## References

- Plan 060 (piecewise nodes, battery sub-steps), whose `battery_trace` this extends.
- Plan 063 (single-engine descent): the running-engine count per node.

## Model

`node_trace(segments, count_rotors, count_turbogenerators, value)` reads each node's `FlightPoint`:

- time: a segment's nodes ("label i/n", n + 1 of them) are instants evenly spaced over its duration; quasi-steady
  sub-points are intervals and take their start times;
- flight path angle asin(climb rate / airspeed) (0 in hover); angle of attack from the trim; pitch attitude =
  angle of attack + flight path angle in airplane mode (None in hover, where the model has no attitude);
- CL, CD; mass at the node, weight, fuel flow (running engines);
- thrust per rotor, rotor speed; shaft power of the rotors, motors and running turboshafts; motor electrical power;
- battery power, hybridization, SOC, voltage and current; machine temperatures.

## Verification

- Unit (`tests/mission/test_node_trace.py`, stand-in segments, no solve): node times across segments, flight path
  and pitch in climb and descent, hover without attitude, powers over the running units (single-engine descent).
- Sizing: v3.11 with the battery stepped, 4 nodes per segment, one engine in the descent; the trace's turbine power
  at the segment starts equals the result's `segments[*].power_turboshafts_W`.

## Assumptions

- Climb and descent nodes report the segment's mean altitude (the model's); a plot of altitude against time uses
  the prescribed climb and descent rates.

## Progress log

- 2026-10-12: `node_trace`, unit tests (3).
