# Validation failure classification (issue #78)

This document defines the failure taxonomy a validation run is classified into, the named
off-track definitions, and how to read the evidence. Everything here is implemented twice and
only twice: once in `src/game/validation_classifier.c` (the pure reducer) and once in the
scenario that proves it by construction (`failure-classification` in
`tests/scenarios/gameplay_tests.c`). The thresholds below are the exact values the validation
runner (`src/platform/main.c`) feeds the classifier.

## Why

A bare lap count cannot explain a failed run. The classifier turns one telemetry file into a
primary reason, the earliest causal tick, the contributing events, and where the run stopped
and what it owed. That is what makes a physics change look different from a checkpoint bug, a
slow AI from a stalled car, and a spin that became a departure from a plain departure.

## The classification block in run.json

Every validation run writes a `classification` block (run schema 1.1.0):

```json
"classification": {
  "reason": "spin_then_departure",
  "first_fault_tick": 1480,
  "contributing": ["spin_then_departure", "route_departure"],
  "last_checkpoint_index": 6,
  "expected_checkpoint_index": 7,
  "furthest_route_distance_m": 512.0,
  "time_since_progress_s": 1.4
}
```

- `reason` — the primary class (below), or `"pass"`.
- `first_fault_tick` — the **earliest detected causal event**, never the final budget timeout
  unless nothing earlier happened. For `collision_stuck` it is the contact; for
  `spin_then_departure` it is the spin onset.
- `contributing` — every class detected, in first-occurrence order.
- `last_checkpoint_index` / `expected_checkpoint_index` — the last gate crossed and the next
  one owed at the end.
- `furthest_route_distance_m` — the furthest lap-relative progress reached (from the 10 m
  diagnostic progress bins).
- `time_since_progress_s` — how long before run end the furthest bin last advanced; a large
  value is the signature of a car that stopped long before the budget expired.

The coarse `result.status` in the same file stays the closed-set verdict; the classification
block carries the fine reason. The mapping from `reason` to `result.status`:

| primary class | RunStatus |
|---|---|
| `invalid_physics` | `invalid_state` |
| `checkpoint_out_of_order`, `checkpoint_skipped` | `checkpoint_out_of_order` |
| `stalled_on_track`, `stalled_off_track`, `collision_stuck`, `spin_then_departure` | `stalled` |
| `slow_timeout` | `tick_budget_exceeded` |
| `wrong_way`, `route_departure`, `localization_lost`, `planner_localization_mismatch` | `checkpoint_missed` (did not finish; the classification names why) |

## Primary-selection rule

`first_fault_tick` is the earliest detected causal event. The primary reason is the class
attached to that tick, with two adjustments:

1. `invalid_physics` always wins when a non-finite pose/speed appears — no other verdict is
   meaningful once the state is non-finite.
2. `slow_timeout` only applies when the tick budget expired **and** the car was still making
   forward progress (furthest bin advanced within the last 2 s). A stopped car at budget
   expiry is a stall, not a slow timeout.

A run that completed its target laps is always `pass`, even with transient
planner/localization disagreement — the fault classes classify failures.

## The classes

All thresholds are from `ClassificationInputs` (the runner's values shown).

| class | detection | thresholds |
|---|---|---|
| `pass` | completed the target laps | — |
| `invalid_physics` | any non-finite `position_x_m` / `position_y_m` / `speed_mps` | — |
| `checkpoint_out_of_order` | an out-of-order crossing of a gate **behind** the owed one | `checkpointEvent == 2`, crossed < owed |
| `checkpoint_skipped` | an out-of-order crossing of a gate **ahead** of the owed one | `checkpointEvent == 2`, crossed > owed (≤ half lap forward) |
| `collision_stuck` | a contact (lockout rising edge, approach speed ≥ 0.1 m/s) followed by a committed stall | `collisionSpeedMpsEps = 0.1` |
| `spin_then_departure` | a sustained spin followed by a sustained route departure | spin: \|sideslip\| > 1.48 rad while speed > 2 m/s for ≥ 0.25 s; departure: beyond runoff ≥ 0.75 s |
| `stalled_on_track` | stopped ≥ 3 s on the racing surface | `stallSpeedMps = 0.5`, `stallDurationS = 3.0`, `wheelsOffAsphalt < 4`, not beyond runoff |
| `stalled_off_track` | stopped ≥ 3 s beyond the surface/runoff | same, `wheelsOffAsphalt >= 4` or beyond runoff |
| `wrong_way` | the latched wrong-way flag sustained | `wrongWayFlag == 1` ≥ 1.5 s |
| `route_departure` | beyond the runoff sustained, no preceding spin | `beyondRunoff == 1` ≥ 0.75 s |
| `localization_lost` | route segment invalid sustained | `routeSegmentIndex < 0` ≥ 0.75 s |
| `planner_localization_mismatch` | AI segment ≠ route segment sustained | `aiSegment != routeSegmentIndex` ≥ 1.0 s |
| `slow_timeout` | budget expired while still progressing | `ticksRun >= tickBudget`, progress within last 2 s, no stall/spin/departure class |

## Named off-track definitions

There is no single "off track" boolean; the definitions below are reported side by side (as
telemetry columns and as classifier inputs) so a report is explicit about which one fired:

| definition | telemetry column | meaning |
|---|---|---|
| car centre on the racing surface | `on_route_flag` | `RouteLocation.onRoute`: distance to the closest centreline point within the segment's racing half-width |
| all wheels on the racing surface | `on_track` (Phase 5) | all four `surface_id`s are asphalt |
| N wheels off the racing surface | `wheels_off_asphalt` | count of the four wheels not on asphalt (0..4) |
| at or beyond the barrier | `beyond_runoff` | `routeConfidence <= 0` (barrier stands at the runoff edge) |
| leave distance | `route_departure_dist_m` | \|pose − closest centreline point\| |

`ai-roster-laps`, the telemetry CSV, run.json, and the failure bundle all consume these same
definitions; nothing recomputes a private "off track" from a different signal.

## Progress bins

Non-scoring bins derived from `RouteLocation.longitudinalM` at a **10 m** interval
(`AI_PROGRESS_BIN_M`). They record the current lap-relative bin, the furthest bin reached in
the current lap, and the tick it last advanced. They are **diagnostic only**: they never mutate
`nextCheckpoint`, lap validity, sector timing, classification, replay authority, or the rolling
checksum (`docs/SIMULATION_OWNERSHIP.md` classifies them as recomputed diagnostics, excluded
from `hash_entrant()`). The classifier uses them to tell a slow-but-moving car
(`slow_timeout`) from a stationary one (`stalled_*`).

## Replay parity

The classifier is a pure function of the telemetry rows; the failure bundle carries the replay
and the telemetry CSV, so replaying a captured failure reproduces the same first-fault tick,
classification, and checksum deterministically.
