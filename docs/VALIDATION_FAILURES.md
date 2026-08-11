# Validation failure classification (issue #78)

This document defines the failure taxonomy a validation run is classified into, the named
off-track definitions, and how to read the evidence. The taxonomy is implemented once in
`src/game/validation_classifier.c` (the pure reducer) and proven by construction by the
`failure-classification` scenario in `tests/scenarios/gameplay_tests.c`. The thresholds are
defined once, in `validation_classification_inputs_default()`, and the validation runner
(`src/platform/main.c`), `ai-roster-laps`, and the by-construction scenario all call it — so
the values below are the exact numbers every classifier consumer quotes (PR #80 review).

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

- `reason` — the primary class (below), `"pass"`, or `"unclassified"` when no classifier ran
  (pre-run failures such as an invalid car/spec or an ffmpeg start error).
- `first_fault_tick` — the **earliest detected causal event**, never the final budget timeout
  unless nothing earlier happened. For `collision_stuck` it is the contact; for
  `spin_then_departure` it is the spin onset.
- `contributing` — the classes detected, in first-occurrence order (ties broken by the fixed
  severity order), truncated to the 8 earliest — a run that detects more classes keeps the
  causal head of the sequence.
- `last_checkpoint_index` / `expected_checkpoint_index` — the last gate crossed and the next
  one owed at the end; `-1` when no validation run happened.
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
| `wrong_way`, `route_departure`, `localization_lost`, `planner_localization_mismatch`, `unexplained` | `checkpoint_missed` (did not finish; the classification names why, or records that it cannot) |
| any class except `invalid_physics` | `video_encode_failed`, when the video pipe or write failed — the encode failure is selected ahead of every classifier branch except `invalid_physics`, because the run produced no usable video regardless of why it failed (PR #80 review) |

## Primary-selection rule

`first_fault_tick` is the earliest detected causal event. The primary reason is the class
attached to that tick, with two adjustments:

1. `invalid_physics` always wins when a non-finite pose/speed appears — no other verdict is
   meaningful once the state is non-finite.
2. `slow_timeout` only applies when the tick budget expired **and** the car was still making
   forward progress (furthest bin advanced within the last `progressRecencyS`, 2.0 s). A
   stopped car at budget expiry is a stall, not a slow timeout.
3. A run that did **not** complete its target laps and matched no class at all is
   `unexplained`, never `pass`. This is the catch-all, and the only class with no detector of
   its own: it is reachable precisely when every reducer above declined the run. Degenerate
   inputs (no rows, or no `inputs`) stay `pass` — there is no run to judge.

A run that completed its target laps is `pass`, even with transient
planner/localization disagreement — the fault classes classify failures. The one exception is
`invalid_physics`: a non-finite pose/speed is a failure no matter where it appeared, so a
completed run never erases it.

## The classes

All thresholds are from `ClassificationInputs` (the runner's values shown).

| class | detection | thresholds |
|---|---|---|
| `pass` | completed the target laps | — |
| `invalid_physics` | any non-finite `position_x_m` / `position_y_m` / `speed_mps` | — |
| `checkpoint_out_of_order` | an out-of-order crossing of a gate **behind** the owed one | `checkpointEvent == 2`; modular forward distance from the owed gate exceeds half a lap: `(crossed − owed) mod count > count/2` (a gate behind by less than half a lap — e.g. crossed 1 while owing 20 on 25 gates — is actually a *skip*, because the forward distance is 6; plain `crossed < owed` is not the rule). crossed is the event's own gate index (`checkpoint_crossed_index`; an out-of-order event always carries it, `-1` when there is no crossing) |
| `checkpoint_skipped` | an out-of-order crossing of a gate **ahead** of the owed one | `checkpointEvent == 2`; forward distance `(crossed − owed) mod count` in `(0, count/2]` — a gate up to half a lap ahead |
| `collision_stuck` | a contact (lockout rising edge, approach speed ≥ 0.1 m/s) followed by a committed stall | `collisionSpeedMpsEps = 0.1` |
| `spin_then_departure` | a sustained spin followed by a sustained route departure | spin: \|sideslip\| > 1.48 rad while speed > 2 m/s for ≥ 0.25 s; departure: beyond runoff ≥ 0.75 s |
| `stalled_on_track` | stopped ≥ 3 s on the racing surface | `stallSpeedMps = 0.5`, `stallDurationS = 3.0`, `wheelsOffAsphalt < 4`, not beyond runoff |
| `stalled_off_track` | stopped ≥ 3 s beyond the surface/runoff | same, `wheelsOffAsphalt >= 4` or beyond runoff |
| `wrong_way` | the latched wrong-way flag sustained | `wrongWayFlag == 1` ≥ 1.5 s |
| `route_departure` | beyond the runoff sustained, no preceding spin | `beyondRunoff == 1` ≥ 0.75 s |
| `localization_lost` | route segment invalid sustained | `routeSegmentIndex < 0` ≥ 0.75 s |
| `planner_localization_mismatch` | AI segment ≠ route segment sustained, AI runs only | `ai_present == 1` and `aiSegment != routeSegmentIndex` ≥ 1.0 s |
| `slow_timeout` | budget expired while still progressing | `ticksRun >= tickBudget`, progress within the last `progressRecencyS` (2.0 s), no stall/spin/departure class; progress is per-lap (the furthest bin resets at each lap close) and counts only full-bin advances |
| `unexplained` | did not finish, and no class above fits | no detector: selected when `rows[last].lapIndex < targetLaps` and nothing else was detected. `first_fault_tick` is anchored at `last_progress_tick` — there is no causal event, so the report points at the last moment the run was still going right |

A run classified `unexplained` is a **gap in the table above, not a diagnosis**. It exists so
that a failure mode nobody has anticipated reports as an unrecognised failure instead of
falling through to `pass`, which is what it did before (`awd_rally` on `technical`: still
rolling at ~2.6 m/s mean, no forward progress for the last ~225 s of its budget, so the stall
detector never fires and `slow_timeout`'s recency test rejects it — reported `FAIL` with reason
`pass`). The evidence fields carry enough state to identify what happened; when a mode recurs,
promote it to a class with a detector of its own and it will stop landing here.

## Named off-track definitions

There is no single "off track" boolean; the definitions below are reported side by side (as
telemetry columns and as classifier inputs) so a report is explicit about which one fired:

| definition | telemetry column | meaning |
|---|---|---|
| car centre on the racing surface | `on_route_flag` | `RouteLocation.onRoute`: distance to the closest centreline point within the segment's racing half-width |
| all wheels on the racing surface | `on_track` (Phase 5) | all four `surface_id`s are asphalt |
| N wheels off the racing surface | `wheels_off_asphalt` | count of the four wheels not on asphalt (0..4) |
| at or beyond the barrier | `beyond_runoff` | valid localization and `routeConfidence <= 0` (barrier stands at the runoff edge); an invalid localization is not a barrier crossing |
| leave distance | `route_departure_dist_m` | \|pose − closest centreline point\|; 0 when localization is invalid (no closest point) |

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
