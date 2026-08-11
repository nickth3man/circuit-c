/*
 * validation_classifier.c — the pure reducer declared in validation_classifier.h.
 *
 * One forward pass over the telemetry rows classifies a validation run into a primary failure
 * reason plus contributing events, and records where the run stopped and what it owed. Every
 * threshold comes from ClassificationInputs so the test and the report quote the same numbers,
 * and the interval detectors mirror validation_metrics.c (they track the wall-clock start of a
 * qualifying interval via row.timeS, so the result never depends on the telemetry sample rate).
 *
 * The selection rule for `primary` is in the header: INVALID_PHYSICS always wins; otherwise the
 * class attached to the earliest causal event wins, except SLOW_TIMEOUT which applies only when
 * the budget expired and the car was still progressing (a stopped car at budget expiry is a
 * stall, not a slow timeout).
 */
#include "game/validation_classifier.h"

#include <math.h>
#include <string.h>

/* Per-class detection state held across the single pass. `tick`/`timeS` record the FIRST
 * occurrence once the class is committed; until then the interval is pending in onset*. */
typedef struct {
    bool detected;
    uint64_t tick;
    double timeS;
} ClassHit;

typedef struct {
    bool active;
    uint64_t onsetTick;
    double onsetTimeS;
} Interval;

static void record_hit(ClassHit *h, uint64_t tick, double timeS)
{
    if (h->detected) return;
    h->detected = true;
    h->tick = tick;
    h->timeS = timeS;
}

static void commit_if_sustained(ClassHit *h, Interval *iv, const TelemetryRow *row,
                                double holdS)
{
    /* Called when the condition just went false after being active, or from the end-of-run
     * close with the row that ended the run. Commits the class if the interval met the hold. */
    if (iv->active && row->timeS - iv->onsetTimeS >= holdS) {
        record_hit(h, iv->onsetTick, iv->onsetTimeS);
    }
    iv->active = false;
}

static bool finite_row(const TelemetryRow *r)
{
    return isfinite((double)r->positionXM) && isfinite((double)r->positionYM) &&
           isfinite((double)r->speedMps);
}

const char *failure_class_reason(FailureClass c)
{
    switch (c) {
        case RUN_CLASS_PASS: return "pass";
        case RUN_CLASS_INVALID_PHYSICS: return "invalid_physics";
        case RUN_CLASS_CHECKPOINT_OUT_OF_ORDER: return "checkpoint_out_of_order";
        case RUN_CLASS_CHECKPOINT_SKIPPED: return "checkpoint_skipped";
        case RUN_CLASS_COLLISION_STUCK: return "collision_stuck";
        case RUN_CLASS_SPIN_THEN_DEPARTURE: return "spin_then_departure";
        case RUN_CLASS_STALLED_OFF_TRACK: return "stalled_off_track";
        case RUN_CLASS_STALLED_ON_TRACK: return "stalled_on_track";
        case RUN_CLASS_WRONG_WAY: return "wrong_way";
        case RUN_CLASS_ROUTE_DEPARTURE: return "route_departure";
        case RUN_CLASS_LOCALIZATION_LOST: return "localization_lost";
        case RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH: return "planner_localization_mismatch";
        case RUN_CLASS_SLOW_TIMEOUT: return "slow_timeout";
    }
    return "unknown";
}

const char *failure_class_label(FailureClass c)
{
    switch (c) {
        case RUN_CLASS_PASS: return "completed cleanly";
        case RUN_CLASS_INVALID_PHYSICS: return "non-finite simulation state";
        case RUN_CLASS_CHECKPOINT_OUT_OF_ORDER: return "gate crossed out of order";
        case RUN_CLASS_CHECKPOINT_SKIPPED: return "a required gate was skipped";
        case RUN_CLASS_COLLISION_STUCK: return "collision then stall";
        case RUN_CLASS_SPIN_THEN_DEPARTURE: return "spin followed by route departure";
        case RUN_CLASS_STALLED_OFF_TRACK: return "stopped beyond the racing surface";
        case RUN_CLASS_STALLED_ON_TRACK: return "stopped on the racing surface";
        case RUN_CLASS_WRONG_WAY: return "sustained travel against the route direction";
        case RUN_CLASS_ROUTE_DEPARTURE: return "left the route beyond the runoff";
        case RUN_CLASS_LOCALIZATION_LOST: return "route localization lost";
        case RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH:
            return "planner disagreed with route localization";
        case RUN_CLASS_SLOW_TIMEOUT: return "tick budget expired while still progressing";
    }
    return "unknown";
}

void validation_classify(const TelemetryRow *rows, int count, const ValidationMetrics *metrics,
                         const ClassificationInputs *in, ValidationClassification *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    out->primary = RUN_CLASS_PASS;
    out->lastCheckpointIndex = -1;
    out->expectedCheckpointIndex = -1;
    if (rows == NULL || count <= 0 || in == NULL) return;

    ClassHit invalidPhysics = { 0 };
    ClassHit outOfOrder = { 0 };
    ClassHit skipped = { 0 };
    ClassHit collisionStuck = { 0 };
    ClassHit spinThenDeparture = { 0 };
    ClassHit stalledOn = { 0 };
    ClassHit stalledOff = { 0 };
    ClassHit wrongWayHit = { 0 };
    ClassHit routeDeparture = { 0 };
    ClassHit localizationLost = { 0 };
    ClassHit plannerMismatch = { 0 };
    ClassHit slowTimeout = { 0 };

    /* Sustained-condition intervals. */
    Interval spinIv = { 0 };     /* spin attitude sustained */
    Interval stallIv = { 0 };    /* stopped sustained */
    Interval wrongWayIv = { 0 }; /* wrong-way flag sustained */
    Interval departIv = { 0 };   /* beyond runoff sustained */
    Interval locLostIv = { 0 };  /* localization invalid sustained */
    Interval mismatchIv = { 0 }; /* AI/route segment disagreement sustained */

    bool hadSpin = false;       /* any spin committed this run (for spin_then_departure) */
    uint64_t spinOnsetTick = 0; /* onset of the first qualifying spin */
    bool firstCollisionSeen = false;
    uint64_t firstCollisionTick = 0;
    double firstCollisionTimeS = 0.0;
    float prevLockout = 0.0f;
    bool stallOnsetOffTrack = false; /* off-track state at the current stall interval's onset */
    /* Forward-progress bookkeeping. furthestProgressM is PER-LAP (the progress stage resets it
     * at every lap close), so the run-wide maximum must reset when the rows move into a new
     * lap; otherwise a slow-but-moving car in lap 2+ can never beat lap 1's max and stops
     * looking like it is making progress (PR #80 review). meaningfulProgressM is the bin
     * advance that counts: sub-bin creep is not progress. */
    int progressLap = -1;
    double furthestProgressM = -1.0;
    uint64_t lastProgressTick = 0;
    double lastProgressTimeS = 0.0;

    for (int i = 0; i < count; i++) {
        const TelemetryRow *r = &rows[i];
        const double t = r->timeS;
        const uint64_t tick = r->tick;

        if (!finite_row(r)) {
            record_hit(&invalidPhysics, tick, t);
            /* Non-finite state ends meaningful classification; stop scanning. */
            break;
        }

        /* Forward progress bookkeeping, reset per lap and gated by the meaningful-progress
         * threshold (e.g. one 10 m bin). */
        if (r->lapIndex != progressLap) {
            progressLap = r->lapIndex;
            furthestProgressM = -1.0;
        }
        if ((double)r->furthestProgressM >= furthestProgressM + in->meaningfulProgressM) {
            furthestProgressM = (double)r->furthestProgressM;
            lastProgressTick = tick;
            lastProgressTimeS = t;
        }

        /* Checkpoint events. checkpointEvent: 0 none, 1 in-order, 2 out-of-order, 3 lap-complete.
         * An out-of-order crossing's own gate index travels in checkpointCrossedIndex (an
         * out-of-order crossing never advances lastCrossedIndex, which still names the previous
         * legal gate); the hand-built test rows predate that column, so fall back to
         * lastCrossedIndex when it is absent. checkpointIndex still names the gate owed;
         * crossing a gate AHEAD of the owed one is a forward skip (checkpoint_skipped),
         * crossing one BEHIND or off-sequence is checkpoint_out_of_order. */
        if (r->checkpointEvent == 2) {
            const int crossed = (r->checkpointCrossedIndex >= 0) ? r->checkpointCrossedIndex
                                                                 : r->lastCrossedIndex;
            const int owed = r->checkpointIndex;
            if (in->checkpointCount > 1 && crossed >= 0 && owed >= 0) {
                const int forward =
                    ((crossed - owed) % in->checkpointCount + in->checkpointCount) %
                    in->checkpointCount;
                if (forward > 0 && forward <= in->checkpointCount / 2) {
                    record_hit(&skipped, tick, t);
                } else {
                    record_hit(&outOfOrder, tick, t);
                }
            } else {
                record_hit(&outOfOrder, tick, t);
            }
        }

        /* Collisions: rising edge of the crash lockout, above the contact-speed floor. */
        if (i > 0 && r->crashLockoutS > 0.0f && prevLockout <= 0.0f &&
            (double)r->speedMps >= in->collisionSpeedMpsEps) {
            if (!firstCollisionSeen) {
                firstCollisionSeen = true;
                firstCollisionTick = tick;
                firstCollisionTimeS = t;
            }
        }
        prevLockout = r->crashLockoutS;

        /* Spin attitude (sustained). */
        {
            const bool cond = (double)fabsf(r->bodySideslipRad) > in->spinSideslipRad &&
                              (double)r->speedMps > in->spinMinSpeedMps;
            if (cond && !spinIv.active) {
                spinIv.active = true;
                spinIv.onsetTick = tick;
                spinIv.onsetTimeS = t;
            } else if (!cond && spinIv.active) {
                if (t - spinIv.onsetTimeS >= in->spinMinDurationS) {
                    hadSpin = true;
                    if (spinOnsetTick == 0) {
                        spinOnsetTick = spinIv.onsetTick;
                    }
                }
                spinIv.active = false;
            }
        }

        /* Stopped (sustained), split by the surface that held AT THE ONSET. The release row's
         * surface is irrelevant: a car that stops on asphalt and is pushed off while stopped
         * still stalled on track (PR #80 review). */
        {
            const bool stopped = (double)r->speedMps <= in->stallSpeedMps;
            if (stopped && !stallIv.active) {
                stallIv.active = true;
                stallIv.onsetTick = tick;
                stallIv.onsetTimeS = t;
                stallOnsetOffTrack = (r->beyondRunoff == 1 || r->wheelsOffAsphalt >= 4);
            } else if (!stopped && stallIv.active) {
                Interval ivCopy = stallIv;
                commit_if_sustained(stallOnsetOffTrack ? &stalledOff : &stalledOn, &ivCopy, r,
                                    in->stallDurationS);
                stallIv.active = false;
            }
        }

        /* Wrong-way (sustained). */
        {
            const bool cond = (r->wrongWayFlag == 1);
            if (cond && !wrongWayIv.active) {
                wrongWayIv.active = true;
                wrongWayIv.onsetTick = tick;
                wrongWayIv.onsetTimeS = t;
            } else if (!cond && wrongWayIv.active) {
                Interval ivCopy = wrongWayIv;
                commit_if_sustained(&wrongWayHit, &ivCopy, r, in->wrongWayHoldS);
                wrongWayIv.active = false;
            }
        }

        /* Route departure: beyond runoff sustained. (Spin coupling handled after the pass.) */
        {
            const bool cond = (r->beyondRunoff == 1);
            if (cond && !departIv.active) {
                departIv.active = true;
                departIv.onsetTick = tick;
                departIv.onsetTimeS = t;
            } else if (!cond && departIv.active) {
                Interval ivCopy = departIv;
                commit_if_sustained(&routeDeparture, &ivCopy, r, in->departureHoldS);
                departIv.active = false;
            }
        }

        /* Localization lost: route segment invalid sustained. */
        {
            const bool cond = (r->routeSegmentIndex < 0);
            if (cond && !locLostIv.active) {
                locLostIv.active = true;
                locLostIv.onsetTick = tick;
                locLostIv.onsetTimeS = t;
            } else if (!cond && locLostIv.active) {
                Interval ivCopy = locLostIv;
                commit_if_sustained(&localizationLost, &ivCopy, r, in->departureHoldS);
                locLostIv.active = false;
            }
        }

        /* Planner/localization mismatch: AI segment != route segment sustained (AI runs only). */
        if (r->routeSegmentIndex >= 0) {
            const bool cond = (r->aiSegment != r->routeSegmentIndex);
            if (cond && !mismatchIv.active) {
                mismatchIv.active = true;
                mismatchIv.onsetTick = tick;
                mismatchIv.onsetTimeS = t;
            } else if (!cond && mismatchIv.active) {
                Interval ivCopy = mismatchIv;
                commit_if_sustained(&plannerMismatch, &ivCopy, r, in->mismatchHoldS);
                mismatchIv.active = false;
            }
        }
    }

    /* Close any interval still open at the end of the run, using the last row as the reference. */
    if (count > 0) {
        const TelemetryRow *last = &rows[count - 1];
        if (spinIv.active && last->timeS - spinIv.onsetTimeS >= in->spinMinDurationS) {
            hadSpin = true;
            if (spinOnsetTick == 0) spinOnsetTick = spinIv.onsetTick;
        }
        if (stallIv.active) {
            Interval ivCopy = stallIv;
            commit_if_sustained(stallOnsetOffTrack ? &stalledOff : &stalledOn, &ivCopy, last,
                                in->stallDurationS);
        }
        if (wrongWayIv.active) {
            Interval ivCopy = wrongWayIv;
            commit_if_sustained(&wrongWayHit, &ivCopy, last, in->wrongWayHoldS);
        }
        if (departIv.active) {
            Interval ivCopy = departIv;
            commit_if_sustained(&routeDeparture, &ivCopy, last, in->departureHoldS);
        }
        if (locLostIv.active) {
            Interval ivCopy = locLostIv;
            commit_if_sustained(&localizationLost, &ivCopy, last, in->departureHoldS);
        }
        if (mismatchIv.active) {
            Interval ivCopy = mismatchIv;
            commit_if_sustained(&plannerMismatch, &ivCopy, last, in->mismatchHoldS);
        }
    }

    /* Spin-then-departure: a committed departure PRECEDED BY a qualifying spin this run. The
     * contributing event carries the SPIN onset (the earliest causal tick), which is what makes
     * a spin-that-became-a-departure distinguishable from a plain departure. A departure that
     * happened before any spin is not this class (PR #80 review). */
    if (hadSpin && routeDeparture.detected && spinOnsetTick < routeDeparture.tick) {
        record_hit(&spinThenDeparture, spinOnsetTick,
                   (spinOnsetTick > 0 && in->fixedDtS > 0.0)
                       ? (double)spinOnsetTick * in->fixedDtS
                       : routeDeparture.timeS);
    }

    /* Collision-stuck: a committed stall whose onset is AFTER a contact this run. A stall that
     * began before the collision was not caused by it (PR #80 review). */
    {
        bool haveStallTick = false;
        uint64_t stallTick = 0;
        if (stalledOn.detected) {
            haveStallTick = true;
            stallTick = stalledOn.tick;
        }
        if (stalledOff.detected && (!haveStallTick || stalledOff.tick < stallTick)) {
            haveStallTick = true;
            stallTick = stalledOff.tick;
        }
        if (firstCollisionSeen && haveStallTick && firstCollisionTick < stallTick) {
            record_hit(&collisionStuck, firstCollisionTick, firstCollisionTimeS);
        }
    }

    /* Slow timeout: budget expired while still progressing, with no stall/spin/departure/wrong-way
     * /localization class having fired. A car that stopped at budget expiry is a stall, not this. */
    if (in->ticksRun >= in->tickBudget &&
        !(stalledOn.detected || stalledOff.detected || hadSpin || routeDeparture.detected ||
          wrongWayHit.detected || localizationLost.detected)) {
        const double runEndS = (count > 0) ? rows[count - 1].timeS : 0.0;
        const double since = runEndS - lastProgressTimeS;
        if (since < 2.0) {
            record_hit(&slowTimeout, (uint64_t)in->ticksRun, runEndS);
        }
    }

    /* Assemble contributing in first-occurrence order and pick the primary. */
    struct Detected {
        FailureClass cls;
        const ClassHit *hit;
    } hits[] = {
        { RUN_CLASS_INVALID_PHYSICS, &invalidPhysics },
        { RUN_CLASS_CHECKPOINT_OUT_OF_ORDER, &outOfOrder },
        { RUN_CLASS_CHECKPOINT_SKIPPED, &skipped },
        { RUN_CLASS_COLLISION_STUCK, &collisionStuck },
        { RUN_CLASS_SPIN_THEN_DEPARTURE, &spinThenDeparture },
        { RUN_CLASS_STALLED_OFF_TRACK, &stalledOff },
        { RUN_CLASS_STALLED_ON_TRACK, &stalledOn },
        { RUN_CLASS_WRONG_WAY, &wrongWayHit },
        { RUN_CLASS_ROUTE_DEPARTURE, &routeDeparture },
        { RUN_CLASS_LOCALIZATION_LOST, &localizationLost },
        { RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH, &plannerMismatch },
        { RUN_CLASS_SLOW_TIMEOUT, &slowTimeout },
    };
    const int nhits = (int)(sizeof(hits) / sizeof(hits[0]));

    /* Collect the detected classes, then append them to `contributing` in first-occurrence
     * (tick) order — the public contract — with the fixed hits[] order breaking ties so the
     * output is deterministic. The earliest ticks are kept when the run exceeds
     * CLASSIFICATION_MAX_CONTRIBUTING (PR #80 review). */
    struct Detected detected[nhits];
    int ndetected = 0;
    for (int i = 0; i < nhits; i++) {
        if (hits[i].hit->detected) detected[ndetected++] = hits[i];
    }
    for (int i = 1; i < ndetected; i++) {
        const struct Detected key = detected[i];
        int j = i - 1;
        while (j >= 0 && detected[j].hit->tick > key.hit->tick) {
            detected[j + 1] = detected[j];
            j--;
        }
        detected[j + 1] = key;
    }
    for (int i = 0; i < ndetected && out->contributingCount < CLASSIFICATION_MAX_CONTRIBUTING;
         i++) {
        out->contributing[out->contributingCount].reason = detected[i].cls;
        out->contributing[out->contributingCount].tick = detected[i].hit->tick;
        out->contributing[out->contributingCount].timeS = detected[i].hit->timeS;
        out->contributingCount++;
    }

    FailureClass primary = RUN_CLASS_PASS;
    uint64_t earliestTick = 0;
    double earliestTimeS = 0.0;
    bool haveEarliest = false;
    for (int i = 0; i < nhits; i++) {
        if (!hits[i].hit->detected) continue;
        /* INVALID_PHYSICS forces the headline; otherwise earliest causal tick wins. */
        if (hits[i].cls == RUN_CLASS_INVALID_PHYSICS) {
            primary = RUN_CLASS_INVALID_PHYSICS;
            earliestTick = hits[i].hit->tick;
            earliestTimeS = hits[i].hit->timeS;
            haveEarliest = true;
            break;
        }
        /* Earliest causal tick wins. Ties resolve to the first in hits[] order, which is fixed
         * by severity, so the pick is deterministic without a separate ranking call. */
        if (!haveEarliest || hits[i].hit->tick < earliestTick) {
            primary = hits[i].cls;
            earliestTick = hits[i].hit->tick;
            earliestTimeS = hits[i].hit->timeS;
            haveEarliest = true;
        }
    }
    (void)metrics;

    /* A run that completed its target laps is a PASS regardless of transient events. The fault
     * classes classify FAILURES, and a completed run can carry a transient planner/localization
     * disagreement (the knife-edge awd_rally slides but still makes every gate) that is not a
     * failure. Clear the headline and the contributing list; keep the evidence fields. The one
     * verdict that survives completion is INVALID_PHYSICS: a non-finite state is a failure no
     * matter where it appeared (PR #80 review). */
    if (!invalidPhysics.detected && count > 0 && rows[count - 1].lapIndex >= in->targetLaps) {
        primary = RUN_CLASS_PASS;
        haveEarliest = false;
        earliestTick = 0;
        earliestTimeS = 0.0;
        out->contributingCount = 0;
    }

    out->primary = primary;
    out->firstFaultTick = haveEarliest ? earliestTick : 0;
    out->firstFaultTimeS = haveEarliest ? earliestTimeS : 0.0;

    /* Final evidence. */
    out->lastCheckpointIndex = (count > 0) ? rows[count - 1].lastCrossedIndex : -1;
    out->expectedCheckpointIndex = (count > 0) ? rows[count - 1].checkpointIndex : -1;
    out->furthestRouteDistanceM = (furthestProgressM < 0.0) ? 0.0 : furthestProgressM;
    out->lastProgressTick = lastProgressTick;
    {
        const double runEndS = (count > 0) ? rows[count - 1].timeS : 0.0;
        out->timeSinceProgressS = runEndS - lastProgressTimeS;
    }
}
