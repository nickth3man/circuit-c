/*
 * validation_classifier.h — deterministic failure classification for a validation run.
 *
 * The validation runner's status (run_report.h) is a coarse, closed set: it says a run failed,
 * not WHY in driver-relevant terms. Issue #78 asks every failed run to identify a primary
 * reason, the first causal tick, and the contributing events — distinguishing a spin that became
 * a departure from a checkpoint bug, a slow-but-moving timeout from a stationary stall, a stop on
 * asphalt from one beyond the runoff.
 *
 * This is a PURE reducer over the telemetry rows a run already collected plus the ValidationMetrics
 * and a small inputs struct. No Game, no global state, no I/O: the same rows produce the same
 * classification every time, and the unit test (a hand-built row array) checks every class by
 * construction the way validation_metrics' test does.
 *
 * Classification is deliberately a reduction over AUTHORITATIVE events/telemetry. It does not
 * invent new physics or new gates, and it does not mutate RacerProgress, the checksum, or race
 * state. Diagnostic progress bins feed it through the rows.
 */
#ifndef CIRCUIT_VALIDATION_CLASSIFIER_H
#define CIRCUIT_VALIDATION_CLASSIFIER_H

#include <stdbool.h>
#include <stdint.h>

#include "game/telemetry.h"
#include "game/validation_metrics.h"

/* The closed set of classified outcomes. RUN_CLASS_PASS is the only one that is not a failure.
 * Ordered roughly by how strongly the evidence forces the verdict, but the classifier selects
 * the PRIMARY reason from the earliest causal event (see validation_classify), not from this
 * order. */
typedef enum {
    RUN_CLASS_PASS = 0,
    RUN_CLASS_INVALID_PHYSICS,         /* a non-finite pose/speed appeared */
    RUN_CLASS_CHECKPOINT_OUT_OF_ORDER, /* a gate was crossed backwards / out of sequence */
    RUN_CLASS_CHECKPOINT_SKIPPED,  /* a later gate was crossed while an earlier one was owed */
    RUN_CLASS_COLLISION_STUCK,     /* a contact preceded and led to a stall */
    RUN_CLASS_SPIN_THEN_DEPARTURE, /* a sustained spin was followed by a route departure */
    RUN_CLASS_STALLED_OFF_TRACK,   /* stopped beyond the racing surface / runoff */
    RUN_CLASS_STALLED_ON_TRACK,    /* stopped on the racing surface */
    RUN_CLASS_WRONG_WAY,           /* sustained travel against the route direction */
    RUN_CLASS_ROUTE_DEPARTURE,     /* left the route (beyond runoff) without a preceding spin */
    RUN_CLASS_LOCALIZATION_LOST,   /* route localization invalidated or jumped strands */
    RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH, /* the AI segment disagreed with the route segment */
    RUN_CLASS_SLOW_TIMEOUT /* the tick budget expired while still making forward progress */
} FailureClass;

/* A single contributing event: a class that was detected, the tick it first occurred, and the
 * simulation time then. The classification's `primary` is one of these, enlarged. */
typedef struct {
    FailureClass reason;
    uint64_t tick; /* simulation tick of first occurrence; 0 if not tick-based */
    double timeS;  /* simulation time at that tick */
} ContributingEvent;

#define CLASSIFICATION_MAX_CONTRIBUTING 8

typedef struct {
    FailureClass primary;    /* RUN_CLASS_PASS if the run succeeded */
    uint64_t firstFaultTick; /* earliest causal-event tick; 0 if the run passed */
    double firstFaultTimeS;  /* simulation time at that tick */

    /* Every distinct failure class detected, in first-occurrence order (ties broken by the
     * fixed severity order), truncated to CLASSIFICATION_MAX_CONTRIBUTING: a run that detects
     * more classes keeps only the EARLIEST ones, so the bounded report never drops the causal
     * head of the sequence. `primary` is one of these (the one selected as the headline). A
     * spin-then-departure, for instance, lists both the spin and the departure though only one
     * is primary. */
    ContributingEvent contributing[CLASSIFICATION_MAX_CONTRIBUTING];
    int contributingCount;

    /* Final-state evidence, from the last row plus run inputs, so a report states where the run
     * stopped and what it owed. */
    int lastCheckpointIndex;       /* last gate crossed this run, -1 if none */
    int expectedCheckpointIndex;   /* next gate the car owed at the end */
    double furthestRouteDistanceM; /* furthest lap-relative progress reached */
    uint64_t lastProgressTick;     /* last tick the furthest progress bin advanced */
    double timeSinceProgressS;     /* run end time minus lastProgressTick time */
} ValidationClassification;

/* Inputs the classifier cannot derive from the rows alone. Every threshold is a named, documented
 * constant so the test and the report quote the same number. */
typedef struct {
    int checkpointCount;      /* gates per lap */
    int startCheckpointIndex; /* gate the run started at (lap-close anchor) */
    int targetLaps;           /* e.g. VALIDATION_RUN_LAPS */
    int tickBudget;           /* e.g. REPLAY_CAPACITY_TICKS */
    int ticksRun;             /* actual ticks simulated */
    double fixedDtS;          /* seconds per tick, for tick<->time conversions */

    /* Thresholds. */
    double meaningfulProgressM; /* bin advance that counts as forward progress (e.g. 10) */
    double stallSpeedMps;       /* speed at/under which the car counts as stopped (e.g. 0.5) */
    double stallDurationS;      /* sustained stop this long => stalled (e.g. 3.0) */
    double wrongWayHoldS;       /* sustained wrong-way this long => wrong_way */
    double departureHoldS;      /* sustained off-route this long => route_departure */
    double mismatchHoldS;       /* sustained AI/route segment mismatch => planner mismatch */
    double
        collisionSpeedMpsEps; /* contacts below this approach speed don't attribute (e.g. 0.1) */
    double spinSideslipRad;   /* |body sideslip| above this is a spin attitude */
    double spinMinSpeedMps;   /* ...only while still moving */
    double spinMinDurationS;  /* ...that must persist this long */
    double progressRecencyS;  /* progress this recently at budget expiry => slow_timeout, not a
                                stall (e.g. 2.0) */
} ClassificationInputs;

/*
 * The shared threshold defaults, defined once so the validation runner, `ai-roster-laps`, and
 * the by-construction scenario all classify with exactly the same numbers (PR #80 review).
 * Only the per-run fields (checkpointCount, startCheckpointIndex, targetLaps, tickBudget,
 * ticksRun, fixedDtS) are left to the caller after this returns.
 */
void validation_classification_inputs_default(ClassificationInputs *out);

/*
 * Reduce `rows[0 .. count)` + `metrics` + `inputs` into `out`. Pure: allocates no persistent
 * state and touches nothing outside `out`. Safe on count <= 0 (writes a PASS classification).
 * `metrics` is currently reserved: it stays in the signature as the classification input
 * contract, but no reducer reads it yet (PR #80 review).
 *
 * Selection rule for `primary`: the run's headline reason is the class attached to the EARLIEST
 * causal event, with two adjustments — INVALID_PHYSICS always wins when present (no other verdict
 * is meaningful once the state is non-finite), and SLOW_TIMEOUT only applies when the budget
 * expired and the car was still progressing (a stopped car at budget expiry is a stall, not a
 * slow timeout). `firstFaultTick` is the earliest detected causal-event tick, never the budget
 * timeout unless nothing earlier happened.
 */
void validation_classify(const TelemetryRow *rows, int count, const ValidationMetrics *metrics,
                         const ClassificationInputs *inputs, ValidationClassification *out);

/* The closed-set token for a class ("spin_then_departure", ...), or "pass". */
const char *failure_class_reason(FailureClass c);

/* One-line human-readable summary of a classification, for run logs and the failure bundle. */
const char *failure_class_label(FailureClass c);

#endif /* CIRCUIT_VALIDATION_CLASSIFIER_H */
