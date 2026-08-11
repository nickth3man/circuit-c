/*
 * run_report.h — write the machine-readable run.json for one validation lap.
 *
 * run.json is the contract a handling change is judged against: it carries the car, the track,
 * the build, the lap result, and the full metrics block, so a diff between two runs is a diff
 * between two of these files. The writer is hand-rolled fprintf with JSON escaping, exactly as
 * failure_bundle.c does for summary.json — no JSON library is pulled into the game module.
 *
 * Failure reasons are a closed set so the suite orchestrator can group and count them without
 * parsing free text.
 */
#ifndef CIRCUIT_RUN_REPORT_H
#define CIRCUIT_RUN_REPORT_H

#include <stdbool.h>

#include "game/validation_metrics.h"
#include "world/track.h" /* RacerProgress */

/* The closed set of run outcomes. RUN_PASS is the only one that is not a failure. */
typedef enum {
    RUN_PASS = 0,
    RUN_FAIL_CHECKPOINT_MISSED,
    RUN_FAIL_CHECKPOINT_OUT_OF_ORDER,
    RUN_FAIL_STALLED,
    RUN_FAIL_TICK_BUDGET_EXCEEDED,
    RUN_FAIL_INVALID_STATE,
    RUN_FAIL_VIDEO_ENCODE_FAILED,
    RUN_FAIL_SPEC_INVALID
} RunStatus;

/* Capacity of RunReportInput.contributingReasons, shared with the writer loop and the runner's
 * copy loop so a change cannot silently truncate at one site (PR #80 review). */
#define RUN_REPORT_MAX_CONTRIBUTING 8

typedef struct {
    /* Identity. runId is caller-built (e.g. "20260807-143201-chicane_v1-rwd_grip"). */
    const char *runId;

    /* Car. specHash/finalStateChecksum are caller-formatted hex strings. */
    const char *carId;
    const char *carDisplayName;
    const char *carDrivetrain; /* "RWD" / "FWD" / "AWD" */
    double carMassKg;
    const char *carSpecHash;

    /* Track. */
    const char *trackId;
    const char *trackVersion;
    const char *trackGeometryHash;
    int trackCheckpointCount;
    double trackLengthM;
    int startCheckpointIndex;

    /* Simulation. */
    int fixedHz;
    int telemetryHz;
    int videoFps;
    const char *buildCommit;
    bool buildDirty;
    const char *finalStateChecksum;
    int tickBudget;
    int ticksRun;

    /* Result + lap accounting. checkpointsMissed is what the CURRENT incomplete lap still
     * owed (scored gates 1..24 of a 25-gate lap; the start/finish gate is the lap anchor), so
     * it stays in [0, checkpointCount) and never goes negative across completed laps. */
    RunStatus status;
    int checkpointsPassed;
    int checkpointsMissed;
    int outOfOrderEvents;

    /* Failure classification (issue #78): the fine-grained reason a run failed, the earliest
     * causal tick, the contributing events in first-occurrence order, and where the run stopped
     * and what it owed. All zero/empty for a PASS. */
    const char
        *classificationReason; /* primary class token ("pass", "spin_then_departure", ...) */
    uint64_t firstFaultTick;   /* earliest causal-event tick; 0 if pass */
    int contributingCount;     /* 0..RUN_REPORT_MAX_CONTRIBUTING, in first-occurrence order */
    const char *contributingReasons[RUN_REPORT_MAX_CONTRIBUTING]; /* their closed-set tokens */
    int lastCheckpointIndex;       /* last gate crossed this run, -1 if none */
    int expectedCheckpointIndex;   /* next gate owed at the end */
    double furthestRouteDistanceM; /* furthest lap-relative progress reached (m) */
    double timeSinceProgressS;     /* run end minus last meaningful progress (s) */

    /* Metrics block. May be NULL only when status is RUN_FAIL_SPEC_INVALID (no run happened). */
    const ValidationMetrics *metrics;

    /* Artifacts present in the same directory as the run.json. */
    bool hasVideo;
    bool hasReplay;
} RunReportInput;

/* "PASS" for the one passing status, "FAIL" otherwise. */
const char *run_status_label(RunStatus s);

/* The closed-set failure token ("checkpoint_missed", ...), or NULL for RUN_PASS. */
const char *run_failure_reason(RunStatus s);

/*
 * Lap-aware missing-checkpoint accounting (issue #78 §5): how many gates of the CURRENT
 * incomplete lap are still owed, from the racer's real progress state. The start/finish gate
 * is the lap anchor (crossed at spawn, never scored), so a 25-gate lap starts at 24 owed and
 * reaches 0 once every scored gate is crossed — including the nextCheckpoint == lapStart
 * wrap, where the plain modulo would read a full lap as zero crossings. Never negative; 0
 * when progress is NULL or checkpointCount <= 0. Extracted from the validation runner so the
 * accounting test exercises the same function the runner reports with (PR #80 review).
 */
int run_report_missed_checkpoints(const RacerProgress *progress, int checkpointCount);

/*
 * Write run.json to `path`. Returns false only when the file could not be opened. The input
 * struct borrows every string; they only need to outlive this call.
 */
bool run_report_write(const char *path, const RunReportInput *in);
#endif /* CIRCUIT_RUN_REPORT_H */
