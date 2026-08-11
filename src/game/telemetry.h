/*
 * telemetry.h — CSV row writer shared by every headless scenario.
 *
 * Deliberately independent of raylib and of any window: it is plain <stdio.h>, so the
 * headless test executable and (later) the game module can both emit telemetry.
 *
 * TelemetryRow carries the stable physics schema. The header string is generated from the
 * same field list that the row writer formats, so columns and values cannot drift apart.
 *
 * Phase 3 appended the load-transfer, acceleration-filter, and resistance block at the end,
 * leaving every earlier column in place and in order. `front_normal_load_n` and
 * `dynamic_front_load_n` therefore carry the same value: the first is the Phase 2 column
 * name that existing baselines and tools use, the second is the Phase 3 name that reads
 * correctly beside `static_front_load_n`. Both are written rather than one being renamed.
 *
 * Formatting is fixed-precision (%.6f) rather than %g so that byte-for-byte diffs against a
 * committed baseline are meaningful.
 */
#ifndef CIRCUIT_TELEMETRY_H
#define CIRCUIT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Stable Phase 2 telemetry schema. */
typedef struct {
    uint64_t tick;
    double timeS;
    float positionXM;
    float positionYM;
    float headingRad;
    float velocityLongitudinalMps;
    float velocityLateralMps;
    float speedMps;
    float yawRateRadS;
    float steeringAngleRad;
    float engineRpm;
    int selectedGear;
    float frontSlipAngleRad;
    float rearSlipAngleRad;
    float frontSlipRatio;
    float rearSlipRatio;
    float frontWheelOmegaRadS;
    float rearWheelOmegaRadS;
    float frontNormalLoadN;
    float rearNormalLoadN;
    float frontFxPureN;
    float rearFxPureN;
    float frontFyPureN;
    float rearFyPureN;
    float frontFxLimitedN;
    float rearFxLimitedN;
    float frontFyLimitedN;
    float rearFyLimitedN;
    float frontFrictionUsage;
    float rearFrictionUsage;
    int frontLocked;
    int rearLocked;
    float driveTorqueNm;
    float frontBrakeTorqueNm;
    float rearBrakeTorqueNm;
    float handbrakeTorqueNm;
    float totalForceXN;
    float totalForceYN;
    float yawTorqueNm;
    float bodySideslipRad;
    float lowSpeedBlend;
    int substepCount;
    int backlogDrops;
    uint32_t stateChecksum;

    /* Phase 3: load transfer, the acceleration filter, and separated resistance. */
    float staticFrontLoadN;
    float staticRearLoadN;
    float dynamicFrontLoadN;
    float dynamicRearLoadN;
    float loadTransferN;
    float previousLongAccelMps2;
    float filteredLongAccelMps2;
    float solvedLongAccelMps2;
    float lateralAccelMps2;
    float aeroDragN;
    float aeroDragXN;
    float aeroDragYN;
    float rollingResistanceN;
    float rollingResistanceXN;
    float rollingResistanceYN;

    /* The driver's held controls, so a report can show what was asked for beside what the
     * car did. */
    float steeringInput;
    float throttleInput;
    float brakeInput;
    float handbrakeInput;

    /* Per-wheel surface identity, appended so existing columns remain stable. */

    int surfaceFrontLeft;
    int surfaceFrontRight;
    int surfaceRearLeft;
    int surfaceRearRight;

    /* Phase 5: lap-validation columns. Appended so every earlier column stays in place and
     * in order; zero for runs that have no checkpointed track, which is why appending them
     * cannot move a committed baseline. The metrics layer (validation_metrics.c) is a pure
     * reducer over these. */
    int checkpointIndex; /* next required gate, or the last one taken */
    int lapIndex;        /* laps completed so far */
    int lapState;        /* 0 out-lap / 1 timed / 2 complete / 3 aborted */
    int checkpointEvent; /* 0 none / 1 in-order / 2 out-of-order / 3 lap-complete */
    float crashLockoutS; /* seconds left in post-impact lockout; rising edge = collision */
    float distanceToCenterlineM; /* nearest-segment distance, signed magnitude */
    int onTrack;                 /* 1 iff all four wheels report the racing surface */

    /* Phase 5: per-wheel slip, so a diagnosis can distinguish the two wheels on an axle —
     * inside/outside load split, one locked wheel, a differential sending torque to the
     * unloaded side. These are the values the tire model is actually evaluated at
     * (physics.c computes them per wheel), not a reconstruction.
     *
     * All four wheels are written for both quantities rather than only the two the axle
     * columns omit, because the axle columns are not wheel values: `front_slip_angle_rad`
     * is the bicycle-model axle slip angle from physics_axle_slip_angles(), and
     * `front_slip_ratio` is the mean of the two front wheels. Neither equals its left
     * wheel, so appending only FR/RR would pair columns that do not mean the same thing.
     * The axle columns keep their names and their meaning, per append-never-rename. */
    float slipAngleFrontLeftRad;
    float slipAngleFrontRightRad;
    float slipAngleRearLeftRad;
    float slipAngleRearRightRad;
    float slipRatioFrontLeft;
    float slipRatioFrontRight;
    float slipRatioRearLeft;
    float slipRatioRearRight;

    /* Phase 6: validation diagnosability (#78). Appended so every earlier column stays in place
     * and in order; zero for runs that have no checkpointed track or no AI controller, which is
     * why appending cannot move a committed baseline. The metrics layer and the failure
     * classifier (validation_classifier.h) are pure reducers over these. */

    /* Authoritative route localization (RacerProgress.location), so a failed run reports WHERE
     * on the route it stopped rather than only which gate it owed. */
    int routeSegmentIndex;      /* centreline segment nodes[i]->nodes[i+1]; -1 if invalid */
    float routeSegmentT;        /* [0,1] along that segment */
    float routeLongitudinalM;   /* arc length from node 0 along the route */
    float routeLateralM;        /* signed offset from centreline, +left of travel */
    float routeHeadingErrorRad; /* car heading minus route heading, [-PI,PI) */
    float routeConfidence;      /* 1 on surface -> 0 across runoff -> 0 at/beyond barrier */
    int onRouteFlag;            /* 1 iff within the segment's racing half-width */
    float routeDepartureDistM; /* |pos - closest centreline point|; the actual leave distance */

    /* Named off-track definitions (#78 §5), reported side by side rather than collapsed into one
     * ambiguous boolean. iRacing/ACC/FIA each name a different definition; this carries all of
     * them so a report is explicit about which one fired. */
    int wheelsOffAsphalt; /* 0..4 count of wheels not on the racing surface */
    int beyondRunoff;     /* 1 iff at/past the barrier (routeConfidence <= 0) */

    /* Non-scoring progress bins (#78 §2). Derived from routeLongitudinalM at a 10 m interval;
     * DIAGNOSTIC ONLY — they never mutate nextCheckpoint, lap validity, or the checksum. */
    float progressBinM;      /* current lap-relative bin (longitudinalM mod lap, binned) */
    float furthestProgressM; /* furthest bin reached in the current lap */

    /* Lap/checkpoint state the classifier correlates with route position. */
    int lastCrossedIndex; /* last gate crossed this run, -1 initially */
    int ticksSinceCross;  /* hysteresis ticks since the last crossing */
    int lapArmedFlag;     /* SF latch state */
    int lapInvalidFlag;   /* a required gate was skipped */
    float lapTimerSCol;   /* seconds since the last checkpoint/lap */
    int wrongWayFlag;     /* latched wrong-way state */

    /* AI decision telemetry (AiDriverState + emitted controls), so a planner/localization
     * disagreement is visible rather than inferred. Reported beside the authoritative route
     * segment above. Zero on non-AI runs. */
    int aiSegment;          /* the segment the driver matched last tick */
    float aiCrossTrackM;    /* signed; + when left of the planned line */
    float aiTargetSpeedMps; /* what the speed controller was aiming for */
    float aiLookaheadRad;   /* bearing to the lookahead point, body frame */
    float aiBindingCurv1pm; /* curvature that set the speed target */
    float aiBindingDistM;   /* how far ahead that curvature was */
    float aiPedalAxis;      /* +1 full throttle / -1 full brake (one signed axis) */
    float aiSteerAxis;      /* the steering axis actually emitted */
    float aiGripCut;        /* throttle surrendered to traction management */
    int aiPlanBaseNode;     /* centreline node the plan window starts at */
    int aiPlanLayerCount;   /* plan layers currently held */

    /* PR #80 review follow-up: an explicit AI-presence flag and the gate the most recent
     * checkpoint event actually crossed. An out-of-order crossing does NOT advance
     * progress->lastCrossedIndex (it names the previous legal gate), so the classifier needs
     * the event's own index to tell a forward skip from a gate behind the owed one. Both stay
     * zero/absent on rows without a crossing or an AI controller; appending keeps every
     * earlier column in place and in order. */
    int aiPresent;              /* 1 iff an AI controller drove this row; 0 otherwise */
    int checkpointCrossedIndex; /* gate crossed by the last checkpoint event; -1 when none */
} TelemetryRow;

typedef struct {
    FILE *file;
    long rowCount;
    bool failed; /* set once any write fails; further writes are refused */
} TelemetryWriter;

/* Create dirPath if it does not exist. Returns false if it could not be created and does
 * not already exist. */
bool telemetry_ensure_dir(const char *dirPath);

/* Open path for writing and emit the header row. Returns false on failure, leaving the
 * writer closed and safe to pass to telemetry_close(). */
bool telemetry_open(TelemetryWriter *writer, const char *path);

/* Append one row. Returns false if the writer is not open or the write failed. */
bool telemetry_write_row(TelemetryWriter *writer, const TelemetryRow *row);

/* Flush and close. Returns false if the file was open and closing or any earlier write
 * failed. Safe to call on a writer that was never opened. */
bool telemetry_close(TelemetryWriter *writer);

/* The exact header line (without newline) that telemetry_open() writes. */
const char *telemetry_header(void);

#endif /* CIRCUIT_TELEMETRY_H */
