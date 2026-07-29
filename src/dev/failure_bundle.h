/*
 * failure_bundle.h — everything needed to reproduce a failure, in one directory.
 *
 * When a headless scenario fails, the runner drops a self-contained bundle:
 *
 *     artifacts/failure-<scenario>-<timestamp>/
 *     ├── replay.bin           the exact input timeline (dev_replay format)
 *     ├── telemetry.csv        the run's telemetry, copied verbatim
 *     ├── summary.json         scenario, first failing tick, checksum, counts, build facts
 *     ├── config_snapshot.txt  every tunable's value at failure time
 *     ├── git_info.txt         commit, dirty flag, compiler, flags, platform, build mode
 *     └── failure.txt          the failing check, verbatim
 *
 * The point is that a human or an agent can act on the bundle alone, without reconstructing
 * the environment that produced it. Nothing here is raylib-dependent.
 */
#ifndef DRIFTY_FAILURE_BUNDLE_H
#define DRIFTY_FAILURE_BUNDLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "game/replay.h"
#include "physics/vehicle.h"

typedef struct {
    const char *scenario;       /* required; used in the directory name */
    const char *failureText;    /* the failing check message, may be multi-line */
    const char *telemetryPath;  /* CSV copied into the bundle, or NULL */
    const char *screenshotPath; /* PNG copied into the bundle, or NULL (headless runs) */
    const ReplayBuffer *replay; /* timeline written as replay.bin, or NULL */
    const VehicleSpec *spec;    /* tunables snapshot, or NULL */
    const char *activeProfile;  /* profile label recorded in config and summary */
    uint64_t failingTick;
    uint32_t checksum;
    uint32_t seed;
    int checksRun;
    int checksFailed;
} FailureBundle;

/* Create <rootDir>/failure-<scenario>-<YYYYmmdd-HHMMSS>/ and fill it. rootDir and its parent
 * are created if missing. On success, dirOut receives the bundle path (when non-NULL).
 * Returns false if the directory could not be created; individual files that fail to write
 * are reported on stderr and noted in summary.json rather than aborting the bundle. */
bool failure_bundle_write(const char *rootDir, const FailureBundle *bundle, char *dirOut,
                          size_t dirCapacity);

#endif /* DRIFTY_FAILURE_BUNDLE_H */
