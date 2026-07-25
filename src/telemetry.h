/*
 * telemetry.h — CSV row writer shared by every headless scenario.
 *
 * Deliberately independent of raylib and of any window: it is plain <stdio.h>, so the
 * headless test executable and (later) the game module can both emit telemetry.
 *
 * The Phase 0 row is the infrastructure subset. Phase 1 onward extends TelemetryRow with
 * the full physics schema from docs/SPEC.md ("Physics Validation and Regression Testing").
 * The header string is generated from the same field list that the row writer formats, so
 * columns and values cannot drift apart.
 *
 * Formatting is fixed-precision (%.6f) rather than %g so that byte-for-byte diffs against a
 * committed baseline are meaningful.
 */
#ifndef DRIFTY_TELEMETRY_H
#define DRIFTY_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Phase 0 telemetry schema. */
typedef struct {
    uint64_t tick;
    double   timeS;
    float    heldSteer;
    float    heldThrottle;
    bool     pausePressed;
    bool     resetPressed;
    int      substepCount;
    int      backlogDrops;
    uint32_t stateChecksum;
} TelemetryRow;

typedef struct {
    FILE *file;
    long  rowCount;
    bool  failed;       /* set once any write fails; further writes are refused */
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

#endif /* DRIFTY_TELEMETRY_H */
