#include "telemetry.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(_WIN32)
#include <direct.h>
#define drifty_mkdir(path) _mkdir(path)
#else
#define drifty_mkdir(path) mkdir((path), 0775)
#endif

#define TELEMETRY_HEADER \
    "tick,time_s,held_steer,held_throttle,pause_pressed,reset_pressed," \
    "substep_count,backlog_drops,state_checksum"

const char *telemetry_header(void)
{
    return TELEMETRY_HEADER;
}

bool telemetry_ensure_dir(const char *dirPath)
{
    if (dirPath == NULL || dirPath[0] == '\0') return false;

    if (drifty_mkdir(dirPath) == 0) return true;
    if (errno == EEXIST) return true;

    fprintf(stderr, "TELEMETRY: could not create directory '%s': %s\n",
            dirPath, strerror(errno));
    return false;
}

bool telemetry_open(TelemetryWriter *writer, const char *path)
{
    if (writer == NULL) return false;

    writer->file     = NULL;
    writer->rowCount = 0;
    writer->failed   = false;

    if (path == NULL || path[0] == '\0') {
        writer->failed = true;
        return false;
    }

    writer->file = fopen(path, "wb");
    if (writer->file == NULL) {
        fprintf(stderr, "TELEMETRY: could not open '%s' for writing: %s\n",
                path, strerror(errno));
        writer->failed = true;
        return false;
    }

    if (fprintf(writer->file, "%s\n", TELEMETRY_HEADER) < 0) {
        fprintf(stderr, "TELEMETRY: could not write header to '%s'\n", path);
        writer->failed = true;
        fclose(writer->file);
        writer->file = NULL;
        return false;
    }

    return true;
}

bool telemetry_write_row(TelemetryWriter *writer, const TelemetryRow *row)
{
    if (writer == NULL || writer->file == NULL || writer->failed || row == NULL) return false;

    const int written = fprintf(writer->file,
                                "%" PRIu64 ",%.6f,%.6f,%.6f,%d,%d,%d,%d,%" PRIu32 "\n",
                                row->tick,
                                row->timeS,
                                (double)row->heldSteer,
                                (double)row->heldThrottle,
                                row->pausePressed ? 1 : 0,
                                row->resetPressed ? 1 : 0,
                                row->substepCount,
                                row->backlogDrops,
                                row->stateChecksum);
    if (written < 0) {
        fprintf(stderr, "TELEMETRY: write failed after %ld rows\n", writer->rowCount);
        writer->failed = true;
        return false;
    }

    writer->rowCount++;
    return true;
}

bool telemetry_close(TelemetryWriter *writer)
{
    if (writer == NULL) return false;
    if (writer->file == NULL) return !writer->failed;

    const bool closedCleanly = (fclose(writer->file) == 0);
    writer->file = NULL;

    if (!closedCleanly) {
        fprintf(stderr, "TELEMETRY: fclose failed: %s\n", strerror(errno));
        writer->failed = true;
    }

    return !writer->failed;
}
