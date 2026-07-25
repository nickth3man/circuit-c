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
    "tick,time_s,position_x_m,position_y_m,heading_rad," \
    "velocity_longitudinal_mps,velocity_lateral_mps,speed_mps,yaw_rate_rad_s," \
    "steering_angle_rad,front_slip_angle_rad,rear_slip_angle_rad," \
    "front_normal_load_n,rear_normal_load_n,front_lateral_force_n," \
    "rear_lateral_force_n,total_force_x_n,total_force_y_n,yaw_torque_nm," \
    "body_sideslip_rad,low_speed_blend,substep_count,backlog_drops,state_checksum"

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

    const int written = fprintf(
        writer->file,
        "%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%d,%d,%" PRIu32 "\n",
        row->tick, row->timeS,
        (double)row->positionXM, (double)row->positionYM,
        (double)row->headingRad,
        (double)row->velocityLongitudinalMps,
        (double)row->velocityLateralMps, (double)row->speedMps,
        (double)row->yawRateRadS, (double)row->steeringAngleRad,
        (double)row->frontSlipAngleRad, (double)row->rearSlipAngleRad,
        (double)row->frontNormalLoadN, (double)row->rearNormalLoadN,
        (double)row->frontLateralForceN, (double)row->rearLateralForceN,
        (double)row->totalForceXN, (double)row->totalForceYN,
        (double)row->yawTorqueNm, (double)row->bodySideslipRad,
        (double)row->lowSpeedBlend, row->substepCount, row->backlogDrops,
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
