/*
 * ghost.c — issue #51 implementation.
 */
#include "game/ghost.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool valid_magic(const char magic[14])
{
    return memcmp(magic, GHOST_MAGIC, 13) == 0 && magic[13] == '\n';
}

bool ghost_validate(const GhostRecording *rec, uint32_t trackHash, const char *trackId,
                    const VehicleDefinition *definition, const VehicleSetup *setup,
                    char *reason, size_t reasonCap)
{
    if (rec == NULL) {
        if (reason != NULL && reasonCap > 0) snprintf(reason, reasonCap, "no ghost recording");
        return false;
    }
    if (rec->schema != GHOST_SCHEMA_VERSION) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "ghost schema %u unsupported (need %u)", rec->schema,
                     GHOST_SCHEMA_VERSION);
        return false;
    }
    if (trackId != NULL && strcmp(rec->trackId, trackId) != 0) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "ghost track '%s' does not match '%s'", rec->trackId,
                     trackId);
        return false;
    }
    if (rec->trackHash != trackHash) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "ghost track hash %08x does not match %08x",
                     rec->trackHash, trackHash);
        return false;
    }
    if (definition != NULL) {
        if (strcmp(rec->carId, definition->id) != 0) {
            if (reason != NULL && reasonCap > 0)
                snprintf(reason, reasonCap, "ghost car '%s' does not match '%s'", rec->carId,
                         definition->id);
            return false;
        }
        if (rec->carHash != definition->contentHash) {
            if (reason != NULL && reasonCap > 0)
                snprintf(reason, reasonCap, "ghost car hash %08x does not match %08x",
                         rec->carHash, definition->contentHash);
            return false;
        }
    }
    if (setup != NULL && rec->setupHash != vehicle_setup_hash(setup)) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "ghost setup hash %08x does not match %08x",
                     rec->setupHash, vehicle_setup_hash(setup));
        return false;
    }
    if (!rec->replay.initialVehicle.valid || rec->replay.count <= 0) {
        if (reason != NULL && reasonCap > 0)
            snprintf(reason, reasonCap, "ghost recording is empty or incomplete");
        return false;
    }
    return true;
}

bool ghost_store_save(const char *path, const GhostRecording *rec)
{
    if (path == NULL || rec == NULL) return false;
    char tmp[1024];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (f == NULL) return false;

    fprintf(f, "%s\n", GHOST_MAGIC);
    fprintf(f, "schema=%u\n", rec->schema);
    fprintf(f, "track=%s\n", rec->trackId);
    fprintf(f, "trackHash=%08x\n", rec->trackHash);
    fprintf(f, "car=%s\n", rec->carId);
    fprintf(f, "carHash=%08x\n", rec->carHash);
    fprintf(f, "setupHash=%08x\n", rec->setupHash);
    fprintf(f, "bestLap=%.9g\n", (double)rec->bestLapTimeS);
    fprintf(f, "frames=%d\n", rec->replay.count);
    fprintf(f, "BEGIN\n");

    /* The initial vehicle snapshot. */
    const ReplayVehicleSnapshot *iv = &rec->replay.initialVehicle;
    fprintf(f, "IV %u %u %u %d\n", iv->definitionVersion, iv->definitionHash,
            iv->valid ? 1u : 0u, (int)sizeof(iv->setup));
    fwrite(&iv->setup, sizeof(iv->setup), 1, f);
    fwrite(&iv->vehicle, sizeof(iv->vehicle), 1, f);
    fwrite(&iv->renderState, sizeof(iv->renderState), 1, f);
    fwrite(&iv->autoTrans, sizeof(iv->autoTrans), 1, f);
    fwrite(&iv->vehicleControls, sizeof(iv->vehicleControls), 1, f);
    fwrite(&iv->fuelKg, sizeof(iv->fuelKg), 1, f);
    fwrite(iv->tireState, sizeof(iv->tireState), 1, f);
    fwrite(&iv->damage, sizeof(iv->damage), 1, f);
    fwrite(&iv->crashLockoutTimerS, sizeof(iv->crashLockoutTimerS), 1, f);

    /* The controller frames: packed, portable (no padding). */
    for (int i = 0; i < rec->replay.count; i++) {
        const int idx = (rec->replay.head + i) % REPLAY_CAPACITY_TICKS;
        const ReplayFrame *fr = &rec->replay.frames[idx];
        const float steer = fr->steer;
        const float throttle = fr->throttle;
        const float brake = fr->brake;
        const float handbrake = fr->handbrake;
        const uint8_t oneshot = fr->oneshotBits;
        fwrite(&steer, sizeof(steer), 1, f);
        fwrite(&throttle, sizeof(throttle), 1, f);
        fwrite(&brake, sizeof(brake), 1, f);
        fwrite(&handbrake, sizeof(handbrake), 1, f);
        fwrite(&oneshot, sizeof(oneshot), 1, f);
    }
    if (fclose(f) != 0) return false;
    bool moved = false;
#if defined(_WIN32)
    /* The MSYS runtime's rename() refuses to replace an existing destination, so remove it
     * first. The window between remove and rename is tiny and the tmp file keeps the data
     * intact if the process dies inside it. */
    remove(path);
    moved = (rename(tmp, path) == 0);
#else
    moved = (rename(tmp, path) == 0);
#endif
    if (!moved) {
        remove(tmp);
        return false;
    }
    return true;
}

bool ghost_store_load(const char *path, GhostRecording *rec)
{
    if (path == NULL || rec == NULL) return false;
    memset(rec, 0, sizeof(*rec));
    FILE *f = fopen(path, "rb");
    if (f == NULL) return false;

    char line[512];
    if (fgets(line, sizeof(line), f) == NULL || !valid_magic(line)) {
        fclose(f);
        return false;
    }
    int frames = 0;
    bool haveAll = true;
    while (fgets(line, sizeof(line), f) != NULL) {
        if (strncmp(line, "schema=", 7) == 0) {
            rec->schema = (uint32_t)strtoul(line + 7, NULL, 10);
        } else if (strncmp(line, "track=", 6) == 0) {
            line[strcspn(line, "\n")] = '\0';
            snprintf(rec->trackId, sizeof(rec->trackId), "%s", line + 6);
        } else if (strncmp(line, "trackHash=", 10) == 0) {
            rec->trackHash = (uint32_t)strtoul(line + 10, NULL, 16);
        } else if (strncmp(line, "car=", 4) == 0) {
            line[strcspn(line, "\n")] = '\0';
            snprintf(rec->carId, sizeof(rec->carId), "%s", line + 4);
        } else if (strncmp(line, "carHash=", 8) == 0) {
            rec->carHash = (uint32_t)strtoul(line + 8, NULL, 16);
        } else if (strncmp(line, "setupHash=", 10) == 0) {
            rec->setupHash = (uint32_t)strtoul(line + 10, NULL, 16);
        } else if (strncmp(line, "bestLap=", 8) == 0) {
            rec->bestLapTimeS = (float)strtod(line + 8, NULL);
        } else if (strncmp(line, "frames=", 7) == 0) {
            frames = (int)strtol(line + 7, NULL, 10);
            if (frames <= 0 || frames > REPLAY_CAPACITY_TICKS) {
                haveAll = false;
                break;
            }
        } else if (strcmp(line, "BEGIN\n") == 0) {
            break;
        }
    }
    if (!haveAll || frames <= 0) {
        fclose(f);
        return false;
    }

    /* Initial vehicle snapshot. */
    char ivLine[128];
    if (fgets(ivLine, sizeof(ivLine), f) == NULL) {
        fclose(f);
        return false;
    }
    ReplayVehicleSnapshot *iv = &rec->replay.initialVehicle;
    char *p = ivLine + 3; /* skip "IV " */
    char *end = NULL;
    const unsigned long version = strtoul(p, &end, 10);
    if (end == p) {
        fclose(f);
        return false;
    }
    p = end;
    const unsigned long hash = strtoul(p, &end, 10);
    if (end == p) {
        fclose(f);
        return false;
    }
    p = end;
    const unsigned long valid = strtoul(p, &end, 10);
    if (end == p) {
        fclose(f);
        return false;
    }
    p = end;
    const long setupBytes = strtol(p, &end, 10);
    if (end == p || (size_t)setupBytes != sizeof(iv->setup)) {
        fclose(f);
        return false;
    }
    iv->definitionVersion = version;
    iv->definitionHash = hash;
    iv->valid = (valid != 0);
    if (!iv->valid) {
        fclose(f);
        return false;
    }
    if (fread(&iv->setup, sizeof(iv->setup), 1, f) != 1 ||
        fread(&iv->vehicle, sizeof(iv->vehicle), 1, f) != 1 ||
        fread(&iv->renderState, sizeof(iv->renderState), 1, f) != 1 ||
        fread(&iv->autoTrans, sizeof(iv->autoTrans), 1, f) != 1 ||
        fread(&iv->vehicleControls, sizeof(iv->vehicleControls), 1, f) != 1 ||
        fread(&iv->fuelKg, sizeof(iv->fuelKg), 1, f) != 1 ||
        fread(iv->tireState, sizeof(iv->tireState), 1, f) != 1 ||
        fread(&iv->damage, sizeof(iv->damage), 1, f) != 1 ||
        fread(&iv->crashLockoutTimerS, sizeof(iv->crashLockoutTimerS), 1, f) != 1) {
        fclose(f);
        return false;
    }
    snprintf(iv->definitionId, sizeof(iv->definitionId), "%s", rec->carId);

    /* Frames. */
    for (int i = 0; i < frames; i++) {
        float steer, throttle, brake, handbrake;
        uint8_t oneshot = 0;
        if (fread(&steer, sizeof(steer), 1, f) != 1 ||
            fread(&throttle, sizeof(throttle), 1, f) != 1 ||
            fread(&brake, sizeof(brake), 1, f) != 1 ||
            fread(&handbrake, sizeof(handbrake), 1, f) != 1 ||
            fread(&oneshot, sizeof(oneshot), 1, f) != 1) {
            fclose(f);
            return false;
        }
        const int idx = (rec->replay.head + i) % REPLAY_CAPACITY_TICKS;
        rec->replay.frames[idx].steer = steer;
        rec->replay.frames[idx].throttle = throttle;
        rec->replay.frames[idx].brake = brake;
        rec->replay.frames[idx].handbrake = handbrake;
        rec->replay.frames[idx].oneshotBits = oneshot;
    }
    rec->replay.count = frames;
    rec->replay.firstTick = 0;
    rec->replay.overwrittenTicks = 0;
    rec->replay.mode = REPLAY_MODE_IDLE;
    rec->replay.playbackCursor = 0;

    fclose(f);
    return rec->schema == GHOST_SCHEMA_VERSION;
}

bool ghost_store_remove(const char *path)
{
    if (path == NULL) return false;
    return remove(path) == 0;
}
