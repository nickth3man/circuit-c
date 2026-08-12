/*
 * ghost.h — issue #51: persistent, non-interacting player ghosts.
 *
 * A ghost is a recorded controller stream plus the compatible session/content/setup metadata
 * that proves the recording belongs to this car/track/setup. It is replayed as its OWN
 * vehicle simulation OUTSIDE the race roster: it has no collision body, no rules
 * participation, no events, and it never enters the authoritative checksum — so a ghost's
 * presence cannot change the player car, the AI, weather, classification, contacts, or the
 * checksum of the authoritative participants. It only draws (presentation consumes its pose).
 */
#ifndef CIRCUIT_GHOST_H
#define CIRCUIT_GHOST_H

#include <stdbool.h>
#include <stdint.h>

#include "content/vehicle_manifest.h" /* VEHICLE_CONTENT_ID_CAPACITY */
#include "game/controller.h"
#include "game/replay.h"
#include "physics/vehicle.h"
#include "world/track.h"

#define GHOST_SCHEMA_VERSION 1u
#define GHOST_MAGIC "CIRCUIT-GHOST"

/* One durable ghost recording. Plain value data (fixed arrays), versioned and portable. */
typedef struct {
    uint32_t schema;
    char trackId[TRACK_ID_CHARS];
    uint32_t trackHash; /* track_geometry_hash of the recorded track */
    char carId[VEHICLE_CONTENT_ID_CAPACITY];
    uint32_t carHash;    /* VehicleDefinition.contentHash of the recorded car */
    uint32_t setupHash;  /* vehicle_setup_hash of the recorded setup */
    float bestLapTimeS;  /* the recording's best valid lap, for best-record replacement */
    ReplayBuffer replay; /* the controller stream + initial vehicle snapshot */
} GhostRecording;

/* The live, non-interacting ghost running inside Game. Zeroed state = no ghost. */
typedef struct {
    bool active;
    GhostRecording recording; /* the loaded recording (valid while active) */
    VehicleInstance instance; /* its own vehicle simulation, outside the roster */
    Controller controller;    /* kind = REPLAY while active */
    ControllerOutput controllerOutput;
    RacerProgress progress; /* presentation-only timing display */
} Ghost;

/* Compatibility gate: does this recording belong to the current track/car/setup? Writes an
 * exact player-facing reason on mismatch. */
bool ghost_validate(const GhostRecording *rec, uint32_t trackHash, const char *trackId,
                    const VehicleDefinition *definition, const VehicleSetup *setup,
                    char *reason, size_t reasonCap);

/* Atomic persistence: write via a temporary file + rename so a crash mid-write can never
 * leave a truncated recording where the good one was. */
bool ghost_store_save(const char *path, const GhostRecording *rec);
bool ghost_store_load(const char *path, GhostRecording *rec);
/* Remove the store file. */
bool ghost_store_remove(const char *path);

#endif /* CIRCUIT_GHOST_H */
