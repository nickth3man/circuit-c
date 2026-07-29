/*
 * track.h — track geometry and per-wheel surface query.
 *
 * A Track owns a heap-allocated array of TrackNode entries forming a closed centerline.
 * The nodes pointer survives hot reloads because the memory is heap-allocated and the Game
 * block is platform-owned. Never point nodes at module static data.
 *
 * This translation unit calls no raylib function; Vector2 from the header is fine.
 * SurfaceId is defined in vehicle.h.
 */
#ifndef DRIFTY_TRACK_H
#define DRIFTY_TRACK_H

#include "raylib.h"          /* Vector2 */
#include "physics/vehicle.h" /* SurfaceId */

typedef struct {
    Vector2 centerM;     /* centerline point, world meters */
    float halfWidthM;    /* segment half-width, meters */
    SurfaceId surfaceId; /* surface inside this segment */
} TrackNode;

typedef struct {
    TrackNode *nodes; /* heap-allocated, platform-owned, survives reload */
    int count;
    SurfaceId offTrackSurfaceId; /* surface returned when point is off-track */
    /* Parking lot mode: rectangular open area instead of laned road. */
    bool isParkingLot;
    float lotMinXM, lotMaxXM, lotMinYM, lotMaxYM;
    int nextCheckpoint; /* index of the next gate the car must cross */
    int lap;            /* completed laps */
    float lapTimerS;    /* seconds elapsed since the last checkpoint/lap */
    float lastLapTimeS; /* time of the most recently completed lap */
} Track;

void track_init(Track *track); /* allocate + populate the oval */
void track_free(Track *track); /* free nodes, zero the struct */
SurfaceId Track_SurfaceAt(const Track *track, Vector2 pointM);

/* Distance from pointM to the nearest centreline segment, in metres.
 * Returns 0.0f if track is NULL or has no nodes. Optionally writes
 * the half-width of that segment to *halfWidthM when non-NULL. */
float track_distance_to_centerline_m(const Track *track, Vector2 pointM, float *halfWidthM);

/* Advance checkpoint/lap state based on the car crossing a gate this tick.
 * prevPosM/currPosM in world meters, the car's position at start and end of the tick.
 * Returns true if a checkpoint was passed. Forward-only: reverse crossings are ignored. */
bool track_update_checkpoints(Track *track, Vector2 prevPosM, Vector2 currPosM);

#endif /* DRIFTY_TRACK_H */
