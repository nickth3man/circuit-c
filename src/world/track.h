/*
 * track.h — track geometry, surface bands, ordered lap checkpoints, and per-racer progress.
 *
 * THREE TYPES, THREE OWNERS. Authored geometry and one racer's lap cursor used to share a
 * single `Track` struct, which meant a track could only ever serve one car. They are now
 * separate: an immutable TrackDefinition shared by every entrant, one RacerProgress per
 * entrant, and a TrackRuntime for state that is mutable but session-wide rather than
 * racer-specific. See docs/SIMULATION_OWNERSHIP.md.
 *
 * A TrackDefinition owns two heap-allocated arrays: a centreline of TrackNode entries and an
 * ordered array of Checkpoint gates. Both survive hot reloads because the memory is
 * heap-allocated and the Game block is platform-owned. Never point either at module static
 * data, and never store a `const char *` in this struct for the same reason — the id and
 * version below are fixed char arrays precisely because the definition lives inside Game.
 *
 * THREE SURFACE BANDS. A node describes the racing surface out to halfWidthM, a runoff band
 * from there out to runoffHalfWidthM, and off-track beyond. Barriers stand at the runoff
 * edge, so leaving the racing surface is a recoverable mistake that costs grip rather than an
 * instant wall strike. When runoffHalfWidthM <= halfWidthM there is no runoff band and the
 * barrier sits on the track edge, which is the behaviour every track had before runoff
 * existed — so a node built without the field keeps working unchanged.
 *
 * CHECKPOINTS ARE EXPLICIT. Gates are their own ordered array rather than being implied by
 * the centreline nodes, because a lap needs far fewer gates than a curve needs nodes, and
 * because the gate a lap is validated against should not silently change when someone
 * refines the geometry. Index 0 is the start/finish by convention.
 *
 * This translation unit calls no raylib function; Vector2 from the header is fine.
 * SurfaceId is defined in vehicle.h.
 */
#ifndef CIRCUIT_TRACK_H
#define CIRCUIT_TRACK_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"          /* Vector2 */
#include "physics/vehicle.h" /* SurfaceId */

#define TRACK_ID_CHARS 32
#define TRACK_VERSION_CHARS 16

typedef struct {
    Vector2 centerM;     /* authored centreline point, world meters */
    float halfWidthM;    /* racing-surface half-width, meters */
    SurfaceId surfaceId; /* surface inside this segment */
    /* Barrier distance from the centreline, meters. <= halfWidthM means "no runoff band":
     * the barrier stands on the track edge. Deliberately LAST in the struct so the existing
     * positional initialisers `{ {x,y}, hw, surface }` keep meaning what they always did. */
    float runoffHalfWidthM;
} TrackNode;

typedef struct {
    Vector2 centerM;     /* gate midpoint, world meters */
    Vector2 forwardUnit; /* the direction the car must be travelling to score this gate */
    float halfWidthM;    /* gate half-length, perpendicular to forwardUnit */
    bool required;       /* a lap is invalid unless every required gate was taken, in order */
} Checkpoint;

/* What track_update_checkpoints() observed this tick. `crossed` is what the function used to
 * return as a bare bool; the rest is the detail telemetry needs to explain an invalid lap. */
typedef struct {
    bool crossed;      /* some gate was crossed this tick */
    int index;         /* which gate; -1 when nothing was crossed */
    bool outOfOrder;   /* it was not the gate the car was supposed to take next */
    bool lapCompleted; /* the crossing closed a lap */
    float lapTimeS;    /* the completed lap's time; meaningful only when lapCompleted */
} TrackCheckpointEvent;

/*
 * Authored track content. Immutable once loaded, and shared by every entrant in a session:
 * nothing here may be written while a race is running. It owns no lap cursor and no timer,
 * which is what lets two cars read one definition without disturbing each other.
 */
typedef struct {
    TrackNode *nodes; /* heap-allocated, survives reload (plain heap, not module static) */
    int count;
    Checkpoint *checkpoints; /* heap-allocated, ordered; index 0 is start/finish */
    int checkpointCount;
    SurfaceId offTrackSurfaceId; /* surface returned beyond the runoff band */
    SurfaceId runoffSurfaceId;   /* surface between halfWidthM and runoffHalfWidthM */
    /* Parking lot mode: rectangular open area instead of laned road. */
    bool isParkingLot;
    float lotMinXM, lotMaxXM, lotMinYM, lotMaxYM;
    /* Identity, for telemetry and run metadata. Fixed arrays, never pointers: see the header
     * comment. `version` changes whenever the geometry changes, so a run recorded against an
     * older shape is identifiable rather than silently comparable. */
    char id[TRACK_ID_CHARS];
    char version[TRACK_VERSION_CHARS];
} TrackDefinition;

/*
 * One racer's position around the route. Exactly one per entrant, never shared, and the only
 * thing a checkpoint crossing writes. A zeroed RacerProgress is a valid "start of an out-lap
 * from gate 0" state, so a caller that calloc's one does not have to initialise it.
 */
typedef struct {
    int nextCheckpoint;     /* index of the next gate the car must cross */
    int lap;                /* completed laps */
    int lapStartCheckpoint; /* gate whose crossing closes one lap for this run */
    float lapTimerS;        /* seconds elapsed since the last checkpoint/lap */
    float lastLapTimeS;     /* time of the most recently completed lap */
} RacerProgress;

/*
 * Mutable track state that belongs to the session rather than to any one racer.
 *
 * Today it holds only the hash of the definition it was bound to, which is what lets a test
 * prove the shared geometry was not written during a session. Deterministic weather/wetness
 * and derived query caches join it later; a cache that differs per racer belongs in that
 * entrant's RacerProgress instead, not here.
 */
typedef struct {
    /* track_geometry_hash() of the definition bound at session start. */
    uint32_t definitionHash;
} TrackRuntime;

/*
 * Barrier distance from the centreline for this node.
 *
 * A node whose runoff is not wider than its racing surface has no runoff band, and its
 * barrier stands on the track edge — the behaviour every node had before the field existed,
 * which is what keeps a hand-built ribbon working without being rewritten. Inline in the
 * header because the surface query and the collision solver must agree on it exactly.
 */
static inline float track_node_barrier_half_width(const TrackNode *node)
{
    if (node == NULL) return 0.0f;
    return (node->runoffHalfWidthM > node->halfWidthM) ? node->runoffHalfWidthM
                                                       : node->halfWidthM;
}

void track_init(TrackDefinition *track); /* allocate + populate the parking lot */
void track_free(TrackDefinition *track); /* free arrays, zero the struct */

/*
 * Bind a runtime to the definition a session is about to race on, recording its geometry
 * hash. Call after the definition is loaded and before the first fixed update.
 */
void track_runtime_bind(TrackRuntime *runtime, const TrackDefinition *track);

/*
 * True when `track` still hashes to what `runtime` recorded at bind time — that is, when the
 * shared definition was not mutated during the session. This is the immutability check the
 * ownership split exists to make possible.
 */
bool track_runtime_definition_unchanged(const TrackRuntime *runtime,
                                        const TrackDefinition *track);

/* The chicane validation circuit: two straights joined by 180-degree curves, with a
 * left-right chicane set into the far straight. Closed loop, 8 required gates, gate 0 the
 * start/finish. This is the track Milestone 1 validates every car against. */
void track_load_chicane(TrackDefinition *track);
/* A second authored layout for multi-track AI validation. It preserves the checkpoint contract
 * while changing the stadium proportions and chicane displacement. */
void track_load_sprint(TrackDefinition *track);
/* A tighter technical layout derived from the authored chicane with shorter radii and narrower runoff. */
void track_load_technical(TrackDefinition *track);

/* Derive one gate per centreline node, forward-facing and spanning the node width — the
 * implicit scheme the checkpoint code used before gates became explicit data. Lets a caller
 * that hand-builds a node ribbon get lap validation without authoring gates by hand. */
bool track_build_checkpoints_from_nodes(TrackDefinition *track);

/*
 * Put one racer's progress back to the start of an out-lap at `startCheckpointIndex`: no laps
 * completed, timers zeroed, and the next required gate set to the one after it, because a
 * standing start places the car ON its gate and it must not score that gate without driving a
 * lap. Reads the definition only for its gate count; resetting one racer cannot touch another.
 */
void track_reset_progress_at(RacerProgress *progress, const TrackDefinition *track,
                             int startCheckpointIndex);

/* Where a standing start puts the car: the start/finish gate's midpoint, facing along its
 * forward direction. Returns false (and writes nothing) when the track has no gates. */
bool track_start_pose(const TrackDefinition *track, Vector2 *positionM, float *headingRad);
/* Start pose at an arbitrary checkpoint, facing that gate's forward direction. */
bool track_start_pose_at(const TrackDefinition *track, int checkpointIndex, Vector2 *positionM,
                         float *headingRad);

/* FNV-1a over the node and checkpoint arrays. Two tracks with the same hash have the same
 * shape, so a run's metadata can prove which geometry produced it even if `version` was not
 * bumped after an edit. */
uint32_t track_geometry_hash(const TrackDefinition *track);

/* Total centreline length, meters. */
float track_length_m(const TrackDefinition *track);

/* Surface under a world point. `runtime` carries the session-wide environment the query will
 * consult once wetness exists; it is accepted now so the call sites do not move again, and
 * NULL is valid and means "definition only". */
SurfaceId Track_SurfaceAt(const TrackDefinition *track, const TrackRuntime *runtime,
                          Vector2 pointM);

/* Distance from pointM to the nearest centreline segment, in metres.
 * Returns 0.0f if track is NULL or has no nodes. Optionally writes
 * the half-width of that segment to *halfWidthM when non-NULL. */
float track_distance_to_centerline_m(const TrackDefinition *track, Vector2 pointM,
                                     float *halfWidthM);

/*
 * Advance ONE racer's checkpoint/lap state from that car's movement this tick.
 *
 * prevPosM/currPosM are world meters, the car's position at the start and end of the tick.
 * EVERY gate is tested, not only the expected one, so a car that cuts the course is reported
 * through TrackCheckpointEvent.outOfOrder instead of silently failing to advance. Only the
 * expected gate advances progress. Crossings against a gate's forward direction are ignored,
 * so reversing over a line cannot score it.
 *
 * The definition is read-only: the gates a car is measured against cannot change because a
 * car drove through one, and two entrants may call this against the same definition in the
 * same tick without interacting.
 */
TrackCheckpointEvent track_update_checkpoints(const TrackDefinition *track,
                                              RacerProgress *progress, Vector2 prevPosM,
                                              Vector2 currPosM);

#endif /* CIRCUIT_TRACK_H */
