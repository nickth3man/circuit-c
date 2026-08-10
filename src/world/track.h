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
#define TRACK_MAX_GRID_SLOTS 32
#define TRACK_MAX_SECTOR_MARKERS 16
#define TRACK_MAX_SERVICE_BOXES 16
#define TRACK_GRID_MIN_SPACING_M 3.0f

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

/* Sector split: independent of route validation. A car crossing a sector marker records
 * intermediate timing without affecting lap validity. Forward-only, same hysteresis as
 * route gates. */
typedef struct {
    Vector2 centerM;
    Vector2 forwardUnit;
    float halfWidthM;
} SectorMarker;

/* Start/finish line: controls lap timing. When present it is the authoritative lap
 * boundary; otherwise gate 0 of the route is used. Authored separately so lap validation
 * and timing are independent. */
typedef struct {
    Vector2 centerM;
    Vector2 forwardUnit;
    float halfWidthM;
} StartFinishLine;

/* Deterministic grid slot: where a car starts. Heading is radians CCW from +X. */
typedef struct {
    Vector2 positionM;
    float headingRad;
} GridSlot;

/* Pit lane markers: each is a forward-gated line like a checkpoint but with distinct
 * semantics. The track only authors the geometry; RaceSession decides whether to enforce
 * pit rules. */
typedef struct {
    Vector2 centerM;
    Vector2 forwardUnit;
    float halfWidthM;
} PitGate;

typedef struct {
    Vector2 minM;
    Vector2 maxM;
} ServiceBox;

/* What track_update_checkpoints() observed this tick. `crossed` is what the function used to
 * return as a bare bool; the rest is the detail telemetry needs to explain an invalid lap. */
typedef struct {
    bool crossed;      /* some gate was crossed this tick */
    int index;         /* which gate; -1 when nothing was crossed */
    bool outOfOrder;   /* it was not the gate the car was supposed to take next */
    bool lapCompleted; /* the crossing closed a lap */
    float lapTimeS;    /* the completed lap's time; meaningful only when lapCompleted */
} TrackCheckpointEvent;

/* Sector crossing event, independent of lap validation. */
typedef struct {
    bool crossed;
    int index;         /* which sector marker; -1 when none */
    float sectorTimeS; /* time since last sector boundary */
} TrackSectorEvent;

/* Everything one entrant's progress stage produced this tick. See track_update_progress(). */
typedef struct {
    TrackCheckpointEvent checkpoint;
    TrackSectorEvent sector;
    bool wrongWay;        /* the latched state after this tick */
    bool wrongWayChanged; /* it differs from what it was before this tick */
} TrackProgressEvent;

/* ServiceBox crossing not currently gated through a dedicated event; the session can query
 * containment directly via track_point_in_service_box(). */

/*
 * Authored track content. Immutable once loaded, and shared by every entrant in a session:
 * nothing here may be written while a race is running. It owns no lap cursor and no timer,
 * which is what lets two cars read one definition without disturbing each other.
 */
typedef struct {
    TrackNode *nodes; /* heap-allocated, survives reload (plain heap, not module static) */
    int count;
    Checkpoint *
        checkpoints; /* heap-allocated, ordered; index 0 is start/finish when no explicit start/finish */
    int checkpointCount;
    bool routeClosed; /* true = closed circuit (lap wraps), false = open point-to-point */
    SurfaceId offTrackSurfaceId; /* surface returned beyond the runoff band */
    SurfaceId runoffSurfaceId;   /* surface between halfWidthM and runoffHalfWidthM */
    /* Parking lot mode: rectangular open area instead of laned road. */
    bool isParkingLot;
    float lotMinXM, lotMaxXM, lotMinYM, lotMaxYM;
    /* Distributed markers: each owned by the definition, shared read-only by every entrant. */
    SectorMarker *sectorMarkers;
    int sectorMarkerCount;
    bool hasStartFinish;
    StartFinishLine startFinish;
    GridSlot *gridSlots;
    int gridSlotCount;
    bool hasPitEntry;
    PitGate pitEntry;
    bool hasPitExit;
    PitGate pitExit;
    bool hasPitSpeedLine;
    PitGate pitSpeedLine;
    ServiceBox *serviceBoxes;
    int serviceBoxCount;
    /* Identity, for telemetry and run metadata. Fixed arrays, never pointers: see the header
     * comment. `version` changes whenever the geometry changes, so a run recorded against an
     * older shape is identifiable rather than silently comparable. */
    char id[TRACK_ID_CHARS];
    char version[TRACK_VERSION_CHARS];
} TrackDefinition;
/*
 * Where one entrant is on the route, expressed in the route's own frame.
 *
 * This is the single localization contract issue #38 exists to define: timing, wrong-way,
 * race distance, AI and camera all read these fields rather than each re-deriving "which bit
 * of track am I on" from a private nearest-segment scan that answers slightly differently.
 *
 * RECOVERY CANDIDATE. `pointM` and `forwardUnit` are the pose a car rejoining the route should
 * be put back at, and `lateralM`'s sign says which side it left from. There is no separate
 * recovery function because there is nothing left to compute: the localization already found
 * the point.
 *
 * `valid` is false when the track has no usable centreline (no nodes, a single node, or an
 * open route with fewer than two). Every other field is then zero, so a consumer that forgets
 * to check reads "start of the route" rather than uninitialised memory.
 */
typedef struct {
    bool valid;
    int segmentIndex; /* centreline segment i: nodes[i] -> nodes[i+1] (wrapping when closed) */
    float segmentT;   /* [0, 1] along that segment */
    Vector2 pointM;   /* the closest centreline point, world metres */
    Vector2 forwardUnit; /* that segment's travel direction, unit length */
    float longitudinalM; /* arc length from node 0 to pointM along the route */
    /* Signed offset from the centreline, positive LEFT of forwardUnit. This says which SIDE of
     * the route the car is on. It is NOT the distance to the route and must not be used as one:
     * where the closest point is a clamped segment endpoint the displacement can be almost
     * entirely longitudinal, leaving this near zero for a pose fifty metres past the end of an
     * open route. Use the distance from pointM for that, as the two fields below do. */
    float lateralM;
    float headingErrorRad; /* car heading minus route heading, wrapped to [-PI, PI) */
    /* How much the geometry supports this being the car's route position: 1 on the racing
     * surface, falling linearly to 0 across the runoff band, 0 at and beyond the barrier. It is
     * a containment measure, not a probability. Measured, like onRoute, from |posM - pointM|. */
    float confidence;
    bool
        onRoute; /* the distance from posM to pointM is within the segment's racing half-width */
} RouteLocation;

/* How far along the route, in metres of arc either side of the previous position, a continuity
 * search looks.
 *
 * A window in METRES rather than in segments, because a segment is not a unit of anything: the
 * authored circuits place nodes a few metres apart and a hand-built ribbon can place them
 * twenty, so a fixed segment count is a different search on every track — and on a short one it
 * wraps far enough to reach the opposing strand it exists to exclude. At 120 Hz nothing in this
 * simulation covers 25 m in a tick, while the two sides of a circuit are always most of a lap
 * apart in arc length. That gap is what makes the window work. */
#define ROUTE_LOCALIZE_WINDOW_M 25.0f

/* How far outside a segment's barrier corridor the car may be and still keep its cached
 * segment. Wide enough that running wide onto the grass and rejoining never re-localizes
 * (which is exactly when a global scan would jump to a parallel section), narrow enough that
 * a teleport, a spawn, or a reset does. */
#define ROUTE_LOCALIZE_ACCEPT_MARGIN_M 5.0f

/* Wrong-way hysteresis. The car must be pointing more than ENTER away from the route AND
 * losing longitudinal ground for HOLD_S continuously before the flag sets, and must spend the
 * same HOLD_S not doing so before it clears. Between those the previous answer stands, which
 * is what stops a spin from strobing the flag. EXIT is deliberately narrower than ENTER so the
 * angular test itself has a deadband too. */
#define ROUTE_WRONG_WAY_ENTER_RAD 2.0943951f /* 120 degrees */
#define ROUTE_WRONG_WAY_EXIT_RAD 1.5707963f  /* 90 degrees */
#define ROUTE_WRONG_WAY_HOLD_S 0.75f
/* Longitudinal movement per tick below this counts as "not going anywhere" rather than as
 * backwards progress, so a stationary car cannot arm the flag on float noise. */
#define ROUTE_WRONG_WAY_BACKWARD_EPS_M 0.001f

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
    /* Sector timing: independent of route validation. */
    int nextSector;        /* index of the next sector marker */
    float sectorTimerS;    /* time since last sector boundary */
    float lastSectorTimeS; /* time of the most recently completed sector */
    bool lapInvalid;       /* true if a required checkpoint was skipped (outOfOrder) */
    bool
        lapArmed; /* SF latch: set when the start/finish line is crossed, cleared when a lap closes */
    bool routeFinished;   /* open point-to-point: true once final checkpoint crossed */
    int lastCrossedIndex; /* debounce: last checkpoint index crossed, -1 initially */
    int ticksSinceCross;  /* hysteresis ticks since last crossing */
    /*
     * Route localization (issue #38). Per entrant, never shared: two cars on the same track
     * localize independently, which is what stops one recovering car from moving another's
     * route cursor.
     *
     * `location` is BOTH this tick's answer and next tick's continuity hint, so it is
     * authoritative state and not a rebuildable cache: rebuilding it with a global scan can
     * legitimately produce a different segment where the route runs beside itself, and
     * docs/SIMULATION_OWNERSHIP.md says a "cache" that changes a result is authoritative.
     */
    RouteLocation location;
    /* Signed route distance accumulated from the per-tick longitudinal delta, so it keeps
     * rising across the start/finish seam instead of sawtoothing back to zero, and falls again
     * when a car goes backwards. This is the ordering key: entrants sort by it, descending,
     * with ascending EntrantId as the tie-break the roster already guarantees. */
    float raceDistanceM;
    bool wrongWay;        /* latched by the hysteresis below, not by this tick's angle */
    float wrongWayTimerS; /* [0, ROUTE_WRONG_WAY_HOLD_S]; the hysteresis integrator */
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

/* Independent sector timing: advance ONE racer's sector state. Sectors do not affect
 * lap validity and are ordered independently of route checkpoints. */
TrackSectorEvent track_update_sectors(const TrackDefinition *track, RacerProgress *progress,
                                      Vector2 prevPosM, Vector2 currPosM);

/* --------------- route localization (issue #38, src/world/route_localization.c) ----------- */

/*
 * Localize by scanning every segment. This is the reference answer: no history, no hint, and
 * therefore no way for it to be wrong about which lap the car is on when the route runs beside
 * itself. Exact ties go to the LOWEST segment index, so two equidistant candidates at a
 * crossing resolve the same way on every platform and in every run.
 *
 * `headingRad` only fills in headingErrorRad; it never influences which segment is chosen.
 */
RouteLocation route_localize_global(const TrackDefinition *track, Vector2 posM,
                                    float headingRad);

/*
 * Localize preferring continuity with a previous answer.
 *
 * A NULL, invalid, or out-of-range `previous` means "no prior" and the call degenerates to
 * route_localize_global(). Otherwise only the segments within ROUTE_LOCALIZE_WINDOW_M of arc
 * length either side of `previous` are candidates, and the nearest of THOSE wins. Excluding the
 * rest of the route is the entire mechanism: a figure-eight crossing, a hairpin, and a pit lane
 * beside the main straight are all cases where the geometrically nearest segment belongs to a
 * part of the lap the car demonstrably was not on a tick ago.
 *
 * Within the window, distance decides and arc proximity only breaks ties: candidates are
 * visited as previous, +1, -1, +2, -2, ... under a strict `<`, so an exact tie goes to the
 * candidate nearest the previous position and, at equal remove, to the one ahead — a car moves
 * forwards. Distance must stay primary, because a continuity search that preferred the closest
 * arc position outright would clamp to the end of its old segment and never advance past a node.
 *
 * The global scan is the bounded fallback: it runs at most once, and only when the best
 * windowed candidate is further off than ROUTE_LOCALIZE_ACCEPT_MARGIN_M beyond that segment's
 * barrier corridor — that is, when continuity has stopped being credible at all, which is a
 * teleport, a spawn, or a reset rather than anything a driver can do.
 */
RouteLocation route_localize_near(const TrackDefinition *track, const RouteLocation *previous,
                                  Vector2 posM, float headingRad);

/*
 * THE per-entrant progress stage. Localizes from this racer's cached prior location, advances
 * its route gates and its sector gates, then updates wrong-way and race distance from the same
 * localization — one call, one contract, one place where a tick's route facts are decided.
 *
 * `currPosM`/`headingRad` are the car's authoritative pose at the END of the tick and
 * `prevPosM` is where it started, because a gate is crossed by a swept segment rather than by
 * a point. `dt` is used only by the wrong-way hysteresis; the lap and sector timers stay with
 * the caller, which advances them after reading the returned times.
 *
 * The definition is read-only. Two entrants may call this against one track in the same tick
 * without touching each other's answer.
 *
 * Gate crossing itself is still the exact swept-line test in track_update_checkpoints(): a
 * localization cannot make an intersection test more correct than it already is. What it adds
 * is the route frame that wrong-way, race distance and rejoin candidates need, established
 * before the gates are judged so every consumer of this tick sees one position.
 */
TrackProgressEvent track_update_progress(const TrackDefinition *track, RacerProgress *progress,
                                         Vector2 prevPosM, Vector2 currPosM, float headingRad,
                                         float dt);

/* Grid validation: deterministic, order-independent. Returns false and writes a reason
 * when slots overlap or lie off the racing surface. */
bool track_validate_grid_slots(const TrackDefinition *track, char *error, size_t errorCap);
bool track_grid_slot_pose(const TrackDefinition *track, int slotIndex, Vector2 *positionM,
                          float *headingRad);

/* Pit geometry queries: the track only authors these; RaceSession decides whether to
 * enforce pit rules. */
bool track_pit_has_geometry(const TrackDefinition *track);
bool track_point_in_service_box(const TrackDefinition *track, Vector2 pointM);

/* Open/closed route semantics. */
bool track_is_closed(const TrackDefinition *track);
bool track_is_open(const TrackDefinition *track);

/* Session-mode marker requirements: does this track have the geometry a mode needs? */
bool track_has_required_markers_for_mode(const TrackDefinition *track, const char *mode,
                                         char *error, size_t errorCap);

#endif /* CIRCUIT_TRACK_H */
