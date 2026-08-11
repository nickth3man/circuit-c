/*
 * collision_world.h — one deterministic collision world for static shapes and dynamic bodies.
 *
 * Issue 26: static barrier/object shapes and dynamic body proxies live in one place with
 * stable ids and collision layers, the swept vehicle-track narrowphase runs against the
 * shapes through a common candidate query, and physical contacts are recorded as data for
 * presentation to consume rather than being sniffed out of simulation state.
 *
 * DETERMINISM IS THE CONTRACT. Every shape has a stable id (its insertion index), every body
 * a caller-chosen nonzero id (an EntrantId in a session), and every candidate list, contact
 * list, and resolution pass is ordered by those ids. Nothing here iterates a hash table or
 * follows a pointer, so two worlds built from the same definition produce the same query
 * answers and the same contact order on every platform.
 *
 * THE WORLD IS PLAIN DATA. It owns no heap memory, so it needs no init/free pair beyond
 * memset/zeroing, cannot leak, and survives a hot reload as inert storage. The static-shape
 * array and the broadphase cells are fixed-size bounds, not growth policies: a track larger
 * than the shape cap is refused at build time rather than silently losing barriers (the
 * track format's validators are the enforcement point for authored content).
 *
 * OWNERSHIP. Per docs/SIMULATION_OWNERSHIP.md, the world is a rebuildable cache derived only
 * from the immutable TrackDefinition, so it lives inside TrackRuntime and is rebuilt by
 * track_runtime_bind() whenever the bound definition changes. Rebuilding it deterministically
 * is why it is excluded from the rolling checksum: its contents cannot change results.
 *
 * LAYERS. Each shape and body carries a bitmask layer. A body's `mask` names the layers its
 * narrowphase tests against; a body whose mask excludes a layer never sees those shapes.
 *
 * This translation unit calls no raylib function; it uses the Vector2 type from raylib.h.
 */
#ifndef CIRCUIT_COLLISION_WORLD_H
#define CIRCUIT_COLLISION_WORLD_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"

/* Layer names. Vehicle-vs-vehicle contact (issue 27) will pair bodies on
 * COLLISION_LAYER_VEHICLE_BODY; today no response consumes it. */
#define COLLISION_LAYER_STATIC_BARRIER 0x1u
#define COLLISION_LAYER_VEHICLE_BODY 0x2u

/* Stable ids. Shape ids are 0-based insertion indices; a body id is caller-chosen and must
 * be nonzero (the session uses EntrantId, whose "none" value is 0). */
typedef uint32_t CollisionShapeId;
typedef uint32_t CollisionBodyId;
#define COLLISION_SHAPE_ID_NONE UINT32_MAX

/* Shape capacity: 2048 static segments = a 1024-node centreline (both barrier sides), the
 * current largest track is 170 nodes. This is the content bound the track validators must
 * stay inside, not a size the world grows toward. */
#define COLLISION_WORLD_MAX_STATIC_SHAPES 2048
/* Body capacity: matches RACE_MAX_ENTRANTS. */
#define COLLISION_WORLD_MAX_BODIES 8
/* Per-tick physical contact event feed. Resolution never depends on this buffer — it stops
 * recording, sets contactsOverflowed, and keeps resolving — so it is a presentation feed,
 * not simulation state. */
#define COLLISION_WORLD_MAX_CONTACTS 256

/* Uniform-grid broadphase bounds. Cell count adapts to the track extent (cell size grows so
 * the grid always fits these bounds), and the entry cap is sized so that a segment spans at
 * most four cells: cell size is forced to at least the longest segment, so a segment touches
 * at most 2x2 cells. */
#define COLLISION_WORLD_GRID_MAX_COLS 64
#define COLLISION_WORLD_GRID_MAX_ROWS 64
#define COLLISION_WORLD_GRID_MAX_CELLS \
    (COLLISION_WORLD_GRID_MAX_COLS * COLLISION_WORLD_GRID_MAX_ROWS)
#define COLLISION_WORLD_GRID_MAX_ENTRIES (COLLISION_WORLD_MAX_STATIC_SHAPES * 4)
/* Smallest allowed grid cell, metres. */
#define COLLISION_WORLD_GRID_CELL_MIN_M 4.0f

/* A world uses the grid only when it holds at least this many shapes; below it the single
 * ascending scan over the shape array is the cheaper candidate generator. The value is
 * chosen from the measured crossover in the collision-broadphase scenario, not from taste. */
#define COLLISION_WORLD_GRID_MIN_SHAPES 128

/* One static barrier/object: a segment with a push normal. The push normal points away from
 * the barrier into free space and is unit length; the response math assumes both. */
typedef struct {
    Vector2 aM;          /* segment start, world metres */
    Vector2 bM;          /* segment end, world metres */
    Vector2 pushNormalM; /* unit vector pointing away from the barrier */
    Vector2 minM;        /* segment AABB, cached at insertion */
    Vector2 maxM;
    uint32_t layer;
} CollisionStaticShape;

/* One dynamic body proxy: a capsule (two circles on the body X axis) swept between two poses.
 * The narrowphase and response consume this representation; the physics state it represents
 * stays with the entrant that owns it. */
typedef struct {
    CollisionBodyId id; /* stable; nonzero (an EntrantId in a session) */
    uint32_t layer;
    uint32_t mask;    /* layers this body tests against */
    float cgToFrontM; /* front circle distance ahead of the CG, metres */
    float cgToRearM;  /* rear circle distance behind the CG, metres */
    float radiusM;    /* circle radius = body half width, metres */
    Vector2 prevPosM; /* swept start pose, world metres */
    Vector2 currPosM; /* swept end pose, world metres */
    float prevHdgRad;
    float currHdgRad;
} CollisionBody;

/* One physical contact this tick, in resolution order (ascending body id, then ascending
 * shape id). Presentation — audio, particles, later damage/penalty rules — consumes the
 * feed after the collision stage; nothing in the simulation reads it back. */
typedef struct {
    CollisionBodyId bodyId;
    CollisionShapeId shapeId;
    Vector2 pointM;         /* contact point, world metres */
    Vector2 normalM;        /* push normal, points away from the barrier */
    float approachSpeedMps; /* positive = approaching; <= 0 = separating */
} CollisionContact;

/* The world. Zero-initialised (calloc, or collision_world_init) is a valid empty world. */
typedef struct {
    /* Static shapes; stable id = array index. */
    CollisionStaticShape shapes[COLLISION_WORLD_MAX_STATIC_SHAPES];
    int shapeCount;

    /* Broadphase. gridDirty means shapes changed since the grid was built; a query rebuilds
     * the grid before answering. gridEnabled is chosen at build time from measured workloads
     * (COLLISION_WORLD_GRID_MIN_SHAPES) and can be forced by a test to prove the two paths
     * agree. */
    bool gridDirty;
    bool gridEnabled;
    float gridOriginXM;
    float gridOriginYM;
    float gridCellSizeM;
    int gridCols;
    int gridRows;
    int32_t gridCellStart[COLLISION_WORLD_GRID_MAX_CELLS + 1];
    CollisionShapeId gridEntries[COLLISION_WORLD_GRID_MAX_ENTRIES];
    uint32_t queryEpoch; /* stamp counter; wraps with a full stamp reset */
    uint32_t queryStamps[COLLISION_WORLD_MAX_STATIC_SHAPES];

    /* Dynamic bodies, packed in ascending id order (insertion keeps the sort). */
    CollisionBody bodies[COLLISION_WORLD_MAX_BODIES];
    int bodyCount;

    /* Per-tick physical contact event feed. */
    CollisionContact contacts[COLLISION_WORLD_MAX_CONTACTS];
    int contactCount;
    bool contactsOverflowed;

    /* Per-query candidate scratch, sized to the shape cap so an internal query never
     * truncates. */
    CollisionShapeId queryScratch[COLLISION_WORLD_MAX_STATIC_SHAPES];
} CollisionWorld;

/* Zero the world. A zeroed world is valid and empty; call before first use when the storage
 * was not calloc'd. */
void collision_world_init(CollisionWorld *world);

/*
 * Add one static segment, returning its stable id (insertion index) or -1 when the segment
 * is rejected: world full, non-finite coordinates, a push normal that is not unit length
 * (the impulse math assumes it), or a zero-length segment. Marks the broadphase dirty, so a
 * later query rebuilds it deterministically.
 */
int collision_world_add_static_segment(CollisionWorld *world, Vector2 aM, Vector2 bM,
                                       Vector2 pushNormal, uint32_t layer);

/*
 * Rebuild the broadphase from the current shapes when the grid is dirty. Idempotent and
 * cheap when clean. Called automatically by queries; public so a test can force the build
 * and then flip gridEnabled to compare paths. Returns true when the world holds shapes.
 */
bool collision_world_finalize(CollisionWorld *world);

/*
 * Candidate shapes whose AABBs overlap the query box, are in `layerMask`, and have an id
 * strictly greater than `afterId`, returned in ascending id order. `afterId` is the
 * single-pass continuation token the swept narrowphase uses to re-query after a contact
 * pushes the body without revisiting shapes it has already passed. `idsOut` must hold at
 * least `capacity` entries; pass a capacity of at least the shape count for a complete
 * answer. A non-finite query box returns no candidates (the narrowphase would find no
 * penetration anyway).
 */
int collision_world_query_static(CollisionWorld *world, Vector2 minM, Vector2 maxM,
                                 uint32_t layerMask, CollisionShapeId afterId,
                                 CollisionShapeId *idsOut, int capacity);

/* Clear the dynamic bodies and the contact feed for a new tick. Bodies are per-tick proxies:
 * each collision stage re-registers its entrants' swept poses. */
void collision_world_begin_tick(CollisionWorld *world);

/*
 * Register one dynamic body, keeping the storage packed in ascending id order. Rejects a
 * zero id, a duplicate id, and a full world. The order two bodies are added in cannot change
 * the order they are resolved in.
 */
bool collision_world_add_body(CollisionWorld *world, const CollisionBody *body);

/* Index of the stored body with this id, or -1. */
int collision_world_find_body(const CollisionWorld *world, CollisionBodyId id);

#endif /* CIRCUIT_COLLISION_WORLD_H */
