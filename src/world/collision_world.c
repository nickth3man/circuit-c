/*
 * collision_world.c — deterministic collision world: static shapes, dynamic body proxies,
 * candidate queries, and the per-tick contact feed.
 *
 * Determinism rules enforced here:
 *   - A shape's stable id is its insertion index; bodies are packed in ascending id order by
 *     insertion; candidates are returned ascending by id (the grid path collects and sorts,
 *     the brute path is a single ascending scan); contacts are recorded in resolution order.
 *   - The uniform grid is a pure function of the shape array: cell size adapts to extent and
 *     longest segment, cells are filled in ascending (row, column) order per shape and shapes
 *     in ascending id order, and per-query dedupe uses a stamp epoch rather than a hash.
 *   - A non-finite query box returns no candidates, which is exactly what the narrowphase
 *     would conclude (no float comparison with NaN is true).
 *
 * This translation unit calls no raylib function; it uses the Vector2 type from raylib.h.
 */

#include "world/collision_world.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "world/track.h" /* TrackDefinition + track_node_barrier_half_width for the builder */

/* ====================================================================================== */
/*  Small helpers                                                                          */
/* ====================================================================================== */

static int compare_shape_ids(const void *a, const void *b)
{
    const CollisionShapeId ia = *(const CollisionShapeId *)a;
    const CollisionShapeId ib = *(const CollisionShapeId *)b;
    return (ia > ib) - (ia < ib);
}

static int clamp_int(int value, int lo, int hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/* Exact AABB overlap; the grid's cells are conservative, so the narrowphase candidate set
 * re-verifies the shape's real AABB against the query box. */
static bool aabb_overlaps(Vector2 minA, Vector2 maxA, Vector2 minB, Vector2 maxB)
{
    return minA.x <= maxB.x && maxA.x >= minB.x && minA.y <= maxB.y && maxA.y >= minB.y;
}

/* Cell index range spanned by a box, clamped to the grid. The grid covers the whole shape
 * AABB plus a cell margin, so every shape sits inside the clamped range. */
static void cell_range(const CollisionWorld *world, Vector2 minM, Vector2 maxM, int *c0,
                       int *r0, int *c1, int *r1)
{
    *c0 = clamp_int((int)floorf((minM.x - world->gridOriginXM) / world->gridCellSizeM), 0,
                    world->gridCols - 1);
    *c1 = clamp_int((int)floorf((maxM.x - world->gridOriginXM) / world->gridCellSizeM), 0,
                    world->gridCols - 1);
    *r0 = clamp_int((int)floorf((minM.y - world->gridOriginYM) / world->gridCellSizeM), 0,
                    world->gridRows - 1);
    *r1 = clamp_int((int)floorf((maxM.y - world->gridOriginYM) / world->gridCellSizeM), 0,
                    world->gridRows - 1);
}

/* ====================================================================================== */
/*  World lifecycle and static shapes                                                      */
/* ====================================================================================== */

void collision_world_init(CollisionWorld *world)
{
    if (world != NULL) memset(world, 0, sizeof(*world));
}

int collision_world_add_static_segment(CollisionWorld *world, Vector2 aM, Vector2 bM,
                                       Vector2 pushNormal, uint32_t layer)
{
    if (world == NULL || world->shapeCount >= COLLISION_WORLD_MAX_STATIC_SHAPES) return -1;
    /* Malformed input is an error, not a silently dropped barrier. */
    if (!isfinite(aM.x) || !isfinite(aM.y) || !isfinite(bM.x) || !isfinite(bM.y) ||
        !isfinite(pushNormal.x) || !isfinite(pushNormal.y)) {
        return -1;
    }
    /* The impulse response assumes a unit push normal; reject anything else up front. */
    const float lenSq = pushNormal.x * pushNormal.x + pushNormal.y * pushNormal.y;
    if (fabsf(lenSq - 1.0f) > 1e-3f) return -1;
    if (aM.x == bM.x && aM.y == bM.y) return -1; /* zero-length segment contributes nothing */

    CollisionStaticShape *shape = &world->shapes[world->shapeCount];
    shape->aM = aM;
    shape->bM = bM;
    shape->pushNormalM = pushNormal;
    shape->layer = layer;
    shape->minM = (Vector2){ fminf(aM.x, bM.x), fminf(aM.y, bM.y) };
    shape->maxM = (Vector2){ fmaxf(aM.x, bM.x), fmaxf(aM.y, bM.y) };
    world->gridDirty = true;
    return world->shapeCount++;
}

/* ====================================================================================== */
/*  Broadphase                                                                            */
/* ====================================================================================== */

/* Build the uniform grid over the current shapes. The cell size adapts so the grid always
 * fits the fixed arrays: at least the longest segment (so a segment spans at most 2x2 cells,
 * which is the entry cap's assumption), and at least the track extent spread over the capped
 * grid (so a huge track degrades to fewer, coarser cells rather than failing). Returns false
 * when the exact entry count would exceed the cap; the caller then falls back to the brute
 * scan, which is correct at every size, only slower. */
static bool build_grid(CollisionWorld *world)
{
    Vector2 minM = { FLT_MAX, FLT_MAX };
    Vector2 maxM = { -FLT_MAX, -FLT_MAX };
    float maxSegLen = 0.0f;
    for (int i = 0; i < world->shapeCount; i++) {
        const CollisionStaticShape *shape = &world->shapes[i];
        minM.x = fminf(minM.x, shape->minM.x);
        minM.y = fminf(minM.y, shape->minM.y);
        maxM.x = fmaxf(maxM.x, shape->maxM.x);
        maxM.y = fmaxf(maxM.y, shape->maxM.y);
        const float segLen = sqrtf((shape->bM.x - shape->aM.x) * (shape->bM.x - shape->aM.x) +
                                   (shape->bM.y - shape->aM.y) * (shape->bM.y - shape->aM.y));
        if (segLen > maxSegLen) maxSegLen = segLen;
    }

    const float extentXM = maxM.x - minM.x;
    const float extentYM = maxM.y - minM.y;
    float cellSizeM = fmaxf(COLLISION_WORLD_GRID_CELL_MIN_M, maxSegLen);
    cellSizeM = fmaxf(cellSizeM, extentXM / (float)COLLISION_WORLD_GRID_MAX_COLS);
    cellSizeM = fmaxf(cellSizeM, extentYM / (float)COLLISION_WORLD_GRID_MAX_ROWS);
    const int cols =
        clamp_int((int)ceilf(extentXM / cellSizeM) + 1, 1, COLLISION_WORLD_GRID_MAX_COLS);
    const int rows =
        clamp_int((int)ceilf(extentYM / cellSizeM) + 1, 1, COLLISION_WORLD_GRID_MAX_ROWS);
    world->gridOriginXM = minM.x;
    world->gridOriginYM = minM.y;
    world->gridCellSizeM = cellSizeM;
    world->gridCols = cols;
    world->gridRows = rows;

    const int cellCount = cols * rows;
    for (int c = 0; c <= cellCount; c++) world->gridCellStart[c] = 0;
    for (int i = 0; i < world->shapeCount; i++) {
        int c0, r0, c1, r1;
        cell_range(world, world->shapes[i].minM, world->shapes[i].maxM, &c0, &r0, &c1, &r1);
        for (int r = r0; r <= r1; r++)
            for (int c = c0; c <= c1; c++) world->gridCellStart[r * cols + c + 1]++;
    }
    for (int c = 0; c < cellCount; c++) world->gridCellStart[c + 1] += world->gridCellStart[c];
    if (world->gridCellStart[cellCount] > COLLISION_WORLD_GRID_MAX_ENTRIES) return false;

    /* Second pass fills the entries: shapes in ascending id order, their cells in ascending
     * (row, column) order, so each cell's list is ascending and the whole fill is a pure
     * function of the shape array. */
    int32_t cursor[COLLISION_WORLD_GRID_MAX_CELLS + 1];
    memcpy(cursor, world->gridCellStart, (size_t)(cellCount + 1) * sizeof(int32_t));
    for (int i = 0; i < world->shapeCount; i++) {
        int c0, r0, c1, r1;
        cell_range(world, world->shapes[i].minM, world->shapes[i].maxM, &c0, &r0, &c1, &r1);
        for (int r = r0; r <= r1; r++)
            for (int c = c0; c <= c1; c++)
                world->gridEntries[cursor[r * cols + c]++] = (CollisionShapeId)i;
    }
    return true;
}

bool collision_world_finalize(CollisionWorld *world)
{
    if (world == NULL) return false;
    if (!world->gridDirty) return world->shapeCount > 0;
    world->gridDirty = false;
    if (world->shapeCount < COLLISION_WORLD_GRID_MIN_SHAPES) {
        world->gridEnabled = false; /* measured: brute scan is cheaper at this size */
        return true;
    }
    world->gridEnabled = build_grid(world);
    return true;
}

/* ====================================================================================== */
/*  Candidate query                                                                        */
/* ====================================================================================== */

int collision_world_query_static(CollisionWorld *world, Vector2 minM, Vector2 maxM,
                                 uint32_t layerMask, CollisionShapeId afterId,
                                 CollisionShapeId *idsOut, int capacity)
{
    if (world == NULL || idsOut == NULL || capacity <= 0) return 0;
    if (world->shapeCount == 0) return 0;
    if (!(isfinite(minM.x) && isfinite(minM.y) && isfinite(maxM.x) && isfinite(maxM.y))) {
        return 0;
    }
    if (world->gridDirty && !collision_world_finalize(world)) return 0;

    int count = 0;
    if (world->gridEnabled) {
        /* Grid path: collect from each overlapped cell, dedupe with a per-query stamp, and
         * re-verify the exact AABB overlap (cells are conservative). The collected ids are
         * then sorted ascending so the answer is bit-identical to the brute scan whatever
         * the cell traversal produced. */
        uint32_t epoch = world->queryEpoch + 1u;
        // cppcheck-suppress knownConditionTrueFalse -- the wrap after 2^32 queries is real;
        // the checker's range analysis cannot see past the uint32 addition.
        if (epoch == 0u) { /* stamp counter wrapped; reset every stamp */
            memset(world->queryStamps, 0, sizeof(world->queryStamps));
            epoch = 1u;
        }
        world->queryEpoch = epoch;

        int c0, r0, c1, r1;
        cell_range(world, minM, maxM, &c0, &r0, &c1, &r1);
        for (int r = r0; r <= r1; r++) {
            for (int c = c0; c <= c1; c++) {
                const int cell = r * world->gridCols + c;
                for (int e = world->gridCellStart[cell]; e < world->gridCellStart[cell + 1];
                     e++) {
                    const CollisionShapeId id = world->gridEntries[e];
                    if (world->queryStamps[id] == epoch) continue;
                    world->queryStamps[id] = epoch;
                    if (afterId != COLLISION_SHAPE_ID_NONE && id <= afterId) continue;
                    if ((layerMask & world->shapes[id].layer) == 0u) continue;
                    if (!aabb_overlaps(world->shapes[id].minM, world->shapes[id].maxM, minM,
                                       maxM)) {
                        continue;
                    }
                    if (count < capacity) idsOut[count] = id;
                    count++;
                }
            }
        }
        if (count > capacity) count = capacity;
        qsort(idsOut, (size_t)count, sizeof(CollisionShapeId), compare_shape_ids);
        return count;
    }

    /* Brute path: one ascending scan, already in id order. */
    for (CollisionShapeId id = 0; id < (CollisionShapeId)world->shapeCount; id++) {
        if (afterId != COLLISION_SHAPE_ID_NONE && id <= afterId) continue;
        if ((layerMask & world->shapes[id].layer) == 0u) continue;
        if (!aabb_overlaps(world->shapes[id].minM, world->shapes[id].maxM, minM, maxM))
            continue;
        if (count < capacity) idsOut[count] = id;
        count++;
    }
    if (count > capacity) count = capacity;
    return count;
}

/* ====================================================================================== */
/*  Dynamic bodies and the contact feed                                                    */
/* ====================================================================================== */

void collision_world_begin_tick(CollisionWorld *world)
{
    if (world == NULL) return;
    world->bodyCount = 0;
    world->contactCount = 0;
    world->contactsOverflowed = false;
}

bool collision_world_add_body(CollisionWorld *world, const CollisionBody *body)
{
    if (world == NULL || body == NULL || body->id == 0u) return false;
    if (world->bodyCount >= COLLISION_WORLD_MAX_BODIES) return false;
    int at = 0;
    while (at < world->bodyCount && world->bodies[at].id < body->id) at++;
    if (at < world->bodyCount && world->bodies[at].id == body->id) return false; /* duplicate */
    for (int i = world->bodyCount; i > at; i--) world->bodies[i] = world->bodies[i - 1];
    world->bodies[at] = *body;
    world->bodyCount++;
    return true;
}

int collision_world_find_body(const CollisionWorld *world, CollisionBodyId id)
{
    if (world == NULL) return -1;
    for (int i = 0; i < world->bodyCount; i++) {
        if (world->bodies[i].id == id) return i;
    }
    return -1;
}

/* ====================================================================================== */
/*  Track barrier extraction (issue 26)                                                    */
/* ====================================================================================== */

/* Replicate the barrier polyline the narrowphase used to derive per tick: one segment per
 * centreline edge (closing the loop on a closed route), each side stored as its own shape,
 * left first, with the same endpoint and push-normal arithmetic. Shape ids therefore run in
 * the exact iteration order the legacy loop resolved in, so the candidate-driven narrowphase
 * resolves contacts in the same sequence. */
bool collision_world_build_from_track(CollisionWorld *world, const TrackDefinition *track)
{
    if (world == NULL) return false;
    collision_world_init(world);
    if (track == NULL || track->nodes == NULL || track->count < 2) return false;

    const int n = track->count;
    const int limit = track->routeClosed ? n : n - 1;
    for (int i = 0; i < limit; i++) {
        const int j = track->routeClosed ? (i + 1) % n : i + 1;
        const TrackNode *ni = &track->nodes[i];
        const TrackNode *nj = &track->nodes[j];
        const float segDx = nj->centerM.x - ni->centerM.x;
        const float segDy = nj->centerM.y - ni->centerM.y;
        const float segLen = sqrtf(segDx * segDx + segDy * segDy);
        if (segLen < 1e-12f) continue;
        const float invLen = 1.0f / segLen;
        const Vector2 dir = { segDx * invLen, segDy * invLen };
        const Vector2 perp = { -dir.y, dir.x };

        /* Barriers stand at the RUNOFF edge, not the racing-surface edge, so a car can run
         * wide onto the runoff and lose grip without instantly striking a wall. A node with
         * no runoff band reports its racing half-width here, which is exactly where its
         * barrier used to be. */
        const float hwI = track_node_barrier_half_width(ni);
        const float hwJ = track_node_barrier_half_width(nj);
        const Vector2 leftA = { ni->centerM.x + perp.x * hwI, ni->centerM.y + perp.y * hwI };
        const Vector2 leftB = { nj->centerM.x + perp.x * hwJ, nj->centerM.y + perp.y * hwJ };
        const Vector2 rightA = { ni->centerM.x - perp.x * hwI, ni->centerM.y - perp.y * hwI };
        const Vector2 rightB = { nj->centerM.x - perp.x * hwJ, nj->centerM.y - perp.y * hwJ };
        /* Push normal: for ribbon tracks, left barrier pushes right ({dir.y, -dir.x}), right
         * barrier pushes left ({-dir.y, dir.x}). For the parking lot perimeter BOTH barriers
         * push INWARD (perp). */
        const Vector2 pushLeft = track->isParkingLot ? perp : (Vector2){ dir.y, -dir.x };
        const Vector2 pushRight = track->isParkingLot ? perp : (Vector2){ -dir.y, dir.x };

        if (collision_world_add_static_segment(world, leftA, leftB, pushLeft,
                                               COLLISION_LAYER_STATIC_BARRIER) < 0) {
            return false; /* shape cap exceeded: refuse the track rather than drop barriers */
        }
        if (collision_world_add_static_segment(world, rightA, rightB, pushRight,
                                               COLLISION_LAYER_STATIC_BARRIER) < 0) {
            return false;
        }
    }
    (void)collision_world_finalize(world);
    return true;
}
