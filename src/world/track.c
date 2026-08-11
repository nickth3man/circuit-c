/*
 * track.c — track geometry, surface bands, and ordered lap checkpoints.
 *
 * Two layouts live here: the open parking lot used for free driving, and the chicane circuit
 * every car is validated against. Both arrays are calloc'd here and freed by track_free();
 * they survive hot reloads because they are heap memory, not module static data.
 */
#include "world/track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME 16777619u

/* --------------- straight-line helpers ---------------------------------------------------- */

/* Squared distance from point p to the finite line segment a→b. */
static float point_to_segment_sq(Vector2 p, Vector2 a, Vector2 b)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-12f) {
        /* Degenerate segment: distance to the single point. */
        const float ex = p.x - a.x;
        const float ey = p.y - a.y;
        return ex * ex + ey * ey;
    }
    /* Projection parameter t clamped to [0, 1]. */
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float closestX = a.x + t * dx;
    const float closestY = a.y + t * dy;
    const float ex = p.x - closestX;
    const float ey = p.y - closestY;
    return ex * ex + ey * ey;
}

/* Closest centreline segment: returns the squared perpendicular distance and sets *closestIdx
 * to the segment index (the i in nodes[i]→nodes[(i+1)%count] for closed, or nodes[i]→nodes[i+1]
 * for open). *closestIdx is untouched when closestIdx is NULL. */
static float nearest_centerline_distance_sq(const TrackNode *nodes, int count, bool closed,
                                            Vector2 point, int *closestIdx)
{
    float best = 1e30f;
    int bestIdx = 0;
    const int limit = closed ? count : count - 1;
    for (int i = 0; i < limit; i++) {
        const int j = closed ? (i + 1) % count : i + 1;
        const float dSq = point_to_segment_sq(point, nodes[i].centerM, nodes[j].centerM);
        if (dSq < best) {
            best = dSq;
            bestIdx = i;
        }
    }
    if (closestIdx != NULL) *closestIdx = bestIdx;
    return best;
}

/* Build the exact nearest-segment grid from a definition's centreline (#39). The grid is a
 * pure derived cache: the build is deterministic and reads only geometry. Returns false when
 * the track exceeds the grid caps, in which case queries fall back to the linear scan. */
static bool track_grid_build(TrackQueryGrid *grid, const TrackDefinition *track)
{
    memset(grid, 0, sizeof(*grid));
    if (track == NULL || track->nodes == NULL || track->count < 2) return false;
    const int count = track->count;
    const int segCount = track->routeClosed ? count : count - 1;

    float minX = 1e30f, minY = 1e30f, maxX = -1e30f, maxY = -1e30f;
    float maxSegLen = 0.0f;
    for (int i = 0; i < segCount; i++) {
        const int j = track->routeClosed ? (i + 1) % count : i + 1;
        const Vector2 a = track->nodes[i].centerM;
        const Vector2 b = track->nodes[j].centerM;
        minX = fminf(minX, fminf(a.x, b.x));
        minY = fminf(minY, fminf(a.y, b.y));
        maxX = fmaxf(maxX, fmaxf(a.x, b.x));
        maxY = fmaxf(maxY, fmaxf(a.y, b.y));
        const float len = sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
        if (len > maxSegLen) maxSegLen = len;
    }
    if (!isfinite(maxSegLen) || maxSegLen <= 0.0f) return false;

    /* Cell size is forced to at least the longest segment so a segment spans at most 2x2
     * cells (same sizing rule as the collision broadphase). */
    const float spanX = fmaxf(maxX - minX, 1e-6f);
    const float spanY = fmaxf(maxY - minY, 1e-6f);
    int cols = (int)ceilf(spanX / maxSegLen);
    int rows = (int)ceilf(spanY / maxSegLen);
    if (cols > TRACK_GRID_MAX_COLS) cols = TRACK_GRID_MAX_COLS;
    if (rows > TRACK_GRID_MAX_ROWS) rows = TRACK_GRID_MAX_ROWS;
    if (cols < 1) cols = 1;
    if (rows < 1) rows = 1;
    const float cellW = spanX / (float)cols;
    const float cellH = spanY / (float)rows;

    /* Pass 1: count segments per cell, using cellStarts as a scratch count array. */
    memset(grid->cellStarts, 0, sizeof(int) * (size_t)(cols * rows + 1));
    int total = 0;
    for (int s = 0; s < segCount && total <= TRACK_GRID_MAX_ENTRIES; s++) {
        const int j = track->routeClosed ? (s + 1) % count : s + 1;
        const Vector2 a = track->nodes[s].centerM;
        const Vector2 b = track->nodes[j].centerM;
        int c0x = (int)floorf((fminf(a.x, b.x) - minX) / cellW);
        int c1x = (int)floorf((fmaxf(a.x, b.x) - minX) / cellW);
        int c0y = (int)floorf((fminf(a.y, b.y) - minY) / cellH);
        int c1y = (int)floorf((fmaxf(a.y, b.y) - minY) / cellH);
        if (c0x < 0) c0x = 0;
        if (c1x >= cols) c1x = cols - 1;
        if (c0y < 0) c0y = 0;
        if (c1y >= rows) c1y = rows - 1;
        for (int cy = c0y; cy <= c1y; cy++) {
            for (int cx = c0x; cx <= c1x; cx++) {
                grid->cellStarts[cy * cols + cx]++;
                total++;
            }
        }
    }
    if (total > TRACK_GRID_MAX_ENTRIES) return false; /* too dense: fall back to linear */

    /* Prefix sums: cellStarts[c] becomes the start offset of cell c's entries. */
    int acc = 0;
    for (int c = 0; c < cols * rows; c++) {
        const int n = grid->cellStarts[c];
        grid->cellStarts[c] = acc;
        acc += n;
    }
    grid->cellStarts[cols * rows] = acc;

    /* Pass 2: fill entries in ascending segment order (deterministic). */
    int cursor[TRACK_GRID_MAX_CELLS];
    memcpy(cursor, grid->cellStarts, sizeof(int) * (size_t)(cols * rows));
    for (int s = 0; s < segCount; s++) {
        const int j = track->routeClosed ? (s + 1) % count : s + 1;
        const Vector2 a = track->nodes[s].centerM;
        const Vector2 b = track->nodes[j].centerM;
        int c0x = (int)floorf((fminf(a.x, b.x) - minX) / cellW);
        int c1x = (int)floorf((fmaxf(a.x, b.x) - minX) / cellW);
        int c0y = (int)floorf((fminf(a.y, b.y) - minY) / cellH);
        int c1y = (int)floorf((fmaxf(a.y, b.y) - minY) / cellH);
        if (c0x < 0) c0x = 0;
        if (c1x >= cols) c1x = cols - 1;
        if (c0y < 0) c0y = 0;
        if (c1y >= rows) c1y = rows - 1;
        for (int cy = c0y; cy <= c1y; cy++) {
            for (int cx = c0x; cx <= c1x; cx++) {
                const int cell = cy * cols + cx;
                grid->entries[cursor[cell]++] = s;
            }
        }
    }

    grid->cols = cols;
    grid->rows = rows;
    grid->minX = minX;
    grid->minY = minY;
    grid->cellW = cellW;
    grid->cellH = cellH;
    grid->built = true;
    return true;
}

/* Exact nearest-segment query through the grid: expands in Chebyshev rings and stops when the
 * nearest unsearched ring square is farther than the current best — the classic exact
 * grid nearest-neighbour termination. Returns the squared distance and writes *closestIdx. */
static float track_grid_nearest_sq(const TrackQueryGrid *grid, const TrackDefinition *track,
                                   Vector2 point, int *closestIdx)
{
    const int count = track->count;
    const int cols = grid->cols;
    const int rows = grid->rows;

    /* Clamp the point into the grid; every segment is inside the grid. */
    const float px = point.x < grid->minX ? grid->minX
                                          : (point.x > grid->minX + (float)cols * grid->cellW
                                                 ? grid->minX + (float)cols * grid->cellW
                                                 : point.x);
    const float py = point.y < grid->minY ? grid->minY
                                          : (point.y > grid->minY + (float)rows * grid->cellH
                                                 ? grid->minY + (float)rows * grid->cellH
                                                 : point.y);
    int cx = (int)floorf((px - grid->minX) / grid->cellW);
    int cy = (int)floorf((py - grid->minY) / grid->cellH);
    if (cx < 0) cx = 0;
    if (cx >= cols) cx = cols - 1;
    if (cy < 0) cy = 0;
    if (cy >= rows) cy = rows - 1;

    float best = 1e30f;
    int bestIdx = -1;
    const int maxRing = (cols > rows ? cols : rows);

    for (int r = 0; r <= maxRing; r++) {
        /* Distance from the point to the ring square [cx-r, cx+r] x [cy-r, cy+r]. */
        const int gx0 = cx - r < 0 ? 0 : cx - r;
        const int gx1 = cx + r >= cols ? cols - 1 : cx + r;
        const int gy0 = cy - r < 0 ? 0 : cy - r;
        const int gy1 = cy + r >= rows ? rows - 1 : cy + r;
        const float wx0 = grid->minX + (float)gx0 * grid->cellW;
        const float wx1 = grid->minX + (float)(gx1 + 1) * grid->cellW;
        const float wy0 = grid->minY + (float)gy0 * grid->cellH;
        const float wy1 = grid->minY + (float)(gy1 + 1) * grid->cellH;
        const float dx = fmaxf(wx0 - point.x, fmaxf(0.0f, point.x - wx1));
        const float dy = fmaxf(wy0 - point.y, fmaxf(0.0f, point.y - wy1));
        if (dx * dx + dy * dy >= best) break; /* no unsearched cell can beat the best */

        for (int cyy = gy0; cyy <= gy1; cyy++) {
            for (int cxx = gx0; cxx <= gx1; cxx++) {
                const int cdist = (cxx > cx ? cxx - cx : cx - cxx);
                const int cdistY = (cyy > cy ? cyy - cy : cy - cyy);
                const int cheb = cdist > cdistY ? cdist : cdistY;
                if (cheb != r) continue; /* inner cells were handled by earlier rings */
                const int cell = cyy * cols + cxx;
                const int start = grid->cellStarts[cell];
                const int end = grid->cellStarts[cell + 1];
                for (int e = start; e < end; e++) {
                    const int s = grid->entries[e];
                    const int j = track->routeClosed ? (s + 1) % count : s + 1;
                    const float dSq = point_to_segment_sq(point, track->nodes[s].centerM,
                                                          track->nodes[j].centerM);
                    if (dSq < best) {
                        best = dSq;
                        bestIdx = s;
                    }
                }
            }
        }
    }

    if (bestIdx < 0) /* defensive: grid empty or all segments degenerate */
        return nearest_centerline_distance_sq(track->nodes, count, track->routeClosed, point,
                                              closestIdx);
    if (closestIdx != NULL) *closestIdx = bestIdx;
    return best;
}

/* --------------- public API -------------------------------------------------------------- */

void track_init(TrackDefinition *track)
{
    if (track == NULL) return;
    /* Defensive: free any previously-allocated state first. */
    track_free(track);

    /* Parking lot: 400m x 300m open rectangle centred at origin. */
    track->isParkingLot = true;
    track->lotMinXM = -200.0f;
    track->lotMaxXM = 200.0f;
    track->lotMinYM = -150.0f;
    track->lotMaxYM = 150.0f;

    /* Perimeter centreline for collision barriers. 4 sides in clockwise order:
     * bottom (L->R), right (B->T), top (R->L), left (T->B), plus a closing node. */
#define LOT_NODES 5
    track->count = LOT_NODES;
    track->nodes = (TrackNode *)calloc((size_t)track->count, sizeof(TrackNode));
    if (!track->nodes) return;

    /* No runoff band: the lot's perimeter barrier stands on the edge of the drivable area,
     * which is what it has always done. Stated explicitly rather than left to zero-fill so
     * the intent is visible and the compiler does not warn about a partial initialiser. */
    const float hw = 4.0f; /* wide enough so inner/outer barriers don't sandwich the car */
    const float noRunoff = 0.0f;
    int i = 0;
    const Vector2 corners[LOT_NODES] = {
        { track->lotMinXM, track->lotMinYM }, { track->lotMaxXM, track->lotMinYM },
        { track->lotMaxXM, track->lotMaxYM }, { track->lotMinXM, track->lotMaxYM },
        { track->lotMinXM, track->lotMinYM },
    };
    for (i = 0; i < LOT_NODES; i++) {
        track->nodes[i] = (TrackNode){ corners[i], hw, SURFACE_ASPHALT, noRunoff };
    }

    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    track->routeClosed = true;
    track->aiEligible = true;

    /* The lot is an open area, not a circuit, but the perimeter still carries gates so that
     * lap bookkeeping behaves identically to before gates became explicit data. A racer's
     * starting gate is now that racer's business: see track_reset_progress_at(). */
    track_build_checkpoints_from_nodes(track);
    snprintf(track->id, sizeof(track->id), "%s", "parking_lot");
    snprintf(track->version, sizeof(track->version), "%s", "v1");
}

void track_free(TrackDefinition *track)
{
    if (track == NULL) return;
    free(track->nodes);
    track->nodes = NULL;
    track->count = 0;
    free(track->checkpoints);
    track->checkpoints = NULL;
    track->checkpointCount = 0;
    free(track->sectorMarkers);
    track->sectorMarkers = NULL;
    track->sectorMarkerCount = 0;
    free(track->gridSlots);
    track->gridSlots = NULL;
    track->gridSlotCount = 0;
    free(track->serviceBoxes);
    track->serviceBoxes = NULL;
    track->serviceBoxCount = 0;
    track->offTrackSurfaceId = SURFACE_ASPHALT;
    track->runoffSurfaceId = SURFACE_ASPHALT;
    track->routeClosed = true;
    track->hasStartFinish = false;
    memset(&track->startFinish, 0, sizeof(track->startFinish));
    track->hasPitEntry = false;
    memset(&track->pitEntry, 0, sizeof(track->pitEntry));
    track->hasPitExit = false;
    memset(&track->pitExit, 0, sizeof(track->pitExit));
    track->hasPitSpeedLine = false;
    memset(&track->pitSpeedLine, 0, sizeof(track->pitSpeedLine));
    track->id[0] = '\0';
    track->version[0] = '\0';
}

bool track_runtime_bind(TrackRuntime *runtime, const TrackDefinition *track)
{
    if (runtime == NULL) return false;
    if (!collision_world_build_from_track(&runtime->collisionWorld, track)) {
        /* The definition's collision world could not be built (no geometry, or more
         * barriers than the world holds). Leave the hash untouched so a later bind retries
         * and the immutability check cannot pass for a runtime that was never bound; the
         * build failure path has already emptied the world so nothing resolves against a
         * partial fence. */
        return false;
    }
    /* Publish the hash only after a successful build: recording it first would let the
     * collision stage treat a failed bind as current and never rebuild. */
    runtime->definitionHash = track_geometry_hash(track);
    return true;
}

bool track_runtime_definition_unchanged(const TrackRuntime *runtime,
                                        const TrackDefinition *track)
{
    if (runtime == NULL) return false;
    return runtime->definitionHash == track_geometry_hash(track);
}

SurfaceId Track_SurfaceAt(const TrackDefinition *track, const TrackRuntime *runtime,
                          Vector2 pointM)
{
    /* Accepted now so #39/#41 can add cached queries and wetness without moving every call
     * site again; the surface a point sits on is pure definition geometry today. */
    (void)runtime;

    /* Headless tests and scenarios that don't initialise a track hit this safe default, so
     * existing scenarios remain on asphalt and their CSVs are unchanged. */
    if (track == NULL || track->nodes == NULL || track->count <= 0) {
        return SURFACE_ASPHALT;
    }

    /* Parking lot: simple AABB test. */
    if (track->isParkingLot) {
        if (pointM.x >= track->lotMinXM && pointM.x <= track->lotMaxXM &&
            pointM.y >= track->lotMinYM && pointM.y <= track->lotMaxYM)
            return SURFACE_ASPHALT;
        return track->offTrackSurfaceId; /* grass */
    }

    int closestIdx = 0;
    float dSq;
    TrackRuntime *mutableRuntime = (TrackRuntime *)runtime;
    if (mutableRuntime != NULL && !mutableRuntime->queryGrid.built) {
        /* Build the exact spatial index lazily on first query (#39). */
        track_grid_build(&mutableRuntime->queryGrid, track);
    }
    if (mutableRuntime != NULL && mutableRuntime->queryGrid.built) {
        dSq = track_grid_nearest_sq(&mutableRuntime->queryGrid, track, pointM, &closestIdx);
    } else {
        dSq = nearest_centerline_distance_sq(track->nodes, track->count, track->routeClosed,
                                             pointM, &closestIdx);
    }
    const TrackNode *seg = &track->nodes[closestIdx];

    /* Three bands: racing surface, then runoff, then off-track. The runoff band is what makes
     * an excursion measurable — without it the racing surface ends exactly where the barrier
     * begins and a car can never be off-track without also being in a wall. */
    if (dSq <= seg->halfWidthM * seg->halfWidthM) {
        return seg->surfaceId;
    }
    const float barrierHalfWidthM = track_node_barrier_half_width(seg);
    if (dSq <= barrierHalfWidthM * barrierHalfWidthM) {
        return track->runoffSurfaceId;
    }
    return track->offTrackSurfaceId;
}

float track_distance_to_centerline_m(const TrackDefinition *track, Vector2 pointM,
                                     float *halfWidthM)
{
    if (track == NULL || track->nodes == NULL || track->count <= 0) {
        if (halfWidthM != NULL) *halfWidthM = 0.0f;
        return 0.0f;
    }
    int closestIdx = 0;
    const float dSq = nearest_centerline_distance_sq(track->nodes, track->count,
                                                     track->routeClosed, pointM, &closestIdx);
    if (halfWidthM != NULL) {
        *halfWidthM = track->nodes[closestIdx].halfWidthM;
    }
    return sqrtf(dSq);
}

/* --------------- checkpoint crossing ------------------------------------------------------
 *
 * A gate is a line segment at each TrackNode, perpendicular to the centreline, spanning the
 * track width. The car crosses a gate when its prev→curr position segment intersects it, and
 * only when the car is moving in the track's forward direction.
 */

/* 2D cross-product z-component: a.x * b.y - a.y * b.x */
static float cross_z(Vector2 a, Vector2 b)
{
    return a.x * b.y - a.y * b.x;
}

/* Orientation test: > 0 if points a,b,c are counterclockwise. */
static float orient(Vector2 a, Vector2 b, Vector2 c)
{
    return cross_z((Vector2){ b.x - a.x, b.y - a.y }, (Vector2){ c.x - a.x, c.y - a.y });
}

/* Standard segment-segment intersection test (including endpoints). */
static bool segments_intersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
{
    const float d1 = orient(p3, p4, p1);
    const float d2 = orient(p3, p4, p2);
    const float d3 = orient(p1, p2, p3);
    const float d4 = orient(p1, p2, p4);

    /* Use a tolerance rather than strict sign checks so collinear grazing counts. */
    const float eps = 1e-9f;

    /* General case: the endpoints of each segment are on opposite sides of the other. */
    if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
        ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
        return true;
    }

    /* Degenerate / collinear: any endpoint lies on the other segment. */
    if (fabsf(d1) <= eps && fabsf(d2) <= eps && fabsf(d3) <= eps && fabsf(d4) <= eps) {
        /* Overlap check: bounding-box test. */
        const float mnx1 = fminf(p1.x, p2.x) - eps;
        const float mxx1 = fmaxf(p1.x, p2.x) + eps;
        const float mny1 = fminf(p1.y, p2.y) - eps;
        const float mxy1 = fmaxf(p1.y, p2.y) + eps;
        const float mnx2 = fminf(p3.x, p4.x) - eps;
        const float mxx2 = fmaxf(p3.x, p4.x) + eps;
        const float mny2 = fminf(p3.y, p4.y) - eps;
        const float mxy2 = fmaxf(p3.y, p4.y) + eps;

        /* Check if any endpoint of one segment falls inside the bounding box of the other. */
        if ((p3.x >= mnx1 && p3.x <= mxx1 && p3.y >= mny1 && p3.y <= mxy1) ||
            (p4.x >= mnx1 && p4.x <= mxx1 && p4.y >= mny1 && p4.y <= mxy1) ||
            (p1.x >= mnx2 && p1.x <= mxx2 && p1.y >= mny2 && p1.y <= mxy2) ||
            (p2.x >= mnx2 && p2.x <= mxx2 && p2.y >= mny2 && p2.y <= mxy2)) {
            return true;
        }
        return false;
    }

    return false;
}

/* --------------- the chicane validation circuit --------------------------------------------
 *
 * A closed stadium — two 200 m straights joined by 45 m-radius 180-degree curves — with a
 * left-right chicane set into the far straight. About 690 m, so a competent lap is roughly
 * half a minute and an out-lap plus a timed lap fits comfortably inside the replay buffer.
 *
 * The shape is chosen for what it measures, not for interest: the main straight is long
 * enough to reach a speed where braking matters, the curves are constant-radius so a car's
 * steady-state balance is observable, and the chicane forces a genuine direction change with
 * no time to settle between the two apexes. That last part is the one a car with lazy
 * turn-in, too much rearward brake bias, or a snappy rear axle will fail.
 *
 * The chicane's lateral displacement follows A*sin^2(pi*u), which starts and ends with zero
 * slope. That matters structurally, not just aesthetically: barriers are built per segment,
 * so a kink in the centreline becomes a concave joint that a swept capsule can catch on.
 */

#define CHICANE_STRAIGHT_HALF_X_M 100.0f /* straights run x = -100 .. +100 */
#define CHICANE_CURVE_RADIUS_M 45.0f
#define CHICANE_FAR_STRAIGHT_Y_M 90.0f /* = 2 * curve radius, so the curves close the loop */
#define CHICANE_NODE_SPACING_M 4.0f
#define CHICANE_OFFSET_M 16.0f    /* chicane lateral displacement */
#define CHICANE_SPAN_M 80.0f      /* distance over which it displaces and returns */
#define CHICANE_ENTRY_X_M 55.0f   /* chicane begins here on the far straight */
#define TRACK_HALF_WIDTH_M 8.0f   /* racing surface, straights and curves */
#define TRACK_RUNOFF_HALF_M 12.0f /* barrier stands here; 4 m of grass in between */
#define CHICANE_HALF_WIDTH_M 6.0f /* the chicane is deliberately tighter */
#define CHICANE_RUNOFF_HALF_M 8.0f
#define GATE_HALF_WIDTH_M \
    10.0f /* gates span past the racing surface so a wide but legal
                                 * line still scores; they validate route, not precision */

typedef struct {
    TrackNode *nodes;
    int count;
    int capacity;
} NodeBuilder;

static void builder_push(NodeBuilder *b, float x, float y, float halfWidthM, float runoffM)
{
    if (b->nodes == NULL || b->count >= b->capacity) return;
    b->nodes[b->count].centerM = (Vector2){ x, y };
    b->nodes[b->count].halfWidthM = halfWidthM;
    b->nodes[b->count].surfaceId = SURFACE_ASPHALT;
    b->nodes[b->count].runoffHalfWidthM = runoffM;
    b->count++;
}

/* The chicane's lateral offset at arc fraction u in [0,1]. Zero slope at both ends. */
static float chicane_offset_at(float u)
{
    const float s = sinf(3.14159265358979323846f * u);
    return CHICANE_OFFSET_M * s * s;
}

/*
 * World-space centreline point on the far straight at x: (x, farY + offset(u)). Same u as
 * chicane_far_forward(), so a gate's centre and its forward are derived from the same authored
 * geometry and can never drift apart (PR #80 review).
 */
static Vector2 chicane_far_point(float x)
{
    const float u = (CHICANE_ENTRY_X_M - x) / CHICANE_SPAN_M;
    return (Vector2){ x, CHICANE_FAR_STRAIGHT_Y_M + chicane_offset_at(u) };
}

/*
 * Unit forward (travel direction) at world x on the far straight, where the chicane displaces
 * the centreline vertically by offset(u), u = (ENTRY_X - x) / SPAN. Travel is -X, so the
 * tangent for an infinitesimal step in -X is (-1, -dy/dx) normalized, where dy/dx is the
 * centreline slope. Derived from the same offset() the node builder uses, so a gate's forward
 * is exactly unit and exactly aligned with the authored geometry rather than hand-trig. Returns
 * (-1, 0) on the straight portions outside the chicane (zero slope there).
 */
static Vector2 chicane_far_forward(float x)
{
    const float pi = 3.14159265358979323846f;
    const float intoChicane = CHICANE_ENTRY_X_M - x;
    float dydx = 0.0f;
    if (intoChicane >= 0.0f && intoChicane <= CHICANE_SPAN_M) {
        const float u = intoChicane / CHICANE_SPAN_M;
        /* offset(u) = OFFSET * sin^2(pi u); d/du = OFFSET * pi * sin(2 pi u); du/dx = -1/SPAN. */
        dydx = CHICANE_OFFSET_M * pi * sinf(2.0f * pi * u) * (-1.0f / CHICANE_SPAN_M);
    }
    const float fx = -1.0f;
    const float fy = -dydx;
    const float len = sqrtf(fx * fx + fy * fy);
    return (Vector2){ fx / len, fy / len };
}

void track_load_chicane(TrackDefinition *track)
{
    if (track == NULL) return;
    track_free(track);

    track->isParkingLot = false;
    track->routeClosed = true;
    track->aiEligible = true;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    snprintf(track->id, sizeof(track->id), "%s", "chicane");
    snprintf(track->version, sizeof(track->version), "%s", "chicane_v2");

    /* Generous upper bound; the builder stops at capacity and count is what is used. */
    const int capacity = 512;
    track->nodes = (TrackNode *)calloc((size_t)capacity, sizeof(TrackNode));
    if (track->nodes == NULL) return;
    NodeBuilder b = { track->nodes, 0, capacity };

    const float pi = 3.14159265358979323846f;
    const float halfX = CHICANE_STRAIGHT_HALF_X_M;
    const float radius = CHICANE_CURVE_RADIUS_M;
    const float farY = CHICANE_FAR_STRAIGHT_Y_M;

    /* 1. Near straight, travelling +X along y = 0. */
    for (float x = -halfX; x < halfX - 0.5f * CHICANE_NODE_SPACING_M;
         x += CHICANE_NODE_SPACING_M) {
        builder_push(&b, x, 0.0f, TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
    }

    /* 2. Right curve: centre (halfX, radius), sweeping -90 to +90 degrees, so the car turns
     * left through 180 degrees and comes back along the far straight. */
    {
        const int steps = (int)((pi * radius) / CHICANE_NODE_SPACING_M);
        for (int i = 0; i < steps; i++) {
            const float theta = -0.5f * pi + pi * ((float)i / (float)steps);
            builder_push(&b, halfX + radius * cosf(theta), radius + radius * sinf(theta),
                         TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    /* 3. Far straight, travelling -X along y = farY, with the chicane set into it. */
    for (float x = halfX; x > -halfX + 0.5f * CHICANE_NODE_SPACING_M;
         x -= CHICANE_NODE_SPACING_M) {
        const float intoChicane = CHICANE_ENTRY_X_M - x; /* grows as the car travels -X */
        const bool inChicane = (intoChicane >= 0.0f && intoChicane <= CHICANE_SPAN_M);
        if (inChicane) {
            const float u = intoChicane / CHICANE_SPAN_M;
            builder_push(&b, x, farY + chicane_offset_at(u), CHICANE_HALF_WIDTH_M,
                         CHICANE_RUNOFF_HALF_M);
        } else {
            builder_push(&b, x, farY, TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    /* 4. Left curve: centre (-halfX, radius), sweeping +90 to +270 degrees, closing the loop
     * back onto the start of the near straight. */
    {
        const int steps = (int)((pi * radius) / CHICANE_NODE_SPACING_M);
        for (int i = 0; i < steps; i++) {
            const float theta = 0.5f * pi + pi * ((float)i / (float)steps);
            builder_push(&b, -halfX + radius * cosf(theta), radius + radius * sinf(theta),
                         TRACK_HALF_WIDTH_M, TRACK_RUNOFF_HALF_M);
        }
    }

    track->count = b.count;

    /*
     * Twenty-five required gates (issue #78). The eight-gate layout could prove a car went the
     * right way round, but it could not say WHERE a failed run stopped: ~690 m of route reduced
     * to eight scoring gates left a spin, a departure, and a stall all reading as "missed a
     * gate somewhere". The denser layout places a gate at every corner entry/apex/exit, before
     * and after the chicane's direction changes, and along the straights, so the first gate a
     * failed run stops reporting narrows the fault to ~30 m of road. The 10 m non-scoring
     * progress bins (route_localization.c) carry the finer resolution; these gates stay
     * route-validation rules, not timing loops.
     *
     * Gate forwards are the route tangent at the gate (axis-aligned on the straights and at the
     * curve apices, 45 deg at the curve quarters, and the local chicane tangent through the
     * chicane direction changes). A car that cuts the chicane by holding y = farY crosses the
     * entry-side gates but misses the apex gate and then trips the chicane-exit gate out of
     * order, so a shortcut is rejected loudly rather than silently scored. Half-width stays at
     * GATE_HALF_WIDTH_M so a wide but legal line still scores.
     */
    const int gateCount = 25;
    track->checkpoints = (Checkpoint *)calloc((size_t)gateCount, sizeof(Checkpoint));
    if (track->checkpoints == NULL) return;
    track->checkpointCount = gateCount;

    const float q = 0.7071068f;                      /* cos/sin 45 deg, unit to ~6e-7 */
    const float curveQ = CHICANE_CURVE_RADIUS_M * q; /* 45 m radius quarter offset */
    const float chicaneApexX = CHICANE_ENTRY_X_M - 0.5f * CHICANE_SPAN_M;
    const Checkpoint gates[25] = {
        /* Near straight, travelling +X along y = 0. Gate 0 is start/finish. */
        { { -60.0f, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 0: start/finish */
        { { -20.0f, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 1 */
        { { 20.0f, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },  /* 2 */
        { { 60.0f, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },  /* 3 */
        { { halfX, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },  /* 4: turn-in */
        /* Right curve (centre (halfX, radius), -90 to +90 deg). */
        { { halfX + curveQ, radius - curveQ },
          { q, q },
          GATE_HALF_WIDTH_M,
          true }, /* 5: entry quarter */
        { { halfX + radius, radius },
          { 0.0f, 1.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 6: apex +Y */
        { { halfX + curveQ, radius + curveQ },
          { -q, q },
          GATE_HALF_WIDTH_M,
          true }, /* 7: exit quarter */
        /* Far straight, travelling -X along y = farY, with the chicane set into it. */
        { { halfX, farY },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 8: far-straight entry */
        { { 75.0f, farY }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 9 */
        { { CHICANE_ENTRY_X_M, farY },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 10: chicane entry */
        /* Chicane direction changes: centres and forwards are derived from the same authored
         * geometry the centreline was built from (chicane_far_point / chicane_far_forward
         * below), so a change to the constants cannot leave a gate floating off the route.
         * The y in the table is a placeholder (-1) overwritten by the derived centre. */
        { { 44.0f, -1.0f }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 11: climbing */
        { { 36.0f, -1.0f }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 12: climbing */
        { { 28.0f, -1.0f },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 13: approaching apex */
        { { chicaneApexX, farY + CHICANE_OFFSET_M },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true },                                                        /* 14: apex */
        { { 4.0f, -1.0f }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },   /* 15: descending */
        { { -8.0f, -1.0f }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true },  /* 16: descending */
        { { -16.0f, -1.0f }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 17: exiting */
        { { CHICANE_ENTRY_X_M - CHICANE_SPAN_M, farY },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true },                                                       /* 18: chicane exit */
        { { -60.0f, farY }, { -1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 19 */
        { { -halfX, farY },
          { -1.0f, 0.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 20: left-curve turn-in */
        /* Left curve (centre (-halfX, radius), +90 to +270 deg). */
        { { -halfX - curveQ, radius + curveQ },
          { -q, -q },
          GATE_HALF_WIDTH_M,
          true }, /* 21: entry quarter */
        { { -halfX - radius, radius },
          { 0.0f, -1.0f },
          GATE_HALF_WIDTH_M,
          true }, /* 22: apex -Y */
        { { -halfX - curveQ, radius - curveQ },
          { q, -q },
          GATE_HALF_WIDTH_M,
          true }, /* 23: exit quarter */
        /* Seam approach: the last gate before the wrap back to start/finish. */
        { { -halfX, 0.0f }, { 1.0f, 0.0f }, GATE_HALF_WIDTH_M, true }, /* 24 */
    };
    memcpy(track->checkpoints, gates, sizeof(gates));

    /* The chicane tangent gates (11, 12, 13, 15, 16, 17) sit on direction changes; their
     * centres and forwards are derived from the same geometry the centreline was built from.
     * Done after the copy because C initializers cannot call functions. */
    track->checkpoints[11].centerM = chicane_far_point(track->checkpoints[11].centerM.x);
    track->checkpoints[12].centerM = chicane_far_point(track->checkpoints[12].centerM.x);
    track->checkpoints[13].centerM = chicane_far_point(track->checkpoints[13].centerM.x);
    track->checkpoints[15].centerM = chicane_far_point(track->checkpoints[15].centerM.x);
    track->checkpoints[16].centerM = chicane_far_point(track->checkpoints[16].centerM.x);
    track->checkpoints[17].centerM = chicane_far_point(track->checkpoints[17].centerM.x);
    track->checkpoints[11].forwardUnit = chicane_far_forward(track->checkpoints[11].centerM.x);
    track->checkpoints[12].forwardUnit = chicane_far_forward(track->checkpoints[12].centerM.x);
    track->checkpoints[13].forwardUnit = chicane_far_forward(track->checkpoints[13].centerM.x);
    track->checkpoints[15].forwardUnit = chicane_far_forward(track->checkpoints[15].centerM.x);
    track->checkpoints[16].forwardUnit = chicane_far_forward(track->checkpoints[16].centerM.x);
    track->checkpoints[17].forwardUnit = chicane_far_forward(track->checkpoints[17].centerM.x);
}
void track_load_sprint(TrackDefinition *track)
{
    if (track == NULL) return;

    /* Start from the authored chicane, then apply a fixed affine layout change to both the
     * centreline and its gates. This keeps the route contract identical while exercising AI
     * geometry following on a genuinely different footprint. */
    track_load_chicane(track);
    const float scaleX = 0.82f;
    const float scaleY = 0.88f;
    for (int i = 0; i < track->count; i++) {
        track->nodes[i].centerM.x *= scaleX;
        track->nodes[i].centerM.y *= scaleY;
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        track->checkpoints[i].centerM.x *= scaleX;
        track->checkpoints[i].centerM.y *= scaleY;
        const Vector2 forward = track->checkpoints[i].forwardUnit;
        const float length = sqrtf((forward.x * scaleX) * (forward.x * scaleX) +
                                   (forward.y * scaleY) * (forward.y * scaleY));
        if (length > 1e-12f) {
            track->checkpoints[i].forwardUnit =
                (Vector2){ forward.x * scaleX / length, forward.y * scaleY / length };
        }
    }
    snprintf(track->id, sizeof(track->id), "%s", "sprint");
    snprintf(track->version, sizeof(track->version), "%s", "sprint_v2");
}
void track_load_technical(TrackDefinition *track)
{
    if (track == NULL) return;

    /* The technical circuit keeps the authored route contract but compresses both axes, adds a
     * small affine skew, and narrows every ribbon. The result has materially tighter curves and
     * less recovery room than sprint_v1 while remaining a closed, collision-testable circuit. */
    track_load_chicane(track);
    const float scaleX = 0.62f;
    const float scaleY = 0.58f;
    const float skewX = 0.08f;
    const float skewY = 0.05f;

    for (int i = 0; i < track->count; i++) {
        TrackNode *node = &track->nodes[i];
        const Vector2 source = node->centerM;
        node->centerM = (Vector2){ scaleX * source.x + skewX * source.y,
                                   skewY * source.x + scaleY * source.y };
        const bool wasChicane = node->halfWidthM <= CHICANE_HALF_WIDTH_M;
        node->halfWidthM = wasChicane ? 4.5f : 5.8f;
        node->runoffHalfWidthM = node->halfWidthM + (wasChicane ? 2.0f : 2.5f);
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        Checkpoint *checkpoint = &track->checkpoints[i];
        const Vector2 source = checkpoint->centerM;
        checkpoint->centerM = (Vector2){ scaleX * source.x + skewX * source.y,
                                         skewY * source.x + scaleY * source.y };
        const Vector2 forward = checkpoint->forwardUnit;
        const Vector2 transformed = { scaleX * forward.x + skewX * forward.y,
                                      skewY * forward.x + scaleY * forward.y };
        const float length =
            sqrtf(transformed.x * transformed.x + transformed.y * transformed.y);
        if (length > 1.0e-12f) {
            checkpoint->forwardUnit =
                (Vector2){ transformed.x / length, transformed.y / length };
        }
        checkpoint->halfWidthM = 7.0f;
    }
    snprintf(track->id, sizeof(track->id), "%s", "technical");
    snprintf(track->version, sizeof(track->version), "%s", "technical_v2");
}

bool track_build_checkpoints_from_nodes(TrackDefinition *track)
{
    if (track == NULL || track->nodes == NULL || track->count <= 0) return false;

    free(track->checkpoints);
    track->checkpoints = (Checkpoint *)calloc((size_t)track->count, sizeof(Checkpoint));
    if (track->checkpoints == NULL) {
        track->checkpointCount = 0;
        return false;
    }
    track->checkpointCount = track->count;

    for (int i = 0; i < track->count; i++) {
        const TrackNode *node = &track->nodes[i];
        float dx = 0.0f, dy = 0.0f;
        if (track->routeClosed) {
            const TrackNode *next = &track->nodes[(i + 1) % track->count];
            dx = next->centerM.x - node->centerM.x;
            dy = next->centerM.y - node->centerM.y;
        } else {
            if (i + 1 < track->count) {
                const TrackNode *next = &track->nodes[i + 1];
                dx = next->centerM.x - node->centerM.x;
                dy = next->centerM.y - node->centerM.y;
            } else if (track->count > 1) {
                const TrackNode *prev = &track->nodes[i - 1];
                dx = node->centerM.x - prev->centerM.x;
                dy = node->centerM.y - prev->centerM.y;
            } else {
                dx = 0.0f;
                dy = 0.0f;
            }
        }
        const float len = sqrtf(dx * dx + dy * dy);

        track->checkpoints[i].centerM = node->centerM;
        track->checkpoints[i].halfWidthM = node->halfWidthM;
        track->checkpoints[i].required = true;
        /* A duplicated closing node leaves no direction to face; such a gate can never be
         * crossed, which is the same as the pre-existing behaviour for that degenerate case. */
        track->checkpoints[i].forwardUnit =
            (len < 1e-12f) ? (Vector2){ 0.0f, 0.0f } : (Vector2){ dx / len, dy / len };
    }
    return true;
}

void track_reset_progress_at(RacerProgress *progress, const TrackDefinition *track,
                             int startCheckpointIndex)
{
    if (progress == NULL) return;
    progress->lap = 0;
    progress->lapTimerS = 0.0f;
    progress->lastLapTimeS = 0.0f;
    progress->nextSector = 0;
    progress->sectorTimerS = 0.0f;
    progress->lastSectorTimeS = 0.0f;
    progress->lapInvalid = false;
    progress->lapArmed = false;
    progress->routeFinished = false;
    progress->lastCrossedIndex = -1;
    progress->ticksSinceCross = 1000;
    /* Route localization starts with no history: a car put back on the grid must not inherit
     * the segment it was on when it left, or its first tick would measure a lap-sized
     * longitudinal jump and read as going backwards. */
    memset(&progress->location, 0, sizeof(progress->location));
    progress->raceDistanceM = 0.0f;
    progress->wrongWay = false;
    progress->wrongWayTimerS = 0.0f;
    progress->progressBinM = 0.0f;
    progress->furthestProgressMLap = 0.0f;
    progress->lastProgressTick = 0;
    if (track == NULL || track->checkpointCount <= 0) {
        progress->nextCheckpoint = 0;
        progress->lapStartCheckpoint = 0;
        return;
    }
    if (startCheckpointIndex < 0 || startCheckpointIndex >= track->checkpointCount)
        startCheckpointIndex = 0;
    progress->lapStartCheckpoint = startCheckpointIndex;
    if (track->routeClosed) {
        progress->nextCheckpoint = (startCheckpointIndex + 1) % track->checkpointCount;
    } else {
        progress->nextCheckpoint = startCheckpointIndex + 1;
        if (progress->nextCheckpoint >= track->checkpointCount) {
            progress->nextCheckpoint = track->checkpointCount;
            progress->routeFinished = true;
        }
    }
}

bool track_start_pose_at(const TrackDefinition *track, int checkpointIndex, Vector2 *positionM,
                         float *headingRad)
{
    if (track == NULL) return false;
    if (track->hasStartFinish && checkpointIndex == 0) {
        if (positionM != NULL) *positionM = track->startFinish.centerM;
        if (headingRad != NULL)
            *headingRad =
                atan2f(track->startFinish.forwardUnit.y, track->startFinish.forwardUnit.x);
        return true;
    }
    if (track->checkpoints == NULL || track->checkpointCount <= 0) return false;
    if (checkpointIndex < 0 || checkpointIndex >= track->checkpointCount) return false;
    const Checkpoint *start = &track->checkpoints[checkpointIndex];
    if (positionM != NULL) *positionM = start->centerM;
    if (headingRad != NULL) *headingRad = atan2f(start->forwardUnit.y, start->forwardUnit.x);
    return true;
}

bool track_start_pose(const TrackDefinition *track, Vector2 *positionM, float *headingRad)
{
    if (track != NULL && track->hasStartFinish) {
        return track_start_pose_at(track, 0, positionM, headingRad);
    }
    return track_start_pose_at(track, 0, positionM, headingRad);
}

static uint32_t hash_f32(uint32_t h, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    const unsigned char *bytes = (const unsigned char *)&bits;
    for (size_t i = 0; i < sizeof(bits); i++) {
        h ^= (uint32_t)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

uint32_t track_geometry_hash(const TrackDefinition *track)
{
    if (track == NULL) return 0u;
    uint32_t h = FNV1A_OFFSET_BASIS;
    for (int i = 0; i < track->count; i++) {
        const TrackNode *n = &track->nodes[i];
        h = hash_f32(h, n->centerM.x);
        h = hash_f32(h, n->centerM.y);
        h = hash_f32(h, n->halfWidthM);
        h = hash_f32(h, n->runoffHalfWidthM);
        h = hash_f32(h, (float)n->surfaceId);
    }
    for (int i = 0; i < track->checkpointCount; i++) {
        const Checkpoint *c = &track->checkpoints[i];
        h = hash_f32(h, c->centerM.x);
        h = hash_f32(h, c->centerM.y);
        h = hash_f32(h, c->forwardUnit.x);
        h = hash_f32(h, c->forwardUnit.y);
        h = hash_f32(h, c->halfWidthM);
        h = hash_f32(h, c->required ? 1.0f : 0.0f);
    }
    h = hash_f32(h, track->routeClosed ? 1.0f : 0.0f);
    /* isParkingLot selects the barrier push normals in collision_world_build_from_track,
     * so a change to it must refresh the collision world. */
    h = hash_f32(h, track->isParkingLot ? 1.0f : 0.0f);
    for (int i = 0; i < track->sectorMarkerCount; i++) {
        const SectorMarker *s = &track->sectorMarkers[i];
        h = hash_f32(h, s->centerM.x);
        h = hash_f32(h, s->centerM.y);
        h = hash_f32(h, s->forwardUnit.x);
        h = hash_f32(h, s->forwardUnit.y);
        h = hash_f32(h, s->halfWidthM);
    }
    if (track->hasStartFinish) {
        h = hash_f32(h, track->startFinish.centerM.x);
        h = hash_f32(h, track->startFinish.centerM.y);
        h = hash_f32(h, track->startFinish.forwardUnit.x);
        h = hash_f32(h, track->startFinish.forwardUnit.y);
        h = hash_f32(h, track->startFinish.halfWidthM);
    }
    for (int i = 0; i < track->gridSlotCount; i++) {
        const GridSlot *g = &track->gridSlots[i];
        h = hash_f32(h, g->positionM.x);
        h = hash_f32(h, g->positionM.y);
        h = hash_f32(h, g->headingRad);
    }
    if (track->hasPitEntry) {
        h = hash_f32(h, track->pitEntry.centerM.x);
        h = hash_f32(h, track->pitEntry.centerM.y);
        h = hash_f32(h, track->pitEntry.forwardUnit.x);
        h = hash_f32(h, track->pitEntry.forwardUnit.y);
        h = hash_f32(h, track->pitEntry.halfWidthM);
    }
    if (track->hasPitExit) {
        h = hash_f32(h, track->pitExit.centerM.x);
        h = hash_f32(h, track->pitExit.centerM.y);
        h = hash_f32(h, track->pitExit.forwardUnit.x);
        h = hash_f32(h, track->pitExit.forwardUnit.y);
        h = hash_f32(h, track->pitExit.halfWidthM);
    }
    if (track->hasPitSpeedLine) {
        h = hash_f32(h, track->pitSpeedLine.centerM.x);
        h = hash_f32(h, track->pitSpeedLine.centerM.y);
        h = hash_f32(h, track->pitSpeedLine.forwardUnit.x);
        h = hash_f32(h, track->pitSpeedLine.forwardUnit.y);
        h = hash_f32(h, track->pitSpeedLine.halfWidthM);
    }
    for (int i = 0; i < track->serviceBoxCount; i++) {
        const ServiceBox *b = &track->serviceBoxes[i];
        h = hash_f32(h, b->minM.x);
        h = hash_f32(h, b->minM.y);
        h = hash_f32(h, b->maxM.x);
        h = hash_f32(h, b->maxM.y);
    }
    return h;
}

float track_length_m(const TrackDefinition *track)
{
    if (track == NULL || track->nodes == NULL || track->count <= 1) return 0.0f;
    float total = 0.0f;
    const int limit = track->routeClosed ? track->count : track->count - 1;
    for (int i = 0; i < limit; i++) {
        const Vector2 a = track->nodes[i].centerM;
        const Vector2 b = track->nodes[(i + 1) % track->count].centerM;
        total += sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    }
    return total;
}

/* Generic gate test for any forward-gated line (Checkpoint, SectorMarker, PitGate, StartFinish). */
static bool gate_crossed_generic(Vector2 centerM, Vector2 forwardUnit, float halfWidthM,
                                 Vector2 prevPosM, Vector2 currPosM)
{
    const Vector2 perp = { -forwardUnit.y, forwardUnit.x };
    const Vector2 gateA = { centerM.x + perp.x * halfWidthM, centerM.y + perp.y * halfWidthM };
    const Vector2 gateB = { centerM.x - perp.x * halfWidthM, centerM.y - perp.y * halfWidthM };
    const Vector2 motion = { currPosM.x - prevPosM.x, currPosM.y - prevPosM.y };
    if (motion.x * motion.x + motion.y * motion.y < 1e-24f) return false;
    if (motion.x * forwardUnit.x + motion.y * forwardUnit.y <= 0.0f) return false;
    return segments_intersect(prevPosM, currPosM, gateA, gateB);
}

/* Did the car's prev->curr motion pass through this gate, travelling the right way? */
static bool gate_crossed(const Checkpoint *gate, Vector2 prevPosM, Vector2 currPosM)
{
    if (gate == NULL) return false;
    return gate_crossed_generic(gate->centerM, gate->forwardUnit, gate->halfWidthM, prevPosM,
                                currPosM);
}

TrackCheckpointEvent track_update_checkpoints(const TrackDefinition *track,
                                              RacerProgress *progress, Vector2 prevPosM,
                                              Vector2 currPosM)
{
    TrackCheckpointEvent event;
    memset(&event, 0, sizeof(event));
    event.index = -1;

    if (track == NULL || progress == NULL || track->checkpoints == NULL ||
        track->checkpointCount <= 0)
        return event;

    /* Zeroed RacerProgress is the documented "start of an out-lap from gate 0" state. Its
     * lastCrossedIndex is 0 (from memset) but there has been no crossing yet, so treat it as
     * no recent crossing for debounce purposes. track_reset_progress_at() and
     * race_roster_spawn() set it to -1/1000. */
    if (progress->lastCrossedIndex == 0 && progress->ticksSinceCross == 0 &&
        progress->nextCheckpoint == 0) {
        progress->lastCrossedIndex = -1;
        progress->ticksSinceCross = 1000;
    }
    progress->ticksSinceCross++;
    if (progress->routeFinished) return event;

    /* The start/finish line is a lap boundary independent of the gate sequence. Latch its
     * crossing every tick (lapArmed) so a lap can close even when the line and the
     * lap-close gate are crossed in different ticks; an SF crossing on a tick that crosses
     * no gate must not be discarded. */
    if (track->hasStartFinish && track->routeClosed) {
        if (gate_crossed_generic(track->startFinish.centerM, track->startFinish.forwardUnit,
                                 track->startFinish.halfWidthM, prevPosM, currPosM)) {
            progress->lapArmed = true;
        }
    }

    const int expected = progress->nextCheckpoint;

    bool expectedCrossed = false;
    if (expected >= 0 && expected < track->checkpointCount) {
        if (progress->lastCrossedIndex != expected || progress->ticksSinceCross > 6) {
            if (gate_crossed(&track->checkpoints[expected], prevPosM, currPosM)) {
                expectedCrossed = true;
                event.crossed = true;
                event.index = expected;
            }
        }
    }
    if (!expectedCrossed) {
        for (int i = 0; i < track->checkpointCount; i++) {
            if (i == expected) continue;
            if (progress->lastCrossedIndex == i && progress->ticksSinceCross <= 6) continue;
            if (gate_crossed(&track->checkpoints[i], prevPosM, currPosM)) {
                event.crossed = true;
                event.index = i;
                event.outOfOrder = true;
                break;
            }
        }
    }

    if (!event.crossed) return event;
    if (event.outOfOrder) {
        if (event.index >= 0 && event.index < track->checkpointCount &&
            track->checkpoints[event.index].required) {
            progress->lapInvalid = true;
        }
        return event;
    }

    progress->lastCrossedIndex = event.index;
    progress->ticksSinceCross = 0;

    if (track->checkpointCount > 0) {
        const int firstGate = (progress->lapStartCheckpoint + 1) % track->checkpointCount;
        if (expected == firstGate) {
            progress->lapInvalid = false;
        }
    }

    bool lapCompletedViaRoute = false;

    if (track->routeClosed) {
        progress->nextCheckpoint++;
        if (progress->nextCheckpoint >= track->checkpointCount) {
            progress->nextCheckpoint = 0;
        }
        const int lapCloseNext = (progress->lapStartCheckpoint + 1) % track->checkpointCount;
        bool lapClosed =
            (progress->nextCheckpoint == lapCloseNext || track->checkpointCount == 1);
        if (track->hasStartFinish) {
            /* The SF line is the authoritative lap boundary: a wrapped route closes a lap
             * only when the line was crossed since the last close (lapArmed). Latching
             * decouples the crossing of the line from the crossing of the lap-close gate,
             * and the wrapped-route requirement means an SF that overlaps an early gate
             * cannot award an incomplete lap. The latch is consumed only by the close, so
             * an intermediate gate crossing does not disarm it. */
            lapClosed = lapClosed && progress->lapArmed;
            if (lapClosed) {
                progress->lapArmed = false;
            }
        }
        if (lapClosed) {
            lapCompletedViaRoute = true;
        }
    } else {
        progress->nextCheckpoint++;
        if (progress->nextCheckpoint >= track->checkpointCount) {
            progress->routeFinished = true;
            lapCompletedViaRoute = true;
            progress->nextCheckpoint = track->checkpointCount;
        }
    }

    if (lapCompletedViaRoute) {
        if (!progress->lapInvalid) {
            progress->lap++;
            event.lapCompleted = true;
            event.lapTimeS = progress->lapTimerS;
            progress->lastLapTimeS = progress->lapTimerS;
        }
        progress->lapTimerS = 0.0f;
        progress->lapInvalid = false;
    }

    return event;
}

TrackSectorEvent track_update_sectors(const TrackDefinition *track, RacerProgress *progress,
                                      Vector2 prevPosM, Vector2 currPosM)
{
    TrackSectorEvent event;
    memset(&event, 0, sizeof(event));
    event.index = -1;
    if (track == NULL || progress == NULL || track->sectorMarkers == NULL ||
        track->sectorMarkerCount <= 0)
        return event;
    const int expected = progress->nextSector;
    if (expected < 0 || expected >= track->sectorMarkerCount) return event;
    const SectorMarker *marker = &track->sectorMarkers[expected];
    if (gate_crossed_generic(marker->centerM, marker->forwardUnit, marker->halfWidthM, prevPosM,
                             currPosM)) {
        event.crossed = true;
        event.index = expected;
        event.sectorTimeS = progress->sectorTimerS;
        progress->lastSectorTimeS = progress->sectorTimerS;
        progress->sectorTimerS = 0.0f;
        progress->nextSector++;
        if (progress->nextSector >= track->sectorMarkerCount) {
            progress->nextSector = 0;
        }
    }
    return event;
}

bool track_validate_grid_slots(const TrackDefinition *track, char *error, size_t errorCap)
{
    if (track == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "no track");
        return false;
    }
    if (track->gridSlotCount <= 0) return true;
    if (track->gridSlotCount > TRACK_MAX_GRID_SLOTS) {
        if (error != NULL && errorCap > 0)
            snprintf(error, errorCap, "grid: too many slots (%d > %d)", track->gridSlotCount,
                     TRACK_MAX_GRID_SLOTS);
        return false;
    }
    for (int i = 0; i < track->gridSlotCount; i++) {
        const GridSlot *a = &track->gridSlots[i];
        if (!isfinite(a->positionM.x) || !isfinite(a->positionM.y) ||
            !isfinite(a->headingRad)) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "grid[%d]: non-finite pose", i);
            return false;
        }
        if (!track->isParkingLot) {
            float halfWidth = 0.0f;
            const float dist = track_distance_to_centerline_m(track, a->positionM, &halfWidth);
            if (dist > halfWidth + 5.0f) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap,
                             "grid[%d]: off-track placement (dist %.1f > width)", i,
                             (double)dist);
                return false;
            }
        }
        for (int j = i + 1; j < track->gridSlotCount; j++) {
            const GridSlot *b = &track->gridSlots[j];
            const float dx = a->positionM.x - b->positionM.x;
            const float dy = a->positionM.y - b->positionM.y;
            const float distSq = dx * dx + dy * dy;
            if (distSq < TRACK_GRID_MIN_SPACING_M * TRACK_GRID_MIN_SPACING_M) {
                if (error != NULL && errorCap > 0)
                    snprintf(error, errorCap, "grid: slots %d and %d overlap (%.1f m apart)", i,
                             j, (double)sqrtf(distSq));
                return false;
            }
        }
    }
    return true;
}

bool track_grid_slot_pose(const TrackDefinition *track, int slotIndex, Vector2 *positionM,
                          float *headingRad)
{
    if (track == NULL || track->gridSlots == NULL || slotIndex < 0 ||
        slotIndex >= track->gridSlotCount)
        return false;
    const GridSlot *slot = &track->gridSlots[slotIndex];
    if (positionM != NULL) *positionM = slot->positionM;
    if (headingRad != NULL) *headingRad = slot->headingRad;
    return true;
}

bool track_pit_has_geometry(const TrackDefinition *track)
{
    if (track == NULL) return false;
    return track->hasPitEntry || track->hasPitExit || track->hasPitSpeedLine ||
           track->serviceBoxCount > 0;
}

bool track_point_in_service_box(const TrackDefinition *track, Vector2 pointM)
{
    if (track == NULL || track->serviceBoxes == NULL || track->serviceBoxCount <= 0)
        return false;
    for (int i = 0; i < track->serviceBoxCount; i++) {
        const ServiceBox *b = &track->serviceBoxes[i];
        if (pointM.x >= b->minM.x && pointM.x <= b->maxM.x && pointM.y >= b->minM.y &&
            pointM.y <= b->maxM.y)
            return true;
    }
    return false;
}

bool track_is_closed(const TrackDefinition *track)
{
    if (track == NULL) return false;
    return track->routeClosed;
}

bool track_is_open(const TrackDefinition *track)
{
    if (track == NULL) return false;
    return !track->routeClosed;
}

bool track_has_required_markers_for_mode(const TrackDefinition *track, const char *mode,
                                         char *error, size_t errorCap)
{
    if (track == NULL || mode == NULL) {
        if (error != NULL && errorCap > 0) snprintf(error, errorCap, "no track or mode");
        return false;
    }
    if (strcmp(mode, "race") == 0) {
        if (track->gridSlotCount <= 1) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "race: need at least 2 grid slots (got %d)",
                         track->gridSlotCount);
            return false;
        }
        if (!track->routeClosed && track->checkpointCount < 2) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "race: open route needs checkpoints");
            return false;
        }
        return true;
    }
    if (strcmp(mode, "time_trial") == 0) {
        if (!track->hasStartFinish && track->checkpointCount <= 0) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "time_trial: need start/finish or checkpoints");
            return false;
        }
        return true;
    }
    if (strcmp(mode, "sprint") == 0) {
        if (track->routeClosed) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "sprint: open route required, got closed");
            return false;
        }
        if (track->checkpointCount < 2) {
            if (error != NULL && errorCap > 0)
                snprintf(error, errorCap, "sprint: need checkpoints");
            return false;
        }
        return true;
    }
    return true;
}
