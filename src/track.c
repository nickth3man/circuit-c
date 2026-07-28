/*
 * track.c — oval track geometry and surface queries.
 *
 * Hand-authors a stadium-shaped oval of 24 nodes. The bottom straight passes through the
 * world origin (0,0), so the car's default start position is on asphalt. The nodes array
 * is a literal in this file copied onto the heap by track_init, so the pointer in Game
 * survives hot reloads (it points to heap memory, not module static data).
 */
#include "track.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* --------------- stadium geometry --------------------------------------------------------
 *
 * A ~200 m × 80 m stadium (rounded rectangle) with the bottom straight at y = 0. The world
 * origin (0,0) lies on the centreline, so the car starts on asphalt.
 *
 *   Bottom straight : y = 0,  x ∈ [-60, +60]   (6 nodes, left → right)
 *   Right semicircle: centre (60, 40), r = 40   (6 nodes, bottom → top)
 *   Top straight    : y = 80, x ∈ [+60, -60]   (6 nodes, right → left)
 *   Left semicircle : centre (-60, 40), r = 40  (6 nodes, top → bottom)
 *
 * Total: 24 nodes. halfWidthM = 8 m → a 16 m wide driving surface.
 */
#define TRACK_STADIUM_COUNT   24
#define TRACK_HALF_WIDTH_M    8.0f

#define STADIUM_STRAIGHT_X_MAX 60.0f
#define STADIUM_CIRCLE_RADIUS  40.0f
#define STADIUM_TOP_Y          80.0f

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
 * to the segment index (the i in nodes[i]→nodes[(i+1)%count]). *closestIdx is untouched
 * when closestIdx is NULL. */
static float nearest_centerline_distance_sq(const TrackNode *nodes, int count, Vector2 point,
                                            int *closestIdx)
{
    float best = 1e30f;
    int   bestIdx = 0;
    for (int i = 0; i < count; i++) {
        const int j = (i + 1) % count;
        const float dSq = point_to_segment_sq(point, nodes[i].centerM, nodes[j].centerM);
        if (dSq < best) { best = dSq; bestIdx = i; }
    }
    if (closestIdx != NULL) *closestIdx = bestIdx;
    return best;
}

/* --------------- public API -------------------------------------------------------------- */

void track_init(Track *track)
{
    if (track == NULL) return;
    /* Defensive: free any previously-allocated state first. */
    track_free(track);

    track->nodes = (TrackNode *)calloc(TRACK_STADIUM_COUNT, sizeof(TrackNode));
    if (track->nodes == NULL) {
        track->count = 0;
        return;
    }

    track->count = TRACK_STADIUM_COUNT;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->nextCheckpoint = 0;
    track->lap = 0;
    track->lapTimerS = 0.0f;
    track->lastLapTimeS = 0.0f;

    /* Helper: assign one node. */
    const float hw = TRACK_HALF_WIDTH_M;
    const SurfaceId sid = SURFACE_ASPHALT;

    #define SET_NODE(idx, xval, yval) do { \
        track->nodes[idx].centerM.x = (xval); \
        track->nodes[idx].centerM.y = (yval); \
        track->nodes[idx].halfWidthM = hw; \
        track->nodes[idx].surfaceId = sid; \
    } while (0)

    /* Bottom straight: y = 0, x from -60 to +60, 6 nodes.
     * node indices 0 .. 5  (left → right). */
    for (int i = 0; i < 6; i++) {
        const float x = -STADIUM_STRAIGHT_X_MAX +
                         (2.0f * STADIUM_STRAIGHT_X_MAX) * (float)i / 5.0f;
        SET_NODE(i, x, 0.0f);
    }

    /* Right semicircle: centre (60, 40), radius 40, 6 nodes.
     * Angle from -π/2 (bottom) to +π/2 (top).  node indices 6 .. 11. */
    for (int i = 0; i < 6; i++) {
        const float theta = -3.14159265358979323846f * 0.5f +
                            3.14159265358979323846f * (float)i / 5.0f;
        SET_NODE(6 + i,
                 STADIUM_STRAIGHT_X_MAX + STADIUM_CIRCLE_RADIUS * cosf(theta),
                 STADIUM_CIRCLE_RADIUS + STADIUM_CIRCLE_RADIUS * sinf(theta));
    }

    /* Top straight: y = 80, x from +60 to -60, 6 nodes.
     * node indices 12 .. 17  (right → left). */
    for (int i = 0; i < 6; i++) {
        const float x = STADIUM_STRAIGHT_X_MAX -
                         (2.0f * STADIUM_STRAIGHT_X_MAX) * (float)i / 5.0f;
        SET_NODE(12 + i, x, STADIUM_TOP_Y);
    }

    /* Left semicircle: centre (-60, 40), radius 40, 6 nodes.
     * Angle from +π/2 (top) to +3π/2 (bottom).  node indices 18 .. 23. */
    for (int i = 0; i < 6; i++) {
        const float theta = 3.14159265358979323846f * 0.5f +
                            3.14159265358979323846f * (float)i / 5.0f;
        SET_NODE(18 + i,
                 -STADIUM_STRAIGHT_X_MAX + STADIUM_CIRCLE_RADIUS * cosf(theta),
                 STADIUM_CIRCLE_RADIUS + STADIUM_CIRCLE_RADIUS * sinf(theta));
    }

    #undef SET_NODE
}

void track_free(Track *track)
{
    if (track == NULL) return;
    free(track->nodes);
    track->nodes = NULL;
    track->count = 0;
    track->offTrackSurfaceId = SURFACE_ASPHALT;
    track->nextCheckpoint = 0;
    track->lap = 0;
    track->lapTimerS = 0.0f;
    track->lastLapTimeS = 0.0f;
}

SurfaceId Track_SurfaceAt(const Track *track, Vector2 pointM)
{
    /* Headless tests and scenarios that don't initialise a track hit this safe default, so
     * existing scenarios remain on asphalt and their CSVs are unchanged. */
    if (track == NULL || track->nodes == NULL || track->count <= 0) {
        return SURFACE_ASPHALT;
    }

    int closestIdx = 0;
    const float dSq = nearest_centerline_distance_sq(track->nodes, track->count, pointM,
                                                     &closestIdx);
    const TrackNode *seg = &track->nodes[closestIdx];

    if (dSq <= seg->halfWidthM * seg->halfWidthM) {
        return seg->surfaceId;
    }
    return track->offTrackSurfaceId;
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
    return cross_z((Vector2){ b.x - a.x, b.y - a.y },
                   (Vector2){ c.x - a.x, c.y - a.y });
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

bool track_update_checkpoints(Track *track, Vector2 prevPosM, Vector2 currPosM)
{
    if (track == NULL || track->nodes == NULL || track->count <= 1) return false;

    const int idx = track->nextCheckpoint;
    if (idx < 0 || idx >= track->count) return false;

    /* Build the gate at node idx. The gate is perpendicular to the centreline direction
     * through this node, spanning the full track width. */
    const int nextIdx = (idx + 1) % track->count;
    const TrackNode *node = &track->nodes[idx];
    const TrackNode *nextNode = &track->nodes[nextIdx];

    const float dx = nextNode->centerM.x - node->centerM.x;
    const float dy = nextNode->centerM.y - node->centerM.y;
    const float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-12f) return false;

    const float hw = node->halfWidthM;
    const Vector2 forward = { dx / len, dy / len };
    /* Left perpendicular of the forward direction: rotating (dx,dy) by +90° gives (-dy, dx). */
    const Vector2 perp = { -forward.y, forward.x };

    /* Gate endpoints span from centre ± halfWidth * perp. */
    const Vector2 gateA = { node->centerM.x + perp.x * hw,
                             node->centerM.y + perp.y * hw };
    const Vector2 gateB = { node->centerM.x - perp.x * hw,
                             node->centerM.y - perp.y * hw };

    /* Forward-only: the car's motion direction must have a positive component along the
     * track's forward direction so reverse driving cannot advance checkpoints. */
    const float motionX = currPosM.x - prevPosM.x;
    const float motionY = currPosM.y - prevPosM.y;
    const float motionLen = sqrtf(motionX * motionX + motionY * motionY);
    if (motionLen < 1e-12f) return false;

    const float dotForward = motionX * forward.x + motionY * forward.y;
    if (dotForward <= 0.0f) return false;

    if (!segments_intersect(prevPosM, currPosM, gateA, gateB)) return false;

    /* Valid crossing: advance. */
    track->nextCheckpoint++;
    if (track->nextCheckpoint >= track->count) {
        /* Wrapped: completed a lap. */
        track->nextCheckpoint = 0;
        track->lap++;
        track->lastLapTimeS = track->lapTimerS;
        track->lapTimerS = 0.0f;
    }

    return true;
}
