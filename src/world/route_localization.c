/*
 * route_localization.c — where one entrant is on the route, and what follows from that.
 *
 * WHY THIS IS NOT IN track.c. track.c owns authored geometry and the gate tests. This file
 * owns the one thing every other subsystem wanted to work out for itself: given a pose, which
 * bit of the route is that, how far round is it, which way is it facing, and how much do we
 * believe the answer. Timing, wrong-way, race distance and rejoin candidates are all derived
 * from that single result, so they cannot disagree about where a car is.
 *
 * CONTINUITY, NOT CLEVERNESS. The hard case is a route that runs beside itself — a
 * figure-eight crossing, a pit lane parallel to the main straight, the two halves of a
 * hairpin. A nearest-segment scan answers those with whichever strand happens to be a
 * millimetre closer, which flips between ticks and takes a car's lap count with it. Searching
 * a window around last tick's answer removes the ambiguity entirely, because the car cannot
 * have moved to the other strand in one 120 Hz tick. The global scan is kept as an explicit
 * fallback for the cases where continuity genuinely does not apply.
 *
 * DETERMINISM. Every comparison is a strict `<` over a fixed scan order, so equidistant
 * candidates resolve identically on every platform and in every run. No comparison depends on
 * a pointer, an iteration accident, or the order entrants were spawned in.
 *
 * This translation unit calls no raylib function; Vector2 from the header is fine.
 */
#include <math.h>
#include <string.h>

#include "core/math_utils.h"
#include "world/track.h"

/* --------------- geometry helpers --------------------------------------------------------- */

/* Number of centreline segments: a closed route wraps its last node onto its first, an open
 * one does not. Zero means "no route to localize against". */
static int segment_count(const TrackDefinition *track)
{
    if (track == NULL || track->nodes == NULL || track->count < 2) return 0;
    return track->routeClosed ? track->count : track->count - 1;
}

/* The two endpoints of segment `index`. The caller has already bounded `index`. */
static void segment_endpoints(const TrackDefinition *track, int index, Vector2 *aM, Vector2 *bM)
{
    *aM = track->nodes[index].centerM;
    *bM = track->nodes[(index + 1) % track->count].centerM;
}

/* Closest point on the finite segment a->b to p. Writes the clamped parameter to *t and the
 * point to *closest, and returns the distance in metres. */
static float closest_on_segment(Vector2 aM, Vector2 bM, Vector2 pM, float *t, Vector2 *closest)
{
    const float dx = bM.x - aM.x;
    const float dy = bM.y - aM.y;
    const float lenSq = dx * dx + dy * dy;
    float param = 0.0f;
    if (lenSq >= 1e-12f) {
        param = ((pM.x - aM.x) * dx + (pM.y - aM.y) * dy) / lenSq;
        param = clampf(param, 0.0f, 1.0f);
    }
    const Vector2 point = { aM.x + param * dx, aM.y + param * dy };
    *t = param;
    *closest = point;
    const float ex = pM.x - point.x;
    const float ey = pM.y - point.y;
    return sqrtf(ex * ex + ey * ey);
}

/* Length of segment `index`, metres. */
static float segment_length_m(const TrackDefinition *track, int index)
{
    Vector2 aM, bM;
    segment_endpoints(track, index, &aM, &bM);
    const float dx = bM.x - aM.x;
    const float dy = bM.y - aM.y;
    return sqrtf(dx * dx + dy * dy);
}

/* Arc length from node 0 to the point (segmentIndex, t).
 *
 * Summed on demand rather than cached in TrackRuntime: the authored layouts have 170 nodes and
 * this runs once per entrant per tick, which is not a cost worth a rebuildable cache and its
 * invalidation rules. OVERHAUL_REFERENCE puts a spatial index behind measured need for the
 * same reason. */
static float arc_length_to(const TrackDefinition *track, int segmentIndex, float t)
{
    float total = 0.0f;
    for (int i = 0; i < segmentIndex; i++) {
        total += segment_length_m(track, i);
    }
    return total + t * segment_length_m(track, segmentIndex);
}

/* Build the full result for a segment/parameter that has already been chosen. */
static RouteLocation make_location(const TrackDefinition *track, int segmentIndex, float t,
                                   Vector2 pointM, Vector2 posM, float headingRad)
{
    RouteLocation loc;
    memset(&loc, 0, sizeof(loc));

    Vector2 aM, bM;
    segment_endpoints(track, segmentIndex, &aM, &bM);
    const float dx = bM.x - aM.x;
    const float dy = bM.y - aM.y;
    const float len = sqrtf(dx * dx + dy * dy);
    /* A duplicated node leaves no direction. The loader rejects coincident consecutive nodes,
     * so this is a guard against a hand-built ribbon rather than against authored content. */
    const Vector2 forward =
        (len < 1e-6f) ? (Vector2){ 1.0f, 0.0f } : (Vector2){ dx / len, dy / len };

    loc.valid = true;
    loc.segmentIndex = segmentIndex;
    loc.segmentT = t;
    loc.pointM = pointM;
    loc.forwardUnit = forward;
    loc.longitudinalM = arc_length_to(track, segmentIndex, t);
    /* Left normal of (fx, fy) is (-fy, fx), matching the body-Y-is-left convention the AI's
     * cross-track error already uses. */
    loc.lateralM = (posM.x - pointM.x) * (-forward.y) + (posM.y - pointM.y) * forward.x;
    loc.headingErrorRad = wrap_angle(headingRad - atan2f(forward.y, forward.x));

    /*
     * Containment is measured from the FULL distance to the closest point, not from the
     * lateral component.
     *
     * They are the same number whenever the projection lands inside the segment, which is the
     * ordinary case. They part company when the closest point is a clamped endpoint: there the
     * displacement can be almost entirely along the segment, leaving a lateral component of
     * nearly zero for a pose that is nowhere near the route. Driving straight on past an open
     * route's final node is exactly that, and judging it by lateral offset alone would report a
     * car half a kilometre into the scenery as on the racing surface with full confidence.
     *
     * `lateralM` keeps its own job: which SIDE of the route the car is on, and how far off line
     * it is while it is genuinely alongside the route.
     */
    const float offRouteM = sqrtf((posM.x - pointM.x) * (posM.x - pointM.x) +
                                  (posM.y - pointM.y) * (posM.y - pointM.y));
    const TrackNode *node = &track->nodes[segmentIndex];
    const float halfWidthM = node->halfWidthM;
    const float barrierM = track_node_barrier_half_width(node);
    loc.onRoute = (offRouteM <= halfWidthM);
    if (loc.onRoute) {
        loc.confidence = 1.0f;
    } else if (barrierM > halfWidthM) {
        /* Linear across the runoff band. A node with no runoff band has nowhere to decay
         * through, so its confidence steps from 1 to 0 at the track edge. */
        loc.confidence = clampf((barrierM - offRouteM) / (barrierM - halfWidthM), 0.0f, 1.0f);
    } else {
        loc.confidence = 0.0f;
    }
    return loc;
}

/* --------------- public API --------------------------------------------------------------- */

RouteLocation route_localize_global(const TrackDefinition *track, Vector2 posM,
                                    float headingRad)
{
    RouteLocation loc;
    memset(&loc, 0, sizeof(loc));
    const int segments = segment_count(track);
    if (segments <= 0) return loc;

    int bestIndex = 0;
    float bestT = 0.0f;
    Vector2 bestPoint = track->nodes[0].centerM;
    float bestDistM = INFINITY;
    /* Ascending index with a strict `<`: the lowest index wins an exact tie. */
    for (int i = 0; i < segments; i++) {
        Vector2 aM, bM, point;
        float t;
        segment_endpoints(track, i, &aM, &bM);
        const float d = closest_on_segment(aM, bM, posM, &t, &point);
        if (d < bestDistM) {
            bestDistM = d;
            bestIndex = i;
            bestT = t;
            bestPoint = point;
        }
    }
    return make_location(track, bestIndex, bestT, bestPoint, posM, headingRad);
}

/* One candidate segment considered by the windowed search. Keeps the walk below readable
 * without a second copy of the "is this better" comparison. */
typedef struct {
    int index;
    float t;
    Vector2 pointM;
    float distM;
} RouteCandidate;

/* Strict `<`: the first candidate offered at a given distance keeps the win, and the walk
 * offers them nearest-to-previous first. */
static void consider_segment(const TrackDefinition *track, int index, Vector2 posM,
                             RouteCandidate *best)
{
    Vector2 aM, bM, point;
    float t;
    segment_endpoints(track, index, &aM, &bM);
    const float d = closest_on_segment(aM, bM, posM, &t, &point);
    if (d < best->distM) {
        best->distM = d;
        best->index = index;
        best->t = t;
        best->pointM = point;
    }
}

RouteLocation route_localize_near(const TrackDefinition *track, const RouteLocation *previous,
                                  Vector2 posM, float headingRad)
{
    const int segments = segment_count(track);
    if (segments <= 0) {
        RouteLocation none;
        memset(&none, 0, sizeof(none));
        return none;
    }
    if (previous == NULL || !previous->valid || previous->segmentIndex < 0 ||
        previous->segmentIndex >= segments) {
        return route_localize_global(track, posM, headingRad);
    }
    const int hintSegment = previous->segmentIndex;

    RouteCandidate best = { -1, 0.0f, { 0.0f, 0.0f }, INFINITY };
    consider_segment(track, hintSegment, posM, &best);

    /*
     * Walk outwards from the previous segment, one step forward then one step back, until each
     * direction has spent ROUTE_LOCALIZE_WINDOW_M of arc or run out of route. Visiting in that
     * order is what makes an exact tie fall to the candidate nearest where the car already was,
     * and forward of it before behind it.
     *
     * The budgets are measured from the previous POINT to the nearest end of the candidate, not
     * from the start of the previous segment. Spending the whole of the previous segment before
     * looking at its neighbour would mean that on any layout whose nodes are further apart than
     * the window — the parking lot's 400 m and 300 m edges, most obviously — a car could never
     * see the segment it is about to turn onto. It would stay clamped to the corner it had just
     * reached until it drifted outside the acceptance corridor, freezing its longitudinal
     * progress and then jumping it several metres when the global fallback finally fired.
     */
    const float hintLengthM = segment_length_m(track, hintSegment);
    const float hintT = clampf(previous->segmentT, 0.0f, 1.0f);
    int forwardIndex = hintSegment;
    int backwardIndex = hintSegment;
    float forwardArcM = (1.0f - hintT) * hintLengthM;
    float backwardArcM = hintT * hintLengthM;
    bool forwardOpen = true;
    bool backwardOpen = true;
    for (int step = 1; step < segments && (forwardOpen || backwardOpen); step++) {
        if (forwardOpen) {
            int next = forwardIndex + 1;
            if (next >= segments) next = track->routeClosed ? 0 : -1;
            if (next < 0 || next == hintSegment || forwardArcM > ROUTE_LOCALIZE_WINDOW_M) {
                forwardOpen = false;
            } else {
                forwardIndex = next;
                consider_segment(track, forwardIndex, posM, &best);
                forwardArcM += segment_length_m(track, forwardIndex);
            }
        }
        if (backwardOpen) {
            int next = backwardIndex - 1;
            if (next < 0) next = track->routeClosed ? segments - 1 : -1;
            if (next < 0 || next == hintSegment || backwardArcM > ROUTE_LOCALIZE_WINDOW_M) {
                backwardOpen = false;
            } else {
                backwardIndex = next;
                consider_segment(track, backwardIndex, posM, &best);
                backwardArcM += segment_length_m(track, backwardIndex);
            }
        }
    }

    const int bestIndex = best.index;
    const float bestT = best.t;
    const Vector2 bestPoint = best.pointM;
    const float bestDistM = best.distM;

    /* bestIndex stays -1 only when every candidate distance was NaN, which needs a non-finite
     * pose; the global scan below then answers with the route's first segment rather than
     * reading an unset index. */
    if (bestIndex >= 0) {
        const float acceptM = track_node_barrier_half_width(&track->nodes[bestIndex]) +
                              ROUTE_LOCALIZE_ACCEPT_MARGIN_M;
        if (bestDistM <= acceptM) {
            return make_location(track, bestIndex, bestT, bestPoint, posM, headingRad);
        }
    }
    /* Continuity is no longer credible: one full scan, then done. */
    return route_localize_global(track, posM, headingRad);
}

/*
 * Signed longitudinal movement between two localizations, metres.
 *
 * On a closed route the shorter of the two ways round is the true one: a car crossing the
 * start/finish seam moves a few centimetres forward, not a lap backwards. On an open route
 * there is no seam and the plain difference is already right.
 */
static float longitudinal_delta_m(const TrackDefinition *track, float prevM, float currM)
{
    float delta = currM - prevM;
    if (!track->routeClosed) return delta;
    const float lengthM = track_length_m(track);
    if (lengthM <= 0.0f) return delta;
    const float halfM = 0.5f * lengthM;
    if (delta > halfM)
        delta -= lengthM;
    else if (delta < -halfM)
        delta += lengthM;
    return delta;
}

TrackProgressEvent track_update_progress(const TrackDefinition *track, RacerProgress *progress,
                                         Vector2 prevPosM, Vector2 currPosM, float headingRad,
                                         float dt)
{
    TrackProgressEvent event;
    memset(&event, 0, sizeof(event));
    event.checkpoint.index = -1;
    event.sector.index = -1;
    if (progress == NULL) return event;
    event.wrongWay = progress->wrongWay;
    if (track == NULL) return event;

    /* 1. Localize first, so the gates below and every consumer of this tick agree on one
     *    position. The previous answer is the continuity hint; a zeroed one is invalid and
     *    asks for a global scan, which is what a freshly reset racer gets. */
    const RouteLocation previous = progress->location;
    const RouteLocation current = route_localize_near(track, &previous, currPosM, headingRad);

    /* 2. Route and sector gates, unchanged: an exact swept-line test is already exact. */
    event.checkpoint = track_update_checkpoints(track, progress, prevPosM, currPosM);
    event.sector = track_update_sectors(track, progress, prevPosM, currPosM);

    if (!current.valid) {
        progress->location = current;
        return event;
    }

    /* 3. Longitudinal movement, which both the distance and the wrong-way test read. */
    const float deltaM = previous.valid ? longitudinal_delta_m(track, previous.longitudinalM,
                                                               current.longitudinalM)
                                        : 0.0f;
    progress->raceDistanceM += deltaM;

    /* 4. Wrong-way, latched. Both conditions must hold: facing back up the route AND actually
     *    losing ground. A spin satisfies the angle for a fraction of a second while still
     *    carrying the car forwards, and a car stopped against a barrier satisfies neither. */
    const bool facingBackward = fabsf(current.headingErrorRad) > ROUTE_WRONG_WAY_ENTER_RAD;
    const bool stillFacingBackward = fabsf(current.headingErrorRad) > ROUTE_WRONG_WAY_EXIT_RAD;
    const bool losingGround = deltaM < -ROUTE_WRONG_WAY_BACKWARD_EPS_M;
    const bool arming = previous.valid && losingGround &&
                        (progress->wrongWay ? stillFacingBackward : facingBackward);
    const float step = (dt > 0.0f) ? dt : 0.0f;
    if (arming) {
        progress->wrongWayTimerS =
            minf(progress->wrongWayTimerS + step, ROUTE_WRONG_WAY_HOLD_S);
    } else {
        progress->wrongWayTimerS = maxf(progress->wrongWayTimerS - step, 0.0f);
    }
    const bool wasWrongWay = progress->wrongWay;
    if (progress->wrongWayTimerS >= ROUTE_WRONG_WAY_HOLD_S) {
        progress->wrongWay = true;
    } else if (progress->wrongWayTimerS <= 0.0f) {
        progress->wrongWay = false;
    }
    /* Anywhere between the two bounds the previous answer stands. That band is the hysteresis. */

    progress->location = current;
    event.wrongWay = progress->wrongWay;
    event.wrongWayChanged = (progress->wrongWay != wasWrongWay);
    return event;
}
