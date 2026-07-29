/*
 * collision.c — vehicle-track capsule collision with swept substeps and impulse response.
 *
 * Physics conventions (docs/SPEC.md):
 *   - SI units throughout.
 *   - Body X forward, body Y left.
 *   - Heading counterclockwise from world +X.
 *   - Yaw rate counterclockwise positive.
 *   - Impulses are computed in world frame, then rotated back to body frame.
 *
 * Sign convention for each equation is documented inline.
 *
 * This translation unit calls no raylib function.
 */

#include "world/collision.h"

#include <math.h>
#include <string.h>

/* ====================================================================================== */
/*  Boundary polyline helpers                                                              */
/* ====================================================================================== */

/* Closest point on segment a→b to point p, returned by value. */
static Vector2 closest_point_on_segment(Vector2 p, Vector2 a, Vector2 b)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float lenSq = dx * dx + dy * dy;
    if (lenSq < 1e-12f) return a;

    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (Vector2){ a.x + t * dx, a.y + t * dy };
}

/* Squared distance from point p to segment a→b. */
static float point_segment_sq(Vector2 p, Vector2 a, Vector2 b)
{
    const Vector2 q = closest_point_on_segment(p, a, b);
    const float ex = p.x - q.x;
    const float ey = p.y - q.y;
    return ex * ex + ey * ey;
}

/* ====================================================================================== */
/*  Capsule circle positions                                                               */
/* ====================================================================================== */

/* Body-frame position of the front capsule circle. */
static Vector2 body_front_position(float cgToFrontM)
{
    return (Vector2){ cgToFrontM, 0.0f };
}

/* Body-frame position of the rear capsule circle. */
static Vector2 body_rear_position(float cgToRearM)
{
    return (Vector2){ -cgToRearM, 0.0f };
}

/* Rotate a body-frame point into world frame given the CG position and heading. */
static Vector2 world_from_body(Vector2 bodyPoint, Vector2 cgWorld, float headingRad)
{
    const float cosH = cosf(headingRad);
    const float sinH = sinf(headingRad);
    return (Vector2){
        cgWorld.x + bodyPoint.x * cosH - bodyPoint.y * sinH,
        cgWorld.y + bodyPoint.x * sinH + bodyPoint.y * cosH
    };
}

/* ====================================================================================== */
/*  Velocity helpers                                                                       */
/* ====================================================================================== */

/* World-frame CG velocity from body-frame components. */
static Vector2 world_velocity(float vxBody, float vyBody, float headingRad)
{
    const float cosH = cosf(headingRad);
    const float sinH = sinf(headingRad);
    return (Vector2){
        vxBody * cosH - vyBody * sinH,
        vxBody * sinH + vyBody * cosH
    };
}

/* World-frame contact-point velocity: v_CG + ω × r.  In 2D, ω × r = ω * (-r.y, r.x). */
static Vector2 contact_velocity_world(Vector2 vCgWorld, float yawRateRadS, Vector2 r)
{
    return (Vector2){
        vCgWorld.x - yawRateRadS * r.y,
        vCgWorld.y + yawRateRadS * r.x
    };
}

/* World → body rotation of a velocity vector. */
/* Body X = dot(v_world, (cosH, sinH))   (forward)
 * Body Y = dot(v_world, (-sinH, cosH))  (left)
 */
static void world_vel_to_body(Vector2 vWorld, float headingRad,
                               float *vxBodyOut, float *vyBodyOut)
{
    *vxBodyOut =  vWorld.x * cosf(headingRad) + vWorld.y * sinf(headingRad);
    *vyBodyOut = -vWorld.x * sinf(headingRad) + vWorld.y * cosf(headingRad);
}

/* ====================================================================================== */
/*  Interpolation helpers for swept test                                                   */
/* ====================================================================================== */

static float lerp_angle_simple(float a, float b, float t)
{
    /* Shortest-path interpolation. */
    float delta = b - a;
    /* Wrap delta to [-PI, PI). */
    if (delta > 3.14159265358979323846f)  delta -= 2.0f * 3.14159265358979323846f;
    if (delta < -3.14159265358979323846f) delta += 2.0f * 3.14159265358979323846f;
    return a + delta * t;
}

static Vector2 lerp_vec(Vector2 a, Vector2 b, float t)
{
    return (Vector2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}

/* ====================================================================================== */
/*  Collision resolution — the main function                                               */
/* ====================================================================================== */

int collision_resolve_track(const VehicleSpec *spec,
                            VehicleState *state,
                            VehicleRenderState *renderState,
                            const Track *track,
                            float *crashLockoutTimerS)
{
    if (spec == NULL || state == NULL || renderState == NULL ||
        track == NULL || track->nodes == NULL || track->count < 2) {
        return 0;
    }

    const float radiusM  = spec->bodyHalfWidthM;
    const float radiusSq = radiusM * radiusM;
    if (radiusM <= 0.0f) return 0;

    const float rHalf = spec->collisionRestitution;
    const float muC   = spec->collisionFriction;

    /* Capsule circle body-frame positions. */
    const Vector2 bFront = body_front_position(spec->cgToFrontM);
    const Vector2 bRear  = body_rear_position(spec->cgToRearM);

    /* Start-of-tick and end-of-tick transforms. */
    const Vector2 prevPos = renderState->prevPositionM;
    const Vector2 currPos = renderState->currPositionM;
    const float   prevHdg = renderState->prevHeadingRad;
    const float   currHdg = renderState->currHeadingRad;

    /* World-frame CG velocity at start of tick (before physics integrated this step). */
    const Vector2 vCgWorld = world_velocity(state->velocityLongitudinalMps,
                                             state->velocityLateralMps,
                                             currHdg);

    /* Build boundary segments from the track centreline.
     *
     * For each centreline segment i→j (j = (i+1)%count), create two boundary segments:
     *   - Left boundary:  centreline + halfWidth * (-dir.y, +dir.x)  (the +90° offset)
     *   - Right boundary: centreline - halfWidth * (-dir.y, +dir.x)  (the -90° offset)
     *
     * The "push" normal (pointing back onto the track) is:
     *   - Left barrier:  (+dir.y, -dir.x)   — the -90° direction, back toward centreline
     *   - Right barrier: (-dir.y, +dir.x)   — the +90° direction, back toward centreline
     */

    /* Substep sweep: check in small increments from prev→curr to catch the earliest contact. */
    /* We check t = 0/COLLISION_SUBSTEPS through (COLLISION_SUBSTEPS-1)/COLLISION_SUBSTEPS,
     * which is 6 substeps. We use substeps-1 for interpolation because t=1 is the end state
     * that physics already produced — if the car ended inside a barrier, substep t < 1 should
     * have caught it. */
    for (int sub = 0; sub < COLLISION_SUBSTEPS; sub++) {
        const float t = (float)sub / (float)COLLISION_SUBSTEPS;
        const Vector2 pos = lerp_vec(prevPos, currPos, t);
        const float   hdg = lerp_angle_simple(prevHdg, currHdg, t);

        /* World positions of the two capsule circles. */
        const Vector2 frontWorld = world_from_body(bFront, pos, hdg);
        const Vector2 rearWorld  = world_from_body(bRear,  pos, hdg);

        /* Iterate over every centreline segment and check both boundaries. */
        const int n = track->count;
        for (int i = 0; i < n; i++) {
            const int j = (i + 1) % n;
            const TrackNode *ni = &track->nodes[i];
            const TrackNode *nj = &track->nodes[j];

            /* Segment direction. */
            const float segDx = nj->centerM.x - ni->centerM.x;
            const float segDy = nj->centerM.y - ni->centerM.y;
            const float segLen = sqrtf(segDx * segDx + segDy * segDy);
            if (segLen < 1e-12f) continue;

            const float invLen = 1.0f / segLen;
            const Vector2 dir = { segDx * invLen, segDy * invLen };
            /* Left perpendicular: rotating +90°, i.e. (-dir.y, +dir.x). */
            const Vector2 perp = { -dir.y, dir.x };

            /* Interpolate half-width between the two nodes. */
            const float hw = ni->halfWidthM + (nj->halfWidthM - ni->halfWidthM) * 0.5f;

            /* --- Left barrier (the "outside" boundary) --- */
            {
                const Vector2 pushN = {  dir.y, -dir.x }; /* push onto track: right of segment */

                const Vector2 bA = { ni->centerM.x + perp.x * hw,
                                     ni->centerM.y + perp.y * hw };
                const Vector2 bB = { nj->centerM.x + perp.x * hw,
                                     nj->centerM.y + perp.y * hw };

                /* Check front circle, then rear, against this barrier segment. */
                /* Front circle */
                if (point_segment_sq(frontWorld, bA, bB) < radiusSq) {
                    const Vector2 contactPt = closest_point_on_segment(frontWorld, bA, bB);
                    const float dist = sqrtf(point_segment_sq(frontWorld, bA, bB));
                    const float pen = radiusM - dist;
                    if (pen > 0.0f && dist > 1e-9f) {
                        /* Move the CG so the circle touches the barrier. */
                        state->positionM.x = pos.x + pushN.x * pen;
                        state->positionM.y = pos.y + pushN.y * pen;
                        state->headingRad = hdg;
                        renderState->currPositionM = state->positionM;
                        renderState->currHeadingRad = state->headingRad;

                        /* Impulse response at the contact point. */
                        const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
                        const Vector2 vContact = contact_velocity_world(vCgWorld,
                                                                         state->yawRateRadS,
                                                                         rContact);

                        /* Normal velocity (positive = separating, negative = approaching). */
                        const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
                        if (vn < 0.0f) {
                            /* Tangential direction: rotate normal +90°. */
                            const Vector2 tang = { -pushN.y, pushN.x };
                            const float vt = vContact.x * tang.x + vContact.y * tang.y;

                            /* Effective mass for normal impulse. */
                            /* r × n (2D cross product) */
                            const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
                            const float rXt = rContact.x * tang.y - rContact.y * tang.x;
                            const float effMassN = 1.0f / (1.0f / spec->massKg +
                                                            rXn * rXn / spec->yawInertiaKgM2);
                            const float effMassT = 1.0f / (1.0f / spec->massKg +
                                                            rXt * rXt / spec->yawInertiaKgM2);

                            /* Normal impulse: reflect with restitution.
                             * Δv_n = -(1 + e) * v_n  (v_n is negative when approaching,
                             * so Δv_n is positive — we push away). */
                            const float deltaVN = -(1.0f + rHalf) * vn;
                            const float Jn = effMassN * deltaVN; /* Jn > 0 */

                            /* Friction impulse, Coulomb-clamped. */
                            float Jt = -effMassT * vt;
                            const float JtMax = muC * Jn;
                            if (Jt >  JtMax) Jt =  JtMax;
                            if (Jt < -JtMax) Jt = -JtMax;

                            /* Total impulse in world frame. */
                            const Vector2 J = {
                                Jn * pushN.x + Jt * tang.x,
                                Jn * pushN.y + Jt * tang.y
                            };

                            /* Apply to CG linear velocity. */
                            const Vector2 vCgNew = {
                                vCgWorld.x + J.x / spec->massKg,
                                vCgWorld.y + J.y / spec->massKg
                            };
                            world_vel_to_body(vCgNew, state->headingRad,
                                              &state->velocityLongitudinalMps,
                                              &state->velocityLateralMps);

                            /* Angular impulse: r × J. */
                            const float rXJ = rContact.x * J.y - rContact.y * J.x;
                            state->yawRateRadS += rXJ / spec->yawInertiaKgM2;

                            /* Crash lockout on significant impact. */
                            if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
                                *crashLockoutTimerS = CRASH_LOCKOUT_S;
                            }
                            return 1;
                        }
                    }
                }

                /* Rear circle */
                if (point_segment_sq(rearWorld, bA, bB) < radiusSq) {
                    const Vector2 contactPt = closest_point_on_segment(rearWorld, bA, bB);
                    const float dist = sqrtf(point_segment_sq(rearWorld, bA, bB));
                    const float pen = radiusM - dist;
                    if (pen > 0.0f && dist > 1e-9f) {
                        state->positionM.x = pos.x + pushN.x * pen;
                        state->positionM.y = pos.y + pushN.y * pen;
                        state->headingRad = hdg;
                        renderState->currPositionM = state->positionM;
                        renderState->currHeadingRad = state->headingRad;

                        const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
                        const Vector2 vContact = contact_velocity_world(vCgWorld,
                                                                         state->yawRateRadS,
                                                                         rContact);
                        const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
                        if (vn < 0.0f) {
                            const Vector2 tang = { -pushN.y, pushN.x };
                            const float vt = vContact.x * tang.x + vContact.y * tang.y;
                            const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
                            const float rXt = rContact.x * tang.y - rContact.y * tang.x;
                            const float effMassN = 1.0f / (1.0f / spec->massKg +
                                                            rXn * rXn / spec->yawInertiaKgM2);
                            const float effMassT = 1.0f / (1.0f / spec->massKg +
                                                            rXt * rXt / spec->yawInertiaKgM2);
                            const float deltaVN = -(1.0f + rHalf) * vn;
                            const float Jn = effMassN * deltaVN;
                            float Jt = -effMassT * vt;
                            const float JtMax = muC * Jn;
                            if (Jt >  JtMax) Jt =  JtMax;
                            if (Jt < -JtMax) Jt = -JtMax;
                            const Vector2 J = {
                                Jn * pushN.x + Jt * tang.x,
                                Jn * pushN.y + Jt * tang.y
                            };
                            const Vector2 vCgNew = {
                                vCgWorld.x + J.x / spec->massKg,
                                vCgWorld.y + J.y / spec->massKg
                            };
                            world_vel_to_body(vCgNew, state->headingRad,
                                              &state->velocityLongitudinalMps,
                                              &state->velocityLateralMps);
                            const float rXJ = rContact.x * J.y - rContact.y * J.x;
                            state->yawRateRadS += rXJ / spec->yawInertiaKgM2;
                            if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
                                *crashLockoutTimerS = CRASH_LOCKOUT_S;
                            }
                            return 1;
                        }
                    }
                }
            }

            /* --- Right barrier (the "inside" boundary) --- */
            {
                const Vector2 pushN = { -dir.y, dir.x }; /* push onto track: left of segment */

                const Vector2 bA = { ni->centerM.x - perp.x * hw,
                                     ni->centerM.y - perp.y * hw };
                const Vector2 bB = { nj->centerM.x - perp.x * hw,
                                     nj->centerM.y - perp.y * hw };

                /* Front circle */
                if (point_segment_sq(frontWorld, bA, bB) < radiusSq) {
                    const Vector2 contactPt = closest_point_on_segment(frontWorld, bA, bB);
                    const float dist = sqrtf(point_segment_sq(frontWorld, bA, bB));
                    const float pen = radiusM - dist;
                    if (pen > 0.0f && dist > 1e-9f) {
                        state->positionM.x = pos.x + pushN.x * pen;
                        state->positionM.y = pos.y + pushN.y * pen;
                        state->headingRad = hdg;
                        renderState->currPositionM = state->positionM;
                        renderState->currHeadingRad = state->headingRad;

                        const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
                        const Vector2 vContact = contact_velocity_world(vCgWorld,
                                                                         state->yawRateRadS,
                                                                         rContact);
                        const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
                        if (vn < 0.0f) {
                            const Vector2 tang = { -pushN.y, pushN.x };
                            const float vt = vContact.x * tang.x + vContact.y * tang.y;
                            const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
                            const float rXt = rContact.x * tang.y - rContact.y * tang.x;
                            const float effMassN = 1.0f / (1.0f / spec->massKg +
                                                            rXn * rXn / spec->yawInertiaKgM2);
                            const float effMassT = 1.0f / (1.0f / spec->massKg +
                                                            rXt * rXt / spec->yawInertiaKgM2);
                            const float deltaVN = -(1.0f + rHalf) * vn;
                            const float Jn = effMassN * deltaVN;
                            float Jt = -effMassT * vt;
                            const float JtMax = muC * Jn;
                            if (Jt >  JtMax) Jt =  JtMax;
                            if (Jt < -JtMax) Jt = -JtMax;
                            const Vector2 J = {
                                Jn * pushN.x + Jt * tang.x,
                                Jn * pushN.y + Jt * tang.y
                            };
                            const Vector2 vCgNew = {
                                vCgWorld.x + J.x / spec->massKg,
                                vCgWorld.y + J.y / spec->massKg
                            };
                            world_vel_to_body(vCgNew, state->headingRad,
                                              &state->velocityLongitudinalMps,
                                              &state->velocityLateralMps);
                            const float rXJ = rContact.x * J.y - rContact.y * J.x;
                            state->yawRateRadS += rXJ / spec->yawInertiaKgM2;
                            if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
                                *crashLockoutTimerS = CRASH_LOCKOUT_S;
                            }
                            return 1;
                        }
                    }
                }

                /* Rear circle */
                if (point_segment_sq(rearWorld, bA, bB) < radiusSq) {
                    const Vector2 contactPt = closest_point_on_segment(rearWorld, bA, bB);
                    const float dist = sqrtf(point_segment_sq(rearWorld, bA, bB));
                    const float pen = radiusM - dist;
                    if (pen > 0.0f && dist > 1e-9f) {
                        state->positionM.x = pos.x + pushN.x * pen;
                        state->positionM.y = pos.y + pushN.y * pen;
                        state->headingRad = hdg;
                        renderState->currPositionM = state->positionM;
                        renderState->currHeadingRad = state->headingRad;

                        const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
                        const Vector2 vContact = contact_velocity_world(vCgWorld,
                                                                         state->yawRateRadS,
                                                                         rContact);
                        const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
                        if (vn < 0.0f) {
                            const Vector2 tang = { -pushN.y, pushN.x };
                            const float vt = vContact.x * tang.x + vContact.y * tang.y;
                            const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
                            const float rXt = rContact.x * tang.y - rContact.y * tang.x;
                            const float effMassN = 1.0f / (1.0f / spec->massKg +
                                                            rXn * rXn / spec->yawInertiaKgM2);
                            const float effMassT = 1.0f / (1.0f / spec->massKg +
                                                            rXt * rXt / spec->yawInertiaKgM2);
                            const float deltaVN = -(1.0f + rHalf) * vn;
                            const float Jn = effMassN * deltaVN;
                            float Jt = -effMassT * vt;
                            const float JtMax = muC * Jn;
                            if (Jt >  JtMax) Jt =  JtMax;
                            if (Jt < -JtMax) Jt = -JtMax;
                            const Vector2 J = {
                                Jn * pushN.x + Jt * tang.x,
                                Jn * pushN.y + Jt * tang.y
                            };
                            const Vector2 vCgNew = {
                                vCgWorld.x + J.x / spec->massKg,
                                vCgWorld.y + J.y / spec->massKg
                            };
                            world_vel_to_body(vCgNew, state->headingRad,
                                              &state->velocityLongitudinalMps,
                                              &state->velocityLateralMps);
                            const float rXJ = rContact.x * J.y - rContact.y * J.x;
                            state->yawRateRadS += rXJ / spec->yawInertiaKgM2;
                            if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
                                *crashLockoutTimerS = CRASH_LOCKOUT_S;
                            }
                            return 1;
                        }
                    }
                }
            }
        }
    }

    return 0;
}
