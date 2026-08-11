/*
 * collision.c — vehicle-track capsule collision with swept substeps and impulse response.
 *
 * Physics conventions:
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

/* Closest point on segment a→b to point p, and the squared distance to it.
 * Robust for zero-length segments. */
static Vector2 closest_point_and_dist_sq(Vector2 p, Vector2 a, Vector2 b, float *distSqOut)
{
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float lenSq = dx * dx + dy * dy;
    float t;
    if (lenSq < 1e-12f) {
        t = 0.0f;
    } else {
        t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    const Vector2 closest = { a.x + t * dx, a.y + t * dy };
    const float ex = p.x - closest.x;
    const float ey = p.y - closest.y;
    *distSqOut = ex * ex + ey * ey;
    return closest;
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
    return (Vector2){ cgWorld.x + bodyPoint.x * cosH - bodyPoint.y * sinH,
                      cgWorld.y + bodyPoint.x * sinH + bodyPoint.y * cosH };
}

/* ====================================================================================== */
/*  Velocity helpers                                                                       */
/* ====================================================================================== */

/* World-frame CG velocity from body-frame components. */
static Vector2 world_velocity(float vxBody, float vyBody, float headingRad)
{
    const float cosH = cosf(headingRad);
    const float sinH = sinf(headingRad);
    return (Vector2){ vxBody * cosH - vyBody * sinH, vxBody * sinH + vyBody * cosH };
}

/* World-frame contact-point velocity: v_CG + ω × r.  In 2D, ω × r = ω * (-r.y, r.x). */
static Vector2 contact_velocity_world(Vector2 vCgWorld, float yawRateRadS, Vector2 r)
{
    return (Vector2){ vCgWorld.x - yawRateRadS * r.y, vCgWorld.y + yawRateRadS * r.x };
}

/* World → body rotation of a velocity vector. */
/* Body X = dot(v_world, (cosH, sinH))   (forward)
 * Body Y = dot(v_world, (-sinH, cosH))  (left)
 */
static void world_vel_to_body(Vector2 vWorld, float headingRad, float *vxBodyOut,
                              float *vyBodyOut)
{
    *vxBodyOut = vWorld.x * cosf(headingRad) + vWorld.y * sinf(headingRad);
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
    if (delta > 3.14159265358979323846f) delta -= 2.0f * 3.14159265358979323846f;
    if (delta < -3.14159265358979323846f) delta += 2.0f * 3.14159265358979323846f;
    return a + delta * t;
}

static Vector2 lerp_vec(Vector2 a, Vector2 b, float t)
{
    return (Vector2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}

/* AABB of the two capsule circles at one pose, used to query candidates. A circle's AABB
 * contains any barrier segment that could penetrate it, and the swept pass re-queries after
 * every contact, so a superset of exactly the penetrating shapes is always in hand. */
static void capsule_aabb(const CollisionBody *body, Vector2 pos, float hdg, Vector2 *minMOut,
                         Vector2 *maxMOut)
{
    const float r = body->radiusM;
    /* Same body-frame circle positions resolve_body_vs_static uses, so the query box always
     * matches the narrowphase's capsule geometry. */
    const Vector2 bodyPts[2] = { body_front_position(body->cgToFrontM),
                                 body_rear_position(body->cgToRearM) };
    Vector2 minM = { INFINITY, INFINITY };
    Vector2 maxM = { -INFINITY, -INFINITY };
    for (int c = 0; c < 2; c++) {
        const Vector2 w = world_from_body(bodyPts[c], pos, hdg);
        minM.x = fminf(minM.x, w.x - r);
        minM.y = fminf(minM.y, w.y - r);
        maxM.x = fmaxf(maxM.x, w.x + r);
        maxM.y = fmaxf(maxM.y, w.y + r);
    }
    *minMOut = minM;
    *maxMOut = maxM;
}

/* ====================================================================================== */
/*  Contact recording                                                                      */
/* ====================================================================================== */

/* Append one physical contact to the world's per-tick event feed. The feed is a bounded
 * presentation buffer: when it fills, recording stops and the overflow flag is set, but the
 * resolution that produced the contact has already happened and never depends on the feed. */
static void record_contact(CollisionWorld *world, CollisionBodyId bodyId,
                           CollisionShapeId shapeId, Vector2 pointM, Vector2 normalM,
                           float approachSpeedMps)
{
    if (world == NULL) return;
    if (world->contactCount < COLLISION_WORLD_MAX_CONTACTS) {
        world->contacts[world->contactCount] =
            (CollisionContact){ .bodyId = bodyId,
                                .shapeId = shapeId,
                                .pointM = pointM,
                                .normalM = normalM,
                                .approachSpeedMps = approachSpeedMps };
        world->contactCount++;
    } else {
        world->contactsOverflowed = true;
    }
}

/* ====================================================================================== */
/*  Contact resolution helper                                                              */
/* ====================================================================================== */

/* Resolves one circle-vs-barrier contact: penetration correction + impulse response.
 *
 * Penetration push moves the CG so the circle just touches the barrier. If the contact is
 * approaching (vn < 0), a normal impulse reflects velocity with restitution and a tangential
 * Coulomb-clamped friction impulse is applied; both update CG linear velocity and yaw rate.
 * *vCgWorld is updated to the post-impulse world-frame CG velocity so a subsequent contact
 * in the same substep sees the corrected velocity (no stale-data trap).
 *
 * Returns true if a contact was resolved (penetration corrected; impulse applied when
 * approaching). Returns false if there is no penetration or the contact is degenerate. */
static bool resolve_circle_barrier(const VehicleSpec *spec, VehicleState *state,
                                   VehicleRenderState *renderState, Vector2 pos, float hdg,
                                   Vector2 contactPt, float distSq, Vector2 pushN,
                                   float radiusM, float rHalf, float muC, Vector2 *vCgWorld,
                                   float *crashLockoutTimerS, CollisionWorld *world,
                                   CollisionBodyId bodyId, CollisionShapeId shapeId)
{
    const float dist = sqrtf(distSq);
    const float pen = radiusM - dist;
    if (!(pen > 0.0f) || !(dist > 1e-9f)) return false;

    /* Penetration correction: push the CG along the push normal. */
    state->positionM.x = pos.x + pushN.x * pen;
    state->positionM.y = pos.y + pushN.y * pen;
    state->headingRad = hdg;
    renderState->currPositionM = state->positionM;
    renderState->currHeadingRad = state->headingRad;

    /* Contact-point velocity: v_CG + ω × r. */
    const Vector2 rContact = { contactPt.x - pos.x, contactPt.y - pos.y };
    const Vector2 vContact = contact_velocity_world(*vCgWorld, state->yawRateRadS, rContact);

    /* Normal velocity (positive = separating, negative = approaching). */
    const float vn = vContact.x * pushN.x + vContact.y * pushN.y;
    /* The physical event: positive approach speed means the body was coming in. */
    record_contact(world, bodyId, shapeId, contactPt, pushN, -vn);
    if (vn >= 0.0f) return true; /* separating: push only, no impulse needed */

    /* Tangential direction: rotate normal +90°. */
    const Vector2 tang = { -pushN.y, pushN.x };
    const float vt = vContact.x * tang.x + vContact.y * tang.y;

    /* Effective mass for normal and tangential impulse. */
    const float rXn = rContact.x * pushN.y - rContact.y * pushN.x;
    const float rXt = rContact.x * tang.y - rContact.y * tang.x;
    const float invMass = 1.0f / spec->massKg;
    const float invInertia = 1.0f / spec->yawInertiaKgM2;
    const float effMassN = 1.0f / (invMass + rXn * rXn * invInertia);
    const float effMassT = 1.0f / (invMass + rXt * rXt * invInertia);

    /* Normal impulse: reflect with restitution (Δv_n = -(1+e)·v_n). */
    const float Jn = effMassN * (-(1.0f + rHalf) * vn);

    /* Friction impulse, Coulomb-clamped to |Jt| ≤ μ·Jn. */
    float Jt = -effMassT * vt;
    const float JtMax = muC * Jn;
    if (Jt > JtMax) Jt = JtMax;
    if (Jt < -JtMax) Jt = -JtMax;

    /* Total impulse in world frame, applied to CG linear velocity. */
    const Vector2 J = { Jn * pushN.x + Jt * tang.x, Jn * pushN.y + Jt * tang.y };
    const Vector2 vCgNew = { vCgWorld->x + J.x * invMass, vCgWorld->y + J.y * invMass };
    world_vel_to_body(vCgNew, state->headingRad, &state->velocityLongitudinalMps,
                      &state->velocityLateralMps);

    /* Angular impulse: r × J. */
    const float rXJ = rContact.x * J.y - rContact.y * J.x;
    state->yawRateRadS += rXJ * invInertia;

    /* Update the CG velocity for any subsequent contact in this substep. */
    *vCgWorld = vCgNew;

    /* Crash lockout on significant impact. */
    if (crashLockoutTimerS != NULL && fabsf(vn) > COLLISION_LOCKOUT_THRESHOLD_MPS) {
        *crashLockoutTimerS = CRASH_LOCKOUT_S;
    }
    return true;
}

/* ====================================================================================== */
/*  Swept narrowphase for one body                                                         */
/* ====================================================================================== */

/* Resolve one body's swept capsule against the world's static shapes.
 *
 * Per substep, the candidates are the shapes overlapping the capsule AABB at the substep
 * pose, in ascending shape id order — the exact order the legacy loop walked every barrier
 * in. After a contact pushes the body, the candidate list is re-queried at the new pose
 * STRICTLY AHEAD of the pass cursor (afterId), so shapes already passed are never revisited
 * and shapes that only now penetrate are found before their turn. That keeps the resolution
 * sequence bit-identical to a single ascending brute-force pass while skipping shapes that
 * cannot touch the body.
 *
 * As before, at the FIRST substep that finds penetration all currently-penetrating
 * circle/barrier pairs are resolved in deterministic order, then the substep loop returns
 * the contact count: once state is mutated the prev→curr interpolation is stale. */
static int resolve_body_vs_static(CollisionWorld *world, const CollisionBody *body,
                                  const VehicleSpec *spec, VehicleState *state,
                                  VehicleRenderState *renderState, float *crashLockoutTimerS)
{
    const float radiusM = body->radiusM;
    const float radiusSq = radiusM * radiusM;
    const float rHalf = spec->collisionRestitution;
    const float muC = spec->collisionFriction;

    /* Capsule circle body-frame positions. */
    const Vector2 bFront = body_front_position(body->cgToFrontM);
    const Vector2 bRear = body_rear_position(body->cgToRearM);

    /* World-frame CG velocity at start of tick. Mutable: each resolved contact updates it
     * so a subsequent contact in the same substep uses the corrected velocity. */
    Vector2 vCgWorld = world_velocity(state->velocityLongitudinalMps, state->velocityLateralMps,
                                      body->currHdgRad);

    for (int sub = 0; sub < COLLISION_SUBSTEPS; sub++) {
        const float t = (float)sub / (float)COLLISION_SUBSTEPS;
        Vector2 pos = lerp_vec(body->prevPosM, body->currPosM, t);
        const float hdg = lerp_angle_simple(body->prevHdgRad, body->currHdgRad, t);
        int contacts = 0;

        Vector2 minM, maxM;
        capsule_aabb(body, pos, hdg, &minM, &maxM);
        int n = collision_world_query_static(world, minM, maxM, body->mask,
                                             COLLISION_SHAPE_ID_NONE, world->queryScratch,
                                             COLLISION_WORLD_MAX_STATIC_SHAPES);
        int i = 0;
        while (i < n) {
            const CollisionShapeId id = world->queryScratch[i++];
            const CollisionStaticShape *shape = &world->shapes[id];
            bool resolvedAny = false;

            for (int circle = 0; circle < 2; circle++) {
                const Vector2 bodyPt = (circle == 0) ? bFront : bRear;
                const Vector2 circleWorld = world_from_body(bodyPt, pos, hdg);

                float distSq;
                const Vector2 contactPt =
                    closest_point_and_dist_sq(circleWorld, shape->aM, shape->bM, &distSq);

                if (distSq < radiusSq) {
                    if (resolve_circle_barrier(spec, state, renderState, pos, hdg, contactPt,
                                               distSq, shape->pushNormalM, radiusM, rHalf, muC,
                                               &vCgWorld, crashLockoutTimerS, world, body->id,
                                               id)) {
                        contacts++;
                        resolvedAny = true;
                        pos = state->positionM;
                    }
                }
            }

            if (resolvedAny) {
                /* The body moved: re-query strictly ahead of this shape's id at the new
                 * pose and restart the pass there (single ascending visit, no revisits). */
                capsule_aabb(body, pos, hdg, &minM, &maxM);
                n = collision_world_query_static(world, minM, maxM, body->mask, id,
                                                 world->queryScratch,
                                                 COLLISION_WORLD_MAX_STATIC_SHAPES);
                i = 0;
            }
        }

        if (contacts > 0) return contacts;
    }

    return 0;
}

/* ====================================================================================== */
/*  Ordered multi-body resolution                                                          */
/* ====================================================================================== */

int collision_world_resolve_bodies(CollisionWorld *world, CollisionBodyContext *contexts,
                                   int contextCount)
{
    if (world == NULL || world->bodyCount == 0) return 0;
    if (contexts == NULL && contextCount > 0) return -1;

    int total = 0;
    for (int b = 0; b < world->bodyCount; b++) {
        const CollisionBody *body = &world->bodies[b];
        CollisionBodyContext *ctx = NULL;
        for (int c = 0; c < contextCount; c++) {
            if (contexts[c].id == body->id) {
                ctx = &contexts[c];
                break;
            }
        }
        /* Every registered body must have a response context; a missing one is a caller
         * contract violation and is reported, not silently skipped. */
        if (ctx == NULL) return -1;
        if (ctx->spec == NULL || ctx->state == NULL || ctx->renderState == NULL) return -1;

        const int n = resolve_body_vs_static(world, body, ctx->spec, ctx->state,
                                             ctx->renderState, ctx->crashLockoutTimerS);
        if (n < 0) return -1;
        total += n;
    }
    return total;
}

/* ====================================================================================== */
/*  One-entrant path                                                                       */
/* ====================================================================================== */

int collision_resolve_track(CollisionWorld *world, CollisionBodyId bodyId,
                            const VehicleSpec *spec, VehicleState *state,
                            VehicleRenderState *renderState, float *crashLockoutTimerS)
{
    if (world == NULL || spec == NULL || state == NULL || renderState == NULL) return 0;
    if (!(spec->bodyHalfWidthM > 0.0f)) return 0;

    collision_world_begin_tick(world);

    CollisionBody body;
    memset(&body, 0, sizeof(body));
    body.id = bodyId;
    body.layer = COLLISION_LAYER_VEHICLE_BODY;
    body.mask = COLLISION_LAYER_STATIC_BARRIER;
    body.cgToFrontM = spec->cgToFrontM;
    body.cgToRearM = spec->cgToRearM;
    body.radiusM = spec->bodyHalfWidthM;
    body.prevPosM = renderState->prevPositionM;
    body.currPosM = renderState->currPositionM;
    body.prevHdgRad = renderState->prevHeadingRad;
    body.currHdgRad = renderState->currHeadingRad;
    if (!collision_world_add_body(world, &body)) return 0;

    CollisionBodyContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.id = bodyId;
    ctx.spec = spec;
    ctx.state = state;
    ctx.renderState = renderState;
    ctx.crashLockoutTimerS = crashLockoutTimerS;
    return collision_world_resolve_bodies(world, &ctx, 1);
}
