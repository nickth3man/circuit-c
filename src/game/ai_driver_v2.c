/*
 * ai_driver_v2.c — the #79 limit-aware driver (AI_DRIVER_ARCH_LIMIT).
 *
 * A clean-slate controller behind the AiDriverConfig.architecture switch, built from the #79
 * design review. It drives the SAME minimum-curvature planned line as the baseline driver
 * (the planner is shared through ai_driver_internal.h); what changes is everything after the
 * plan:
 *
 *   STEERING — pure pursuit plus a LOW-PASSED cross-track derivative (the baseline D-term
 *   differentiates the raw 120 Hz error and chatters at tens of Hz; the filter passes the
 *   ~1 Hz human steering band and rejects the chatter), curvature feedforward (the wheel is
 *   already turned toward the plan's curvature ahead of the corner, which is what makes the
 *   inputs look anticipatory), and a yaw-rate error term that only acts while the slide
 *   monitor is active (always-on yaw damping was tried standalone and destabilised the
 *   marginally-stable awd_rally).
 *
 *   SPEED — a friction-ellipse forward+backward profile over the planned line (Kapania-style
 *   three passes): a forward engine-limited pass raises the reachable speed on straights
 *   (the baseline has only the backward braking pass, which is why it coasts), the backward
 *   pass propagates braking and corner limits, and every longitudinal limit is capped by the
 *   combined-grip ellipse sqrt((mu g)^2 - a_lat^2) using the car's own mu, mass, engine
 *   torque curve, gearbox and CG split.
 *
 *   TRACTION — the baseline's reactive gripCut stays as the slow backstop (its 1.4 /s rate is
 *   the value proven stable on awd_rally), but it should rarely engage: the ellipse profile
 *   keeps the car OFF the friction limit by construction. What is NEW is the slide monitor:
 *   when a slip/yaw excursion is detected it HOLDS the throttle steady (never chops — the
 *   direct ellipse throttle limit chopped the apex throttle, unloaded the rear, and spun
 *   awd_rally) and applies BOUNDED counter-steer from yaw-rate error and sideslip. Gating the
 *   stability feedback to actual slides is the difference between this and the failed
 *   always-on attempts.
 *
 * This is Phase 1 of #79: the coupled core built against awd_rally, behind the switch. It is
 * NOT yet the default. The Phase-1 acceptance gate is that awd_rally completes the chicane
 * 4-lap run with no worse off-track/collisions than the baseline driver; the full roster and
 * the other tracks follow in the issue's later sessions.
 *
 * PHASE-2 STATUS (2026-08-11, --ai-mode v2): the whole roster completes chicane and sprint,
 * and 17 of the 18 suite cases pass against the baseline driver's 15. Per track, 6/6 chicane
 * (0.0% off-track and 0 collisions on every car, where the baseline shows 14.5% and 1
 * collision on awd_rally), 6/6 sprint (the baseline fails rwd_power and awd_rally with
 * planner_localization_mismatch), 5/6 technical. The one failure — awd_rally on technical —
 * fails IDENTICALLY on the baseline driver: both stall at checkpoint 23 with no collision and
 * no spin, so it is the pre-existing stuck case (#77/#28), not an architecture regression.
 * v2 is still slower than the baseline everywhere (awd_rally chicane ~60 s/lap vs 49 s).
 *
 * Phase 1 had blamed the awd_gt/rwd_grip/rwd_power chicane failures on understeer at ~24 m/s
 * and the sampled plan's corner limit. The telemetry says otherwise, and the recorded next
 * step (a tracking-error speed penalty) would not have touched them: at the moment the run
 * came apart the cross-track error was 0.03 m. What actually happened is that the slide
 * monitor false-positived on a corner APPROACH — yawErr = yawRate - speed*kappaHere compares
 * the car's current rotation against the plan's curvature a node AHEAD, so a straight, stable
 * car at 37.8 m/s nearing a kappa=0.0211 corner reads -0.766 rad/s and trips the 0.50
 * threshold on geometry alone — and the hold branch then froze the WHOLE pedal axis, braking
 * included, for hundreds of ticks while the profile asked for 12.5 m/s at an actual 37 (awd_gt
 * ticks 8312-8554; rwd_grip froze at pedal 0.03 for 226 ticks, rwd_power at 0.54 for 994).
 * Letting a braking demand through the hold is the whole of the Phase-2 change.
 *
 * PHASE-3 STATUS (2026-08-11, #81 item 1): the slide monitor no longer false-positives on a
 * corner approach. The reference yaw rate is taken AT the car (the plan's curvature at the
 * car's own segment) instead of a node ahead, and the yaw branch requires sideslip
 * corroboration (0.05 rad) because counter-steer plus throttle hold is an oversteer remedy and
 * oversteer is what sideslip identifies. A monitor that was latched for much of a lap was the
 * first suspect for v2's lap-time deficit; lap times before/after are measured per #81.
 *
 * Next steps, in order: (1) the full-path forward pass, which the per-tick reachable-preview
 * formulation could not use (it pinned the target to the current speed and the car crawled —
 * observed and reverted). (2) the tracking-error speed penalty is NOT currently shipped and no
 * longer has a failing case driving it — chicane is at 0.0% off-track — so it should wait for
 * one. Known-good gains from Phase 1: D-filter tau 0.06 s (was 0.12), feedforward gain 0.45
 * (was 0.85); signed (not absolute) curvature in the feedforward was the difference between
 * barrier-pinning and lapping.
 *
 * Everything read is geometry the track publishes, grip the car publishes about itself, or
 * feelable state (sideslip, yaw rate, friction usage) — no force or torque internals. Nothing
 * is written except the ControllerOutput and the driver's own state.
 */
#include "game/ai_driver_v2.h"

#include <math.h>
#include <stddef.h>

#include "core/config.h"
#include "core/math_utils.h"
#include "game/ai_driver_internal.h"
#include "physics/surface.h"

/* ---------------------------------------------------------------------------
 * #79 Phase 1 tuning block. Shared by every car — no per-car tuning, exactly as
 * the ai-roster-laps shared-config assertion demands. Iterated against awd_rally
 * as the knife-edge case.
 * ---------------------------------------------------------------------------
 */
#define AI_V2_MIN_CURVATURE_1PM 1.0e-4f /* curvature below this is a straight */
#define AI_V2_D_FILTER_TAU_S 0.06f      /* low-pass tau for the cross-track derivative */
#define AI_V2_FF_GAIN 0.45f             /* curvature feedforward gain (0 = off) */
#define AI_V2_FF_NODE_LEAD 3            /* plan nodes ahead the feedforward reads */
#define AI_V2_SLIDE_SIDESLIP_RAD 0.12f  /* |sideslip| above this is an excursion */
#define AI_V2_SLIDE_YAW_ERR_RAD_S 0.50f /* yaw-rate error above this is an excursion */
#define AI_V2_SLIDE_YAW_CORROBORATION_RAD \
    0.05f /* the yaw branch also needs |sideslip| above this */
/* The corroboration level sits between stable cornering (p95 |sideslip| <= 0.06 on every roster
 * car, <= 0.04 on all but the knife-edge awd_rally) and a genuine excursion (0.12+), so a
 * corner-entry transient — car not yet rotating, yaw error large, sideslip still ~0.02 — does
 * not corroborate. Counter-steer plus throttle hold is an OVERSTEER remedy, and oversteer is
 * what sideslip identifies. (#81 item 1.) */
#define AI_V2_SLIDE_MIN_SPEED_MPS 3.0f   /* below this the monitor stays off */
#define AI_V2_SLIDE_CONFIRM_TICKS 2      /* threshold ticks before declaring a slide */
#define AI_V2_SLIDE_RELEASE_TICKS 30     /* clean ticks before the slide is over (0.25 s) */
#define AI_V2_CS_YAW_GAIN 0.30f          /* counter-steer per rad/s of yaw-rate error */
#define AI_V2_CS_BETA_GAIN 0.60f         /* counter-steer per rad of sideslip */
#define AI_V2_CS_MAX 0.30f               /* bounded counter-steer, fraction of lock */
#define AI_V2_PROFILE_SCAN_MARGIN_M 8.0f /* same braking-point margin as the baseline */
#define AI_V2_ELLIPSE_FLOOR_MPS2 0.1f    /* floor under the ellipse longitudinal limit */

/* The combined-grip (friction-ellipse) longitudinal limit at a point where the lateral
 * demand is aLat: sqrt(max((mu g)^2 - aLat^2, floor)). */
static float ai_v2_ellipse_long_limit_mps2(float muLat, float aLatMps2)
{
    const float total = muLat * GRAVITY_MPS2;
    const float latClamped = fminf(aLatMps2, total);
    const float v = sqrtf(fmaxf(total * total - latClamped * latClamped, 0.0f));
    return fmaxf(v, AI_V2_ELLIPSE_FLOOR_MPS2);
}

/* Signed curvature of the planned line `lead` nodes ahead of the car's segment. Positive is a
 * LEFT bend relative to travel (the steer demand sign), which the absolute Menger value cannot
 * express: the chicane bends right while climbing, and an always-left feedforward drove the
 * car off the route and into the outside barrier (observed, #79 Phase 1). */
static float ai_v2_signed_curvature(Vector2 a, Vector2 b, Vector2 c)
{
    const Vector2 ab = vec_sub(b, a);
    const Vector2 bc = vec_sub(c, b);
    const Vector2 ca = vec_sub(a, c);
    const float denom = vec_len(ab) * vec_len(bc) * vec_len(ca);
    if (denom < 1.0e-6f) return 0.0f;
    return 2.0f * (ab.x * bc.y - ab.y * bc.x) / denom;
}

static float ai_v2_plan_curvature_ahead(const AiDriverState *state,
                                        const TrackDefinition *track, int segment, int lead)
{
    const int n = segment + lead;
    return ai_v2_signed_curvature(ai_driver_plan_point(state, track, n - 1),
                                  ai_driver_plan_point(state, track, n),
                                  ai_driver_plan_point(state, track, n + 1));
}

/*
 * Friction-ellipse speed profile over the planned line. Returns the fastest speed the car may
 * be going NOW and still stop for everything ahead: the backward (braking + corner) pass with
 * the braking deceleration itself ellipse-capped by the lateral demand at the next node.
 *
 * The engine-limited FORWARD pass of the three-pass algorithm is deliberately NOT applied
 * per-tick in Phase 1: with the reachable envelope evaluated from the current speed at a short
 * preview, the target pinned itself to the current speed plus a few m/s, and the car crawled
 * (observed: awd_rally never regained speed after the chicane). The forward pass belongs to
 * the full-path profile (recomputed on replan, entry-speed anchored) in the issue's later
 * sessions; the baseline's full-throttle-when-under-target rule already covers straights, and
 * the ellipse backstop handles the traction side.
 */
static float ai_v2_speed_target_mps(const AiDriverConfig *cfg, const AiDriverState *state,
                                    const TrackDefinition *track, const VehicleSpec *spec,
                                    int bestSegment, float speedMps, float muLat, float muLong)
{
    (void)spec;
    const float brakeAccel = fmaxf(cfg->brakeGripFraction * muLong * GRAVITY_MPS2, 0.1f);
    const float horizonM =
        speedMps * speedMps / (2.0f * brakeAccel) + AI_V2_PROFILE_SCAN_MARGIN_M;

    /* Sample the planned line out to the horizon: per-node segment length, curvature, and
     * cumulative distance from the car. */
    float ds[AI_PLAN_LAYERS];
    float kappa[AI_PLAN_LAYERS];
    int nodes = 0;
    float walkedM = 0.0f;
    Vector2 walk = ai_driver_plan_point(state, track, bestSegment);
    for (int step = 0; step < AI_PLAN_LAYERS && walkedM <= horizonM; step++) {
        const int node = bestSegment + 1 + step;
        const Vector2 here = ai_driver_plan_point(state, track, node);
        const float legM = vec_len(vec_sub(here, walk));
        walk = here;
        ds[nodes] = legM;
        kappa[nodes] = menger_curvature(ai_driver_plan_point(state, track, node - 1), here,
                                        ai_driver_plan_point(state, track, node + 1));
        walkedM += legM;
        nodes++;
    }
    if (nodes <= 0) return cfg->maxSpeedMps;

    /* Corner limit at each node: lateral grip only. */
    float corner[AI_PLAN_LAYERS];
    for (int i = 0; i < nodes; i++) {
        if (kappa[i] < AI_V2_MIN_CURVATURE_1PM) {
            corner[i] = cfg->maxSpeedMps;
        } else {
            corner[i] = sqrtf(
                fmaxf(cfg->corneringGripFraction * muLat * GRAVITY_MPS2 / kappa[i], 0.0f));
        }
    }

    /* Backward pass: the speed at i must be stoppable before the tighter of the corner limit
     * at i+1 and the backward limit at i+1, with the braking deceleration ellipse-capped by
     * the lateral demand implied at i+1. */
    float back[AI_PLAN_LAYERS];
    back[nodes - 1] = corner[nodes - 1];
    for (int i = nodes - 2; i >= 0; i--) {
        const float limitNext = fminf(back[i + 1], corner[i + 1]);
        const float aLat = kappa[i + 1] * limitNext * limitNext;
        const float aBrake = fminf(brakeAccel, ai_v2_ellipse_long_limit_mps2(muLat, aLat));
        const float allowed =
            sqrtf(fmaxf(limitNext * limitNext + 2.0f * aBrake * ds[i + 1], 0.0f));
        back[i] = fminf(corner[i], allowed);
    }

    return clampf(back[0], 0.0f, cfg->maxSpeedMps);
}

void ai_driver_update_v2(const AiDriverConfig *cfg, AiDriverState *state,
                         const TrackDefinition *track, const TrackRuntime *runtime,
                         const VehicleState *vehicle, const VehicleDerived *derived,
                         const VehicleSpec *spec, ControllerOutput *out, float dt,
                         const AiTraffic *traffic)
{
    if (cfg == NULL || state == NULL || out == NULL) return;
    if (track == NULL || track->nodes == NULL || track->count < 3) return;
    if (vehicle == NULL || derived == NULL || spec == NULL) return;

    const TrackNode *nodes = track->nodes;
    const int count = track->count;
    const Vector2 posM = vehicle->positionM;
    const float speedMps = derived->speedMps;

    /* --- Replan: the same minimum-curvature lattice search as the baseline (#79 keeps the
     * line). Anchored to the CENTRELINE segment, exactly like the baseline. */
    const bool needsPlan =
        (state->planLayerCount <= 0) || (state->ticksSinceReplan >= cfg->planReplanTicks);
    if (needsPlan) {
        int centreSegment = 0;
        float centreDistM = INFINITY;
        Vector2 centrePoint = nodes[0].centerM;
        for (int i = 0; i < count; i++) {
            float t = 0.0f;
            Vector2 p = { 0.0f, 0.0f };
            const float d = closest_on_segment(nodes[i].centerM, nodes[(i + 1) % count].centerM,
                                               posM, &t, &p);
            if (d < centreDistM) {
                centreDistM = d;
                centreSegment = i;
                centrePoint = p;
            }
        }
        const Vector2 normal = node_normal(track, centreSegment);
        const Vector2 toCar = vec_sub(posM, centrePoint);
        const float carOffsetM = toCar.x * normal.x + toCar.y * normal.y;

        plan_path(cfg, state, track, centreSegment, carOffsetM);
        state->ticksSinceReplan = 0;
    } else {
        state->ticksSinceReplan++;
    }

    /* --- Where the car is on the planned line (same walk as the baseline). --- */
    int bestSegment = 0;
    float bestT = 0.0f;
    Vector2 bestPoint = ai_driver_plan_point(state, track, 0);
    float bestDistM = INFINITY;
    for (int i = 0; i < count; i++) {
        float t = 0.0f;
        Vector2 p = { 0.0f, 0.0f };
        const float d =
            closest_on_segment(ai_driver_plan_point(state, track, i),
                               ai_driver_plan_point(state, track, i + 1), posM, &t, &p);
        if (d < bestDistM) {
            bestDistM = d;
            bestSegment = i;
            bestT = t;
            bestPoint = p;
        }
    }
    const Vector2 segDir = vec_sub(ai_driver_plan_point(state, track, bestSegment + 1),
                                   ai_driver_plan_point(state, track, bestSegment));
    const float segLen = vec_len(segDir);
    float crossTrackM = 0.0f;
    if (segLen > 1.0e-6f) {
        const Vector2 leftUnit = { -segDir.y / segLen, segDir.x / segLen };
        const Vector2 toCar = vec_sub(posM, bestPoint);
        crossTrackM = toCar.x * leftUnit.x + toCar.y * leftUnit.y;
    }

    /* --- Steering: pure pursuit + filtered D + curvature feedforward. --- */
    const float lookaheadM =
        maxf(cfg->lookaheadBaseM + cfg->lookaheadSpeedS * maxf(speedMps, 0.0f), 1.0f);
    const Vector2 targetM = plan_point_ahead(state, track, bestSegment, bestT, lookaheadM);

    const Vector2 toTarget = vec_sub(targetM, posM);
    const float cosH = cosf(vehicle->headingRad), sinH = sinf(vehicle->headingRad);
    const float bodyX = cosH * toTarget.x + sinH * toTarget.y;  /* forward */
    const float bodyY = -sinH * toTarget.x + cosH * toTarget.y; /* left */
    const float ldM = maxf(sqrtf(bodyX * bodyX + bodyY * bodyY), 0.5f);
    const float alphaRad = atan2f(bodyY, bodyX);

    const float arcCurvature = 2.0f * sinf(alphaRad) / ldM;
    float steerAngleRad = atanf(arcCurvature * spec->wheelbaseM);

    /* Filtered derivative: an EMA over the raw cross-track error rate passes the human
     * steering band and rejects the tens-of-Hz chatter of the raw signal (#79). */
    if (state->hasPrevError && dt > 0.0f) {
        const float rawRateMps = (crossTrackM - state->prevCrossTrackErrorM) / dt;
        const float alpha = clampf(dt / AI_V2_D_FILTER_TAU_S, 0.0f, 1.0f);
        state->filteredErrorRateMps += alpha * (rawRateMps - state->filteredErrorRateMps);
        steerAngleRad -= cfg->steerGainD * state->filteredErrorRateMps;
    }

    /* Curvature feedforward: turn toward the plan's curvature ahead of the car, so the wheel
     * is already wound on as the corner arrives (anticipatory, smooth). */
    steerAngleRad += AI_V2_FF_GAIN * atanf(ai_v2_plan_curvature_ahead(state, track, bestSegment,
                                                                      AI_V2_FF_NODE_LEAD) *
                                           spec->wheelbaseM);

    /* --- Slide monitor: gated slip/yaw excursion, the coupled stability layer. --- */
    /* Reference yaw rate AT the car, not a node ahead: the plan's curvature at the car's own
     * segment is what its current rotation is being checked against. Reading a node ahead
     * compared a straight, stable car against the corner it was APPROACHING, so the error was
     * large by geometry alone (a 37.8 m/s car nearing a kappa=0.0211 corner read -0.766 rad/s
     * against the 0.50 threshold) and the monitor latched for much of a lap. (#81 item 1.) */
    const float kappaHere = ai_v2_plan_curvature_ahead(state, track, bestSegment, 0);
    const float yawErrRadS = vehicle->yawRateRadS - speedMps * kappaHere;
    /* The yaw branch needs sideslip corroboration: counter-steer plus throttle hold is an
     * OVERSTEER remedy, and oversteer is what sideslip identifies. A pure yaw-error reading on
     * its own cannot tell "not yet rotating into the corner" from "rotating too much". */
    const bool yawExcursion =
        (fabsf(yawErrRadS) > AI_V2_SLIDE_YAW_ERR_RAD_S) &&
        (fabsf(derived->bodySideslipRad) > AI_V2_SLIDE_YAW_CORROBORATION_RAD);
    const bool thresholdHit =
        (speedMps > AI_V2_SLIDE_MIN_SPEED_MPS) &&
        (fabsf(derived->bodySideslipRad) > AI_V2_SLIDE_SIDESLIP_RAD || yawExcursion);
    if (thresholdHit) {
        state->slideTicks++;
        state->slideCleanTicks = 0;
        if (state->slideTicks >= AI_V2_SLIDE_CONFIRM_TICKS) state->slideActive = true;
    } else {
        state->slideCleanTicks++;
        state->slideTicks = 0;
        if (state->slideActive && state->slideCleanTicks >= AI_V2_SLIDE_RELEASE_TICKS) {
            state->slideActive = false;
        }
    }

    /* --- Speed target: the friction-ellipse profile. --- */
    float muLat = 1.0f, muLong = 1.0f;
    available_grip(spec, track, runtime, posM, &muLat, &muLong);
    float targetSpeedMps =
        ai_v2_speed_target_mps(cfg, state, track, spec, bestSegment, speedMps, muLat, muLong);
    targetSpeedMps *= cfg->difficultyScale;
    /* Racecraft (issue #53): follow a slower car ahead and shift the line for a pass. The
     * helper is shared with the baseline driver and writes the same plan offsets. */
    if (cfg->trafficEnabled) {
        apply_traffic(cfg, state, track, traffic, speedMps, &targetSpeedMps);
    }

    /* --- Traction: slow backstop, plus throttle HOLD while sliding. --- */
    float pedalDemand;
    if (state->slideActive) {
        /* Hold the THROTTLE where it is: a chop unloads the rear and snaps the knife-edge AWD
         * (#79). But a hold must never outrank braking for the road ahead. Holding the whole
         * pedal axis is what pinned awd_gt, rwd_grip and rwd_power into the chicane barrier:
         * the monitor latched on a corner approach, the pedal froze mid-throttle for hundreds
         * of ticks, and the car arrived at ~37 m/s with the profile asking for ~13 (observed,
         * awd_gt tick 8312-8554). So a braking demand passes through — rate-limited like any
         * other, which is what stops it being a chop — and only throttle is held steady. */
        const float speedErrorMps = targetSpeedMps - speedMps;
        pedalDemand = (speedErrorMps < -cfg->speedDeadbandMps)
                          ? -clampf(-cfg->speedGainP * speedErrorMps, 0.0f, 1.0f)
                          : state->pedalAxis;
        steerAngleRad += clampf(-AI_V2_CS_YAW_GAIN * yawErrRadS -
                                    AI_V2_CS_BETA_GAIN * derived->bodySideslipRad,
                                -AI_V2_CS_MAX, AI_V2_CS_MAX);
    } else {
        if (derived->maxFrictionUsage >= cfg->gripCutThreshold) {
            state->gripCut += cfg->gripCutRatePerS * dt;
        } else {
            state->gripCut -= cfg->gripRecoverRatePerS * dt;
        }
        state->gripCut = clampf(state->gripCut, 0.0f, cfg->gripCutMax);

        const float speedErrorMps = targetSpeedMps - speedMps;
        if (speedErrorMps >= -cfg->speedDeadbandMps) {
            pedalDemand = clampf(1.0f - state->gripCut, 0.0f, 1.0f);
        } else {
            pedalDemand = -clampf(-cfg->speedGainP * speedErrorMps, 0.0f, 1.0f);
        }
    }

    /* --- Emit, rate-limited exactly like the baseline. --- */
    const float maxAngleRad = maxf(spec->maxRoadWheelAngleRad, 1.0e-3f);
    const float steerDemand =
        clampf(cfg->steerGainP * steerAngleRad / maxAngleRad, -1.0f, 1.0f);
    state->steerAxis = slew_axis(state->steerAxis, steerDemand, cfg->steerPressRatePerS,
                                 cfg->steerReleaseRatePerS, cfg->pedalDeadband, dt);
    out->steer = state->steerAxis;

    state->pedalAxis =
        slew_axis(state->pedalAxis, clampf(pedalDemand, -1.0f, 1.0f), cfg->pedalPressRatePerS,
                  cfg->pedalReleaseRatePerS, cfg->pedalDeadband, dt);
    out->throttle = maxf(state->pedalAxis, 0.0f);
    out->brake = maxf(-state->pedalAxis, 0.0f);
    out->handbrake = 0.0f;

    /* Diagnostics: the same fields the telemetry rows read, so the v2 driver is visible in
     * the same columns (ai_segment, ai_target_speed_mps, ...). */
    state->prevCrossTrackErrorM = crossTrackM;
    state->hasPrevError = true;
    state->nearestSegment = bestSegment;
    state->crossTrackErrorM = crossTrackM;
    state->targetSpeedMps = targetSpeedMps;
    state->lookaheadAngleRad = alphaRad;
    state->bindingCurvature1pm = ai_v2_plan_curvature_ahead(state, track, bestSegment, 2);
    state->bindingDistanceM = lookaheadM;
}
