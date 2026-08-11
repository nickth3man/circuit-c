/*
 * gameplay_tests.c — track geometry and surfaces, barrier collision, checkpoints and laps,
 * the particle pool, and the state machine.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "support/test_harness.h"
#include "support/simulation_fixture.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "content/roster_promotion.h"
#include "dev/car_corpus.h"
#include "game/ai_driver.h"
#include "game/car_roster.h"
#include "game/car_selection.h"
#include "game/setup_editor.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
#include "game/validation_classifier.h"
#include "game/validation_metrics.h"
#include "game/run_report.h"
#include "game/input.h"
#include "core/math_utils.h"
#include "game/particle.h"
#include "physics/physics.h"
#include "render/render.h"
#include "game/replay.h"
#include "game/telemetry.h"
#include "platform/timestep.h"
#include "physics/tire.h"
#include "core/units.h"
#include "world/collision.h"

/* ------------------------------------------------------------------------------------- */
/* Scenario: track surface                                                                 */
/* ------------------------------------------------------------------------------------- */

static void scenario_track_surface(void)
{
    /* Life-cycle: initialise, free, double-free safety. */
    TrackDefinition track;
    memset(&track, 0, sizeof(track));

    track_init(&track);
    check(track.nodes != NULL, "track init: nodes is non-NULL");
    check(track.count == 5, "track init: count == 5 (got %d)", track.count);
    check(track.offTrackSurfaceId == SURFACE_GRASS,
          "track init: offTrackSurfaceId is SURFACE_GRASS (got %d)",
          (int)track.offTrackSurfaceId);

    /* Loading content no longer writes anyone's lap state, so what used to be asserted about
     * the track is now asserted about a racer: a zeroed RacerProgress is a valid start of an
     * out-lap from gate 0, which is exactly what track_init() used to store in the track. */
    RacerProgress progress;
    memset(&progress, 0, sizeof(progress));
    check(progress.nextCheckpoint == 0, "zeroed progress: nextCheckpoint is 0");
    check(progress.lap == 0, "zeroed progress: lap is 0");
    check_near((double)progress.lapTimerS, 0.0, 0.0, "zeroed progress: lapTimerS is 0");

    /* Query the centre at (0, 0): inside the 200×150 m parking lot, so it should be asphalt. */
    const SurfaceId centerSurf = Track_SurfaceAt(&track, NULL, (Vector2){ 0.0f, 0.0f });
    check(centerSurf == SURFACE_ASPHALT, "Track_SurfaceAt origin returns ASPHALT (got %d)",
          (int)centerSurf);

    /* Query at a centreline node point: should be asphalt. */
    const SurfaceId nodeSurf = Track_SurfaceAt(&track, NULL, track.nodes[0].centerM);
    check(nodeSurf == SURFACE_ASPHALT,
          "Track_SurfaceAt(centreline node) returns ASPHALT (got %d)", (int)nodeSurf);

    /* Just inside the lot boundary: offset from a perimeter node by less than halfWidthM. */
    {
        const Vector2 insidePoint = { track.nodes[0].centerM.x,
                                      track.nodes[0].centerM.y +
                                          track.nodes[0].halfWidthM * 0.7f };
        const SurfaceId insideSurf = Track_SurfaceAt(&track, NULL, insidePoint);
        check(insideSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt inside boundary returns ASPHALT (got %d)", (int)insideSurf);
    }

    /* Just outside: (0, 200) is 50 m above the lot top at y = 150. */
    {
        const Vector2 outsidePoint = { 0.0f, 200.0f };
        const SurfaceId outsideSurf = Track_SurfaceAt(&track, NULL, outsidePoint);
        check(outsideSurf == SURFACE_GRASS,
              "Track_SurfaceAt outside boundary returns GRASS (got %d)", (int)outsideSurf);
    }

    /* Far away: (1000, 0). */
    {
        const SurfaceId farSurf = Track_SurfaceAt(&track, NULL, (Vector2){ 1000.0f, 0.0f });
        check(farSurf == SURFACE_GRASS, "Track_SurfaceAt far point returns GRASS (got %d)",
              (int)farSurf);
    }

    /* NULL / uninitialised track returns ASPHALT (defensive default). */
    {
        TrackDefinition dummy;
        memset(&dummy, 0, sizeof(dummy));
        const SurfaceId nullSurf = Track_SurfaceAt(NULL, NULL, (Vector2){ 0.0f, 0.0f });
        check(nullSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(NULL, ...) returns ASPHALT (got %d)", (int)nullSurf);
        const SurfaceId uninitSurf = Track_SurfaceAt(&dummy, NULL, (Vector2){ 0.0f, 0.0f });
        check(uninitSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(uninitialised, ...) returns ASPHALT (got %d)", (int)uninitSurf);
    }
    /* Track distance-to-centerline: on a centreline node, distance should be ~0. */
    {
        float hw = 0.0f;
        const float d = track_distance_to_centerline_m(&track, track.nodes[0].centerM, &hw);
        check(fabsf(d) < 0.01f, "centreline dist at node 0 ~0 (got %.2f)", (double)d);
        check(fabsf(hw - 4.0f) < 0.01f, "half-width at node 0 ~4 (got %.2f)", (double)hw);
    }
    /* Outside: y=200 is 50 m above lot top at y=150. */
    {
        float hw = 0.0f;
        const float d = track_distance_to_centerline_m(&track, (Vector2){ 0, 200 }, &hw);
        check(d > 40.0 && d < 60.0, "centreline dist outside lot ~50 m");
    }
    /* NULL/uninit: returns 0. */
    {
        float hw = -1.0f;
        TrackDefinition dummy;
        memset(&dummy, 0, sizeof(dummy));
        check_near((double)track_distance_to_centerline_m(NULL, (Vector2){ 0, 0 }, &hw), 0.0,
                   0.0, "NULL dist=0");
        check_near((double)hw, 0.0, 0.0, "NULL hw=0");
        hw = -1.0f;
        check_near((double)track_distance_to_centerline_m(&dummy, (Vector2){ 0, 0 }, &hw), 0.0,
                   0.0, "uninit dist=0");
        check_near((double)hw, 0.0, 0.0, "uninit hw=0");
    }
    /* Free and verify clean. */
    track_free(&track);
    check(track.nodes == NULL, "track_free: nodes is NULL");
    check(track.count == 0, "track_free: count is 0");

    /* Double-free safety. */
    track_free(&track);
    check(track.nodes == NULL, "track double-free: nodes stays NULL");
    check(track.count == 0, "track double-free: count stays 0");

    /* Re-init after free works. */
    track_init(&track);
    check(track.nodes != NULL, "track re-init: nodes is non-NULL");
    check(track.count == 5, "track re-init: count == 5");
    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-barrier — capsule vs track boundary, swept test, impulse response  */
/* ------------------------------------------------------------------------------------- */

static void scenario_collision_barrier(void)
{
    /* --- Barrier hit from straight approach: car aims DOWN at the
     *     parking-lot bottom boundary (bottom edge at y = -75 m, barrier at y ≈ -79 m) --- */
    Game *game = alloc_game();
    game_init(game);
    /* In headless builds game_init does NOT call track_init, so we must. */
    track_init(&game->trackDef);

    /* Place the car near the boundary, heading straight down at it.
     * The inner bottom barrier is at y ≈ -146 m (centerline -150 plus halfWidth 4). */
    game->vehicle.positionM = (Vector2){ 0.0f, -142.5f }; /* ~3.5 m above the inner barrier */
    game->vehicle.headingRad = -1.57079632679f;           /* pointing -Y (down) */
    game->vehicle.velocityLongitudinalMps = 30.0f;        /* body X forward = world -Y */
    game->vehicle.velocityLateralMps = 0.0f;
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.currHeadingRad = game->vehicle.headingRad;
    game->state = STATE_PLAYING;

    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -154). */
    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -146). */
    const float yBefore = game->vehicle.positionM.y;
    check(yBefore > -146.0f, "car starts inside the track boundary (y = %.2f > -146.0)",
          (double)yBefore);
    /* Run fixed updates at 120 Hz. The car moves ~0.25 m down per tick at 30 m/s.
     * Even with engine braking, 60 ticks (~0.5 s) is enough to reach y ≈ -79 m. */
    Input tickInput;
    input_zero(&tickInput);
    tickInput.throttle = 0.0f;
    tickInput.brake = 0.0f;

    bool hitBarrier = false;
    float speedBeforeHit = 30.0f;
    float speedAfterHit = 30.0f;
    float yAfter = -72.0f;

    for (int i = 0; i < 60; i++) {
        speedBeforeHit = game->derived.speedMps;
        game->input = tickInput;
        game_fixed_update(game, FIXED_DT_S);
        yAfter = game->vehicle.positionM.y;
        if (game->crashLockoutTimerS > 0.0f) {
            speedAfterHit = game->derived.speedMps;
            hitBarrier = true;
            break;
        }
        if (yAfter < -165.0f) break; /* car passed far beyond, collision didn't work */
    }

    /* After the tick, the car should NOT have passed through the boundary. */
    check(yAfter >= -147.5f,
          "car did not tunnel through the barrier (y = %.4f, must be > -147.5)",
          (double)yAfter);
    check(hitBarrier, "car hit the barrier (crashLockoutTimerS was set)");
    check(game->crashLockoutTimerS > 0.0f,
          "significant impact sets crashLockoutTimerS (%.4f > 0)",
          (double)game->crashLockoutTimerS);
    check(speedAfterHit < speedBeforeHit, "car lost speed from impact (%.2f < %.2f m/s)",
          (double)speedAfterHit, (double)speedBeforeHit);

    /* --- Decay of crash lockout timer --- */
    const float lockoutBefore = game->crashLockoutTimerS;
    game->input = tickInput; /* no input */
    game_fixed_update(game, FIXED_DT_S);
    check(game->crashLockoutTimerS < lockoutBefore, "crashLockoutTimerS decays (%.4f < %.4f)",
          (double)game->crashLockoutTimerS, (double)lockoutBefore);

    track_free(&game->trackDef);
    free(game);

    /* --- Glancing hit: car approaches at shallow angle to produce yaw spin --- */
    Game *game2 = alloc_game();
    game_init(game2);
    track_init(&game2->trackDef);
    /* Place car near the bottom of the lot, heading right-down at a shallow angle
     * toward the bottom barrier at y ≈ -146 m. */
    game2->vehicle.positionM = (Vector2){ 0.0f, -145.0f };
    game2->vehicle.headingRad = -1.2f; /* shallow angle heading right-down */
    game2->vehicle.velocityLongitudinalMps = 30.0f;
    game2->vehicle.yawRateRadS = 0.0f;
    game2->renderState.prevPositionM = game2->vehicle.positionM;
    game2->renderState.prevHeadingRad = game2->vehicle.headingRad;
    game2->renderState.currPositionM = game2->vehicle.positionM;
    game2->renderState.currHeadingRad = game2->vehicle.headingRad;
    game2->state = STATE_PLAYING;

    float peakYawRate = 0.0f;
    input_zero(&tickInput);
    /* Run ticks until we hit the barrier or pass through. */
    for (int i = 0; i < 60; i++) {
        game2->input = tickInput;
        game_fixed_update(game2, FIXED_DT_S);
        peakYawRate = fmaxf(peakYawRate, fabsf(game2->vehicle.yawRateRadS));
        if (game2->crashLockoutTimerS > 0.0f) break;
    }

    check(peakYawRate > 0.1f,
          "glancing hit produces measurable yaw rate (peak %.4f rad/s > 0.1)",
          (double)peakYawRate);

    track_free(&game2->trackDef);
    free(game2);
}
/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-units — direct tests for collision_resolve_track              */
/* ------------------------------------------------------------------------------------- */

/* A CollisionWorld is ~180 KB of fixed storage. A scenario that holds several of them at
 * once must heap-allocate rather than stack-allocate: the sanitizer build inflates frames
 * with redzones and overflows a default 1 MB stack. */
static CollisionWorld *alloc_collision_world(void)
{
    CollisionWorld *world = (CollisionWorld *)calloc(1, sizeof(CollisionWorld));
    if (world == NULL) {
        fprintf(stderr, "FATAL: could not allocate CollisionWorld (%zu bytes)\n",
                sizeof(CollisionWorld));
        exit(126);
    }
    return world;
}

static void scenario_collision_units(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    spec.bodyHalfWidthM = 0.85f;
    spec.collisionRestitution = 0.30f;
    spec.collisionFriction = 0.50f;
    spec.massKg = 1200.0f;
    spec.yawInertiaKgM2 = 1500.0f;
    vehicle_spec_refresh_derived(&spec);

    /* Standard parking-lot track: bottom wall centerline at y=-150, hw=4.
     * Bottom segment (nodes[0]→[1], dir=(1,0)):
     *   left  barrier at y = -146 (perp +4),  pushN = {0,+1} (up)
     *   right barrier at y = -154 (perp -4),  pushN = {0,+1} (up) */
    TrackDefinition track;
    memset(&track, 0, sizeof(track));
    track_init(&track);
    /* The collision world is the pipeline these tests exercise: the track's barriers become
     * its static shapes, and every resolve below runs through the world's candidate query. */
    CollisionWorld *world = alloc_collision_world();
    check(collision_world_build_from_track(world, &track),
          "the parking-lot barriers build into a collision world");

    /* 1. No collision: car dead centre. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, 0 };
        state.headingRad = 0.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n == 0, "no collision returns 0 (got %d)", n);
        check(lockout == 0.0f, "no collision leaves lockout at 0");
    }

    /* 2. Front circle penetrates the left barrier from the track side (y=-146.5, 0.5 m below
     *    the y=-146 barrier). Approaching at 5 m/s: penetration corrected + impulse applied. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -146.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = -5.0f; /* -Y, toward the y=-146 barrier */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const float yBefore = state.positionM.y;
        int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "left barrier contact resolves (got %d)", n);
        check(state.positionM.y > yBefore,
              "penetration push moves CG up, away from barrier (y %.4f > %.4f)",
              (double)state.positionM.y, (double)yBefore);
        check(lockout > 0.0f, "fast approach (5 m/s) sets crash lockout");
        check(state.velocityLateralMps < 5.0f, "impulse reduced lateral velocity (%.4f < 5.0)",
              (double)state.velocityLateralMps);
    }

    /* 3. Separating velocity: penetration corrected, NO impulse (velocity unchanged). */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -146.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = 5.0f; /* moving +Y, AWAY from the y=-146 barrier */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "separating contact still resolves penetration (got %d)", n);
        check(lockout == 0.0f, "separating contact does NOT set lockout");
        /* Velocity is unchanged because no impulse was applied (vn >= 0). */
        check_near((double)state.velocityLateralMps, 5.0, 1e-3,
                   "separating contact leaves velocity unchanged");
    }

    /* 4. Right barrier (bottom wall at y=-154): pushN = {0,+1} (up). */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -154.5f };
        state.headingRad = 0.0f;
        state.velocityLongitudinalMps = 0.0f;
        state.velocityLateralMps = -5.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const float yBefore = state.positionM.y;
        int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "right barrier contact resolves (got %d)", n);
        check(state.positionM.y > yBefore, "right barrier pushes CG up (y %.4f > %.4f)",
              (double)state.positionM.y, (double)yBefore);
    }
    /* 4b. Multi-contact: narrow corridor (hw=1.5), car at 90° spanning both walls.
     *     Front circle hits the left barrier (y=+1.5), rear hits the right (y=-1.5).
     *     These are DIFFERENT walls with opposing push normals, so both resolve. */
    {
        TrackNode corridorNodes[4] = {
            { { -50, 0 }, 1.5f, SURFACE_ASPHALT, 0.0f },
            { { 50, 0 }, 1.5f, SURFACE_ASPHALT, 0.0f },
            { { 50, 100 }, 50.0f, SURFACE_ASPHALT, 0.0f },
            { { -50, 100 }, 50.0f, SURFACE_ASPHALT, 0.0f },
        };
        TrackDefinition corridor = { 0 };
        corridor.nodes = corridorNodes;
        corridor.count = 4;
        corridor.offTrackSurfaceId = SURFACE_ASPHALT;
        CollisionWorld *corridorWorld = alloc_collision_world();
        check(collision_world_build_from_track(corridorWorld, &corridor),
              "the corridor barriers build into a collision world");

        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, 0 };
        state.headingRad = 1.57079632679f; /* 90 deg: body X = world +Y */
        /* Front circle at y approx +cgToFront, rear at y approx -cgToRear,
         * both within 0.85 of the +/-1.5 walls. */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(corridorWorld, 1u, &spec, &state, &rs, &lockout);
        check(n >= 2, "narrow corridor: both walls contacted -> >= 2 (got %d)", n);
        check(isfinite(state.positionM.x) && isfinite(state.positionM.y),
              "multi-contact position stays finite");
        free(corridorWorld);
    }

    /* 5. Lockout threshold: slow kiss does not trigger lockout. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -71.5f };
        state.headingRad = 0.0f;
        state.velocityLateralMps = 1.0f; /* below COLLISION_LOCKOUT_THRESHOLD_MPS (2.0) */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(lockout == 0.0f, "slow kiss (< 2 m/s) does not set lockout");
    }

    /* 6. Determinism: same input twice → identical output. */
    {
        VehicleState s1 = { 0 }, s2 = { 0 };
        VehicleRenderState r1 = { 0 }, r2 = { 0 };
        s1.positionM = s2.positionM = (Vector2){ 0, -71.5f };
        s1.headingRad = s2.headingRad = 0.0f;
        s1.velocityLateralMps = s2.velocityLateralMps = 5.0f;
        r1.prevPositionM = r1.currPositionM = s1.positionM;
        r2.prevPositionM = r2.currPositionM = s2.positionM;
        r1.prevHeadingRad = r1.currHeadingRad = s1.headingRad;
        r2.prevHeadingRad = r2.currHeadingRad = s2.headingRad;
        float lo1 = 0, lo2 = 0;
        collision_resolve_track(world, 1u, &spec, &s1, &r1, &lo1);
        collision_resolve_track(world, 1u, &spec, &s2, &r2, &lo2);
        check(memcmp(&s1, &s2, sizeof(VehicleState)) == 0,
              "collision_resolve_track is deterministic across identical calls");
        check(lo1 == lo2, "lockout is deterministic");
    }

    /* 7. No NaN or infinity in outputs. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -71.5f };
        state.headingRad = 0.0f;
        state.velocityLateralMps = 5.0f;
        state.yawRateRadS = 2.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(isfinite(state.positionM.x) && isfinite(state.positionM.y),
              "position is finite after collision");
        check(isfinite(state.velocityLongitudinalMps) && isfinite(state.velocityLateralMps),
              "velocity is finite after collision");
        check(isfinite(state.yawRateRadS), "yaw rate is finite after collision");
        check(isfinite(lockout), "lockout is finite");
    }

    /* 8. Variable-width segment: per-node half-widths produce a slanted barrier. A car
     *    near the narrow end collides under per-node interpolation. */
    {
        TrackNode vwNodes[4] = {
            { { 0, 0 }, 5.0f, SURFACE_ASPHALT, 0.0f },
            { { 10, 0 }, 10.0f, SURFACE_ASPHALT, 0.0f },
            { { 10, 100 }, 100.0f, SURFACE_ASPHALT, 0.0f },
            { { 0, 100 }, 100.0f, SURFACE_ASPHALT, 0.0f },
        };
        TrackDefinition vwTrack = { 0 };
        vwTrack.nodes = vwNodes;
        vwTrack.count = 4;
        vwTrack.offTrackSurfaceId = SURFACE_ASPHALT;
        CollisionWorld *vwWorld = alloc_collision_world();
        check(collision_world_build_from_track(vwWorld, &vwTrack),
              "the variable-width barriers build into a collision world");

        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 0, -5.5f }; /* 0.5 m below the narrow-end right barrier */
        state.headingRad = 0.0f;
        state.velocityLateralMps = -1.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        int n = collision_resolve_track(vwWorld, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "variable-width segment collides at the narrow end (per-node widths)");
        free(vwWorld);
    }

    /* 9. NULL / degenerate inputs return 0. */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        float lockout = 0.0f;
        check(collision_resolve_track(NULL, 1u, &spec, &state, &rs, &lockout) == 0,
              "NULL world returns 0");
        check(collision_resolve_track(world, 1u, NULL, &state, &rs, &lockout) == 0,
              "NULL spec returns 0");
        check(collision_resolve_track(world, 1u, &spec, NULL, &rs, &lockout) == 0,
              "NULL state returns 0");
        /* A world with no shapes — nothing to collide with. */
        CollisionWorld *empty = alloc_collision_world();
        collision_world_init(empty);
        check(collision_resolve_track(empty, 1u, &spec, &state, &rs, &lockout) == 0,
              "empty world returns 0");
        free(empty);
        /* A track with too few nodes builds no barriers. */
        TrackDefinition tiny = { 0 };
        tiny.count = 1;
        CollisionWorld *tinyWorld = alloc_collision_world();
        check(!collision_world_build_from_track(tinyWorld, &tiny),
              "track with < 2 nodes builds no collision world");
        check(collision_resolve_track(tinyWorld, 1u, &spec, &state, &rs, &lockout) == 0,
              "world from < 2 node track returns 0");
        free(tinyWorld);
    }

    track_free(&track);
    free(world);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-world — the deterministic CollisionWorld contract                  */
/* ------------------------------------------------------------------------------------- */

/* The box: nodes CCW from (0,0), hw 4, runoff 6 -> barriers at +/-6 from the centreline.
 * Closed loop, 4 edges -> 8 barrier shapes, ids in segment order (left then right per edge).
 * Each side is centreline +/- perp * 6 with perp = (-dir.y, dir.x); "left" is the +perp side:
 *   0 seg0 left  (y = +6),  1 seg0 right (y = -6)   edge (0,0)->(20,0),  dir (1,0),  perp (0,1)
 *   2 seg1 left  (x = 14),  3 seg1 right (x = 26)   edge (20,0)->(20,20), dir (0,1),  perp (-1,0)
 *   4 seg2 left  (y = 14),  5 seg2 right (y = 26)   edge (20,20)->(0,20), dir (-1,0), perp (0,-1)
 *   6 seg3 left  (x = 6),   7 seg3 right (x = -6)   edge (0,20)->(0,0),  dir (0,-1), perp (1,0)
 */
static void build_collision_box_track(TrackDefinition *track)
{
    memset(track, 0, sizeof(*track));
    track->nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track->count = 4;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    track->routeClosed = true;
    track->nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 4.0f, SURFACE_ASPHALT, 6.0f };
    track->nodes[1] = (TrackNode){ { 20.0f, 0.0f }, 4.0f, SURFACE_ASPHALT, 6.0f };
    track->nodes[2] = (TrackNode){ { 20.0f, 20.0f }, 4.0f, SURFACE_ASPHALT, 6.0f };
    track->nodes[3] = (TrackNode){ { 0.0f, 20.0f }, 4.0f, SURFACE_ASPHALT, 6.0f };
}

static void scenario_collision_world(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    spec.bodyHalfWidthM = 0.85f;
    spec.collisionRestitution = 0.30f;
    spec.collisionFriction = 0.50f;
    spec.massKg = 1200.0f;
    spec.yawInertiaKgM2 = 1500.0f;
    vehicle_spec_refresh_derived(&spec);

    /* ---- 1. Building the world: stable ids, canonical order, brute-mode at small sizes ---- */
    TrackDefinition track;
    build_collision_box_track(&track);
    CollisionWorld *world = alloc_collision_world();
    check(collision_world_build_from_track(world, &track), "box barriers build into a world");
    check(world->shapeCount == 8, "4 edges x 2 sides = 8 shapes (got %d)", world->shapeCount);
    check(!world->gridEnabled,
          "small worlds stay on the brute scan below COLLISION_WORLD_GRID_MIN_SHAPES");
    check(collision_world_add_static_segment(NULL, (Vector2){ 0, 0 }, (Vector2){ 1, 0 },
                                             (Vector2){ 0, 1 }, 0x1u) == -1,
          "NULL world rejects a segment");

    /* Stable shape ids run in the order the swept pass resolves them: edge 0 left first. */
    check(world->shapes[0].aM.x == 0.0f && world->shapes[0].aM.y == 6.0f &&
              world->shapes[0].bM.x == 20.0f && world->shapes[0].bM.y == 6.0f,
          "shape 0 is edge 0's left barrier at y = +6");
    check(world->shapes[1].aM.y == -6.0f, "shape 1 is edge 0's right barrier at y = -6");
    check(world->shapes[0].pushNormalM.y == -1.0f,
          "ribbon left barrier pushes into the track (toward -y)");
    check(world->shapes[0].layer == COLLISION_LAYER_STATIC_BARRIER,
          "barrier shapes carry the static barrier layer");

    /* Rebuilding from the same definition reproduces the same world bit for bit. */
    {
        CollisionWorld *again = alloc_collision_world();
        check(collision_world_build_from_track(again, &track), "second build succeeds");
        check(memcmp(&again->shapes, &world->shapes, sizeof(CollisionStaticShape) * 8) == 0,
              "the shape array is a pure function of the definition");
        free(again);
    }

    /* ---- 2. Authored objects share the query API, gated by collision layers ---- */
    {
        /* An authored barrier standing inside the box on its own layer (0x4: an object,
         * not a track barrier). */
        const CollisionShapeId objId = collision_world_add_static_segment(
            world, (Vector2){ 10.0f, -3.0f }, (Vector2){ 10.0f, 3.0f }, (Vector2){ -1, 0 },
            0x4u);
        check(objId == 8, "the authored object takes the next stable id (got %d)", (int)objId);
        check(collision_world_add_static_segment(world, (Vector2){ 0, 0 }, (Vector2){ 0, 0 },
                                                 (Vector2){ 1, 0 }, 0x4u) == -1,
              "zero-length authored segment rejected");
        check(collision_world_add_static_segment(world, (Vector2){ 0, 0 }, (Vector2){ 1, 0 },
                                                 (Vector2){ 2, 0 }, 0x4u) == -1,
              "non-unit push normal rejected");
        const float nan = NAN;
        check(collision_world_add_static_segment(world, (Vector2){ 0, nan }, (Vector2){ 1, 0 },
                                                 (Vector2){ 0, 1 }, 0x4u) == -1,
              "non-finite authored segment rejected");

        /* One query API answers for track barriers and authored objects alike. */
        CollisionShapeId ids[16];
        const int both = collision_world_query_static(
            world, (Vector2){ 9.5f, -0.5f }, (Vector2){ 10.5f, 0.5f },
            COLLISION_LAYER_STATIC_BARRIER | 0x4u, COLLISION_SHAPE_ID_NONE, ids, 16);
        check(both == 1 && ids[0] == objId,
              "authored object answers the same query the barriers answer (got %d ids)", both);
        const int barriersOnly = collision_world_query_static(
            world, (Vector2){ 9.5f, -0.5f }, (Vector2){ 10.5f, 0.5f },
            COLLISION_LAYER_STATIC_BARRIER, COLLISION_SHAPE_ID_NONE, ids, 16);
        check(barriersOnly == 0, "layer mask excludes the object from a barrier-only query");

        /* A body whose mask excludes the object layer passes through it; adding the layer to
         * the mask makes the same body stop. */
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 9.0f, 0.0f };
        state.headingRad = 0.0f;
        state.velocityLateralMps = -3.0f; /* moving -y, parallel to the object */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;

        CollisionWorld *objectWorld = alloc_collision_world();
        collision_world_build_from_track(objectWorld, &track);
        (void)collision_world_add_static_segment(objectWorld, (Vector2){ 10.0f, -3.0f },
                                                 (Vector2){ 10.0f, 3.0f }, (Vector2){ -1, 0 },
                                                 0x4u);
        CollisionBody body;
        memset(&body, 0, sizeof(body));
        body.id = 1u;
        body.layer = COLLISION_LAYER_VEHICLE_BODY;
        body.mask = COLLISION_LAYER_STATIC_BARRIER; /* object layer NOT in the mask */
        body.cgToFrontM = spec.cgToFrontM;
        body.cgToRearM = spec.cgToRearM;
        body.radiusM = spec.bodyHalfWidthM;
        body.prevPosM = body.currPosM = state.positionM;
        body.prevHdgRad = body.currHdgRad = state.headingRad;

        collision_world_begin_tick(objectWorld);
        check(collision_world_add_body(objectWorld, &body), "object body registered");
        CollisionBodyContext ctx = { .id = 1u,
                                     .spec = &spec,
                                     .state = &state,
                                     .renderState = &rs,
                                     .crashLockoutTimerS = &lockout };
        int n = collision_world_resolve_bodies(objectWorld, &ctx, 1);
        check(n == 0, "mask without the object layer yields no contact (got %d)", n);

        body.mask |= 0x4u;
        collision_world_begin_tick(objectWorld);
        check(collision_world_add_body(objectWorld, &body), "object body re-registered");
        n = collision_world_resolve_bodies(objectWorld, &ctx, 1);
        check(n >= 1, "mask including the object layer contacts the object (got %d)", n);
        check(objectWorld->contactCount >= 1 && objectWorld->contacts[0].shapeId == 8u,
              "the contact names the authored object's stable id");
        check(objectWorld->contacts[0].normalM.x == -1.0f,
              "the contact carries the object's push normal");
        free(objectWorld);
    }

    /* ---- 3. Penetration recovery: overlapping at rest is pushed out, not accelerated ---- */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ 10.0f, 5.5f }; /* front circle 0.5 m into the y=6 wall */
        state.headingRad = 0.0f;
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "overlap at rest resolves (got %d)", n);
        check_near((double)state.positionM.y, 5.15, 1e-3,
                   "penetration push moves the CG to exactly touching");
        check_near((double)state.velocityLateralMps, 0.0, 0.0,
                   "a resting overlap applies no impulse");
        check(lockout == 0.0f, "a resting overlap sets no crash lockout");
        check(world->contactCount >= 1 && world->contacts[0].approachSpeedMps == 0.0f,
              "the contact feed records the resting touch with zero approach speed");
    }

    /* ---- 4. Barrier corner: contact at a segment endpoint resolves on the push normal ---- */
    {
        VehicleState state = { 0 };
        VehicleRenderState rs = { 0 };
        state.positionM = (Vector2){ -1.6f, 6.4f }; /* front circle near the (0,6) endpoint */
        state.headingRad = 0.0f;
        state.velocityLateralMps = 3.0f; /* +y: approaching the y=6 wall */
        rs.prevPositionM = rs.currPositionM = state.positionM;
        rs.prevHeadingRad = rs.currHeadingRad = state.headingRad;
        float lockout = 0.0f;
        const float yBefore = state.positionM.y;
        const int n = collision_resolve_track(world, 1u, &spec, &state, &rs, &lockout);
        check(n >= 1, "corner contact resolves (got %d)", n);
        check(state.positionM.y < yBefore, "corner push moves the CG into the track");
        check(state.velocityLateralMps < 3.0f, "corner impulse reduced the approach speed");
        check(lockout > 0.0f, "a 3 m/s corner approach sets the crash lockout");
        check(world->contactCount >= 1 && world->contacts[0].approachSpeedMps > 2.0f,
              "the contact feed records the significant corner approach");
    }

    /* ---- 5. Multi-proxy ordering: bodies resolve in ascending id whatever the input order ---- */
    {
        CollisionWorld *multi = alloc_collision_world();
        collision_world_build_from_track(multi, &track);

        CollisionBody bodies[3];
        memset(bodies, 0, sizeof(bodies));
        const Vector2 poses[3] = { { 5.0f, 5.5f }, { 10.0f, 5.5f }, { 15.0f, 5.5f } };
        const CollisionBodyId ids[3] = { 3u, 5u, 7u };
        for (int i = 0; i < 3; i++) {
            bodies[i].id = ids[i];
            bodies[i].layer = COLLISION_LAYER_VEHICLE_BODY;
            bodies[i].mask = COLLISION_LAYER_STATIC_BARRIER;
            bodies[i].cgToFrontM = spec.cgToFrontM;
            bodies[i].cgToRearM = spec.cgToRearM;
            bodies[i].radiusM = spec.bodyHalfWidthM;
            bodies[i].prevPosM = bodies[i].currPosM = poses[i];
            bodies[i].prevHdgRad = bodies[i].currHdgRad = 0.0f;
        }

        /* Register 7, 3, 5 — the world sorts, the caller does not have to. */
        collision_world_begin_tick(multi);
        check(collision_world_add_body(multi, &bodies[2]), "body 7 registered");
        check(collision_world_add_body(multi, &bodies[0]), "body 3 registered");
        check(collision_world_add_body(multi, &bodies[1]), "body 5 registered");
        check(collision_world_add_body(multi, &bodies[0]) == false,
              "a duplicate id is rejected");
        check(collision_world_find_body(multi, 3u) == 0 &&
                  collision_world_find_body(multi, 5u) == 1 &&
                  collision_world_find_body(multi, 7u) == 2,
              "bodies are stored packed in ascending id order");

        VehicleState states[3];
        VehicleRenderState rs[3];
        float lockouts[3] = { 0.0f, 0.0f, 0.0f };
        CollisionBodyContext contexts[3];
        for (int i = 0; i < 3; i++) {
            memset(&states[i], 0, sizeof(states[i]));
            memset(&rs[i], 0, sizeof(rs[i]));
            states[i].positionM = poses[i];
            states[i].velocityLateralMps = 5.0f; /* approaching the y=6 wall */
            rs[i].prevPositionM = rs[i].currPositionM = poses[i];
            contexts[i] = (CollisionBodyContext){ .id = ids[i],
                                                  .spec = &spec,
                                                  .state = &states[i],
                                                  .renderState = &rs[i],
                                                  .crashLockoutTimerS = &lockouts[i] };
        }

        /* Contexts handed over in a different order than the bodies are stored. */
        CollisionBodyContext shuffled[3] = { contexts[2], contexts[0], contexts[1] };
        const int total = collision_world_resolve_bodies(multi, shuffled, 3);
        check(total >= 3,
              "three bodies against the wall resolve at least three contacts "
              "(got %d)",
              total);

        /* Contact order: ascending body id, shapes ascending within a body. */
        bool idsAscending = true;
        CollisionBodyId prevId = 0u;
        for (int i = 0; i < multi->contactCount; i++) {
            if (multi->contacts[i].bodyId < prevId) idsAscending = false;
            prevId = multi->contacts[i].bodyId;
        }
        check(idsAscending, "contact feed is ordered by ascending body id");
        check(multi->contacts[0].bodyId == 3u, "the first contact belongs to the lowest id");
        for (int i = 1; i < multi->contactCount; i++) {
            if (multi->contacts[i].bodyId == multi->contacts[i - 1].bodyId) {
                check(multi->contacts[i].shapeId >= multi->contacts[i - 1].shapeId,
                      "shapes are ascending within one body's contact run");
            }
        }

        /* Isolation: each body's outcome equals an isolated single-body resolve. */
        for (int i = 0; i < 3; i++) {
            VehicleState soloState;
            VehicleRenderState soloRs;
            float soloLockout = 0.0f;
            memset(&soloState, 0, sizeof(soloState));
            memset(&soloRs, 0, sizeof(soloRs));
            soloState.positionM = poses[i];
            soloState.velocityLateralMps = 5.0f;
            soloRs.prevPositionM = soloRs.currPositionM = poses[i];
            CollisionWorld *solo = alloc_collision_world();
            collision_world_build_from_track(solo, &track);
            (void)collision_resolve_track(solo, ids[i], &spec, &soloState, &soloRs,
                                          &soloLockout);
            check(memcmp(&soloState, &states[i], sizeof(VehicleState)) == 0,
                  "body %u resolves the same alone as in the multi-body pass", ids[i]);
            check(soloLockout == lockouts[i], "body %u lockout matches the solo run", ids[i]);
            free(solo);
        }

        /* Order independence: sorted contexts produce the same result as shuffled ones. */
        {
            VehicleState statesB[3];
            VehicleRenderState rsB[3];
            float lockoutsB[3] = { 0.0f, 0.0f, 0.0f };
            CollisionBodyContext sorted[3];
            for (int i = 0; i < 3; i++) {
                memset(&statesB[i], 0, sizeof(statesB[i]));
                memset(&rsB[i], 0, sizeof(rsB[i]));
                statesB[i].positionM = poses[i];
                statesB[i].velocityLateralMps = 5.0f;
                rsB[i].prevPositionM = rsB[i].currPositionM = poses[i];
                sorted[i] = (CollisionBodyContext){ .id = ids[i],
                                                    .spec = &spec,
                                                    .state = &statesB[i],
                                                    .renderState = &rsB[i],
                                                    .crashLockoutTimerS = &lockoutsB[i] };
            }
            collision_world_begin_tick(multi);
            for (int i = 0; i < 3; i++) {
                check(collision_world_add_body(multi, &bodies[i]), "body %u re-registered",
                      ids[i]);
            }
            const int totalB = collision_world_resolve_bodies(multi, sorted, 3);
            check(totalB == total, "sorted and shuffled context order resolve the same count");
            for (int i = 0; i < 3; i++) {
                check(memcmp(&statesB[i], &states[i], sizeof(VehicleState)) == 0,
                      "body %u state is independent of context order", ids[i]);
            }
            check(multi->contactCount == totalB && multi->contacts[0].bodyId == 3u,
                  "the contact feed after a sorted run matches the resolved count and "
                  "order");
        }

        /* A registered body without a context is a contract violation, not a skip. */
        collision_world_begin_tick(multi);
        check(collision_world_add_body(multi, &bodies[0]), "body re-registered for the "
                                                           "missing-context probe");
        /* One registered body (id 3), one context naming a different id: the registered
         * body has no context, which is the contract violation under test. */
        CollisionBodyContext orphan = contexts[1]; /* id 5, never registered this tick */
        check(collision_world_resolve_bodies(multi, &orphan, 1) == -1,
              "a registered body with no matching context is reported as -1");
        check(collision_world_resolve_bodies(multi, NULL, 0) == -1,
              "a null context array is reported as -1");
        free(multi);
    }

    /* ---- 6. The tick feed is a per-tick buffer, cleared by begin_tick ---- */
    {
        const int before = world->contactCount;
        check(before > 0, "precondition: the world holds contacts from earlier probes");
        collision_world_begin_tick(world);
        check(world->contactCount == 0 && world->bodyCount == 0,
              "begin_tick clears the feed and the bodies for the next tick");
    }

    track_free(&track);
    free(world);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: collision-broadphase — grid vs brute-force parity and measured workloads     */
/* ------------------------------------------------------------------------------------- */

/* Deterministic LCG for the property tests: fixed seeds, no rand() (which is not portable
 * across platforms). */
static uint32_t s_broadphaseRngState;

static uint32_t broadphase_rng_next(void)
{
    s_broadphaseRngState = s_broadphaseRngState * 1664525u + 1013904223u;
    return s_broadphaseRngState;
}

static float broadphase_rng_unit(void)
{
    /* Take the high 16 bits: in a power-of-two-modulus LCG, bit k has period 2^(k+1), so
     * the low bits are strongly correlated between consecutive draws. */
    return (float)(broadphase_rng_next() >> 16) / 65535.0f;
}

/* Independent brute-force candidate reference, implemented here rather than reusing the
 * module's brute path, so the grid has to agree with a scan nobody shares code with. */
static int reference_brute_query(const CollisionWorld *world, Vector2 minM, Vector2 maxM,
                                 uint32_t layerMask, CollisionShapeId afterId,
                                 CollisionShapeId *idsOut, int capacity)
{
    int count = 0;
    for (CollisionShapeId id = 0; id < (CollisionShapeId)world->shapeCount; id++) {
        if (afterId != COLLISION_SHAPE_ID_NONE && id <= afterId) continue;
        if ((layerMask & world->shapes[id].layer) == 0u) continue;
        if (!(world->shapes[id].minM.x <= maxM.x && world->shapes[id].maxM.x >= minM.x &&
              world->shapes[id].minM.y <= maxM.y && world->shapes[id].maxM.y >= minM.y)) {
            continue;
        }
        if (count < capacity) idsOut[count] = id;
        count++;
    }
    if (count > capacity) count = capacity;
    return count;
}

/* A closed ring route: constant segment length, like a real circuit's authored nodes. */
static void build_ring_track(TrackDefinition *track, int nodeCount, float radiusM)
{
    memset(track, 0, sizeof(*track));
    track->nodes = (TrackNode *)calloc((size_t)nodeCount, sizeof(TrackNode));
    track->count = nodeCount;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    track->routeClosed = true;
    for (int i = 0; i < nodeCount; i++) {
        const float a = 6.28318530718f * (float)i / (float)nodeCount;
        track->nodes[i] = (TrackNode){
            { cosf(a) * radiusM, sinf(a) * radiusM }, 4.0f, SURFACE_ASPHALT, 6.0f
        };
    }
}

/* A narrow wiggly corridor: walls ~1 m apart, narrower than the capsule diameter, so poses
 * penetrate deeply and the re-query path is stressed. */
static void build_corridor_track(TrackDefinition *track, int nodeCount)
{
    memset(track, 0, sizeof(*track));
    track->nodes = (TrackNode *)calloc((size_t)nodeCount, sizeof(TrackNode));
    track->count = nodeCount;
    track->offTrackSurfaceId = SURFACE_GRASS;
    track->runoffSurfaceId = SURFACE_GRASS;
    track->routeClosed = false;
    for (int i = 0; i < nodeCount; i++) {
        const float x = (float)i * 1.0f;
        const float y = 100.0f + sinf((float)i * 0.7f) * 0.4f;
        track->nodes[i] = (TrackNode){ { x, y }, 0.5f, SURFACE_ASPHALT, 0.0f };
    }
}

static void scenario_collision_broadphase(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    spec.bodyHalfWidthM = 0.85f;
    vehicle_spec_refresh_derived(&spec);

    /* ---- 1. The grid is enabled exactly when the measured threshold justifies it ---- */
    {
        TrackDefinition lot;
        memset(&lot, 0, sizeof(lot));
        track_init(&lot);
        CollisionWorld *lotWorld = alloc_collision_world();
        collision_world_build_from_track(lotWorld, &lot);
        /* The lot's closing node coincides with its first, so the last edge is zero-length
         * and skipped: 5 nodes -> 4 distinct edges -> 8 shapes. The legacy narrowphase
         * skipped that segment the same way. */
        check(lotWorld->shapeCount == 8,
              "parking lot: 4 distinct edges x 2 sides = 8 "
              "shapes (got %d, count %d)",
              lotWorld->shapeCount, lot.count);
        check(!lotWorld->gridEnabled, "parking lot is below the grid threshold");
        free(lotWorld);
        track_free(&lot);

        TrackDefinition ring;
        build_ring_track(&ring, 512, 200.0f);
        CollisionWorld *ringWorld = alloc_collision_world();
        collision_world_build_from_track(ringWorld, &ring);
        check(ringWorld->shapeCount == 1024, "ring of 512 nodes yields 1024 shapes");
        check(ringWorld->gridEnabled,
              "a circuit-scale world uses the grid (shape count %d >= %d)",
              ringWorld->shapeCount, COLLISION_WORLD_GRID_MIN_SHAPES);
        check(ringWorld->gridCols > 0 && ringWorld->gridRows > 0,
              "the grid has a real cell layout (%d x %d)", ringWorld->gridCols,
              ringWorld->gridRows);

        /* ---- 2. Property: grid candidates == brute-force reference over fixed-seed queries ---- */
        s_broadphaseRngState = 0xC0FFEEu;
        CollisionShapeId gridIds[2048];
        CollisionShapeId refIds[2048];
        int mismatches = 0;
        for (int q = 0; q < 2000; q++) {
            /* Random query box inside the ring's extent (radius 200 -> box from -220..220). */
            const float cx = (broadphase_rng_unit() - 0.5f) * 440.0f;
            const float cy = (broadphase_rng_unit() - 0.5f) * 440.0f;
            const float w = 1.0f + broadphase_rng_unit() * 8.0f;
            const Vector2 minM = { cx - w, cy - w };
            const Vector2 maxM = { cx + w, cy + w };
            const CollisionShapeId afterId =
                (CollisionShapeId)((broadphase_rng_next() >> 22) & 0x3FFu); /* high 10 bits */
            const int g = collision_world_query_static(
                ringWorld, minM, maxM, COLLISION_LAYER_STATIC_BARRIER, afterId, gridIds, 2048);
            const int r = reference_brute_query(
                ringWorld, minM, maxM, COLLISION_LAYER_STATIC_BARRIER, afterId, refIds, 2048);
            if (g != r || memcmp(gridIds, refIds, (size_t)g * sizeof(CollisionShapeId)) != 0) {
                mismatches++;
            }
        }
        check(mismatches == 0, "grid and brute-force agree on all %d queries (%d mismatches)",
              2000, mismatches);

        /* The module's own brute path must agree with the independent reference too. */
        ringWorld->gridEnabled = false;
        int bruteMismatches = 0;
        s_broadphaseRngState = 0xBEEFu;
        for (int q = 0; q < 1000; q++) {
            const float cx = (broadphase_rng_unit() - 0.5f) * 440.0f;
            const float cy = (broadphase_rng_unit() - 0.5f) * 440.0f;
            const float w = 1.0f + broadphase_rng_unit() * 8.0f;
            const Vector2 minM = { cx - w, cy - w };
            const Vector2 maxM = { cx + w, cy + w };
            const int g = collision_world_query_static(ringWorld, minM, maxM,
                                                       COLLISION_LAYER_STATIC_BARRIER,
                                                       COLLISION_SHAPE_ID_NONE, gridIds, 2048);
            const int r =
                reference_brute_query(ringWorld, minM, maxM, COLLISION_LAYER_STATIC_BARRIER,
                                      COLLISION_SHAPE_ID_NONE, refIds, 2048);
            if (g != r || memcmp(gridIds, refIds, (size_t)g * sizeof(CollisionShapeId)) != 0) {
                bruteMismatches++;
            }
        }
        check(bruteMismatches == 0,
              "module brute scan agrees with the reference (%d "
              "mismatches over 1000)",
              bruteMismatches);
        free(ringWorld);
        track_free(&ring);

        /* ---- 3. Property: swept resolution via grid == via brute, pose by pose ---- */
        TrackDefinition ring2;
        build_ring_track(&ring2, 256, 150.0f);
        CollisionWorld *gridWorld = alloc_collision_world();
        collision_world_build_from_track(gridWorld, &ring2);
        CollisionWorld *bruteWorld = alloc_collision_world();
        collision_world_build_from_track(bruteWorld, &ring2);
        bruteWorld->gridEnabled = false;

        s_broadphaseRngState = 0xDECAFu;
        int stateMismatches = 0;
        int contactMismatches = 0;
        for (int t = 0; t < 300; t++) {
            VehicleState sA = { 0 }, sB = { 0 };
            VehicleRenderState rA = { 0 }, rB = { 0 };
            const float x = (broadphase_rng_unit() - 0.5f) * 300.0f;
            const float y = (broadphase_rng_unit() - 0.5f) * 300.0f;
            const float hdg = broadphase_rng_unit() * 6.28318530718f;
            const float vx = (broadphase_rng_unit() - 0.5f) * 60.0f;
            const float vy = (broadphase_rng_unit() - 0.5f) * 60.0f;
            sA.positionM = sB.positionM = (Vector2){ x, y };
            sA.headingRad = sB.headingRad = hdg;
            sA.velocityLongitudinalMps = sB.velocityLongitudinalMps = vx;
            sA.velocityLateralMps = sB.velocityLateralMps = vy;
            sA.yawRateRadS = sB.yawRateRadS = (broadphase_rng_unit() - 0.5f) * 6.0f;
            /* Half the probes sweep a whole tick of travel; half are static poses. */
            const bool sweep = (t & 1) == 0;
            if (sweep) {
                const float dt = FIXED_DT_S;
                rA.prevPositionM = rB.prevPositionM =
                    (Vector2){ x - cosf(hdg) * vx * dt + sinf(hdg) * vy * dt,
                               y - sinf(hdg) * vx * dt - cosf(hdg) * vy * dt };
                rA.prevHeadingRad = rB.prevHeadingRad = hdg - 0.05f;
            } else {
                rA.prevPositionM = rB.prevPositionM = (Vector2){ x, y };
                rA.prevHeadingRad = rB.prevHeadingRad = hdg;
            }
            rA.currPositionM = rB.currPositionM = (Vector2){ x, y };
            rA.currHeadingRad = rB.currHeadingRad = hdg;

            float lockA = 0.0f, lockB = 0.0f;
            const int nA = collision_resolve_track(gridWorld, 1u, &spec, &sA, &rA, &lockA);
            const int nB = collision_resolve_track(bruteWorld, 1u, &spec, &sB, &rB, &lockB);
            if (nA != nB || memcmp(&sA, &sB, sizeof(VehicleState)) != 0 || lockA != lockB) {
                stateMismatches++;
            }
            if (nA >= 0 && nB >= 0 &&
                (gridWorld->contactCount != bruteWorld->contactCount ||
                 gridWorld->contactsOverflowed != bruteWorld->contactsOverflowed)) {
                contactMismatches++;
            } else if (nA >= 0 && nB >= 0) {
                for (int c = 0; c < gridWorld->contactCount; c++) {
                    const CollisionContact *ca = &gridWorld->contacts[c];
                    const CollisionContact *cb = &bruteWorld->contacts[c];
                    if (ca->bodyId != cb->bodyId || ca->shapeId != cb->shapeId ||
                        ca->pointM.x != cb->pointM.x || ca->pointM.y != cb->pointM.y ||
                        ca->normalM.x != cb->normalM.x || ca->normalM.y != cb->normalM.y ||
                        ca->approachSpeedMps != cb->approachSpeedMps) {
                        contactMismatches++;
                        break;
                    }
                }
            }
        }
        check(stateMismatches == 0,
              "grid and brute resolve every probe to the same vehicle state (%d mismatches "
              "over 300)",
              stateMismatches);
        check(contactMismatches == 0,
              "grid and brute produce the same contact feed (%d mismatches)",
              contactMismatches);
        free(gridWorld);
        free(bruteWorld);
        track_free(&ring2);

        /* ---- 4. Adversarial geometry: a corridor narrower than the capsule ---- */
        TrackDefinition corridor;
        build_corridor_track(&corridor, 300);
        CollisionWorld *cGrid = alloc_collision_world();
        collision_world_build_from_track(cGrid, &corridor);
        CollisionWorld *cBrute = alloc_collision_world();
        collision_world_build_from_track(cBrute, &corridor);
        cBrute->gridEnabled = false;

        s_broadphaseRngState = 0xABADu;
        int corridorMismatches = 0;
        for (int t = 0; t < 200; t++) {
            VehicleState sA = { 0 }, sB = { 0 };
            VehicleRenderState rA = { 0 }, rB = { 0 };
            const float x = broadphase_rng_unit() * 299.0f;
            const float y = 100.0f + (broadphase_rng_unit() - 0.5f) * 1.0f;
            const float hdg = (broadphase_rng_unit() - 0.5f) * 3.0f;
            sA.positionM = sB.positionM = (Vector2){ x, y };
            sA.headingRad = sB.headingRad = hdg;
            sA.velocityLongitudinalMps = sB.velocityLongitudinalMps =
                (broadphase_rng_unit() - 0.5f) * 30.0f;
            sA.velocityLateralMps = sB.velocityLateralMps =
                (broadphase_rng_unit() - 0.5f) * 30.0f;
            rA.prevPositionM = rB.prevPositionM = (Vector2){ x, y };
            rA.prevHeadingRad = rB.prevHeadingRad = hdg;
            rA.currPositionM = rB.currPositionM = (Vector2){ x, y };
            rA.currHeadingRad = rB.currHeadingRad = hdg;

            float lockA = 0.0f, lockB = 0.0f;
            (void)collision_resolve_track(cGrid, 1u, &spec, &sA, &rA, &lockA);
            (void)collision_resolve_track(cBrute, 1u, &spec, &sB, &rB, &lockB);
            if (memcmp(&sA, &sB, sizeof(VehicleState)) != 0 || lockA != lockB) {
                corridorMismatches++;
            }
        }
        check(corridorMismatches == 0,
              "narrow-corridor probes agree between grid and brute (%d mismatches over 200)",
              corridorMismatches);
        free(cGrid);
        free(cBrute);
        track_free(&corridor);

        /* ---- 5. Contact feed overflow is visible, never a dropped resolution ---- */
        {
            /* 1024 shapes whose segments all pass through the origin: one capsule at the
             * origin overlaps every one of them, far past the 256-entry feed cap. */
            CollisionWorld *dense = alloc_collision_world();
            collision_world_init(dense);
            for (int i = 0; i < 512; i++) {
                const float a = 6.28318530718f * (float)i / 512.0f;
                const Vector2 dir = { cosf(a), sinf(a) };
                const Vector2 perp = { -sinf(a), cosf(a) };
                const Vector2 aM = { dir.x * 5.0f, dir.y * 5.0f };
                const Vector2 bM = { -dir.x * 5.0f, -dir.y * 5.0f };
                (void)collision_world_add_static_segment(dense, aM, bM, perp, 0x1u);
                (void)collision_world_add_static_segment(dense, aM, bM,
                                                         (Vector2){ -perp.x, -perp.y }, 0x1u);
            }
            collision_world_finalize(dense);
            check(dense->shapeCount == 1024, "dense world holds 1024 shapes");

            VehicleState state = { 0 };
            VehicleRenderState rs = { 0 };
            state.positionM = (Vector2){ 0, 0 };
            rs.prevPositionM = rs.currPositionM = state.positionM;
            float lockout = 0.0f;
            const int n = collision_resolve_track(dense, 1u, &spec, &state, &rs, &lockout);
            check(n > COLLISION_WORLD_MAX_CONTACTS,
                  "resolution keeps resolving past the feed cap (%d contacts)", n);
            check(dense->contactCount == COLLISION_WORLD_MAX_CONTACTS,
                  "the feed records up to its cap");
            check(dense->contactsOverflowed, "overflow is flagged, not silent");
            free(dense);
        }
    }

    /* ---- 6. Benchmark: measured query cost at representative track/body counts. The
     * timing loop is gated behind CIRCUIT_COLLISION_BENCH (any value): the deterministic
     * parity assertions above always run, but ~4800 timed resolves must not slow every
     * scenario pass or flood its log. ---- */
    if (getenv("CIRCUIT_COLLISION_BENCH") != NULL) {
        struct BenchCase {
            const char *name;
            int nodes; /* 0 = build the parking lot instead of a ring */
        };
        const struct BenchCase cases[] = {
            { "parking-lot (10)", 0 }, /* filled below from real tracks */
            { "ring-128 (256)", 128 },
            { "ring-512 (1024)", 512 },
            { "ring-1024 (2048)", 1024 },
        };
        const int bodyCounts[] = { 1, 4, 8 };
        const int iterations = 200;

        printf("[collision-broadphase] benchmark: one collision_resolve_track call per "
               "body, %d "
               "iterations (lower is better)\n",
               iterations);
        printf("  %-20s %-8s %-14s %-14s\n", "track", "bodies", "grid (us)", "brute (us)");
        for (size_t ci = 0; ci < sizeof(cases) / sizeof(cases[0]); ci++) {
            TrackDefinition track;
            if (cases[ci].nodes == 0) {
                memset(&track, 0, sizeof(track));
                track_init(&track);
            } else {
                build_ring_track(&track, cases[ci].nodes, 200.0f);
            }
            for (size_t bi = 0; bi < sizeof(bodyCounts) / sizeof(bodyCounts[0]); bi++) {
                CollisionWorld *grid = alloc_collision_world();
                collision_world_build_from_track(grid, &track);
                CollisionWorld *brute = alloc_collision_world();
                collision_world_build_from_track(brute, &track);
                brute->gridEnabled = false;

                struct timespec t0, t1;
                timespec_get(&t0, TIME_UTC);
                for (int it = 0; it < iterations; it++) {
                    for (int b = 0; b < bodyCounts[bi]; b++) {
                        VehicleState st = { 0 };
                        VehicleRenderState rs = { 0 };
                        st.positionM = (Vector2){ (float)b * 0.5f - 100.0f, 0.0f };
                        st.velocityLongitudinalMps = 30.0f;
                        rs.prevPositionM = rs.currPositionM = st.positionM;
                        float lockout = 0.0f;
                        (void)collision_resolve_track(grid, (CollisionBodyId)(b + 1u), &spec,
                                                      &st, &rs, &lockout);
                    }
                }
                timespec_get(&t1, TIME_UTC);
                const double gridUs = ((double)(t1.tv_sec - t0.tv_sec) * 1e6 +
                                       (double)(t1.tv_nsec - t0.tv_nsec) / 1e3) /
                                      (double)(iterations * bodyCounts[bi]);

                timespec_get(&t0, TIME_UTC);
                for (int it = 0; it < iterations; it++) {
                    for (int b = 0; b < bodyCounts[bi]; b++) {
                        VehicleState st = { 0 };
                        VehicleRenderState rs = { 0 };
                        st.positionM = (Vector2){ (float)b * 0.5f - 100.0f, 0.0f };
                        st.velocityLongitudinalMps = 30.0f;
                        rs.prevPositionM = rs.currPositionM = st.positionM;
                        float lockout = 0.0f;
                        (void)collision_resolve_track(brute, (CollisionBodyId)(b + 1u), &spec,
                                                      &st, &rs, &lockout);
                    }
                }
                timespec_get(&t1, TIME_UTC);
                const double bruteUs = ((double)(t1.tv_sec - t0.tv_sec) * 1e6 +
                                        (double)(t1.tv_nsec - t0.tv_nsec) / 1e3) /
                                       (double)(iterations * bodyCounts[bi]);

                printf("  %-20s %-8d %-14.1f %-14.1f\n", cases[ci].name, bodyCounts[bi], gridUs,
                       bruteUs);
                free(grid);
                free(brute);
            }
            track_free(&track);
        }
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: checkpoint-lap — gate crossing, lap counting, forward-only, timer reset      */
/* ------------------------------------------------------------------------------------- */

/* Cross gate `n` of the 10x10 square below, travelling the way the gate faces. Returns the
 * event so a caller can assert on order as well as on the crossing itself. */
static TrackCheckpointEvent cross_square_gate(const TrackDefinition *track,
                                              RacerProgress *progress, int n)
{
    switch (n) {
        /* Gate 0 at (0,0) faces +X and spans y in [-2,+2]. */
        case 0:
            return track_update_checkpoints(track, progress, (Vector2){ -0.1f, 1.0f },
                                            (Vector2){ 0.1f, 1.0f });
        /* Gate 1 at (10,0) faces +Y and spans x in [8,12]. */
        case 1:
            return track_update_checkpoints(track, progress, (Vector2){ 10.0f, -0.1f },
                                            (Vector2){ 10.0f, 0.1f });
        /* Gate 2 at (10,10) faces -X and spans y in [8,12]. */
        case 2:
            return track_update_checkpoints(track, progress, (Vector2){ 10.1f, 10.0f },
                                            (Vector2){ 9.9f, 10.0f });
        /* Gate 3 at (0,10) faces -Y and spans x in [-2,+2]. */
        default:
            return track_update_checkpoints(track, progress, (Vector2){ 1.0f, 10.1f },
                                            (Vector2){ 1.0f, 9.9f });
    }
}

static void scenario_checkpoint_lap(void)
{
    /* A 10 m x 10 m counterclockwise square: (0,0) -> (10,0) -> (10,10) -> (0,10).
     * halfWidthM 2 m, so the car positions below are comfortably in-bounds. Gates are derived
     * from the nodes, which is the scheme a hand-built ribbon gets. */
    TrackDefinition track;
    RacerProgress progress;
    memset(&track, 0, sizeof(track));
    memset(&progress, 0, sizeof(progress));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRASS;
    track.runoffSurfaceId = SURFACE_GRASS;
    track.routeClosed = true;
    track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    check(track_build_checkpoints_from_nodes(&track), "gates derive from the node ribbon");
    check(track.checkpointCount == 4, "one gate per node (got %d)", track.checkpointCount);
    progress.nextCheckpoint = 0;

    /* --- Ordered traversal advances, and gate 0 is the finish line --- */
    /* Gate 0 closes a lap the moment it is taken, because it IS the start/finish. Starting
     * progress at gate 0 therefore scores a lap immediately; a standing start avoids that by
     * expecting gate 1 first, which is what track_reset_progress() sets up. */
    TrackCheckpointEvent ev = cross_square_gate(&track, &progress, 0);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 0 is an in-order crossing");
    check(ev.index == 0, "the event names gate 0 (got %d)", ev.index);
    check(ev.lapCompleted, "gate 0 is the finish line, so taking it closes a lap");
    check(progress.nextCheckpoint == 1, "nextCheckpoint is 1 after gate 0 (got %d)",
          progress.nextCheckpoint);

    ev = cross_square_gate(&track, &progress, 1);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 1 advances");
    check(!ev.lapCompleted, "an intermediate gate does not close a lap");
    check(progress.nextCheckpoint == 2, "nextCheckpoint is 2 after gate 1 (got %d)",
          progress.nextCheckpoint);

    ev = cross_square_gate(&track, &progress, 2);
    check(ev.crossed && !ev.outOfOrder, "crossing gate 2 advances");
    check(progress.nextCheckpoint == 3, "nextCheckpoint is 3 after gate 2 (got %d)",
          progress.nextCheckpoint);

    /* --- An out-of-order crossing is REPORTED and does not advance --- */
    /* Expecting gate 3, the car instead cuts across gate 1. The old scheme only ever looked
     * at the expected gate, so this was indistinguishable from driving nowhere. */
    ev = cross_square_gate(&track, &progress, 1);
    check(ev.crossed, "cutting to gate 1 is detected as a crossing");
    check(ev.outOfOrder, "...and is reported as out of order");
    check(ev.index == 1, "...naming the gate actually crossed (got %d)", ev.index);
    check(!ev.lapCompleted, "an out-of-order crossing cannot close a lap");
    check(progress.nextCheckpoint == 3,
          "out-of-order crossing does not advance progress (got %d)", progress.nextCheckpoint);

    /* --- Reverse crossing does not advance --- */
    progress.nextCheckpoint = 0;
    ev = track_update_checkpoints(&track, &progress, (Vector2){ 0.2f, 1.0f },
                                  (Vector2){ -0.2f, 1.0f });
    check(!ev.crossed, "reverse crossing of gate 0 does NOT advance");
    check(progress.nextCheckpoint == 0,
          "nextCheckpoint still 0 after reverse crossing (got %d)", progress.nextCheckpoint);

    /* --- A full lap: gates 1,2,3 then back through the finish line --- */
    progress.nextCheckpoint = 1;
    progress.lap = 0;
    progress.lapTimerS = 5.0f;
    check(cross_square_gate(&track, &progress, 1).crossed, "gate 1");
    check(cross_square_gate(&track, &progress, 2).crossed, "gate 2");
    check(cross_square_gate(&track, &progress, 3).crossed, "gate 3");
    check(progress.nextCheckpoint == 0, "nextCheckpoint wraps to the finish line (got %d)",
          progress.nextCheckpoint);
    check(progress.lap == 0, "no lap yet: the finish line has not been recrossed (got %d)",
          progress.lap);

    ev = cross_square_gate(&track, &progress, 0);
    check(ev.lapCompleted, "recrossing the finish line completes the lap");
    check(progress.lap == 1, "lap increments to 1 (got %d)", progress.lap);
    check_near((double)ev.lapTimeS, 5.0, 1e-3, "the event reports the completed lap time");
    check(progress.lapTimerS < 0.1f, "lapTimerS resets on lap completion (%.4f s)",
          (double)progress.lapTimerS);
    check(progress.lastLapTimeS > 4.5f,
          "lastLapTimeS records the completed lap time (%.4f s > 4.5)",
          (double)progress.lastLapTimeS);

    /* --- Timer accumulation --- */
    progress.lapTimerS = 0.0f;
    progress.lapTimerS += 0.5f;
    check_near((double)progress.lapTimerS, 0.5, 1e-6, "lapTimerS accumulates");

    /* --- Car outside the gate span does not trigger --- */
    /* Gate 0 spans y in [-2,+2]; crossing the line at y = 5 misses it. No other gate lies on
     * that path either, so this must not register as an out-of-order crossing. */
    progress.nextCheckpoint = 0;
    progress.lap = 0;
    ev = track_update_checkpoints(&track, &progress, (Vector2){ -0.1f, 5.0f },
                                  (Vector2){ 0.1f, 5.0f });
    check(!ev.crossed, "crossing outside the gate span does NOT advance");
    check(progress.nextCheckpoint == 0,
          "nextCheckpoint unchanged after out-of-bounds cross (got %d)",
          progress.nextCheckpoint);

    /* --- Stationary car does not trigger --- */
    ev = track_update_checkpoints(&track, &progress, (Vector2){ 0.0f, 1.0f },
                                  (Vector2){ 0.0f, 1.0f });
    check(!ev.crossed, "stationary car does NOT advance checkpoints");

    /* --- NULL/edge case safety --- */
    ev = track_update_checkpoints(NULL, &progress, (Vector2){ 0, 0 }, (Vector2){ 1, 0 });
    check(!ev.crossed && ev.index == -1,
          "track_update_checkpoints with NULL track reports nothing gracefully");

    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: checkpoint-lap-sf — the start/finish line closes laps through lapArmed        */
/* ------------------------------------------------------------------------------------- */
/* The SF line is the authoritative lap boundary (see the StartFinishLine comment in
 * track.h), authored independently of the gate sequence. These cases pin the latch rules:
 * the lap closes only when the route cursor has wrapped (every required gate crossed in
 * order) AND the line was actually crossed since the last close. */
static void scenario_checkpoint_lap_start_finish(void)
{
    /* Same 10 m x 10 m counterclockwise square as checkpoint-lap, so cross_square_gate()
     * applies: gate 0 at (0,0) faces +X, gate 1 at (10,0) faces +Y, gate 2 at (10,10)
     * faces -X, gate 3 at (0,10) faces -Y. */
    TrackDefinition track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRASS;
    track.runoffSurfaceId = SURFACE_GRASS;
    track.routeClosed = true;
    track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    check(track_build_checkpoints_from_nodes(&track), "gates derive from the node ribbon");
    check(track.checkpointCount == 4, "one gate per node (got %d)", track.checkpointCount);

    RacerProgress progress;
    TrackCheckpointEvent ev;

    /* --- Case 1: an SF overlapping an early gate awards no incomplete lap ---
     * The line sits on gate 1 (10,0) facing +Y, so taking the FIRST gate of the lap crosses
     * it. The old same-tick rule completed a lap right there; the latch rule waits for the
     * wrapped route. */
    memset(&progress, 0, sizeof(progress));
    track.hasStartFinish = true;
    track.startFinish.centerM = (Vector2){ 10.0f, 0.0f };
    track.startFinish.forwardUnit = (Vector2){ 0.0f, 1.0f };
    track.startFinish.halfWidthM = 10.0f;
    progress.lapArmed = true; /* stale state from an earlier run */
    track_reset_progress_at(&progress, &track, 0);
    check(!progress.lapArmed, "reset clears a stale SF latch");

    ev = cross_square_gate(&track, &progress, 1);
    check(ev.crossed && !ev.outOfOrder, "gate 1 is the expected crossing");
    check(!ev.lapCompleted,
          "crossing an SF that overlaps gate 1 does not close a lap before the route wraps");
    check(progress.lap == 0, "lap stays 0 after the first gate (got %d)", progress.lap);
    check(cross_square_gate(&track, &progress, 2).crossed, "gate 2");
    check(cross_square_gate(&track, &progress, 3).crossed, "gate 3");
    ev = cross_square_gate(&track, &progress, 0);
    check(ev.lapCompleted, "wrapped route with a latched SF crossing closes the lap");
    check(progress.lap == 1, "lap increments to 1 (got %d)", progress.lap);
    check(!progress.lapArmed, "the lap close consumes the SF latch");

    /* --- Case 2: an SF crossed on a gate-less tick still closes the lap ---
     * The line sits between gate 3 and gate 0, facing the direction of travel (-Y), so the
     * car crosses it on a tick that crosses no gate. The old code discarded that crossing
     * and no lap ever closed. */
    memset(&progress, 0, sizeof(progress));
    track.hasStartFinish = true;
    track.startFinish.centerM = (Vector2){ 0.0f, 5.0f };
    track.startFinish.forwardUnit = (Vector2){ 0.0f, -1.0f };
    track.startFinish.halfWidthM = 10.0f;
    track_reset_progress_at(&progress, &track, 0);
    progress.lapTimerS = 5.0f;
    check(cross_square_gate(&track, &progress, 1).crossed, "gate 1");
    check(cross_square_gate(&track, &progress, 2).crossed, "gate 2");
    check(cross_square_gate(&track, &progress, 3).crossed, "gate 3");
    ev = track_update_checkpoints(&track, &progress, (Vector2){ 0.0f, 6.0f },
                                  (Vector2){ 0.0f, 4.0f });
    check(!ev.crossed, "the tick between gates 3 and 0 crosses no gate");
    check(progress.nextCheckpoint == 0, "a gate-less tick does not advance the cursor (got %d)",
          progress.nextCheckpoint);
    ev = cross_square_gate(&track, &progress, 0);
    check(ev.lapCompleted,
          "an SF crossing latched on a gate-less tick closes the lap at the wrapped gate");
    check(progress.lap == 1, "lap increments to 1 (got %d)", progress.lap);
    check_near((double)ev.lapTimeS, 5.0, 1e-3, "the event reports the completed lap time");

    /* --- Case 3: wrapping without crossing the SF closes no lap ---
     * The line is narrower than gate 0 (spans y in [-1,+1] vs the gate's [-2,+2]), so the
     * finish gate can be taken outside the line. */
    memset(&progress, 0, sizeof(progress));
    track.hasStartFinish = true;
    track.startFinish.centerM = (Vector2){ 0.0f, 0.0f };
    track.startFinish.forwardUnit = (Vector2){ 1.0f, 0.0f };
    track.startFinish.halfWidthM = 1.0f;
    track_reset_progress_at(&progress, &track, 0);
    progress.lapTimerS = 5.0f;
    check(cross_square_gate(&track, &progress, 1).crossed, "gate 1");
    check(cross_square_gate(&track, &progress, 2).crossed, "gate 2");
    check(cross_square_gate(&track, &progress, 3).crossed, "gate 3");
    ev = track_update_checkpoints(&track, &progress, (Vector2){ -0.1f, 1.5f },
                                  (Vector2){ 0.1f, 1.5f });
    check(ev.crossed && !ev.outOfOrder, "gate 0 taken outside the SF span");
    check(!ev.lapCompleted, "a wrap that never crossed the SF line closes no lap");
    check(progress.lap == 0, "lap stays 0 (got %d)", progress.lap);
    check(!progress.lapArmed, "no SF crossing means no latch");
    /* The next lap crosses the SF at y = 0.5 together with gate 0, so it does close. */
    check(cross_square_gate(&track, &progress, 1).crossed, "gate 1 (lap 2)");
    check(cross_square_gate(&track, &progress, 2).crossed, "gate 2 (lap 2)");
    check(cross_square_gate(&track, &progress, 3).crossed, "gate 3 (lap 2)");
    ev = track_update_checkpoints(&track, &progress, (Vector2){ -0.1f, 0.5f },
                                  (Vector2){ 0.1f, 0.5f });
    check(ev.lapCompleted, "crossing the SF with the finish gate closes the lap");
    check(progress.lap == 1, "lap increments to 1 (got %d)", progress.lap);

    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: progress-isolation — two racers, one shared immutable TrackDefinition         */
/* ------------------------------------------------------------------------------------- */

/*
 * The reason the ownership split exists. A single Track used to hold one lap cursor, so a
 * second car could not be added without corrupting the first one's progress. Here two
 * RacerProgress values advance through the SAME definition in DIFFERENT orders, and neither
 * can move the other or the geometry they are both measured against.
 */
static void scenario_progress_isolation(void)
{
    /* The same 10x10 square scenario_checkpoint_lap uses, so cross_square_gate() applies. */
    TrackDefinition track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRASS;
    track.runoffSurfaceId = SURFACE_GRASS;
    track.routeClosed = true;
    track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    check(track_build_checkpoints_from_nodes(&track), "gates derive from the node ribbon");
    /* Bind a runtime so the definition can be proved untouched at the end of the session.
     * The runtime embeds the collision world (plain data, ~160 KB), so it lives on the heap
     * rather than the scenario's stack frame. */
    TrackRuntime *runtime = (TrackRuntime *)calloc(1, sizeof(TrackRuntime));
    if (runtime == NULL) {
        fprintf(stderr, "FATAL: could not allocate TrackRuntime (%zu bytes)\n",
                sizeof(TrackRuntime));
        exit(126);
    }
    track_runtime_bind(runtime, &track);
    const uint32_t hashAtBind = track_geometry_hash(&track);

    RacerProgress alice;
    RacerProgress bob;
    memset(&alice, 0, sizeof(alice));
    memset(&bob, 0, sizeof(bob));
    track_reset_progress_at(&alice, &track, 0);
    track_reset_progress_at(&bob, &track, 0);
    check(alice.nextCheckpoint == 1 && bob.nextCheckpoint == 1,
          "both racers start an out-lap expecting gate 1 (%d, %d)", alice.nextCheckpoint,
          bob.nextCheckpoint);

    /* --- Independent advance: Alice drives the route, Bob does not move --- */
    cross_square_gate(&track, &alice, 1);
    cross_square_gate(&track, &alice, 2);
    check(alice.nextCheckpoint == 3, "Alice advanced to gate 3 (got %d)", alice.nextCheckpoint);
    check(bob.nextCheckpoint == 1, "Bob did not move while Alice drove (got %d)",
          bob.nextCheckpoint);

    /* --- Different order: Bob takes gate 1 only, and is judged on his OWN cursor --- */
    TrackCheckpointEvent bobEv = cross_square_gate(&track, &bob, 1);
    check(bobEv.crossed && !bobEv.outOfOrder,
          "Bob's gate 1 is in-order for Bob even though Alice is already past it");
    check(bob.nextCheckpoint == 2, "Bob advanced to gate 2 (got %d)", bob.nextCheckpoint);
    check(alice.nextCheckpoint == 3, "Alice is unchanged by Bob's crossing (got %d)",
          alice.nextCheckpoint);

    /* An out-of-order crossing for one racer is not out-of-order for the other. */
    TrackCheckpointEvent aliceEv = cross_square_gate(&track, &alice, 1);
    check(aliceEv.crossed && aliceEv.outOfOrder,
          "gate 1 is out-of-order for Alice, who already took it");
    check(alice.nextCheckpoint == 3, "an out-of-order crossing does not advance Alice (got %d)",
          alice.nextCheckpoint);

    /* --- Independent timing --- */
    alice.lapTimerS = 4.0f;
    bob.lapTimerS = 9.0f;
    check_near((double)alice.lapTimerS, 4.0, 1e-6, "Alice keeps her own lap timer");
    check_near((double)bob.lapTimerS, 9.0, 1e-6, "Bob keeps his own lap timer");

    /* --- Resetting one racer leaves the other and the content alone --- */
    track_reset_progress_at(&bob, &track, 0);
    check(bob.nextCheckpoint == 1 && bob.lap == 0, "Bob's reset returns him to his out-lap");
    check_near((double)bob.lapTimerS, 0.0, 0.0, "Bob's reset zeroes his own timer");
    check(alice.nextCheckpoint == 3, "resetting Bob does not reset Alice (got %d)",
          alice.nextCheckpoint);
    check_near((double)alice.lapTimerS, 4.0, 1e-6,
               "resetting Bob does not clear Alice's timer");
    check(track.count == 4 && track.checkpointCount == 4,
          "resetting a racer does not reload track content");

    /* --- The shared definition was never written --- */
    check(track_geometry_hash(&track) == hashAtBind,
          "the shared geometry hash is unchanged across the session (%08x)", hashAtBind);
    check(track_runtime_definition_unchanged(runtime, &track),
          "the runtime confirms its bound definition was not mutated");

    free(runtime);
    track_free(&track);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: chicane-track — the validation circuit's shape, gates, and identity           */
/* ------------------------------------------------------------------------------------- */

static void scenario_chicane_track(void)
{
    TrackDefinition track;
    RacerProgress progress;
    memset(&track, 0, sizeof(track));
    memset(&progress, 0, sizeof(progress));
    track_load_chicane(&track);
    /* Loading the circuit no longer resets anyone's progress — a racer does that for itself,
     * which is what lets two of them share this definition. */
    track_reset_progress_at(&progress, &track, 0);

    check(track.nodes != NULL && track.count > 0, "the chicane allocates a centreline (%d)",
          track.count);
    check(!track.isParkingLot, "the chicane is a ribbon, not a parking lot");
    check(strcmp(track.version, "chicane_v2") == 0, "the track carries its version (%s)",
          track.version);

    /* --- The loop actually closes ---
     * Every consecutive node pair must be within a node spacing of each other, INCLUDING the
     * wrap from the last node back to the first. A loop that does not close leaves a long
     * phantom segment whose barriers cut straight across the circuit. */
    {
        float longestGapM = 0.0f;
        int longestAt = -1;
        for (int i = 0; i < track.count; i++) {
            const Vector2 a = track.nodes[i].centerM;
            const Vector2 b = track.nodes[(i + 1) % track.count].centerM;
            const float gap = sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
            if (gap > longestGapM) {
                longestGapM = gap;
                longestAt = i;
            }
        }
        check(longestGapM < 6.0f,
              "the loop closes: longest node gap is %.2f m at index %d (spacing is 4 m)",
              (double)longestGapM, longestAt);
    }

    /* --- Length is in the range the lap-time budget assumes --- */
    {
        const float lengthM = track_length_m(&track);
        check(lengthM > 600.0f && lengthM < 780.0f,
              "the lap is about 690 m, so a lap fits the replay buffer (got %.1f m)",
              (double)lengthM);
    }

    /* --- Gates --- */
    check(track.checkpointCount == 25, "twenty-five required gates (got %d)",
          track.checkpointCount);
    for (int i = 0; i < track.checkpointCount; i++) {
        const Checkpoint *c = &track.checkpoints[i];
        const float mag =
            sqrtf(c->forwardUnit.x * c->forwardUnit.x + c->forwardUnit.y * c->forwardUnit.y);
        check(fabsf(mag - 1.0f) < 1e-5f, "gate %d has a unit forward direction (|f| = %.6f)", i,
              (double)mag);
        check(c->required, "gate %d is required", i);
        check(c->halfWidthM > 0.0f, "gate %d has a positive span", i);
    }

    /* Every gate must sit on the racing surface, or a car driving the circuit correctly could
     * never cross it. This is the check that catches a gate placed from stale geometry. */
    for (int i = 0; i < track.checkpointCount; i++) {
        const SurfaceId at = Track_SurfaceAt(&track, NULL, track.checkpoints[i].centerM);
        check(at == SURFACE_ASPHALT, "gate %d sits on the racing surface (got %d)", i, (int)at);
    }

    /* --- A standing start expects gate 1, not gate 0 --- */
    check(progress.nextCheckpoint == 1,
          "a standing start on the finish line expects gate 1 next (got %d)",
          progress.nextCheckpoint);
    check(progress.lap == 0, "a fresh track has no completed laps");

    /* --- Start pose --- */
    {
        Vector2 startM = { 0.0f, 0.0f };
        float headingRad = 0.0f;
        check(track_start_pose(&track, &startM, &headingRad), "the track reports a start pose");
        check_near((double)headingRad, 0.0, 1e-5,
                   "the start pose faces +X along the near straight");
        check(Track_SurfaceAt(&track, NULL, startM) == SURFACE_ASPHALT,
              "the car starts on the racing surface");
    }

    /* --- The chicane genuinely displaces the centreline ---
     * Without this the "chicane" is a straight and the track tests nothing it was built for. */
    {
        float maxYM = 0.0f;
        for (int i = 0; i < track.count; i++) {
            if (track.nodes[i].centerM.y > maxYM) maxYM = track.nodes[i].centerM.y;
        }
        check(maxYM > 100.0f,
              "the chicane displaces the far straight (peak y = %.1f m, straight is at 90 m)",
              (double)maxYM);
    }

    /* --- P4: a car following the centreline never touches a barrier ---
     *
     * Barriers are built per segment, so on the inside of a curve consecutive segments form a
     * concave polyline that bulges toward the racing line, and at a joint a swept capsule can
     * catch on the corner. That would show up as phantom impacts scattered around the curves
     * and through the chicane, which is indistinguishable from a car that genuinely hit a
     * wall — so it has to be ruled out before any lap result can be believed.
     *
     * Walking the centreline is the sharpest available probe: it is where the car is supposed
     * to be, and it has the most clearance, so ANY contact here is geometry, never driving.
     */
    {
        VehicleSpec spec;
        vehicle_spec_set_default(&spec);
        vehicle_spec_refresh_derived(&spec);

        /* The centreline probe drives the swept narrowphase through the collision world. */
        CollisionWorld *world = alloc_collision_world();
        check(collision_world_build_from_track(world, &track),
              "the chicane barriers build into a collision world");
        check(world->shapeCount == 2 * track.count,
              "two barrier shapes per centreline edge (%d == 2 x %d)", world->shapeCount,
              track.count);

        int contactNodes = 0;
        int firstContactAt = -1;
        for (int i = 0; i < track.count; i++) {
            const Vector2 here = track.nodes[i].centerM;
            const Vector2 next = track.nodes[(i + 1) % track.count].centerM;
            const float headingRad = atan2f(next.y - here.y, next.x - here.x);

            VehicleState state;
            VehicleDerived derived;
            VehicleRenderState renderState;
            vehicle_state_reset(&spec, &state, &derived, &renderState);
            state.positionM = here;
            state.headingRad = headingRad;
            /* Sweep the whole previous segment, so joints are crossed rather than sampled. */
            renderState.prevPositionM =
                track.nodes[(i + track.count - 1) % track.count].centerM;
            renderState.prevHeadingRad = headingRad;
            renderState.currPositionM = here;
            renderState.currHeadingRad = headingRad;

            float lockoutS = 0.0f;
            if (collision_resolve_track(world, 1u, &spec, &state, &renderState, &lockoutS) >
                0) {
                contactNodes++;
                if (firstContactAt < 0) firstContactAt = i;
            }
        }
        check(contactNodes == 0,
              "a car on the centreline never touches a barrier: %d of %d nodes reported "
              "contact (first at index %d)",
              contactNodes, track.count, firstContactAt);
        free(world);
    }

    /* --- The geometry hash is stable and shape-sensitive --- */
    {
        const uint32_t hashA = track_geometry_hash(&track);
        TrackDefinition again;
        memset(&again, 0, sizeof(again));
        track_load_chicane(&again);
        check(track_geometry_hash(&again) == hashA,
              "the geometry hash is reproducible across loads");

        again.nodes[3].centerM.x += 0.5f;
        check(track_geometry_hash(&again) != hashA, "moving one node changes the hash");
        track_free(&again);
    }

    /* --- The technical layout is a distinct, tighter authored circuit. --- */
    {
        TrackDefinition technical;
        memset(&technical, 0, sizeof(technical));
        track_load_technical(&technical);
        check(technical.nodes != NULL && technical.count == track.count,
              "technical track keeps the sampled route (%d nodes)", technical.count);
        check(strcmp(technical.id, "technical") == 0, "technical track carries its id (%s)",
              technical.id);
        check(strcmp(technical.version, "technical_v2") == 0,
              "technical track carries its version (%s)", technical.version);
        check(track_geometry_hash(&technical) != track_geometry_hash(&track),
              "technical geometry hash differs from chicane");
        check(track_length_m(&technical) < track_length_m(&track) * 0.80f,
              "technical lap is materially shorter and tighter (%.1f m vs %.1f m)",
              (double)track_length_m(&technical), (double)track_length_m(&track));

        float narrowestHalfWidthM = 1.0e9f;
        for (int i = 0; i < technical.count; i++) {
            if (technical.nodes[i].halfWidthM < narrowestHalfWidthM)
                narrowestHalfWidthM = technical.nodes[i].halfWidthM;
        }
        check(narrowestHalfWidthM < 5.0f, "technical track narrows the racing surface (%.1f m)",
              (double)narrowestHalfWidthM);
        check(technical.checkpointCount == 25,
              "technical track preserves twenty-five required gates (%d)",
              technical.checkpointCount);

        Vector2 startM = { 0.0f, 0.0f };
        float headingRad = 0.0f;
        check(track_start_pose(&technical, &startM, &headingRad),
              "technical track reports a standing start pose");
        check(Track_SurfaceAt(&technical, NULL, startM) == SURFACE_ASPHALT,
              "technical start pose is on the racing surface");
        track_free(&technical);
    }

    track_free(&track);
    check(track.checkpoints == NULL && track.checkpointCount == 0,
          "track_free releases the gate array too");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: ai-lap — the baseline driver gets the default car round the chicane           */
/* ------------------------------------------------------------------------------------- */

/*
 * The one question this scenario exists to answer is whether the control law holds a racing
 * line at all. Everything it measures is therefore about the DRIVER, not the car: whether the
 * gates come in order, how far off the centreline it wanders, and whether it touches a wall.
 *
 * The AI is an ordinary entrant CONTROLLER: game_fixed_update() runs it in the same controller
 * stage a human entrant's latched sample goes through, and it emits the same ControllerOutput.
 * No branch is added to game.c for it, so there is nothing in the simulation that knows a lap
 * was driven by a program.
 */
static void scenario_ai_lap(void)
{
    Game *game = alloc_game();
    game_init(game);

    track_load_chicane(&game->trackDef);
    check(game->trackDef.checkpointCount == 25, "the chicane loaded with its gates (%d)",
          game->trackDef.checkpointCount);
    check(game_spawn_on_track(game), "the car was placed on the start line");

    /* The AI has no gear control, so it drives the way a player with the automatic box on
     * would. forwardOnly keeps it out of reverse, which it has no way to ask for. */
    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = true;
    game->state = STATE_PLAYING;
    /* The run is longer than a race, so it must say so: the results trigger would otherwise
     * stop simulating the car at the gameplay default and strand the last timed lap. */
    game->session.rules.targetLaps = VALIDATION_RUN_LAPS;

    controller_init(&game->controller, CONTROLLER_KIND_AI);
    const AiDriverConfig cfg = game->controller.config.ai;
    const AiDriverState *ai = &game->controller.memory.ai;

    const int budgetTicks = REPLAY_CAPACITY_TICKS; /* 300 s, the replay buffer's capacity */
    const int targetLaps = VALIDATION_RUN_LAPS;    /* out-lap, then the timed laps */

    float maxCrossTrackM = 0.0f;
    float maxSpeedMps = 0.0f;
    float speedSumMps = 0.0f;
    float peakFrictionUsage = 0.0f;
    int ticksNearLimit = 0;
    int outOfOrder = 0;
    int gatesTaken = 0;
    int collisions = 0;
    int offTrackTicks = 0;
    int ticksRun = 0;
    float lapTimeS[VALIDATION_RUN_LAPS] = { 0.0f };
    bool allFinite = true;
    bool handbrakeEverSet = false;
    bool bothPedalsEverSet = false;
    int fullThrottleTicks = 0;
    int longestFullThrottleRun = 0;
    int currentFullThrottleRun = 0;
    float throttleSumTicks = 0.0f;
    int brakingTicks = 0;
    float prevLockoutS = 0.0f;

    /* Pedal travel per tick, to prove the driver emits a signal a trigger could produce. */
    float prevThrottle = 0.0f;
    float maxThrottleStep = 0.0f;
    int throttleReversals = 0;
    float prevThrottleStep = 0.0f;

    /* And stick travel per tick, for the same reason. Steering used to be the one input with
     * no travel limit at all: whatever the pure-pursuit algebra produced went straight out,
     * so a plan that shifted between searches could step the stick in a single tick. */
    float prevSteer = 0.0f;
    float maxSteerStep = 0.0f;
    int steerReversals = 0;
    float prevSteerStep = 0.0f;

    for (int tick = 0; tick < budgetTicks && game->progress.lap < targetLaps; tick++) {
        game_fixed_update(game, FIXED_DT_S);
        ticksRun++;

        /* Everything below reads the output the AI controller emitted for THIS tick, which is
         * what the entrant's driver asked for before any assist rewrote it. */
        if (game->controllerOutput.handbrake != 0.0f) handbrakeEverSet = true;
        if (game->controllerOutput.throttle > 0.0f && game->controllerOutput.brake > 0.0f)
            bothPedalsEverSet = true;
        if (game->controllerOutput.throttle >= 0.999f) {
            fullThrottleTicks++;
            currentFullThrottleRun++;
            if (currentFullThrottleRun > longestFullThrottleRun)
                longestFullThrottleRun = currentFullThrottleRun;
        } else {
            currentFullThrottleRun = 0;
        }
        throttleSumTicks += game->controllerOutput.throttle;
        if (game->controllerOutput.brake > 0.0f) brakingTicks++;

        {
            const float step = game->controllerOutput.throttle - prevThrottle;
            if (tick > 0 && fabsf(step) > maxThrottleStep) maxThrottleStep = fabsf(step);
            if (step * prevThrottleStep < 0.0f &&
                (fabsf(step) > 1.0e-4f || fabsf(prevThrottleStep) > 1.0e-4f))
                throttleReversals++;
            prevThrottleStep = step;
            prevThrottle = game->controllerOutput.throttle;
        }

        {
            const float step = game->controllerOutput.steer - prevSteer;
            if (tick > 0 && fabsf(step) > maxSteerStep) maxSteerStep = fabsf(step);
            if (step * prevSteerStep < 0.0f &&
                (fabsf(step) > 1.0e-4f || fabsf(prevSteerStep) > 1.0e-4f))
                steerReversals++;
            prevSteerStep = step;
            prevSteer = game->controllerOutput.steer;
        }

        const TrackCheckpointEvent ev = game->lastCheckpointEvent;
        if (ev.crossed) {
            gatesTaken++;
            if (ev.outOfOrder) outOfOrder++;
            if (ev.lapCompleted && game->progress.lap >= 1 &&
                game->progress.lap <= VALIDATION_RUN_LAPS) {
                lapTimeS[game->progress.lap - 1] = ev.lapTimeS;
            }
        }

        if (game->crashLockoutTimerS > prevLockoutS) collisions++;
        prevLockoutS = game->crashLockoutTimerS;

        /* The car spawns on the gate centre, which is ON THE CENTRELINE, while the target is
         * the offset racing line — so the first second is a legitimate acquisition transient,
         * not a tracking failure. Measure the control law once it is established. */
        if (tick > 240 && fabsf(ai->crossTrackErrorM) > maxCrossTrackM)
            maxCrossTrackM = fabsf(ai->crossTrackErrorM);
        if (game->derived.speedMps > maxSpeedMps) maxSpeedMps = game->derived.speedMps;
        speedSumMps += game->derived.speedMps;
        if (game->derived.maxFrictionUsage > peakFrictionUsage)
            peakFrictionUsage = game->derived.maxFrictionUsage;
        if (game->derived.maxFrictionUsage > 0.80f) ticksNearLimit++;

        if (Track_SurfaceAt(&game->trackDef, &game->trackRuntime, game->vehicle.positionM) !=
            SURFACE_ASPHALT)
            offTrackTicks++;

        if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y) ||
            !isfinite(game->vehicle.yawRateRadS)) {
            allFinite = false;
        }
    }

    const float meanSpeedMps = (ticksRun > 0) ? speedSumMps / (float)ticksRun : 0.0f;

    printf("    ai-lap           laps %d/%d  gates %d  out %.2fs  timed %.2f/%.2f/%.2fs"
           "  mean %.1f m/s  peak %.1f m/s\n",
           game->progress.lap, targetLaps, gatesTaken, (double)lapTimeS[0], (double)lapTimeS[1],
           (double)lapTimeS[2], (double)lapTimeS[3], (double)meanSpeedMps, (double)maxSpeedMps);
    printf("    ai-lap           max |cross-track| %.2f m  off-track %.1f%%  collisions %d"
           "  out-of-order %d  ticks %d\n",
           (double)maxCrossTrackM,
           100.0 * (double)offTrackTicks / (double)(ticksRun ? ticksRun : 1), collisions,
           outOfOrder, ticksRun);
    printf("    ai-lap           peak friction usage %.3f  grip-limited %.1f%% of the lap\n",
           (double)peakFrictionUsage,
           100.0 * (double)ticksNearLimit / (double)(ticksRun ? ticksRun : 1));
    /* The driver commits to the power when the envelope allows it. This is measured as a
     * sustained stretch at the stop plus the mean pedal position over the lap, not as a share
     * of ticks pinned at exactly 1.0: the pedal is rate limited, so it spends part of every
     * application travelling, and time-at-the-rail alone would fall as pedal fidelity rose.
     * Holding full throttle for half a second is what "uses the envelope" actually means. */
    const float meanThrottle = (ticksRun > 0) ? throttleSumTicks / (float)ticksRun : 0.0f;
    printf("    ai-lap           throttle mean %.2f  full %.1f%%  longest full run %.2f s"
           "  max step/tick %.3f  reversals %.2f/s\n",
           (double)meanThrottle,
           100.0 * (double)fullThrottleTicks / (double)(ticksRun ? ticksRun : 1),
           (double)longestFullThrottleRun * (double)FIXED_DT_S, (double)maxThrottleStep,
           (double)throttleReversals / ((double)ticksRun * (double)FIXED_DT_S));
    check(longestFullThrottleRun >= 60,
          "the driver holds full throttle for at least 0.5 s somewhere on the lap (%.2f s)",
          (double)longestFullThrottleRun * (double)FIXED_DT_S);
    check(meanThrottle > 0.35f,
          "the driver uses the speed envelope rather than crawling "
          "(mean throttle %.2f)",
          (double)meanThrottle);
    check(brakingTicks > 0, "the driver brakes for upcoming curvature instead of coasting");

    /* The car is driven with analogue triggers, so the driver must produce a signal a trigger
     * could actually produce: bounded travel per tick, and no per-tick reversal storm. The
     * original controller read an instantaneous friction reading proportionally and emitted a
     * square wave — >0.5 swings on half of all frames — which no physical pedal can make. */
    const float pedalCeilingPerTick =
        fmaxf(cfg.pedalPressRatePerS, cfg.pedalReleaseRatePerS) * FIXED_DT_S;
    check(
        maxThrottleStep <= pedalCeilingPerTick + 1.0e-4f,
        "throttle never moves faster than the pedal can travel (max %.4f per tick, limit %.4f)",
        (double)maxThrottleStep, (double)pedalCeilingPerTick);
    check((double)throttleReversals / ((double)ticksRun * (double)FIXED_DT_S) < 8.0,
          "the throttle does not chatter (%.2f direction reversals per second)",
          (double)throttleReversals / ((double)ticksRun * (double)FIXED_DT_S));

    /* The same claim for the stick, and it is a stronger one to make: the steering demand is
     * recomputed from a plan that is itself re-searched ten times a second, so an unlimited
     * steer output could step every time the plan moved. These two checks are what make "the
     * inputs stay smooth" a gate rather than a hope.
     *
     * Measured with the limiter removed, this driver's worst steer step is 2.000 per tick — a
     * full lock-to-lock swing inside 1/120 s, which no stick can make. The limiter takes that
     * to 0.100 and costs 0.06 s a lap, less than the lap-to-lap spread. The reversal rate is
     * unchanged either way (7.82/s unlimited, 8.01/s limited), which is the evidence that
     * those reversals are the driver correcting rather than the limiter dithering. */
    const float steerCeilingPerTick =
        fmaxf(cfg.steerPressRatePerS, cfg.steerReleaseRatePerS) * FIXED_DT_S;
    printf("    ai-lap           steer max step/tick %.3f (limit %.3f)  reversals %.2f/s\n",
           (double)maxSteerStep, (double)steerCeilingPerTick,
           (double)steerReversals / ((double)ticksRun * (double)FIXED_DT_S));
    check(maxSteerStep <= steerCeilingPerTick + 1.0e-4f,
          "steering never moves faster than the stick can travel (max %.4f per tick, limit "
          "%.4f)",
          (double)maxSteerStep, (double)steerCeilingPerTick);
    check((double)steerReversals / ((double)ticksRun * (double)FIXED_DT_S) < 12.0,
          "the steering does not chatter (%.2f direction reversals per second)",
          (double)steerReversals / ((double)ticksRun * (double)FIXED_DT_S));

    /* --- What the driver is contractually forbidden from doing --- */
    check(!handbrakeEverSet, "the driver never pulls the handbrake");
    check(!bothPedalsEverSet, "the driver never applies throttle and brake together");

    /* --- Whether the control law works --- */
    check(allFinite, "the simulation stayed finite for the whole attempt");
    check(game->progress.lap >= targetLaps, "the driver completed %d laps (got %d in %d ticks)",
          targetLaps, game->progress.lap, ticksRun);
    check(outOfOrder == 0, "every gate was taken in order (%d out-of-order crossings)",
          outOfOrder);
    check(gatesTaken == targetLaps * game->trackDef.checkpointCount,
          "exactly %d gate crossings for %d laps (got %d)",
          targetLaps * game->trackDef.checkpointCount, targetLaps, gatesTaken);

    /* Two separate claims, now that the target is the learned line rather than the centreline:
     * the driver TRACKS its target, and it stays on the ROAD. The second is no longer implied
     * by the first, because the target line is itself displaced from the centreline.
     *
     * The tracking bound is the geometry's, not a taste: the narrowest racing surface on the
     * circuit is 6 m half-width, so a driver whose error exceeds 4 m is no longer following a
     * line — it is somewhere else on the road, or off it. */
    check(maxCrossTrackM < 4.0f,
          "the driver holds the learned racing line within 4 m (peak %.2f m)",
          (double)maxCrossTrackM);
    check(offTrackTicks == 0, "the driver never left the racing surface (%d ticks)",
          offTrackTicks);
    check(collisions == 0, "the driver never touched a barrier (%d contacts)", collisions);

    /* A driver that crawls proves nothing about the car, so the pace has to be real. Speed
     * alone is not enough: the driver has to be limited by GRIP rather than by its own
     * caution, because a lap time only separates one car from another when the tyres are the
     * thing running out. This is the check that would catch corneringGripFraction being
     * dialled down until every car laps identically. */
    check(meanSpeedMps > 15.0f, "the driver carries racing pace (mean %.1f m/s)",
          (double)meanSpeedMps);
    check(peakFrictionUsage > 0.90f, "the driver actually loads the tyres (peak usage %.3f)",
          (double)peakFrictionUsage);
    check(ticksNearLimit > ticksRun / 10,
          "the lap is grip-limited for a meaningful share of its length (%.1f%% above 0.80 "
          "usage)",
          100.0 * (double)ticksNearLimit / (double)(ticksRun ? ticksRun : 1));

    /* --- Determinism: the same driver and the same car reproduce the same lap --- */
    {
        Game *repeat = alloc_game();
        game_init(repeat);
        track_load_chicane(&repeat->trackDef);
        game_spawn_on_track(repeat);
        repeat->autoTrans.enabled = true;
        repeat->autoTrans.forwardOnly = true;
        repeat->state = STATE_PLAYING;
        repeat->session.rules.targetLaps = VALIDATION_RUN_LAPS;

        controller_init(&repeat->controller, CONTROLLER_KIND_AI);
        for (int tick = 0; tick < ticksRun; tick++) {
            game_fixed_update(repeat, FIXED_DT_S);
        }
        check(repeat->stateChecksum == game->stateChecksum,
              "the AI lap is deterministic across runs (%08x)", game->stateChecksum);
        track_free(&repeat->trackDef);
        free(repeat);
    }

    track_free(&game->trackDef);
    free(game);
}
/* ------------------------------------------------------------------------------------- */
/* Scenario: ai-no-privilege — per-tick checksum parity between live AI and input replay  */
/* ------------------------------------------------------------------------------------- */

static void scenario_ai_no_privilege(void)
{
    Game *game = alloc_game();
    game_init(game);
    track_load_chicane(&game->trackDef);
    game_spawn_on_track(game);

    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = true;
    game->state = STATE_PLAYING;

    controller_init(&game->controller, CONTROLLER_KIND_AI);

    const int runTicks = 3600; /* 30 s of driving */
    uint32_t *checksums = (uint32_t *)malloc(sizeof(uint32_t) * (size_t)runTicks);
    check(checksums != NULL, "checksum buffer allocated");
    if (checksums == NULL) {
        free(game);
        return;
    }

    replay_begin_recording(&game->replay, game->sim.tick);

    for (int t = 0; t < runTicks; t++) {
        game_fixed_update(game, FIXED_DT_S);
        checksums[t] = game->stateChecksum;
    }

    /* Replay the recorded inputs tick-by-tick and verify per-tick state checksum parity. */
    Game *repeat = alloc_game();
    game_init(repeat);
    track_load_chicane(&repeat->trackDef);
    game_spawn_on_track(repeat);
    repeat->autoTrans.enabled = true;
    repeat->autoTrans.forwardOnly = true;
    repeat->state = STATE_PLAYING;

    repeat->replay = game->replay;
    check(replay_begin_playback(&repeat->replay), "replay_begin_playback succeeded");

    int mismatches = 0;
    for (int t = 0; t < runTicks; t++) {
        input_zero(&repeat->input);
        game_fixed_update(repeat, FIXED_DT_S);
        if (repeat->stateChecksum != checksums[t]) mismatches++;
    }

    check(
        mismatches == 0,
        "AI driver has no side channels: %d / %d ticks match replay checksum byte-identically",
        runTicks - mismatches, runTicks);

    free(checksums);
    track_free(&repeat->trackDef);
    free(repeat);
    track_free(&game->trackDef);
    free(game);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: ai-roster-laps — one AiDriverConfig, the whole run, every car on the roster  */
/* ------------------------------------------------------------------------------------- */

static void scenario_ai_roster_laps(void)
{
    /* Milestone 1 acceptance criterion 10: one config drives all six cars. The shared default
     * is snapshotted here and re-checked against every entrant's frozen controller config after
     * its run, so per-car AI tuning -- the sanctioned way to make a car that cannot lap appear
     * to pass -- fails this scenario rather than quietly becoming the new baseline. */
    AiDriverConfig cfgAtStart;
    ai_driver_config_default(&cfgAtStart);

    const int rosterCount = car_roster_count();
    check(rosterCount == 6, "car roster holds 6 cars");

    for (int i = 0; i < rosterCount; i++) {
        VehicleSpec spec;
        check(car_roster_spec(i, &spec), "roster spec %d built successfully", i);

        char carId[64];
        car_roster_id(i, carId, sizeof(carId));

        Game *game = alloc_game();
        game_init(game);
        game_apply_spec(game, &spec);
        track_load_chicane(&game->trackDef);
        game_spawn_on_track(game);

        game->autoTrans.enabled = true;
        game->autoTrans.forwardOnly = true;
        game->state = STATE_PLAYING;
        game->session.rules.targetLaps = VALIDATION_RUN_LAPS;

        controller_init(&game->controller, CONTROLLER_KIND_AI);

        const int budgetTicks = REPLAY_CAPACITY_TICKS; /* 300 s max */
        const int maxRows = budgetTicks / 2;           /* telemetry sampled every 2 ticks */
        TelemetryRow *rows = (TelemetryRow *)calloc((size_t)maxRows, sizeof(TelemetryRow));
        /* The PASS assertion below must not be able to succeed vacuously: with rows == NULL the
         * classifier early-returns PASS on no rows, so the allocation itself is asserted
         * (PR #80 review). */
        check(rows != NULL, "car '%s' telemetry buffer allocated (%d rows)", carId, maxRows);
        int rowCount = 0;
        int outOfOrder = 0;
        bool allFinite = true;
        int ticksRun = 0;
        int collisions = 0;
        int offTrackTicks = 0;
        int stoppedTicks = 0;
        float speedSumMps = 0.0f;
        float prevLockoutS = 0.0f;

        for (int t = 0; t < budgetTicks && game->progress.lap < VALIDATION_RUN_LAPS; t++) {
            game_fixed_update(game, FIXED_DT_S);
            ticksRun++;

            if (game->lastCheckpointEvent.outOfOrder) outOfOrder++;
            if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y))
                allFinite = false;

            if (game->crashLockoutTimerS > prevLockoutS) collisions++;
            prevLockoutS = game->crashLockoutTimerS;
            if (Track_SurfaceAt(&game->trackDef, &game->trackRuntime,
                                game->vehicle.positionM) != SURFACE_ASPHALT)
                offTrackTicks++;
            if (game->derived.speedMps < 1.0f) stoppedTicks++;
            speedSumMps += game->derived.speedMps;

            /* Sample the same telemetry the validation runner classifies, so the roster's
             * per-car evidence comes from the SAME reducer rather than a second incompatible
             * definition (issue #78 §7). */
            if (ticksRun % 2 == 0 && rows != NULL && rowCount < maxRows) {
                rows[rowCount++] = game_telemetry_row(game, 1);
            }
        }

        /* The runner captures a final row when the loop ends on a crossing, because the
         * lap-closing tick is only sampled on even ticks and the loop stops the moment the
         * target laps are reached. Without this capture a lap closed on an odd tick would
         * leave the last sampled row one lap short and the classifier would misjudge the run
         * (PR #80 review). */
        if (game->lastCheckpointEvent.crossed && rows != NULL && rowCount < maxRows) {
            rows[rowCount++] = game_telemetry_row(game, 1);
        }

        /* Classify with the same inputs the runner uses; a passing car must come out PASS. */
        ValidationMetrics metrics;
        check(rowCount > 0, "car '%s' captured telemetry rows to classify (got %d)", carId,
              rowCount);
        validation_metrics_compute(rows, rowCount, &metrics);
        ValidationClassification cls;
        ClassificationInputs in;
        validation_classification_inputs_default(&in);
        in.checkpointCount = game->trackDef.checkpointCount;
        in.startCheckpointIndex = 0;
        in.targetLaps = VALIDATION_RUN_LAPS;
        in.tickBudget = budgetTicks;
        in.ticksRun = ticksRun;
        in.fixedDtS = FIXED_DT_S;
        validation_classify(rows, rowCount, &metrics, &in, &cls);

        /* Concise per-car line: a passing run carries its verdict; a failure names the primary
         * reason and the first causal tick, which is what makes the run diagnosable (issue #78
         * §7). */
        printf("    ai-roster-laps   %-10s laps %d  ticks %5d  mean %.1f m/s  off-track %.1f%%"
               "  stopped %.1f%%  collisions %d  primary %s  first-fault %llu\n",
               carId, game->progress.lap, ticksRun,
               (double)(ticksRun ? speedSumMps / (float)ticksRun : 0.0f),
               100.0 * (double)offTrackTicks / (double)(ticksRun ? ticksRun : 1),
               100.0 * (double)stoppedTicks / (double)(ticksRun ? ticksRun : 1), collisions,
               failure_class_reason(cls.primary), (unsigned long long)cls.firstFaultTick);
        check(allFinite, "car '%s' simulation stayed finite", carId);
        check(game->progress.lap >= VALIDATION_RUN_LAPS,
              "car '%s' completed %d laps (got %d in %d ticks)", carId, VALIDATION_RUN_LAPS,
              game->progress.lap, ticksRun);
        check(outOfOrder == 0, "car '%s' crossed all gates in order (%d out-of-order)", carId,
              outOfOrder);
        check(cls.primary == RUN_CLASS_PASS,
              "car '%s' completed run classifies as pass (got %s)", carId,
              failure_class_reason(cls.primary));
        check(memcmp(&game->controller.config.ai, &cfgAtStart, sizeof(cfgAtStart)) == 0,
              "car '%s' was driven with the unmodified shared AiDriverConfig", carId);

        free(rows);
        track_free(&game->trackDef);
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: planned-line — the driver's own search finds a legal path, and a faster one   */
/* ------------------------------------------------------------------------------------- */

/* Load one of the three authored validation circuits by index. */
static void load_validation_track(int which, TrackDefinition *track)
{
    if (which == 0)
        track_load_chicane(track);
    else if (which == 1)
        track_load_sprint(track);
    else
        track_load_technical(track);
}

/*
 * Drive two laps with the shared AI and report the timed (second) lap. Returns -1 on failure.
 *
 * `collapseCorridor` is the control condition. An edge margin wider than any half-width leaves
 * the search no lateral room at all, so every candidate offset clamps to zero and the plan IS
 * the authored centreline — same driver, same car, same start, same code path, the only
 * difference being whether the planner was allowed to look for a line. Driving the centreline
 * through the existing knob rather than through a second implementation is what makes the
 * comparison about the search and nothing else.
 */
static float drive_planned_lap_s(int which, bool collapseCorridor, int *gatesTakenOut,
                                 int *outOfOrderOut, int *offTrackTicksOut,
                                 float *planExcursionOut)
{
    Game *game = alloc_game();
    game_init(game);
    load_validation_track(which, &game->trackDef);
    game_spawn_on_track_at(game, 8);
    game->autoTrans.enabled = true;
    game->autoTrans.forwardOnly = true;
    game->state = STATE_PLAYING;

    controller_init(&game->controller, CONTROLLER_KIND_AI);
    if (collapseCorridor) game->controller.config.ai.planEdgeMarginM = 1.0e4f;
    const AiDriverState *ai = &game->controller.memory.ai;

    float timedLapS = -1.0f;
    float worstExcursionM = -1.0e9f;
    int gates = 0, outOfOrder = 0, offTrack = 0;
    for (int t = 0; t < 14400 && game->progress.lap < 2; t++) {
        game_fixed_update(game, FIXED_DT_S);

        const TrackCheckpointEvent ev = game->lastCheckpointEvent;
        if (ev.crossed) {
            gates++;
            if (ev.outOfOrder) outOfOrder++;
            if (ev.lapCompleted && game->progress.lap == 2) timedLapS = ev.lapTimeS;
        }
        if (Track_SurfaceAt(&game->trackDef, &game->trackRuntime, game->vehicle.positionM) !=
            SURFACE_ASPHALT)
            offTrack++;

        /* How close the path the driver chose itself came to running out of asphalt. Sampled
         * once a second: the plan only changes every planReplanTicks, and its window slides one
         * node in that time, so nothing is missed by not looking every tick. */
        if (t % 120 == 0) {
            for (int L = 0; L < ai->planLayerCount; L++) {
                const int node = (ai->planBaseNode + L) % game->trackDef.count;
                const Vector2 p = ai_driver_plan_point(ai, &game->trackDef, node);
                const float excursion =
                    track_distance_to_centerline_m(&game->trackDef, p, NULL) -
                    game->trackDef.nodes[node].halfWidthM;
                if (excursion > worstExcursionM) worstExcursionM = excursion;
            }
        }
    }

    if (gatesTakenOut != NULL) *gatesTakenOut = gates;
    if (outOfOrderOut != NULL) *outOfOrderOut = outOfOrder;
    if (offTrackTicksOut != NULL) *offTrackTicksOut = offTrack;
    if (planExcursionOut != NULL) *planExcursionOut = worstExcursionM;
    track_free(&game->trackDef);
    free(game);
    return timedLapS;
}

/*
 * The driver is handed a road, not a line. This scenario is what makes that trustworthy: it
 * proves the path the search returns is a legal one — inside the racing surface, through every
 * ordered gate — and that bothering to search for it is FASTER than driving down the middle of
 * the same road with the same car, the same controller, and the same start.
 *
 * It replaces the scenario that used to validate an offline-optimised line, and asserts more:
 * that line was a fixed artefact checked once, this one is re-derived from geometry every tenth
 * of a second by each car for itself, so the claim has to hold continuously.
 */
static void scenario_planned_line(void)
{
    const char *trackIds[3] = { "chicane", "sprint", "technical" };

    for (int which = 0; which < 3; which++) {
        int planGates = 0, planOutOfOrder = 0, planOffTrack = 0;
        int centreGates = 0, centreOutOfOrder = 0, centreOffTrack = 0;
        float planExcursionM = 0.0f, centreExcursionM = 0.0f;

        const float plannedLapS = drive_planned_lap_s(which, false, &planGates, &planOutOfOrder,
                                                      &planOffTrack, &planExcursionM);
        const float centreLapS = drive_planned_lap_s(
            which, true, &centreGates, &centreOutOfOrder, &centreOffTrack, &centreExcursionM);

        printf("    planned-line     %-9s  timed lap %.3f s (centreline %.3f s, %+.2f%%)"
               "  worst plan margin %.2f m\n",
               trackIds[which], (double)plannedLapS, (double)centreLapS,
               100.0 * ((double)plannedLapS - (double)centreLapS) / (double)centreLapS,
               (double)planExcursionM);

        check(plannedLapS > 0.0f, "%s: the planned line completed a timed lap (%.3f s)",
              trackIds[which], (double)plannedLapS);
        check(centreLapS > 0.0f, "%s: the centreline control completed a timed lap (%.3f s)",
              trackIds[which], (double)centreLapS);
        check(planGates == 2 * 25,
              "%s: the planned line took 50 ordered gates over 2 laps (got %d)",
              trackIds[which], planGates);
        check(planOutOfOrder == 0, "%s: the planned line crossed no gate out of order (%d)",
              trackIds[which], planOutOfOrder);
        check(planOffTrack == 0, "%s: the planned line never left the surface (%d ticks)",
              trackIds[which], planOffTrack);
        /* The corridor bounds the search, so this cannot pass by luck and cannot fail by
         * bad driving: it fails only if the candidate offsets stop being clamped. */
        check(planExcursionM <= 0.0f,
              "%s: every planned point stays on the racing surface (worst margin %.2f m)",
              trackIds[which], (double)planExcursionM);
        check(plannedLapS < centreLapS,
              "%s: searching for a line beats driving down the middle (%.3f s vs %.3f s)",
              trackIds[which], (double)plannedLapS, (double)centreLapS);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: track-runoff — three surface bands and barriers at the runoff edge           */
/* ------------------------------------------------------------------------------------- */

static void scenario_track_runoff(void)
{
    /* A straight ribbon along +X: 6 m racing half-width, barrier at 10 m. */
    TrackDefinition track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRAVEL;
    track.runoffSurfaceId = SURFACE_GRASS;
    for (int i = 0; i < 4; i++) {
        track.nodes[i].centerM = (Vector2){ (float)i * 20.0f, 0.0f };
        track.nodes[i].halfWidthM = 6.0f;
        track.nodes[i].runoffHalfWidthM = 10.0f;
        track.nodes[i].surfaceId = SURFACE_ASPHALT;
    }

    /* The three bands, sampled just inside each boundary. */
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, 0.0f }) == SURFACE_ASPHALT,
          "on the centreline is the racing surface");
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, 5.5f }) == SURFACE_ASPHALT,
          "inside halfWidthM is still the racing surface");
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, 7.0f }) == SURFACE_GRASS,
          "between halfWidthM and runoffHalfWidthM is runoff");
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, 9.5f }) == SURFACE_GRASS,
          "just inside the barrier is still runoff");
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, 12.0f }) == SURFACE_GRAVEL,
          "beyond the barrier is off-track");

    /* Symmetry: the bands are mirrored about the centreline. */
    check(Track_SurfaceAt(&track, NULL, (Vector2){ 20.0f, -7.0f }) == SURFACE_GRASS,
          "the runoff band is symmetric about the centreline");

    /* The barrier stands at the runoff edge, so a car sitting on the runoff band is NOT in
     * contact with it. This is the property that makes an off-track excursion measurable
     * rather than being identical to a wall strike. */
    check_near((double)track_node_barrier_half_width(&track.nodes[0]), 10.0, 1e-6,
               "the barrier stands at the runoff edge");

    /* A node with no runoff band keeps its barrier on the track edge, which is what every
     * ribbon built before runoff existed relies on. */
    TrackNode legacy = (TrackNode){ { 0.0f, 0.0f }, 4.0f, SURFACE_ASPHALT, 0.0f };
    check_near((double)track_node_barrier_half_width(&legacy), 4.0, 1e-6,
               "a node with no runoff keeps its barrier on the track edge");

    track_free(&track);
}

/*
 * lap-target-results: reaching RESULTS_TARGET_LAPS through a LIVE checkpoint crossing
 * (not a hand-set game->state) transitions STATE_PLAYING -> STATE_RESULTS.
 *
 * checkpoint-lap already proves track_update_checkpoints() itself; state-machine already
 * proves the RESULTS-state transitions once entered. Neither exercises the wiring
 * between them in game.c ("if (game->progress.lap >= RESULTS_TARGET_LAPS) ... state =
 * STATE_RESULTS") firing inside a real tick. lap is pre-set to RESULTS_TARGET_LAPS - 1 so
 * only the FINAL gate crossing is needed here — the crossing mechanics are
 * checkpoint-lap's job, not this scenario's.
 */
/* ------------------------------------------------------------------------------------- */
/* Scenario: race-session — lifecycle, phases, pause, restart, rules, and classification    */
/* ------------------------------------------------------------------------------------- */

/* Drive one whole simulating tick of a bare session: open it, then run its rules. The Game
 * path does the same thing with the physics and progress stages in between. */
static void race_session_tick(RaceSession *session, float dt)
{
    race_session_begin_tick(session, dt);
    race_session_update_rules(session);
}

/* Count how many events of one kind the session has raised, oldest first. */
static int session_event_count(const RaceSession *session, RaceEventKind kind)
{
    int found = 0;
    for (int i = 0; i < session->events.count; i++) {
        const int slot = (session->events.head + i) % RACE_EVENT_CAPACITY;
        if (session->events.items[slot].kind == kind) found++;
    }
    return found;
}

/* Spawn `count` AI entrants beside whatever the session already holds. */
static void add_ai_entrants(RaceSession *session, int count)
{
    for (int i = 0; i < count; i++) {
        const RaceEntrantSpawn spawn = { .controllerKind = CONTROLLER_KIND_AI, .gridSlot = -1 };
        check(race_roster_spawn(&session->roster, &spawn, NULL), "AI entrant %d spawned", i);
    }
}

/*
 * The race lifecycle, driven headlessly from configuration to classified results.
 *
 * What this asserts is the session authority itself: that phases move only along legal edges,
 * that a paused race freezes its own clock without disturbing the lap timing or the command
 * latch, that a restart really does return every piece of session state to its starting value,
 * and that the two finish rules differ in the one way they are supposed to.
 */
static void scenario_race_session(void)
{
    /* ---- 1. A fresh session is configured, not racing ---- */
    RaceSession *session = (RaceSession *)calloc(1, sizeof(RaceSession));
    check(session != NULL, "session fixture allocated");
    if (session == NULL) return;

    race_session_init(session);
    check(session->phase == RACE_PHASE_CONFIGURING,
          "an initialised session is CONFIGURING, not racing (got %d)", (int)session->phase);
    check(!race_session_is_simulating(session), "and it does not simulate");
    check(!race_session_pause(session), "a race that has not started cannot be paused");
    check(!race_session_abort(session), "nor aborted");

    /* ---- 2. Starting releases through the grid ---- */
    RaceRules rules;
    race_rules_set_default(&rules);
    rules.targetLaps = 2;
    race_session_start(session, &rules);
    check(session->phase == RACE_PHASE_RUNNING,
          "no countdown releases straight to RUNNING (got %d)", (int)session->phase);
    check(race_session_is_simulating(session), "a running session simulates");
    check(session_event_count(session, RACE_EVENT_PHASE_CHANGED) == 2,
          "the grid is still passed through, so a listener sees GRID then RUNNING (got %d)",
          session_event_count(session, RACE_EVENT_PHASE_CHANGED));

    /* ---- 3. A countdown holds the grid for exactly its configured number of ticks ---- */
    rules.countdownS = 3.0f * FIXED_DT_S;
    race_session_start(session, &rules);
    check(session->phase == RACE_PHASE_COUNTDOWN,
          "a configured countdown starts in COUNTDOWN (got %d)", (int)session->phase);
    check(race_session_is_simulating(session),
          "a countdown still simulates: the cars settle on the grid");

    /* A countdown of N ticks holds the car for N ticks — all of them. Releasing on the tick
     * that reaches zero would hand that tick's controls to the driver, because the pre-physics
     * gating reads the phase after the session has already advanced it. */
    for (int i = 0; i < 3; i++) {
        race_session_tick(session, FIXED_DT_S);
        check(session->phase == RACE_PHASE_COUNTDOWN,
              "countdown tick %d of 3 still holds the grid (got %d)", i + 1,
              (int)session->phase);
    }
    check(session->clockS == 0.0f, "and the race clock has not started yet (got %.6f)",
          (double)session->clockS);

    race_session_tick(session, FIXED_DT_S);
    check(session->phase == RACE_PHASE_RUNNING,
          "the next tick goes green, so all three held ticks were held (got %d)",
          (int)session->phase);
    check(session->tick == 4u, "the session ticked through the countdown (got %llu)",
          (unsigned long long)session->tick);

    /* The degenerate case the off-by-one hid: one tick of countdown must hold one tick. */
    {
        RaceRules oneTick;
        race_rules_set_default(&oneTick);
        oneTick.countdownS = FIXED_DT_S;
        race_session_start(session, &oneTick);
        check(session->phase == RACE_PHASE_COUNTDOWN, "a one-tick countdown starts held");
        race_session_tick(session, FIXED_DT_S);
        check(session->phase == RACE_PHASE_COUNTDOWN,
              "and is still held through its single tick rather than released before it "
              "(got %d)",
              (int)session->phase);
        race_session_tick(session, FIXED_DT_S);
        check(session->phase == RACE_PHASE_RUNNING, "then goes green");
    }

    /* Back to a session that is green from its first tick, for the clock checks below. */
    rules.countdownS = 0.0f;
    race_session_start(session, &rules);

    /* ---- 4. The race clock counts green-flag time only ---- */
    for (int i = 0; i < 10; i++) race_session_tick(session, FIXED_DT_S);
    const float greenClockS = session->clockS;
    check_near((double)greenClockS, (double)(10.0f * FIXED_DT_S), 1e-6,
               "ten green ticks advance the race clock by ten fixed steps");

    check(race_session_pause(session) && session->phase == RACE_PHASE_PAUSED,
          "a running race can be paused");
    for (int i = 0; i < 10; i++) race_session_tick(session, FIXED_DT_S);
    check(session->clockS == greenClockS,
          "and a paused race's clock does not move, however many ticks arrive (%.6f vs %.6f)",
          (double)session->clockS, (double)greenClockS);
    check(!race_session_pause(session), "pausing a paused race is refused, not stacked");
    check(race_session_resume(session) && session->phase == RACE_PHASE_RUNNING,
          "resume returns to the phase the pause interrupted");
    check(!race_session_resume(session), "and resuming a running race is refused");

    /* ---- 5. Restart returns every piece of session state to its start ---- */
    race_session_start(session, &rules);
    check(session->tick == 0u && session->clockS == 0.0f && session->classifiedCount == 0 &&
              !session->results.valid && session->events.totalAppended == 2u,
          "a restart rewinds clock, tick, classification, results and the event log");

    /* ---- 6. Time trial classifies on the first finisher; a race waits for the field ---- */
    {
        RaceSession *trial = (RaceSession *)calloc(1, sizeof(RaceSession));
        RaceSession *field = (RaceSession *)calloc(1, sizeof(RaceSession));
        check(trial != NULL && field != NULL, "mode fixtures allocated");
        if (trial != NULL && field != NULL) {
            RaceRules trialRules;
            race_rules_set_default(&trialRules);
            trialRules.targetLaps = 1;

            race_session_init(trial);
            add_ai_entrants(trial, 3);
            trialRules.mode = RACE_MODE_TIME_TRIAL;
            race_session_start(trial, &trialRules);

            race_session_init(field);
            add_ai_entrants(field, 3);
            trialRules.mode = RACE_MODE_RACE;
            race_session_start(field, &trialRules);

            /* Exactly one entrant completes the distance in each session. */
            trial->roster.entrants[1].progress.lap = 1;
            field->roster.entrants[1].progress.lap = 1;
            race_session_tick(trial, FIXED_DT_S);
            race_session_tick(field, FIXED_DT_S);

            check(trial->phase == RACE_PHASE_CLASSIFIED,
                  "a time trial is over when the timed driver is (got %d)", (int)trial->phase);
            check(field->phase == RACE_PHASE_FINISHING,
                  "a race with cars still running is FINISHING, not classified (got %d)",
                  (int)field->phase);
            check(field->roster.entrants[1].result.finishPosition == 1,
                  "the first car home takes position 1 (got %d)",
                  field->roster.entrants[1].result.finishPosition);

            /* The rest of the field finishes; positions follow the order they arrive in. */
            field->roster.entrants[2].progress.lap = 1;
            race_session_tick(field, FIXED_DT_S);
            field->roster.entrants[0].progress.lap = 1;
            race_session_tick(field, FIXED_DT_S);

            check(field->phase == RACE_PHASE_CLASSIFIED,
                  "the race classifies once the last car is home (got %d)", (int)field->phase);
            check(field->results.valid && field->results.count == 3,
                  "and the results snapshot holds every entrant (count %d)",
                  field->results.count);
            check(field->results.rows[0].finishPosition == 1 &&
                      field->results.rows[1].finishPosition == 2 &&
                      field->results.rows[2].finishPosition == 3,
                  "ordered by finishing position");
            check(field->results.rows[0].entrantId == field->roster.entrants[1].id &&
                      field->results.rows[2].entrantId == field->roster.entrants[0].id,
                  "and the order is the order they finished, not the order they are stored in");
            check(session_event_count(field, RACE_EVENT_ENTRANT_FINISHED) == 3,
                  "one finish event per entrant (got %d)",
                  session_event_count(field, RACE_EVENT_ENTRANT_FINISHED));
            check(race_session_is_over(field) && !race_session_is_simulating(field),
                  "a classified race is over and stops simulating");
        }
        free(field);
        free(trial);
    }

    /* ---- 7. Abort ends a race without classifying it ---- */
    race_session_start(session, &rules);
    check(race_session_abort(session) && session->phase == RACE_PHASE_ABORTED,
          "a running race can be abandoned");
    check(race_session_is_over(session) && !session->results.valid,
          "an aborted race is over and produces no results");
    check(!race_session_resume(session), "and cannot be resumed into");

    /* A race that already has an answer cannot be retrospectively abandoned: that would leave
     * phase ABORTED sitting next to a valid results snapshot. */
    add_ai_entrants(session, 1);
    {
        RaceRules oneLap;
        race_rules_set_default(&oneLap);
        oneLap.targetLaps = 1;
        race_session_start(session, &oneLap);
        session->roster.entrants[0].progress.lap = 1;
        race_session_tick(session, FIXED_DT_S);
        check(session->phase == RACE_PHASE_CLASSIFIED && session->results.valid,
              "precondition: the session classified");
        check(!race_session_abort(session) && session->phase == RACE_PHASE_CLASSIFIED &&
                  session->results.valid,
              "a classified race refuses to be aborted, so its results stay valid");
    }

    /* A countdown that is nonsense must not make the tick conversion undefined. */
    {
        RaceRules absurd;
        race_rules_set_default(&absurd);
        absurd.countdownS = 1.0e30f;
        race_session_start(session, &absurd);
        check(session->countdownTicksRemaining == RACE_COUNTDOWN_MAX_TICKS,
              "an oversized countdown clamps instead of overflowing the cast (got %d)",
              session->countdownTicksRemaining);
        check(session->phase == RACE_PHASE_COUNTDOWN, "and still holds the grid");
    }

    /* Every event a tick raises carries that tick's stamp, whichever stage raised it. */
    {
        RaceRules oneLap;
        race_rules_set_default(&oneLap);
        oneLap.targetLaps = 1;
        race_session_start(session, &oneLap);
        const int eventsBeforeTick = session->events.count; /* the start's own GRID/RUNNING */

        race_session_begin_tick(session, FIXED_DT_S);
        const uint64_t openedTick = session->tick;
        const float openedClockS = session->clockS;
        /* Stand in for the progress stage, which raises its lap event before the rules stage
         * runs — the two used to straddle the tick increment. */
        race_session_log_event(session, RACE_EVENT_LAP_COMPLETED,
                               session->roster.entrants[0].id, 1);
        session->roster.entrants[0].progress.lap = 1;
        race_session_update_rules(session);

        const RaceEvent *finish = race_session_last_event(session);
        check(finish != NULL && finish->kind == RACE_EVENT_PHASE_CHANGED,
              "the classifying tick ends on a phase change");
        int raisedThisTick = 0;
        bool sameStamp = true;
        for (int i = eventsBeforeTick; i < session->events.count; i++) {
            const RaceEvent *ev =
                &session->events.items[(session->events.head + i) % RACE_EVENT_CAPACITY];
            raisedThisTick++;
            if (ev->tick != openedTick || ev->timeS != openedClockS) sameStamp = false;
        }
        check(raisedThisTick == 3, "the tick raised lap, finish and phase events (got %d)",
              raisedThisTick);
        check(sameStamp, "and all three agree about which tick and clock they happened on");
    }

    free(session);

    /* ---- 8. The same lifecycle driven through Game, with pause preserving lap timing ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        check(game->session.phase == RACE_PHASE_RUNNING && game->state == STATE_PLAYING,
              "game_init() leaves a running session on the playing screen (phase %d, state %d)",
              (int)game->session.phase, (int)game->state);

        for (int i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);
        const float clockBefore = game->session.clockS;
        const float lapTimerBefore = game->progress.lapTimerS;
        const uint64_t sessionTickBefore = game->session.tick;

        /* Pause and an upshift latched on the same tick: the session must freeze and the
         * entrant's one-shot must still be consumed exactly once. */
        game->autoTrans.enabled = false;
        game->vehicle.selectedGear = 2;
        game->input.pausePressed = true;
        game->input.shiftUpPressed = true;
        game_fixed_update(game, FIXED_DT_S);

        check(game->session.phase == RACE_PHASE_PAUSED && game->state == STATE_PAUSED,
              "pause moves both axes together (phase %d, state %d)", (int)game->session.phase,
              (int)game->state);
        check(game->sim.shiftUpCount == 1u && game->vehicle.selectedGear == 3,
              "and the gear request latched on the pausing tick is still honoured once "
              "(count %u, gear %d)",
              game->sim.shiftUpCount, game->vehicle.selectedGear);

        for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->session.clockS == clockBefore && game->session.tick == sessionTickBefore,
              "a paused race freezes its own clock and tick across sixty ticks");
        check(game->progress.lapTimerS == lapTimerBefore,
              "and the lap timer with it (%.6f vs %.6f)", (double)game->progress.lapTimerS,
              (double)lapTimerBefore);
        check(
            game->sim.tick > sessionTickBefore,
            "while the application tick keeps counting, because it times the app not the race");

        game->input.pausePressed = true;
        game_fixed_update(game, FIXED_DT_S);
        check(game->session.phase == RACE_PHASE_RUNNING && game->state == STATE_PLAYING,
              "resume moves both axes back");
        check(game->session.clockS > clockBefore, "and the race clock starts advancing again");

        /* Reset restarts the session rather than merely moving the car. */
        game->input.resetPressed = true;
        game_fixed_update(game, FIXED_DT_S);
        check(game->session.phase == RACE_PHASE_RUNNING && game->session.clockS <= FIXED_DT_S,
              "a reset restarts the race from tick zero (phase %d, clock %.6f)",
              (int)game->session.phase, (double)game->session.clockS);

        /* And a restart taken on the final lap is a restart, not a bounce off the results
         * screen: the lap cursor has to go back with the clock. */
        game->progress.lap = game->session.rules.targetLaps;
        game->progress.lapTimerS = 12.0f;
        game->input.resetPressed = true;
        game_fixed_update(game, FIXED_DT_S);
        check(game->progress.lap == 0 && game->progress.lapTimerS == 0.0f,
              "a reset on the final lap rewinds route progress (lap %d, timer %.3f)",
              game->progress.lap, (double)game->progress.lapTimerS);
        check(game->state == STATE_PLAYING && game->session.phase == RACE_PHASE_RUNNING,
              "so the race actually restarts instead of classifying again immediately "
              "(state %d, phase %d)",
              (int)game->state, (int)game->session.phase);

        free(game);
    }

    /* ---- 9. Keeping a track keeps its identity ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        GameRunConfig config;
        memset(&config, 0, sizeof(config));
        snprintf(config.trackId, sizeof(config.trackId), "%s", "chicane");
        config.targetLaps = 3;
        check(game_configure_run(game, &config), "a chicane run configures");
        check(strcmp(game->session.trackId, "chicane") == 0,
              "the session records which track is loaded (got %s)", game->session.trackId);

        memset(&config, 0, sizeof(config));
        config.targetLaps = 3;
        check(game_configure_run(game, &config), "a follow-up run keeps that track");
        check(strcmp(game->session.trackId, "chicane") == 0,
              "and an empty trackId keeps its identity rather than reporting the sentinel "
              "(got %s)",
              game->session.trackId);
        check(game->session.rules.targetLaps == 3, "while the rules are re-frozen as asked");

        track_free(&game->trackDef);
        free(game);
    }

    /* ---- 9b. Missing or corrupt track ids are rejected ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        GameRunConfig config;
        memset(&config, 0, sizeof(config));
        snprintf(config.trackId, sizeof(config.trackId), "%s", "chicane");
        check(game_configure_run(game, &config), "precondition: configuring chicane succeeds");
        check(strcmp(game->session.trackId, "chicane") == 0,
              "precondition: session track is chicane (got %s)", game->session.trackId);
        memset(&config, 0, sizeof(config));
        snprintf(config.trackId, sizeof(config.trackId), "%s", "does_not_exist");
        check(!game_configure_run(game, &config),
              "configuring with a missing track id is rejected");
        check(strcmp(game->session.trackId, "chicane") == 0,
              "rejected configure leaves session track unchanged (got %s)",
              game->session.trackId);
        memset(&config, 0, sizeof(config));
        snprintf(config.trackId, sizeof(config.trackId), "%s", "Chicane");
        check(!game_configure_run(game, &config),
              "configuring with an uppercase track id is rejected");
        check(strcmp(game->session.trackId, "chicane") == 0,
              "uppercase rejection also leaves track unchanged (got %s)",
              game->session.trackId);
        track_free(&game->trackDef);
        free(game);
    }

    /* ---- 10. Playback rewinds the session, not only the car ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        replay_begin_recording(&game->replay, game->sim.tick);
        for (int i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);
        check(game->session.tick == 20u, "precondition: the live session advanced (got %llu)",
              (unsigned long long)game->session.tick);

        /* The dev lab's play button rewinds the car and begins playback. Whatever the live run
         * left the session at must not carry into the replay: a session that had reached a
         * non-simulating phase would make the playback do nothing at all. */
        game_reset_sim(game);
        check(replay_begin_playback(&game->replay), "playback begins");
        const uint64_t simTickBefore = game->sim.tick;
        game_fixed_update(game, FIXED_DT_S);

        check(game->session.tick == 1u,
              "the first playback tick rewinds the session rather than continuing the live one "
              "(got %llu)",
              (unsigned long long)game->session.tick);
        check_near((double)game->session.clockS, (double)FIXED_DT_S, 1e-6,
                   "and its race clock restarts from zero");
        check(
            game->sim.tick == simTickBefore + 1u,
            "while the application tick keeps counting, because it times the app not the race");
        check(race_session_is_simulating(&game->session),
              "so a replay always begins in a phase that actually simulates");

        free(game);
    }

    /* ---- 11. A rejected replay leaves the live race untouched ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        replay_begin_recording(&game->replay, game->sim.tick);
        for (int i = 0; i < 15; i++) game_fixed_update(game, FIXED_DT_S);

        /* Make the snapshot fail its own compatibility check — the recording no longer matches
         * the car in this Game. Rejecting the playback has to be non-destructive: the session
         * the player is in is not ours to clear on the way to refusing. */
        game->replay.initialVehicle.definitionHash ^= 0xFFFFFFFFu;

        const uint64_t sessionTickBefore = game->session.tick;
        check(replay_begin_playback(&game->replay),
              "playback begins on an incompatible replay");
        game_fixed_update(game, FIXED_DT_S);

        check(game->replay.mode != REPLAY_MODE_PLAYBACK,
              "the incompatible recording is rejected (mode %d)", (int)game->replay.mode);
        check(game->session.tick == sessionTickBefore + 1u,
              "and the live session carried on rather than being rewound on the way to "
              "refusing (tick %llu, expected %llu)",
              (unsigned long long)game->session.tick,
              (unsigned long long)(sessionTickBefore + 1u));

        free(game);
    }
}

static void scenario_lap_target_results(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* Reuse checkpoint-lap's tiny 10x10 square track (4 gates, CCW). */
    track_free(&game->trackDef);
    game->trackDef.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    game->trackDef.count = 4;
    game->trackDef.offTrackSurfaceId = SURFACE_GRASS;
    game->trackDef.routeClosed = true;
    game->trackDef.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->trackDef.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->trackDef.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    game->trackDef.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT, 0.0f };
    track_build_checkpoints_from_nodes(&game->trackDef);
    game->progress.nextCheckpoint =
        0; /* gate 0 is the finish line: crossing it completes a lap */
    game->progress.lap = RESULTS_TARGET_LAPS - 1;
    game->progress.lapTimerS = 1.0f;

    check(game->state == STATE_PLAYING, "precondition: game starts in STATE_PLAYING");

    /* Gate 0 is the finish line at (0,0), forward direction (+1,0): cross it moving +X at
     * y=1, exactly matching checkpoint-lap's own proven (-0.1 -> 0.1) crossing.
     *
     * The start of the tick is staged in currPositionM, not prevPositionM: physics shifts
     * curr into prev on entry, so curr is what holds the position the tick begins from. */
    game->renderState.currPositionM = (Vector2){ -0.1f, 1.0f };
    game->vehicle.positionM = (Vector2){ 0.1f, 1.0f };
    game->vehicle.headingRad = 0.0f;
    game->vehicle.velocityLongitudinalMps = 0.0f;
    game->vehicle.velocityLateralMps = 0.0f;
    game->vehicle.yawRateRadS = 0.0f;

    game_fixed_update(game, FIXED_DT_S);

    check(game->progress.lap >= RESULTS_TARGET_LAPS,
          "the crossing completes the target lap count (%d >= %d)", game->progress.lap,
          RESULTS_TARGET_LAPS);
    check(game->state == STATE_RESULTS,
          "reaching RESULTS_TARGET_LAPS transitions STATE_PLAYING -> STATE_RESULTS live, "
          "not by a hand-set game->state (got %d)",
          (int)game->state);
    track_free(&game->trackDef);
    free(game);
}

/* -------------------------------------------------------------------------------------
 * Phase 6 chunk [6c-1] particle-pool lifecycle test.
 * ------------------------------------------------------------------------------------- */
static void scenario_particle_pool(void)
{
    ParticlePool pool;
    int activeCount;

    /* --- Init: zeroes the pool, cursor at 0, everything inactive. --- */
    memset(&pool, 0xFF, sizeof(pool)); /* fill with junk to prove init overwrites */
    particle_pool_init(&pool);
    check(pool.cursor == 0, "init sets cursor to 0");

    activeCount = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        if (pool.particles[i].active) activeCount++;
    }
    check(activeCount == 0, "init deactivates all particles (got %d)", activeCount);

    /* --- Spawn: sets the fields and advances the cursor. --- */
    const Vector2 pos = { 1.0f, 2.0f };
    const Vector2 vel = { 3.0f, 4.0f };
    const Color col = { 200, 200, 200, 180 };
    particle_spawn(&pool, pos, vel, 0.30f, col);

    check(pool.cursor == 1, "spawn advances cursor to 1");
    check(pool.particles[0].active, "spawned particle is active");
    check_near((double)pool.particles[0].positionM.x, 1.0, 1e-6, "spawn sets position.x");
    check_near((double)pool.particles[0].positionM.y, 2.0, 1e-6, "spawn sets position.y");
    check_near((double)pool.particles[0].velocityMps.x, 3.0, 1e-6, "spawn sets velocity.x");
    check_near((double)pool.particles[0].velocityMps.y, 4.0, 1e-6, "spawn sets velocity.y");
    check_near((double)pool.particles[0].lifeS, (double)PARTICLE_LIFE_S, 1e-6,
               "spawn sets life to PARTICLE_LIFE_S");
    check_near((double)pool.particles[0].maxLifeS, (double)PARTICLE_LIFE_S, 1e-6,
               "spawn sets maxLife");
    check(pool.particles[0].sizeM == 0.30f, "spawn sets sizeM");
    check(pool.particles[0].color.r == 200 && pool.particles[0].color.g == 200 &&
              pool.particles[0].color.b == 200 && pool.particles[0].color.a == 180,
          "spawn sets color exactly");

    /* --- Round-robin wrap: after MAX_PARTICLES spawns, cursor returns to 0. --- */
    for (int i = 1; i < MAX_PARTICLES; i++) {
        particle_spawn(&pool, pos, vel, 0.30f, col);
    }
    check(pool.cursor == 0, "cursor wraps to 0 after %d spawns (got %d)", MAX_PARTICLES,
          pool.cursor);

    /* The slot 0 was overwritten by the last wrap-around spawn. */
    check(pool.particles[0].active, "round-robin re-activates slot 0 after wrap");

    /* --- Update: integrates velocity and decays life for active particles. --- */
    ParticlePool pool2;
    particle_pool_init(&pool2);
    particle_spawn(&pool2, (Vector2){ 0.0f, 0.0f }, (Vector2){ 10.0f, 0.0f }, 0.30f, col);
    pool2.particles[0].lifeS = 1.0f;
    pool2.particles[0].maxLifeS = 1.0f;

    particle_pool_update(&pool2, 0.50f);
    check_near((double)pool2.particles[0].positionM.x, 5.0, 1e-6,
               "update integrates x (10 m/s * 0.5 s)");
    check_near((double)pool2.particles[0].lifeS, 0.5, 1e-6, "update reduces life by dt");

    /* --- Update: deactivates particle when life reaches zero. --- */
    pool2.particles[0].lifeS = 0.10f;
    particle_pool_update(&pool2, 0.20f);
    check(!pool2.particles[0].active, "particle deactivates when life drops to or below 0");

    /* --- Update: does not move inactive particles. --- */
    ParticlePool pool3;
    particle_pool_init(&pool3);
    particle_spawn(&pool3, (Vector2){ 0.0f, 0.0f }, (Vector2){ 5.0f, 5.0f }, 0.30f, col);
    pool3.particles[0].active = false;
    const Vector2 savedPos = pool3.particles[0].positionM;
    particle_pool_update(&pool3, 0.50f);
    check_near((double)pool3.particles[0].positionM.x, (double)savedPos.x, 1e-6,
               "inactive particle position.x unchanged");
    check_near((double)pool3.particles[0].positionM.y, (double)savedPos.y, 1e-6,
               "inactive particle position.y unchanged");

    /* --- Update: zero or negative dt is a no-op. --- */
    ParticlePool pool4;
    particle_pool_init(&pool4);
    particle_spawn(&pool4, (Vector2){ 0.0f, 0.0f }, (Vector2){ 5.0f, 0.0f }, 0.30f, col);
    pool4.particles[0].lifeS = 1.0f;
    const float savedLife = pool4.particles[0].lifeS;
    particle_pool_update(&pool4, 0.0f);
    check(pool4.particles[0].active, "zero-dt update keeps particle active");
    check_near((double)pool4.particles[0].positionM.x, 0.0, 1e-6,
               "zero-dt update does not move particle");
    check_near((double)pool4.particles[0].lifeS, (double)savedLife, 1e-6,
               "zero-dt update does not reduce life");
}

/* -------------------------------------------------------------------------------------
 * Phase 6 chunk [6c-1] state-machine transition test.
 *
 * Drives the Game through game_fixed_update with one-shot inputs and asserts the state
 * machine transitions are correct. No physics are exercised — this tests only the
 * apply_oneshots logic and the camera initialisation.
 * ------------------------------------------------------------------------------------- */
static void scenario_state_machine(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* --- Camera zoom is set to CAMERA_BASE_ZOOM on init. --- */
    check_near((double)game->camera.zoom, (double)CAMERA_BASE_ZOOM, 1e-6,
               "camera zoom initialised to CAMERA_BASE_ZOOM");

    /* --- Test state-machine from MENU (set it explicitly for both build modes). --- */
    game->state = STATE_MENU;
    // cppcheck-suppress knownConditionTrueFalse
    check(game->state == STATE_MENU, "state can be set to STATE_MENU (got %d)",
          (int)game->state);

    /* --- MENU + pause → PLAYING (with vehicle reset). --- */
    game->vehicle.positionM.x = 100.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from MENU → PLAYING (got %d)", (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset to origin on MENU→PLAYING");

    /* --- PLAYING + pause → PAUSED. --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PAUSED, "pause from PLAYING → PAUSED (got %d)",
          (int)game->state);

    /* --- PAUSED + pause → PLAYING. --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from PAUSED → PLAYING (got %d)",
          (int)game->state);

    /* --- PLAYING + reset → PLAYING (vehicle reset). --- */
    game->vehicle.positionM.x = 150.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PLAYING stays PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PLAYING reset");

    /* --- PAUSED + reset → PLAYING (vehicle reset). --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PAUSED, "in PAUSED before reset test");
    game->vehicle.positionM.x = 50.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PAUSED → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PAUSED reset");

    /* --- RESULTS + pause → PLAYING (reset). --- */
    game->state = STATE_RESULTS;
    game->vehicle.positionM.x = 200.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from RESULTS → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset on RESULTS→PLAYING");
    /* --- RESULTS + reset → MENU. --- */
    game->state = STATE_RESULTS;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_MENU, "reset from RESULTS → MENU (got %d)", (int)game->state);

    /* --- One-shot flags are consumed (not sticky across ticks). --- */
    game->input.pausePressed = true;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(!game->input.pausePressed, "pausePressed cleared after consumption");
    check(!game->input.resetPressed, "resetPressed cleared after consumption");

    free(game);
}

/*
 * scenario_lap_average — drives a full track loop via the recorded ScriptFrame/replay path
 * (see scenario_shared.h's run_recording/run_playback, already used by core_tests.c's
 * `replay` scenario), completes several laps, and asserts per-lap time and total energy stay
 * within a tight tolerance across repeated runs of the identical script.
 *
 * Inherited from the testing-overhaul plan, Track B3 (not a fresh arXiv finding this round):
 * "the *one* end-to-end 'the simulation actually drives around a track' scenario."
 *
 * TRACK REALITY, MEASURED. track_init() builds a 200 m x 150 m parking-lot rectangle whose
 * five nodes are the four corners plus a closing duplicate of the first, with checkpoint gates
 * on the nodes and collision barriers 4 m either side of every segment. Two consequences were
 * established by probing rather than by reading:
 *
 *   - The rectangle's *interior* is not drivable as a circuit. Each segment's inner barrier
 *     spans the full side, so a car coming down the left corridor meets the bottom segment's
 *     inner barrier head-on: the corners are walled off. The drivable route is the *outer*
 *     lane, just outside the rectangle, where the corner pockets are clear of both adjacent
 *     barrier segments. The waypoint ring below is (+/-102, +/-77) for that reason.
 *   - A lap can never complete on this track. Node 4 duplicates node 0, so gate 4's forward
 *     direction is a zero-length vector and track_update_checkpoints() returns false for it
 *     forever. The car can cross gates 0 through 3 — the whole perimeter — and nextCheckpoint
 *     reaches 4, but the fifth crossing that would roll over to lap 1 is unreachable. This
 *     scenario therefore asserts full perimeter progress and does not assert a lap time; it
 *     deliberately does not assert lap == 0 either, because that would enshrine the defect.
 *
 * WHY THE SCRIPT IS GENERATED RATHER THAN HAND-TYPED. The scaffold called for a hand-crafted
 * ScriptFrame[]. A closed-loop waypoint driver produces the same artefact — a fixed array of
 * per-tick inputs — but is reproducible and re-tunable, and it is run once up front and then
 * discarded: the record and replay passes consume only the frozen array, so the determinism
 * comparison is over a fixed open-loop timeline exactly as intended.
 *
 * The route is tuned to 22 m/s on the straights, braking to 5 m/s 35 m before each corner.
 * That is not arbitrary: slower corner entries hit no barriers but overrun REPLAY_CAPACITY_TICKS
 * (7200 ticks = 60 s), and faster corner entries hit the barriers. The usable window is narrow,
 * and a route that no longer fits the ring would silently truncate the timeline's head.
 */
/* Put a freshly-initialised Game at the route's start pose, with the track loaded. Headless
 * game_init() does not call track_init(), so every pass here does it explicitly. */
static void lap_prepare_game(Game *game)
{
    game_init(game);
    track_init(&game->trackDef);
    game->state = STATE_PLAYING;
    /* The route is a fixed pedal script, so it drives the gearbox manually too. */
    game->autoTrans.enabled = false;
    game->vehicle.positionM = (Vector2){ -202.0f, -152.0f };
    set_vehicle_rolling_speed(game, 8.0f);
}

/* One pass's observable outcome, compared between the recording and every replay. */
typedef struct {
    uint32_t checksum;
    uint64_t ticks;
    int nextCheckpoint;
    int lap;
    float posX, posY;
    double energyProxy; /* sum of speed^2 per tick; proportional to kinetic energy */
} LapRun;

static void scenario_lap_average(void)
{
    /* Outer-lane waypoints: just outside the rectangle, inside each segment's 4 m barrier
     * corridor, with the corner pockets clear of both adjacent barrier segments. */
    static const Vector2 route[4] = {
        { 202.0f, -152.0f }, { 202.0f, 152.0f }, { -202.0f, 152.0f }, { -202.0f, -152.0f }
    };
    const int lapTicks = 12400; /* 103.3 s: the full perimeter, inside the 14400-tick ring */
    const int replayCount = 10;

    check(lapTicks < REPLAY_CAPACITY_TICKS,
          "lap-average: the script fits the replay ring (%d of %d ticks)", lapTicks,
          REPLAY_CAPACITY_TICKS);

    ScriptFrame *frames = (ScriptFrame *)calloc((size_t)lapTicks, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the lap script\n");
        exit(126);
    }
    {
        Game *pilot = alloc_game();
        lap_prepare_game(pilot);

        int target = 0, contactTicks = 0, reachedFinalGateAt = -1;
        float maxAbsX = 0.0f, maxAbsY = 0.0f;

        for (int i = 0; i < lapTicks; i++) {
            const Vector2 waypoint = route[target];
            const float dx = waypoint.x - pilot->vehicle.positionM.x;
            const float dy = waypoint.y - pilot->vehicle.positionM.y;
            const float distance = sqrtf(dx * dx + dy * dy);
            if (distance < 30.0f) target = (target + 1) % 4;

            const float bearing = atan2f(dy, dx);
            const float headingError = wrap_angle(bearing - pilot->vehicle.headingRad);
            const float wantedMps = (distance < 50.0f) ? 12.0f : 18.0f;
            const float speedError = wantedMps - pilot->derived.speedMps;

            pilot->input.steer = clampf(headingError * 3.0f, -1.0f, 1.0f);
            pilot->input.throttle = clampf(speedError * 0.35f, 0.0f, 1.0f);
            pilot->input.brake = clampf(-speedError * 0.30f, 0.0f, 0.6f);
            pilot->input.handbrake = 0.0f;

            frames[i].steer = pilot->input.steer;
            frames[i].throttle = pilot->input.throttle;
            frames[i].brake = pilot->input.brake;
            frames[i].handbrake = 0.0f;
            frames[i].frameTimeS = FIXED_DT_S;
            game_fixed_update(pilot, FIXED_DT_S);

            if (pilot->crashLockoutTimerS > 0.0f) contactTicks++;
            if (reachedFinalGateAt < 0 && pilot->progress.nextCheckpoint >= 1)
                reachedFinalGateAt = i;
            maxAbsX = fmaxf(maxAbsX, fabsf(pilot->vehicle.positionM.x));
            maxAbsY = fmaxf(maxAbsY, fabsf(pilot->vehicle.positionM.y));
        }

        check(pilot->progress.nextCheckpoint >= 1,
              "lap-average: the script drives across checkpoints (reached gate %d of %d)",
              pilot->progress.nextCheckpoint, pilot->trackDef.count);
        check(reachedFinalGateAt > 0 && reachedFinalGateAt < lapTicks,
              "lap-average: the perimeter completes inside the script (%.1f s of %.1f s)",
              (double)reachedFinalGateAt / (double)FIXED_HZ,
              (double)lapTicks / (double)FIXED_HZ);
        check(contactTicks == 0,
              "lap-average: the route clears every barrier (%d ticks in crash lockout)",
              contactTicks);
        check(maxAbsX < 210.0f && maxAbsY < 160.0f,
              "lap-average: the car stays in the outer lane (max |x| %.1f m, |y| %.1f m)",
              (double)maxAbsX, (double)maxAbsY);
        track_free(&pilot->trackDef);
        free(pilot);
    }

    /* ---- 2. Record the frozen script once. ---- */
    LapRun recorded;
    memset(&recorded, 0, sizeof(recorded));
    ReplayBuffer *timeline = (ReplayBuffer *)calloc(1, sizeof(ReplayBuffer));
    if (timeline == NULL) {
        fprintf(stderr, "FATAL: could not allocate a ReplayBuffer\n");
        exit(126);
    }

    {
        Game *game = alloc_game();
        lap_prepare_game(game);
        replay_begin_recording(&game->replay, game->sim.tick);

        for (int i = 0; i < lapTicks; i++) {
            game->input.steer = frames[i].steer;
            game->input.throttle = frames[i].throttle;
            game->input.brake = frames[i].brake;
            game->input.handbrake = frames[i].handbrake;
            game_fixed_update(game, FIXED_DT_S);
            recorded.energyProxy +=
                (double)game->derived.speedMps * (double)game->derived.speedMps;
        }

        recorded.checksum = game->stateChecksum;
        recorded.ticks = game->sim.tick;
        recorded.nextCheckpoint = game->progress.nextCheckpoint;
        recorded.lap = game->progress.lap;
        recorded.posX = game->vehicle.positionM.x;
        recorded.posY = game->vehicle.positionM.y;
        *timeline = game->replay;

        check(game->replay.count == lapTicks,
              "lap-average: one timeline entry per tick (%d of %d)", game->replay.count,
              lapTicks);
        check(game->replay.overwrittenTicks == 0u,
              "lap-average: the ring never overwrote the head (%llu overwritten)",
              (unsigned long long)game->replay.overwrittenTicks);

        track_free(&game->trackDef);
        free(game);
    }

    /* ---- 3. Replay the recorded timeline ten times and compare everything. ---- */
    int matchingChecksums = 0, matchingCheckpoints = 0, matchingEnergy = 0, matchingPose = 0;

    for (int r = 0; r < replayCount; r++) {
        Game *game = alloc_game();
        lap_prepare_game(game);

        game->replay = *timeline;
        if (!replay_begin_playback(&game->replay)) {
            track_free(&game->trackDef);
            free(game);
            continue;
        }

        LapRun run;
        memset(&run, 0, sizeof(run));
        for (int i = 0; i < lapTicks; i++) {
            input_zero(&game->input);
            game_fixed_update(game, FIXED_DT_S);
            run.energyProxy += (double)game->derived.speedMps * (double)game->derived.speedMps;
        }

        if (game->stateChecksum == recorded.checksum && game->sim.tick == recorded.ticks)
            matchingChecksums++;
        if (game->progress.nextCheckpoint == recorded.nextCheckpoint &&
            game->progress.lap == recorded.lap)
            matchingCheckpoints++;
        if (run.energyProxy == recorded.energyProxy) matchingEnergy++;
        if (game->vehicle.positionM.x == recorded.posX &&
            game->vehicle.positionM.y == recorded.posY)
            matchingPose++;

        track_free(&game->trackDef);
        free(game);
    }

    check(matchingChecksums == replayCount,
          "lap-average: all %d replays reproduce the recorded checksum %08x (%d matched)",
          replayCount, recorded.checksum, matchingChecksums);
    check(matchingCheckpoints == replayCount,
          "lap-average: all %d replays reproduce the checkpoint and lap state "
          "(gate %d, lap %d; %d matched)",
          replayCount, recorded.nextCheckpoint, recorded.lap, matchingCheckpoints);
    check(matchingEnergy == replayCount,
          "lap-average: all %d replays reproduce the summed energy proxy exactly "
          "(%.6f; %d matched)",
          replayCount, recorded.energyProxy, matchingEnergy);
    check(matchingPose == replayCount,
          "lap-average: all %d replays finish at the recorded position (%.4f, %.4f; "
          "%d matched)",
          replayCount, (double)recorded.posX, (double)recorded.posY, matchingPose);

    free(timeline);
    free(frames);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: car-selection — issue #32 model: the player-selectable roster as a           */
/* deterministic, id-sorted selection view, with stable resolve and recall persistence    */
/* ------------------------------------------------------------------------------------- */
static void scenario_car_selection(void)
{
    /* The selection view is exactly the six roster cars, in stable id-sorted order. */
    check(car_selection_count() == 6, "car selection offers exactly the 6 roster cars (got %d)",
          car_selection_count());

    static const char *const kExpectedOrder[6] = { "awd_gt",    "awd_rally", "fwd_hot",
                                                   "fwd_light", "rwd_grip",  "rwd_power" };
    for (int i = 0; i < car_selection_count(); i++) {
        CarSelectionEntry entry;
        memset(&entry, 0, sizeof(entry));
        check(car_selection_entry(i, &entry), "selection entry %d is addressable", i);
        check(strcmp(entry.id, kExpectedOrder[i]) == 0,
              "selection entry %d id is '%s' (got '%s')", i, kExpectedOrder[i], entry.id);
        check(entry.manifest != NULL, "selection entry %d carries its manifest", i);
        if (entry.manifest != NULL) {
            check(strcmp(entry.manifest->definition.id, entry.id) == 0,
                  "selection entry %d manifest id matches its entry id", i);
        }
    }

    /* Resolution is by stable id, never by raw selection slot. */
    check(car_selection_resolve("rwd_grip") == 4,
          "rwd_grip resolves to its id-sorted selection index 4 (got %d)",
          car_selection_resolve("rwd_grip"));
    check(car_selection_resolve("gone") == -1, "an unknown id resolves to -1 (got %d)",
          car_selection_resolve("gone"));
    check(car_selection_index_or_default("gone") == 0,
          "an unknown id falls back to selection index 0");
    check(car_selection_index_or_default(NULL) == 0, "NULL falls back to selection index 0");

    /*
     * The recall path is real player state: the menu opens on whatever car it names, and the
     * game rewrites it on every selection. Exercising the round-trip therefore has to borrow
     * the file rather than own it — the developer's own last car is read out first and put back
     * at the end, including the case where there was no file to begin with. Without that, a
     * test run silently changes which car the game starts on, and an interrupted run leaves a
     * stale 'awd_gt' behind.
     */
    char preserved[CAR_SELECTION_ID_CHARS];
    memset(preserved, 0, sizeof(preserved));
    const bool hadRecall = car_selection_load_recall(preserved, sizeof(preserved));

    /* Round-trip persistence: save a valid id, read the same id back. */
    check(car_selection_save_recall("awd_gt"), "recall saved 'awd_gt'");
    char recalled[CAR_SELECTION_ID_CHARS];
    memset(recalled, 0, sizeof(recalled));
    check(car_selection_load_recall(recalled, sizeof(recalled)), "recall loads the saved file");
    check(strcmp(recalled, "awd_gt") == 0, "recall round-trips 'awd_gt' (got '%s')", recalled);

    /* A missing recall file is a clean 'no stored choice', not an error. */
    check(remove(CAR_SELECTION_RECALL_PATH) == 0, "recall file removed for the absent case");
    memset(recalled, 0, sizeof(recalled));
    check(!car_selection_load_recall(recalled, sizeof(recalled)),
          "loading with no recall file returns false");

    /* Hand the developer's selection back exactly as it was found. */
    if (hadRecall) {
        check(car_selection_save_recall(preserved), "pre-existing recall '%s' restored",
              preserved);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: setup-editor — issue #33 model: bounded, validated setup edits on a private  */
/* working copy, with reset and independent-editor isolation                              */
/* ------------------------------------------------------------------------------------- */
static void scenario_setup_editor(void)
{
    const VehicleManifest *manifest = car_roster_manifest(0);
    check(manifest != NULL, "roster index 0 has a manifest");
    if (manifest == NULL) return;

    SetupEditor ed;
    setup_editor_init(&ed, &manifest->definition, &manifest->defaultSetup);

    /* Every setup-owned physics-input registry key (drive.gear1..gear5, drive.reverse,
     * drive.final, drive.diff_mode, drive.diff_bias_ratio, drive.diff_preload,
     * brake.bias_front, and — since issue #14 activated static alignment — susp.toe_front and
     * susp.toe_rear) plus the typed int drive.gear_count. */
    check(ed.itemCount == 14,
          "editor exposes 13 registry setup-physics keys + gear_count (got %d)", ed.itemCount);
    check(setup_editor_is_valid(&ed), "baseline setup is valid");
    const uint32_t h0 = setup_editor_hash(&ed);
    const float baseValue0 = setup_editor_value(&ed, 0);

    /* A step edit lands inside the item's registry bounds and moves the hash. */
    check(setup_editor_adjust(&ed, 0, 1), "item 0 accepts a +1 step");
    const float stepped0 = setup_editor_value(&ed, 0);
    check(stepped0 != baseValue0, "item 0 value changed by the step (%.4f -> %.4f)",
          (double)baseValue0, (double)stepped0);
    check(stepped0 >= ed.items[0].min && stepped0 <= ed.items[0].max,
          "item 0 value stays inside [%.4f, %.4f] (got %.4f)", (double)ed.items[0].min,
          (double)ed.items[0].max, (double)stepped0);
    check(setup_editor_is_valid(&ed), "the edited setup is still valid");
    check(setup_editor_hash(&ed) != h0, "the edit changed the setup hash");

    /* Reset restores the exact baseline. */
    setup_editor_reset(&ed);
    check(setup_editor_hash(&ed) == h0, "reset restores the baseline hash");
    check(setup_editor_value(&ed, 0) == baseValue0, "reset restores item 0's baseline value");

    /* Two editors over the same definition share no mutable state. */
    SetupEditor ed1, ed2;
    setup_editor_init(&ed1, &manifest->definition, &manifest->defaultSetup);
    setup_editor_init(&ed2, &manifest->definition, &manifest->defaultSetup);
    check(setup_editor_hash(&ed2) == h0, "a second editor starts at the same baseline");
    check(setup_editor_adjust(&ed1, 0, 1), "the first editor accepts a step");
    check(setup_editor_hash(&ed1) != h0, "the first editor moved");
    check(setup_editor_hash(&ed2) == h0,
          "the second editor is untouched: no shared-state mutation");

    /* Repeated upward steps clamp at the item maximum. */
    int guard = 0;
    float value = setup_editor_value(&ed, 0);
    while (guard < 1000) {
        setup_editor_adjust(&ed, 0, 1);
        const float next = setup_editor_value(&ed, 0);
        guard++;
        if (next == value) break;
        value = next;
    }
    check(guard < 1000, "clamp reached within the iteration cap (%d steps)", guard);
    check(setup_editor_value(&ed, 0) == ed.items[0].max,
          "repeated +1 steps clamp item 0 at its max %.4f (got %.4f, %d steps)",
          (double)ed.items[0].max, (double)setup_editor_value(&ed, 0), guard);

    /*
     * The registry sweep and the key -> VehicleSetup binding table must agree. A registry key
     * with no binding would render as an item, display 0.0, and do nothing when adjusted; the
     * editor refuses to create such an item and counts it here instead, so this assertion is
     * what catches the two lists drifting apart.
     */
    check(ed.unboundKeyCount == 0,
          "every setup-owned physics-input registry key has a field binding (got %d unbound)",
          ed.unboundKeyCount);

    /*
     * gear_count may never exceed the forward ratios the editor exposes. Raising it past the
     * last editable ratio would commit the setup to a gear whose ratio the menu cannot reach —
     * usually 0, which vehicle_setup_is_valid rejects — leaving a setup the editor broke and
     * cannot repair. Drive the count all the way up and require the result to still be valid.
     */
    int gearItem = -1;
    for (int i = 0; i < ed.itemCount; i++) {
        if (ed.items[i].isGearCount) gearItem = i;
    }
    check(gearItem >= 0, "the editor exposes a gear_count item");
    if (gearItem >= 0) {
        setup_editor_reset(&ed);
        int editableRatios = 0;
        for (int i = 0; i < ed.itemCount; i++) {
            if (strncmp(ed.items[i].key, "drive.gear", 10) == 0 && !ed.items[i].isGearCount)
                editableRatios++;
        }
        check((int)ed.items[gearItem].max == editableRatios,
              "gear_count tops out at the %d editable ratios, not MAX_GEARS=%d (max %d)",
              editableRatios, MAX_GEARS, (int)ed.items[gearItem].max);
        for (int step = 0; step < MAX_GEARS + 4; step++) setup_editor_adjust(&ed, gearItem, 1);
        check(setup_editor_value(&ed, gearItem) <= (float)editableRatios,
              "repeated +1 steps cannot raise gear_count past %d (got %.0f)", editableRatios,
              (double)setup_editor_value(&ed, gearItem));
        check(setup_editor_is_valid(&ed),
              "a fully raised gear_count still produces a launchable setup");
    }

    /*
     * Save/load round-trip. The saved file is the deterministic serialization a replay or a
     * stored profile would carry, so what saves cleanly must load cleanly — in particular the
     * gear count, whose editor ceiling exists to stop the *menu* creating an unrepairable
     * setup and is deliberately not applied on load. Applying it there would make a setup
     * authored with six to eight gears save and then fail to come back.
     */
    {
        char path[640];
        telemetry_ensure_dir(TELEMETRY_DIR);
        snprintf(path, sizeof(path), "%s/_setup_editor_roundtrip.txt", TELEMETRY_DIR);

        SetupEditor saved;
        setup_editor_init(&saved, &manifest->definition, &manifest->defaultSetup);
        check(setup_editor_adjust(&saved, 0, 1), "round-trip source takes one step");
        const uint32_t savedHash = setup_editor_hash(&saved);

        char error[256] = "";
        check(setup_editor_save(&saved, path, error, sizeof(error)),
              "setup saves to disk (error: %s)", error);

        SetupEditor loaded;
        setup_editor_init(&loaded, &manifest->definition, &manifest->defaultSetup);
        check(setup_editor_load(&loaded, path, error, sizeof(error)),
              "setup loads back (error: %s)", error);
        check(setup_editor_hash(&loaded) == savedHash,
              "the loaded setup reproduces the saved hash (%08x vs %08x)",
              setup_editor_hash(&loaded), savedHash);

        /*
         * A setup authored with more forward gears than the editor exposes must survive the
         * round-trip. The five-ratio ceiling is a menu-authoring rule — it stops the player
         * creating a count with no ratio control to repair it — and not a claim that six- to
         * eight-gear cars are invalid. Applying it on load would make such a setup save
         * cleanly and then fail to come back.
         *
         * The baseline is authored here with real ratios for every gear, so the setup is
         * genuinely valid (vehicle_spec_is_valid requires ratios[0..gearCount-1] finite and
         * positive) and the assertions below turn on the ceiling rather than on the setup
         * being rejected for an unrelated reason.
         */
        if (gearItem >= 0 && MAX_GEARS > (int)ed.items[gearItem].max) {
            VehicleSetup wideSetup = manifest->defaultSetup;
            wideSetup.gearCount = MAX_GEARS;
            for (int g = 0; g < MAX_GEARS; g++) {
                wideSetup.gearRatios[g] = 3.4f - 0.3f * (float)g; /* descending, all positive */
            }
            check(vehicle_setup_is_valid(&manifest->definition, &wideSetup),
                  "an authored %d-gear setup is valid", MAX_GEARS);

            SetupEditor wide;
            setup_editor_init(&wide, &manifest->definition, &wideSetup);
            check(setup_editor_is_valid(&wide), "the editor accepts a %d-gear baseline",
                  MAX_GEARS);
            check(wide.working.gearCount == MAX_GEARS,
                  "the %d-gear count survives editor init (got %d)", MAX_GEARS,
                  wide.working.gearCount);

            /* Raising must not silently lower an authored count to the editor's ceiling. */
            check(setup_editor_adjust(&wide, gearItem, 1),
                  "the gear-count item accepts a step");
            check(wide.working.gearCount == MAX_GEARS,
                  "a +1 step leaves an already-high authored count alone (got %d)",
                  wide.working.gearCount);

            const uint32_t wideHash = setup_editor_hash(&wide);
            check(setup_editor_save(&wide, path, error, sizeof(error)),
                  "the %d-gear setup saves (error: %s)", MAX_GEARS, error);
            SetupEditor wideBack;
            setup_editor_init(&wideBack, &manifest->definition, &wideSetup);
            check(setup_editor_load(&wideBack, path, error, sizeof(error)),
                  "the %d-gear setup loads back — the UI ceiling is not applied on load "
                  "(error: %s)",
                  MAX_GEARS, error);
            check(wideBack.working.gearCount == MAX_GEARS,
                  "the loaded gear count is still %d (got %d)", MAX_GEARS,
                  wideBack.working.gearCount);
            check(setup_editor_hash(&wideBack) == wideHash,
                  "the %d-gear round-trip reproduces its hash", MAX_GEARS);
        }
        /*
         * Malformed input is rejected rather than tolerated. A `setup.hash` line that does not
         * parse used to leave verification silently disabled, which meant the one line whose
         * job is to detect corruption was switched off by exactly the corruption it guards
         * against. An out-of-range gear count is rejected without first narrowing the value —
         * lrint() of 1e300 and the int conversion that followed were undefined behaviour on
         * the way to the rejection.
         */
        static const struct {
            const char *body;
            const char *what;
        } kMalformed[] = {
            { "setup.hash=notahash\n", "non-hex setup.hash" },
            { "setup.hash=1234\n", "short setup.hash" },
            { "setup.hash=0123456789\n", "over-long setup.hash" },
            { "drive.gear_count=1e300\n", "gear_count far outside int range" },
            { "drive.gear_count=-1e300\n", "negative gear_count outside int range" },
            { "drive.gear_count=2.5\n", "non-integral gear_count" },
            { "drive.gear_count=0\n", "gear_count below 1" },
        };
        for (size_t k = 0; k < sizeof(kMalformed) / sizeof(kMalformed[0]); k++) {
            FILE *bad = fopen(path, "wb");
            if (bad != NULL) {
                fputs(kMalformed[k].body, bad);
                fclose(bad);
            }
            SetupEditor victim;
            setup_editor_init(&victim, &manifest->definition, &manifest->defaultSetup);
            const uint32_t before = setup_editor_hash(&victim);
            error[0] = '\0';
            check(!setup_editor_load(&victim, path, error, sizeof(error)),
                  "%s is rejected (error: %s)", kMalformed[k].what, error);
            check(setup_editor_hash(&victim) == before, "%s leaves the working setup untouched",
                  kMalformed[k].what);
        }
        remove(path);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: roster-gate — the promotion checklist and class rules are enforced when the  */
/* live roster is built, not only in tests (issues #31, #33)                              */
/* ------------------------------------------------------------------------------------- */
static void scenario_roster_gate(void)
{
    /*
     * The shipped content must pass the gates it declares. This is the regression guard that
     * makes the gate meaningful in both directions: if a manifest is ever committed that calls
     * itself player-selectable but fails a promotion check or its own class's numeric rules,
     * the roster silently shrinks — and this scenario names the car and the reason instead of
     * leaving a mystery missing entry.
     */
    const int rejected = car_roster_rejection_count();
    for (int i = 0; i < rejected; i++) {
        const CarRosterRejection *r = car_roster_rejection(i);
        check(false, "shipped manifest '%s' was refused by the roster gate: %s",
              r != NULL ? r->id : "?", r != NULL ? r->reason : "?");
    }
    /* The total is the honest count; the stored-diagnostic list above is capped. Asserting the
     * total is what makes "nothing was refused" trustworthy even past the detail cap. */
    check(car_roster_refused_count() == 0,
          "no shipped player-selectable manifest fails the roster gate (got %d)",
          car_roster_refused_count());
    check(car_roster_count() == 6, "all 6 shipped cars survive both gates (got %d)",
          car_roster_count());

    /* Every surviving car satisfies the checklist directly — the roster's filter and the
     * checklist agree rather than the roster merely claiming they do. */
    for (int i = 0; i < car_roster_count(); i++) {
        const VehicleManifest *m = car_roster_manifest(i);
        check(m != NULL, "roster car %d has a manifest", i);
        if (m == NULL) continue;
        VehiclePromotionReport report;
        check(vehicle_promotion_evaluate(m, &report),
              "roster car '%s' passes the promotion checklist", m->definition.id);
        check(m->contentKind == VEHICLE_CONTENT_PLAYER_SELECTABLE,
              "roster car '%s' is player-selectable", m->definition.id);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: launch-per-drivetrain — every roster car spawns through the same entrant     */
/* path and the spawned local entrant carries the selected car's stable id                */
/* ------------------------------------------------------------------------------------- */
static void scenario_launch_per_drivetrain(void)
{
    const int count = car_roster_count();
    check(count == 6, "roster has 6 cars to launch (got %d)", count);

    for (int i = 0; i < count; i++) {
        const VehicleManifest *manifest = car_roster_manifest(i);
        check(manifest != NULL, "roster car %d has a manifest", i);
        if (manifest == NULL) continue;

        /* A fresh session per car: race_session_init gives the roster its empty valid
         * state (game_init would already spawn the default local entrant, which would
         * refuse a second localPlayer designation). */
        Game *game = alloc_game();
        race_session_init(&game->session);
        const RaceEntrantSpawn spawn = { .definition = &manifest->definition,
                                         .setup = &manifest->defaultSetup,
                                         .controllerKind = CONTROLLER_KIND_HUMAN,
                                         .localPlayer = true,
                                         .gridSlot = 0 };
        check(race_roster_spawn(&game->session.roster, &spawn, NULL),
              "roster car %d ('%s') spawns through the entrant path", i,
              manifest->definition.id);
        const RaceEntrant *local = race_roster_local(&game->session.roster);
        check(local != NULL, "roster car %d ('%s') is the local entrant", i,
              manifest->definition.id);
        if (local != NULL) {
            check(strcmp(local->definition.id, manifest->definition.id) == 0,
                  "the spawned entrant carries the selected id '%s' (got '%s')",
                  manifest->definition.id, local->definition.id);
        }
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: failure-classification — every #78 failure class, proven by construction       */
/* ------------------------------------------------------------------------------------- */

/* Fill a telemetry row with the fields the classifier reads; everything else stays zero. */
static void cls_fill_row(TelemetryRow *r, int tick, float speedMps, float sideslipRad,
                         int checkpointEvent, int lastCrossed, int expected, int beyondRunoff,
                         int wrongWay, int routeSegment, int aiSegment, float furthestM,
                         float lockoutS, int lapIndex)
{
    memset(r, 0, sizeof(*r));
    r->tick = (uint64_t)tick;
    r->timeS = (double)tick * FIXED_DT_S;
    r->positionXM = 10.0f;
    r->positionYM = 10.0f;
    r->speedMps = speedMps;
    r->bodySideslipRad = sideslipRad;
    r->checkpointEvent = checkpointEvent;
    r->lastCrossedIndex = lastCrossed;
    r->checkpointIndex = expected;
    /* Hand-built rows predate the checkpoint_crossed_index telemetry column; -1 makes the
     * classifier fall back to lastCrossedIndex exactly as it did before (a zero here would be
     * read as "gate 0 was the crossing"). */
    r->checkpointCrossedIndex = -1;
    r->beyondRunoff = beyondRunoff;
    r->wrongWayFlag = wrongWay;
    r->routeSegmentIndex = routeSegment;
    r->aiSegment = aiSegment;
    r->furthestProgressM = furthestM;
    r->crashLockoutS = lockoutS;
    r->lapIndex = lapIndex;
}

/* The same inputs the validation runner feeds the classifier: the threshold defaults come from
 * the shared function, so a retune in validation_classifier.c cannot leave this scenario
 * quoting stale numbers; only the per-run fields are local (PR #80 review). */
static void cls_default_inputs(ClassificationInputs *in)
{
    validation_classification_inputs_default(in);
    in->checkpointCount = 25;
    in->startCheckpointIndex = 0;
    in->targetLaps = 4;
    in->tickBudget = 36000;
    in->ticksRun = 36000; /* budget expired unless a test overrides */
    in->fixedDtS = FIXED_DT_S;
}

/* Rows are sampled every 2nd tick (60 Hz), like the validation runner. */
static int cls_tick(int i)
{
    return 2 * i;
}

static void scenario_failure_classification(void)
{
    ValidationClassification cls;
    ClassificationInputs in;

    /* 0. Degenerate inputs reduce to the documented no-run state: PASS with the no-crossing
     * sentinels and no fault evidence. The ai-roster-laps false-pass path exists precisely
     * because this contract was untested (PR #80 review). */
    {
        ValidationClassification z;
        cls_default_inputs(&in);
        validation_classify(NULL, 0, NULL, &in, &z);
        check(z.primary == RUN_CLASS_PASS && z.lastCheckpointIndex == -1 &&
                  z.expectedCheckpointIndex == -1 && z.contributingCount == 0 &&
                  z.firstFaultTick == 0,
              "NULL rows reduce to the no-run PASS state");
        TelemetryRow one;
        cls_fill_row(&one, 0, 10.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 10.0f, 0.0f, 0);
        validation_classify(&one, 1, NULL, NULL, &z);
        check(z.primary == RUN_CLASS_PASS && z.contributingCount == 0,
              "NULL inputs reduce to the no-run PASS state");
    }

    /* 1. invalid_physics dominates every other verdict and names the bad row. */
    {
        TelemetryRow rows[3];
        for (int i = 0; i < 3; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 1, i, i + 1, 0, 0, 5, 5,
                         10.0f * (float)(i + 1), 0.0f, 0);
        }
        rows[1].positionXM = NAN;
        cls_default_inputs(&in);
        validation_classify(rows, 3, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_INVALID_PHYSICS, "invalid_physics dominates (got %s)",
              failure_class_reason(cls.primary));
        check(cls.firstFaultTick == (uint64_t)cls_tick(1),
              "invalid_physics first-fault tick is the bad row (%llu)",
              (unsigned long long)cls.firstFaultTick);
    }

    /* 2/3. An out-of-order crossing is checkpoint_out_of_order when it is BEHIND the owed gate
     * and checkpoint_skipped when it jumps AHEAD of it. The rows carry the event's own crossed
     * index (checkpointCrossedIndex), exactly as the runner's telemetry does. */
    {
        TelemetryRow rows[3];
        for (int i = 0; i < 3; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 2, 2, 5, 0, 0, 5, 5,
                         10.0f * (float)(i + 1), 0.0f, 0);
            rows[i].checkpointCrossedIndex = 2;
        }
        cls_default_inputs(&in);
        validation_classify(rows, 3, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_CHECKPOINT_OUT_OF_ORDER,
              "gate 2 while owing gate 5 is out_of_order (got %s)",
              failure_class_reason(cls.primary));
    }
    {
        TelemetryRow rows[3];
        for (int i = 0; i < 3; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 2, 7, 5, 0, 0, 5, 5,
                         10.0f * (float)(i + 1), 0.0f, 0);
            rows[i].checkpointCrossedIndex = 7;
        }
        cls_default_inputs(&in);
        validation_classify(rows, 3, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_CHECKPOINT_SKIPPED,
              "gate 7 while owing gate 5 is a forward skip (got %s)",
              failure_class_reason(cls.primary));
        check(cls.contributingCount >= 1 &&
                  cls.contributing[0].reason == RUN_CLASS_CHECKPOINT_SKIPPED,
              "the skip is the first contributing event");
    }

    /* 3b. The skip/out-of-order split sits at forward == checkpointCount / 2: crossing 12
     * while owing 0 (forward 12 == 25/2) is a skip; crossing 13 (forward 13) is out of order.
     * An off-by-one there is exactly the kind of change these tests should catch (PR #80
     * review). */
    {
        const struct {
            int crossed;
            FailureClass want;
        } edge[] = {
            { 12, RUN_CLASS_CHECKPOINT_SKIPPED },
            { 13, RUN_CLASS_CHECKPOINT_OUT_OF_ORDER },
        };
        for (int k = 0; k < 2; k++) {
            TelemetryRow rows[3];
            for (int i = 0; i < 3; i++) {
                cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 2, edge[k].crossed, 0, 0, 0, 5,
                             5, 10.0f * (float)(i + 1), 0.0f, 0);
                rows[i].checkpointCrossedIndex = edge[k].crossed;
            }
            cls_default_inputs(&in);
            validation_classify(rows, 3, NULL, &in, &cls);
            check(cls.primary == edge[k].want, "crossed %d owing 0 classifies as %s (got %s)",
                  edge[k].crossed, failure_class_reason(edge[k].want),
                  failure_class_reason(cls.primary));
        }
    }

    /* 4. slow_timeout: budget expired while the car is still making progress — distinct from a
     * stall and from a missed gate. */
    {
        TelemetryRow rows[30];
        for (int i = 0; i < 30; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 1, 1, 2, 0, 0, 5, 5,
                         10.0f * (float)(i + 1), 0.0f, 0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 30, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_SLOW_TIMEOUT,
              "budget expired while progressing is slow_timeout (got %s)",
              failure_class_reason(cls.primary));
        check(cls.contributingCount == 1 &&
                  cls.contributing[0].reason == RUN_CLASS_SLOW_TIMEOUT,
              "slow_timeout is the only contributing event");
    }

    /* 4b. The catch-all. The contrast with slow_timeout above: the budget expired while the car
     * was still MOVING but had long since stopped PROGRESSING, so no stall fires (speed is over
     * the threshold), no departure/spin/wrong-way fires, and slow_timeout's progress-recency
     * test rejects it. Nothing in the closed set describes it. That must report as a failure,
     * not as "pass" beside a FAIL status — the awd_rally/technical case (#79). */
    {
        TelemetryRow rows[400]; /* 400 rows at 60 Hz = 6.65 s */
        for (int i = 0; i < 400; i++) {
            /* Progress advances for the first 100 rows, then stops dead while the car keeps
             * rolling at 10 m/s: moving, but going nowhere. */
            const float furthestM = (i < 100) ? 10.0f * (float)(i + 1) : 1000.0f;
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 0, 5, 6, 0, 0, 5, 5, furthestM,
                         0.0f, 1);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 400, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_UNEXPLAINED,
              "an incomplete run matching no class is unexplained, never pass (got %s)",
              failure_class_reason(cls.primary));
        check(strcmp(failure_class_reason(cls.primary), "unexplained") == 0,
              "the catch-all's token is 'unexplained'");
        check(
            cls.contributingCount == 1 && cls.contributing[0].reason == RUN_CLASS_UNEXPLAINED,
            "unexplained is its own sole contributing event, so primary is always one of them");
        check(
            cls.firstFaultTick == cls.lastProgressTick,
            "unexplained anchors the fault at the last tick that made progress (%llu vs %llu)",
            (unsigned long long)cls.firstFaultTick, (unsigned long long)cls.lastProgressTick);

        /* The same rows on a run that DID complete its laps stay a pass: the catch-all must not
         * turn a clean finish into a failure. */
        for (int i = 0; i < 400; i++) rows[i].lapIndex = in.targetLaps;
        validation_classify(rows, 400, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_PASS && cls.contributingCount == 0,
              "a completed run with no detected class is still a pass (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 5/6. A sustained stop is stalled_on_track on the surface and stalled_off_track beyond it. */
    {
        TelemetryRow rows[200]; /* 200 rows at 60 Hz = 3.33 s > 3.0 s */
        for (int i = 0; i < 200; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 0.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 200, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_STALLED_ON_TRACK,
              "stopped on the surface is stalled_on_track (got %s)",
              failure_class_reason(cls.primary));
    }
    {
        TelemetryRow rows[200];
        for (int i = 0; i < 200; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 0.0f, 0.0f, 0, -1, 1, 1, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 200, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_STALLED_OFF_TRACK,
              "stopped beyond the runoff is stalled_off_track (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 7. collision_stuck: a contact at meaningful speed precedes the immobility; the first-fault
     * tick is the CONTACT, not the stop. The rising edge needs a row before it with no lockout
     * (the detector watches prevLockout -> lockout), so the contact sits mid-run. */
    {
        TelemetryRow rows[205];
        for (int i = 0; i < 4; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 5.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_fill_row(&rows[4], cls_tick(4), 5.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 1.0f, 0);
        for (int i = 5; i < 205; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 0.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 1.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 205, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_COLLISION_STUCK,
              "collision then immobility is collision_stuck (got %s)",
              failure_class_reason(cls.primary));
        check(cls.firstFaultTick == (uint64_t)cls_tick(4),
              "collision_stuck first-fault is the contact tick (%llu)",
              (unsigned long long)cls.firstFaultTick);
    }

    /* 8. spin_then_departure: a sustained spin precedes the route departure, and the first-fault
     * tick is the SPIN onset. */
    {
        TelemetryRow rows[80];
        for (int i = 0; i < 20; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 5.0f, 1.5f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        for (int i = 20; i < 80; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 3.0f, 0.0f, 0, -1, 1, 1, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 80, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_SPIN_THEN_DEPARTURE,
              "spin then departure is spin_then_departure (got %s)",
              failure_class_reason(cls.primary));
        check(cls.firstFaultTick == (uint64_t)cls_tick(0),
              "spin_then_departure first-fault is the spin onset (%llu)",
              (unsigned long long)cls.firstFaultTick);
    }

    /* 9. wrong_way: the latched flag sustained past the hold time. */
    {
        TelemetryRow rows[100]; /* 1.67 s > 1.5 s */
        for (int i = 0; i < 100; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 8.0f, 0.0f, 0, -1, 1, 0, 1, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 100, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_WRONG_WAY, "sustained wrong-way is wrong_way (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 10. route_departure: beyond the runoff without a preceding spin. */
    {
        TelemetryRow rows[60]; /* 1.0 s > 0.75 s */
        for (int i = 0; i < 60; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 5.0f, 0.0f, 0, -1, 1, 1, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 60, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_ROUTE_DEPARTURE,
              "beyond-runoff without a spin is route_departure (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 11. localization_lost: an invalid route segment sustained. */
    {
        TelemetryRow rows[60];
        for (int i = 0; i < 60; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 5.0f, 0.0f, 0, -1, 1, 0, 0, -1, -1, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        validation_classify(rows, 60, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_LOCALIZATION_LOST,
              "invalid localization sustained is localization_lost (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 12. planner_localization_mismatch: the AI and route segments disagree sustained. The
     * fixture marks the rows AI-driven (aiPresent = 1) — without the marker the classifier
     * must not compare a zero-filled aiSegment against the route (PR #80 review). */
    {
        TelemetryRow rows[70]; /* 1.17 s > 1.0 s */
        for (int i = 0; i < 70; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 1, 1, 2, 0, 0, 7, 3,
                         10.0f * (float)(i + 1), 0.0f, 0);
            rows[i].aiPresent = 1;
        }
        cls_default_inputs(&in);
        validation_classify(rows, 70, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH,
              "sustained AI/route disagreement is planner_localization_mismatch (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 12b. Non-AI rows never produce planner_localization_mismatch: aiPresent stays 0, so the
     * zero-filled aiSegment must not be read as an AI decision (PR #80 review).
     *
     * These rows never complete their laps, so the verdict is the catch-all rather than a pass
     * — this case previously asserted RUN_CLASS_PASS, which was the "FAIL with reason pass"
     * fallthrough itself (#79). What it is really testing is unchanged and asserted directly
     * below: the mismatch reducer stays silent without aiPresent. */
    {
        TelemetryRow rows[70]; /* 1.17 s > 1.0 s; long enough to exceed the mismatch hold */
        for (int i = 0; i < 70; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 1, 1, 2, 0, 0, 7, 3,
                         10.0f * (float)(i + 1), 0.0f, 0);
        }
        cls_default_inputs(&in);
        in.ticksRun = 0; /* the budget did not expire */
        validation_classify(rows, 70, NULL, &in, &cls);
        check(cls.primary != RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH,
              "non-AI rows are not a planner mismatch (got %s)",
              failure_class_reason(cls.primary));
        for (int i = 0; i < cls.contributingCount; i++) {
            check(cls.contributing[i].reason != RUN_CLASS_PLANNER_LOCALIZATION_MISMATCH,
                  "non-AI rows never contribute a planner mismatch");
        }
        check(cls.primary == RUN_CLASS_UNEXPLAINED,
              "an incomplete run with no detected class is the catch-all (got %s)",
              failure_class_reason(cls.primary));
    }

    /* 13. A run that COMPLETED its target laps is a pass despite a transient disagreement. */
    {
        TelemetryRow rows[70];
        for (int i = 0; i < 69; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 10.0f, 0.0f, 1, 1, 2, 0, 0, 7, 3,
                         10.0f * (float)(i + 1), 0.0f, 0);
            rows[i].aiPresent = 1;
        }
        cls_fill_row(&rows[69], cls_tick(69), 10.0f, 0.0f, 1, 1, 2, 0, 0, 7, 3, 700.0f, 0.0f,
                     4);
        rows[69].aiPresent = 1;
        cls_default_inputs(&in);
        validation_classify(rows, 70, NULL, &in, &cls);
        check(cls.primary == RUN_CLASS_PASS,
              "a completed run is pass despite a transient mismatch (got %s)",
              failure_class_reason(cls.primary));
        check(cls.firstFaultTick == 0 && cls.contributingCount == 0,
              "a completed run clears the fault list");
    }

    /* 14. Missing-checkpoint accounting is lap-aware and never negative (the runner's
     * formula, extracted as run_report_missed_checkpoints() so this test exercises the SAME
     * function the runner reports with). Drive the REAL progress machine for 0..3 completed
     * laps plus 0..24 scored crossings of the current lap, count the crossings the machine
     * actually emitted, and check the formula's answer against that independent count. The
     * old block re-derived `missed` from its own arithmetic, so it could never fail and
     * validated nothing about the runner (PR #80 review). */
    {
        const int gates = 25;
        TrackDefinition t;
        memset(&t, 0, sizeof(t));
        track_load_chicane(&t);
        check(t.checkpointCount == gates, "precondition: the chicane carries %d gates (got %d)",
              gates, t.checkpointCount);

        for (int completed = 0; completed <= 3; completed++) {
            for (int crossed = 0; crossed < gates; crossed++) {
                RacerProgress p;
                memset(&p, 0, sizeof(p));
                track_reset_progress_at(&p, &t, 0);
                int crossingsThisLap = 0;

                /* completed full laps: sweep scored gates 1..24, then gate 0 closes the lap. */
                for (int lap = 0; lap < completed; lap++) {
                    for (int g = 1; g < gates; g++) {
                        const Checkpoint *c = &t.checkpoints[g];
                        const Vector2 a = { c->centerM.x - c->forwardUnit.x * 2.0f,
                                            c->centerM.y - c->forwardUnit.y * 2.0f };
                        const Vector2 b = { c->centerM.x + c->forwardUnit.x * 2.0f,
                                            c->centerM.y + c->forwardUnit.y * 2.0f };
                        const TrackCheckpointEvent ev = track_update_checkpoints(&t, &p, a, b);
                        check(ev.crossed && !ev.outOfOrder, "lap %d: gate %d crosses in order",
                              lap, g);
                    }
                    {
                        const Checkpoint *c0 = &t.checkpoints[0];
                        const Vector2 a0 = { c0->centerM.x - c0->forwardUnit.x * 2.0f,
                                             c0->centerM.y - c0->forwardUnit.y * 2.0f };
                        const Vector2 b0 = { c0->centerM.x + c0->forwardUnit.x * 2.0f,
                                             c0->centerM.y + c0->forwardUnit.y * 2.0f };
                        const TrackCheckpointEvent ev =
                            track_update_checkpoints(&t, &p, a0, b0);
                        check(ev.lapCompleted, "lap %d closes at gate 0", lap);
                    }
                    check(p.lap == lap + 1, "lap counter advances to %d (got %d)", lap + 1,
                          p.lap);
                }

                /* crossed scored gates of the current (incomplete) lap. */
                for (int g = 1; g <= crossed; g++) {
                    const Checkpoint *c = &t.checkpoints[g];
                    const Vector2 a = { c->centerM.x - c->forwardUnit.x * 2.0f,
                                        c->centerM.y - c->forwardUnit.y * 2.0f };
                    const Vector2 b = { c->centerM.x + c->forwardUnit.x * 2.0f,
                                        c->centerM.y + c->forwardUnit.y * 2.0f };
                    const TrackCheckpointEvent ev = track_update_checkpoints(&t, &p, a, b);
                    check(ev.crossed && !ev.outOfOrder, "current lap: gate %d crosses in order",
                          g);
                    crossingsThisLap++;
                }
                check(p.lap == completed, "state holds %d completed laps (got %d)", completed,
                      p.lap);
                check(p.nextCheckpoint == (crossed + 1) % gates,
                      "next owed gate is %d after %d crossings (got %d)", (crossed + 1) % gates,
                      crossed, p.nextCheckpoint);

                /* The runner's formula, computed from the real state. */
                const int missed = run_report_missed_checkpoints(&p, gates);
                /* Independent expectation from the machine's event count: the scored gates
                 * 1..24 still owed. The start/finish gate is the lap anchor (never scored), and
                 * once every scored gate is crossed (the wrap) nothing is missed. */
                const int expectedMissed = (gates - 1) - crossingsThisLap;
                check(missed == expectedMissed,
                      "lap %d, %d crossings: missed is the scored gates still owed (%d vs %d)",
                      completed, crossed, missed, expectedMissed);
                check(missed >= 0 && missed < gates,
                      "missed is never negative and bounded (%d, lap %d, crossings %d)", missed,
                      completed, crossed);
            }
        }
        track_free(&t);
    }

    /* 15. The classification is a pure function: identical rows, identical verdict. */
    {
        TelemetryRow rows[100];
        for (int i = 0; i < 100; i++) {
            cls_fill_row(&rows[i], cls_tick(i), 0.0f, 0.0f, 0, -1, 1, 0, 0, 5, 5, 0.0f, 0.0f,
                         0);
        }
        cls_default_inputs(&in);
        ValidationClassification a, b;
        validation_classify(rows, 100, NULL, &in, &a);
        validation_classify(rows, 100, NULL, &in, &b);
        check(a.primary == b.primary && a.firstFaultTick == b.firstFaultTick &&
                  a.contributingCount == b.contributingCount &&
                  a.lastCheckpointIndex == b.lastCheckpointIndex &&
                  a.expectedCheckpointIndex == b.expectedCheckpointIndex &&
                  a.furthestRouteDistanceM == b.furthestRouteDistanceM &&
                  a.timeSinceProgressS == b.timeSinceProgressS,
              "classification is deterministic across runs");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: chicane-gates-dense — the 25-gate chicane accepts a legal lap, rejects the     */
/* y=farY shortcut, and reports a forward skip                                             */
/* ------------------------------------------------------------------------------------- */

static void scenario_chicane_gates_dense(void)
{
    TrackDefinition track;
    RacerProgress progress;
    memset(&track, 0, sizeof(track));
    memset(&progress, 0, sizeof(progress));
    track_load_chicane(&track);
    check(track.checkpointCount == 25, "the densified chicane carries 25 gates (got %d)",
          track.checkpointCount);
    track_reset_progress_at(&progress, &track, 0);

    /* A legal lap: cross gates 1..24 in order, then gate 0 closes the lap. Each sweep runs
     * from 2 m upstream of the gate to 2 m downstream, through the gate centre. */
    {
        int outOfOrder = 0;
        for (int g = 1; g < 25; g++) {
            const Checkpoint *c = &track.checkpoints[g];
            const Vector2 a = { c->centerM.x - c->forwardUnit.x * 2.0f,
                                c->centerM.y - c->forwardUnit.y * 2.0f };
            const Vector2 b = { c->centerM.x + c->forwardUnit.x * 2.0f,
                                c->centerM.y + c->forwardUnit.y * 2.0f };
            const TrackCheckpointEvent ev = track_update_checkpoints(&track, &progress, a, b);
            check(ev.crossed && !ev.outOfOrder, "gate %d crossed in order on a legal lap", g);
            if (ev.outOfOrder) outOfOrder++;
        }
        const Checkpoint *c0 = &track.checkpoints[0];
        const Vector2 a0 = { c0->centerM.x - c0->forwardUnit.x * 2.0f,
                             c0->centerM.y - c0->forwardUnit.y * 2.0f };
        const Vector2 b0 = { c0->centerM.x + c0->forwardUnit.x * 2.0f,
                             c0->centerM.y + c0->forwardUnit.y * 2.0f };
        const TrackCheckpointEvent ev = track_update_checkpoints(&track, &progress, a0, b0);
        check(ev.lapCompleted, "gate 0 closes the lap on a legal lap");
        check(outOfOrder == 0, "no out-of-order crossing on a legal lap");
        check(progress.lap == 1, "one lap completed (got %d)", progress.lap);
        check(!progress.lapInvalid, "a legal lap is not invalidated");
    }

    /* The shortcut: driving straight along y = farY crosses the entry-side gates but MISSES the
     * chicane apex gates (13-15), then trips the exit side out of order — the layout's purpose
     * (a shortcut is rejected loudly, not silently scored). */
    {
        RacerProgress p2;
        memset(&p2, 0, sizeof(p2));
        track_reset_progress_at(&p2, &track, 0);
        const float y = 90.0f;
        bool apexTouched = false;
        int entryCrossings = 0;
        int outOfOrder = 0;
        for (int xi = 0; xi <= 25; xi++) {
            const float x = 60.0f - 4.0f * (float)xi;
            const TrackCheckpointEvent ev = track_update_checkpoints(
                &track, &p2, (Vector2){ x + 2.0f, y }, (Vector2){ x - 2.0f, y });
            if (ev.crossed && (ev.index == 13 || ev.index == 14 || ev.index == 15))
                apexTouched = true;
            if (ev.crossed) entryCrossings++;
            if (ev.outOfOrder) outOfOrder++;
        }
        /* The negative apex assertion is only meaningful if the sweep actually reaches the
         * track and crosses the entry-side gates; a future content change that moves the
         * chicane out from under y = 90 must fail here loudly (PR #80 review). */
        check(entryCrossings > 0,
              "the shortcut sweep reaches the track and crosses gates (got %d)",
              entryCrossings);
        check(!apexTouched, "the shortcut never crosses the chicane apex gates (13-15)");
        check(p2.lapInvalid, "the shortcut invalidates the lap");
        check(outOfOrder > 0, "the shortcut trips an out-of-order crossing at the exit");
        check(p2.lap == 0, "the shortcut closes no lap");
    }

    /* A forward skip on the real track: cross gate 5 while owing gate 3. */
    {
        RacerProgress p3;
        memset(&p3, 0, sizeof(p3));
        track_reset_progress_at(&p3, &track, 0);
        for (int g = 1; g <= 2; g++) {
            const Checkpoint *c = &track.checkpoints[g];
            const Vector2 a = { c->centerM.x - c->forwardUnit.x * 2.0f,
                                c->centerM.y - c->forwardUnit.y * 2.0f };
            const Vector2 b = { c->centerM.x + c->forwardUnit.x * 2.0f,
                                c->centerM.y + c->forwardUnit.y * 2.0f };
            track_update_checkpoints(&track, &p3, a, b);
        }
        check(p3.nextCheckpoint == 3, "owed gate 3 after gates 1,2 (got %d)",
              p3.nextCheckpoint);
        const Checkpoint *c5 = &track.checkpoints[5];
        const Vector2 a5 = { c5->centerM.x - c5->forwardUnit.x * 2.0f,
                             c5->centerM.y - c5->forwardUnit.y * 2.0f };
        const Vector2 b5 = { c5->centerM.x + c5->forwardUnit.x * 2.0f,
                             c5->centerM.y + c5->forwardUnit.y * 2.0f };
        const TrackCheckpointEvent ev = track_update_checkpoints(&track, &p3, a5, b5);
        check(ev.outOfOrder, "gate 5 while owing gate 3 is out of order");
        check(ev.index == 5, "the event names the gate actually crossed (got %d)", ev.index);
        check(p3.lapInvalid, "the forward skip invalidates the lap");
        check(p3.nextCheckpoint == 3, "the skip does not advance progress (got %d)",
              p3.nextCheckpoint);
    }

    track_free(&track);
}

static const TestScenario kGameplayScenarios[] = {
    { "track-surface", "track geometry, init/free life-cycle, and per-point surface query",
      scenario_track_surface },
    { "lap-target-results", "reaching RESULTS_TARGET_LAPS live flips PLAYING to RESULTS",
      scenario_lap_target_results },
    { "race-session", "session lifecycle: phases, countdown, pause, restart, modes, results",
      scenario_race_session },
    { "collision-barrier", "capsule barrier collision, swept test, impulse, and crash lockout",
      scenario_collision_barrier },
    { "collision-units",
      "direct collision_resolve_track tests: count, push, impulse, multi-contact",
      scenario_collision_units },
    { "collision-world",
      "issue #26 world contract: stable ids, layers, authored objects, multi-proxy order, "
      "penetration recovery, corners, contact feed",
      scenario_collision_world },
    { "collision-broadphase",
      "issue #26 broadphase: grid-vs-brute property tests with fixed seeds, overflow flag, "
      "measured query benchmark (timing gated behind CIRCUIT_COLLISION_BENCH)",
      scenario_collision_broadphase },
    { "checkpoint-lap", "ordered gates, out-of-order detection, forward-only, and lap timing",
      scenario_checkpoint_lap },
    { "checkpoint-lap-sf",
      "start/finish line lap closing: latch across ticks, no early award, no SF no close",
      scenario_checkpoint_lap_start_finish },
    { "progress-isolation",
      "two racers advance independent progress through one immutable TrackDefinition",
      scenario_progress_isolation },
    { "track-runoff", "three surface bands and barriers standing at the runoff edge",
      scenario_track_runoff },
    { "chicane-track", "the validation circuit: closed loop, gates, start pose, geometry hash",
      scenario_chicane_track },
    { "ai-lap",
      "the driver laps the chicane through Input alone, in order, on the line it found",
      scenario_ai_lap },
    { "ai-no-privilege",
      "AI driver has no side channels: 3600-tick checksum parity with input replay",
      scenario_ai_no_privilege },
    { "ai-roster-laps", "uniform AiDriverConfig completes the full run on all 6 roster cars",
      scenario_ai_roster_laps },
    { "planned-line",
      "the driver's own path search is legal, checkpoint-valid, and faster than the centreline",
      scenario_planned_line },
    { "failure-classification",
      "every #78 failure class classifies correctly, by construction, on hand-built rows",
      scenario_failure_classification },
    { "chicane-gates-dense",
      "the 25-gate chicane accepts a legal lap, rejects the y=farY shortcut, reports skips",
      scenario_chicane_gates_dense },
    { "particle-pool", "init, spawn, round-robin wrap, update, and lifecycle",
      scenario_particle_pool },
    { "state-machine", "MENU/PLAYING/PAUSED/RESULTS transitions", scenario_state_machine },
    { "lap-average", "perimeter drive recorded once, replayed 10x: checksum, gates, energy",
      scenario_lap_average },
    { "car-selection",
      "issue #32 selection model: id-sorted view of the 6 roster cars, stable resolve, "
      "recall round-trip",
      scenario_car_selection },
    { "setup-editor",
      "issue #33 setup model: 14 editable items, bounded validated edits, reset, editor "
      "isolation, clamp at max",
      scenario_setup_editor },
    { "launch-per-drivetrain",
      "every roster car spawns through the same entrant path carrying its stable id",
      scenario_launch_per_drivetrain },
    { "roster-gate",
      "issues #31/#33: the live roster enforces the promotion checklist and class rules",
      scenario_roster_gate },
};

TestScenarioGroup test_gameplay_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kGameplayScenarios;
    group.count = sizeof(kGameplayScenarios) / sizeof(kGameplayScenarios[0]);
    return group;
}
