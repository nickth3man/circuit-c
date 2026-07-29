/*
 * gameplay_tests.c — track geometry and surfaces, barrier collision, scoring, high-score
 * persistence, checkpoints and laps, the particle pool, and the state machine.
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

#include "dev/car_corpus.h"
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
#include "game/scoring.h"
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

/* ------------------------------------------------------------------------------------- */
/* Scenario: track surface                                                                 */
/* ------------------------------------------------------------------------------------- */

static void scenario_track_surface(void)
{
    /* Life-cycle: initialise, free, double-free safety. */
    Track track;
    memset(&track, 0, sizeof(track));

    track_init(&track);
    check(track.nodes != NULL, "track init: nodes is non-NULL");
    check(track.count == 5, "track init: count == 5 (got %d)", track.count);
    check(track.offTrackSurfaceId == SURFACE_GRASS,
          "track init: offTrackSurfaceId is SURFACE_GRASS (got %d)",
          (int)track.offTrackSurfaceId);
    check(track.nextCheckpoint == 0, "track init: nextCheckpoint is 0");
    check(track.lap == 0, "track init: lap is 0");
    check_near((double)track.lapTimerS, 0.0, 0.0, "track init: lapTimerS is 0");

    /* Query the centre at (0, 0): inside the 200×150 m parking lot, so it should be asphalt. */
    const SurfaceId centerSurf = Track_SurfaceAt(&track, (Vector2){ 0.0f, 0.0f });
    check(centerSurf == SURFACE_ASPHALT, "Track_SurfaceAt origin returns ASPHALT (got %d)",
          (int)centerSurf);

    /* Query at a centreline node point: should be asphalt. */
    const SurfaceId nodeSurf = Track_SurfaceAt(&track, track.nodes[0].centerM);
    check(nodeSurf == SURFACE_ASPHALT,
          "Track_SurfaceAt(centreline node) returns ASPHALT (got %d)", (int)nodeSurf);

    /* Just inside the lot boundary: offset from a perimeter node by less than halfWidthM. */
    {
        const Vector2 insidePoint = { track.nodes[0].centerM.x,
                                      track.nodes[0].centerM.y +
                                          track.nodes[0].halfWidthM * 0.7f };
        const SurfaceId insideSurf = Track_SurfaceAt(&track, insidePoint);
        check(insideSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt inside boundary returns ASPHALT (got %d)", (int)insideSurf);
    }

    /* Just outside: (0, 100) is 25 m above the lot top at y = 75. */
    {
        const Vector2 outsidePoint = { 0.0f, 100.0f };
        const SurfaceId outsideSurf = Track_SurfaceAt(&track, outsidePoint);
        check(outsideSurf == SURFACE_GRASS,
              "Track_SurfaceAt outside boundary returns GRASS (got %d)", (int)outsideSurf);
    }

    /* Far away: (1000, 0). */
    {
        const SurfaceId farSurf = Track_SurfaceAt(&track, (Vector2){ 1000.0f, 0.0f });
        check(farSurf == SURFACE_GRASS, "Track_SurfaceAt far point returns GRASS (got %d)",
              (int)farSurf);
    }

    /* NULL / uninitialised track returns ASPHALT (defensive default). */
    {
        Track dummy;
        memset(&dummy, 0, sizeof(dummy));
        const SurfaceId nullSurf = Track_SurfaceAt(NULL, (Vector2){ 0.0f, 0.0f });
        check(nullSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(NULL, ...) returns ASPHALT (got %d)", (int)nullSurf);
        const SurfaceId uninitSurf = Track_SurfaceAt(&dummy, (Vector2){ 0.0f, 0.0f });
        check(uninitSurf == SURFACE_ASPHALT,
              "Track_SurfaceAt(uninitialised, ...) returns ASPHALT (got %d)", (int)uninitSurf);
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
    track_init(&game->track);

    /* Place the car near the boundary, heading straight down at it.
     * The bottom barrier is at y ≈ -79 m (centerline -75 minus halfWidth 4). */
    game->vehicle.positionM = (Vector2){ 0.0f, -75.5f }; /* ~3.5 m above the bottom barrier */
    game->vehicle.headingRad = -1.57079632679f;          /* pointing -Y (down) */
    game->vehicle.velocityLongitudinalMps = 30.0f;       /* body X forward = world -Y */
    game->vehicle.velocityLateralMps = 0.0f;
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.currHeadingRad = game->vehicle.headingRad;
    game->state = STATE_PLAYING;

    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -79). */
    const float yBefore = game->vehicle.positionM.y;
    check(yBefore > -79.0f, "car starts inside the track boundary (y = %.2f > -79.0)",
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
        if (yAfter < -85.0f) break; /* car passed far beyond, collision didn't work */
    }

    /* After the tick, the car should NOT have passed through the boundary. */
    check(yAfter >= -80.5f,
          "car did not tunnel through the barrier (y = %.4f, must be > -80.5)", (double)yAfter);
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

    track_free(&game->track);
    free(game);

    /* --- Glancing hit: car approaches at shallow angle to produce yaw spin --- */
    Game *game2 = alloc_game();
    game_init(game2);
    track_init(&game2->track);

    /* Place car near the bottom-right of the lot, heading right-down at a shallow angle
     * toward the bottom barrier at y ≈ -79 m. */
    game2->vehicle.positionM = (Vector2){ 80.0f, -75.5f };
    game2->vehicle.headingRad = -1.2f; /* ~68° clockwise from +X, i.e. heading right-down */
    game2->vehicle.velocityLongitudinalMps = 30.0f;
    game2->vehicle.velocityLateralMps = 0.0f;
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

    track_free(&game2->track);
    free(game2);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: checkpoint-lap — gate crossing, lap counting, forward-only, timer reset      */
/* ------------------------------------------------------------------------------------- */

static void scenario_checkpoint_lap(void)
{
    /* Build a tiny 4-node square track: 10 m × 10 m, counterclockwise.
     * Nodes: (0,0) → (10,0) → (10,10) → (0,10) → back to (0,0).
     * halfWidthM = 2 m so car positions are comfortably in-bounds. */
    Track track;
    memset(&track, 0, sizeof(track));
    track.nodes = (TrackNode *)calloc(4, sizeof(TrackNode));
    track.count = 4;
    track.offTrackSurfaceId = SURFACE_GRASS;
    track.nodes[0] = (TrackNode){ { 0.0f, 0.0f }, 2.0f, SURFACE_ASPHALT };
    track.nodes[1] = (TrackNode){ { 10.0f, 0.0f }, 2.0f, SURFACE_ASPHALT };
    track.nodes[2] = (TrackNode){ { 10.0f, 10.0f }, 2.0f, SURFACE_ASPHALT };
    track.nodes[3] = (TrackNode){ { 0.0f, 10.0f }, 2.0f, SURFACE_ASPHALT };
    track.nextCheckpoint = 0;
    track.lap = 0;
    track.lapTimerS = 0.0f;
    track.lastLapTimeS = 0.0f;

    /* --- Basic gate crossing --- */
    /* Gate 0: at (0,0), direction (+X), perpendicular = (0,±1). Gate spans y ∈ [-2, +2]. */
    /* Move car from negative X to positive X crossing x=0 at y=1 (inside the gate). */
    bool crossed =
        track_update_checkpoints(&track, (Vector2){ -0.1f, 1.0f }, (Vector2){ 0.1f, 1.0f });
    check(crossed, "crossing gate 0 advances the checkpoint");
    check(track.nextCheckpoint == 1, "nextCheckpoint is 1 after gate 0 (got %d)",
          track.nextCheckpoint);
    check(track.lap == 0, "lap stays 0 after first gate (got %d)", track.lap);

    /* Gate 1: at (10,0), direction (+Y), perpendicular = (±1,0). Gate spans x ∈ [8,12]. */
    crossed =
        track_update_checkpoints(&track, (Vector2){ 10.0f, -0.1f }, (Vector2){ 10.0f, 0.1f });
    check(crossed, "crossing gate 1 advances the checkpoint");
    check(track.nextCheckpoint == 2, "nextCheckpoint is 2 after gate 1 (got %d)",
          track.nextCheckpoint);

    /* Gate 2: at (10,10), direction (-X), perpendicular = (0,±1). Gate spans y ∈ [8,12]. */
    crossed =
        track_update_checkpoints(&track, (Vector2){ 10.1f, 10.0f }, (Vector2){ 9.9f, 10.0f });
    check(crossed, "crossing gate 2 advances the checkpoint");
    check(track.nextCheckpoint == 3, "nextCheckpoint is 3 after gate 2 (got %d)",
          track.nextCheckpoint);

    /* --- Reverse crossing does NOT advance --- */
    /* Move backward through gate 2 (from right to left at the top). Direction is -X, which
     * is opposite to the track's forward direction (+X when moving from 1 to 2, but gate 2 
     * direction is from node 2 to node 3 which is (-X) for the top straight going right to
     * left. The gate at node 2 (top-right) has direction from (10,10) → (0,10) which is -X.
     * The car moving left at the top is WITH the track direction, so let me construct a 
     * proper reverse test: gate 0's forward direction is (+X). Moving -X through it: */
    track.nextCheckpoint = 0;
    crossed =
        track_update_checkpoints(&track, (Vector2){ 0.2f, 1.0f }, (Vector2){ -0.2f, 1.0f });
    check(!crossed, "reverse crossing of gate 0 does NOT advance");
    check(track.nextCheckpoint == 0, "nextCheckpoint still 0 after reverse crossing (got %d)",
          track.nextCheckpoint);

    /* --- Lap completion --- */
    /* Cross gate 0 forward, then 1, 2, 3 (last gate). */
    track.nextCheckpoint = 0;
    track.lap = 0;
    track.lapTimerS = 5.0f;
    check(track_update_checkpoints(&track, (Vector2){ -0.1f, 1.0f }, (Vector2){ 0.1f, 1.0f }),
          "gate 0");
    check(track_update_checkpoints(&track, (Vector2){ 10.0f, -0.1f }, (Vector2){ 10.0f, 0.1f }),
          "gate 1");
    check(track_update_checkpoints(&track, (Vector2){ 10.1f, 10.0f }, (Vector2){ 9.9f, 10.0f }),
          "gate 2");
    /* Gate 3: at (0,10), direction (+X)? No, from 3 to 0: (0,10) → (0,0) which is -Y.
     * Direction = (0, -1). Perpendicular = (-1, 0) → gate spans x ∈ [-2, +2]. */
    /* Move downward (-Y) from y=10.1 to y=9.9 at x=1.0. The forward direction of the gate
     * at node 3 is from (0,10) to (0,0) which is (0, -Y). The car moving -Y matches
     * the track's forward direction. */
    crossed =
        track_update_checkpoints(&track, (Vector2){ 1.0f, 10.1f }, (Vector2){ 1.0f, 9.9f });
    check(crossed, "crossing gate 3 (last gate) advances");
    check(track.nextCheckpoint == 0, "nextCheckpoint wraps to 0 (got %d)",
          track.nextCheckpoint);
    check(track.lap == 1, "lap increments to 1 (got %d)", track.lap);
    check(track.lapTimerS < 0.1f, "lapTimerS resets on lap completion (%.4f s)",
          (double)track.lapTimerS);
    check(track.lastLapTimeS > 4.5f,
          "lastLapTimeS records the completed lap time (%.4f s > 4.5)",
          (double)track.lastLapTimeS);

    /* --- Timer accumulation --- */
    track.lapTimerS = 0.0f;
    track.lapTimerS += 0.5f;
    check_near((double)track.lapTimerS, 0.5, 1e-6, "lapTimerS accumulates");

    /* --- Car outside gate span does NOT trigger --- */
    /* Gate 0 spans y ∈ [-2, +2]. Car crosses at y = 5 should NOT count. */
    track.nextCheckpoint = 0;
    track.lap = 0;
    crossed =
        track_update_checkpoints(&track, (Vector2){ -0.1f, 5.0f }, (Vector2){ 0.1f, 5.0f });
    check(!crossed, "crossing outside gate span does NOT advance");
    check(track.nextCheckpoint == 0,
          "nextCheckpoint unchanged after out-of-bounds cross (got %d)", track.nextCheckpoint);

    /* --- Stationary car does NOT trigger --- */
    crossed =
        track_update_checkpoints(&track, (Vector2){ 0.0f, 1.0f }, (Vector2){ 0.0f, 1.0f });
    check(!crossed, "stationary car does NOT advance checkpoints");

    /* --- NULL/edge case safety --- */
    check(!track_update_checkpoints(NULL, (Vector2){ 0, 0 }, (Vector2){ 1, 0 }),
          "track_update_checkpoints with NULL track returns false gracefully");

    free(track.nodes);
}

/* ------------------------------------------------------------------------------------- */
/* Phase 6: Scoring scenarios                                                            */
/* ------------------------------------------------------------------------------------- */

/*
 * scoring-accumulation: drive into a sustained drift and verify score accumulation,
 * combo multiplier, and grace-period reset.
 */
static void scenario_scoring_accumulation(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.differentialMode = (float)DIFF_LOCKED;
    game->spec.lateralLoadTransferEnabled = false;
    game->spec.engineRedlineRpm = 10000.0f;

    /* Build enough speed for a drift. Cruise at ~18 m/s on asphalt. */
    set_vehicle_rolling_speed(game, 18.0f);
    game->autoTrans.enabled = false;
    game->vehicle.selectedGear = 1;

    /* Run 20 ticks with gentle steer — score should be near zero. */
    int i;
    game->input.throttle = 0.60f;
    game->input.steer = 0.0f;
    game->input.handbrake = 0.0f;
    for (i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);
    float scoreBefore = game->driftScore;
    check(scoreBefore < 0.01f, "no significant score before drift initiation (%.1f)",
          (double)scoreBefore);
    /* Handbrake entry: zero throttle so drive torque doesn't fight the lock. */
    game->input.throttle = 0.0f;
    game->input.steer = 1.0f;
    game->input.handbrake = 1.0f;
    for (i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);

    /* Release handbrake; apply throttle and steer to maintain the slide. */
    game->input.handbrake = 0.0f;
    game->input.throttle = 0.50f;
    game->input.steer = 0.70f;

    bool everScoring = false;
    float lastScore = scoreBefore;
    bool scoreMonotonic = true;
    float peakCombo = 1.0f;

    for (i = 0; i < 400; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->derived.scoringDrift) {
            everScoring = true;
            if (game->driftScore < lastScore - 0.001f) scoreMonotonic = false;
            lastScore = game->driftScore;
            peakCombo = game->comboMultiplier;
        }
    }

    check(everScoring, "the car achieves scoringDrift at least once");
    check(scoreMonotonic, "driftScore increases monotonically while scoringDrift is true");
    check(game->driftScore > 20.0f, "driftScore accumulates beyond 20 (got %.1f)",
          (double)game->driftScore);
    check(peakCombo > 1.5f,
          "comboMultiplier rises above 1.5 during a sustained drift (peak %.3f)",
          (double)peakCombo);
    check(peakCombo <= 4.0f + 1e-4f, "comboMultiplier is capped at 4.0 (peak %.3f)",
          (double)peakCombo);
    check(game->comboMultiplier >= 1.0f, "comboMultiplier is never below 1.0 (final %.3f)",
          (double)game->comboMultiplier);

    /* Now stop drifting: straighten the wheel, drop throttle, apply brake. */
    game->input.steer = 0.0f;
    game->input.throttle = 0.0f;
    game->input.brake = 1.0f;

    /* Run until well past COMBO_GRACE_S (1.5 s → 180 ticks at 120 Hz). */
    bool comboReset = false;
    float timeReset = 0.0f;
    for (i = 0; i < 400; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->comboMultiplier < 1.0f + 1e-6f && game->driftTimeS < 1e-6f) {
            comboReset = true;
            timeReset = (float)i * FIXED_DT_S;
            break;
        }
    }
    check(comboReset, "comboMultiplier and driftTimeS reset after COMBO_GRACE_S (%.3f s)",
          (double)timeReset);

    /* driftTimeS should be zero now (or very close). */
    check(game->driftTimeS < 0.01f, "driftTimeS resets to zero after the grace period (%.4f)",
          (double)game->driftTimeS);

    free(game);
}

/*
 * scoring-rejection: assert scoringDrift stays false under conditions that must NOT score.
 *   - Creeping forward at 2 m/s (below MIN_DRIFT_SPEED_MPS)
 *   - Reversing with slide
 *   - Spinning in place (high yaw, ~0 speed)
 *   - Post-crash lockout
 *   - Past spin cutoff
 */
static void scenario_scoring_rejection(void)
{
    Game *game = alloc_game();

    /* --- Rejection 1: Creeping forward below MIN_DRIFT_SPEED_MPS --- */
    game_init(game);
    set_vehicle_rolling_speed(game, 2.0f);
    game->input.steer = 0.50f;
    game->input.throttle = 0.20f;
    int i;
    for (i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);
    check(!game->derived.scoringDrift, "creeping at %.1f m/s (below %.1f) does NOT score",
          (double)game->derived.speedMps, (double)MIN_DRIFT_SPEED_MPS);
    check(game->driftScore < 0.01f, "no score accrued while creeping");

    /* --- Rejection 2: Reversing with slide --- */
    game_init(game);
    /* Reverse: set longitudinal velocity negative. */
    game->vehicle.velocityLongitudinalMps = -8.0f;
    game->vehicle.velocityLateralMps = 3.0f;
    game->vehicle.yawRateRadS = 1.0f;
    game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS =
        -8.0f / game->spec.wheelRadiusM;
    game->vehicle.selectedGear = -1;

    /* Force derived values to match so classification sees the slide. */
    game->derived.speedMps = 8.0f;
    game->derived.bodySideslipRad = atan2f(3.0f, 8.0f);
    game->derived.rearSlipAngleRad = 0.15f;

    /* Call classify directly on our carefully set reverse state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "reversing (vx %.1f < 0) does NOT score",
          (double)game->vehicle.velocityLongitudinalMps);

    /* --- Rejection 3: Spinning in place (high yaw, ~0 speed) --- */
    game_init(game);
    game->vehicle.velocityLongitudinalMps = 0.1f;
    game->vehicle.velocityLateralMps = 0.1f;
    game->vehicle.yawRateRadS = 3.0f;
    /* Force derived state to match before classification. */
    game->derived.speedMps = 0.1414f;
    game->derived.bodySideslipRad = 0.8f;
    game->derived.rearSlipAngleRad = 0.5f;
    /* Call classify directly — avoids physics overwriting our setup. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "spinning at near-zero speed (%.2f m/s) does NOT score",
          (double)game->derived.speedMps);

    /* --- Rejection 4: Post-crash lockout --- */
    game_init(game);
    set_vehicle_rolling_speed(game, 18.0f);
    game->input.steer = 0.40f;
    game->input.throttle = 0.80f;
    /* Force the car into a slide then activate crash lockout. */
    game->input.handbrake = 1.0f;
    for (i = 0; i < 40; i++) game_fixed_update(game, FIXED_DT_S);
    game->input.handbrake = 0.0f;
    /* Force crash lockout and run one tick to let scoring classify. */
    game->crashLockoutTimerS = CRASH_LOCKOUT_S;
    game_fixed_update(game, FIXED_DT_S);
    check(!game->derived.scoringDrift,
          "post-crash lockout prevents scoringDrift (lockout %.3f s)",
          (double)game->crashLockoutTimerS);
    check(game->driftScore < 0.01f, "no score accrued during crash lockout");

    /* --- Rejection 5: Past spin cutoff --- */
    game_init(game);
    /* Force a huge body sideslip — vx tiny, vy large gives atan2(-large, small) ~ -pi/2
     * For SPIN_CUTOFF_RAD = 1.48: use vy = -20, vx = 2 → atan2(-20, 2) = -1.471
     * Need abs > 1.48, so use vy = -20, vx = 1 → atan2(-20, 1) = -1.5208 rad > 1.48. */
    game->vehicle.velocityLongitudinalMps = 1.0f;
    game->vehicle.velocityLateralMps = -20.0f;
    game->vehicle.yawRateRadS = 5.0f;
    game->derived.speedMps = sqrtf(1.0f * 1.0f + 20.0f * 20.0f);
    game->derived.bodySideslipRad = atan2f(-20.0f, 1.0f);
    game->derived.rearSlipAngleRad = 0.30f;
    /* Ensure sideslip exceeds SPIN_CUTOFF_RAD (1.48 rad ~ 85 deg). */
    check(fabsf(game->derived.bodySideslipRad) > SPIN_CUTOFF_RAD,
          "precondition: sideslip (%.3f rad) exceeds spin cutoff (%.3f rad)",
          (double)fabsf(game->derived.bodySideslipRad), (double)SPIN_CUTOFF_RAD);
    /* Call classify directly — physics would alter our carefully set state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift, "past spin cutoff (%.3f rad > %.3f rad) does NOT score",
          (double)fabsf(game->derived.bodySideslipRad), (double)SPIN_CUTOFF_RAD);

    free(game);
}

/*
 * scoring-determinism: prove that scoring state changes do not feed back into any
 * physical force. Three sub-proofs:
 *
 *   1. Two identical runs produce identical checksums (basic determinism).
 *   2. Corrupting scoring state before every step produces the SAME checksum,
 *      proving the scoring fields are not read by any force computation.
 *   3. Vehicle states are bit-identical after the corrupted run, proving no
 *      scoring state leaked into integration.
 */
static void scenario_scoring_determinism(void)
{
    Game *a = alloc_game();
    Game *b = alloc_game();
    game_init(a);
    game_init(b);

    /* Give both games the same initial rolling speed and inputs. */
    set_vehicle_rolling_speed(a, 14.0f);
    set_vehicle_rolling_speed(b, 14.0f);

    /* Run 150 ticks with handbrake + steer to build a slide. */
    const float steerInputs[] = { 0.35f, 0.35f, 0.35f, 0.35f, -0.10f };
    const float throttleInputs[] = { 0.80f, 0.80f, 0.40f, 0.30f, 0.00f };
    const float handbrakeInputs[] = { 1.0f, 0.5f, 0.0f, 0.0f, 0.0f };
    const int switchTicks[] = { 0, 20, 40, 60, 90 };
    int sw = 0;
    int i;

    for (i = 0; i < 150; i++) {
        while (sw < 4 && i >= switchTicks[sw + 1]) sw++;
        a->input.steer = steerInputs[sw];
        a->input.throttle = throttleInputs[sw];
        a->input.handbrake = handbrakeInputs[sw];
        b->input.steer = steerInputs[sw];
        b->input.throttle = throttleInputs[sw];
        b->input.handbrake = handbrakeInputs[sw];

        /* Sabotage b's scoring state before the step. */
        b->driftScore = (float)(i * 137);
        b->bestScore = (float)(i * 251);
        b->driftTimeS = (float)(i % 10);
        b->comboMultiplier = (float)((i % 5) + 1.0f);
        b->comboTimerS = (float)(i % 3);
        b->derived.scoringDrift = ((i & 1) == 0);
        b->derived.physicallySliding = ((i & 2) == 0);

        game_fixed_update(a, FIXED_DT_S);
        game_fixed_update(b, FIXED_DT_S);
    }

    /* Proof 1: checksums match — scoring state changes produced identical physics. */
    check(a->stateChecksum == b->stateChecksum,
          "identical inputs produce identical physics checksums (%08x vs %08x)",
          a->stateChecksum, b->stateChecksum);

    /* Proof 2: vehicle states are bit-identical (scoring never touched them). */
    check(memcmp(&a->vehicle, &b->vehicle, sizeof(VehicleState)) == 0,
          "vehicle states are bit-identical after scoring corruption");

    free(b);
    free(a);
}

/* Helper for the highscore-persistence scenario: write a string to a file. */
static void hs_write_file(const char *path, const char *contents)
{
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "%s", contents);
        fclose(f);
    }
}

/* Helper: read and validate exactly like persistence_load_score in game.c. */
static float hs_read_score(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0.0f;
    long parsed = 0;
    int matched = fscanf(f, "%ld", &parsed);
    fclose(f);
    if (matched == 1 && parsed >= 0 && parsed <= (long)MAX_VALID_SCORE) return (float)parsed;
    return 0.0f;
}

/*
 * highscore-persistence: file I/O validation for the score persistence pattern.
 *   - Round-trip a known score
 *   - Reject garbage/corrupted data
 *   - Reject out-of-range values
 *   - Reject negative values
 * Uses standard C I/O (the same pattern persistence_load_score uses).
 */
static void scenario_highscore_persistence(void)
{
    const char *tempPath = TELEMETRY_DIR "/_test_highscore.txt";
    float loaded;
    char buf[32];

    /* The fixture lives under the telemetry root, which a clean checkout does not have yet.
     * Deliberately not a check(): this scenario asserts score persistence, and adding an
     * assertion here would change the suite's check count for an unrelated reason. */
    (void)telemetry_ensure_dir(TELEMETRY_DIR);
    remove(tempPath);

    /* --- Round-trip: write 12345, read back --- */
    hs_write_file(tempPath, "12345");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 12345.0) < 0.5,
          "score 12345 round-trips through file (got %.0f)", (double)loaded);

    /* --- Round-trip: write 0, read back --- */
    hs_write_file(tempPath, "0");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "score 0 round-trips (got %.0f)", (double)loaded);

    /* --- Round-trip: write MAX_VALID_SCORE, read back --- */
    snprintf(buf, sizeof(buf), "%ld", (long)MAX_VALID_SCORE);
    hs_write_file(tempPath, buf);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - (double)MAX_VALID_SCORE) < 0.5,
          "MAX_VALID_SCORE round-trips (got %.0f)", (double)loaded);

    /* --- Reject garbage: write "hello world" --- */
    hs_write_file(tempPath, "hello world");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5,
          "garbage file is rejected, bestScore stays at 0 (got %.0f)", (double)loaded);

    /* --- Reject empty file --- */
    hs_write_file(tempPath, "");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "empty file is rejected (got %.0f)",
          (double)loaded);

    /* --- Reject out-of-range value (> MAX_VALID_SCORE) --- */
    hs_write_file(tempPath, "9999999999");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5,
          "out-of-range value 9999999999 (>%ld) is rejected (got %.0f)", (long)MAX_VALID_SCORE,
          (double)loaded);

    /* --- Reject negative value --- */
    hs_write_file(tempPath, "-42");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "negative value -42 is rejected (got %.0f)",
          (double)loaded);

    /* --- Leading integer in "100abc" is parsed (fscanf behaviour) --- */
    hs_write_file(tempPath, "100abc");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 100.0) < 0.5,
          "leading integer in '100abc' is parsed as 100 (got %.0f)", (double)loaded);

    /* --- File does not exist --- */
    remove(tempPath);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "missing file returns default 0 (got %.0f)",
          (double)loaded);

    remove(tempPath);
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

    /* --- MENU + pause → PLAYING (with vehicle reset and score zeroed). --- */
    game->vehicle.positionM.x = 100.0f;
    game->driftScore = 500.0f;
    game->driftTimeS = 3.0f;
    game->comboMultiplier = 2.5f;
    game->comboTimerS = 1.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from MENU → PLAYING (got %d)", (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset to origin on MENU→PLAYING");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on MENU→PLAYING");
    check_near((double)game->comboMultiplier, 1.0, 1e-6, "combo reset on MENU→PLAYING");

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

    /* --- PLAYING + reset → PLAYING (vehicle reset, score zeroed). --- */
    game->vehicle.positionM.x = 150.0f;
    game->driftScore = 250.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PLAYING stays PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PLAYING reset");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on PLAYING reset");

    /* --- PAUSED + reset → PLAYING (vehicle reset). --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PAUSED, "in PAUSED before reset test");
    game->vehicle.positionM.x = 50.0f;
    game->driftScore = 100.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PAUSED → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6, "vehicle reset on PAUSED reset");

    /* --- RESULTS + pause → PLAYING (reset + score zeroed). --- */
    game->state = STATE_RESULTS;
    game->vehicle.positionM.x = 200.0f;
    game->driftScore = 750.0f;
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from RESULTS → PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset on RESULTS→PLAYING");
    check_near((double)game->driftScore, 0.0, 0.5, "score zeroed on RESULTS→PLAYING");

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

static const TestScenario kGameplayScenarios[] = {
    { "track-surface", "track geometry, init/free life-cycle, and per-point surface query",
      scenario_track_surface },
    { "collision-barrier", "capsule barrier collision, swept test, impulse, and crash lockout",
      scenario_collision_barrier },
    { "scoring-accumulation", "score accrues during a drift; combo multiplier rises and resets",
      scenario_scoring_accumulation },
    { "scoring-rejection", "low speed, reverse, spin, crash, and past-spin-cutoff rejected",
      scenario_scoring_rejection },
    { "scoring-determinism", "scoring state provably changes no physical force or checksum",
      scenario_scoring_determinism },
    { "highscore-persistence", "file load/save, garbage, range, and negative-value validated",
      scenario_highscore_persistence },
    { "checkpoint-lap", "gate crossing, lap counting, forward-only, and lap timer reset",
      scenario_checkpoint_lap },
    { "particle-pool", "init, spawn, round-robin wrap, update, and lifecycle",
      scenario_particle_pool },
    { "state-machine", "MENU/PLAYING/PAUSED/RESULTS transitions and scoring reset",
      scenario_state_machine },
};

TestScenarioGroup test_gameplay_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kGameplayScenarios;
    group.count = sizeof(kGameplayScenarios) / sizeof(kGameplayScenarios[0]);
    return group;
}
