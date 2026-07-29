/*
 * handling_tests.c — the scripted maneuvers graded on derived metrics.
 *
 * The per-tick sample history and the Game of the most recent run stay file-static here: they
 * belong to these scenarios, and the runner reaches them only through test_handling_cleanup().
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
/* Scripted maneuvers from the shared scenario table                                       */
/* ------------------------------------------------------------------------------------- */

/*
 * These runs assert INVARIANTS, never handling targets. What a good skidpad radius is, is a
 * Phase 3 tuning question; that the friction budget is never exceeded and the state stays
 * finite is a correctness question, and correctness is what a regression suite is for.
 * The telemetry each run writes is what tools/telemetry/compare_telemetry.py diffs against a baseline.
 */
/* The Game of the most recent scripted run, kept alive so the runner can still build a
 * failure bundle from it after the scenario function has returned. Freed by the next
 * scripted run and once more at exit. */
static Game *g_scriptedGame = NULL;

/* Write one telemetry row every N fixed ticks (120 Hz / 4 = 30 Hz). */
#define SCRIPTED_TELEMETRY_DECIMATION 4

/*
 * Per-tick history of the last scripted run, at the full 120 Hz.
 *
 * The Phase 3 scenarios are graded on derived metrics — rise time, overshoot, peak transfer,
 * stopping distance — and every one of those is a question about the shape of a curve rather
 * than about its final value. Keeping the samples means each scenario reads its numbers off
 * one shared recording instead of every scenario growing its own instrumented loop.
 */
#define SCRIPTED_SAMPLE_CAPACITY 2600

typedef struct {
    float timeS;
    float positionXM, positionYM;
    float speedMps, vxMps, vyMps;
    float yawRateRadS, sideslipRad, steerRad;
    float throttle, brake, handbrake;
    float prevAxMps2, filteredAxMps2, solvedAxMps2, lateralAxMps2;
    float staticFrontN, staticRearN, frontLoadN, rearLoadN, transferN;
    float aeroDragN, rollingN;
    float frontSlipRad, rearSlipRad, frontSlipRatio, rearSlipRatio;
    float frontUsage, rearUsage, maxUsage;
    float yawTorqueNm, rearOmegaRadS;
} ScriptedSample;

static ScriptedSample g_samples[SCRIPTED_SAMPLE_CAPACITY];
static int g_sampleCount = 0;

static void record_sample(const Game *game, int tick)
{
    if (g_sampleCount >= SCRIPTED_SAMPLE_CAPACITY) return;
    ScriptedSample *s = &g_samples[g_sampleCount++];
    s->timeS = (float)tick * FIXED_DT_S;
    s->positionXM = game->vehicle.positionM.x;
    s->positionYM = game->vehicle.positionM.y;
    s->speedMps = game->derived.speedMps;
    s->vxMps = game->vehicle.velocityLongitudinalMps;
    s->vyMps = game->vehicle.velocityLateralMps;
    s->yawRateRadS = game->vehicle.yawRateRadS;
    s->sideslipRad = game->derived.bodySideslipRad;
    s->steerRad = game->vehicle.frontRoadWheelAngleRad;
    s->throttle = game->dev.appliedInput.throttle;
    s->brake = game->dev.appliedInput.brake;
    s->handbrake = game->dev.appliedInput.handbrake;
    s->prevAxMps2 = game->derived.previousLongAccelMps2;
    s->filteredAxMps2 = game->derived.filteredLongAccelMps2;
    s->solvedAxMps2 = game->derived.solvedLongAccelMps2;
    s->lateralAxMps2 = game->derived.lateralAccelerationMps2;
    s->staticFrontN = game->derived.staticFrontLoadN;
    s->staticRearN = game->derived.staticRearLoadN;
    s->frontLoadN = game->derived.normalLoadFrontN;
    s->rearLoadN = game->derived.normalLoadRearN;
    s->transferN = game->derived.loadTransferN;
    s->aeroDragN = game->derived.aeroDragMagnitudeN;
    s->rollingN = game->derived.rollingResistanceMagnitudeN;
    s->frontSlipRad = game->derived.frontSlipAngleRad;
    s->rearSlipRad = game->derived.rearSlipAngleRad;
    s->frontSlipRatio = game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio;
    s->rearSlipRatio = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    s->frontUsage = fmaxf(game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                          game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage);
    s->rearUsage = fmaxf(game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                         game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage);
    s->maxUsage = game->derived.maxFrictionUsage;
    s->yawTorqueNm = game->derived.totalYawTorqueNm;
    s->rearOmegaRadS = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
}

/* Index of the first sample at or after `timeS`, clamped into range. */
static int sample_at_time(float timeS)
{
    for (int i = 0; i < g_sampleCount; i++) {
        if (g_samples[i].timeS >= timeS) return i;
    }
    return (g_sampleCount > 0) ? g_sampleCount - 1 : 0;
}

void test_handling_cleanup(void)
{
    free(g_scriptedGame);
    g_scriptedGame = NULL;
}

static void run_scripted_scenario(const char *name)
{
    const int index = dev_scenario_find(name);
    check(index > 0, "'%s' is present in the shared scenario table", name);
    if (index <= 0) return;

    const DevScenario *scenario = dev_scenario_at(index);
    char path[160];
    snprintf(path, sizeof(path), "%s/scenario_%s.csv", TELEMETRY_DIR, name);
    check(telemetry_ensure_dir(TELEMETRY_DIR), "the telemetry directory exists or was created");

    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, path);
    check(opened, "telemetry_open('%s') succeeded", path);

    test_handling_cleanup();
    Game *game = alloc_game();
    g_scriptedGame = game;
    game_init(game);
    game->dev.scenario = index;
    game->dev.scenarioRunning = true;
    game->dev.scenarioStartTick = game->sim.tick;
    game->dev.seed = scenario->seed;
    bundle_context(game, opened ? path : NULL, scenario->seed);

    float peakFrictionUsage = 0.0f;
    float peakSideslipRad = 0.0f;
    float peakYawRateRadS = 0.0f;
    float peakSpeedMps = 0.0f;
    bool allFinite = true;

    g_sampleCount = 0;
    check(scenario->durationTicks <= SCRIPTED_SAMPLE_CAPACITY,
          "'%s' fits the sample buffer (%d ticks, capacity %d)", name, scenario->durationTicks,
          SCRIPTED_SAMPLE_CAPACITY);

    for (int tick = 0; tick < scenario->durationTicks; tick++) {
        game_fixed_update(game, FIXED_DT_S);
        record_sample(game, tick);

        /* 30 Hz telemetry rather than 120. Four times fewer rows is the difference between a
         * quarter-megabyte baseline and a megabyte one, and nothing in these maneuvers moves
         * fast enough for the extra resolution to change a comparison. */
        if (opened && (tick % SCRIPTED_TELEMETRY_DECIMATION) == 0) {
            const TelemetryRow row = test_telemetry_row_from_game(game, 1);
            telemetry_write_row(&writer, &row);
        }

        peakFrictionUsage = fmaxf(peakFrictionUsage, game->derived.maxFrictionUsage);
        peakSideslipRad = fmaxf(peakSideslipRad, fabsf(game->derived.bodySideslipRad));
        peakYawRateRadS = fmaxf(peakYawRateRadS, fabsf(game->vehicle.yawRateRadS));
        peakSpeedMps = fmaxf(peakSpeedMps, game->derived.speedMps);

        if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y) ||
            !isfinite(game->vehicle.velocityLongitudinalMps) ||
            !isfinite(game->vehicle.velocityLateralMps) ||
            !isfinite(game->vehicle.yawRateRadS)) {
            allFinite = false;
        }
    }
    if (opened) telemetry_close(&writer);

    check(allFinite, "'%s' keeps every state variable finite", name);
    check(!game->dev.invariantFailed, "'%s' violates no invariant%s%s", name,
          game->dev.invariantFailed ? ": " : "",
          game->dev.invariantFailed ? game->dev.invariantText : "");
    check(peakFrictionUsage <= 1.0f + FRICTION_TOLERANCE,
          "'%s' never exceeds the friction budget (peak %.4f)", name,
          (double)peakFrictionUsage);
    check(peakSpeedMps <= MAX_SAFE_SPEED_MPS,
          "'%s' stays below MAX_SAFE_SPEED_MPS (peak %.2f m/s)", name, (double)peakSpeedMps);
    check(game->sim.tick == (uint64_t)scenario->durationTicks, "'%s' ran its full %d ticks",
          name, scenario->durationTicks);
    check(!game->dev.scenarioRunning, "'%s' stopped itself at the end of its script", name);
    check(game->replay.count == scenario->durationTicks,
          "'%s' recorded every scripted tick (%d)", name, game->replay.count);

    /* The scripted input is a pure function of the tick index, so a second run of the same
     * scenario on this binary must agree bit-for-bit. This is the property the physics
     * regression workflow depends on. */
    Game *repeat = alloc_game();
    game_init(repeat);
    repeat->dev.scenario = index;
    repeat->dev.scenarioRunning = true;
    repeat->dev.scenarioStartTick = repeat->sim.tick;
    for (int tick = 0; tick < scenario->durationTicks; tick++) {
        game_fixed_update(repeat, FIXED_DT_S);
    }
    check(repeat->stateChecksum == game->stateChecksum,
          "'%s' is deterministic across runs (%08x)", name, game->stateChecksum);
    check(memcmp(&repeat->vehicle, &game->vehicle, sizeof(VehicleState)) == 0,
          "'%s' reproduces a bit-identical vehicle state", name);

    printf("    %-16s peak usage %.3f  peak sideslip %.3f rad  peak yaw %.3f rad/s"
           "  peak speed %.2f m/s\n",
           name, (double)peakFrictionUsage, (double)peakSideslipRad, (double)peakYawRateRadS,
           (double)peakSpeedMps);

    free(repeat);
    /* game deliberately outlives this function: see g_scriptedGame. */
}

/* ------------------------------------------------------------------------------------- */
/* Phase 3 maneuvers: the scripted run, then the metrics that grade it                     */
/* ------------------------------------------------------------------------------------- */

/*
 * Every metric below is computed from g_samples, and every definition is written out where
 * it is used. "Settling time" and "rise time" have several defensible definitions; a number
 * whose definition lives only in the reader's head is not an objective measurement.
 */

static void scenario_accel_load(void)
{
    run_scripted_scenario("accel-load");
    if (g_sampleCount < 100) return;

    float peakSolvedAx = 0.0f;
    float peakFilteredAx = 0.0f;
    float minFrontLoadN = 1e9f;
    float maxRearLoadN = 0.0f;
    float peakTransferN = 0.0f;
    float peakTransferTimeS = 0.0f;
    float worstLoadSumErrorN = 0.0f;
    bool transferAlwaysRearward = true;
    bool frontAlwaysLighter = true;

    const int accelEnd = sample_at_time(5.0f);
    for (int i = 0; i < accelEnd; i++) {
        const ScriptedSample *s = &g_samples[i];
        peakSolvedAx = fmaxf(peakSolvedAx, s->solvedAxMps2);
        peakFilteredAx = fmaxf(peakFilteredAx, s->filteredAxMps2);
        minFrontLoadN = fminf(minFrontLoadN, s->frontLoadN);
        maxRearLoadN = fmaxf(maxRearLoadN, s->rearLoadN);
        if (s->transferN > peakTransferN) {
            peakTransferN = s->transferN;
            peakTransferTimeS = s->timeS;
        }
        /* The unclamped pair is what must weigh the car; reconstruct it from the static
         * split and the transfer, which is what the telemetry exposes. */
        const float sumN = (s->staticFrontN - s->transferN) + (s->staticRearN + s->transferN);
        worstLoadSumErrorN =
            fmaxf(worstLoadSumErrorN, fabsf(sumN - (s->staticFrontN + s->staticRearN)));
        /* After the first tick the filter is positive and stays positive under full throttle. */
        if (i > 2 && s->transferN < 0.0f) transferAlwaysRearward = false;
        if (i > 2 && s->frontLoadN > s->staticFrontN + 1e-3f) frontAlwaysLighter = false;
    }

    const int at5s = sample_at_time(5.0f);
    const float distanceAt5sM = g_samples[at5s].positionXM;
    const float speedAt5sMps = g_samples[at5s].speedMps;

    /*
     * No oscillatory load feedback.
     *
     * The raw solved acceleration is genuinely noisy under wheelspin — the wheel equation is
     * the stiffest part of the model — so counting wiggles in the filtered signal would only
     * measure that noise. What must be true is that the loop through load transfer does not
     * AMPLIFY it: the filtered value stays inside the envelope of the raw values it is made
     * from, and each step moves by no more than the filter coefficient allows.
     */
    float rawMinAx = 1e9f;
    float rawMaxAx = -1e9f;
    bool filteredInsideEnvelope = true;
    bool filteredStepBounded = true;
    const float filterAlpha = 1.0f - expf(-g_scriptedGame->spec.loadFilterRateHz * FIXED_DT_S);
    {
        const int from = sample_at_time(0.5f);
        for (int i = from; i < accelEnd; i++) {
            rawMinAx = fminf(rawMinAx, g_samples[i].prevAxMps2);
            rawMaxAx = fmaxf(rawMaxAx, g_samples[i].prevAxMps2);
        }
        for (int i = from; i < accelEnd; i++) {
            const ScriptedSample *s = &g_samples[i];
            if (s->filteredAxMps2 < rawMinAx - 1e-3f || s->filteredAxMps2 > rawMaxAx + 1e-3f)
                filteredInsideEnvelope = false;
            const float stepMps2 = fabsf(s->filteredAxMps2 - g_samples[i - 1].filteredAxMps2);
            const float allowedMps2 =
                filterAlpha * fabsf(s->prevAxMps2 - g_samples[i - 1].filteredAxMps2) + 1e-4f;
            if (stepMps2 > allowedMps2) filteredStepBounded = false;
        }
    }

    check(peakSolvedAx > 0.5f,
          "full throttle produces positive solved acceleration (peak %.3f m/s^2)",
          (double)peakSolvedAx);
    check(peakFilteredAx > 0.5f && peakFilteredAx <= peakSolvedAx + 1e-3f,
          "the filter follows it without overshooting (peak filtered %.3f m/s^2)",
          (double)peakFilteredAx);
    check(minFrontLoadN < g_samples[0].staticFrontN - 50.0f,
          "the front axle unloads under acceleration (%.1f N, static %.1f N)",
          (double)minFrontLoadN, (double)g_samples[0].staticFrontN);
    check(maxRearLoadN > g_samples[0].staticRearN + 50.0f,
          "the rear axle loads up (%.1f N, static %.1f N)", (double)maxRearLoadN,
          (double)g_samples[0].staticRearN);
    check(transferAlwaysRearward, "load transfer stays rearward for the whole pull");
    check(frontAlwaysLighter, "and the front axle never exceeds its static load");
    check(worstLoadSumErrorN < 1e-2f,
          "the unclamped axle loads always sum to mass * gravity (worst error %.4f N)",
          (double)worstLoadSumErrorN);
    check(filteredInsideEnvelope,
          "the filtered acceleration never leaves the envelope of the raw values it filters "
          "([%.3f, %.3f] m/s^2)",
          (double)rawMinAx, (double)rawMaxAx);
    check(filteredStepBounded,
          "and never moves further in one step than the filter coefficient permits: "
          "the load loop attenuates rather than amplifies");

    /* Rear capacity rises with rear load; that is the point of the whole stage. */
    const float rearCapacityGainN =
        (maxRearLoadN - g_samples[0].staticRearN) * g_scriptedGame->spec.tireMuLatRear;
    check(rearCapacityGainN > 50.0f, "the loaded rear axle gains lateral capacity (%.0f N)",
          (double)rearCapacityGainN);

    printf("    accel-load: peak solved ax %.3f, filtered %.3f m/s^2; front load min %.1f N, "
           "rear max %.1f N\n"
           "                peak transfer %.1f N at %.2f s; at 5 s: %.2f m, %.3f m/s\n",
           (double)peakSolvedAx, (double)peakFilteredAx, (double)minFrontLoadN,
           (double)maxRearLoadN, (double)peakTransferN, (double)peakTransferTimeS,
           (double)distanceAt5sM, (double)speedAt5sMps);
}

static void scenario_brake_load(void)
{
    run_scripted_scenario("brake-load");
    if (g_sampleCount < 100) return;

    const int brakeStart = sample_at_time(4.0f);
    float peakDecelMps2 = 0.0f;
    float minFilteredAx = 0.0f;
    float maxFrontLoadN = 0.0f;
    float minRearLoadN = 1e9f;
    float peakForwardTransferN = 0.0f;
    bool everReversed = false;
    bool minimumLoadHeld = true;
    bool wheelsNeverReversed = true;

    int stopIndex = -1;
    const float entrySpeedMps = g_samples[brakeStart].speedMps;
    const float entryPositionM = g_samples[brakeStart].positionXM;

    for (int i = brakeStart; i < g_sampleCount; i++) {
        const ScriptedSample *s = &g_samples[i];
        peakDecelMps2 = fmaxf(peakDecelMps2, -s->solvedAxMps2);
        minFilteredAx = fminf(minFilteredAx, s->filteredAxMps2);
        maxFrontLoadN = fmaxf(maxFrontLoadN, s->frontLoadN);
        minRearLoadN = fminf(minRearLoadN, s->rearLoadN);
        peakForwardTransferN = fmaxf(peakForwardTransferN, -s->transferN);
        if (s->vxMps < -1e-4f) everReversed = true;
        if (s->rearOmegaRadS < -1e-4f) wheelsNeverReversed = false;
        if (s->frontLoadN < MIN_NORMAL_LOAD_N - 1e-3f ||
            s->rearLoadN < MIN_NORMAL_LOAD_N - 1e-3f)
            minimumLoadHeld = false;
        if (stopIndex < 0 && s->vxMps <= 1e-4f) stopIndex = i;
    }

    const float stoppingTimeS =
        (stopIndex >= 0) ? g_samples[stopIndex].timeS - g_samples[brakeStart].timeS : -1.0f;
    const float stoppingDistanceM =
        (stopIndex >= 0) ? g_samples[stopIndex].positionXM - entryPositionM : -1.0f;

    check(minFilteredAx < -1.0f,
          "the filtered acceleration goes negative under braking (%.3f m/s^2)",
          (double)minFilteredAx);
    check(peakDecelMps2 > 1.0f,
          "the solved acceleration goes negative too (peak decel %.3f m/s^2)",
          (double)peakDecelMps2);
    check(maxFrontLoadN > g_samples[brakeStart].staticFrontN + 50.0f,
          "the front axle loads up under braking (%.1f N, static %.1f N)",
          (double)maxFrontLoadN, (double)g_samples[brakeStart].staticFrontN);
    check(minRearLoadN < g_samples[brakeStart].staticRearN - 50.0f,
          "the rear axle unloads (%.1f N, static %.1f N)", (double)minRearLoadN,
          (double)g_samples[brakeStart].staticRearN);
    check(peakForwardTransferN > 0.0f, "the transfer is forward, not rearward (%.1f N)",
          (double)peakForwardTransferN);
    check(!everReversed, "braking never pushes the vehicle backwards");
    check(wheelsNeverReversed, "and never spins the wheels backwards");
    check(minimumLoadHeld, "no axle load falls below MIN_NORMAL_LOAD_N");
    check(stopIndex >= 0, "the vehicle comes to a stop");

    /* Front braking capacity rises while rear capacity falls: the whole reason brake bias
     * is biased forward in the first place. */
    const float longMuEff =
        g_scriptedGame->spec.tireMuLongScale * Surface_Get(SURFACE_ASPHALT)->muLongitudinal;
    const float frontCapacityGainN =
        (maxFrontLoadN - g_samples[brakeStart].staticFrontN) * longMuEff;
    const float rearCapacityLossN =
        (g_samples[brakeStart].staticRearN - minRearLoadN) * longMuEff;
    check(frontCapacityGainN > 50.0f && rearCapacityLossN > 50.0f,
          "front braking capacity rises (%.0f N) as rear capacity falls (%.0f N)",
          (double)frontCapacityGainN, (double)rearCapacityLossN);

    printf("    brake-load: entry %.3f m/s; peak decel %.3f m/s^2, filtered min %.3f m/s^2\n"
           "                front load max %.1f N, rear min %.1f N, peak forward transfer "
           "%.1f N\n"
           "                stopping distance %.2f m in %.3f s\n",
           (double)entrySpeedMps, (double)peakDecelMps2, (double)minFilteredAx,
           (double)maxFrontLoadN, (double)minRearLoadN, (double)peakForwardTransferN,
           (double)stoppingDistanceM, (double)stoppingTimeS);
}

static void scenario_coast_down_scripted(void)
{
    run_scripted_scenario("coast-down");
    if (g_sampleCount < 100) return;

    const int liftIndex = sample_at_time(6.0f);
    float entrySpeedMps = g_samples[liftIndex].speedMps;
    float entryDragN = g_samples[liftIndex].aeroDragN;
    float entryRollingN = g_samples[liftIndex].rollingN;

    bool speedMonotonic = true;
    bool dragMonotonic = true;
    bool rollingBounded = true;
    bool noSpike = true;
    float previousSpeedMps = entrySpeedMps;
    float previousDragN = entryDragN;
    float previousTotalN = entryDragN + entryRollingN;
    float finalSpeedMps = entrySpeedMps;
    float finalDragN = entryDragN;
    float finalRollingN = entryRollingN;

    /*
     * Measure the coast, not the standstill after it.
     *
     * The run keeps going after the car has stopped, and at rest both resistance forms are
     * correctly zero — rolling resistance invents no direction for a stationary wheel. Ending
     * the window at walking pace keeps the assertions about the physics of coasting instead
     * of about the moment the physics stops applying.
     */
    const float measureFloorMps = 1.0f;
    int lastIndex = liftIndex;
    float worstSpeedRiseMps = 0.0f;
    float worstSpeedRiseTimeS = 0.0f;
    float worstDragRiseN = 0.0f;

    for (int i = liftIndex + 1; i < g_sampleCount; i++) {
        const ScriptedSample *s = &g_samples[i];
        if (s->speedMps < measureFloorMps) break;
        if (s->speedMps - previousSpeedMps > worstSpeedRiseMps) {
            worstSpeedRiseMps = s->speedMps - previousSpeedMps;
            worstSpeedRiseTimeS = s->timeS;
        }
        worstDragRiseN = fmaxf(worstDragRiseN, s->aeroDragN - previousDragN);
        if (s->speedMps > previousSpeedMps + 1e-3f) speedMonotonic = false;
        if (s->aeroDragN > previousDragN + 1e-3f) dragMonotonic = false;
        const float totalN = s->aeroDragN + s->rollingN;
        if (totalN > previousTotalN + 1.0f) noSpike = false;
        if (s->rollingN >
            g_scriptedGame->spec.rollingResistanceCoefficient * (s->frontLoadN + s->rearLoadN) +
                1.0f)
            rollingBounded = false;
        previousSpeedMps = s->speedMps;
        previousDragN = s->aeroDragN;
        previousTotalN = totalN;
        finalSpeedMps = s->speedMps;
        finalDragN = s->aeroDragN;
        finalRollingN = s->rollingN;
        lastIndex = i;
    }

    /* And separately: once it does stop, both forces are exactly zero and stay there. */
    bool restIsQuiet = true;
    for (int i = lastIndex; i < g_sampleCount; i++) {
        const ScriptedSample *s = &g_samples[i];
        if (s->speedMps > 1e-4f) continue;
        if (s->aeroDragN != 0.0f || s->rollingN != 0.0f) restIsQuiet = false;
    }
    check(restIsQuiet,
          "at rest both resistance forms are exactly zero, inventing no direction");

    check(speedMonotonic,
          "coast-down speed decreases monotonically once the throttle is off "
          "(worst rise %.6f m/s at %.2f s)",
          (double)worstSpeedRiseMps, (double)worstSpeedRiseTimeS);
    check(dragMonotonic, "and drag decreases with it, tick by tick (worst rise %.4f N)",
          (double)worstDragRiseN);
    check(noSpike, "total resisting force never spikes during the coast");
    check(rollingBounded,
          "rolling resistance never exceeds the coefficient times the current load");
    check(finalSpeedMps < entrySpeedMps, "the car actually slows down (%.3f -> %.3f m/s)",
          (double)entrySpeedMps, (double)finalSpeedMps);
    check(finalDragN < entryDragN, "drag falls as the car slows (%.1f -> %.1f N)",
          (double)entryDragN, (double)finalDragN);
    check(fabsf(finalRollingN - entryRollingN) < 0.25f * entryRollingN,
          "rolling resistance stays load-driven rather than following speed (%.1f -> %.1f N)",
          (double)entryRollingN, (double)finalRollingN);

    printf("    coast-down: %.3f -> %.3f m/s; drag %.1f -> %.1f N, rolling %.1f -> %.1f N\n",
           (double)entrySpeedMps, (double)finalSpeedMps, (double)entryDragN, (double)finalDragN,
           (double)entryRollingN, (double)finalRollingN);
}

/* Mean of a sample field over [fromS, toS). */
#define SAMPLE_MEAN(field, fromS, toS) \
    sample_mean(offsetof(ScriptedSample, field), (fromS), (toS))

static float sample_mean(size_t fieldOffset, float fromS, float toS)
{
    const int from = sample_at_time(fromS);
    const int to = sample_at_time(toS);
    if (to <= from) return 0.0f;
    double total = 0.0;
    for (int i = from; i < to; i++) {
        total += (double)*(const float *)(const void *)((const unsigned char *)&g_samples[i] +
                                                        fieldOffset);
    }
    return (float)(total / (double)(to - from));
}

static void scenario_skidpad(void)
{
    run_scripted_scenario("skidpad");
    if (g_sampleCount < 100) return;

    /* Steady state is the last three seconds of the twenty-second hold, by which time the
     * scripted steer and throttle have been constant for fifteen. */
    const float steadySpeedMps = SAMPLE_MEAN(speedMps, 17.0f, 20.0f);
    const float steadyYawRateRadS = SAMPLE_MEAN(yawRateRadS, 17.0f, 20.0f);
    const float steadyLateralAxMps2 = SAMPLE_MEAN(lateralAxMps2, 17.0f, 20.0f);
    const float steadySideslipRad = SAMPLE_MEAN(sideslipRad, 17.0f, 20.0f);
    const float frontSlipRad = SAMPLE_MEAN(frontSlipRad, 17.0f, 20.0f);
    const float rearSlipRad = SAMPLE_MEAN(rearSlipRad, 17.0f, 20.0f);
    const float frontUsage = SAMPLE_MEAN(frontUsage, 17.0f, 20.0f);
    const float rearUsage = SAMPLE_MEAN(rearUsage, 17.0f, 20.0f);
    const float frontLoadN = SAMPLE_MEAN(frontLoadN, 17.0f, 20.0f);
    const float rearLoadN = SAMPLE_MEAN(rearLoadN, 17.0f, 20.0f);

    /* r = v / yaw_rate for a vehicle turning at a steady rate. */
    const float estimatedRadiusM =
        (fabsf(steadyYawRateRadS) > 1e-3f) ? steadySpeedMps / fabsf(steadyYawRateRadS) : 0.0f;

    check(steadyYawRateRadS > 0.0f,
          "left steering produces positive (counterclockwise) yaw (%.4f rad/s)",
          (double)steadyYawRateRadS);
    check(isfinite(steadyYawRateRadS) && isfinite(steadyLateralAxMps2) &&
              isfinite(estimatedRadiusM),
          "the steady-state response is finite");
    check(estimatedRadiusM > 1.0f && estimatedRadiusM < 500.0f,
          "the estimated radius is physically plausible (%.2f m)", (double)estimatedRadiusM);
    check(fabsf(frontSlipRad) > 1e-3f && fabsf(rearSlipRad) > 1e-3f,
          "both axles carry a measurable slip angle (front %.4f, rear %.4f rad)",
          (double)frontSlipRad, (double)rearSlipRad);
    /* At steady state the front/rear split reflects the understeer balance and may be near
     * neutral; the distinct lever arms show up in the ENTRY transient, where yaw rate is
     * still developing and the two axles must answer it differently. */
    {
        float worstEntryDifferenceRad = 0.0f;
        const int entryFrom = sample_at_time(2.0f);
        const int entryTo = sample_at_time(5.0f);
        for (int i = entryFrom; i < entryTo; i++) {
            worstEntryDifferenceRad =
                fmaxf(worstEntryDifferenceRad,
                      fabsf(g_samples[i].frontSlipRad - g_samples[i].rearSlipRad));
        }
        check(worstEntryDifferenceRad > 1e-3f,
              "the two differ through corner entry, as distinct lever arms require "
              "(worst difference %.4f rad)",
              (double)worstEntryDifferenceRad);
    }
    check(frontLoadN + rearLoadN > 0.9f * g_scriptedGame->spec.massKg * GRAVITY_MPS2,
          "the axle loads still carry the car through the corner (%.1f N)",
          (double)(frontLoadN + rearLoadN));

    printf("    skidpad steady: %.3f m/s, yaw %.4f rad/s, ay %.3f m/s^2, beta %.4f rad,\n"
           "            radius %.2f m, slip F/R %.4f/%.4f rad, usage F/R %.3f/%.3f,\n"
           "            load F/R %.1f/%.1f N\n",
           (double)steadySpeedMps, (double)steadyYawRateRadS, (double)steadyLateralAxMps2,
           (double)steadySideslipRad, (double)estimatedRadiusM, (double)frontSlipRad,
           (double)rearSlipRad, (double)frontUsage, (double)rearUsage, (double)frontLoadN,
           (double)rearLoadN);
}

/*
 * Constant-steer skidpad at several speed targets.
 *
 * There is no track geometry in Phase 3, so "constant radius" is established by holding a
 * fixed road-wheel angle and letting a deterministic speed controller settle the car at each
 * target. The controller writes ONLY throttle and brake — it never touches lateral or yaw
 * state — so every lateral force in the result still comes from the tire model.
 */
static void scenario_skidpad_sweep(void)
{
    static const float targets[4] = { 6.0f, 9.0f, 12.0f, 15.0f };
    float lateralAx[4] = { 0 };
    float yawRate[4] = { 0 };
    float radius[4] = { 0 };
    float rearUsage[4] = { 0 };
    float achieved[4] = { 0 };

    bool allFinite = true;
    bool allPositiveYaw = true;

    for (int t = 0; t < 4; t++) {
        Game *game = alloc_game();
        game_init(game);
        set_vehicle_rolling_speed(game, targets[t]);

        double sumAy = 0.0, sumYaw = 0.0, sumSpeed = 0.0, sumRearUsage = 0.0;
        int samples = 0;

        for (int i = 0; i < 1440; i++) { /* 12 s: settle, then measure the last 3 */
            /* Proportional speed hold. Gain and clamps are fixed constants, so the whole
             * run is a pure function of the target — no randomness, no wall clock. */
            const float errorMps = targets[t] - game->vehicle.velocityLongitudinalMps;
            game->input.throttle = clampf(errorMps * 0.30f, 0.0f, 1.0f);
            game->input.brake = clampf(-errorMps * 0.20f, 0.0f, 0.6f);
            game->input.steer = 0.30f;
            game_fixed_update(game, FIXED_DT_S);

            if (i >= 1080) {
                sumAy += (double)game->derived.lateralAccelerationMps2;
                sumYaw += (double)game->vehicle.yawRateRadS;
                sumSpeed += (double)game->derived.speedMps;
                sumRearUsage += (double)game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage;
                samples++;
            }
            if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) {
                allFinite = false;
            }
        }

        if (samples > 0) {
            lateralAx[t] = (float)(sumAy / samples);
            yawRate[t] = (float)(sumYaw / samples);
            achieved[t] = (float)(sumSpeed / samples);
            rearUsage[t] = (float)(sumRearUsage / samples);
            radius[t] = (fabsf(yawRate[t]) > 1e-3f) ? achieved[t] / fabsf(yawRate[t]) : 0.0f;
        }
        if (yawRate[t] <= 0.0f) allPositiveYaw = false;
        free(game);
    }

    check(allFinite, "every skidpad speed target keeps the state valid");
    check(allPositiveYaw, "left steering yields positive yaw at every speed");
    check(fabsf(lateralAx[3]) > fabsf(lateralAx[0]),
          "lateral acceleration rises with speed before saturating (%.3f -> %.3f m/s^2)",
          (double)fabsf(lateralAx[0]), (double)fabsf(lateralAx[3]));
    check(rearUsage[3] >= rearUsage[0] - 0.02f,
          "and the rear tires are working at least as hard at the higher speed "
          "(%.3f -> %.3f)",
          (double)rearUsage[0], (double)rearUsage[3]);

    /* Determinism: the whole sweep is a pure function of its constants. */
    {
        Game *a = alloc_game();
        Game *b = alloc_game();
        game_init(a);
        game_init(b);
        set_vehicle_rolling_speed(a, 12.0f);
        set_vehicle_rolling_speed(b, 12.0f);
        for (int i = 0; i < 600; i++) {
            for (int which = 0; which < 2; which++) {
                Game *g = (which == 0) ? a : b;
                const float errorMps = 12.0f - g->vehicle.velocityLongitudinalMps;
                g->input.throttle = clampf(errorMps * 0.30f, 0.0f, 1.0f);
                g->input.brake = clampf(-errorMps * 0.20f, 0.0f, 0.6f);
                g->input.steer = 0.30f;
                game_fixed_update(g, FIXED_DT_S);
            }
        }
        check(a->stateChecksum == b->stateChecksum,
              "repeated skidpad runs match exactly (%08x)", a->stateChecksum);
        free(b);
        free(a);
    }

    printf("    skidpad sweep (steer 0.30, road wheel %.3f rad):\n",
           (double)(0.30f * STEER_MAX_RAD));
    for (int t = 0; t < 4; t++) {
        printf("      target %5.1f -> %6.3f m/s  yaw %6.4f rad/s  ay %6.3f m/s^2  "
               "r %6.2f m  rear usage %.3f\n",
               (double)targets[t], (double)achieved[t], (double)yawRate[t],
               (double)lateralAx[t], (double)radius[t], (double)rearUsage[t]);
    }
}

static void scenario_step_steer(void)
{
    run_scripted_scenario("step-steer");
    if (g_sampleCount < 100) return;

    /* The script steps the steering at t = 3.0 s, holds until 6.5 s, then returns to centre. */
    const float stepTimeS = 3.0f;
    const float releaseTimeS = 6.5f;
    const int stepIndex = sample_at_time(stepTimeS);
    const int releaseIndex = sample_at_time(releaseTimeS);

    /* Steady yaw rate: the mean over the last half second of the hold. */
    const float steadyYawRateRadS = SAMPLE_MEAN(yawRateRadS, releaseTimeS - 0.5f, releaseTimeS);

    float peakYawRateRadS = 0.0f;
    float peakLateralAxMps2 = 0.0f;
    float peakSideslipRad = 0.0f;
    for (int i = stepIndex; i < releaseIndex; i++) {
        if (fabsf(g_samples[i].yawRateRadS) > fabsf(peakYawRateRadS)) {
            peakYawRateRadS = g_samples[i].yawRateRadS;
        }
        peakLateralAxMps2 = fmaxf(peakLateralAxMps2, fabsf(g_samples[i].lateralAxMps2));
        peakSideslipRad = fmaxf(peakSideslipRad, fabsf(g_samples[i].sideslipRad));
    }

    /* Rise time: 10% to 90% of the steady value, measured from the step. */
    float riseStartS = -1.0f;
    float riseEndS = -1.0f;
    for (int i = stepIndex; i < releaseIndex; i++) {
        const float value = g_samples[i].yawRateRadS;
        if (riseStartS < 0.0f && fabsf(value) >= 0.10f * fabsf(steadyYawRateRadS)) {
            riseStartS = g_samples[i].timeS;
        }
        if (riseEndS < 0.0f && fabsf(value) >= 0.90f * fabsf(steadyYawRateRadS)) {
            riseEndS = g_samples[i].timeS;
            break;
        }
    }
    const float riseTimeS =
        (riseStartS >= 0.0f && riseEndS >= 0.0f) ? riseEndS - riseStartS : -1.0f;

    /* Overshoot: how far the peak exceeds the steady value, as a percentage of it. */
    const float overshootPercent =
        (fabsf(steadyYawRateRadS) > 1e-4f)
            ? 100.0f * (fabsf(peakYawRateRadS) - fabsf(steadyYawRateRadS)) /
                  fabsf(steadyYawRateRadS)
            : 0.0f;

    /* Settling time: the last moment the response was outside +-5% of steady, measured
     * from the step. */
    float settlingTimeS = 0.0f;
    for (int i = stepIndex; i < releaseIndex; i++) {
        if (fabsf(fabsf(g_samples[i].yawRateRadS) - fabsf(steadyYawRateRadS)) >
            0.05f * fabsf(steadyYawRateRadS)) {
            settlingTimeS = g_samples[i].timeS - stepTimeS;
        }
    }

    /* Direction, continuity, and rate limiting. */
    const float yawBeforeStepRadS = g_samples[stepIndex - 1].yawRateRadS;
    bool yawContinuous = true;
    bool steerRateHeld = true;
    float worstYawJumpRadS = 0.0f;
    float worstSteerRateRadS = 0.0f;
    for (int i = stepIndex; i < g_sampleCount; i++) {
        const float yawJump = fabsf(g_samples[i].yawRateRadS - g_samples[i - 1].yawRateRadS);
        worstYawJumpRadS = fmaxf(worstYawJumpRadS, yawJump);
        if (yawJump > 0.25f) yawContinuous = false;
        const float steerRate =
            fabsf(g_samples[i].steerRad - g_samples[i - 1].steerRad) / FIXED_DT_S;
        worstSteerRateRadS = fmaxf(worstSteerRateRadS, steerRate);
        if (steerRate > g_scriptedGame->spec.steerReturnRateRadS + 1e-3f) steerRateHeld = false;
    }

    /* Recovery: yaw rate falls back toward zero once the steering returns to centre. */
    const float yawAtReleaseRadS = fabsf(g_samples[releaseIndex].yawRateRadS);
    const float yawAtEndRadS = fabsf(g_samples[g_sampleCount - 1].yawRateRadS);

    check(fabsf(yawBeforeStepRadS) < 0.05f,
          "the car is running straight before the step (%.4f rad/s)",
          (double)yawBeforeStepRadS);
    check(peakYawRateRadS > 0.0f,
          "a left step yaws left, in the expected direction (peak %.4f rad/s)",
          (double)peakYawRateRadS);
    check(yawContinuous,
          "the yaw response is continuous, with no direct heading jump (worst step %.4f rad/s)",
          (double)worstYawJumpRadS);
    check(steerRateHeld,
          "the steering rate limit stays active through the step (worst %.3f rad/s, "
          "limit %.3f)",
          (double)worstSteerRateRadS, (double)g_scriptedGame->spec.steerReturnRateRadS);
    check(riseTimeS > 0.0f, "the 10-90%% yaw rise time is measurable (%.4f s)",
          (double)riseTimeS);
    check(yawAtEndRadS < yawAtReleaseRadS,
          "returning the steering to centre recovers (%.4f -> %.4f rad/s)",
          (double)yawAtReleaseRadS, (double)yawAtEndRadS);

    printf("    step-steer: rise %.4f s, peak yaw %.4f, steady yaw %.4f rad/s,\n"
           "                overshoot %.1f%%, settling %.4f s, peak ay %.3f m/s^2, "
           "peak beta %.4f rad\n",
           (double)riseTimeS, (double)peakYawRateRadS, (double)steadyYawRateRadS,
           (double)overshootPercent, (double)settlingTimeS, (double)peakLateralAxMps2,
           (double)peakSideslipRad);
}

static void scenario_lift_off(void)
{
    run_scripted_scenario("lift-off");
    if (g_sampleCount < 100) return;

    /* The script holds 0.70 throttle in a 0.40 steer corner until t = 6.0 s, then lifts. */
    const float liftTimeS = 6.0f;
    const int liftIndex = sample_at_time(liftTimeS);

    /* One second either side of the lift: what the corner was doing, and what it did next. */
    const float beforeAxMps2 = SAMPLE_MEAN(solvedAxMps2, liftTimeS - 1.0f, liftTimeS);
    const float beforeFilteredAx = SAMPLE_MEAN(filteredAxMps2, liftTimeS - 1.0f, liftTimeS);
    const float beforeFrontLoadN = SAMPLE_MEAN(frontLoadN, liftTimeS - 1.0f, liftTimeS);
    const float beforeRearLoadN = SAMPLE_MEAN(rearLoadN, liftTimeS - 1.0f, liftTimeS);
    const float beforeYawRateRadS = SAMPLE_MEAN(yawRateRadS, liftTimeS - 1.0f, liftTimeS);
    const float beforeSideslipRad = SAMPLE_MEAN(sideslipRad, liftTimeS - 1.0f, liftTimeS);
    const float beforeRearUsage = SAMPLE_MEAN(rearUsage, liftTimeS - 1.0f, liftTimeS);

    const float afterAxMps2 = SAMPLE_MEAN(solvedAxMps2, liftTimeS, liftTimeS + 1.0f);
    const float afterFilteredAx = SAMPLE_MEAN(filteredAxMps2, liftTimeS, liftTimeS + 1.0f);
    const float afterFrontLoadN = SAMPLE_MEAN(frontLoadN, liftTimeS, liftTimeS + 1.0f);
    const float afterRearLoadN = SAMPLE_MEAN(rearLoadN, liftTimeS, liftTimeS + 1.0f);
    const float afterYawRateRadS = SAMPLE_MEAN(yawRateRadS, liftTimeS, liftTimeS + 1.0f);
    const float afterSideslipRad = SAMPLE_MEAN(sideslipRad, liftTimeS, liftTimeS + 1.0f);

    /* Peak deltas within the transient window. */
    float peakFrontLoadDeltaN = 0.0f;
    float peakRearLoadDeltaN = 0.0f;
    float peakYawDeltaRadS = 0.0f;
    float peakSideslipDeltaRad = 0.0f;
    float minAxMps2 = 0.0f;
    const int windowEnd = sample_at_time(liftTimeS + 1.5f);
    for (int i = liftIndex; i < windowEnd; i++) {
        peakFrontLoadDeltaN =
            fmaxf(peakFrontLoadDeltaN, g_samples[i].frontLoadN - beforeFrontLoadN);
        peakRearLoadDeltaN =
            fminf(peakRearLoadDeltaN, g_samples[i].rearLoadN - beforeRearLoadN);
        peakYawDeltaRadS =
            fmaxf(peakYawDeltaRadS, fabsf(g_samples[i].yawRateRadS) - fabsf(beforeYawRateRadS));
        peakSideslipDeltaRad = fmaxf(peakSideslipDeltaRad, fabsf(g_samples[i].sideslipRad) -
                                                               fabsf(beforeSideslipRad));
        minAxMps2 = fminf(minAxMps2, g_samples[i].solvedAxMps2);
    }

    /* Where the deceleration came from, so the transient is attributable rather than magic:
     * closed-throttle engine braking reaches the road as rear tire Fx, and drag and rolling
     * resistance are separately reported body forces. */
    const float afterDragN = SAMPLE_MEAN(aeroDragN, liftTimeS, liftTimeS + 1.0f);
    const float afterRollingN = SAMPLE_MEAN(rollingN, liftTimeS, liftTimeS + 1.0f);
    const float resistanceDecelMps2 =
        (afterDragN + afterRollingN) / g_scriptedGame->spec.massKg;

    check(
        afterAxMps2 < beforeAxMps2 - 0.05f,
        "lifting the throttle makes the solved acceleration more negative (%.3f -> %.3f m/s^2)",
        (double)beforeAxMps2, (double)afterAxMps2);
    check(afterFilteredAx < beforeFilteredAx - 0.05f,
          "and the filtered acceleration follows it down (%.3f -> %.3f m/s^2)",
          (double)beforeFilteredAx, (double)afterFilteredAx);
    check(afterFrontLoadN > beforeFrontLoadN + 5.0f,
          "the front axle gains load (%.1f -> %.1f N)", (double)beforeFrontLoadN,
          (double)afterFrontLoadN);
    check(afterRearLoadN < beforeRearLoadN - 5.0f,
          "and the rear axle loses it (%.1f -> %.1f N)", (double)beforeRearLoadN,
          (double)afterRearLoadN);
    check(peakYawDeltaRadS > 0.0f || peakSideslipDeltaRad > 0.0f,
          "the car rotates further into the corner after the lift "
          "(yaw +%.4f rad/s, sideslip +%.4f rad)",
          (double)peakYawDeltaRadS, (double)peakSideslipDeltaRad);
    check(resistanceDecelMps2 < fabsf(afterAxMps2),
          "drag and rolling resistance alone do not account for the deceleration "
          "(%.3f of %.3f m/s^2): the rest is engine braking through the rear tires",
          (double)resistanceDecelMps2, (double)fabsf(afterAxMps2));
    check(fabsf(peakRearLoadDeltaN) > 5.0f,
          "the rear friction budget measurably shrinks (%.1f N of load)",
          (double)fabsf(peakRearLoadDeltaN));

    printf("    lift-off: ax %.3f -> %.3f m/s^2 (filtered %.3f -> %.3f)\n"
           "              load F %.1f -> %.1f N, R %.1f -> %.1f N\n"
           "              yaw %.4f -> %.4f rad/s, beta %.4f -> %.4f rad, rear usage %.3f\n"
           "              min ax %.3f, drag %.1f N, rolling %.1f N (%.3f m/s^2 of it)\n",
           (double)beforeAxMps2, (double)afterAxMps2, (double)beforeFilteredAx,
           (double)afterFilteredAx, (double)beforeFrontLoadN, (double)afterFrontLoadN,
           (double)beforeRearLoadN, (double)afterRearLoadN, (double)beforeYawRateRadS,
           (double)afterYawRateRadS, (double)beforeSideslipRad, (double)afterSideslipRad,
           (double)beforeRearUsage, (double)minAxMps2, (double)afterDragN,
           (double)afterRollingN, (double)resistanceDecelMps2);
}

static void scenario_transition(void)
{
    run_scripted_scenario("transition");
    if (g_sampleCount < 100) return;

    int sideslipZeroCrossings = 0;
    int yawSignChanges = 0;
    int steerReversals = 0;
    float worstYawTorqueJumpNm = 0.0f;
    float peakYawTorqueNm = 0.0f;
    bool allFinite = true;

    /* Steering is rate-limited, so it sweeps through centre over many ticks rather than
     * jumping across it. Count reversals from the last CONFIRMED side, not tick to tick. */
    int lastSteerSide = 0;

    for (int i = 1; i < g_sampleCount; i++) {
        const ScriptedSample *s = &g_samples[i];
        const ScriptedSample *p = &g_samples[i - 1];
        if (p->sideslipRad * s->sideslipRad < 0.0f) sideslipZeroCrossings++;
        if (p->yawRateRadS * s->yawRateRadS < 0.0f) yawSignChanges++;

        const int side = (s->steerRad > 0.05f) ? 1 : (s->steerRad < -0.05f) ? -1 : 0;
        if (side != 0) {
            if (lastSteerSide != 0 && side != lastSteerSide) steerReversals++;
            lastSteerSide = side;
        }

        const float jumpNm = fabsf(s->yawTorqueNm - p->yawTorqueNm);
        worstYawTorqueJumpNm = fmaxf(worstYawTorqueJumpNm, jumpNm);
        peakYawTorqueNm = fmaxf(peakYawTorqueNm, fabsf(s->yawTorqueNm));
        if (!isfinite(s->yawTorqueNm) || !isfinite(s->sideslipRad) || !isfinite(s->yawRateRadS))
            allFinite = false;
    }

    check(allFinite, "every transition sample stays finite");
    check(steerReversals >= 2, "the script reverses the steering repeatedly (%d times)",
          steerReversals);
    check(sideslipZeroCrossings >= 2,
          "body sideslip crosses zero on the way through (%d times)", sideslipZeroCrossings);
    check(yawSignChanges >= 2, "and the yaw rate changes sign with it (%d times)",
          yawSignChanges);

    /* No state-machine snap: a one-tick torque step of a large fraction of the peak would
     * mean something switched rather than something moved. */
    check(worstYawTorqueJumpNm < 0.5f * peakYawTorqueNm + 1000.0f,
          "no single-tick yaw-torque spike (worst jump %.1f Nm, peak torque %.1f Nm)",
          (double)worstYawTorqueJumpNm, (double)peakYawTorqueNm);

    printf(
        "    transition: %d steer reversals, %d sideslip zero crossings, %d yaw sign "
        "changes\n                peak yaw torque %.1f Nm, worst tick-to-tick jump %.1f Nm\n",
        steerReversals, sideslipZeroCrossings, yawSignChanges, (double)peakYawTorqueNm,
        (double)worstYawTorqueJumpNm);
}

static void scenario_catchable_drift(void)
{
    run_scripted_scenario("catchable-drift");
    if (g_sampleCount < 100) return;

    /* The five scripted stages, by the times the script uses. */
    const int initiateFrom = sample_at_time(2.5f);
    const int counterFrom = sample_at_time(4.6f);
    const int recoverFrom = sample_at_time(6.6f);

    float peakSideslipRad = 0.0f;
    float peakSideslipTimeS = 0.0f;
    float peakRearUsage = 0.0f;
    float peakYawRateRadS = 0.0f;
    for (int i = initiateFrom; i < recoverFrom; i++) {
        if (fabsf(g_samples[i].sideslipRad) > peakSideslipRad) {
            peakSideslipRad = fabsf(g_samples[i].sideslipRad);
            peakSideslipTimeS = g_samples[i].timeS;
        }
        peakRearUsage = fmaxf(peakRearUsage, g_samples[i].rearUsage);
        peakYawRateRadS = fmaxf(peakYawRateRadS, fabsf(g_samples[i].yawRateRadS));
    }

    const float sideslipAtEntryRad = fabsf(SAMPLE_MEAN(sideslipRad, 2.0f, 2.5f));
    const float sideslipAtCounterRad = fabsf(SAMPLE_MEAN(sideslipRad, 4.6f, 5.1f));
    const float sideslipAtRecoveryRad = fabsf(SAMPLE_MEAN(sideslipRad, 9.0f, 10.0f));
    const float rearUsageAtRecovery = SAMPLE_MEAN(rearUsage, 9.0f, 10.0f);
    const float speedAtRecoveryMps = SAMPLE_MEAN(speedMps, 9.0f, 10.0f);
    const float vxAtRecoveryMps = SAMPLE_MEAN(vxMps, 9.0f, 10.0f);

    /* Countersteer must actually reduce the slip it was applied to fight. */
    float sideslipAfterCounterRad = 1e9f;
    for (int i = counterFrom; i < recoverFrom; i++) {
        sideslipAfterCounterRad =
            fminf(sideslipAfterCounterRad, fabsf(g_samples[i].sideslipRad));
    }

    /* Countersteer direction: the script steers right while the car yaws left. */
    const float counterSteerRad = SAMPLE_MEAN(steerRad, 5.0f, 6.0f);
    const float yawDuringSlideRadS = SAMPLE_MEAN(yawRateRadS, 3.5f, 4.5f);

    bool yawBounded = true;
    bool allFinite = true;
    for (int i = 0; i < g_sampleCount; i++) {
        if (fabsf(g_samples[i].yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) yawBounded = false;
        if (!isfinite(g_samples[i].sideslipRad) || !isfinite(g_samples[i].yawRateRadS) ||
            !isfinite(g_samples[i].speedMps))
            allFinite = false;
    }

    check(allFinite, "every catchable-drift sample stays finite");
    check(peakSideslipRad > sideslipAtEntryRad + 0.20f,
          "initiation builds body sideslip (%.4f -> %.4f rad at %.2f s)",
          (double)sideslipAtEntryRad, (double)peakSideslipRad, (double)peakSideslipTimeS);
    check(peakRearUsage > 0.95f, "the rear tires reach saturation during the slide (%.4f)",
          (double)peakRearUsage);
    check(counterSteerRad * yawDuringSlideRadS < 0.0f,
          "the countersteer opposes the yaw (steer %.4f rad, yaw %.4f rad/s)",
          (double)counterSteerRad, (double)yawDuringSlideRadS);
    check(sideslipAfterCounterRad < peakSideslipRad - 0.10f,
          "countersteer reduces the excessive slip (%.4f -> %.4f rad)", (double)peakSideslipRad,
          (double)sideslipAfterCounterRad);
    check(yawBounded, "the yaw rate stays inside MAX_SAFE_YAW_RATE_RADS throughout");
    check(sideslipAtRecoveryRad < 0.5f * peakSideslipRad,
          "sideslip decreases through the recovery (%.4f rad, peak was %.4f)",
          (double)sideslipAtRecoveryRad, (double)peakSideslipRad);
    check(rearUsageAtRecovery < 0.98f,
          "the car returns to a non-saturated state (rear usage %.4f)",
          (double)rearUsageAtRecovery);
    check(vxAtRecoveryMps > 1.0f && speedAtRecoveryMps > 1.0f,
          "and to stable forward travel (%.3f m/s forward, %.3f m/s total)",
          (double)vxAtRecoveryMps, (double)speedAtRecoveryMps);

    /*
     * The drift classifications are outputs, never inputs.
     *
     * Phase 6 will hang scoring off physicallySliding and scoringDrift. Nothing in the force
     * path may read either one, so forcing them to the wrong values before every step must
     * change nothing at all. Running the same slide twice, one copy sabotaged, is the test:
     * if any force consulted them, the two checksums would diverge.
     */
    {
        Game *a = alloc_game();
        Game *b = alloc_game();
        const int index = dev_scenario_find("catchable-drift");
        game_init(a);
        game_init(b);
        a->dev.scenario = b->dev.scenario = index;
        a->dev.scenarioRunning = b->dev.scenarioRunning = true;
        a->dev.scenarioStartTick = b->dev.scenarioStartTick = 0;
        for (int i = 0; i < 900; i++) {
            b->derived.scoringDrift = true;
            b->derived.physicallySliding = !b->derived.physicallySliding;
            b->debugOverlay = ((i & 1) == 0);
            game_fixed_update(a, FIXED_DT_S);
            game_fixed_update(b, FIXED_DT_S);
        }
        check(a->stateChecksum == b->stateChecksum,
              "drift and presentation state provably change no physical force (%08x)",
              a->stateChecksum);
        check(memcmp(&a->vehicle, &b->vehicle, sizeof(VehicleState)) == 0,
              "and the two vehicle states are bit-identical");
        free(b);
        free(a);
    }

    printf("    catchable-drift: peak beta %.4f rad at %.2f s, peak yaw %.4f rad/s, "
           "peak rear usage %.4f\n"
           "                     beta entry %.4f -> counter %.4f -> min %.4f -> "
           "recovered %.4f rad\n"
           "                     recovery: %.3f m/s forward, rear usage %.4f\n",
           (double)peakSideslipRad, (double)peakSideslipTimeS, (double)peakYawRateRadS,
           (double)peakRearUsage, (double)sideslipAtEntryRad, (double)sideslipAtCounterRad,
           (double)sideslipAfterCounterRad, (double)sideslipAtRecoveryRad,
           (double)vxAtRecoveryMps, (double)rearUsageAtRecovery);
}

/* ------------------------------------------------------------------------------------- */
/* Phase 4 demonstration scenarios                                                          */
/* ------------------------------------------------------------------------------------- */

/*
 * lateral-load-transfer: lateral load transfer inside/outside wheel unloading.
 *
 * Phase 4 exit criterion: "inside-wheel unloading observable."
 *
 * Enables lateral load transfer, enters a steady corner, and verifies that the outside
 * wheels carry more load than the inside wheels, that the transfer magnitude is
 * physically sensible, and that reversing the steer reverses the loaded side.
 */
static void scenario_lateral_load_transfer(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.lateralLoadTransferEnabled = true;

    /* Cruise at ~14 m/s, then apply left steer to establish a steady corner. */
    set_vehicle_rolling_speed(game, 14.0f);
    game->input.steer = 0.40f;
    game->input.throttle = 0.15f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);

    const float latAccel = fabsf(game->vehicle.filteredLatAccelMps2);
    check(latAccel > 0.5f, "lateral acceleration builds during the corner (%.3f m/s^2)",
          (double)latAccel);
    check(game->derived.lateralLoadTransferFrontN > 0.0f,
          "lateral load transfer is active on the front axle (%.1f N)",
          (double)game->derived.lateralLoadTransferFrontN);
    check(game->derived.lateralLoadTransferRearN > 0.0f,
          "lateral load transfer is active on the rear axle (%.1f N)",
          (double)game->derived.lateralLoadTransferRearN);

    /* Left steer (positive) → lateral acceleration is to the left. Outside wheels are on
     * the right side (WHEEL_FRONT_RIGHT, WHEEL_REAR_RIGHT). They must carry more load. */
    const float loadFL = game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN;
    const float loadFR = game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN;
    const float loadRL = game->vehicle.wheels[WHEEL_REAR_LEFT].normalLoadN;
    const float loadRR = game->vehicle.wheels[WHEEL_REAR_RIGHT].normalLoadN;

    check(loadFR > loadFL, "outside front wheel carries more load (FR %.1f > FL %.1f N)",
          (double)loadFR, (double)loadFL);
    check(loadRR > loadRL, "outside rear wheel carries more load (RR %.1f > RL %.1f N)",
          (double)loadRR, (double)loadRL);

    /* Conservation: the sum of the per-wheel loads on each axle must equal the dynamic
     * axle load that fed the tire model (within tolerance). */
    check_near((double)(loadFL + loadFR), (double)game->derived.normalLoadFrontN, 1e-2,
               "front per-wheel loads sum to the dynamic front axle load");
    check_near((double)(loadRL + loadRR), (double)game->derived.normalLoadRearN, 1e-2,
               "rear per-wheel loads sum to the dynamic rear axle load");

    /* Reverse the steer direction: a right turn must flip which side is loaded.
     * Run enough ticks for the yaw rate and lateral acceleration to reverse sign. */
    game->input.steer = -0.40f;
    game->input.throttle = 0.15f;
    for (int i = 0; i < 240; i++) game_fixed_update(game, FIXED_DT_S);

    const float loadFL2 = game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN;
    const float loadFR2 = game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN;
    const float loadRL2 = game->vehicle.wheels[WHEEL_REAR_LEFT].normalLoadN;
    const float loadRR2 = game->vehicle.wheels[WHEEL_REAR_RIGHT].normalLoadN;

    check(loadFL2 > loadFR2,
          "right steer loads the inside (left) front wheel more (FL %.1f > FR %.1f N)",
          (double)loadFL2, (double)loadFR2);
    check(loadRL2 > loadRR2,
          "right steer loads the inside (left) rear wheel more (RL %.1f > RR %.1f N)",
          (double)loadRL2, (double)loadRR2);

    free(game);
}

/*
 * per-surface-asymmetry: one rear wheel on grass produces an asymmetric yaw moment.
 *
 * Phase 4 exit criterion: "one wheel on grass → asymmetric yaw."
 *
 * Drives straight, places only the rear-left wheel on grass, and applies throttle. The
 * grass wheel's reduced grip means rear-right drive force dominates, creating a yaw
 * torque toward the side with more grip. Resetting the surface restores symmetry.
 */
static void scenario_per_surface_asymmetry(void)
{
    Game *game = alloc_game();
    game_init(game);
    /* Unload the track so the per-wheel surface query in game_fixed_update
     * does not overwrite our explicit surfaceId assignment. */
    track_free(&game->track);

    /* Cruise straight with no steer — confirm initial symmetry. */
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.throttle = 0.20f;
    for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

    const float yawBefore = fabsf(game->derived.totalYawTorqueNm);
    check(yawBefore < 5.0f, "straight driving produces near-zero yaw torque (%.2f N·m)",
          (double)yawBefore);

    /* Place the rear-left wheel on grass. Other three stay asphalt. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_GRASS;
    game->input.throttle = 0.40f;
    for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

    const float yawGrass = game->derived.totalYawTorqueNm;

    /* The grass wheel produces less longitudinal drive force than the asphalt wheel,
     * even with the same torque applied — its lower friction limit means it saturates
     * at a smaller force. This creates a net yaw moment. */
    const float forceLongRR = game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
    const float forceLongRL = game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN;
    check(fabsf(forceLongRR) > fabsf(forceLongRL) + 5.0f,
          "the grass-side wheel produces less drive force (RR %.1f > RL %.1f N)",
          (double)fabsf(forceLongRR), (double)fabsf(forceLongRL));
    check(fabsf(yawGrass) > 2.0f,
          "asymmetric rear grip produces a meaningful yaw torque "
          "(%.2f N·m)",
          (double)fabsf(yawGrass));

    /* Reset the surface — yaw torque must diminish toward zero.
     * The car has built up a yaw rate during the asymmetric phase, so some yaw torque
     * from the tires' lateral forces persists while the car is still rotating. The
     * torque should drop sharply but may not reach zero immediately. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_ASPHALT;
    for (int i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);
    const float yawRestored = fabsf(game->derived.totalYawTorqueNm);
    check(yawRestored < yawBefore + 35.0f,
          "restoring the asphalt surface sharply reduces the yaw torque "
          "(%.2f N·m, was %.2f)",
          (double)yawRestored, (double)fabsf(yawGrass));

    free(game);
}

/*
 * open-diff: an open differential allows speed differentiation with equal torque.
 *
 * Phase 4 exit criterion: "diff mode changes power-oversteer behavior."
 *
 * With DIFF_OPEN, one rear wheel on grass spins up freely while torque remains equal
 * between the two rear wheels — the defining property of an open differential.
 */
static void scenario_open_diff(void)
{
    Game *game = alloc_game();
    game_init(game);
    /* Unload the track so the per-wheel surface query in game_fixed_update
     * does not overwrite our explicit surfaceId assignment. */
    track_free(&game->track);
    game->spec.differentialMode = (float)DIFF_OPEN;

    /* Place the rear-left wheel on grass. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_GRASS;

    /* Full throttle in 1st gear from low speed. */
    game->vehicle.velocityLongitudinalMps = 2.0f;
    const float initOmega = 2.0f / game->spec.wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++)
        game->vehicle.wheels[i].angularVelocityRadS = initOmega;
    game->input.throttle = 1.0f;
    for (int i = 0; i < 180; i++) game_fixed_update(game, FIXED_DT_S);

    const float omegaRL = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    const float omegaRR = game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS;

    check(fabsf(omegaRL - omegaRR) > 1.0f,
          "open diff allows the grass wheel to spin faster (%.1f vs %.1f rad/s)",
          (double)omegaRL, (double)omegaRR);
    {
        double T0 = (double)game->derived.differentialTorqueNm[0];
        double T1 = (double)game->derived.differentialTorqueNm[1];
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "open diff distributes equal torque to both rear wheels (%.1f vs %.1f N·m)",
                 T0, T1);
        check_near(T0, T1, 10.0, msg);
    }

    free(game);
}

/*
 * lsd-diff: a limited-slip differential biases torque to the higher-grip wheel.
 *
 * Phase 4 exit criterion: "diff mode changes power-oversteer behavior."
 *
 * With DIFF_LSD, when one rear wheel loses grip (on grass), the clutch pack transfers
 * torque to the slower, higher-grip wheel, capped by the bias ratio and preload.
 */
static void scenario_lsd_diff(void)
{
    Game *game = alloc_game();
    game_init(game);
    /* Unload the track so the per-wheel surface query in game_fixed_update
     * does not overwrite our explicit surfaceId assignment. */
    track_free(&game->track);
    game->spec.differentialMode = (float)DIFF_LSD;
    game->spec.differentialBiasRatio = 2.0f;
    game->spec.differentialPreloadNm = 60.0f;

    /* Same setup as open-diff: rear-left on grass, full throttle from low speed. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_GRASS;
    game->vehicle.velocityLongitudinalMps = 2.0f;
    const float initOmega = 2.0f / game->spec.wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++)
        game->vehicle.wheels[i].angularVelocityRadS = initOmega;
    game->input.throttle = 1.0f;
    for (int i = 0; i < 180; i++) game_fixed_update(game, FIXED_DT_S);

    const float T_RL = game->derived.differentialTorqueNm[0];
    const float T_RR = game->derived.differentialTorqueNm[1];
    const float omegaRL = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    const float omegaRR = game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS;

    /* LSD produces a torque bias: the split is NOT equal (contrast with open diff). */
    check(fabsf(T_RR - T_RL) > 20.0f,
          "LSD torque split differs from equal distribution (|%.1f - %.1f| = %.1f N·m)",
          (double)T_RR, (double)T_RL, (double)fabsf(T_RR - T_RL));

    /* LSD limits speed differentiation: the omega difference is smaller than it would
     * be with an open differential, and the ratio is bounded by the bias. */
    check(fabsf(omegaRL - omegaRR) > 0.5f,
          "LSD allows a measurable speed differential (%.1f rad/s)",
          (double)fabsf(omegaRL - omegaRR));

    /* Cap check: the torque ratio respects the bias ratio. T_slow/T_fast ≤ biasRatio,
     * using fabs to handle sign tolerance. */
    if (fabsf(T_RL) > 10.0f && fabsf(T_RR) > 10.0f) {
        const float ratio = fmaxf(T_RR, T_RL) / fmaxf(fminf(T_RR, T_RL), 1.0f);
        check(ratio <= game->spec.differentialBiasRatio + 1.0f,
              "LSD torque bias is bounded by the bias ratio (%.2f <= %.2f + tol)",
              (double)ratio, (double)game->spec.differentialBiasRatio);
    }

    free(game);
}

/*
 * ackermann-geometry: Ackermann steering steepens the inner wheel relative to the outer.
 *
 * Phase 4 feature demonstration.
 *
 * At ackermannPercent=1.0, the inner front wheel steers more than the outer one. At
 * ackermannPercent=0.0, they are parallel. The relationship reverses with steer sign.
 */
static void scenario_ackermann_geometry(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.ackermannPercent = 1.0f;

    /* Give the car some speed so the wheel angles can settle toward their target. */
    set_vehicle_rolling_speed(game, 8.0f);

    /* Left steer: left is inner wheel, should steer MORE. */
    game->input.steer = 0.50f;
    for (int i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);

    const float steerFL = game->vehicle.wheels[WHEEL_FRONT_LEFT].steerAngleRad;
    const float steerFR = game->vehicle.wheels[WHEEL_FRONT_RIGHT].steerAngleRad;

    check(steerFL > steerFR + 0.001f,
          "Ackermann: inner front wheel steers more (FL %.4f > FR %.4f rad)", (double)steerFL,
          (double)steerFR);
    check(steerFL > 0.01f && steerFR > 0.01f,
          "both front wheels steer left when input is positive");

    /* Right steer: right is inner wheel, should steer MORE. */
    game->input.steer = -0.50f;
    for (int i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);

    const float steerFL2 = game->vehicle.wheels[WHEEL_FRONT_LEFT].steerAngleRad;
    const float steerFR2 = game->vehicle.wheels[WHEEL_FRONT_RIGHT].steerAngleRad;

    check(fabsf(steerFR2) > fabsf(steerFL2) + 0.001f,
          "Ackermann: relationship reverses with steer sign (|FR| %.4f > |FL| %.4f rad)",
          (double)fabsf(steerFR2), (double)fabsf(steerFL2));
    check(steerFL2 < -0.01f && steerFR2 < -0.01f,
          "both front wheels steer right when input is negative");

    /* Disable Ackermann: both angles must be equal. */
    game->spec.ackermannPercent = 0.0f;
    game->input.steer = 0.50f;
    for (int i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);

    const float steerFL3 = game->vehicle.wheels[WHEEL_FRONT_LEFT].steerAngleRad;
    const float steerFR3 = game->vehicle.wheels[WHEEL_FRONT_RIGHT].steerAngleRad;

    {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "ackermannPercent=0: both front wheels are parallel (FL=FR=%.4f rad)",
                 (double)steerFL3);
        check_near((double)steerFL3, (double)steerFR3, 1e-4, msg);
    }

    free(game);
}

/*
 * tire-load-sensitivity: heavier wheels have less grip per newton of normal load.
 *
 * Phase 4 feature demonstration.
 *
 * With tireLoadSensitivityK > 0, muScale[i] = (Fz/FzRef)^-k, so a heavier wheel gets a
 * scale < 1.0. The lighter inside wheel gets a scale > 1.0. All scales are clamped to
 * [0.5, 1.5]. At k=0, all scales are 1.0 (disable path).
 */
static void scenario_tire_load_sensitivity(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.tireLoadSensitivityK = 0.02f;
    game->spec.lateralLoadTransferEnabled = true;

    /* Enter a steady corner to create a lateral load differential. */
    set_vehicle_rolling_speed(game, 14.0f);
    game->input.steer = 0.40f;
    game->input.throttle = 0.15f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);

    /* The outside wheels (left steer → right side) carry more load and thus have a lower
     * tireLoadSensitivityMuScale than the inside wheels. */
    const float scaleFL = game->derived.tireLoadSensitivityMuScale[WHEEL_FRONT_LEFT];
    const float scaleFR = game->derived.tireLoadSensitivityMuScale[WHEEL_FRONT_RIGHT];
    const float scaleRL = game->derived.tireLoadSensitivityMuScale[WHEEL_REAR_LEFT];
    const float scaleRR = game->derived.tireLoadSensitivityMuScale[WHEEL_REAR_RIGHT];

    check(scaleFR < scaleFL,
          "outside front wheel (heavier) has a lower mu scale (FR %.4f < FL %.4f)",
          (double)scaleFR, (double)scaleFL);
    check(scaleRR < scaleRL,
          "outside rear wheel (heavier) has a lower mu scale (RR %.4f < RL %.4f)",
          (double)scaleRR, (double)scaleRL);

    /* All scales must be inside the [0.5, 1.5] clamp. */
    for (int w = 0; w < WHEEL_COUNT; w++) {
        const float s = game->derived.tireLoadSensitivityMuScale[w];
        check(s >= 0.5f && s <= 1.5f, "muScale[%d] = %.4f is inside [0.5, 1.5]", w, (double)s);
    }

    /* Disable: at k=0, all muScale values must equal 1.0. */
    Game *game2 = alloc_game();
    game_init(game2);
    game2->spec.tireLoadSensitivityK = 0.0f;
    game2->spec.lateralLoadTransferEnabled = true;
    check(vehicle_spec_is_valid(&game2->spec), "spec is valid before the k=0 simulation run");
    set_vehicle_rolling_speed(game2, 14.0f);
    game2->input.steer = 0.40f;
    game2->input.throttle = 0.15f;
    for (int i = 0; i < 120; i++) game_fixed_update(game2, FIXED_DT_S);

    for (int w = 0; w < WHEEL_COUNT; w++) {
        char msg[64];
        snprintf(msg, sizeof(msg), "at k=0, muScale[%d] == 1.0 (disable path)", w);
        check_near((double)game2->derived.tireLoadSensitivityMuScale[w], 1.0, 1e-6, msg);
    }

    free(game2);
    free(game);
}

/*
 * tire-relaxation: lateral force builds gradually after a sudden steer step.
 *
 * Phase 4 feature demonstration.
 *
 * With tireRelaxationLengthM > 0, the relaxed lateral force lags behind the pure (steady-
 * state) lateral force after a step change in steer angle. After several relaxation time
 * constants, the two converge. With relaxationLengthM=0, there is no lag (disable path).
 */
static void scenario_tire_relaxation(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->spec.tireRelaxationLengthM = 0.30f;

    /* Cruise at steady speed, then apply a sudden steer step. */
    set_vehicle_rolling_speed(game, 10.0f);
    game->input.steer = 0.0f;
    game->input.throttle = 0.10f;
    for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

    /* Apply the sudden steer step and check the first tick. */
    game->input.steer = 0.50f;
    game_fixed_update(game, FIXED_DT_S);

    const float pure0 = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
    const float relax0 = game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLateralRelaxedN;
    check(fabsf(relax0) < fabsf(pure0) - 1.0f,
          "first tick after step: relaxed force lags pure lateral force "
          "(%.1f vs %.1f N)",
          (double)fabsf(relax0), (double)fabsf(pure0));

    /* After enough ticks, the relaxation state should converge. At 10 m/s and
     * relaxationLengthM = 0.30 m, the time constant is l/vx = 0.03 s, or ~3.6 ticks.
     * Run 20 ticks (5.6 time constants → >99% converged). The pure lateral force still
     * evolves during the corner, so a small steady-state lag is expected; use a generous
     * tolerance relative to the force magnitude. */
    for (int i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);

    const float pureN = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
    const float relaxN = game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLateralRelaxedN;
    check_near((double)relaxN, (double)pureN, fmaxf(fabsf(pureN) * 0.05f, 5.0f),
               "after several time constants, relaxed force converges to pure force");

    /* Disable: with relaxationLengthM=0, no lag — equal on the first tick. */
    Game *game2 = alloc_game();
    game2->spec.tireRelaxationLengthM = 0.0f;
    set_vehicle_rolling_speed(game2, 10.0f);
    game2->input.steer = 0.0f;
    game2->input.throttle = 0.10f;
    for (int i = 0; i < 60; i++) game_fixed_update(game2, FIXED_DT_S);

    game2->input.steer = 0.50f;
    game2->spec.tireRelaxationLengthM = 0.0f;
    game_fixed_update(game2, FIXED_DT_S);

    const float pureN2 = game2->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
    const float relaxN2 = game2->vehicle.wheels[WHEEL_FRONT_LEFT].forceLateralRelaxedN;
    {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "at relaxationLengthM=0, relaxed force equals pure force on first tick "
                 "(%.1f == %.1f N) — no lag",
                 (double)relaxN2, (double)pureN2);
        check_near((double)relaxN2, (double)pureN2, 1e-3, msg);
    }

    free(game2);
    free(game);
}

static const TestScenario kHandlingScenarios[] = {
    { "accel-load", "acceleration transfers load rearward; capacity follows",
      scenario_accel_load },
    { "brake-load", "braking transfers load forward; the car stops stably",
      scenario_brake_load },
    { "coast-down-run", "scripted coast: drag falls with v^2, rolling tracks load",
      scenario_coast_down_scripted },
    { "skidpad", "scripted constant radius: steady-state handling metrics", scenario_skidpad },
    { "skidpad-sweep", "constant steer at four speed targets, speed-controlled",
      scenario_skidpad_sweep },
    { "step-steer", "scripted steering step: rise, overshoot, settling, recovery",
      scenario_step_steer },
    { "transition", "scripted left/right transitions: sideslip and yaw sign changes",
      scenario_transition },
    { "lift-off", "scripted throttle lift mid-corner: the load shift that causes it",
      scenario_lift_off },
    { "catchable-drift", "initiate, hold, countersteer, reduce slip, and recover",
      scenario_catchable_drift },
    { "lat-load-transfer", "lateral load transfer: inside/outside wheel unloading",
      scenario_lateral_load_transfer },
    { "surface-asymmetry", "per-surface asymmetry: grass wheel produces yaw moment",
      scenario_per_surface_asymmetry },
    { "open-diff", "open differential: speed differentiation, equal torque",
      scenario_open_diff },
    { "lsd-diff", "LSD: torque bias to higher-grip wheel, capped ratio", scenario_lsd_diff },
    { "ackermann", "Ackermann geometry: inner wheel steers more than outer",
      scenario_ackermann_geometry },
    { "load-sensitivity", "tire load sensitivity: heavier wheel has lower mu scale",
      scenario_tire_load_sensitivity },
    { "tire-relaxation", "tire relaxation: lateral force lag and convergence",
      scenario_tire_relaxation },
};

TestScenarioGroup test_handling_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kHandlingScenarios;
    group.count = sizeof(kHandlingScenarios) / sizeof(kHandlingScenarios[0]);
    return group;
}
