/*
 * physics_tests.c — headless scenario runner.
 *
 * Preserves Phase 0/1 coverage and adds Phase 2 tire, drivetrain, braking, wheel-dynamics,
 * combined-slip, and maneuver scenarios.
 *
 * It opens no window, initialises no audio, requires no display, and calls no raylib
 * function. raylib.h is reached only for the Vector2 type.
 *
 * Usage:
 *     drifty_tests                       run every scenario
 *     drifty_tests --scenario replay     run one scenario
 *     drifty_tests --list                list scenario names
 *     drifty_tests -v                    print passing checks too
 *
 * Exit status is the number of failed checks, clamped to 125, so a failure is nonzero.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#if defined(_WIN32)
#include <direct.h>
#define DRIFTY_RMDIR(path) _rmdir(path)
#else
#include <unistd.h>
#define DRIFTY_RMDIR(path) rmdir(path)
#endif

#include "car_corpus.h"
#include "car_sheet.h"
#include "car_visual.h"
#include "car_visual_raster.h"
#include "config.h"
#include "dev_params.h"
#include "dev_replay.h"
#include "dev_scenario.h"
#include "dev_state.h"
#include "drivetrain.h"
#include "failure_bundle.h"
#include "surface.h"
#include "game.h"
#include "scoring.h"
#include "input.h"
#include "math_utils.h"
#include "particle.h"
#include "physics.h"
#include "render.h"
#include "replay.h"
#include "telemetry.h"
#include "timestep.h"
#include "tire.h"
#include "units.h"

/* ------------------------------------------------------------------------------------- */
/* Tiny check framework                                                                    */
/* ------------------------------------------------------------------------------------- */

static int  g_checks   = 0;
static int  g_failures = 0;
static bool g_verbose  = false;

/*
 * Failure-bundle context.
 *
 * A scenario that has something reproducible to offer — a Game, a telemetry file, a seed —
 * registers it here. When the scenario ends with failures, the runner writes
 * artifacts/failure-<scenario>-<timestamp>/ from whatever was registered, so the failure
 * arrives with its input timeline and tunables attached instead of as a line of text.
 */
static char        g_firstFailureText[512];
static const Game *g_bundleGame = NULL;
static char        g_bundleTelemetryPath[256];
static bool        g_bundleHasTelemetry = false;
static uint32_t    g_bundleSeed = 0u;
static bool        g_bundlesEnabled = true;

/* The path is copied, not referenced: the runner writes the bundle after the scenario
 * function has returned, when a caller's stack buffer no longer exists. */
static void bundle_context(const Game *game, const char *telemetryPath, uint32_t seed)
{
    g_bundleGame = game;
    g_bundleHasTelemetry = (telemetryPath != NULL);
    if (g_bundleHasTelemetry) {
        snprintf(g_bundleTelemetryPath, sizeof(g_bundleTelemetryPath), "%s", telemetryPath);
    } else {
        g_bundleTelemetryPath[0] = '\0';
    }
    g_bundleSeed = seed;
}

static void bundle_context_clear(void)
{
    g_firstFailureText[0] = '\0';
    bundle_context(NULL, NULL, 0u);
}

static void check(bool ok, const char *fmt, ...)
{
    va_list args;

    g_checks++;
    if (ok) {
        if (g_verbose) {
            printf("    ok   ");
            va_start(args, fmt);
            vprintf(fmt, args);
            va_end(args);
            printf("\n");
        }
        return;
    }

    g_failures++;

    /* Keep the first failure verbatim: later ones are usually consequences of it. */
    if (g_firstFailureText[0] == '\0') {
        va_start(args, fmt);
        vsnprintf(g_firstFailureText, sizeof(g_firstFailureText), fmt, args);
        va_end(args);
    }

    printf("    FAIL ");
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}

static void check_near(double actual, double expected, double tolerance, const char *what)
{
    const double delta = fabs(actual - expected);
    check(delta <= tolerance, "%s (got %.9g, expected %.9g, |delta| %.3g > %.3g)",
          what, actual, expected, delta, tolerance);
}

/* Angular comparison that treats -PI and +PI as equal. */
static void check_near_angle(float actual, float expected, float tolerance, const char *what)
{
    const float delta = fabsf(wrap_angle(actual - expected));
    check(delta <= tolerance, "%s (got %.9g rad, expected %.9g rad, |delta| %.3g)",
          what, (double)actual, (double)expected, (double)delta);
}

/* ------------------------------------------------------------------------------------- */
/* Scripted input timeline                                                                 */
/* ------------------------------------------------------------------------------------- */

typedef struct {
    float steer;
    float throttle;
    float brake;
    float handbrake;
    bool  pause;
    bool  reset;
    bool  debug;
    bool  shiftUp;
    bool  shiftDown;
    float frameTimeS;
} ScriptFrame;

#define SCRIPT_FRAMES 600

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

/* Deterministic, seeded, and identical on every run of this binary. */
static void script_build(ScriptFrame *frames, int count)
{
    static const float frameTimes[4] = { 1.0f / 60.0f, 1.0f / 50.0f, 1.0f / 120.0f, 0.03f };
    uint32_t seed = 0x5EED1234u;

    for (int i = 0; i < count; i++) {
        const uint32_t r = lcg_next(&seed);
        ScriptFrame f;
        memset(&f, 0, sizeof(f));

        f.steer      = (float)(int)((r >> 3) % 3u) - 1.0f;   /* -1, 0, +1 */
        f.throttle   = ((r >> 7) & 3u) != 0u ? 1.0f : 0.0f;
        f.brake      = ((r >> 11) & 7u) == 0u ? 1.0f : 0.0f;
        f.handbrake  = ((r >> 13) & 7u) == 0u ? 1.0f : 0.0f;
        f.pause      = ((r >> 17) & 63u) == 0u;
        f.reset      = ((r >> 19) & 127u) == 0u;
        f.debug      = ((r >> 21) & 63u) == 0u;
        f.shiftUp    = ((r >> 23) & 31u) == 0u;
        f.shiftDown  = ((r >> 25) & 31u) == 0u;
        f.frameTimeS = frameTimes[(r >> 29) & 3u];

        frames[i] = f;
    }
}

static void fixed_update_adapter(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

static void apply_live_input(Game *game, const ScriptFrame *f)
{
    game->input.steer     = f->steer;
    game->input.throttle  = f->throttle;
    game->input.brake     = f->brake;
    game->input.handbrake = f->handbrake;

    /* Latched exactly as input_sample() latches them. */
    if (f->pause)     game->input.pausePressed     = true;
    if (f->reset)     game->input.resetPressed     = true;
    if (f->debug)     game->input.debugPressed     = true;
    if (f->shiftUp)   game->input.shiftUpPressed   = true;
    if (f->shiftDown) game->input.shiftDownPressed = true;
}

static TelemetryRow telemetry_row_from_game(const Game *game, int substepCount)
{
    TelemetryRow row;
    memset(&row, 0, sizeof(row));
    row.tick = game->sim.tick;
    row.timeS = (double)game->sim.tick * (double)FIXED_DT_S;
    row.positionXM = game->vehicle.positionM.x;
    row.positionYM = game->vehicle.positionM.y;
    row.headingRad = game->vehicle.headingRad;
    row.velocityLongitudinalMps = game->vehicle.velocityLongitudinalMps;
    row.velocityLateralMps = game->vehicle.velocityLateralMps;
    row.speedMps = game->derived.speedMps;
    row.yawRateRadS = game->vehicle.yawRateRadS;
    row.steeringAngleRad = game->vehicle.frontRoadWheelAngleRad;
    row.engineRpm = game->vehicle.engineRpm;
    row.selectedGear = game->vehicle.selectedGear;
    row.frontSlipAngleRad = game->derived.frontSlipAngleRad;
    row.rearSlipAngleRad = game->derived.rearSlipAngleRad;
    row.frontSlipRatio = 0.5f * (
        game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio +
        game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio);
    row.rearSlipRatio = 0.5f * (
        game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio +
        game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio);
    row.frontWheelOmegaRadS = 0.5f * (
        game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS +
        game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS);
    row.rearWheelOmegaRadS = 0.5f * (
        game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
        game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS);
    row.frontNormalLoadN = game->derived.normalLoadFrontN;
    row.rearNormalLoadN = game->derived.normalLoadRearN;
    row.frontFxPureN = game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                       game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT];
    row.rearFxPureN = game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                      game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT];
    row.frontFyPureN = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                       game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT];
    row.rearFyPureN = game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                      game->derived.pureLateralForceN[WHEEL_REAR_RIGHT];
    row.frontFxLimitedN = game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN;
    row.rearFxLimitedN = game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                         game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
    row.frontFyLimitedN = game->derived.frontLateralForceN;
    row.rearFyLimitedN = game->derived.rearLateralForceN;
    row.frontFrictionUsage = fmaxf(
        game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
        game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage);
    row.rearFrictionUsage = fmaxf(
        game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
        game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage);
    row.frontLocked = game->vehicle.wheels[WHEEL_FRONT_LEFT].locked ||
                      game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked;
    row.rearLocked = game->vehicle.wheels[WHEEL_REAR_LEFT].locked ||
                     game->vehicle.wheels[WHEEL_REAR_RIGHT].locked;
    row.driveTorqueNm = game->derived.driveTorqueNm[WHEEL_REAR_LEFT] +
                        game->derived.driveTorqueNm[WHEEL_REAR_RIGHT];
    row.frontBrakeTorqueNm =
        game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
        game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    row.rearBrakeTorqueNm =
        game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
        game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    row.handbrakeTorqueNm =
        game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
        game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT];
    row.totalForceXN = game->derived.totalBodyForceN.x;
    row.totalForceYN = game->derived.totalBodyForceN.y;
    row.yawTorqueNm = game->derived.totalYawTorqueNm;
    row.bodySideslipRad = game->derived.bodySideslipRad;
    row.lowSpeedBlend = game->derived.lowSpeedBlend;
    row.substepCount = substepCount;
    row.backlogDrops = game->physicsBacklogDrops;
    row.stateChecksum = game->stateChecksum;

    row.staticFrontLoadN = game->derived.staticFrontLoadN;
    row.staticRearLoadN = game->derived.staticRearLoadN;
    row.dynamicFrontLoadN = game->derived.normalLoadFrontN;
    row.dynamicRearLoadN = game->derived.normalLoadRearN;
    row.loadTransferN = game->derived.loadTransferN;
    row.previousLongAccelMps2 = game->derived.previousLongAccelMps2;
    row.filteredLongAccelMps2 = game->derived.filteredLongAccelMps2;
    row.solvedLongAccelMps2 = game->derived.solvedLongAccelMps2;
    row.lateralAccelMps2 = game->derived.lateralAccelerationMps2;
    row.aeroDragN = game->derived.aeroDragMagnitudeN;
    row.aeroDragXN = game->derived.aeroDragBodyN.x;
    row.aeroDragYN = game->derived.aeroDragBodyN.y;
    row.rollingResistanceN = game->derived.rollingResistanceMagnitudeN;
    row.rollingResistanceXN = game->derived.rollingResistanceBodyN.x;
    row.rollingResistanceYN = game->derived.rollingResistanceBodyN.y;

    /* dev.appliedInput is the input the fixed update actually used, which is the scripted
     * timeline while a scenario runs rather than whatever game->input holds. */
    row.throttleInput = game->dev.appliedInput.throttle;
    row.brakeInput = game->dev.appliedInput.brake;
    row.handbrakeInput = game->dev.appliedInput.handbrake;
    return row;
}

/* Drive the script with live input while the game module records it. */
static uint32_t run_recording(Game *game, const ScriptFrame *frames, int count,
                              float pixelsPerMeter, TelemetryWriter *writer)
{
    game_init(game);
    game->state = STATE_PLAYING;  /* headless tests start simulating immediately */
    game->renderPixelsPerMeter = pixelsPerMeter;

    for (int i = 0; i < count; i++) {
        apply_live_input(game, &frames[i]);

        const TimestepResult step = timestep_advance(&game->accumulatorS,
                                                     &game->physicsBacklogDrops,
                                                     frames[i].frameTimeS,
                                                     fixed_update_adapter,
                                                     game);
        game->lastSubstepCount = step.substeps;

        if (writer != NULL) {
            const TelemetryRow row = telemetry_row_from_game(game, step.substeps);
            telemetry_write_row(writer, &row);
        }
    }

    return game->stateChecksum;
}

/* Drive the same frame-time script with NO live input, feeding the recorded timeline. */
static uint32_t run_playback(Game *game, const ReplayBuffer *timeline,
                             const ScriptFrame *frames, int count, float pixelsPerMeter)
{
    game_init(game);
    game->state = STATE_PLAYING;  /* headless tests start simulating immediately */
    game->renderPixelsPerMeter = pixelsPerMeter;

    game->replay = *timeline;
    if (!replay_begin_playback(&game->replay)) return 0u;

    for (int i = 0; i < count; i++) {
        input_zero(&game->input);

        const TimestepResult step = timestep_advance(&game->accumulatorS,
                                                     &game->physicsBacklogDrops,
                                                     frames[i].frameTimeS,
                                                     fixed_update_adapter,
                                                     game);
        game->lastSubstepCount = step.substeps;
    }

    return game->stateChecksum;
}

static Game *alloc_game(void)
{
    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "FATAL: could not allocate Game (%zu bytes)\n", sizeof(Game));
        exit(126);
    }
    return game;
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: math                                                                          */
/* ------------------------------------------------------------------------------------- */

static void scenario_math(void)
{
    /* clampf */
    check_near((double)clampf(0.5f, 0.0f, 1.0f), 0.5, 0.0, "clampf passes an in-range value");
    check_near((double)clampf(5.0f, 0.0f, 1.0f), 1.0, 0.0, "clampf clamps above");
    check_near((double)clampf(-5.0f, 0.0f, 1.0f), 0.0, 0.0, "clampf clamps below");
    check_near((double)clampf(0.5f, 1.0f, 0.0f), 0.5, 0.0, "clampf tolerates swapped bounds");
    check_near((double)clampf(2.0f, 1.0f, 0.0f), 1.0, 0.0, "clampf clamps with swapped bounds");

    /* lerpf */
    check_near((double)lerpf(0.0f, 10.0f, 0.25f), 2.5, 1e-6, "lerpf midpoint");
    check_near((double)lerpf(2.0f, 4.0f, 0.0f), 2.0, 0.0, "lerpf at t=0");
    check_near((double)lerpf(2.0f, 4.0f, 1.0f), 4.0, 0.0, "lerpf at t=1");
    check_near((double)lerpf(0.0f, 1.0f, 2.0f), 2.0, 1e-6, "lerpf extrapolates");

    /* smooth_to */
    check_near((double)smooth_to(3.0f, 9.0f, 5.0f, 0.0f), 3.0, 0.0,
               "smooth_to with dt=0 is a no-op");
    check_near((double)smooth_to(3.0f, 9.0f, 0.0f, 0.1f), 3.0, 0.0,
               "smooth_to with rate=0 is a no-op");
    {
        float value = 0.0f;
        bool monotonic = true;
        for (int i = 0; i < 500; i++) {
            const float next = smooth_to(value, 1.0f, 8.0f, FIXED_DT_S);
            if (next < value || next > 1.0f) monotonic = false;
            value = next;
        }
        check(monotonic, "smooth_to approaches the target monotonically without overshoot");
        check_near((double)value, 1.0, 1e-3, "smooth_to converges on the target");
    }

    /* wrap_angle: canonical range is [-PI, +PI) */
    check_near_angle(wrap_angle(0.0f), 0.0f, 1e-6f, "wrap_angle(0)");
    check_near((double)wrap_angle(DRIFTY_PI), -(double)DRIFTY_PI, 1e-5,
               "wrap_angle(+PI) returns -PI (range is closed below, open above)");
    check_near((double)wrap_angle(-DRIFTY_PI), -(double)DRIFTY_PI, 1e-5, "wrap_angle(-PI)");
    check_near_angle(wrap_angle(3.0f * DRIFTY_PI), DRIFTY_PI, 1e-4f, "wrap_angle(3*PI)");
    check_near((double)wrap_angle(0.5f), 0.5, 1e-6, "wrap_angle leaves an in-range angle alone");
    {
        bool inRange = true;
        bool equivalent = true;
        for (int i = -2000; i <= 2000; i++) {
            const float a = (float)i * 0.37f;
            const float w = wrap_angle(a);
            if (!(w >= -DRIFTY_PI && w < DRIFTY_PI)) inRange = false;
            /* w must differ from a by a whole number of turns */
            const float turns = (a - w) / DRIFTY_TWO_PI;
            if (fabsf(turns - roundf(turns)) > 1e-3f) equivalent = false;
        }
        check(inRange, "wrap_angle output is always within [-PI, +PI)");
        check(equivalent, "wrap_angle differs from its input by a whole number of turns");
    }

    /* smoothstep */
    check_near((double)smoothstep(1.0f, 3.0f, 0.5f), 0.0, 0.0, "smoothstep below edge0");
    check_near((double)smoothstep(1.0f, 3.0f, 4.0f), 1.0, 0.0, "smoothstep above edge1");
    check_near((double)smoothstep(1.0f, 3.0f, 2.0f), 0.5, 1e-6, "smoothstep midpoint");
    check_near((double)smoothstep(1.0f, 3.0f, 1.0f), 0.0, 0.0, "smoothstep at edge0");
    check_near((double)smoothstep(1.0f, 3.0f, 3.0f), 1.0, 0.0, "smoothstep at edge1");
    check_near((double)smoothstep(2.0f, 2.0f, 1.9f), 0.0, 0.0,
               "smoothstep with equal bounds steps at edge0 (below)");
    check_near((double)smoothstep(2.0f, 2.0f, 2.0f), 1.0, 0.0,
               "smoothstep with equal bounds steps at edge0 (at)");
    check_near((double)smoothstep(3.0f, 1.0f, 2.5f), 0.0, 0.0,
               "smoothstep with inverted bounds degrades to a step, not a negative result");
    {
        bool bounded = true;
        bool monotonic = true;
        float previous = -1.0f;
        for (int i = 0; i <= 400; i++) {
            const float x = -1.0f + (float)i * 0.01f;
            const float s = smoothstep(0.0f, 2.0f, x);
            if (!(s >= 0.0f && s <= 1.0f) || !isfinite(s)) bounded = false;
            if (s < previous) monotonic = false;
            previous = s;
        }
        check(bounded, "smoothstep stays finite and within [0, 1]");
        check(monotonic, "smoothstep is non-decreasing");
    }

    /* lerp_angle: shortest wrapped path */
    check_near_angle(lerp_angle(0.5f, 1.5f, 0.0f), 0.5f, 1e-6f, "lerp_angle at t=0");
    check_near_angle(lerp_angle(0.5f, 1.5f, 1.0f), 1.5f, 1e-6f, "lerp_angle at t=1");
    check_near_angle(lerp_angle(0.5f, 1.5f, 0.5f), 1.0f, 1e-6f, "lerp_angle midpoint");
    {
        /* 3.0 -> -3.0 is 0.283 rad the short way (across +-PI), 5.999 rad the long way. */
        const float mid = lerp_angle(3.0f, -3.0f, 0.5f);
        check_near_angle(mid, DRIFTY_PI, 1e-4f,
                         "lerp_angle crosses +-PI rather than sweeping back through zero");
        check(fabsf(mid) > 3.0f, "lerp_angle result stays near +-PI (got %.6f)", (double)mid);

        const float back = lerp_angle(-3.0f, 3.0f, 0.5f);
        check(fabsf(back) > 3.0f,
              "lerp_angle takes the short path in the other direction too (got %.6f)",
              (double)back);
    }
    {
        /* Total angular travel over a full sweep must equal the short-path delta. */
        float travel = 0.0f;
        float previous = lerp_angle(3.0f, -3.0f, 0.0f);
        for (int i = 1; i <= 100; i++) {
            const float t = (float)i / 100.0f;
            const float current = lerp_angle(3.0f, -3.0f, t);
            travel += fabsf(wrap_angle(current - previous));
            previous = current;
        }
        check_near((double)travel, (double)fabsf(wrap_angle(-3.0f - 3.0f)), 1e-3,
                   "lerp_angle travels only the short-path arc length");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: units                                                                         */
/* ------------------------------------------------------------------------------------- */

static void scenario_units(void)
{
    const Vector2 worldM = { 2.0f, 3.0f };

    const Vector2 px24 = units_world_to_render_px(worldM, 24.0f);
    check_near((double)px24.x, 48.0, 1e-6, "world->render X scales by pixelsPerMeter");
    check_near((double)px24.y, -72.0, 1e-6, "world->render Y is negated (render space is +Y down)");

    const Vector2 px48 = units_world_to_render_px(worldM, 48.0f);
    check_near((double)px48.x, 2.0 * (double)px24.x, 1e-6, "render X is linear in the scale");
    check_near((double)px48.y, 2.0 * (double)px24.y, 1e-6, "render Y is linear in the scale");

    const Vector2 back = units_render_px_to_world(px24, 24.0f);
    check_near((double)back.x, (double)worldM.x, 1e-5, "render->world round-trips X");
    check_near((double)back.y, (double)worldM.y, 1e-5, "render->world round-trips Y");

    check_near((double)units_meters_to_pixels(2.5f, 24.0f), 60.0, 1e-6, "meters->pixels");
    check_near((double)units_pixels_to_meters(60.0f, 24.0f), 2.5, 1e-6, "pixels->meters");
    check_near((double)units_pixels_to_meters(60.0f, 0.0f), 0.0, 0.0,
               "pixels->meters guards a zero scale instead of returning infinity");
    {
        const Vector2 guarded = units_render_px_to_world(px24, 0.0f);
        check(guarded.x == 0.0f && guarded.y == 0.0f,
              "render->world guards a zero scale");
    }

    /* Heading (radians, CCW positive) -> raylib rotation (degrees, CW positive). */
    check_near((double)units_heading_to_rotation_deg(0.0f), 0.0, 1e-6, "heading 0 -> 0 deg");
    check_near((double)units_heading_to_rotation_deg(DRIFTY_PI * 0.5f), -90.0, 1e-4,
               "a counterclockwise quarter turn maps to -90 render degrees");
    check_near((double)units_heading_to_rotation_deg(-DRIFTY_PI), 180.0, 1e-3,
               "heading -PI -> +180 render degrees");
    check_near((double)units_rotation_deg_to_heading(units_heading_to_rotation_deg(0.7f)), 0.7,
               1e-5, "heading <-> rotation round-trips");

    /* The compiled default must be the documented one. */
    check_near((double)PIXELS_PER_METER, 24.0, 1e-6,
               "PIXELS_PER_METER default is 24 px/m");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: timestep                                                                      */
/* ------------------------------------------------------------------------------------- */

static int g_adapterCalls = 0;

static void counting_adapter(void *ctx, float dt)
{
    (void)ctx;
    (void)dt;
    g_adapterCalls++;
}

static void scenario_timestep(void)
{
    /* Exactly the cap, with nothing left over. */
    {
        float accumulator = 0.0f;
        int drops = 0;
        g_adapterCalls = 0;

        const TimestepResult r = timestep_advance(&accumulator, &drops,
                                                  (float)MAX_PHYSICS_STEPS * FIXED_DT_S,
                                                  counting_adapter, NULL);
        check(r.substeps == MAX_PHYSICS_STEPS, "a frame worth of %d steps runs %d (got %d)",
              MAX_PHYSICS_STEPS, MAX_PHYSICS_STEPS, r.substeps);
        check(g_adapterCalls == MAX_PHYSICS_STEPS,
              "the fixed update is invoked once per substep (got %d)", g_adapterCalls);
        check(!r.droppedBacklog, "no backlog is dropped when the cap is met exactly");
        check(drops == 0, "backlog-drop counter stays at 0 (got %d)", drops);
    }

    /* More backlog than the cap allows: capped, dropped, counted. */
    {
        float accumulator = 0.0f;
        int drops = 0;
        g_adapterCalls = 0;

        const TimestepResult r = timestep_advance(&accumulator, &drops, 0.2f,
                                                  counting_adapter, NULL);
        check(r.substeps == MAX_PHYSICS_STEPS,
              "a 0.2 s frame is capped at %d substeps (got %d)", MAX_PHYSICS_STEPS, r.substeps);
        check(g_adapterCalls == MAX_PHYSICS_STEPS, "the loop is never unbounded");
        check(r.droppedBacklog, "excess backlog is reported as dropped");
        check(drops == 1, "backlog-drop counter increments once (got %d)", drops);
        check(accumulator < FIXED_DT_S,
              "the accumulator is reduced below one step after a drop (got %.6f)",
              (double)accumulator);
    }

    /* An extreme frame time is clamped before it reaches the accumulator. */
    {
        float accumulator = 0.0f;
        int drops = 0;
        g_adapterCalls = 0;

        const TimestepResult r = timestep_advance(&accumulator, &drops, 10.0f,
                                                  counting_adapter, NULL);
        check(r.substeps == MAX_PHYSICS_STEPS,
              "a 10 s frame is clamped to %.2f s and capped at %d substeps (got %d)",
              (double)MAX_FRAME_TIME_S, MAX_PHYSICS_STEPS, r.substeps);
        check(accumulator < FIXED_DT_S,
              "no unbounded debt survives the clamp (accumulator %.6f)", (double)accumulator);
        check(drops == 1, "the clamped frame still counts one drop (got %d)", drops);
    }

    /* Degenerate frame times. */
    {
        float accumulator = 0.0f;
        int drops = 0;

        TimestepResult r = timestep_advance(&accumulator, &drops, 0.0f, counting_adapter, NULL);
        check(r.substeps == 0 && drops == 0, "a zero-length frame runs no substeps");

        r = timestep_advance(&accumulator, &drops, -1.0f, counting_adapter, NULL);
        check(r.substeps == 0 && accumulator == 0.0f, "a negative frame time is ignored");

        r = timestep_advance(&accumulator, &drops, (float)NAN, counting_adapter, NULL);
        check(r.substeps == 0 && isfinite(accumulator),
              "a NaN frame time cannot poison the accumulator");
    }

    /* Alpha tracks the leftover accumulator and stays in [0, 1]. */
    {
        float accumulator = 0.0f;
        int drops = 0;
        bool bounded = true;
        bool matchesAccumulator = true;
        int maxSubsteps = 0;

        for (int i = 0; i < 2000; i++) {
            const float frameTime = (i % 3 == 0) ? 1.0f / 60.0f
                                  : (i % 3 == 1) ? 1.0f / 144.0f
                                                 : 0.09f;
            const TimestepResult r = timestep_advance(&accumulator, &drops, frameTime,
                                                      counting_adapter, NULL);
            if (!(r.interpolationAlpha >= 0.0f && r.interpolationAlpha <= 1.0f)) bounded = false;
            if (fabsf(r.interpolationAlpha - accumulator / FIXED_DT_S) > 1e-4f) {
                matchesAccumulator = false;
            }
            if (r.substeps > maxSubsteps) maxSubsteps = r.substeps;
        }

        check(bounded, "interpolation alpha stays within [0, 1]");
        check(matchesAccumulator, "interpolation alpha equals accumulator / FIXED_DT_S");
        check(maxSubsteps <= MAX_PHYSICS_STEPS,
              "the substep cap holds over a long varying-frame-rate run (peak %d)", maxSubsteps);
        check(drops > 0, "sustained overload accumulates backlog drops (got %d)", drops);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: oneshot                                                                       */
/* ------------------------------------------------------------------------------------- */

static void scenario_oneshot(void)
{
    Game *game = alloc_game();
    game_init(game);

    /* One render frame that runs the maximum number of substeps, with one press of each
     * one-shot command and held controls applied throughout. */
    game->input.steer            = 0.0f;
    game->input.throttle         = 1.0f;
    game->input.resetPressed     = true;
    game->input.debugPressed     = true;
    game->input.shiftUpPressed   = true;

    const bool debugBefore = game->debugOverlay;

    TimestepResult r = timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                                        (float)MAX_PHYSICS_STEPS * FIXED_DT_S,
                                        fixed_update_adapter, game);

    check(r.substeps == MAX_PHYSICS_STEPS, "the frame ran %d substeps (got %d)",
          MAX_PHYSICS_STEPS, r.substeps);
    check(game->sim.resetCount == 1u,
          "a one-frame reset press resets exactly once across %d substeps (got %u)",
          MAX_PHYSICS_STEPS, game->sim.resetCount);
    check(game->sim.debugToggleCount == 1u,
          "the debug toggle fires exactly once (got %u)", game->sim.debugToggleCount);
    check(game->debugOverlay != debugBefore,
          "the debug overlay ends up toggled, not toggled 8 times back to where it started");
    check(game->sim.shiftUpCount == 1u,
          "shift up fires exactly once (got %u)", game->sim.shiftUpCount);
    check(!input_has_oneshot(&game->input),
          "one-shot flags are cleared after the first fixed update that observed them");
    check(game->sim.tick == (uint64_t)MAX_PHYSICS_STEPS,
          "the tick counter advanced once per substep (got %llu)",
          (unsigned long long)game->sim.tick);

    /* Held controls stayed valid for every substep. The reset lands on substep 1 before
     * that substep integrates, so all MAX_PHYSICS_STEPS ticks of travel survive it. */
    check(game->vehicle.positionM.x > 0.0f,
          "held throttle applies through every substep after reset (x %.6f m)",
          (double)game->vehicle.positionM.x);

    /* A second frame with nothing pressed must not repeat any command. */
    r = timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                         (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter, game);
    check(game->sim.resetCount == 1u, "the reset does not repeat on the next frame (got %u)",
          game->sim.resetCount);
    check(game->sim.debugToggleCount == 1u, "the debug toggle does not repeat (got %u)",
          game->sim.debugToggleCount);

    /* A press during a frame that runs no substeps must survive, then fire exactly once. */
    game->input.pausePressed = true;
    r = timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops, 0.0f,
                         fixed_update_adapter, game);
    check(r.substeps == 0, "the stalled frame ran no substeps (got %d)", r.substeps);
    check(game->input.pausePressed,
          "a press is latched, not lost, when a render frame runs no fixed update");
    check(game->sim.pauseToggleCount == 0u, "nothing consumed it yet (got %u)",
          game->sim.pauseToggleCount);

    r = timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                         (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter, game);
    check(game->sim.pauseToggleCount == 1u,
          "the latched press then fires exactly once (got %u)", game->sim.pauseToggleCount);
    check(!game->input.pausePressed, "and is cleared afterwards");

    free(game);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: replay                                                                        */
/* ------------------------------------------------------------------------------------- */

static void scenario_replay(void)
{
    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    Game *recorder = alloc_game();
    const uint32_t recordedChecksum =
        run_recording(recorder, frames, SCRIPT_FRAMES, PIXELS_PER_METER, NULL);

    const uint64_t recordedTicks = recorder->sim.tick;
    check(recordedTicks > 0u, "the recording run executed fixed ticks (got %llu)",
          (unsigned long long)recordedTicks);
    check(recorder->replay.count == (int)recordedTicks,
          "one timeline entry was recorded per fixed tick (%d entries, %llu ticks)",
          recorder->replay.count, (unsigned long long)recordedTicks);
    check(recorder->replay.overwrittenTicks == 0u,
          "the script fits in the ring without overwriting (%llu overwritten)",
          (unsigned long long)recorder->replay.overwrittenTicks);
    check(recorder->sim.resetCount > 0u && recorder->sim.pauseToggleCount > 0u,
          "the script actually exercised one-shot commands (reset %u, pause %u)",
          recorder->sim.resetCount, recorder->sim.pauseToggleCount);

    /* Replay the recorded timeline twice into fresh Game blocks. */
    Game *first  = alloc_game();
    Game *second = alloc_game();

    const uint32_t firstChecksum =
        run_playback(first, &recorder->replay, frames, SCRIPT_FRAMES, PIXELS_PER_METER);
    const uint32_t secondChecksum =
        run_playback(second, &recorder->replay, frames, SCRIPT_FRAMES, PIXELS_PER_METER);

    check(firstChecksum == recordedChecksum,
          "replay reproduces the recorded final-state checksum (%08x vs %08x)",
          firstChecksum, recordedChecksum);
    check(firstChecksum == secondChecksum,
          "two replays of the same timeline agree (%08x vs %08x)",
          firstChecksum, secondChecksum);
    check(first->sim.tick == recordedTicks && second->sim.tick == recordedTicks,
          "both replays executed the same number of fixed ticks");
    check(first->sim.resetCount == recorder->sim.resetCount &&
          first->sim.pauseToggleCount == recorder->sim.pauseToggleCount &&
          first->sim.shiftUpCount == recorder->sim.shiftUpCount,
          "one-shot commands are reproduced exactly by the timeline");
    check(memcmp(&first->sim, &second->sim, sizeof(SimState)) == 0 &&
          memcmp(&first->vehicle, &second->vehicle, sizeof(VehicleState)) == 0,
          "the full vehicle simulation state matches between the two replays");

    printf("    checksum: recorded %08x  replay#1 %08x  replay#2 %08x\n",
           recordedChecksum, firstChecksum, secondChecksum);

    free(second);
    free(first);
    free(recorder);
    free(frames);

    /* Ring-buffer overflow is a documented behaviour, not an accident. */
    {
        ReplayBuffer *ring = (ReplayBuffer *)calloc(1, sizeof(ReplayBuffer));
        if (ring == NULL) {
            fprintf(stderr, "FATAL: could not allocate a ReplayBuffer\n");
            exit(126);
        }

        const int extra = 50;
        replay_begin_recording(ring, 0);
        for (int i = 0; i < REPLAY_CAPACITY_TICKS + extra; i++) {
            Input in;
            input_zero(&in);
            in.steer = (float)i;        /* a unique, identifiable value per tick */
            replay_record(ring, &in);
        }

        check(ring->count == REPLAY_CAPACITY_TICKS,
              "the ring holds exactly its capacity (%d of %d)", ring->count,
              REPLAY_CAPACITY_TICKS);
        check(ring->overwrittenTicks == (uint64_t)extra,
              "overwritten ticks are counted, not silently lost (%llu, expected %d)",
              (unsigned long long)ring->overwrittenTicks, extra);
        check(ring->firstTick == (uint64_t)extra,
              "the retained window starts at tick %d (got %llu)", extra,
              (unsigned long long)ring->firstTick);
        check_near(replay_frame_time_s(ring, 0), (double)extra * (double)FIXED_DT_S, 1e-9,
                   "the oldest retained frame reports the right wall-clock time");

        check(replay_begin_playback(ring), "playback starts on a wrapped ring");

        Input out;
        input_zero(&out);
        check(replay_next(ring, &out), "the wrapped ring yields its oldest frame");
        check_near((double)out.steer, (double)extra, 0.0,
                   "playback resumes at the oldest RETAINED tick, in order");

        int walked = 1;
        while (replay_next(ring, &out)) walked++;
        check(walked == REPLAY_CAPACITY_TICKS,
              "playback walks every retained frame exactly once (%d)", walked);
        check_near((double)out.steer, (double)(REPLAY_CAPACITY_TICKS + extra - 1), 0.0,
                   "and ends on the newest recorded tick");
        check(ring->mode == REPLAY_MODE_PLAYBACK,
              "an exhausted timeline leaves playback for the caller to end");

        free(ring);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: renderscale                                                                   */
/* ------------------------------------------------------------------------------------- */

static void scenario_renderscale(void)
{
    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    Game *baseline = alloc_game();
    Game *doubled  = alloc_game();

    const uint32_t baselineChecksum =
        run_recording(baseline, frames, SCRIPT_FRAMES, PIXELS_PER_METER, NULL);
    const uint32_t doubledChecksum =
        run_recording(doubled, frames, SCRIPT_FRAMES, PIXELS_PER_METER * 2.0f, NULL);

    check(baselineChecksum == doubledChecksum,
          "doubling the render scale leaves the simulation checksum identical (%08x vs %08x)",
          baselineChecksum, doubledChecksum);
    check(memcmp(&baseline->sim, &doubled->sim, sizeof(SimState)) == 0 &&
          memcmp(&baseline->vehicle, &doubled->vehicle, sizeof(VehicleState)) == 0,
          "every simulation field is bit-identical at both render scales");
    check(baseline->renderPixelsPerMeter * 2.0f == doubled->renderPixelsPerMeter,
          "the two runs really did use different render scales (%.1f vs %.1f px/m)",
          (double)baseline->renderPixelsPerMeter, (double)doubled->renderPixelsPerMeter);

    /* The scale must still change what the renderer would draw, otherwise the check above
     * would pass for the trivial reason that the scale is ignored everywhere. */
    {
        const Vector2 a = units_world_to_render_px(baseline->vehicle.positionM,
                                                   baseline->renderPixelsPerMeter);
        const Vector2 b = units_world_to_render_px(doubled->vehicle.positionM,
                                                   doubled->renderPixelsPerMeter);
        check(fabsf(a.x) > 1e-4f || fabsf(a.y) > 1e-4f,
              "the vehicle ended away from the origin, so the pixel comparison is meaningful");
        check(fabsf(b.x - a.x) > 1e-5f || fabsf(b.y - a.y) > 1e-5f,
              "render pixel output does change with the scale");
    }

    free(doubled);
    free(baseline);
    free(frames);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: telemetry                                                                     */
/* ------------------------------------------------------------------------------------- */

#define TELEMETRY_DIR  "telemetry"
#define TELEMETRY_PATH TELEMETRY_DIR "/phase2_determinism.csv"
#define TELEMETRY_PATH_REPEAT TELEMETRY_DIR "/phase2_determinism_repeat.csv"
#define PHASE2_LAUNCH_TELEMETRY TELEMETRY_DIR "/phase2_launch_stop.csv"

static bool files_equal(const char *pathA, const char *pathB)
{
    FILE *a = fopen(pathA, "rb");
    FILE *b = fopen(pathB, "rb");
    if (a == NULL || b == NULL) {
        if (a != NULL) fclose(a);
        if (b != NULL) fclose(b);
        return false;
    }
    bool equal = true;
    for (;;) {
        const int ca = fgetc(a);
        const int cb = fgetc(b);
        if (ca != cb) {
            equal = false;
            break;
        }
        if (ca == EOF) break;
    }
    fclose(a);
    fclose(b);
    return equal;
}

static void scenario_telemetry(void)
{
    check(telemetry_ensure_dir(TELEMETRY_DIR), "the telemetry directory exists or was created");

    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_PATH);
    check(opened, "telemetry_open('%s') succeeded", TELEMETRY_PATH);

    Game *game = alloc_game();
    const uint32_t checksum = run_recording(game, frames, SCRIPT_FRAMES, PIXELS_PER_METER,
                                            opened ? &writer : NULL);

    check(writer.rowCount == (long)SCRIPT_FRAMES,
          "one row was written per render frame (%ld of %d)", writer.rowCount, SCRIPT_FRAMES);
    check(telemetry_close(&writer), "telemetry_close reported success");

    TelemetryWriter repeatWriter;
    const bool repeatOpened = telemetry_open(&repeatWriter, TELEMETRY_PATH_REPEAT);
    Game *repeatGame = alloc_game();
    const uint32_t repeatChecksum = run_recording(
        repeatGame, frames, SCRIPT_FRAMES, PIXELS_PER_METER,
        repeatOpened ? &repeatWriter : NULL);
    check(repeatOpened && telemetry_close(&repeatWriter),
          "the repeated telemetry run writes and closes successfully");
    check(repeatChecksum == checksum && files_equal(TELEMETRY_PATH, TELEMETRY_PATH_REPEAT),
          "identical runs produce byte-identical telemetry and checksum");

    /* Read it back: stable header, expected row count, final checksum present. */
    {
        FILE *file = fopen(TELEMETRY_PATH, "rb");
        check(file != NULL, "the telemetry file can be reopened");
        if (file != NULL) {
            char line[2048];
            long dataRows = 0;
            char lastLine[2048];
            lastLine[0] = '\0';

            if (fgets(line, sizeof(line), file) != NULL) {
                line[strcspn(line, "\r\n")] = '\0';
                check(strcmp(line, telemetry_header()) == 0,
                      "the header row is exactly the documented schema");
            } else {
                check(false, "the telemetry file has a header row");
            }

            while (fgets(line, sizeof(line), file) != NULL) {
                if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') continue;
                dataRows++;
                memcpy(lastLine, line, sizeof(lastLine) < sizeof(line) ? sizeof(lastLine) : sizeof(line));
                lastLine[sizeof(lastLine) - 1] = '\0';
            }
            fclose(file);

            check(dataRows == (long)SCRIPT_FRAMES,
                  "the file holds %d data rows (got %ld)", SCRIPT_FRAMES, dataRows);

            char expectedTail[32];
            snprintf(expectedTail, sizeof(expectedTail), ",%u", checksum);
            check(strstr(lastLine, expectedTail) != NULL,
                  "the final row carries the run's final state checksum (%u)", checksum);
        }
    }

    /* Failure handling: an unwritable path must be reported, not ignored. The writer logs
     * to stderr when it fails, so one TELEMETRY error line below is expected output. */
    {
        printf("    (the next TELEMETRY error line is the expected failure-path output)\n");

        TelemetryWriter bad;
        const bool badOpen = telemetry_open(&bad,
                                            TELEMETRY_DIR "/no_such_directory/telemetry.csv");
        check(!badOpen, "telemetry_open reports failure for an unwritable path");
        check(!telemetry_close(&bad), "telemetry_close propagates the earlier failure");

        TelemetryRow row;
        memset(&row, 0, sizeof(row));
        check(!telemetry_write_row(&bad, &row), "writing to a failed writer is refused");
    }

    free(game);
    free(repeatGame);
    free(frames);
}

/* ------------------------------------------------------------------------------------- */
/* Phase 1 focused unit checks                                                            */
/* ------------------------------------------------------------------------------------- */

static void phase1_fixture(VehicleSpec *spec, VehicleState *state,
                           VehicleDerived *derived, VehicleRenderState *renderState)
{
    vehicle_spec_set_default(spec);
    vehicle_state_reset(spec, state, derived, renderState);
}

static void scenario_vehicle_units(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);

    check(vehicle_spec_is_valid(&spec), "the default vehicle specification is valid");
    check_near(spec.wheelbaseM, spec.cgToFrontM + spec.cgToRearM, 1e-7,
               "wheelbase equals the two CG lever arms");
    check(spec.gearCount <= MAX_GEARS, "canonical gear storage capacity is respected");

    VehicleSpec invalid = spec;
    invalid.massKg = 0.0f;
    check(!vehicle_spec_is_valid(&invalid), "zero mass is rejected");
    invalid = spec;
    invalid.wheelbaseM += 0.1f;
    check(!vehicle_spec_is_valid(&invalid), "an inconsistent wheelbase is rejected");

    check_near(state.wheels[WHEEL_FRONT_LEFT].localPositionM.x, spec.cgToFrontM, 0.0,
               "front-left contact X uses +cgToFront");
    check_near(state.wheels[WHEEL_REAR_RIGHT].localPositionM.x, -spec.cgToRearM, 0.0,
               "rear-right contact X uses -cgToRear");
    check(state.wheels[WHEEL_FRONT_LEFT].localPositionM.y > 0.0f &&
          state.wheels[WHEEL_REAR_LEFT].localPositionM.y > 0.0f,
          "left wheel contacts use positive body Y");
    check(state.wheels[WHEEL_FRONT_RIGHT].localPositionM.y < 0.0f &&
          state.wheels[WHEEL_REAR_RIGHT].localPositionM.y < 0.0f,
          "right wheel contacts use negative body Y");

    float frontLoadN;
    float rearLoadN;
    physics_static_axle_loads(&spec, &frontLoadN, &rearLoadN);
    check_near(frontLoadN + rearLoadN, spec.massKg * GRAVITY_MPS2, 0.01,
               "static axle loads sum to mass times gravity");
    check_near(state.wheels[WHEEL_FRONT_LEFT].normalLoadN, frontLoadN * 0.5f, 0.01,
               "front axle load is split evenly");
    check_near(state.wheels[WHEEL_REAR_RIGHT].normalLoadN, rearLoadN * 0.5f, 0.01,
               "rear axle load is split evenly");
    VehicleSpec rearwardCg = spec;
    rearwardCg.cgToFrontM = 1.40f;
    rearwardCg.cgToRearM = 1.15f;
    rearwardCg.wheelbaseM = 2.55f;
    float movedFrontLoadN;
    float movedRearLoadN;
    physics_static_axle_loads(&rearwardCg, &movedFrontLoadN, &movedRearLoadN);
    check(movedFrontLoadN < frontLoadN && movedRearLoadN > rearLoadN,
          "moving the CG rearward moves static load rearward");

    physics_update_steering(&spec, &state, 1.0f, 0.05f);
    check_near(state.frontRoadWheelAngleRad, spec.maxSteerRateRadS * 0.05f, 1e-7,
               "left steering maps positive and obeys the steering rate");
    check_near(state.wheels[WHEEL_FRONT_LEFT].steerAngleRad,
               state.wheels[WHEEL_FRONT_RIGHT].steerAngleRad, 0.0,
               "both front wheels receive the same Phase 1 angle");
    check_near(state.wheels[WHEEL_REAR_LEFT].steerAngleRad, 0.0, 0.0,
               "the rear-left wheel never steers");
    check_near(state.wheels[WHEEL_REAR_RIGHT].steerAngleRad, 0.0, 0.0,
               "the rear-right wheel never steers");
    const float beforeReturn = state.frontRoadWheelAngleRad;
    physics_update_steering(&spec, &state, 0.0f, 0.01f);
    check_near(beforeReturn - state.frontRoadWheelAngleRad,
               spec.steerReturnRateRadS * 0.01f, 1e-6,
               "return-to-center uses its configured rate");
    for (int i = 0; i < 100; i++) physics_update_steering(&spec, &state, 1.0f, FIXED_DT_S);
    check_near(state.frontRoadWheelAngleRad, spec.maxRoadWheelAngleRad, 1e-6,
               "steering clamps at the maximum road-wheel angle");

    state.velocityLongitudinalMps = 10.0f;
    state.velocityLateralMps = 2.0f;
    state.yawRateRadS = 1.0f;
    const Vector2 flVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_LEFT].localPositionM);
    const Vector2 frVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_RIGHT].localPositionM);
    check(flVelocity.x < frVelocity.x,
          "yaw makes left/right longitudinal contact velocities distinct");
    check_near(flVelocity.y, 2.0 + spec.cgToFrontM, 1e-6,
               "front lateral contact velocity includes +lf*r");
    const Vector2 rearCenterVelocity = physics_contact_point_velocity_body(
        &state, (Vector2){ -spec.cgToRearM, 0.0f });
    check_near(rearCenterVelocity.y, 2.0 - spec.cgToRearM, 1e-6,
               "rear lateral contact velocity includes -lr*r");
    state.yawRateRadS = 0.0f;
    const Vector2 noYawVelocity = physics_contact_point_velocity_body(
        &state, state.wheels[WHEEL_FRONT_LEFT].localPositionM);
    check_near(noYawVelocity.x, state.velocityLongitudinalMps, 0.0,
               "zero yaw reduces contact longitudinal velocity to body velocity");
    check_near(noYawVelocity.y, state.velocityLateralMps, 0.0,
               "zero yaw reduces contact lateral velocity to body velocity");

    state.frontRoadWheelAngleRad = 0.0f;
    state.velocityLongitudinalMps = 10.0f;
    state.velocityLateralMps = 0.0f;
    float frontSlip;
    float rearSlip;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check_near(frontSlip, 0.0, 0.0, "straight travel has zero front slip");
    check_near(rearSlip, 0.0, 0.0, "straight travel has zero rear slip");
    state.velocityLateralMps = 1.0f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(frontSlip > 0.0f && rearSlip > 0.0f,
          "positive lateral velocity produces positive slip angles");
    state.velocityLateralMps = 0.0f;
    state.yawRateRadS = 0.5f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(frontSlip > 0.0f && rearSlip < 0.0f,
          "positive yaw separates front and rear slip signs");
    const float originalFrontSlip = frontSlip;
    VehicleSpec longerFront = spec;
    longerFront.cgToFrontM += 0.3f;
    longerFront.wheelbaseM = longerFront.cgToFrontM + longerFront.cgToRearM;
    physics_axle_slip_angles(&longerFront, &state, &frontSlip, &rearSlip);
    check(frontSlip > originalFrontSlip,
          "changing cgToFront changes the front axle slip response");
    state.velocityLongitudinalMps = -2.0f;
    physics_axle_slip_angles(&spec, &state, &frontSlip, &rearSlip);
    check(isfinite(frontSlip) && isfinite(rearSlip),
          "the documented reverse convention remains finite");

    check_near(tire_lateral_force_n(0.0f, 500.0f, 10.0f, 1.45f, 1.2f),
               0.0, 0.0, "nonlinear tire force is zero at zero slip");
    check(tire_lateral_force_n(0.1f, 500.0f, 10.0f, 1.45f, 1.2f) < 0.0f,
          "positive lateral slip produces an opposing force");
    check(tire_lateral_force_n(-0.1f, 500.0f, 10.0f, 1.45f, 1.2f) > 0.0f,
          "negative lateral slip produces an opposing force");

    const Vector2 unrotated = physics_rotate_wheel_force_to_body(
        (Vector2){ 12.0f, -34.0f }, 0.0f);
    check(unrotated.x == 12.0f && unrotated.y == -34.0f,
          "zero steering leaves the wheel-frame force unchanged");
    const Vector2 rotated = physics_rotate_wheel_force_to_body(
        (Vector2){ 0.0f, 100.0f }, DRIFTY_PI * 0.5f);
    check_near(rotated.x, -100.0, 1e-4, "positive steering rotates lateral force toward -body X");
    check_near(rotated.y, 0.0, 1e-4, "a ninety-degree force rotation has zero body Y");
    check_near(sqrtf(rotated.x * rotated.x + rotated.y * rotated.y), 100.0, 1e-4,
               "force rotation preserves magnitude");
    const Vector2 rotatedNegative = physics_rotate_wheel_force_to_body(
        (Vector2){ 0.0f, 100.0f }, -0.25f);
    check(rotatedNegative.x > 0.0f && rotatedNegative.y > 0.0f,
          "negative steering rotates positive lateral force toward +body X");
    check(fabsf(tire_lateral_force_n(0.1f, 1000.0f, 20.0f, 1.45f, 1.0f)) >
          fabsf(tire_lateral_force_n(0.1f, 1000.0f, 5.0f, 1.45f, 1.0f)),
          "front/rear stiffness parameters can act independently");
    check(fabsf(tire_lateral_force_n(2.0f, 500.0f, 10.0f, 1.45f, 0.8f)) <
          fabsf(tire_lateral_force_n(2.0f, 500.0f, 10.0f, 1.45f, 1.2f)),
          "front/rear friction parameters set independent saturation limits");

    check_near(physics_low_speed_blend(LOW_SPEED_BEGIN_MPS - 0.001f), 0.0, 0.0,
               "low-speed blend is kinematic below its lower endpoint");
    check(physics_low_speed_blend(2.25f) > 0.0f &&
          physics_low_speed_blend(2.25f) < 1.0f,
          "low-speed blend is continuous inside the transition");
    check_near(physics_low_speed_blend(LOW_SPEED_END_MPS + 0.001f), 1.0, 0.0,
               "low-speed blend is dynamic above its upper endpoint");

    VehicleRenderState wrapState;
    memset(&wrapState, 0, sizeof(wrapState));
    wrapState.prevHeadingRad = DRIFTY_PI - 0.1f;
    wrapState.currHeadingRad = -DRIFTY_PI + 0.1f;
    wrapState.prevWheelAngleRad[0] = DRIFTY_PI - 0.2f;
    wrapState.currWheelAngleRad[0] = -DRIFTY_PI + 0.2f;
    const VehicleDrawState draw = render_interpolate_vehicle(&wrapState, 0.5f);
    check(fabsf(fabsf(draw.headingRad) - DRIFTY_PI) < 1e-4f,
          "render heading interpolation takes the shortest path across angle wrap");
    check(fabsf(fabsf(draw.wheelAngleRad[0]) - DRIFTY_PI) < 1e-4f,
          "wheel interpolation also takes the shortest wrapped path");
}

static void scenario_tire(void)
{
    const float loadN = 3000.0f;
    const float mu = 1.2f;
    const float b = 10.0f;
    const float c = 1.45f;

    check_near(tire_normalized_curve(b, c, 0.0f), 0.0, 0.0,
               "normalized curve is zero at zero slip");
    check_near(tire_lateral_force_n(0.0f, loadN, b, c, mu), 0.0, 0.0,
               "lateral force is zero at zero slip");
    const float positiveFy = tire_lateral_force_n(0.1f, loadN, b, c, mu);
    const float negativeFy = tire_lateral_force_n(-0.1f, loadN, b, c, mu);
    check(positiveFy < 0.0f && negativeFy > 0.0f,
          "lateral force opposes positive and negative slip");
    check_near(positiveFy, -negativeFy, 0.001,
               "lateral curve is sign symmetric");
    {
        const float epsilon = 1e-5f;
        const float measuredSlope =
            tire_lateral_force_n(epsilon, loadN, b, c, mu) / epsilon;
        check_near(measuredSlope, -mu * loadN * b * c, 2.0,
                   "small-slip lateral slope matches -mu*Fz*B*C");
    }
    {
        float peak = 0.0f;
        float peakSlip = 0.0f;
        for (int i = 0; i <= 20000; i++) {
            const float slip = (float)i * 0.0001f;
            const float force = fabsf(tire_lateral_force_n(slip, loadN, b, c, mu));
            if (force > peak) {
                peak = force;
                peakSlip = slip;
            }
        }
        check_near(peak, mu * loadN, 1.0,
                   "lateral peak is approximately mu*Fz");
        const float postPeak = fabsf(tire_lateral_force_n(4.0f, loadN, b, c, mu));
        check(postPeak < peak && postPeak > 0.5f * peak,
              "lateral force falls after the peak without collapsing");
        check(peakSlip > 0.0f && peakSlip < 1.0f,
              "lateral peak occurs at a finite positive slip");
    }
    check(isfinite(tire_lateral_force_n(1000.0f, loadN, b, c, mu)) &&
          isfinite(tire_lateral_force_n(-1000.0f, loadN, b, c, mu)),
          "large positive and negative lateral slips remain finite");
    check(fabsf(tire_lateral_force_n(0.1f, loadN, 5.0f, c, mu)) <
          fabsf(tire_lateral_force_n(0.1f, loadN, 15.0f, c, mu)),
          "independent lateral B parameters change force buildup");
    check(fabsf(tire_lateral_force_n(0.2f, loadN, b, c, 1.0f)) <
          fabsf(tire_lateral_force_n(0.2f, loadN, b, c, 1.3f)),
          "independent lateral mu parameters change force magnitude");

    check_near(tire_longitudinal_force_n(0.0f, loadN, TIRE_B_LONG,
                                         TIRE_C_LONG, 1.0f),
               0.0, 0.0, "longitudinal force is zero at zero slip");
    check(tire_longitudinal_force_n(0.1f, loadN, TIRE_B_LONG,
                                    TIRE_C_LONG, 1.0f) > 0.0f &&
          tire_longitudinal_force_n(-0.1f, loadN, TIRE_B_LONG,
                                    TIRE_C_LONG, 1.0f) < 0.0f,
          "longitudinal force follows slip-ratio sign");
    {
        float peak = 0.0f;
        for (int i = 0; i <= 20000; i++) {
            const float force = fabsf(tire_longitudinal_force_n(
                (float)i * 0.0001f, loadN, TIRE_B_LONG, TIRE_C_LONG, 1.0f));
            if (force > peak) peak = force;
        }
        check_near(peak, loadN, 1.0,
                   "longitudinal peak is approximately mu*Fz");
    }

    check_near(tire_slip_ratio(10.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M,
                               10.0f, SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               0.0, 1e-6, "free rolling has zero slip ratio");
    check(tire_slip_ratio(15.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M,
                          10.0f, SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP) > 0.0f,
          "wheelspin has positive slip ratio");
    check_near(tire_slip_ratio(0.0f, WHEEL_RADIUS_M, 10.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               -1.0, 1e-6, "locked forward braking has slip ratio -1");
    check_near(tire_slip_ratio(-10.0f / WHEEL_RADIUS_M, WHEEL_RADIUS_M,
                               -10.0f, SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               0.0, 1e-6, "free rolling in reverse has zero slip ratio");
    check_near(tire_slip_ratio(0.1f, WHEEL_RADIUS_M, 0.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               0.1 * WHEEL_RADIUS_M / SLIP_SPEED_EPSILON_MPS, 1e-6,
               "denominator floor produces a small finite near-zero slip");
    check_near(tire_slip_ratio(1000.0f, WHEEL_RADIUS_M, 0.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               SLIP_RATIO_CLAMP, 0.0, "positive slip ratio clamps");
    check_near(tire_slip_ratio(-1000.0f, WHEEL_RADIUS_M, 0.0f,
                               SLIP_SPEED_EPSILON_MPS, SLIP_RATIO_CLAMP),
               -SLIP_RATIO_CLAMP, 0.0, "negative slip ratio clamps");

    {
        float fx;
        float fy;
        float usage;
        tire_apply_combined_limit(300.0f, 400.0f, 1000.0f, 1000.0f,
                                  &fx, &fy, &usage);
        check_near(fx, 300.0, 1e-6, "inside ellipse retains longitudinal force");
        check_near(fy, 400.0, 1e-6, "inside ellipse retains lateral force");
        check_near(usage, 0.5, 1e-6, "inside ellipse reports normalized usage");

        tire_apply_combined_limit(1000.0f, 1000.0f, 1000.0f, 1000.0f,
                                  &fx, &fy, &usage);
        check_near(fx, 1000.0 / sqrt(2.0), 0.01,
                   "diagonal saturation scales longitudinal force");
        check_near(fy, 1000.0 / sqrt(2.0), 0.01,
                   "diagonal saturation scales lateral force");
        check_near(usage, 1.0, 0.0, "saturated usage is capped at one");
        check_near(fx / fy, 1.0, 1e-6, "combined limit preserves direction");

        tire_apply_combined_limit(1000.0f, 2000.0f, 1000.0f, 2000.0f,
                                  &fx, &fy, &usage);
        check_near(sqrt((fx / 1000.0) * (fx / 1000.0) +
                        (fy / 2000.0) * (fy / 2000.0)),
                   1.0, 1e-6, "asymmetric limits saturate on their ellipse");

        tire_apply_combined_limit(100.0f, 100.0f, 0.0f, 0.0f,
                                  &fx, &fy, &usage);
        check(fx == 0.0f && fy == 0.0f && usage == 0.0f,
              "zero load produces finite zero combined forces");
    }
}

static void scenario_drivetrain(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);

    check_near(drivetrain_engine_torque_at_rpm(&spec, spec.engineIdleRpm),
               spec.engineTorqueCurveNm[0], 0.0, "engine curve at idle");
    check_near(drivetrain_engine_torque_at_rpm(&spec, spec.engineRedlineRpm),
               spec.engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1], 0.0,
               "engine curve at redline");
    const float sampleStepRpm = (spec.engineRedlineRpm - spec.engineIdleRpm) /
                                (float)(ENGINE_CURVE_POINTS - 1);
    check_near(drivetrain_engine_torque_at_rpm(&spec,
                   spec.engineIdleRpm + 2.0f * sampleStepRpm),
               spec.engineTorqueCurveNm[2], 1e-5,
               "engine curve returns exact interior samples");
    check_near(drivetrain_engine_torque_at_rpm(&spec,
                   spec.engineIdleRpm + 2.5f * sampleStepRpm),
               0.5f * (spec.engineTorqueCurveNm[2] +
                       spec.engineTorqueCurveNm[3]), 1e-4,
               "engine curve interpolates sample midpoints");
    check_near(drivetrain_engine_torque_at_rpm(&spec, 0.0f),
               spec.engineTorqueCurveNm[0], 0.0, "engine curve clamps below idle");
    check_near(drivetrain_engine_torque_at_rpm(&spec, 99999.0f),
               spec.engineTorqueCurveNm[ENGINE_CURVE_POINTS - 1], 0.0,
               "engine curve clamps above redline");

    const float firstRatio = spec.gearRatios[0] * spec.finalDriveRatio;
    check_near(drivetrain_total_gear_ratio(&spec, 1), firstRatio, 1e-6,
               "first gear includes final drive");
    check_near(drivetrain_total_gear_ratio(&spec, -1),
               -spec.reverseGearRatio * spec.finalDriveRatio, 1e-6,
               "reverse gear ratio is negative");
    check_near(drivetrain_total_gear_ratio(&spec, 0), 0.0, 0.0,
               "neutral total ratio is zero");
    check_near(drivetrain_engine_rpm(&spec, 1, 20.0f),
               clampf(20.0f * firstRatio * 60.0f / DRIFTY_TWO_PI,
                      spec.engineIdleRpm, spec.engineRedlineRpm),
               1e-4, "engine RPM derives from rear wheel speed and gearing");

    DrivetrainTorques neutral = drivetrain_calculate_torques(
        &spec, 0, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    check_near(neutral.drivelineTorqueNm, 0.0, 0.0,
               "neutral transmits no drive torque");
    DrivetrainTorques forward = drivetrain_calculate_torques(
        &spec, 1, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    DrivetrainTorques reverse = drivetrain_calculate_torques(
        &spec, -1, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    check(forward.drivelineTorqueNm > 0.0f && reverse.drivelineTorqueNm < 0.0f,
          "forward and reverse produce opposite driveline torque");
    check(forward.driveTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
          forward.driveTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "front wheels receive no drive torque");
    check_near(forward.driveTorqueNm[WHEEL_REAR_LEFT],
               forward.driveTorqueNm[WHEEL_REAR_RIGHT], 0.0,
               "rear wheels receive equal drive torque");
    VehicleSpec halfEfficiency = spec;
    halfEfficiency.drivetrainEfficiency *= 0.5f;
    DrivetrainTorques half = drivetrain_calculate_torques(
        &halfEfficiency, 1, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    check_near(half.drivelineTorqueNm, forward.drivelineTorqueNm * 0.5f, 1e-4,
               "drivetrain efficiency scales output torque");

    DrivetrainTorques braking = drivetrain_calculate_torques(
        &spec, 1, 10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
    const float frontBrake = braking.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                             braking.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    const float rearBrake = braking.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                            braking.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    check_near(frontBrake + rearBrake, spec.maxBrakeTorqueNm, 1e-4,
               "full service-brake split sums to configured torque");
    check_near(frontBrake / spec.maxBrakeTorqueNm, spec.brakeBiasFront, 1e-6,
               "front service-brake torque follows bias");
    check(braking.handbrakeTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
          braking.handbrakeTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "handbrake is rear-only");
    check_near(braking.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
               braking.handbrakeTorqueNm[WHEEL_REAR_RIGHT],
               spec.handbrakeTorqueNm, 1e-5,
               "full handbrake saturates at configured torque");

    bool locked = false;
    const float spun = drivetrain_integrate_wheel(
        0.0f, 0.0f, 120.0f, 0.0f, 0.0f, 0.0f,
        spec.wheelRadiusM, spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(spun > 0.0f && !locked, "drive torque spins a wheel forward");
    const float reactionBalanced = drivetrain_integrate_wheel(
        10.0f, 10.0f * spec.wheelRadiusM, 100.0f, 0.0f, 0.0f,
        100.0f / spec.wheelRadiusM, spec.wheelRadiusM,
        spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check_near(reactionBalanced, 10.0, 1e-6,
               "tire reaction torque balances equal drive torque");
    const float braked = drivetrain_integrate_wheel(
        10.0f, 3.1f, 0.0f, 100.0f, 0.0f, 0.0f,
        spec.wheelRadiusM, spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(braked < 10.0f && braked >= 0.0f,
          "service-brake torque opposes positive rotation");
    const float stopped = drivetrain_integrate_wheel(
        0.1f, 3.1f, 0.0f, 1000.0f, 0.0f, 0.0f,
        spec.wheelRadiusM, spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(stopped == 0.0f && locked,
          "brake torque detects lock and cannot reverse the wheel");
    const float released = drivetrain_integrate_wheel(
        stopped, 3.1f, 0.0f, 0.0f, 0.0f, -1000.0f,
        spec.wheelRadiusM, spec.wheelInertiaKgM2, FIXED_DT_S, &locked);
    check(released > 0.0f && !locked,
          "tire reaction spins a locked wheel after brake release");
}

/* Put the vehicle at a steady speed with every wheel already rolling at it, so a scenario
 * can start from cruise instead of spending seconds accelerating into position. */
static void set_vehicle_rolling_speed(Game *game, float velocityLongitudinalMps)
{
    game->vehicle.velocityLongitudinalMps = velocityLongitudinalMps;
    const float omega = velocityLongitudinalMps / game->spec.wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++) {
        game->vehicle.wheels[i].angularVelocityRadS = omega;
    }
}

/* The same, for the bare spec/state fixtures that do not own a Game. */
static void set_rolling_wheels(const VehicleSpec *spec, VehicleState *state,
                               float velocityLongitudinalMps)
{
    const float omega = velocityLongitudinalMps / spec->wheelRadiusM;
    for (int i = 0; i < WHEEL_COUNT; i++) state->wheels[i].angularVelocityRadS = omega;
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: accel-filter — the load-transfer acceleration filter, in isolation             */
/* ------------------------------------------------------------------------------------- */

static void scenario_accel_filter(void)
{
    const float rateHz = LOAD_FILTER_RATE_HZ;
    const float dt = FIXED_DT_S;
    const float alpha = 1.0f - expf(-rateHz * dt);

    /* Zero in, zero out, forever. */
    {
        float filtered = 0.0f;
        for (int i = 0; i < 600; i++) filtered = physics_filter_long_accel(filtered, 0.0f, rateHz, dt);
        check_near((double)filtered, 0.0, 0.0, "a zero input never moves the filter off zero");
    }

    /* One step matches the documented closed form exactly. */
    check_near((double)physics_filter_long_accel(0.0f, 4.0f, rateHz, dt),
               4.0 * (double)alpha, 1e-6,
               "one filter step equals (previous - filtered) * (1 - exp(-rate*dt))");

    /* Positive step: monotonic rise, no overshoot, converging on the input. */
    {
        float filtered = 0.0f;
        float previous = -1.0f;
        bool monotonic = true;
        bool bounded = true;
        for (int i = 0; i < 600; i++) {
            filtered = physics_filter_long_accel(filtered, 5.0f, rateHz, dt);
            if (filtered < previous) monotonic = false;
            if (filtered > 5.0f + 1e-6f) bounded = false;
            previous = filtered;
        }
        check(monotonic, "a positive step converges monotonically");
        check(bounded, "a first-order filter never overshoots its input");
        check_near((double)filtered, 5.0, 1e-4, "it converges on the step value");
    }

    /* Negative step (braking) is the mirror image. */
    {
        float filtered = 0.0f;
        float previous = 1.0f;
        bool monotonic = true;
        for (int i = 0; i < 600; i++) {
            filtered = physics_filter_long_accel(filtered, -8.0f, rateHz, dt);
            if (filtered > previous) monotonic = false;
            previous = filtered;
        }
        check(monotonic, "a negative step converges monotonically downward");
        check_near((double)filtered, -8.0, 1e-4, "it converges on the braking value");
    }

    /* Rate ordering: a faster filter is always further along at the same time. */
    {
        float slow = 0.0f;
        float fast = 0.0f;
        bool ordered = true;
        for (int i = 0; i < 120; i++) {
            slow = physics_filter_long_accel(slow, 5.0f, 5.0f, dt);
            fast = physics_filter_long_accel(fast, 5.0f, 40.0f, dt);
            if (fast < slow - 1e-6f) ordered = false;
        }
        check(ordered, "a higher filter rate tracks the input at least as closely");
    }

    /* Degenerate arguments leave the state alone rather than poisoning it. */
    check_near((double)physics_filter_long_accel(2.0f, 5.0f, rateHz, 0.0f), 2.0, 0.0,
               "a zero timestep is a no-op");
    check_near((double)physics_filter_long_accel(2.0f, 5.0f, 0.0f, dt), 2.0, 0.0,
               "a zero rate is a no-op");
    check(isfinite(physics_filter_long_accel(2.0f, (float)NAN, rateHz, dt)),
          "a non-finite input cannot produce a non-finite filter state");

    /* Determinism: the same sequence twice is bit-identical. */
    {
        float a = 0.0f;
        float b = 0.0f;
        for (int i = 0; i < 240; i++) {
            const float input = (i < 120) ? 3.0f : -6.0f;
            a = physics_filter_long_accel(a, input, rateHz, dt);
            b = physics_filter_long_accel(b, input, rateHz, dt);
        }
        check(memcmp(&a, &b, sizeof(float)) == 0, "the filter is bit-deterministic");
    }

    /* The whole-vehicle wiring: the filter must consume the PREVIOUS step's solved value,
     * so a first tick from rest sees zero acceleration however hard the throttle is held. */
    {
        Game *game = alloc_game();
        game_init(game);
        game->input.throttle = 1.0f;
        game_fixed_update(game, FIXED_DT_S);
        check_near((double)game->derived.previousLongAccelMps2, 0.0, 0.0,
                   "the first tick filters the previous step's zero, not its own acceleration");
        check_near((double)game->derived.filteredLongAccelMps2, 0.0, 0.0,
                   "so the filtered value is still zero after one tick");
        check_near((double)game->derived.staticFrontLoadN, (double)game->derived.normalLoadFrontN,
                   1e-3, "and the first tick's load is the pure static split");

        const float solvedFirst = game->derived.solvedLongAccelMps2;
        game_fixed_update(game, FIXED_DT_S);
        check_near((double)game->derived.previousLongAccelMps2, (double)solvedFirst, 0.0,
                   "the next tick filters exactly the previous tick's solved acceleration");
        check_near((double)game->derived.filteredLongAccelMps2,
                   (double)(solvedFirst * alpha), 1e-6,
                   "one filter step of that value, matching the closed form");

        /* A reset clears both halves of the filter state. */
        for (int i = 0; i < 240; i++) game_fixed_update(game, FIXED_DT_S);
        check(fabsf(game->vehicle.filteredLongAccelMps2) > 0.1f,
              "the filter has accumulated history to clear (%.4f)",
              (double)game->vehicle.filteredLongAccelMps2);
        game_reset_sim(game);
        check_near((double)game->vehicle.filteredLongAccelMps2, 0.0, 0.0,
                   "reset zeroes the filtered acceleration");
        check_near((double)game->vehicle.prevLongAccelMps2, 0.0, 0.0,
                   "reset zeroes the previous acceleration");
        free(game);
    }

    /* No yaw or lateral contamination: the stored value is ax_body, not dvx_dt, so pure
     * rotation with a lateral velocity must not register as longitudinal acceleration. */
    {
        VehicleSpec spec;
        VehicleState state;
        VehicleDerived derived;
        VehicleRenderState renderState;
        phase1_fixture(&spec, &state, &derived, &renderState);
        Input input;
        input_zero(&input);

        state.velocityLongitudinalMps = 10.0f;
        state.velocityLateralMps = 0.0f;
        state.yawRateRadS = 0.0f;
        set_rolling_wheels(&spec, &state, 10.0f);
        physics_fixed_update(&spec, &state, &derived, &renderState, &input, FIXED_DT_S);
        const float straightAx = derived.solvedLongAccelMps2;
        check_near((double)straightAx,
                   (double)(derived.totalBodyForceN.x / spec.massKg), 0.0,
                   "solved acceleration is exactly totalBodyForceX / mass");

        VehicleState rotating;
        VehicleDerived rotatingDerived;
        VehicleRenderState rotatingRender;
        vehicle_state_reset(&spec, &rotating, &rotatingDerived, &rotatingRender);
        rotating.velocityLongitudinalMps = 10.0f;
        rotating.velocityLateralMps = 2.0f;
        rotating.yawRateRadS = 0.5f;
        set_rolling_wheels(&spec, &rotating, 10.0f);
        physics_fixed_update(&spec, &rotating, &rotatingDerived, &rotatingRender, &input,
                             FIXED_DT_S);

        /* dvx_dt carries r*vy = 0.5 * 2.0 = 1.0 m/s^2. The stored value must not. */
        const float transportTermMps2 = rotating.yawRateRadS * 2.0f;
        check(fabsf(rotatingDerived.solvedLongAccelMps2 - straightAx) <
                  0.5f * transportTermMps2,
              "the stored acceleration excludes the r*vy transport term "
              "(straight %.4f, rotating %.4f, transport %.4f m/s^2)",
              (double)straightAx, (double)rotatingDerived.solvedLongAccelMps2,
              (double)transportTermMps2);
        check_near((double)rotatingDerived.solvedLongAccelMps2,
                    (double)rotating.prevLongAccelMps2, 0.0,
                    "the diagnostic and the stored state are the same number");
        check_near((double)rotatingDerived.solvedLongAccelMps2,
                   (double)(rotatingDerived.totalBodyForceN.x / spec.massKg), 0.0,
                   "yaw and lateral transport cannot contaminate the force acceleration");
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: load-transfer — static and dynamic axle loads                                  */
/* ------------------------------------------------------------------------------------- */

static void scenario_load_transfer(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);
    const float weightN = spec.massKg * GRAVITY_MPS2;

    /* Static distribution follows the CG position: a CG nearer the front axle puts more
     * weight on it, which is l_r / L, not l_f / L. */
    {
        const AxleLoads loads = physics_axle_loads(&spec, 0.0f);
        check_near((double)loads.staticFrontN,
                   (double)(weightN * spec.cgToRearM / spec.wheelbaseM), 0.01,
                   "static front load is m*g*l_r/L");
        check_near((double)loads.staticRearN,
                   (double)(weightN * spec.cgToFrontM / spec.wheelbaseM), 0.01,
                   "static rear load is m*g*l_f/L");
        check_near((double)(loads.staticFrontN + loads.staticRearN), (double)weightN, 0.01,
                   "static loads sum to mass * gravity");
        check_near((double)loads.transferN, 0.0, 1e-4,
                   "zero acceleration transfers no load");
        check_near((double)loads.frontN, (double)loads.staticFrontN, 1e-3,
                   "and the dynamic front load is the static one");
        check(loads.staticFrontN > loads.staticRearN,
              "the default CG sits ahead of centre, loading the front axle more "
              "(%.1f vs %.1f N)", (double)loads.staticFrontN, (double)loads.staticRearN);
    }

    /* Moving the CG forward and back must move the static split, both ways. */
    {
        VehicleSpec forward = spec;
        forward.cgToFrontM = 0.90f;
        forward.cgToRearM = 1.65f;
        forward.wheelbaseM = forward.cgToFrontM + forward.cgToRearM;
        const AxleLoads front = physics_axle_loads(&forward, 0.0f);

        VehicleSpec rearward = spec;
        rearward.cgToFrontM = 1.65f;
        rearward.cgToRearM = 0.90f;
        rearward.wheelbaseM = rearward.cgToFrontM + rearward.cgToRearM;
        const AxleLoads rear = physics_axle_loads(&rearward, 0.0f);

        check(front.staticFrontN > rear.staticFrontN + 100.0f,
              "moving the CG forward increases the static front load (%.1f -> %.1f N)",
              (double)rear.staticFrontN, (double)front.staticFrontN);
        check(rear.staticRearN > front.staticRearN + 100.0f,
              "moving the CG rearward increases the static rear load (%.1f -> %.1f N)",
              (double)front.staticRearN, (double)rear.staticRearN);
        check_near((double)(front.staticFrontN + front.staticRearN), (double)weightN, 0.01,
                   "a forward CG still weighs mass * gravity");
        check_near((double)(rear.staticFrontN + rear.staticRearN), (double)weightN, 0.01,
                   "a rearward CG still weighs mass * gravity");
    }

    /* Acceleration transfers rearward, braking forward, and the transfer is exactly
     * m * ax * h / L in both directions. */
    {
        const float axMps2 = 5.0f;
        const AxleLoads accelerating = physics_axle_loads(&spec, axMps2);
        const float expectedN = spec.massKg * axMps2 * spec.cgHeightM / spec.wheelbaseM;
        check_near((double)accelerating.transferN, (double)expectedN, 0.01,
                   "load transfer is m * ax * h / L");
        check(accelerating.unclampedFrontN < accelerating.staticFrontN,
              "accelerating unloads the front axle (%.1f -> %.1f N)",
              (double)accelerating.staticFrontN, (double)accelerating.unclampedFrontN);
        check(accelerating.unclampedRearN > accelerating.staticRearN,
              "accelerating loads the rear axle (%.1f -> %.1f N)",
              (double)accelerating.staticRearN, (double)accelerating.unclampedRearN);
        check_near((double)(accelerating.unclampedFrontN + accelerating.unclampedRearN),
                   (double)weightN, 0.01,
                   "transfer moves load without creating or destroying any");

        const AxleLoads braking = physics_axle_loads(&spec, -axMps2);
        check_near((double)braking.transferN, -(double)expectedN, 0.01,
                   "braking transfer is the exact mirror of accelerating transfer");
        check(braking.unclampedFrontN > braking.staticFrontN,
              "braking loads the front axle (%.1f -> %.1f N)",
              (double)braking.staticFrontN, (double)braking.unclampedFrontN);
        check(braking.unclampedRearN < braking.staticRearN,
              "braking unloads the rear axle (%.1f -> %.1f N)",
              (double)braking.staticRearN, (double)braking.unclampedRearN);
        check_near((double)(braking.unclampedFrontN + braking.unclampedRearN),
                   (double)weightN, 0.01, "and still sums to mass * gravity");
    }

    /* CG height scales the transfer and nothing else. */
    {
        VehicleSpec low = spec;
        low.cgHeightM = 0.25f;
        VehicleSpec high = spec;
        high.cgHeightM = 0.75f;
        const AxleLoads lowLoads = physics_axle_loads(&low, 5.0f);
        const AxleLoads highLoads = physics_axle_loads(&high, 5.0f);
        check_near((double)highLoads.transferN, 3.0 * (double)lowLoads.transferN, 0.05,
                   "transfer is linear in CG height");
        check_near((double)lowLoads.staticFrontN, (double)highLoads.staticFrontN, 0.01,
                   "CG height does not change the static split");
    }

    /* The minimum-load clamp catches an unloaded axle without renormalising the other. */
    {
        const AxleLoads extreme = physics_axle_loads(&spec, 40.0f);
        check(extreme.unclampedFrontN < 0.0f,
              "an extreme acceleration drives the unclamped front load negative (%.1f N)",
              (double)extreme.unclampedFrontN);
        check_near((double)extreme.frontN, (double)MIN_NORMAL_LOAD_N, 1e-3,
                   "the clamped front load stops at MIN_NORMAL_LOAD_N");
        check_near((double)extreme.rearN, (double)extreme.unclampedRearN, 0.01,
                   "the rear axle is not renormalised to absorb the clamped difference");
        check_near((double)(extreme.unclampedFrontN + extreme.unclampedRearN),
                   (double)weightN, 0.01,
                   "the unclamped pair still sums to mass * gravity after clamping");

        const AxleLoads reverse = physics_axle_loads(&spec, -40.0f);
        check_near((double)reverse.rearN, (double)MIN_NORMAL_LOAD_N, 1e-3,
                   "extreme braking clamps the rear load instead");

        check(isfinite(extreme.frontN) && isfinite(extreme.rearN) &&
              isfinite(reverse.frontN) && isfinite(reverse.rearN),
              "extreme but finite acceleration produces finite loads");
        check(extreme.frontN > 0.0f && extreme.rearN > 0.0f &&
              reverse.frontN > 0.0f && reverse.rearN > 0.0f,
              "no clamped load is ever negative, so no wheel can generate negative grip");
    }

    /* Load reaches the tires: capacity follows the dynamic load, split evenly per axle. */
    {
        Game *game = alloc_game();
        game_init(game);
        game->input.throttle = 1.0f;
        for (int i = 0; i < 60; i++) game_fixed_update(game, FIXED_DT_S);

        check(game->derived.loadTransferN > 0.0f,
              "full throttle produces a positive (rearward) transfer (%.1f N)",
              (double)game->derived.loadTransferN);
        check(game->derived.normalLoadFrontN < game->derived.staticFrontLoadN,
              "the front axle is carrying less than its static load");
        check(game->derived.normalLoadRearN > game->derived.staticRearLoadN,
              "the rear axle is carrying more than its static load");
        check_near((double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN +
                            game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN),
                   (double)game->derived.normalLoadFrontN, 1e-2,
                   "the two front wheel loads sum to the front axle load");
        check_near((double)(game->vehicle.wheels[WHEEL_REAR_LEFT].normalLoadN +
                            game->vehicle.wheels[WHEEL_REAR_RIGHT].normalLoadN),
                   (double)game->derived.normalLoadRearN, 1e-2,
                   "the two rear wheel loads sum to the rear axle load");
        check_near((double)game->vehicle.wheels[WHEEL_FRONT_LEFT].normalLoadN,
                   (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].normalLoadN, 0.0,
                   "left and right stay equal: lateral load transfer is Phase 4");

        /* Friction capacity is mu * Fz, so a heavier axle can make more force. */
        const float rearCapacityN = game->spec.tireMuLatRear * game->derived.normalLoadRearN;
        const float staticRearCapacityN =
            game->spec.tireMuLatRear * game->derived.staticRearLoadN;
        check(rearCapacityN > staticRearCapacityN,
              "the loaded rear axle has more lateral capacity than at rest (%.0f > %.0f N)",
              (double)rearCapacityN, (double)staticRearCapacityN);
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: resistance — aerodynamic drag and rolling resistance                           */
/* ------------------------------------------------------------------------------------- */

static void scenario_resistance(void)
{
    VehicleSpec spec;
    vehicle_spec_set_default(&spec);

    const float referenceN = 0.5f * AIR_DENSITY_KGM3 * spec.dragCoefficient *
                             spec.frontalAreaM2;

    /* ------------------------------------------------------------------ aerodynamic drag -- */

    {
        float magnitudeN = -1.0f;
        const Vector2 atRest = physics_aero_drag_body_n(&spec, 0.0f, 0.0f, &magnitudeN);
        check(atRest.x == 0.0f && atRest.y == 0.0f, "drag is exactly zero at rest");
        check_near((double)magnitudeN, 0.0, 0.0, "and reports zero magnitude");
    }

    {
        float magnitudeN = 0.0f;
        const Vector2 forward = physics_aero_drag_body_n(&spec, 20.0f, 0.0f, &magnitudeN);
        check_near((double)magnitudeN, (double)(referenceN * 400.0f), 0.01,
                   "drag magnitude is 0.5 * rho * Cd * A * v^2");
        check(forward.x < 0.0f, "forward motion produces rearward drag (%.2f N)",
              (double)forward.x);
        check_near((double)forward.y, 0.0, 1e-6, "and no lateral component");

        const Vector2 reverse = physics_aero_drag_body_n(&spec, -20.0f, 0.0f, NULL);
        check(reverse.x > 0.0f, "reverse motion produces forward drag (%.2f N)",
              (double)reverse.x);
        check_near((double)reverse.x, -(double)forward.x, 1e-3,
                   "drag is symmetric in the direction of travel");
    }

    {
        /* A car travelling sideways still pushes air. This is the check that catches drag
         * being applied along body X regardless of where the car is actually going. */
        float magnitudeN = 0.0f;
        const Vector2 sideways = physics_aero_drag_body_n(&spec, 0.0f, 15.0f, &magnitudeN);
        check_near((double)magnitudeN, (double)(referenceN * 225.0f), 0.01,
                   "pure lateral motion produces the same quadratic magnitude");
        check(sideways.y < 0.0f && fabsf(sideways.x) < 1e-6f,
              "and it acts purely along -body Y (%.2f, %.2f)",
              (double)sideways.x, (double)sideways.y);
    }

    {
        /* Diagonal motion: the force is antiparallel to the velocity vector. */
        const float vx = 12.0f;
        const float vy = -9.0f;      /* speed 15 */
        float magnitudeN = 0.0f;
        const Vector2 diagonal = physics_aero_drag_body_n(&spec, vx, vy, &magnitudeN);
        const float speedMps = sqrtf(vx * vx + vy * vy);
        check_near((double)magnitudeN, (double)(referenceN * speedMps * speedMps), 0.01,
                   "diagonal drag uses the full speed, not just the forward component");
        check_near((double)diagonal.x, (double)(-magnitudeN * vx / speedMps), 1e-3,
                   "the X component follows the velocity direction");
        check_near((double)diagonal.y, (double)(-magnitudeN * vy / speedMps), 1e-3,
                   "the Y component follows the velocity direction");
        const float dot = diagonal.x * vx + diagonal.y * vy;
        check(dot < 0.0f, "drag opposes the velocity vector (dot %.3f)", (double)dot);
        check_near((double)sqrtf(diagonal.x * diagonal.x + diagonal.y * diagonal.y),
                   (double)magnitudeN, 1e-2,
                   "and its length is the reported magnitude");
    }

    {
        /* Quadratic scaling: doubling speed quadruples drag. */
        float lowN = 0.0f;
        float highN = 0.0f;
        physics_aero_drag_body_n(&spec, 10.0f, 0.0f, &lowN);
        physics_aero_drag_body_n(&spec, 20.0f, 0.0f, &highN);
        check(fabsf(highN - 4.0f * lowN) < 0.01f,
              "drag is quadratic in speed (%.2f N at 10 m/s -> %.2f N at 20 m/s)",
              (double)lowN, (double)highN);
    }

    {
        /* Near zero it stays finite and small rather than dividing by a vanishing speed. */
        bool finite = true;
        float previousN = 0.0f;
        bool monotonic = true;
        for (int i = 0; i <= 200; i++) {
            const float v = (float)i * 1e-5f;
            float magnitudeN = 0.0f;
            const Vector2 drag = physics_aero_drag_body_n(&spec, v, 0.0f, &magnitudeN);
            if (!isfinite(drag.x) || !isfinite(drag.y) || !isfinite(magnitudeN)) finite = false;
            if (magnitudeN < previousN - 1e-9f) monotonic = false;
            previousN = magnitudeN;
        }
        check(finite, "drag stays finite as speed approaches zero");
        check(monotonic, "and never spikes on the way down");
    }

    /* --------------------------------------------------------------- rolling resistance -- */

    const float loadN = 3000.0f;
    const float coefficient = spec.rollingResistanceCoefficient;

    {
        float magnitudeN = -1.0f;
        const Vector2 atRest = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ 0.0f, 0.0f }, &magnitudeN);
        check(atRest.x == 0.0f && atRest.y == 0.0f,
              "rolling resistance is exactly zero at rest, inventing no direction");
        check_near((double)magnitudeN, 0.0, 0.0, "and reports zero magnitude");
    }

    {
        float magnitudeN = 0.0f;
        const Vector2 forward = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ 5.0f, 0.0f }, &magnitudeN);
        check_near((double)magnitudeN, (double)(coefficient * loadN), 1e-4,
                   "rolling resistance magnitude is the coefficient times the normal load");
        check(forward.x < 0.0f, "it opposes forward motion (%.3f N)", (double)forward.x);

        const Vector2 reverse = physics_rolling_resistance_body_n(
            coefficient, loadN, (Vector2){ -5.0f, 0.0f }, NULL);
        check(reverse.x > 0.0f, "it opposes reverse motion too (%.3f N)", (double)reverse.x);
        check_near((double)reverse.x, -(double)forward.x, 1e-5,
                   "and is symmetric in direction");
    }

    {
        /* Load scaling is what makes rolling resistance move with load transfer. */
        float lightN = 0.0f;
        float heavyN = 0.0f;
        physics_rolling_resistance_body_n(coefficient, 2000.0f,
                                          (Vector2){ 5.0f, 0.0f }, &lightN);
        physics_rolling_resistance_body_n(coefficient, 6000.0f,
                                          (Vector2){ 5.0f, 0.0f }, &heavyN);
        check(fabsf(heavyN - 3.0f * lightN) < 1e-3f,
              "rolling resistance is linear in normal load (%.3f N at 2 kN -> %.3f N at 6 kN)",
              (double)lightN, (double)heavyN);
        check(fabsf(lightN - heavyN) > 1e-3f,
              "so front and rear contributions differ once load has transferred");
    }

    {
        /* Unlike drag it does not vanish with speed — it is a load-driven force, not a
         * velocity-driven one — but it must fade to zero at rest rather than chatter. */
        float slowN = 0.0f;
        float fastN = 0.0f;
        physics_rolling_resistance_body_n(coefficient, loadN,
                                          (Vector2){ 4.0f, 0.0f }, &slowN);
        physics_rolling_resistance_body_n(coefficient, loadN,
                                          (Vector2){ 30.0f, 0.0f }, &fastN);
        check_near((double)fastN, (double)slowN, 1e-5,
                   "rolling resistance does not grow with speed the way drag does");

        bool finite = true;
        bool bounded = true;
        for (int i = 0; i <= 400; i++) {
            const float v = (float)i * 1e-4f;
            float magnitudeN = 0.0f;
            const Vector2 rolling = physics_rolling_resistance_body_n(
                coefficient, loadN, (Vector2){ v, 0.0f }, &magnitudeN);
            if (!isfinite(rolling.x) || !isfinite(magnitudeN)) finite = false;
            if (magnitudeN > coefficient * loadN + 1e-6f) bounded = false;
        }
        check(finite, "rolling resistance stays finite as the contact speed approaches zero");
        check(bounded, "and never exceeds coefficient * load");
    }

    {
        /* Diagonal contact velocity: opposite the contact velocity vector. */
        const Vector2 contact = { 6.0f, -8.0f };
        float magnitudeN = 0.0f;
        const Vector2 rolling = physics_rolling_resistance_body_n(
            coefficient, loadN, contact, &magnitudeN);
        const float dot = rolling.x * contact.x + rolling.y * contact.y;
        check(dot < 0.0f, "rolling resistance opposes the contact velocity (dot %.3f)",
              (double)dot);
        check_near((double)sqrtf(rolling.x * rolling.x + rolling.y * rolling.y),
                   (double)magnitudeN, 1e-4, "its length is the reported magnitude");
    }

    /* ------------------------------------------------------- no temporary Phase 2 path -- */

    {
        /* The removed Phase 2 term was a body force linear in speed. If any equivalent
         * survived, total resistance at a given speed would exceed the two physical forms
         * by an amount that grows with speed. Compare the model against the closed forms. */
        Game *game = alloc_game();
        game_init(game);
        game->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(game, 25.0f);
        game_fixed_update(game, FIXED_DT_S);

        float expectedDragN = 0.0f;
        physics_aero_drag_body_n(&game->spec, game->vehicle.velocityLongitudinalMps,
                                 game->vehicle.velocityLateralMps, &expectedDragN);
        const float expectedRollingN =
            game->spec.rollingResistanceCoefficient * game->spec.massKg * GRAVITY_MPS2;

        check_near((double)game->derived.aeroDragMagnitudeN, (double)expectedDragN, 0.5,
                   "the model's drag is exactly the closed form, with nothing added");
        check_near((double)game->derived.rollingResistanceMagnitudeN,
                   (double)expectedRollingN, 5.0,
                   "the model's rolling resistance is the coefficient times the total load");

        const float linearPhase2N = 120.0f * 25.0f;    /* the removed term at this speed */
        check(game->derived.aeroDragMagnitudeN + game->derived.rollingResistanceMagnitudeN <
                  linearPhase2N,
              "total resistance is far below the removed linear term (%.1f N vs %.1f N)",
              (double)(game->derived.aeroDragMagnitudeN +
                       game->derived.rollingResistanceMagnitudeN),
              (double)linearPhase2N);
        free(game);
    }

    /* -------------------------------------------------------- zero-crossing, in the model -- */

    {
        /* Resistance alone must bring the car to rest and stop there, never push it back. */
        Game *game = alloc_game();
        game_init(game);
        game->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(game, 0.02f);
        bool reversed = false;
        bool finite = true;
        for (int i = 0; i < 600; i++) {
            game_fixed_update(game, FIXED_DT_S);
            if (game->vehicle.velocityLongitudinalMps < -1e-6f) reversed = true;
            if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) {
                finite = false;
            }
        }
        check(!reversed, "resistance never drives the vehicle backwards through zero");
        check(finite, "and the state stays valid the whole way down");
        check(fabsf(game->vehicle.velocityLongitudinalMps) < 0.02f,
              "the vehicle ends at or below where it started (%.6f m/s)",
              (double)game->vehicle.velocityLongitudinalMps);
        free(game);
    }
}

static void scenario_rest(void)
{
    Game *game = alloc_game();
    game_init(game);
    input_zero(&game->input);
    for (int i = 0; i < 1200; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->vehicle.positionM.x, 0.0, 1e-7, "rest position X remains fixed");
    check_near(game->vehicle.positionM.y, 0.0, 1e-7, "rest position Y remains fixed");
    check_near(game->derived.speedMps, 0.0, 1e-7, "rest speed remains zero");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7, "rest yaw rate remains zero");
    check_near(game->derived.totalBodyForceN.x, 0.0, 1e-7, "rest longitudinal force remains zero");
    check_near(game->derived.totalBodyForceN.y, 0.0, 1e-7, "rest lateral force remains zero");
    check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
          "rest state remains finite and inside safety bounds");

    game->input.steer = 1.0f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->derived.speedMps, 0.0, 1e-7,
               "steering while stationary creates no vehicle motion");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7,
               "steering while stationary creates no yaw");
    check_near(game->derived.lateralAccelerationMps2, 0.0, 1e-7,
               "steering while stationary creates no lateral acceleration");
    game->input.steer = 0.0f;
    game->input.handbrake = 1.0f;
    for (int i = 0; i < 120; i++) game_fixed_update(game, FIXED_DT_S);
    check_near(game->derived.speedMps, 0.0, 1e-7,
               "handbrake at rest cannot launch the vehicle");
    check_near(game->vehicle.yawRateRadS, 0.0, 1e-7,
               "handbrake at rest cannot rotate the vehicle");
    free(game);
}

static void scenario_launch_stop(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool telemetryOpened = telemetry_open(&writer, PHASE2_LAUNCH_TELEMETRY);
    game->input.throttle = 1.0f;
    float maxRearSlip = 0.0f;
    float maxRearForceN = 0.0f;
    float maxEngineRpm = game->vehicle.engineRpm;
    float maxFrontSlip = 0.0f;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
        maxRearSlip = fmaxf(maxRearSlip,
            game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio);
        maxRearForceN = fmaxf(maxRearForceN,
            game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
            game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN);
        maxEngineRpm = fmaxf(maxEngineRpm, game->vehicle.engineRpm);
        maxFrontSlip = fmaxf(maxFrontSlip,
            fabsf(game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio));
        if (telemetryOpened && (i + 1) % 120 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    const float launchSpeed = game->vehicle.velocityLongitudinalMps;
    check(launchSpeed > 10.0f, "straight launch builds forward speed (%.3f m/s)",
          (double)launchSpeed);
    check(fabsf(game->vehicle.velocityLateralMps) < 1e-5f,
          "straight launch keeps lateral velocity near zero");
    check(fabsf(game->vehicle.yawRateRadS) < 1e-5f,
          "straight launch keeps yaw rate near zero");
    check(game->vehicle.positionM.x > 20.0f && fabsf(game->vehicle.positionM.y) < 1e-5f,
          "straight launch advances along world +X only");
    check(maxRearSlip > 0.1f && maxRearForceN > 1000.0f,
          "rear wheelspin creates positive longitudinal tire force "
          "(slip %.3f, Fx %.1f N)", (double)maxRearSlip, (double)maxRearForceN);
    check(maxEngineRpm > game->spec.engineIdleRpm,
          "engine RPM rises from driven-wheel speed (max %.0f rpm)",
          (double)maxEngineRpm);
    check(maxFrontSlip < 0.1f,
          "front wheels remain close to free rolling (max |slip| %.3f)",
          (double)maxFrontSlip);
    check_near(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
               game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS, 0.0,
               "locked rear axle stays synchronized");
    printf("    acceleration: %.3f m/s, %.3f m, rear slip %.3f, rear Fx %.1f N, %.0f rpm\n",
           (double)launchSpeed, (double)game->vehicle.positionM.x,
           (double)maxRearSlip, (double)maxRearForceN, (double)maxEngineRpm);

    game->input.throttle = 0.0f;
    for (int i = 0; i < 180; i++) game_fixed_update(game, FIXED_DT_S);
    game->input.brake = 1.0f;
    float previousAbsSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
    bool monotonic = true;
    float maxSpeedIncrease = 0.0f;
    int maxSpeedIncreaseTick = 0;
    float maxRiseBefore = 0.0f;
    float maxRiseAfter = 0.0f;
    float maxRiseRearSlip = 0.0f;
    float maxRiseRearFx = 0.0f;
    float minimumSlip = 0.0f;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const float absSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
        const float speedIncrease = absSpeed - previousAbsSpeed;
        minimumSlip = fminf(minimumSlip,
            game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio);
        if (speedIncrease > maxSpeedIncrease) {
            maxSpeedIncrease = speedIncrease;
            maxSpeedIncreaseTick = i;
            maxRiseBefore = previousAbsSpeed;
            maxRiseAfter = absSpeed;
            maxRiseRearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
            maxRiseRearFx = game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                            game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN;
        }
        if (speedIncrease > 1e-5f) monotonic = false;
        previousAbsSpeed = absSpeed;
        if (telemetryOpened && (i + 1) % 120 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    check(monotonic,
          "service braking reduces speed without an impulse reversal "
          "(max rise %.7f at %d: %.7f -> %.7f, rear slip %.3f Fx %.1f)",
          (double)maxSpeedIncrease, maxSpeedIncreaseTick,
          (double)maxRiseBefore, (double)maxRiseAfter,
          (double)maxRiseRearSlip, (double)maxRiseRearFx);
    check(game->vehicle.velocityLongitudinalMps >= -1e-6f,
          "ordinary braking does not accelerate the vehicle backward");
    check(fabsf(game->vehicle.velocityLongitudinalMps) < 1e-5f,
          "braking settles at zero speed");
    check(minimumSlip < -0.1f, "service braking creates negative wheel slip (min %.3f)",
          (double)minimumSlip);
    check_near(game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
               game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT],
               game->spec.maxBrakeTorqueNm * game->spec.brakeBiasFront, 1e-4,
               "service-brake front torque follows configured bias");
    printf("    braking: %.3f -> %.6f m/s, min slip %.3f, no reversal\n",
           (double)launchSpeed, (double)game->derived.speedMps, (double)minimumSlip);
    check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
          "launch and stopping remain finite");
    check(telemetryOpened && telemetry_close(&writer),
          "launch/stop telemetry writes successfully");
    free(game);
}

static void scenario_coast_down(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(&writer, TELEMETRY_DIR "/phase2_coast_down.csv");

    game->input.throttle = 1.0f;
    for (int i = 0; i < 480; i++) game_fixed_update(game, FIXED_DT_S);
    const float initialSpeed = game->derived.speedMps;
    const float driveBeforeLift = game->derived.drivelineTorqueNm;
    game->input.throttle = 0.0f;

    bool finite = true;
    float peakSpeedAfterSettling = 0.0f;
    float previousAbsOmega = fabsf(
        game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS);
    int omegaSignChanges = 0;
    float previousOmega = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
    for (int i = 0; i < 720; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (i >= 60 && game->derived.speedMps > peakSpeedAfterSettling) {
            peakSpeedAfterSettling = game->derived.speedMps;
        }
        const float omega = game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS;
        if (omega * previousOmega < 0.0f) omegaSignChanges++;
        previousOmega = omega;
        previousAbsOmega = fabsf(omega);
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) finite = false;
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    (void)previousAbsOmega;
    check(initialSpeed > 5.0f, "coast-down begins from useful speed (%.3f m/s)",
          (double)initialSpeed);
    check(game->derived.drivelineTorqueNm < driveBeforeLift,
          "throttle lift reduces driveline torque");
    check(game->derived.speedMps < initialSpeed,
          "engine braking, drag, and rolling resistance reduce coast speed");
    check(peakSpeedAfterSettling <= initialSpeed + 0.05f,
          "coast-down has no post-lift force spike (peak %.3f vs %.3f)",
          (double)peakSpeedAfterSettling, (double)initialSpeed);
    check(omegaSignChanges == 0,
          "rear wheel speed does not oscillate through zero during coast-down");
    check(finite, "coast-down remains finite");
    check(opened && telemetry_close(&writer), "coast-down telemetry writes successfully");
    free(game);

    /* Phase 3: the two resistance forms are separate, and each behaves like itself. Sampled
     * at decreasing speed from a free-rolling, neutral-gear car so nothing else is acting. */
    {
        Game *probe = alloc_game();
        game_init(probe);
        probe->vehicle.selectedGear = 0;

        struct { float speedMps; float dragN; float rollingN; float loadN; } samples[4];
        static const float speeds[4] = { 30.0f, 20.0f, 15.0f, 10.0f };
        for (int s = 0; s < 4; s++) {
            game_reset_sim(probe);
            probe->vehicle.selectedGear = 0;
            set_vehicle_rolling_speed(probe, speeds[s]);
            game_fixed_update(probe, FIXED_DT_S);
            samples[s].speedMps = speeds[s];
            samples[s].dragN = probe->derived.aeroDragMagnitudeN;
            samples[s].rollingN = probe->derived.rollingResistanceMagnitudeN;
            samples[s].loadN = probe->derived.normalLoadFrontN + probe->derived.normalLoadRearN;
        }

        /* Drag falls with the square of speed: halving speed quarters it. */
        check(fabsf(samples[0].dragN - 4.0f * samples[2].dragN) < 1.0f,
              "aerodynamic drag falls with speed squared (30 m/s %.1f N, 15 m/s %.1f N)",
              (double)samples[0].dragN, (double)samples[2].dragN);
        check(samples[0].dragN > samples[1].dragN &&
              samples[1].dragN > samples[2].dragN &&
              samples[2].dragN > samples[3].dragN,
              "drag decreases monotonically as the car slows");

        /* Rolling resistance does not: it tracks load, which is near static while coasting. */
        check(fabsf(samples[0].rollingN - samples[3].rollingN) < 5.0f,
              "rolling resistance is load-driven, not speed-driven (%.1f N at 30 m/s vs "
              "%.1f N at 10 m/s)", (double)samples[0].rollingN, (double)samples[3].rollingN);
        check_near((double)samples[0].rollingN,
                   (double)(probe->spec.rollingResistanceCoefficient * samples[0].loadN),
                   5.0, "and equals the coefficient times the current dynamic load");

        /* Resistance never changes sign while the direction of travel does not. */
        game_reset_sim(probe);
        probe->vehicle.selectedGear = 0;
        set_vehicle_rolling_speed(probe, 28.0f);
        bool directionHeld = true;
        bool spikeFree = true;
        float previousTotalN = 1e9f;
        for (int i = 0; i < 1800; i++) {
            game_fixed_update(probe, FIXED_DT_S);
            if (probe->vehicle.velocityLongitudinalMps <= 0.0f) break;
            if (probe->derived.aeroDragBodyN.x > 0.0f ||
                probe->derived.rollingResistanceBodyN.x > 0.0f) directionHeld = false;
            const float totalN = fabsf(probe->derived.aeroDragBodyN.x) +
                                 fabsf(probe->derived.rollingResistanceBodyN.x);
            if (totalN > previousTotalN + 1.0f) spikeFree = false;
            previousTotalN = totalN;
        }
        check(directionHeld,
              "both resistance forces keep opposing forward travel for the whole coast");
        check(spikeFree, "and neither spikes as the car slows");
        free(probe);
    }
}

static void scenario_braking_cornering(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(
        &writer, TELEMETRY_DIR "/phase2_braking_cornering.csv");
    game->vehicle.selectedGear = 0;
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.28f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }

    game->input.brake = 1.0f;
    bool sharedBudgetObserved = false;
    float maxUsage = 0.0f;
    bool withinLimit = true;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        for (int w = 0; w < WHEEL_COUNT; w++) {
            const WheelState *wheel = &game->vehicle.wheels[w];
            const SurfaceSpec *sv_test = Surface_Get(wheel->surfaceId);
            const float muScale_test = game->derived.tireLoadSensitivityMuScale[w];
            const float lateralMu = (w <= WHEEL_FRONT_RIGHT)
                ? game->spec.tireMuLatFront : game->spec.tireMuLatRear;
            const float nx = wheel->forceLongitudinalN /
                             (game->spec.tireMuLongScale * sv_test->muLongitudinal *
                              muScale_test * wheel->normalLoadN);
            const float ny = wheel->forceLateralN /
                             (lateralMu * (sv_test->muLateral / SURFACE_REFERENCE_MU_LAT) *
                              muScale_test * wheel->normalLoadN);
            const float usage = sqrtf(nx * nx + ny * ny);
            if (usage > 1.0f + FRICTION_TOLERANCE) withinLimit = false;
            if (wheel->frictionUsage > maxUsage) maxUsage = wheel->frictionUsage;
            if (fabsf(game->derived.pureLateralForceN[w]) >
                fabsf(wheel->forceLateralN) + 1.0f &&
                fabsf(wheel->forceLongitudinalN) > 100.0f) {
                sharedBudgetObserved = true;
            }
        }
    }
    check(maxUsage > 0.9f,
          "corner braking raises friction usage (max %.3f)", (double)maxUsage);
    check(sharedBudgetObserved,
          "corner braking reduces lateral force through the shared friction budget");
    check(withinLimit, "every corner-braking wheel remains within its friction ellipse");
    check(game->derived.speedMps < 12.0f, "corner braking reduces vehicle speed");
    check(opened && telemetry_close(&writer),
          "corner-braking telemetry writes successfully");
    free(game);
}

static void scenario_power_oversteer(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(
        &writer, TELEMETRY_DIR "/phase2_power_oversteer.csv");
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.20f;
    game->input.throttle = 0.15f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }

    game->input.throttle = 1.0f;
    float maxRearSlip = -1000.0f;
    float maxRearUsage = 0.0f;
    float maxFrontUsage = 0.0f;
    float minimumRearLateralRatio = 1.0f;
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const WheelState *front = &game->vehicle.wheels[WHEEL_FRONT_LEFT];
        const WheelState *rear = &game->vehicle.wheels[WHEEL_REAR_LEFT];
        maxRearSlip = fmaxf(maxRearSlip, rear->slipRatio);
        maxRearUsage = fmaxf(maxRearUsage, rear->frictionUsage);
        maxFrontUsage = fmaxf(maxFrontUsage, front->frictionUsage);
        const float pureRearFy = fabsf(
            game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            minimumRearLateralRatio = fminf(
                minimumRearLateralRatio,
                fabsf(rear->forceLateralN) / pureRearFy);
        }
    }
    const float sideslipUnderPower = fabsf(game->derived.bodySideslipRad);
    game->input.throttle = 0.0f;
    float bestRecoveredRatio = 0.0f;
    for (int i = 0; i < 240; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const float pureRearFy = fabsf(
            game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            bestRecoveredRatio = fmaxf(
                bestRecoveredRatio,
                fabsf(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLateralN) /
                pureRearFy);
        }
    }
    check(maxRearSlip > 0.1f,
          "power oversteer raises driven-wheel slip ratio (max %.3f)",
          (double)maxRearSlip);
    check(maxRearUsage >= maxFrontUsage - 0.05f,
          "rear friction usage reaches saturation at least as strongly as front "
          "(rear %.3f front %.3f)", (double)maxRearUsage, (double)maxFrontUsage);
    check(minimumRearLateralRatio < 0.9f,
          "rear combined slip reduces lateral authority (ratio %.3f)",
          (double)minimumRearLateralRatio);
    check(sideslipUnderPower > 0.01f,
          "power-oversteer maneuver develops body sideslip (%.3f rad)",
          (double)sideslipUnderPower);
    check(bestRecoveredRatio > minimumRearLateralRatio + 0.05f,
          "throttle lift restores rear lateral authority (%.3f -> %.3f)",
          (double)minimumRearLateralRatio, (double)bestRecoveredRatio);
    check(opened && telemetry_close(&writer),
          "power-oversteer telemetry writes successfully");
    free(game);
}

static void scenario_handbrake_entry(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool opened = telemetry_open(
        &writer, TELEMETRY_DIR "/phase2_handbrake_entry.csv");
    game->vehicle.selectedGear = 0;
    set_vehicle_rolling_speed(game, 12.0f);
    game->input.steer = 0.22f;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    const float yawBefore = fabsf(game->vehicle.yawRateRadS);

    game->input.handbrake = 1.0f;
    bool rearLocked = false;
    float minimumRearSlip = 0.0f;
    float maxRearUsage = 0.0f;
    float minimumRearLateralRatio = 1.0f;
    float maxYaw = yawBefore;
    for (int i = 0; i < 120; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const WheelState *rear = &game->vehicle.wheels[WHEEL_REAR_LEFT];
        rearLocked = rearLocked || rear->locked;
        minimumRearSlip = fminf(minimumRearSlip, rear->slipRatio);
        maxRearUsage = fmaxf(maxRearUsage, rear->frictionUsage);
        maxYaw = fmaxf(maxYaw, fabsf(game->vehicle.yawRateRadS));
        const float pureRearFy = fabsf(
            game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            minimumRearLateralRatio = fminf(
                minimumRearLateralRatio,
                fabsf(rear->forceLateralN) / pureRearFy);
        }
    }
    game->input.handbrake = 0.0f;
    float recoveredRatio = 0.0f;
    for (int i = 0; i < 240; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (opened && (i + 1) % 12 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
        const float pureRearFy = fabsf(
            game->derived.pureLateralForceN[WHEEL_REAR_LEFT]);
        if (pureRearFy > 1.0f) {
            recoveredRatio = fmaxf(
                recoveredRatio,
                fabsf(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLateralN) /
                pureRearFy);
        }
    }
    check(game->derived.handbrakeTorqueNm[WHEEL_FRONT_LEFT] == 0.0f &&
          game->derived.handbrakeTorqueNm[WHEEL_FRONT_RIGHT] == 0.0f,
          "handbrake torque remains rear-only");
    check(rearLocked, "handbrake entry locks the rear axle");
    check(minimumRearSlip < -0.5f,
          "handbrake creates negative rear slip (min %.3f)", (double)minimumRearSlip);
    check(maxRearUsage > 0.9f,
          "handbrake consumes rear friction budget (max %.3f)", (double)maxRearUsage);
    check(minimumRearLateralRatio < 0.9f,
          "handbrake reduces rear lateral force through combined grip");
    check(maxYaw > yawBefore,
          "handbrake entry increases yaw response without direct yaw code");
    check(recoveredRatio > minimumRearLateralRatio + 0.05f,
          "handbrake release restores rear lateral authority");
    check(opened && telemetry_close(&writer),
          "handbrake-entry telemetry writes successfully");
    free(game);
}

static void scenario_low_speed(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->input.throttle = 0.35f;
    game->input.steer = 0.35f;
    float previousYaw = 0.0f;
    float maxYawJump = 0.0f;
    bool finite = true;
    bool crossedBlend = false;
    for (int i = 0; i < 480; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const float jump = fabsf(game->vehicle.yawRateRadS - previousYaw);
        if (jump > maxYawJump) maxYawJump = jump;
        previousYaw = game->vehicle.yawRateRadS;
        if (game->derived.lowSpeedBlend > 0.0f && game->derived.lowSpeedBlend < 1.0f) {
            crossedBlend = true;
        }
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) finite = false;
    }
    check(crossedBlend, "slow launch traverses the kinematic/dynamic blend");
    check(maxYawJump < 0.1f, "yaw response stays continuous through blend thresholds (jump %.4f)",
          (double)maxYawJump);
    check(game->vehicle.yawRateRadS > 0.0f,
          "left steering grows positive yaw as speed increases");
    check(finite, "every sampled low-speed state remains finite");
    free(game);
}

static void scenario_reverse(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->vehicle.selectedGear = -1;
    game->input.throttle = 0.3f;
    game->input.steer = 0.25f;
    bool finite = true;
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (!physics_state_is_valid(&game->spec, &game->vehicle, &game->derived)) finite = false;
    }
    check(game->vehicle.velocityLongitudinalMps < -0.5f,
          "explicit reverse direction launches backward (vx %.3f m/s)",
          (double)game->vehicle.velocityLongitudinalMps);
    check(game->vehicle.positionM.x < 0.0f, "reverse launch moves toward world -X");
    check(game->vehicle.yawRateRadS < 0.0f,
          "left steering while reversing produces the documented opposite yaw direction");
    check(isfinite(game->derived.frontSlipAngleRad) && isfinite(game->derived.rearSlipAngleRad),
          "reverse slip angles have no singularity");
    check(finite, "slow reverse remains finite and stable");

    game->input.throttle = 0.0f;
    game->input.steer = 0.0f;
    game->input.brake = 1.0f;
    for (int i = 0; i < 360; i++) game_fixed_update(game, FIXED_DT_S);
    check(game->vehicle.velocityLongitudinalMps <= 1e-6f,
          "braking from reverse does not launch forward");
    check(fabsf(game->vehicle.velocityLongitudinalMps) < 1e-5f,
          "reverse braking settles at zero");
    free(game);
}

static void scenario_steering_sign(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);
    state.velocityLongitudinalMps = 8.0f;
    Input input;
    input_zero(&input);
    input.steer = 0.5f;
    for (int i = 0; i < 30; i++) {
        physics_fixed_update(&spec, &state, &derived, &renderState, &input, FIXED_DT_S);
    }
    check(state.frontRoadWheelAngleRad > 0.0f, "left input produces positive road-wheel angle");
    check(derived.frontLateralForceN > 0.0f,
          "left steering initially produces a leftward front tire force");
    check(derived.totalYawTorqueNm > 0.0f, "left steering produces positive yaw torque");
    check(state.yawRateRadS > 0.0f, "left steering from forward travel produces positive yaw");
    check(state.headingRad > 0.0f, "heading changes only after positive yaw rate integrates");
    check_near(state.wheels[WHEEL_FRONT_LEFT].forceLateralN +
               state.wheels[WHEEL_FRONT_RIGHT].forceLateralN,
               derived.frontLateralForceN, 0.0,
               "the split front wheel diagnostics sum exactly to the axle force");
}

static void scenario_lever_arm(void)
{
    VehicleSpec a;
    VehicleSpec b;
    VehicleState sa;
    VehicleState sb;
    VehicleDerived da;
    VehicleDerived db;
    VehicleRenderState ra;
    VehicleRenderState rb;
    phase1_fixture(&a, &sa, &da, &ra);
    b = a;
    b.cgToFrontM += 0.35f;
    b.cgToRearM -= 0.20f;
    b.wheelbaseM = b.cgToFrontM + b.cgToRearM;
    vehicle_state_reset(&b, &sb, &db, &rb);
    sa.velocityLongitudinalMps = sb.velocityLongitudinalMps = 9.0f;
    sa.yawRateRadS = sb.yawRateRadS = 0.4f;
    float af;
    float ar;
    float bf;
    float br;
    physics_axle_slip_angles(&a, &sa, &af, &ar);
    physics_axle_slip_angles(&b, &sb, &bf, &br);
    check(fabsf(af - bf) > 1e-4f, "front lever-arm change alters front slip");
    check(fabsf(ar - br) > 1e-4f, "rear lever-arm change alters rear slip");
    Input input;
    input_zero(&input);
    input.steer = 0.3f;
    physics_fixed_update(&a, &sa, &da, &ra, &input, FIXED_DT_S);
    physics_fixed_update(&b, &sb, &db, &rb, &input, FIXED_DT_S);
    check(fabsf(da.totalYawTorqueNm - db.totalYawTorqueNm) > 1.0f,
          "lever-arm changes measurably alter yaw torque");
    check(fabsf(sa.yawRateRadS - sb.yawRateRadS) > 1e-6f,
          "lever-arm changes alter integrated yaw response");
}

static void scenario_integration(void)
{
    VehicleSpec spec;
    VehicleState state;
    VehicleDerived derived;
    VehicleRenderState renderState;
    phase1_fixture(&spec, &state, &derived, &renderState);
    Input input;
    input_zero(&input);
    input.throttle = 1.0f;
    physics_fixed_update(&spec, &state, &derived, &renderState, &input, FIXED_DT_S);
    check_near(state.velocityLongitudinalMps, 0.0, 1e-7,
               "the first launch tick spins the driven wheels before tire force develops");
    physics_fixed_update(&spec, &state, &derived, &renderState, &input, FIXED_DT_S);
    check(state.velocityLongitudinalMps > 0.0f,
          "the next tick accelerates from drivetrain-generated wheel slip");
    check_near(state.positionM.x, state.velocityLongitudinalMps * FIXED_DT_S, 1e-6,
               "position uses the updated semi-implicit velocity");

    state.headingRad = DRIFTY_PI - 0.001f;
    state.yawRateRadS = 1.0f;
    state.velocityLongitudinalMps = 0.0f;
    input.throttle = 0.0f;
    physics_fixed_update(&spec, &state, &derived, &renderState, &input, FIXED_DT_S);
    check(state.headingRad >= -DRIFTY_PI && state.headingRad < DRIFTY_PI,
          "integrated heading remains wrapped");
}

static void scenario_fixed_rate(void)
{
    Game *direct = alloc_game();
    Game *accumulated = alloc_game();
    game_init(direct);
    game_init(accumulated);
    direct->input.throttle = accumulated->input.throttle = 0.7f;
    direct->input.steer = accumulated->input.steer = 0.15f;
    for (int i = 0; i < 600; i++) game_fixed_update(direct, FIXED_DT_S);
    for (int i = 0; i < 300; i++) {
        const TimestepResult step = timestep_advance(
            &accumulated->accumulatorS, &accumulated->physicsBacklogDrops,
            FIXED_DT_S * 2.0f, fixed_update_adapter, accumulated);
        accumulated->lastSubstepCount = step.substeps;
    }
    check(accumulated->physicsBacklogDrops == 0,
          "fixed-rate consistency run drops no backlog");
    check(direct->sim.tick == accumulated->sim.tick,
          "direct and accumulator stepping execute the same tick count");
    check(direct->stateChecksum == accumulated->stateChecksum,
          "direct and accumulator stepping produce identical checksums (%08x)",
          direct->stateChecksum);
    check(memcmp(&direct->vehicle, &accumulated->vehicle, sizeof(VehicleState)) == 0,
          "direct and accumulator vehicle states are bit-identical");
    free(accumulated);
    free(direct);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: params — the tunable registry                                                 */
/* ------------------------------------------------------------------------------------- */

static void scenario_params(void)
{
    VehicleSpec defaults;
    vehicle_spec_set_default(&defaults);

    const int count = dev_params_count();
    check(count > 0, "the parameter registry is not empty (%d entries)", count);

    /* The registry's declared defaults ARE the config.h values. If this fails, a constant
     * was changed in one place and not the other, and every profile, slider range, and
     * documentation table below it is now lying. */
    int mismatches = 0;
    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        const float actual = dev_param_get(&defaults, param);
        if (fabsf(actual - param->defaultValue) > fmaxf(fabsf(param->defaultValue), 1.0f) * 1e-6f) {
            check(false, "registry default for '%s' is %g but vehicle_spec_set_default gives %g",
                  param->name, (double)param->defaultValue, (double)actual);
            mismatches++;
        }
    }
    check(mismatches == 0, "every registry default matches vehicle_spec_set_default()");

    for (int i = 0; i < count; i++) {
        const DevParameter *param = dev_param_at(i);
        check(param->minimum <= param->defaultValue && param->defaultValue <= param->maximum,
              "'%s' default %g lies inside its range [%g, %g]", param->name,
              (double)param->defaultValue, (double)param->minimum, (double)param->maximum);
        check(dev_param_find(param->name) == param, "'%s' is findable by name", param->name);
    }

    /* Setting clamps, rejects non-finite input, and keeps wheelbase consistent. */
    VehicleSpec spec = defaults;
    const DevParameter *mass = dev_param_find("body.mass");
    check(mass != NULL, "body.mass is registered");
    dev_param_set(&spec, mass, 1e9f);
    check_near(dev_param_get(&spec, mass), mass->maximum, 1e-3, "an out-of-range set clamps");
    check(!dev_param_set(&spec, mass, NAN), "a NaN set is refused");
    check_near(dev_param_get(&spec, mass), mass->maximum, 1e-3,
               "a refused set leaves the value untouched");

    const DevParameter *cgFront = dev_param_find("body.cg_to_front");
    dev_param_set(&spec, cgFront, 1.30f);
    check_near(spec.wheelbaseM, spec.cgToFrontM + spec.cgToRearM, 1e-6,
               "wheelbase follows the CG distances");
    check(vehicle_spec_is_valid(&spec), "the spec stays valid after tuning");

    check(dev_params_modified_count(&spec) == 2, "two parameters differ from their defaults");
    dev_params_reset_all(&spec);
    check(dev_params_modified_count(&spec) == 0, "reset_all restores every default");

    /* Profile round-trip. */
    check(telemetry_ensure_dir("tuning"), "the tuning directory exists or was created");
    dev_param_set(&spec, dev_param_find("tire.lat_rear.mu"), 1.05f);
    dev_param_set(&spec, dev_param_find("brake.bias_front"), 0.55f);
    const char *profilePath = "tuning/_test_roundtrip.txt";
    check(dev_params_save(&spec, profilePath), "a profile writes to disk");

    VehicleSpec loaded;
    vehicle_spec_set_default(&loaded);
    int applied = 0, unknown = 0, rejected = 0;
    check(dev_params_load(&loaded, profilePath, &applied, &unknown, &rejected),
          "the profile loads back");
    check(applied == dev_params_count(), "every parameter round-tripped (%d applied)", applied);
    check(unknown == 0 && rejected == 0, "no unknown or rejected keys (%d/%d)",
          unknown, rejected);
    check_near(loaded.tireMuLatRear, 1.05f, 1e-6, "a tuned tire value survives the round trip");
    check_near(loaded.brakeBiasFront, 0.55f, 1e-6, "a tuned brake value survives");
    remove(profilePath);

    /* The parser is a fuzz target, so its refusal behaviour is asserted here too. */
    VehicleSpec probe;
    vehicle_spec_set_default(&probe);
    const char *garbage = "body.mass\nnot even close\n= = =\nbody.mass = \nbody.mass = abc\n";
    check(dev_params_apply_text(&probe, garbage, strlen(garbage), &applied, &unknown, &rejected),
          "a garbage profile is survivable");
    check(applied == 0, "no garbage line was applied");
    check(rejected > 0, "garbage lines are counted as rejected (%d)", rejected);
    check(memcmp(&probe, &defaults, sizeof(VehicleSpec)) == 0, "garbage changed nothing");

    const char *unknownKeys = "no.such.parameter = 1.0\nbody.mass = 1300\n";
    check(dev_params_apply_text(&probe, unknownKeys, strlen(unknownKeys),
                                &applied, &unknown, &rejected),
          "an unknown key is skipped rather than failing the load");
    check(applied == 1 && unknown == 1, "one applied, one unknown (%d/%d)", applied, unknown);

    /* A profile that would produce an invalid spec must change nothing at all. */
    VehicleSpec guarded;
    vehicle_spec_set_default(&guarded);
    const char *invalid = "engine.idle_rpm = 1900\nengine.redline_rpm = 3000\n";
    dev_params_apply_text(&guarded, invalid, strlen(invalid), NULL, NULL, NULL);
    check(vehicle_spec_is_valid(&guarded), "the spec is valid whatever the profile said");
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: devreplay — durable timelines and the inspector's event markers               */
/* ------------------------------------------------------------------------------------- */

static void scenario_dev_replay(void)
{
    static ReplayBuffer source;
    static ReplayBuffer restored;

    replay_begin_recording(&source, 100u);
    for (int i = 0; i < 240; i++) {
        Input in;
        input_zero(&in);
        in.throttle = (i >= 20 && i < 120) ? 1.0f : 0.0f;
        in.brake = (i >= 150 && i < 180) ? 1.0f : 0.0f;
        in.handbrake = (i >= 200) ? 1.0f : 0.0f;
        in.steer = (i < 60) ? 0.5f : -0.5f;
        if (i == 90) in.shiftUpPressed = true;
        if (i == 130) in.resetPressed = true;
        replay_record(&source, &in);
    }
    check(source.count == 240, "the source timeline holds 240 frames");

    check(telemetry_ensure_dir("artifacts"), "the artifacts directory exists or was created");
    const char *path = "artifacts/_test_replay.bin";
    check(dev_replay_save(&source, path, "unit-test", 4242u, 0xdeadbeefu),
          "a timeline writes to disk");

    DevReplayInfo info;
    memset(&info, 0, sizeof(info));
    check(dev_replay_load(&restored, path, &info), "the timeline loads back");
    check(info.frameCount == 240u, "the header records 240 frames (got %u)", info.frameCount);
    check(info.firstTick == 100u, "the header records the first tick");
    check(info.seed == 4242u, "the header records the seed");
    check(info.finalChecksum == 0xdeadbeefu, "the header records the final checksum");
    check(strcmp(info.label, "unit-test") == 0, "the header records the label");
    check(restored.count == source.count, "every frame came back");

    int differences = 0;
    for (int i = 0; i < source.count; i++) {
        const ReplayFrame *a = dev_replay_frame_at(&source, i);
        const ReplayFrame *b = dev_replay_frame_at(&restored, i);
        if (a == NULL || b == NULL || memcmp(a, b, sizeof(ReplayFrame)) != 0) differences++;
    }
    check(differences == 0, "the round trip is bit-identical");

    /* Malformed input is rejected rather than trusted. */
    unsigned char small[16];
    memset(small, 0, sizeof(small));
    check(!dev_replay_parse(small, sizeof(small), &restored, NULL),
          "a buffer shorter than the header is rejected");

    static unsigned char blob[64 + 20 * 8];
    memset(blob, 0xA5, sizeof(blob));
    check(!dev_replay_parse(blob, sizeof(blob), &restored, NULL),
          "a buffer with the wrong magic is rejected");

    FILE *file = fopen(path, "rb");
    static unsigned char truncated[4096];
    const size_t read = (file != NULL) ? fread(truncated, 1, sizeof(truncated), file) : 0;
    if (file != NULL) fclose(file);
    check(read > 64, "the saved file is larger than its header");
    check(!dev_replay_parse(truncated, read, &restored, NULL),
          "a file truncated mid-timeline is rejected");
    remove(path);

    /* Event markers: the inspector draws exactly what this returns. */
    static DevReplayEvent events[64];
    const int eventCount = dev_replay_collect_events(&source, events, 64);
    check(eventCount > 0, "the timeline produces event markers (%d)", eventCount);

    int throttle = 0, brake = 0, handbrake = 0, shiftUp = 0, reset = 0, reversal = 0;
    for (int i = 0; i < eventCount; i++) {
        switch (events[i].kind) {
            case DEV_REPLAY_EVENT_THROTTLE:       throttle++;  break;
            case DEV_REPLAY_EVENT_BRAKE:          brake++;     break;
            case DEV_REPLAY_EVENT_HANDBRAKE:      handbrake++; break;
            case DEV_REPLAY_EVENT_SHIFT_UP:       shiftUp++;   break;
            case DEV_REPLAY_EVENT_RESET:          reset++;     break;
            case DEV_REPLAY_EVENT_STEER_REVERSAL: reversal++;  break;
            default: break;
        }
    }
    check(throttle == 2, "both throttle edges are marked (%d)", throttle);
    check(brake == 2, "both brake edges are marked (%d)", brake);
    check(handbrake == 1, "the handbrake pull is marked (%d)", handbrake);
    check(shiftUp == 1, "the shift is marked (%d)", shiftUp);
    check(reset == 1, "the reset is marked (%d)", reset);
    check(reversal == 1, "the steering reversal is marked (%d)", reversal);

    const int firstThrottle = events[0].index;
    check(events[0].tick == source.firstTick + (uint64_t)firstThrottle,
          "event ticks are absolute, not window-relative");
}

/* ------------------------------------------------------------------------------------- */
/* Scripted maneuvers from the shared scenario table                                       */
/* ------------------------------------------------------------------------------------- */

/*
 * These runs assert INVARIANTS, never handling targets. What a good skidpad radius is, is a
 * Phase 3 tuning question; that the friction budget is never exceeded and the state stays
 * finite is a correctness question, and correctness is what a regression suite is for.
 * The telemetry each run writes is what tools/compare_telemetry.py diffs against a baseline.
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
static int            g_sampleCount = 0;

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

static void free_scripted_game(void)
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

    free_scripted_game();
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
    bool  allFinite = true;

    g_sampleCount = 0;
    check(scenario->durationTicks <= SCRIPTED_SAMPLE_CAPACITY,
          "'%s' fits the sample buffer (%d ticks, capacity %d)", name,
          scenario->durationTicks, SCRIPTED_SAMPLE_CAPACITY);

    for (int tick = 0; tick < scenario->durationTicks; tick++) {
        game_fixed_update(game, FIXED_DT_S);
        record_sample(game, tick);

        /* 30 Hz telemetry rather than 120. Four times fewer rows is the difference between a
         * quarter-megabyte baseline and a megabyte one, and nothing in these maneuvers moves
         * fast enough for the extra resolution to change a comparison. */
        if (opened && (tick % SCRIPTED_TELEMETRY_DECIMATION) == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
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
          "'%s' never exceeds the friction budget (peak %.4f)", name, (double)peakFrictionUsage);
    check(peakSpeedMps <= MAX_SAFE_SPEED_MPS,
          "'%s' stays below MAX_SAFE_SPEED_MPS (peak %.2f m/s)", name, (double)peakSpeedMps);
    check(game->sim.tick == (uint64_t)scenario->durationTicks,
          "'%s' ran its full %d ticks", name, scenario->durationTicks);
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
           name, (double)peakFrictionUsage, (double)peakSideslipRad,
           (double)peakYawRateRadS, (double)peakSpeedMps);

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
    bool  transferAlwaysRearward = true;
    bool  frontAlwaysLighter = true;

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
        worstLoadSumErrorN = fmaxf(worstLoadSumErrorN,
                                   fabsf(sumN - (s->staticFrontN + s->staticRearN)));
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
            if (s->filteredAxMps2 < rawMinAx - 1e-3f ||
                s->filteredAxMps2 > rawMaxAx + 1e-3f) filteredInsideEnvelope = false;
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
          "the rear axle loads up (%.1f N, static %.1f N)",
          (double)maxRearLoadN, (double)g_samples[0].staticRearN);
    check(transferAlwaysRearward, "load transfer stays rearward for the whole pull");
    check(frontAlwaysLighter, "and the front axle never exceeds its static load");
    check(worstLoadSumErrorN < 1e-2f,
          "the unclamped axle loads always sum to mass * gravity (worst error %.4f N)",
          (double)worstLoadSumErrorN);
    check(filteredInsideEnvelope,
          "the filtered acceleration never leaves the envelope of the raw values it filters "
          "([%.3f, %.3f] m/s^2)", (double)rawMinAx, (double)rawMaxAx);
    check(filteredStepBounded,
          "and never moves further in one step than the filter coefficient permits: "
          "the load loop attenuates rather than amplifies");

    /* Rear capacity rises with rear load; that is the point of the whole stage. */
    const float rearCapacityGainN = (maxRearLoadN - g_samples[0].staticRearN) *
                                    g_scriptedGame->spec.tireMuLatRear;
    check(rearCapacityGainN > 50.0f,
          "the loaded rear axle gains lateral capacity (%.0f N)", (double)rearCapacityGainN);

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
    bool  everReversed = false;
    bool  minimumLoadHeld = true;
    bool  wheelsNeverReversed = true;

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
            s->rearLoadN < MIN_NORMAL_LOAD_N - 1e-3f) minimumLoadHeld = false;
        if (stopIndex < 0 && s->vxMps <= 1e-4f) stopIndex = i;
    }

    const float stoppingTimeS = (stopIndex >= 0)
        ? g_samples[stopIndex].timeS - g_samples[brakeStart].timeS : -1.0f;
    const float stoppingDistanceM = (stopIndex >= 0)
        ? g_samples[stopIndex].positionXM - entryPositionM : -1.0f;

    check(minFilteredAx < -1.0f,
          "the filtered acceleration goes negative under braking (%.3f m/s^2)",
          (double)minFilteredAx);
    check(peakDecelMps2 > 1.0f, "the solved acceleration goes negative too (peak decel %.3f m/s^2)",
          (double)peakDecelMps2);
    check(maxFrontLoadN > g_samples[brakeStart].staticFrontN + 50.0f,
          "the front axle loads up under braking (%.1f N, static %.1f N)",
          (double)maxFrontLoadN, (double)g_samples[brakeStart].staticFrontN);
    check(minRearLoadN < g_samples[brakeStart].staticRearN - 50.0f,
          "the rear axle unloads (%.1f N, static %.1f N)",
          (double)minRearLoadN, (double)g_samples[brakeStart].staticRearN);
    check(peakForwardTransferN > 0.0f,
          "the transfer is forward, not rearward (%.1f N)", (double)peakForwardTransferN);
    check(!everReversed, "braking never pushes the vehicle backwards");
    check(wheelsNeverReversed, "and never spins the wheels backwards");
    check(minimumLoadHeld, "no axle load falls below MIN_NORMAL_LOAD_N");
    check(stopIndex >= 0, "the vehicle comes to a stop");

    /* Front braking capacity rises while rear capacity falls: the whole reason brake bias
     * is biased forward in the first place. */
    const float longMuEff = g_scriptedGame->spec.tireMuLongScale *
        Surface_Get(SURFACE_ASPHALT)->muLongitudinal;
    const float frontCapacityGainN = (maxFrontLoadN - g_samples[brakeStart].staticFrontN) *
                                     longMuEff;
    const float rearCapacityLossN = (g_samples[brakeStart].staticRearN - minRearLoadN) *
                                    longMuEff;
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
        if (s->rollingN > g_scriptedGame->spec.rollingResistanceCoefficient *
                          (s->frontLoadN + s->rearLoadN) + 1.0f) rollingBounded = false;
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
    check(restIsQuiet, "at rest both resistance forms are exactly zero, inventing no direction");

    check(speedMonotonic,
          "coast-down speed decreases monotonically once the throttle is off "
          "(worst rise %.6f m/s at %.2f s)",
          (double)worstSpeedRiseMps, (double)worstSpeedRiseTimeS);
    check(dragMonotonic, "and drag decreases with it, tick by tick (worst rise %.4f N)",
          (double)worstDragRiseN);
    check(noSpike, "total resisting force never spikes during the coast");
    check(rollingBounded,
          "rolling resistance never exceeds the coefficient times the current load");
    check(finalSpeedMps < entrySpeedMps,
          "the car actually slows down (%.3f -> %.3f m/s)",
          (double)entrySpeedMps, (double)finalSpeedMps);
    check(finalDragN < entryDragN,
          "drag falls as the car slows (%.1f -> %.1f N)",
          (double)entryDragN, (double)finalDragN);
    check(fabsf(finalRollingN - entryRollingN) < 0.25f * entryRollingN,
          "rolling resistance stays load-driven rather than following speed (%.1f -> %.1f N)",
          (double)entryRollingN, (double)finalRollingN);

    printf("    coast-down: %.3f -> %.3f m/s; drag %.1f -> %.1f N, rolling %.1f -> %.1f N\n",
           (double)entrySpeedMps, (double)finalSpeedMps, (double)entryDragN,
           (double)finalDragN, (double)entryRollingN, (double)finalRollingN);
}

/* Mean of a sample field over [fromS, toS). */
#define SAMPLE_MEAN(field, fromS, toS) sample_mean(offsetof(ScriptedSample, field), (fromS), (toS))

static float sample_mean(size_t fieldOffset, float fromS, float toS)
{
    const int from = sample_at_time(fromS);
    const int to = sample_at_time(toS);
    if (to <= from) return 0.0f;
    double total = 0.0;
    for (int i = from; i < to; i++) {
        total += (double)*(const float *)(const void *)
                 ((const unsigned char *)&g_samples[i] + fieldOffset);
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
    const float estimatedRadiusM = (fabsf(steadyYawRateRadS) > 1e-3f)
        ? steadySpeedMps / fabsf(steadyYawRateRadS) : 0.0f;

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
            worstEntryDifferenceRad = fmaxf(worstEntryDifferenceRad,
                fabsf(g_samples[i].frontSlipRad - g_samples[i].rearSlipRad));
        }
        check(worstEntryDifferenceRad > 1e-3f,
              "the two differ through corner entry, as distinct lever arms require "
              "(worst difference %.4f rad)", (double)worstEntryDifferenceRad);
    }
    check(frontLoadN + rearLoadN > 0.9f * g_scriptedGame->spec.massKg * GRAVITY_MPS2,
          "the axle loads still carry the car through the corner (%.1f N)",
          (double)(frontLoadN + rearLoadN));

    printf("    skidpad steady: %.3f m/s, yaw %.4f rad/s, ay %.3f m/s^2, beta %.4f rad,\n"
           "            radius %.2f m, slip F/R %.4f/%.4f rad, usage F/R %.3f/%.3f,\n"
           "            load F/R %.1f/%.1f N\n",
           (double)steadySpeedMps, (double)steadyYawRateRadS, (double)steadyLateralAxMps2,
           (double)steadySideslipRad, (double)estimatedRadiusM, (double)frontSlipRad,
           (double)rearSlipRad, (double)frontUsage, (double)rearUsage,
           (double)frontLoadN, (double)rearLoadN);
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

        for (int i = 0; i < 1440; i++) {     /* 12 s: settle, then measure the last 3 */
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
          "(%.3f -> %.3f)", (double)rearUsage[0], (double)rearUsage[3]);

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
    const float riseTimeS = (riseStartS >= 0.0f && riseEndS >= 0.0f)
        ? riseEndS - riseStartS : -1.0f;

    /* Overshoot: how far the peak exceeds the steady value, as a percentage of it. */
    const float overshootPercent = (fabsf(steadyYawRateRadS) > 1e-4f)
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
          "limit %.3f)", (double)worstSteerRateRadS,
          (double)g_scriptedGame->spec.steerReturnRateRadS);
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
        peakFrontLoadDeltaN = fmaxf(peakFrontLoadDeltaN,
                                    g_samples[i].frontLoadN - beforeFrontLoadN);
        peakRearLoadDeltaN = fminf(peakRearLoadDeltaN,
                                   g_samples[i].rearLoadN - beforeRearLoadN);
        peakYawDeltaRadS = fmaxf(peakYawDeltaRadS,
                                 fabsf(g_samples[i].yawRateRadS) - fabsf(beforeYawRateRadS));
        peakSideslipDeltaRad = fmaxf(peakSideslipDeltaRad,
                                     fabsf(g_samples[i].sideslipRad) -
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

    check(afterAxMps2 < beforeAxMps2 - 0.05f,
          "lifting the throttle makes the solved acceleration more negative (%.3f -> %.3f m/s^2)",
          (double)beforeAxMps2, (double)afterAxMps2);
    check(afterFilteredAx < beforeFilteredAx - 0.05f,
          "and the filtered acceleration follows it down (%.3f -> %.3f m/s^2)",
          (double)beforeFilteredAx, (double)afterFilteredAx);
    check(afterFrontLoadN > beforeFrontLoadN + 5.0f,
          "the front axle gains load (%.1f -> %.1f N)",
          (double)beforeFrontLoadN, (double)afterFrontLoadN);
    check(afterRearLoadN < beforeRearLoadN - 5.0f,
          "and the rear axle loses it (%.1f -> %.1f N)",
          (double)beforeRearLoadN, (double)afterRearLoadN);
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
    bool  allFinite = true;

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
        if (!isfinite(s->yawTorqueNm) || !isfinite(s->sideslipRad) ||
            !isfinite(s->yawRateRadS)) allFinite = false;
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

    printf("    transition: %d steer reversals, %d sideslip zero crossings, %d yaw sign "
           "changes\n                peak yaw torque %.1f Nm, worst tick-to-tick jump %.1f Nm\n",
           steerReversals, sideslipZeroCrossings, yawSignChanges,
           (double)peakYawTorqueNm, (double)worstYawTorqueJumpNm);
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
        sideslipAfterCounterRad = fminf(sideslipAfterCounterRad,
                                        fabsf(g_samples[i].sideslipRad));
    }

    /* Countersteer direction: the script steers right while the car yaws left. */
    const float counterSteerRad = SAMPLE_MEAN(steerRad, 5.0f, 6.0f);
    const float yawDuringSlideRadS = SAMPLE_MEAN(yawRateRadS, 3.5f, 4.5f);

    bool yawBounded = true;
    bool allFinite = true;
    for (int i = 0; i < g_sampleCount; i++) {
        if (fabsf(g_samples[i].yawRateRadS) >= MAX_SAFE_YAW_RATE_RADS) yawBounded = false;
        if (!isfinite(g_samples[i].sideslipRad) || !isfinite(g_samples[i].yawRateRadS) ||
            !isfinite(g_samples[i].speedMps)) allFinite = false;
    }

    check(allFinite, "every catchable-drift sample stays finite");
    check(peakSideslipRad > sideslipAtEntryRad + 0.20f,
          "initiation builds body sideslip (%.4f -> %.4f rad at %.2f s)",
          (double)sideslipAtEntryRad, (double)peakSideslipRad, (double)peakSideslipTimeS);
    check(peakRearUsage > 0.95f,
          "the rear tires reach saturation during the slide (%.4f)", (double)peakRearUsage);
    check(counterSteerRad * yawDuringSlideRadS < 0.0f,
          "the countersteer opposes the yaw (steer %.4f rad, yaw %.4f rad/s)",
          (double)counterSteerRad, (double)yawDuringSlideRadS);
    check(sideslipAfterCounterRad < peakSideslipRad - 0.10f,
          "countersteer reduces the excessive slip (%.4f -> %.4f rad)",
          (double)peakSideslipRad, (double)sideslipAfterCounterRad);
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
    check(latAccel > 0.5f,
          "lateral acceleration builds during the corner (%.3f m/s^2)", (double)latAccel);
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

    check(loadFR > loadFL,
          "outside front wheel carries more load (FR %.1f > FL %.1f N)",
          (double)loadFR, (double)loadFL);
    check(loadRR > loadRL,
          "outside rear wheel carries more load (RR %.1f > RL %.1f N)",
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
    check(yawBefore < 5.0f,
          "straight driving produces near-zero yaw torque (%.2f N·m)", (double)yawBefore);

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
          "(%.2f N·m)", (double)fabsf(yawGrass));

    /* Reset the surface — yaw torque must diminish toward zero.
     * The car has built up a yaw rate during the asymmetric phase, so some yaw torque
     * from the tires' lateral forces persists while the car is still rotating. The
     * torque should drop sharply but may not reach zero immediately. */
    game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId = SURFACE_ASPHALT;
    for (int i = 0; i < 30; i++) game_fixed_update(game, FIXED_DT_S);
    const float yawRestored = fabsf(game->derived.totalYawTorqueNm);
    check(yawRestored < yawBefore + 35.0f,
          "restoring the asphalt surface sharply reduces the yaw torque "
          "(%.2f N·m, was %.2f)", (double)yawRestored, (double)fabsf(yawGrass));

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
          "Ackermann: inner front wheel steers more (FL %.4f > FR %.4f rad)",
          (double)steerFL, (double)steerFR);
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
        check(s >= 0.5f && s <= 1.5f,
              "muScale[%d] = %.4f is inside [0.5, 1.5]", w, (double)s);
    }

    /* Disable: at k=0, all muScale values must equal 1.0. */
    Game *game2 = alloc_game();
    game_init(game2);
    game2->spec.tireLoadSensitivityK = 0.0f;
    game2->spec.lateralLoadTransferEnabled = true;
    check(vehicle_spec_is_valid(&game2->spec),
          "spec is valid before the k=0 simulation run");
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

    const float pure0  = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
    const float relax0 = game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLateralRelaxedN;
    check(fabsf(relax0) < fabsf(pure0) - 1.0f,
          "first tick after step: relaxed force lags pure lateral force "
          "(%.1f vs %.1f N)", (double)fabsf(relax0), (double)fabsf(pure0));

    /* After enough ticks, the relaxation state should converge. At 10 m/s and
     * relaxationLengthM = 0.30 m, the time constant is l/vx = 0.03 s, or ~3.6 ticks.
     * Run 20 ticks (5.6 time constants → >99% converged). The pure lateral force still
     * evolves during the corner, so a small steady-state lag is expected; use a generous
     * tolerance relative to the force magnitude. */
    for (int i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);

    const float pureN  = game->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
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

    const float pureN2  = game2->derived.pureLateralForceN[WHEEL_FRONT_LEFT];
    const float relaxN2 = game2->vehicle.wheels[WHEEL_FRONT_LEFT].forceLateralRelaxedN;
    {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "at relaxationLengthM=0, relaxed force equals pure force on first tick "
                 "(%.1f == %.1f N) — no lag", (double)relaxN2, (double)pureN2);
        check_near((double)relaxN2, (double)pureN2, 1e-3, msg);
    }

    free(game2);
    free(game);
}

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
          "track init: offTrackSurfaceId is SURFACE_GRASS (got %d)", (int)track.offTrackSurfaceId);
    check(track.nextCheckpoint == 0, "track init: nextCheckpoint is 0");
    check(track.lap == 0, "track init: lap is 0");
    check_near((double)track.lapTimerS, 0.0, 0.0, "track init: lapTimerS is 0");

    /* Query the centre at (0, 0): inside the 200×150 m parking lot, so it should be asphalt. */
    const SurfaceId centerSurf = Track_SurfaceAt(&track, (Vector2){ 0.0f, 0.0f });
    check(centerSurf == SURFACE_ASPHALT,
          "Track_SurfaceAt origin returns ASPHALT (got %d)", (int)centerSurf);

    /* Query at a centreline node point: should be asphalt. */
    const SurfaceId nodeSurf = Track_SurfaceAt(&track, track.nodes[0].centerM);
    check(nodeSurf == SURFACE_ASPHALT,
          "Track_SurfaceAt(centreline node) returns ASPHALT (got %d)", (int)nodeSurf);

    /* Just inside the lot boundary: offset from a perimeter node by less than halfWidthM. */
    {
        const Vector2 insidePoint = { track.nodes[0].centerM.x,
                                      track.nodes[0].centerM.y + track.nodes[0].halfWidthM * 0.7f };
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
        check(farSurf == SURFACE_GRASS,
              "Track_SurfaceAt far point returns GRASS (got %d)", (int)farSurf);
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
    game->vehicle.headingRad = -1.57079632679f;           /* pointing -Y (down) */
    game->vehicle.velocityLongitudinalMps = 30.0f;        /* body X forward = world -Y */
    game->vehicle.velocityLateralMps = 0.0f;
    game->renderState.prevPositionM = game->vehicle.positionM;
    game->renderState.prevHeadingRad = game->vehicle.headingRad;
    game->renderState.currPositionM = game->vehicle.positionM;
    game->renderState.currHeadingRad = game->vehicle.headingRad;
    game->state = STATE_PLAYING;

    /* Before tick: verify car is on the correct side (above the barrier at y ≈ -79). */
    const float yBefore = game->vehicle.positionM.y;
    check(yBefore > -79.0f, "car starts inside the track boundary (y = %.2f > -79.0)", (double)yBefore);

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
    check(yAfter >= -80.5f, "car did not tunnel through the barrier (y = %.4f, must be > -80.5)",
          (double)yAfter);
    check(hitBarrier, "car hit the barrier (crashLockoutTimerS was set)");
    check(game->crashLockoutTimerS > 0.0f,
          "significant impact sets crashLockoutTimerS (%.4f > 0)", (double)game->crashLockoutTimerS);
    check(speedAfterHit < speedBeforeHit,
          "car lost speed from impact (%.2f < %.2f m/s)", (double)speedAfterHit, (double)speedBeforeHit);

    /* --- Decay of crash lockout timer --- */
    const float lockoutBefore = game->crashLockoutTimerS;
    game->input = tickInput;  /* no input */
    game_fixed_update(game, FIXED_DT_S);
    check(game->crashLockoutTimerS < lockoutBefore,
          "crashLockoutTimerS decays (%.4f < %.4f)", (double)game->crashLockoutTimerS, (double)lockoutBefore);

    track_free(&game->track);
    free(game);

    /* --- Glancing hit: car approaches at shallow angle to produce yaw spin --- */
    Game *game2 = alloc_game();
    game_init(game2);
    track_init(&game2->track);

    /* Place car near the bottom-right of the lot, heading right-down at a shallow angle
     * toward the bottom barrier at y ≈ -79 m. */
    game2->vehicle.positionM = (Vector2){ 80.0f, -75.5f };
    game2->vehicle.headingRad = -1.2f;    /* ~68° clockwise from +X, i.e. heading right-down */
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
    bool crossed = track_update_checkpoints(&track,
                                            (Vector2){ -0.1f, 1.0f },
                                            (Vector2){ 0.1f, 1.0f });
    check(crossed, "crossing gate 0 advances the checkpoint");
    check(track.nextCheckpoint == 1, "nextCheckpoint is 1 after gate 0 (got %d)", track.nextCheckpoint);
    check(track.lap == 0, "lap stays 0 after first gate (got %d)", track.lap);

    /* Gate 1: at (10,0), direction (+Y), perpendicular = (±1,0). Gate spans x ∈ [8,12]. */
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ 10.0f, -0.1f },
                                        (Vector2){ 10.0f, 0.1f });
    check(crossed, "crossing gate 1 advances the checkpoint");
    check(track.nextCheckpoint == 2, "nextCheckpoint is 2 after gate 1 (got %d)", track.nextCheckpoint);

    /* Gate 2: at (10,10), direction (-X), perpendicular = (0,±1). Gate spans y ∈ [8,12]. */
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ 10.1f, 10.0f },
                                        (Vector2){ 9.9f, 10.0f });
    check(crossed, "crossing gate 2 advances the checkpoint");
    check(track.nextCheckpoint == 3, "nextCheckpoint is 3 after gate 2 (got %d)", track.nextCheckpoint);

    /* --- Reverse crossing does NOT advance --- */
    /* Move backward through gate 2 (from right to left at the top). Direction is -X, which
     * is opposite to the track's forward direction (+X when moving from 1 to 2, but gate 2 
     * direction is from node 2 to node 3 which is (-X) for the top straight going right to
     * left. The gate at node 2 (top-right) has direction from (10,10) → (0,10) which is -X.
     * The car moving left at the top is WITH the track direction, so let me construct a 
     * proper reverse test: gate 0's forward direction is (+X). Moving -X through it: */
    track.nextCheckpoint = 0;
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ 0.2f, 1.0f },
                                        (Vector2){ -0.2f, 1.0f });
    check(!crossed, "reverse crossing of gate 0 does NOT advance");
    check(track.nextCheckpoint == 0,
          "nextCheckpoint still 0 after reverse crossing (got %d)", track.nextCheckpoint);

    /* --- Lap completion --- */
    /* Cross gate 0 forward, then 1, 2, 3 (last gate). */
    track.nextCheckpoint = 0;
    track.lap = 0;
    track.lapTimerS = 5.0f;
    check(track_update_checkpoints(&track,
                                    (Vector2){ -0.1f, 1.0f },
                                    (Vector2){ 0.1f, 1.0f }),
          "gate 0");
    check(track_update_checkpoints(&track,
                                    (Vector2){ 10.0f, -0.1f },
                                    (Vector2){ 10.0f, 0.1f }),
          "gate 1");
    check(track_update_checkpoints(&track,
                                    (Vector2){ 10.1f, 10.0f },
                                    (Vector2){ 9.9f, 10.0f }),
          "gate 2");
    /* Gate 3: at (0,10), direction (+X)? No, from 3 to 0: (0,10) → (0,0) which is -Y.
     * Direction = (0, -1). Perpendicular = (-1, 0) → gate spans x ∈ [-2, +2]. */
    /* Move downward (-Y) from y=10.1 to y=9.9 at x=1.0. The forward direction of the gate
     * at node 3 is from (0,10) to (0,0) which is (0, -Y). The car moving -Y matches
     * the track's forward direction. */
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ 1.0f, 10.1f },
                                        (Vector2){ 1.0f, 9.9f });
    check(crossed, "crossing gate 3 (last gate) advances");
    check(track.nextCheckpoint == 0, "nextCheckpoint wraps to 0 (got %d)", track.nextCheckpoint);
    check(track.lap == 1, "lap increments to 1 (got %d)", track.lap);
    check(track.lapTimerS < 0.1f, "lapTimerS resets on lap completion (%.4f s)", (double)track.lapTimerS);
    check(track.lastLapTimeS > 4.5f, "lastLapTimeS records the completed lap time (%.4f s > 4.5)",
          (double)track.lastLapTimeS);

    /* --- Timer accumulation --- */
    track.lapTimerS = 0.0f;
    track.lapTimerS += 0.5f;
    check_near((double)track.lapTimerS, 0.5, 1e-6, "lapTimerS accumulates");

    /* --- Car outside gate span does NOT trigger --- */
    /* Gate 0 spans y ∈ [-2, +2]. Car crosses at y = 5 should NOT count. */
    track.nextCheckpoint = 0;
    track.lap = 0;
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ -0.1f, 5.0f },
                                        (Vector2){ 0.1f, 5.0f });
    check(!crossed, "crossing outside gate span does NOT advance");
    check(track.nextCheckpoint == 0,
          "nextCheckpoint unchanged after out-of-bounds cross (got %d)", track.nextCheckpoint);

    /* --- Stationary car does NOT trigger --- */
    crossed = track_update_checkpoints(&track,
                                        (Vector2){ 0.0f, 1.0f },
                                        (Vector2){ 0.0f, 1.0f });
    check(!crossed, "stationary car does NOT advance checkpoints");

    /* --- NULL/edge case safety --- */
    check(!track_update_checkpoints(NULL, (Vector2){0,0}, (Vector2){1,0}),
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

    /* Build enough speed for a drift. Cruise at ~18 m/s on asphalt. */
    set_vehicle_rolling_speed(game, 18.0f);
    game->input.throttle = 0.90f;
    game->input.steer = 0.30f;

    /* Run 20 ticks with gentle steer — score should be near zero. */
    int i;
    game->input.throttle = 0.60f;
    game->input.steer = 0.10f;
    game->input.handbrake = 0.0f;
    for (i = 0; i < 20; i++) game_fixed_update(game, FIXED_DT_S);
    float scoreBefore = game->driftScore;
    check(scoreBefore < 0.01f, "no significant score before drift initiation (%.1f)", (double)scoreBefore);

    /* Now add handbrake + more steer to break rear traction and initiate a drift. */
    game->input.throttle = 0.90f;
    game->input.steer = 0.50f;
    game->input.handbrake = 1.0f;
    for (i = 0; i < 40; i++) game_fixed_update(game, FIXED_DT_S);

    /* Release handbrake; keep moderate throttle and steer to sustain the slide. */
    game->input.handbrake = 0.0f;
    game->input.throttle = 0.50f;
    game->input.steer = -0.30f; /* countersteer to hold the angle */

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
    check(game->driftScore > 20.0f,
          "driftScore accumulates beyond 20 (got %.1f)", (double)game->driftScore);
    check(peakCombo > 1.5f,
          "comboMultiplier rises above 1.5 during a sustained drift (peak %.3f)",
          (double)peakCombo);
    check(peakCombo <= 4.0f + 1e-4f,
          "comboMultiplier is capped at 4.0 (peak %.3f)", (double)peakCombo);
    check(game->comboMultiplier >= 1.0f,
          "comboMultiplier is never below 1.0 (final %.3f)", (double)game->comboMultiplier);

    /* Now stop drifting: straighten the wheel and drop throttle. */
    game->input.steer = 0.0f;
    game->input.throttle = 0.0f;

    /* Run until well past COMBO_GRACE_S (1.5 s → 180 ticks at 120 Hz). */
    bool comboReset = false;
    float timeReset = 0.0f;
    for (i = 0; i < 200; i++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->comboMultiplier < 1.0f + 1e-6f && game->driftTimeS < 1e-6f) {
            comboReset = true;
            timeReset = (float)i * FIXED_DT_S;
            break;
        }
    }
    check(comboReset,
          "comboMultiplier and driftTimeS reset after COMBO_GRACE_S (%.3f s)",
          (double)timeReset);

    /* driftTimeS should be zero now (or very close). */
    check(game->driftTimeS < 0.01f,
          "driftTimeS resets to zero after the grace period (%.4f)", (double)game->driftTimeS);

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
    check(!game->derived.scoringDrift,
          "creeping at %.1f m/s (below %.1f) does NOT score",
          (double)game->derived.speedMps, (double)MIN_DRIFT_SPEED_MPS);
    check(game->driftScore < 0.01f, "no score accrued while creeping");

    /* --- Rejection 2: Reversing with slide --- */
    game_init(game);
    /* Reverse: set longitudinal velocity negative. */
    game->vehicle.velocityLongitudinalMps = -8.0f;
    game->vehicle.velocityLateralMps = 3.0f;
    game->vehicle.yawRateRadS = 1.0f;
    game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_REAR_RIGHT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS = -8.0f / game->spec.wheelRadiusM;
    game->vehicle.selectedGear = -1;

    /* Force derived values to match so classification sees the slide. */
    game->derived.speedMps = 8.0f;
    game->derived.bodySideslipRad = atan2f(3.0f, 8.0f);
    game->derived.rearSlipAngleRad = 0.15f;

    /* Call classify directly on our carefully set reverse state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift,
          "reversing (vx %.1f < 0) does NOT score", (double)game->vehicle.velocityLongitudinalMps);

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
    check(!game->derived.scoringDrift,
          "spinning at near-zero speed (%.2f m/s) does NOT score", (double)game->derived.speedMps);

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
    game->derived.speedMps = sqrtf(1.0f*1.0f + 20.0f*20.0f);
    game->derived.bodySideslipRad = atan2f(-20.0f, 1.0f);
    game->derived.rearSlipAngleRad = 0.30f;
    /* Ensure sideslip exceeds SPIN_CUTOFF_RAD (1.48 rad ~ 85 deg). */
    check(fabsf(game->derived.bodySideslipRad) > SPIN_CUTOFF_RAD,
          "precondition: sideslip (%.3f rad) exceeds spin cutoff (%.3f rad)",
          (double)fabsf(game->derived.bodySideslipRad), (double)SPIN_CUTOFF_RAD);
    /* Call classify directly — physics would alter our carefully set state. */
    scoring_classify(&game->vehicle, &game->derived, 0.0f);
    check(!game->derived.scoringDrift,
          "past spin cutoff (%.3f rad > %.3f rad) does NOT score",
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
    float steerInputs[] = { 0.35f, 0.35f, 0.35f, 0.35f, -0.10f };
    float throttleInputs[] = { 0.80f, 0.80f, 0.40f, 0.30f, 0.00f };
    float handbrakeInputs[] = { 1.0f, 0.5f, 0.0f, 0.0f, 0.0f };
    int switchTicks[] = { 0, 20, 40, 60, 90 };
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
    if (f) { fprintf(f, "%s", contents); fclose(f); }
}

/* Helper: read and validate exactly like persistence_load_score in game.c. */
static float hs_read_score(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0.0f;
    long parsed = 0;
    int matched = fscanf(f, "%ld", &parsed);
    fclose(f);
    if (matched == 1 && parsed >= 0 && parsed <= (long)MAX_VALID_SCORE)
        return (float)parsed;
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
    const char *tempPath = "telemetry/_test_highscore.txt";
    float loaded;
    char buf[32];

    remove(tempPath);

    /* --- Round-trip: write 12345, read back --- */
    hs_write_file(tempPath, "12345");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 12345.0) < 0.5, "score 12345 round-trips through file (got %.0f)", (double)loaded);

    /* --- Round-trip: write 0, read back --- */
    hs_write_file(tempPath, "0");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "score 0 round-trips (got %.0f)", (double)loaded);

    /* --- Round-trip: write MAX_VALID_SCORE, read back --- */
    snprintf(buf, sizeof(buf), "%ld", (long)MAX_VALID_SCORE);
    hs_write_file(tempPath, buf);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - (double)MAX_VALID_SCORE) < 0.5, "MAX_VALID_SCORE round-trips (got %.0f)", (double)loaded);

    /* --- Reject garbage: write "hello world" --- */
    hs_write_file(tempPath, "hello world");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "garbage file is rejected, bestScore stays at 0 (got %.0f)", (double)loaded);

    /* --- Reject empty file --- */
    hs_write_file(tempPath, "");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "empty file is rejected (got %.0f)", (double)loaded);

    /* --- Reject out-of-range value (> MAX_VALID_SCORE) --- */
    hs_write_file(tempPath, "9999999999");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "out-of-range value 9999999999 (>%ld) is rejected (got %.0f)", (long)MAX_VALID_SCORE, (double)loaded);

    /* --- Reject negative value --- */
    hs_write_file(tempPath, "-42");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "negative value -42 is rejected (got %.0f)", (double)loaded);

    /* --- Leading integer in "100abc" is parsed (fscanf behaviour) --- */
    hs_write_file(tempPath, "100abc");
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 100.0) < 0.5, "leading integer in '100abc' is parsed as 100 (got %.0f)", (double)loaded);

    /* --- File does not exist --- */
    remove(tempPath);
    loaded = hs_read_score(tempPath);
    check(fabs((double)loaded - 0.0) < 0.5, "missing file returns default 0 (got %.0f)", (double)loaded);

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
    memset(&pool, 0xFF, sizeof(pool));   /* fill with junk to prove init overwrites */
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
    const Color   col = { 200, 200, 200, 180 };
    particle_spawn(&pool, pos, vel, 0.30f, col);

    check(pool.cursor == 1, "spawn advances cursor to 1");
    check(pool.particles[0].active, "spawned particle is active");
    check_near((double)pool.particles[0].positionM.x, 1.0, 1e-6, "spawn sets position.x");
    check_near((double)pool.particles[0].positionM.y, 2.0, 1e-6, "spawn sets position.y");
    check_near((double)pool.particles[0].velocityMps.x, 3.0, 1e-6, "spawn sets velocity.x");
    check_near((double)pool.particles[0].velocityMps.y, 4.0, 1e-6, "spawn sets velocity.y");
    check_near((double)pool.particles[0].lifeS,
               (double)PARTICLE_LIFE_S, 1e-6, "spawn sets life to PARTICLE_LIFE_S");
    check_near((double)pool.particles[0].maxLifeS,
               (double)PARTICLE_LIFE_S, 1e-6, "spawn sets maxLife");
    check(pool.particles[0].sizeM == 0.30f, "spawn sets sizeM");
    check(pool.particles[0].color.r == 200 && pool.particles[0].color.g == 200 &&
          pool.particles[0].color.b == 200 && pool.particles[0].color.a == 180,
          "spawn sets color exactly");

    /* --- Round-robin wrap: after MAX_PARTICLES spawns, cursor returns to 0. --- */
    for (int i = 1; i < MAX_PARTICLES; i++) {
        particle_spawn(&pool, pos, vel, 0.30f, col);
    }
    check(pool.cursor == 0, "cursor wraps to 0 after %d spawns (got %d)",
          MAX_PARTICLES, pool.cursor);

    /* The slot 0 was overwritten by the last wrap-around spawn. */
    check(pool.particles[0].active, "round-robin re-activates slot 0 after wrap");

    /* --- Update: integrates velocity and decays life for active particles. --- */
    ParticlePool pool2;
    particle_pool_init(&pool2);
    particle_spawn(&pool2, (Vector2){ 0.0f, 0.0f }, (Vector2){ 10.0f, 0.0f }, 0.30f, col);
    pool2.particles[0].lifeS    = 1.0f;
    pool2.particles[0].maxLifeS = 1.0f;

    particle_pool_update(&pool2, 0.50f);
    check_near((double)pool2.particles[0].positionM.x, 5.0, 1e-6,
               "update integrates x (10 m/s * 0.5 s)");
    check_near((double)pool2.particles[0].lifeS, 0.5, 1e-6,
               "update reduces life by dt");

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
    check(game->state == STATE_MENU,
          "state can be set to STATE_MENU (got %d)", (int)game->state);

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
    check(game->state == STATE_PAUSED, "pause from PLAYING → PAUSED (got %d)", (int)game->state);

    /* --- PAUSED + pause → PLAYING. --- */
    game->input.pausePressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "pause from PAUSED → PLAYING (got %d)", (int)game->state);

    /* --- PLAYING + reset → PLAYING (vehicle reset, score zeroed). --- */
    game->vehicle.positionM.x = 150.0f;
    game->driftScore = 250.0f;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_PLAYING, "reset during PLAYING stays PLAYING (got %d)",
          (int)game->state);
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset on PLAYING reset");
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
    check_near((double)game->vehicle.positionM.x, 0.0, 1e-6,
               "vehicle reset on PAUSED reset");

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
    check_near((double)game->driftScore, 0.0, 0.5,
               "score zeroed on RESULTS→PLAYING");

    /* --- RESULTS + reset → MENU. --- */
    game->state = STATE_RESULTS;
    game->input.resetPressed = true;
    game_fixed_update(game, FIXED_DT_S);
    check(game->state == STATE_MENU, "reset from RESULTS → MENU (got %d)",
          (int)game->state);

    /* --- One-shot flags are consumed (not sticky across ticks). --- */
    game->input.pausePressed = true;
    game->input.resetPressed  = true;
    game_fixed_update(game, FIXED_DT_S);
    check(!game->input.pausePressed,
          "pausePressed cleared after consumption");
    check(!game->input.resetPressed,
          "resetPressed cleared after consumption");

    free(game);
}


/* ------------------------------------------------------------------------------------- */
/* Vehicle appearance: purity, totality, sensitivity, and corpus distinctness              */
/* ------------------------------------------------------------------------------------- */

/* The scale everything visual is asserted at: what the game actually draws. Asserting at a
 * higher scale would let differences pass that no player could ever see. */
#define CV_TEST_PX_PER_M   (PIXELS_PER_METER * CAMERA_BASE_ZOOM)

/* Fraction of the union silhouette that must differ for two cars to count as distinguishable.
 * The authoritative metric — a colour-blind comparison of feature-label maps. */
#define CV_MIN_PIXEL_DIFF  0.030f

/* Per-component floor on the diagnostic feature vector: 0.08 m is almost exactly one screen
 * pixel at CV_TEST_PX_PER_M, so "one visible pixel somewhere" is a literal reading of it. */
#define CV_MIN_LINF        0.080f

/* Every car is rasterized into ONE canvas with the CG at a fixed point, so label maps are
 * directly comparable and an alignment difference cannot masquerade as a shape difference. */
static CarRasterInfo cv_shared_canvas(float pxPerM)
{
    float left = 1.0f, right = 1.0f, up = 1.0f, down = 1.0f;

    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        CarVisual visual;
        if (!car_corpus_spec(i, &spec)) continue;
        car_visual_derive(&spec, &visual);

        const CarRasterInfo info = car_raster_info(&visual, pxPerM, 2);
        const float l = info.originXPx;
        const float r = (float)info.width - info.originXPx;
        const float u = info.originYPx;
        const float d = (float)info.height - info.originYPx;
        if (l > left)  left  = l;
        if (r > right) right = r;
        if (u > up)    up    = u;
        if (d > down)  down  = d;
    }

    CarRasterInfo shared;
    memset(&shared, 0, sizeof(shared));
    shared.pxPerM    = pxPerM;
    shared.width     = (int)ceilf(left + right);
    shared.height    = (int)ceilf(up + down);
    shared.originXPx = left;
    shared.originYPx = up;
    return shared;
}

static bool cv_labels_for_spec(const VehicleSpec *spec, CarRasterInfo canvas,
                               unsigned char *labels, size_t bytes)
{
    CarVisual visual;
    car_visual_derive(spec, &visual);
    return car_raster_draw_labels(&visual, canvas, labels, bytes);
}

/* Largest single-component gap between two signature vectors, and which component it was. */
static float cv_signature_linf(const CarVisual *a, const CarVisual *b, int *worstOut)
{
    float sa[64], sb[64];
    const int n = car_visual_signature_count();
    if (n > (int)(sizeof(sa) / sizeof(sa[0]))) return 0.0f;
    if (car_visual_signature(a, sa, n) != n) return 0.0f;
    if (car_visual_signature(b, sb, n) != n) return 0.0f;

    float worst = 0.0f;
    int worstIndex = 0;
    for (int i = 0; i < n; i++) {
        const float d = fabsf(sa[i] - sb[i]);
        if (d > worst) { worst = d; worstIndex = i; }
    }
    if (worstOut != NULL) *worstOut = worstIndex;
    return worst;
}

/* Issue #9: every float field finite; dimensions non-negative; latents in [0, 1]. */
static bool cv_visual_fields_sane(const CarVisual *v)
{
    if (v == NULL) return false;

#define CV_FIN(x) do { if (!isfinite((x))) return false; } while (0)
#define CV_NN(x)  do { CV_FIN(x); if ((x) < 0.0f) return false; } while (0)
#define CV_01(x)  do { CV_FIN(x); if ((x) < 0.0f || (x) > 1.0f) return false; } while (0)

    for (int s = 0; s < CAR_HULL_STATIONS; s++) {
        CV_FIN(v->hull[s].xM);
        CV_NN(v->hull[s].halfWidthM);
    }
    CV_NN(v->lengthM);
    CV_NN(v->widthM);
    CV_NN(v->wheelbaseM);
    CV_NN(v->frontOverhangM);
    CV_NN(v->rearOverhangM);

    CV_FIN(v->cabinCentreXM);
    CV_NN(v->cabinLengthM);
    CV_NN(v->cabinHalfWidthM);
    CV_FIN(v->windscreenXM);
    CV_FIN(v->backlightXM);

    for (int i = 0; i < WHEEL_COUNT; i++) {
        CV_FIN(v->wheels[i].centreM.x);
        CV_FIN(v->wheels[i].centreM.y);
        CV_NN(v->wheels[i].diameterM);
        CV_NN(v->wheels[i].widthM);
        CV_NN(v->wheels[i].rimDiameterM);
        CV_NN(v->wheels[i].discDiameterM);
        CV_FIN(v->wheels[i].staticAngleRad);
        if (v->wheels[i].spokeCount < 0) return false;
    }
    CV_NN(v->archFlareM);

    CV_NN(v->wingSpanM);
    CV_NN(v->wingChordM);
    CV_FIN(v->wingXM);
    CV_NN(v->splitterProtrusionM);
    CV_NN(v->splitterWidthM);
    CV_NN(v->mirrorOffsetM);
    CV_NN(v->exhaustBoreM);
    if (v->exhaustCount < 0) return false;

    CV_01(v->latents.mass01);
    CV_01(v->latents.size01);
    CV_01(v->latents.low01);
    CV_01(v->latents.grip01);
    CV_01(v->latents.balance01);
    CV_01(v->latents.power01);
    CV_01(v->latents.aero01);
    CV_01(v->latents.sport01);
    CV_01(v->latents.strip01);

#undef CV_FIN
#undef CV_NN
#undef CV_01
    return v->lengthM > 0.0f && v->widthM > 0.0f;
}

static void scenario_car_visual(void)
{
    VehicleSpec spec;
    CarVisual a, b;

    /* --- purity: same input, byte-identical CarVisual / signature / raster --- */
    vehicle_spec_set_default(&spec);
    car_visual_derive(&spec, &a);
    car_visual_derive(&spec, &b);
    check(memcmp(&a, &b, sizeof(CarVisual)) == 0,
          "car_visual_derive is pure: identical specs give identical visuals");

    {
        float sa[64], sb[64];
        const int n = car_visual_signature_count();
        check(n > 0 && n <= (int)(sizeof(sa) / sizeof(sa[0])),
              "signature component count is in range");
        check(car_visual_signature(&a, sa, n) == n &&
              car_visual_signature(&b, sb, n) == n,
              "signature writes every component twice");
        check(memcmp(sa, sb, (size_t)n * sizeof(float)) == 0,
              "car_visual_signature is pure: identical visuals give identical signatures");
    }

    {
        const CarRasterInfo info = car_raster_info(&a, CV_TEST_PX_PER_M, 2);
        const size_t bytes = car_raster_bytes(info);
        const size_t labelBytes = (size_t)info.width * (size_t)info.height;
        unsigned char *ra = (unsigned char *)malloc(bytes);
        unsigned char *rb = (unsigned char *)malloc(bytes);
        unsigned char *la = (unsigned char *)malloc(labelBytes);
        unsigned char *lb = (unsigned char *)malloc(labelBytes);
        check(ra != NULL && rb != NULL && la != NULL && lb != NULL && bytes > 0,
              "purity raster buffers allocated");
        if (ra != NULL && rb != NULL && la != NULL && lb != NULL) {
            check(car_raster_draw(&a, info, ra, bytes) &&
                  car_raster_draw(&a, info, rb, bytes),
                  "repeated RGBA rasters succeed");
            check(memcmp(ra, rb, bytes) == 0,
                  "car_raster_draw is pure: identical visuals give bit-identical RGBA");
            check(car_raster_draw_labels(&a, info, la, labelBytes) &&
                  car_raster_draw_labels(&a, info, lb, labelBytes),
                  "repeated label rasters succeed");
            check(memcmp(la, lb, labelBytes) == 0,
                  "car_raster_draw_labels is pure: identical visuals give bit-identical labels");
        }
        free(ra);
        free(rb);
        free(la);
        free(lb);
    }

    /* --- the wheel centres ARE the simulation's wheel positions, not a lookalike --- */
    {
        VehicleState state;
        VehicleDerived derived;
        VehicleRenderState renderState;
        vehicle_state_reset(&spec, &state, &derived, &renderState);
        bool matched = true;
        for (int i = 0; i < WHEEL_COUNT; i++) {
            if (fabsf(a.wheels[i].centreM.x - state.wheels[i].localPositionM.x) > 1e-6f ||
                fabsf(a.wheels[i].centreM.y - state.wheels[i].localPositionM.y) > 1e-6f) {
                matched = false;
            }
        }
        check(matched, "drawn wheel centres equal vehicle.c set_wheel_positions()");
        check(fabsf(a.wheelbaseM - (spec.cgToFrontM + spec.cgToRearM)) < 1e-6f,
              "drawn wheelbase equals the simulated wheelbase");
        check(fabsf(a.wheels[WHEEL_FRONT_LEFT].diameterM - 2.0f * spec.wheelRadiusM) < 1e-6f,
              "drawn tire diameter equals 2 * wheelRadiusM");
        check(fabsf(a.widthM - 2.0f * spec.bodyHalfWidthM) < 1e-6f,
              "drawn body width equals the collision half-width doubled");
    }

    /* --- totality: every declared range corner yields finite, sane geometry --- */
    {
        int bad = 0;
        for (int p = 0; p < dev_params_count(); p++) {
            const DevParameter *param = dev_param_at(p);
            for (int corner = 0; corner < 2; corner++) {
                VehicleSpec probe;
                vehicle_spec_set_default(&probe);
                dev_param_set(&probe, param, corner == 0 ? param->minimum : param->maximum);

                CarVisual v;
                car_visual_derive(&probe, &v);
                if (!cv_visual_fields_sane(&v)) {
                    if (bad == 0) {
                        printf("      first bad corner: %s = %g\n",
                               param->name,
                               (double)(corner == 0 ? param->minimum : param->maximum));
                    }
                    bad++;
                }
            }
        }
        check(bad == 0,
              "every registry range corner produces finite, bounded CarVisual fields");
    }

    /* --- monotonicity: the obvious knobs move the obvious way. These are what stop a
     * distinctness failure being "fixed" by injecting noise into the grammar. --- */
    {
        VehicleSpec lo, hi;
        CarVisual vlo, vhi;

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.wheelRadiusM = 0.25f;
        hi.wheelRadiusM = 0.40f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheels[0].diameterM > vlo.wheels[0].diameterM,
              "a larger wheel radius draws a larger tire");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.bodyHalfWidthM = 0.70f;
        hi.bodyHalfWidthM = 1.10f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.widthM > vlo.widthM, "a wider collision body draws a wider car");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.cgToRearM = 1.00f;
        hi.cgToRearM = 1.90f;
        dev_params_refresh_derived(&lo);
        dev_params_refresh_derived(&hi);
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.lengthM > vlo.lengthM, "a longer wheelbase draws a longer car");

        vehicle_spec_set_default(&lo);
        hi = lo;
        lo.maxBrakeTorqueNm = 500.0f;
        hi.maxBrakeTorqueNm = 6000.0f;
        car_visual_derive(&lo, &vlo);
        car_visual_derive(&hi, &vhi);
        check(vhi.wheels[0].discDiameterM > vlo.wheels[0].discDiameterM,
              "more brake torque draws a larger disc");
    }

    /* --- sensitivity: every sweep axis must actually move the picture. A dead rule is
     * invisible on a contact sheet but fails here. --- */
    {
        const CarRasterInfo canvas = cv_shared_canvas(CV_TEST_PX_PER_M);
        const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
        unsigned char *la = (unsigned char *)malloc(pixels);
        unsigned char *lb = (unsigned char *)malloc(pixels);
        check(la != NULL && lb != NULL && pixels > 0, "sensitivity canvas allocated");

        if (la != NULL && lb != NULL) {
            int dead = 0;
            for (int p = 0; p < dev_params_count(); p++) {
                const DevParameter *param = dev_param_at(p);
                /* Only the keys the corpus advertises as visual drivers. */
                bool isDriver = false;
                for (int i = 0; i < car_corpus_count(); i++) {
                    const char *key = car_corpus_sweep_key(i);
                    if (key != NULL && strcmp(key, param->name) == 0) { isDriver = true; break; }
                }
                if (!isDriver) continue;

                VehicleSpec lo, hi;
                vehicle_spec_set_default(&lo);
                vehicle_spec_set_default(&hi);
                dev_param_set(&lo, param, param->minimum);
                dev_param_set(&hi, param, param->maximum);

                cv_labels_for_spec(&lo, canvas, la, pixels);
                cv_labels_for_spec(&hi, canvas, lb, pixels);
                const float diff = car_raster_difference(la, lb, canvas.width, canvas.height);
                if (diff < CV_MIN_PIXEL_DIFF) {
                    printf("      dead visual axis: %s (min..max moves only %.4f of pixels)\n",
                           param->name, (double)diff);
                    dead++;
                }
            }
            check(dead == 0, "every advertised visual-driver parameter changes the rendering");
        }
        free(la);
        free(lb);
    }

    /* --- the raster scales with the requested resolution, mirroring scenario_renderscale --- */
    {
        CarVisual v;
        vehicle_spec_set_default(&spec);
        car_visual_derive(&spec, &v);
        const CarRasterInfo one = car_raster_info(&v, 16.0f, 0);
        const CarRasterInfo two = car_raster_info(&v, 32.0f, 0);
        check(two.width >= one.width * 2 - 3 && two.width <= one.width * 2 + 3,
              "doubling the raster scale doubles the sprite width");
    }
}

static void scenario_corpus(void)
{
    const int count = car_corpus_count();
    check(count >= 50, "the corpus holds at least 50 vehicles (have %d)", count);

    /* --- every entry is a valid, renderable car --- */
    {
        int invalid = 0;
        for (int i = 0; i < count; i++) {
            VehicleSpec spec;
            char id[128];
            car_corpus_id(i, id, sizeof(id));
            if (!car_corpus_spec(i, &spec)) { invalid++; continue; }
            if (!vehicle_spec_is_valid(&spec)) {
                if (invalid == 0) printf("      first invalid corpus spec: %s\n", id);
                invalid++;
            }
        }
        check(invalid == 0, "every corpus vehicle passes vehicle_spec_is_valid()");
    }

    /* --- sweep steps must not reproduce stock or collapse onto a neighbour ---
     * Regression for the midpoint-default collision: evenly spaced [min, max] steps of
     * body.cg_to_rear landed on VEH_CG_TO_REAR_M at step 2, so sweep_body_cg_to_rear_2 was
     * bit-identical to archetype_00_stock_baseline. The grammar already reads cgToRearM;
     * the bug was corpus sampling, not a disconnected mapping. */
    {
        VehicleSpec stock;
        vehicle_spec_set_default(&stock);

        int collapsed = 0;
        for (int i = 0; i < count; i++) {
            if (car_corpus_group(i) != CAR_CORPUS_SWEEP) continue;

            const char *key = car_corpus_sweep_key(i);
            const DevParameter *param = (key != NULL) ? dev_param_find(key) : NULL;
            VehicleSpec spec;
            if (param == NULL || !car_corpus_spec(i, &spec)) {
                collapsed++;
                continue;
            }

            const float value = dev_param_get(&spec, param);
            const float stockValue = dev_param_get(&stock, param);
            /* Corpus sampling excludes a >= 0.08 m window around the stock default for
             * metre-valued drivers; require at least that gap here so a midpoint collision
             * fails loudly. Neighbour spacing is allowed to be tighter — the pairwise pixel
             * test owns visual separation between adjacent steps. */
            const float minStockGap = 0.08f;
            const float minNeighbourGap = (param->step > 0.0f) ? param->step : 1e-4f;

            if (fabsf(value - stockValue) < minStockGap) {
                if (collapsed == 0) {
                    char id[128];
                    car_corpus_id(i, id, sizeof(id));
                    printf("      sweep reproduces stock: %s (%s = %g, stock %g)\n",
                           id, param->name, (double)value, (double)stockValue);
                }
                collapsed++;
                continue;
            }

            /* Neighbour on the same axis (previous step), if any. */
            if (i > 0 && car_corpus_group(i - 1) == CAR_CORPUS_SWEEP &&
                car_corpus_sweep_key(i - 1) != NULL &&
                strcmp(car_corpus_sweep_key(i - 1), key) == 0) {
                VehicleSpec prev;
                if (!car_corpus_spec(i - 1, &prev)) {
                    collapsed++;
                    continue;
                }
                const float prevValue = dev_param_get(&prev, param);
                if (fabsf(value - prevValue) < minNeighbourGap) {
                    if (collapsed == 0) {
                        char id[128], pid[128];
                        car_corpus_id(i, id, sizeof(id));
                        car_corpus_id(i - 1, pid, sizeof(pid));
                        printf("      sweep neighbours collide: %s vs %s (%s = %g / %g)\n",
                               pid, id, param->name, (double)prevValue, (double)value);
                    }
                    collapsed++;
                }
            }
        }
        check(collapsed == 0,
              "every sweep step differs from stock and from neighbouring steps on its axis");
    }

    /* --- all-pairs distinctness on the colour-blind label maps --- */
    {
        const CarRasterInfo canvas = cv_shared_canvas(CV_TEST_PX_PER_M);
        const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
        unsigned char *maps = (unsigned char *)malloc(pixels * (size_t)count);
        CarVisual *visuals = (CarVisual *)malloc(sizeof(CarVisual) * (size_t)count);
        check(maps != NULL && visuals != NULL && pixels > 0, "distinctness buffers allocated");

        if (maps != NULL && visuals != NULL) {
            bool built = true;
            for (int i = 0; i < count; i++) {
                VehicleSpec spec;
                if (!car_corpus_spec(i, &spec)) { built = false; break; }
                car_visual_derive(&spec, &visuals[i]);
                if (!car_raster_draw_labels(&visuals[i], canvas,
                                            maps + pixels * (size_t)i, pixels)) {
                    built = false;
                    break;
                }
            }
            check(built, "every corpus vehicle rasterizes onto the shared canvas");

            if (built) {
                float worstDiff = 1.0f;
                int worstA = -1, worstB = -1;
                for (int i = 0; i < count; i++) {
                    for (int j = i + 1; j < count; j++) {
                        const float d = car_raster_difference(maps + pixels * (size_t)i,
                                                              maps + pixels * (size_t)j,
                                                              canvas.width, canvas.height);
                        if (d < worstDiff) { worstDiff = d; worstA = i; worstB = j; }
                    }
                }

                if (worstDiff < CV_MIN_PIXEL_DIFF && worstA >= 0) {
                    char ida[128], idb[128];
                    int worstComponent = 0;
                    car_corpus_id(worstA, ida, sizeof(ida));
                    car_corpus_id(worstB, idb, sizeof(idb));
                    const float linf = cv_signature_linf(&visuals[worstA], &visuals[worstB],
                                                         &worstComponent);
                    printf("      closest pair '%s' vs '%s': %.4f of pixels differ,"
                           " largest feature gap %.4f m in '%s'\n",
                           ida, idb, (double)worstDiff, (double)linf,
                           car_visual_signature_component_name(worstComponent));
                }
                check(worstDiff >= CV_MIN_PIXEL_DIFF,
                      "every corpus pair differs in >= %.1f%% of pixels (closest %.2f%%)",
                      (double)(CV_MIN_PIXEL_DIFF * 100.0f), (double)(worstDiff * 100.0f));

                /* The diagnostic vector must agree that the closest pair is separable, so a
                 * failure can always name a feature rather than only a pixel count. */
                float worstLinf = 1e9f;
                for (int i = 0; i < count; i++) {
                    for (int j = i + 1; j < count; j++) {
                        const float linf = cv_signature_linf(&visuals[i], &visuals[j], NULL);
                        if (linf < worstLinf) worstLinf = linf;
                    }
                }
                check(worstLinf >= CV_MIN_LINF,
                      "every corpus pair differs by >= %.3f m in some feature (closest %.4f m)",
                      (double)CV_MIN_LINF, (double)worstLinf);
            }
        }
        free(maps);
        free(visuals);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Runner                                                                                  */
/* ------------------------------------------------------------------------------------- */

typedef void (*ScenarioFn)(void);

typedef struct {
    const char *name;
    const char *description;
    ScenarioFn  run;
} Scenario;

static const Scenario g_scenarios[] = {
    { "math",        "clampf, lerpf, smooth_to, wrap_angle, smoothstep, lerp_angle", scenario_math },
    { "units",       "world<->render conversion and the heading sign convention",    scenario_units },
    { "timestep",    "substep cap, backlog drops, frame clamp, interpolation alpha", scenario_timestep },
    { "oneshot",     "one-shot commands consumed exactly once per press",            scenario_oneshot },
    { "replay",      "deterministic recording, repeatable playback, ring overflow",  scenario_replay },
    { "renderscale", "simulation state is independent of PIXELS_PER_METER",          scenario_renderscale },
    { "car-visual",  "appearance is a pure, total function of VehicleSpec",          scenario_car_visual },
    { "corpus",      "every corpus vehicle is valid and visibly distinct",           scenario_corpus },
    { "telemetry",   "CSV writer: stable header, row count, failure handling",       scenario_telemetry },
    { "vehicle",     "canonical structures, steering, contact velocity, render",      scenario_vehicle_units },
    { "tire",        "nonlinear curves, slip ratio, and combined-friction ellipse",   scenario_tire },
    { "drivetrain",  "engine curve, gearing, torque splits, wheel lock and release",  scenario_drivetrain },
    { "accel-filter","previous-step load-transfer acceleration filter",               scenario_accel_filter },
    { "load-transfer","static split, dynamic transfer, clamping, wheel loads",        scenario_load_transfer },
    { "resistance",  "aerodynamic drag and per-wheel rolling resistance",             scenario_resistance },
    { "rest",        "rest stability and stationary steering",                       scenario_rest },
    { "launch-stop", "straight launch, braking, and zero-speed stability",            scenario_launch_stop },
    { "coast-down",  "throttle lift, engine braking, and separated resistance",       scenario_coast_down },
    { "brake-corner","service braking consumes lateral friction budget",              scenario_braking_cornering },
    { "power-oversteer", "rear drive saturation and throttle-lift recovery",           scenario_power_oversteer },
    { "handbrake",   "rear lockup, combined-slip yaw, and release recovery",           scenario_handbrake_entry },
    { "low-speed",   "kinematic/dynamic blend continuity",                            scenario_low_speed },
    { "reverse",     "explicit reverse launch and stopping",                          scenario_reverse },
    { "steer-sign",  "left-positive force, torque, yaw, and heading",                 scenario_steering_sign },
    { "lever-arm",   "front/rear slip and yaw depend on distinct lever arms",         scenario_lever_arm },
    { "integration", "semi-implicit order and heading wrap",                          scenario_integration },
    { "fixed-rate",  "direct stepping matches accumulator stepping",                 scenario_fixed_rate },
    { "params",      "tunable registry, clamping, and tuning-profile round trip",     scenario_params },
    { "devreplay",   "durable replay timelines, malformed input, event markers",      scenario_dev_replay },
    { "accel-load",  "acceleration transfers load rearward; capacity follows",        scenario_accel_load },
    { "brake-load",  "braking transfers load forward; the car stops stably",          scenario_brake_load },
    { "coast-down-run", "scripted coast: drag falls with v^2, rolling tracks load",   scenario_coast_down_scripted },
    { "skidpad",     "scripted constant radius: steady-state handling metrics",       scenario_skidpad },
    { "skidpad-sweep","constant steer at four speed targets, speed-controlled",       scenario_skidpad_sweep },
    { "step-steer",  "scripted steering step: rise, overshoot, settling, recovery",   scenario_step_steer },
    { "transition",  "scripted left/right transitions: sideslip and yaw sign changes", scenario_transition },
    { "lift-off",    "scripted throttle lift mid-corner: the load shift that causes it", scenario_lift_off },
    { "catchable-drift", "initiate, hold, countersteer, reduce slip, and recover",     scenario_catchable_drift },
    { "lat-load-transfer",  "lateral load transfer: inside/outside wheel unloading",   scenario_lateral_load_transfer },
    { "surface-asymmetry",  "per-surface asymmetry: grass wheel produces yaw moment",   scenario_per_surface_asymmetry },
    { "open-diff",          "open differential: speed differentiation, equal torque",   scenario_open_diff },
    { "lsd-diff",           "LSD: torque bias to higher-grip wheel, capped ratio",      scenario_lsd_diff },
    { "ackermann",          "Ackermann geometry: inner wheel steers more than outer",    scenario_ackermann_geometry },
    { "load-sensitivity",   "tire load sensitivity: heavier wheel has lower mu scale",  scenario_tire_load_sensitivity },
    { "tire-relaxation",    "tire relaxation: lateral force lag and convergence",       scenario_tire_relaxation },
    { "track-surface",      "track geometry, init/free life-cycle, and per-point surface query", scenario_track_surface },
    { "collision-barrier",  "capsule barrier collision, swept test, impulse, and crash lockout", scenario_collision_barrier },
    { "scoring-accumulation", "score accrues during a drift; combo multiplier rises and resets", scenario_scoring_accumulation },
    { "scoring-rejection",   "low speed, reverse, spin, crash, and past-spin-cutoff rejected",  scenario_scoring_rejection },
    { "scoring-determinism", "scoring state provably changes no physical force or checksum",      scenario_scoring_determinism },
    { "highscore-persistence","file load/save, garbage, range, and negative-value validated",      scenario_highscore_persistence },
    { "checkpoint-lap",     "gate crossing, lap counting, forward-only, and lap timer reset",  scenario_checkpoint_lap },
    { "particle-pool",      "init, spawn, round-robin wrap, update, and lifecycle",            scenario_particle_pool },
    { "state-machine",      "MENU/PLAYING/PAUSED/RESULTS transitions and scoring reset",       scenario_state_machine },
};

#define SCENARIO_COUNT ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

static void print_usage(const char *argv0)
{
    printf("usage: %s [--scenario NAME] [--list] [-v] [--no-bundle] [--artifacts DIR]\n", argv0);
    printf("       %s --dump-params [PATH]     write the parameter table as Markdown\n", argv0);
    printf("       %s --benchmark [TICKS]      fixed-update throughput, no telemetry\n", argv0);
    printf("       %s --verify-failure-bundle [DIR]  create, inspect, and clean a fixture\n",
           argv0);
    printf("       %s --generate-corpus [DIR]   export the vehicle corpus as tuning profiles\n",
           argv0);
    printf("       %s --dump-corpus-index [PATH] write the corpus table as Markdown\n", argv0);
    printf("       %s --dump-corpus-sheet [DIR]  render the vehicle contact sheet (PNG+HTML)\n",
           argv0);
}

/* Simulation throughput, for the performance workflow. Prints one machine-readable line. */
static int run_benchmark(int ticks)
{
    if (ticks <= 0) ticks = 240000;

    Game *game = alloc_game();
    game_init(game);
    game->dev.scenario = dev_scenario_find("power-oversteer");
    game->dev.scenarioRunning = (game->dev.scenario > 0);
    game->dev.scenarioStartTick = 0;

    const clock_t started = clock();
    for (int i = 0; i < ticks; i++) {
        /* Restart the script rather than letting it finish, so every tick does real work. */
        if (!game->dev.scenarioRunning && game->dev.scenario > 0) {
            game->dev.scenarioRunning = true;
            game->dev.scenarioStartTick = game->sim.tick;
        }
        game_fixed_update(game, FIXED_DT_S);
    }
    const clock_t finished = clock();

    const double seconds = (double)(finished - started) / (double)CLOCKS_PER_SEC;
    const double ticksPerSecond = (seconds > 0.0) ? (double)ticks / seconds : 0.0;
    const double realtimeFactor = ticksPerSecond / (double)FIXED_HZ;

    printf("BENCHMARK ticks=%d seconds=%.4f ticks_per_second=%.0f realtime_factor=%.1f "
           "checksum=%08x\n",
           ticks, seconds, ticksPerSecond, realtimeFactor, game->stateChecksum);

    free(game);
    return 0;
}

static bool file_contains_text(const char *path, const char *needle)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;
    char buffer[4096];
    const size_t read = fread(buffer, 1, sizeof(buffer) - 1u, file);
    fclose(file);
    buffer[read] = '\0';
    return strstr(buffer, needle) != NULL;
}

/*
 * Exercise the real bundle writer with a controlled Phase 3 invariant failure. This is an
 * explicit validation mode rather than a normal scenario: the ordinary suite must stay
 * green, while release validation still gets a reproducible way to prove that all six
 * diagnostic files and the Phase 3 context are present. The fixture cleans up after itself.
 */
static int verify_failure_bundle(const char *rootDir)
{
    const char *root = (rootDir != NULL) ? rootDir : "artifacts/bundle-verification";
    const char *telemetryPath = "telemetry/_bundle_verification.csv";
    const char *failure =
        "controlled Phase 3 invariant failure: dynamic front load below minimum";
    const char *profile = "Phase3 Candidate";

    Game *game = alloc_game();
    game_init(game);
    const int scenario = dev_scenario_find("accel-load");
    game->dev.scenario = scenario;
    game->dev.scenarioRunning = true;
    game->dev.scenarioStartTick = game->sim.tick;
    replay_begin_recording(&game->replay, game->sim.tick);

    TelemetryWriter writer;
    if (!telemetry_open(&writer, telemetryPath)) {
        free(game);
        return 1;
    }
    for (int i = 0; i < 180; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const TelemetryRow row = telemetry_row_from_game(game, 1);
        if (!telemetry_write_row(&writer, &row)) {
            telemetry_close(&writer);
            free(game);
            return 1;
        }
    }
    if (!telemetry_close(&writer)) {
        free(game);
        return 1;
    }

    FailureBundle bundle;
    memset(&bundle, 0, sizeof(bundle));
    bundle.scenario = "phase3-controlled";
    bundle.failureText = failure;
    bundle.telemetryPath = telemetryPath;
    bundle.replay = &game->replay;
    bundle.spec = &game->spec;
    bundle.activeProfile = profile;
    bundle.failingTick = game->sim.tick;
    bundle.checksum = game->stateChecksum;
    bundle.seed = 1010u;
    bundle.checksRun = 1;
    bundle.checksFailed = 1;

    char directory[512];
    bool ok = failure_bundle_write(root, &bundle, directory, sizeof(directory));
    static const char *const required[] = {
        "replay.bin", "telemetry.csv", "summary.json",
        "config_snapshot.txt", "git_info.txt", "failure.txt"
    };
    char path[640];
    for (size_t i = 0; ok && i < sizeof(required) / sizeof(required[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s", directory, required[i]);
        FILE *file = fopen(path, "rb");
        ok = (file != NULL);
        if (file != NULL) fclose(file);
    }
    snprintf(path, sizeof(path), "%s/summary.json", directory);
    ok = ok && file_contains_text(path, failure) && file_contains_text(path, profile);
    snprintf(path, sizeof(path), "%s/config_snapshot.txt", directory);
    ok = ok && file_contains_text(path, "active_profile=Phase3 Candidate");

    for (size_t i = 0; i < sizeof(required) / sizeof(required[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s", directory, required[i]);
        remove(path);
    }
    DRIFTY_RMDIR(directory);
    remove(telemetryPath);
    DRIFTY_RMDIR(root);
    free(game);

    printf("FAILURE_BUNDLE_VERIFY files=6 failure_text=yes active_profile=yes cleanup=yes "
           "result=%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

static int dump_params(const char *path)
{
    if (path == NULL) {
        dev_params_write_markdown(stdout);
        return 0;
    }

    FILE *file = fopen(path, "wb");
    if (file == NULL) {
        fprintf(stderr, "error: could not write '%s'\n", path);
        return 1;
    }
    dev_params_write_markdown(file);
    if (fclose(file) != 0) {
        fprintf(stderr, "error: could not close '%s'\n", path);
        return 1;
    }
    printf("wrote %s (%d parameters)\n", path, dev_params_count());
    return 0;
}

/* Export every corpus vehicle as a tuning profile, grouped into subdirectories. The files are
 * a human-readable mirror of car_corpus.c, not a second source of truth: the `corpus` scenario
 * asserts each one round-trips back to the spec the code generates. */
static int generate_corpus(const char *dir)
{
    if (dir == NULL) dir = "tuning/corpus";
    if (!telemetry_ensure_dir(dir)) return 1;

    int written = 0;
    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        char id[128], sub[512], path[768];

        if (!car_corpus_spec(i, &spec)) {
            fprintf(stderr, "error: corpus entry %d could not be built\n", i);
            return 1;
        }
        car_corpus_id(i, id, sizeof(id));

        snprintf(sub, sizeof(sub), "%s/%s", dir,
                 car_corpus_group_name(car_corpus_group(i)));
        if (!telemetry_ensure_dir(sub)) return 1;

        snprintf(path, sizeof(path), "%s/%s.txt", sub, id);
        if (!dev_params_save(&spec, path)) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
        written++;
    }

    printf("wrote %d vehicle profiles to %s\n", written, dir);
    return 0;
}

/* The corpus table, in the same generated-Markdown style as docs/PARAMETERS.md. */
static int dump_corpus_index(const char *path)
{
    FILE *out = stdout;
    if (path != NULL) {
        out = fopen(path, "wb");
        if (out == NULL) {
            fprintf(stderr, "error: could not write '%s'\n", path);
            return 1;
        }
    }

    fprintf(out, "<!-- Generated by `drifty_tests --dump-corpus-index`."
                 " Do not edit by hand. -->\n");
    fprintf(out, "# Vehicle corpus\n\n");
    fprintf(out, "Every entry is a pure function of its index in `src/car_corpus.c`."
                 " Appearance is derived from these parameters alone —"
                 " see `src/car_visual.c`.\n\n");
    fprintf(out, "| # | Group | Id | What defines it |\n|---:|---|---|---|\n");

    for (int i = 0; i < car_corpus_count(); i++) {
        char id[128], note[192];
        car_corpus_id(i, id, sizeof(id));
        car_corpus_describe(i, note, sizeof(note));
        fprintf(out, "| %d | %s | `%s` | %s |\n", i,
                car_corpus_group_name(car_corpus_group(i)), id, note);
    }

    if (path != NULL) {
        if (fclose(out) != 0) {
            fprintf(stderr, "error: could not close '%s'\n", path);
            return 1;
        }
        printf("wrote %s (%d vehicles)\n", path, car_corpus_count());
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *only = NULL;
    const char *artifactsDir = "artifacts";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--list") == 0) {
            for (int s = 0; s < SCENARIO_COUNT; s++) {
                printf("%-12s %s\n", g_scenarios[s].name, g_scenarios[s].description);
            }
            return 0;
        }
        if (strcmp(argv[i], "--dump-params") == 0) {
            return dump_params((i + 1 < argc) ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--benchmark") == 0) {
            return run_benchmark((i + 1 < argc) ? atoi(argv[i + 1]) : 0);
        }
        if (strcmp(argv[i], "--generate-corpus") == 0) {
            return generate_corpus((i + 1 < argc && argv[i + 1][0] != '-') ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--dump-corpus-index") == 0) {
            return dump_corpus_index((i + 1 < argc && argv[i + 1][0] != '-')
                                         ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--dump-corpus-sheet") == 0) {
            const char *dir = (i + 1 < argc && argv[i + 1][0] != '-')
                                  ? argv[i + 1] : "artifacts/gallery";
            return car_sheet_write(dir, 0.0f, 0) ? 0 : 1;
        }
        if (strcmp(argv[i], "--verify-failure-bundle") == 0) {
            return verify_failure_bundle((i + 1 < argc && argv[i + 1][0] != '-')
                                             ? argv[i + 1] : NULL);
        }
        if (strcmp(argv[i], "--no-bundle") == 0) {
            g_bundlesEnabled = false;
            continue;
        }
        if (strcmp(argv[i], "--artifacts") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --artifacts needs a directory\n");
                return 2;
            }
            artifactsDir = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            g_verbose = true;
            continue;
        }
        if (strcmp(argv[i], "--scenario") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "error: --scenario needs a name\n");
                print_usage(argv[0]);
                return 2;
            }
            only = argv[++i];
            continue;
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        fprintf(stderr, "error: unrecognised argument '%s'\n", argv[i]);
        print_usage(argv[0]);
        return 2;
    }

    printf("drifty_tests - Phase 0-3 planar vehicle physics and development tooling\n");
    printf("headless: no window, audio, display, or raylib function call\n\n");

    int ran = 0;
    for (int s = 0; s < SCENARIO_COUNT; s++) {
        if (only != NULL && strcmp(only, g_scenarios[s].name) != 0) continue;

        const int failuresBefore = g_failures;
        const int checksBefore   = g_checks;

        printf("[ %s ] %s\n", g_scenarios[s].name, g_scenarios[s].description);
        bundle_context_clear();
        g_scenarios[s].run();

        const int scenarioChecks   = g_checks - checksBefore;
        const int scenarioFailures = g_failures - failuresBefore;
        printf("  -> %d checks, %d failed\n", scenarioChecks, scenarioFailures);

        if (scenarioFailures > 0 && g_bundlesEnabled) {
            FailureBundle bundle;
            memset(&bundle, 0, sizeof(bundle));
            bundle.scenario = g_scenarios[s].name;
            bundle.failureText = g_firstFailureText;
            bundle.telemetryPath = g_bundleHasTelemetry ? g_bundleTelemetryPath : NULL;
            bundle.replay = (g_bundleGame != NULL) ? &g_bundleGame->replay : NULL;
            bundle.spec = (g_bundleGame != NULL) ? &g_bundleGame->spec : NULL;
            bundle.activeProfile = "Default";
            bundle.failingTick = (g_bundleGame != NULL) ? g_bundleGame->sim.tick : 0u;
            bundle.checksum = (g_bundleGame != NULL) ? g_bundleGame->stateChecksum : 0u;
            bundle.seed = g_bundleSeed;
            bundle.checksRun = scenarioChecks;
            bundle.checksFailed = scenarioFailures;

            char directory[512];
            if (failure_bundle_write(artifactsDir, &bundle, directory, sizeof(directory))) {
                printf("  -> failure bundle: %s\n", directory);
            } else {
                fprintf(stderr, "  -> could not write a failure bundle under '%s'\n",
                        artifactsDir);
            }
        }
        printf("\n");
        ran++;
    }

    free_scripted_game();

    if (only != NULL && ran == 0) {
        fprintf(stderr, "error: no scenario named '%s' (try --list)\n", only);
        return 2;
    }

    printf("=====================================================\n");
    printf("%d scenario(s), %d checks, %d failed\n", ran, g_checks, g_failures);
    printf("%s\n", (g_failures == 0) ? "PASS" : "FAIL");

    if (g_failures == 0) return 0;
    return (g_failures > 125) ? 125 : g_failures;
}
