/*
 * physics_tests.c — headless scenario runner.
 *
 * Preserves the Phase 0 infrastructure checks and adds Phase 1 rigid-body vehicle unit and
 * scenario coverage. Phase 2 tire/drivetrain behavior is intentionally absent.
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

#include "config.h"
#include "game.h"
#include "input.h"
#include "math_utils.h"
#include "physics.h"
#include "render.h"
#include "replay.h"
#include "telemetry.h"
#include "timestep.h"
#include "units.h"

/* ------------------------------------------------------------------------------------- */
/* Tiny check framework                                                                    */
/* ------------------------------------------------------------------------------------- */

static int  g_checks   = 0;
static int  g_failures = 0;
static bool g_verbose  = false;

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
    row.frontSlipAngleRad = game->derived.frontSlipAngleRad;
    row.rearSlipAngleRad = game->derived.rearSlipAngleRad;
    row.frontNormalLoadN = game->derived.normalLoadFrontN;
    row.rearNormalLoadN = game->derived.normalLoadRearN;
    row.frontLateralForceN = game->derived.frontLateralForceN;
    row.rearLateralForceN = game->derived.rearLateralForceN;
    row.totalForceXN = game->derived.totalBodyForceN.x;
    row.totalForceYN = game->derived.totalBodyForceN.y;
    row.yawTorqueNm = game->derived.totalYawTorqueNm;
    row.bodySideslipRad = game->derived.bodySideslipRad;
    row.lowSpeedBlend = game->derived.lowSpeedBlend;
    row.substepCount = substepCount;
    row.backlogDrops = game->physicsBacklogDrops;
    row.stateChecksum = game->stateChecksum;
    return row;
}

/* Drive the script with live input while the game module records it. */
static uint32_t run_recording(Game *game, const ScriptFrame *frames, int count,
                              float pixelsPerMeter, TelemetryWriter *writer)
{
    game_init(game);
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
#define TELEMETRY_PATH TELEMETRY_DIR "/phase1_determinism.csv"
#define TELEMETRY_PATH_REPEAT TELEMETRY_DIR "/phase1_determinism_repeat.csv"
#define PHASE1_LAUNCH_TELEMETRY TELEMETRY_DIR "/phase1_launch_stop.csv"
#define PHASE1_LAUNCH_BASELINE "tests/baselines/phase1_launch_stop.csv"

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

    check_near(physics_linear_lateral_force(0.0f, 1000.0f, 1.0f, 500.0f),
               0.0, 0.0, "linear tire force is zero at zero slip");
    check_near(physics_linear_lateral_force(0.1f, 1000.0f, 1.0f, 500.0f),
               -100.0, 1e-5, "linear tire force has slope -cornering stiffness");
    check_near(physics_linear_lateral_force(2.0f, 1000.0f, 1.2f, 500.0f),
               -600.0, 1e-5, "positive slip saturates at -mu*Fz");
    check_near(physics_linear_lateral_force(-2.0f, 1000.0f, 1.2f, 500.0f),
               600.0, 1e-5, "negative slip saturates at +mu*Fz");

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
    check(physics_linear_lateral_force(0.1f, 2000.0f, 1.0f, 1000.0f) <
          physics_linear_lateral_force(0.1f, 1000.0f, 1.0f, 1000.0f),
          "front/rear cornering stiffness parameters can act independently");
    check(fabsf(physics_linear_lateral_force(2.0f, 1000.0f, 0.8f, 500.0f)) <
          fabsf(physics_linear_lateral_force(2.0f, 1000.0f, 1.2f, 500.0f)),
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
    free(game);
}

static void scenario_launch_stop(void)
{
    Game *game = alloc_game();
    game_init(game);
    TelemetryWriter writer;
    const bool telemetryOpened = telemetry_open(&writer, PHASE1_LAUNCH_TELEMETRY);
    game->input.throttle = 1.0f;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
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

    game->input.throttle = 0.0f;
    game->input.brake = 1.0f;
    float previousAbsSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
    bool monotonic = true;
    for (int i = 0; i < 600; i++) {
        game_fixed_update(game, FIXED_DT_S);
        const float absSpeed = fabsf(game->vehicle.velocityLongitudinalMps);
        if (absSpeed > previousAbsSpeed + 1e-5f) monotonic = false;
        previousAbsSpeed = absSpeed;
        if (telemetryOpened && (i + 1) % 120 == 0) {
            const TelemetryRow row = telemetry_row_from_game(game, 1);
            (void)telemetry_write_row(&writer, &row);
        }
    }
    check(monotonic, "service braking reduces speed without an impulse reversal");
    check(game->vehicle.velocityLongitudinalMps >= -1e-6f,
          "ordinary braking does not accelerate the vehicle backward");
    check(fabsf(game->vehicle.velocityLongitudinalMps) < 1e-5f,
          "braking settles at zero speed");
    check(physics_state_is_valid(&game->spec, &game->vehicle, &game->derived),
          "launch and stopping remain finite");
    check(telemetryOpened && telemetry_close(&writer),
          "launch/stop telemetry writes successfully");
    check(files_equal(PHASE1_LAUNCH_TELEMETRY, PHASE1_LAUNCH_BASELINE),
          "launch/stop telemetry matches the reviewed Phase 1 baseline");
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
          "explicit reverse direction launches backward");
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
    const float expectedVelocity = PHASE1_MAX_DRIVE_FORCE_N / spec.massKg * FIXED_DT_S;
    check_near(state.velocityLongitudinalMps, expectedVelocity, 1e-7,
               "velocity integrates force before position");
    check_near(state.positionM.x, expectedVelocity * FIXED_DT_S, 1e-7,
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
    { "telemetry",   "CSV writer: stable header, row count, failure handling",       scenario_telemetry },
    { "vehicle",     "Phase 1 structures, steering, contact velocity, tires, render", scenario_vehicle_units },
    { "rest",        "rest stability and stationary steering",                       scenario_rest },
    { "launch-stop", "straight launch, braking, and zero-speed stability",            scenario_launch_stop },
    { "low-speed",   "kinematic/dynamic blend continuity",                            scenario_low_speed },
    { "reverse",     "explicit reverse launch and stopping",                          scenario_reverse },
    { "steer-sign",  "left-positive force, torque, yaw, and heading",                 scenario_steering_sign },
    { "lever-arm",   "front/rear slip and yaw depend on distinct lever arms",         scenario_lever_arm },
    { "integration", "semi-implicit order and heading wrap",                          scenario_integration },
    { "fixed-rate",  "direct stepping matches accumulator stepping",                 scenario_fixed_rate },
};

#define SCENARIO_COUNT ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

static void print_usage(const char *argv0)
{
    printf("usage: %s [--scenario NAME] [--list] [-v]\n", argv0);
}

int main(int argc, char **argv)
{
    const char *only = NULL;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--list") == 0) {
            for (int s = 0; s < SCENARIO_COUNT; s++) {
                printf("%-12s %s\n", g_scenarios[s].name, g_scenarios[s].description);
            }
            return 0;
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

    printf("drifty_tests - Phase 0 infrastructure + Phase 1 rigid-body vehicle\n");
    printf("headless: no window, audio, display, or raylib function call\n\n");

    int ran = 0;
    for (int s = 0; s < SCENARIO_COUNT; s++) {
        if (only != NULL && strcmp(only, g_scenarios[s].name) != 0) continue;

        const int failuresBefore = g_failures;
        const int checksBefore   = g_checks;

        printf("[ %s ] %s\n", g_scenarios[s].name, g_scenarios[s].description);
        g_scenarios[s].run();

        const int scenarioChecks   = g_checks - checksBefore;
        const int scenarioFailures = g_failures - failuresBefore;
        printf("  -> %d checks, %d failed\n\n", scenarioChecks, scenarioFailures);
        ran++;
    }

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
