/*
 * core_tests.c — math, units, the fixed-timestep accumulator, one-shot input, replay
 * determinism, and render-scale independence.
 *
 * It also implements the scripted input timeline declared in scenario_shared.h: the replay,
 * render-scale and one-shot scenarios here are its primary users, and the physics telemetry
 * scenario reads it.
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
#define CIRCUIT_RMDIR(path) _rmdir(path)
#else
#include <unistd.h>
#define CIRCUIT_RMDIR(path) rmdir(path)
#endif

#include "support/test_harness.h"
#include "support/simulation_fixture.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "dev/car_corpus.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "render/track_ribbon_geometry.h"
#include "core/config.h"
#include "dev/dev_params.h"
#include "dev/dev_replay.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "physics/drivetrain.h"
#include "dev/failure_bundle.h"
#include "physics/surface.h"
#include "game/game.h"
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
/* Scripted input timeline                                                                 */
/* ------------------------------------------------------------------------------------- */

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

/* Deterministic, seeded, and identical on every run of this binary. */
void script_build(ScriptFrame *frames, int count)
{
    static const float frameTimes[4] = { 1.0f / 60.0f, 1.0f / 50.0f, 1.0f / 120.0f, 0.03f };
    uint32_t seed = 0x5EED1234u;

    for (int i = 0; i < count; i++) {
        const uint32_t r = lcg_next(&seed);
        ScriptFrame f;
        memset(&f, 0, sizeof(f));

        f.steer = (float)(int)((r >> 3) % 3u) - 1.0f; /* -1, 0, +1 */
        f.throttle = ((r >> 7) & 3u) != 0u ? 1.0f : 0.0f;
        f.brake = ((r >> 11) & 7u) == 0u ? 1.0f : 0.0f;
        f.handbrake = ((r >> 13) & 7u) == 0u ? 1.0f : 0.0f;
        f.pause = ((r >> 17) & 63u) == 0u;
        f.reset = ((r >> 19) & 127u) == 0u;
        f.debug = ((r >> 21) & 63u) == 0u;
        f.shiftUp = ((r >> 23) & 31u) == 0u;
        f.shiftDown = ((r >> 25) & 31u) == 0u;
        f.frameTimeS = frameTimes[(r >> 29) & 3u];

        frames[i] = f;
    }
}

void fixed_update_adapter(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

void apply_live_input(Game *game, const ScriptFrame *f)
{
    game->input.steer = f->steer;
    game->input.throttle = f->throttle;
    game->input.brake = f->brake;
    game->input.handbrake = f->handbrake;

    /* Latched exactly as input_sample() latches them. */
    if (f->pause) game->input.pausePressed = true;
    if (f->reset) game->input.resetPressed = true;
    if (f->debug) game->input.debugPressed = true;
    if (f->shiftUp) game->input.shiftUpPressed = true;
    if (f->shiftDown) game->input.shiftDownPressed = true;
}

/* Drive the script with live input while the game module records it. */
uint32_t run_recording(Game *game, const ScriptFrame *frames, int count, float pixelsPerMeter,
                       TelemetryWriter *writer)
{
    game_init(game);
    game->state = STATE_PLAYING; /* headless tests start simulating immediately */
    /* The script drives the pedals and the shift keys directly, so the driver aid is off. */
    game->autoTrans.enabled = false;
    game->renderPixelsPerMeter = pixelsPerMeter;

    for (int i = 0; i < count; i++) {
        apply_live_input(game, &frames[i]);

        const TimestepResult step =
            timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                             frames[i].frameTimeS, fixed_update_adapter, game);
        game->lastSubstepCount = step.substeps;

        if (writer != NULL) {
            const TelemetryRow row = test_telemetry_row_from_game(game, step.substeps);
            telemetry_write_row(writer, &row);
        }
    }

    return game->stateChecksum;
}

/* Drive the same frame-time script with NO live input, feeding the recorded timeline. */
uint32_t run_playback(Game *game, const ReplayBuffer *timeline, const ScriptFrame *frames,
                      int count, float pixelsPerMeter)
{
    game_init(game);
    game->state = STATE_PLAYING; /* headless tests start simulating immediately */
    /* The script drives the pedals and the shift keys directly, so the driver aid is off. */
    game->autoTrans.enabled = false;
    game->renderPixelsPerMeter = pixelsPerMeter;

    game->replay = *timeline;
    if (!replay_begin_playback(&game->replay)) return 0u;

    for (int i = 0; i < count; i++) {
        input_zero(&game->input);

        const TimestepResult step =
            timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                             frames[i].frameTimeS, fixed_update_adapter, game);
        game->lastSubstepCount = step.substeps;
    }

    return game->stateChecksum;
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
    /* maxf / minf */
    check_near((double)maxf(3.0f, 5.0f), 5.0, 0.0, "maxf picks the larger");
    check_near((double)maxf(5.0f, 3.0f), 5.0, 0.0, "maxf is order-independent");
    check_near((double)maxf(-1.0f, -2.0f), -1.0, 0.0, "maxf with negatives");
    check_near((double)minf(3.0f, 5.0f), 3.0, 0.0, "minf picks the smaller");
    check_near((double)minf(5.0f, 3.0f), 3.0, 0.0, "minf is order-independent");
    check_near((double)minf(-1.0f, -2.0f), -2.0, 0.0, "minf with negatives");

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
    check_near((double)wrap_angle(CIRCUIT_PI), -(double)CIRCUIT_PI, 1e-5,
               "wrap_angle(+PI) returns -PI (range is closed below, open above)");
    check_near((double)wrap_angle(-CIRCUIT_PI), -(double)CIRCUIT_PI, 1e-5, "wrap_angle(-PI)");
    check_near_angle(wrap_angle(3.0f * CIRCUIT_PI), CIRCUIT_PI, 1e-4f, "wrap_angle(3*PI)");
    check_near((double)wrap_angle(0.5f), 0.5, 1e-6,
               "wrap_angle leaves an in-range angle alone");
    {
        bool inRange = true;
        bool equivalent = true;
        for (int i = -2000; i <= 2000; i++) {
            const float a = (float)i * 0.37f;
            const float w = wrap_angle(a);
            if (!(w >= -CIRCUIT_PI && w < CIRCUIT_PI)) inRange = false;
            /* w must differ from a by a whole number of turns */
            const float turns = (a - w) / CIRCUIT_TWO_PI;
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
        check_near_angle(mid, CIRCUIT_PI, 1e-4f,
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
    /* Surface_Get: every valid id resolves; out-of-range clamps to asphalt. */
    {
        for (SurfaceId id = SURFACE_ASPHALT; id < SURFACE_COUNT; id++) {
            const SurfaceSpec *s = Surface_Get(id);
            check(s != NULL, "Surface_Get(valid %d) is non-NULL", (int)id);
            check(s->muLongitudinal > 0.0f, "Surface_Get(valid %d) has positive mu (got %.2f)",
                  (int)id, (double)s->muLongitudinal);
        }
        const SurfaceSpec *asphalt = Surface_Get(SURFACE_ASPHALT);
        check(Surface_Get(SURFACE_COUNT) == asphalt,
              "Surface_Get(SURFACE_COUNT) falls back to asphalt");
        check(Surface_Get((SurfaceId)100) == asphalt, "Surface_Get(100) falls back to asphalt");
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
    check_near((double)px24.y, -72.0, 1e-6,
               "world->render Y is negated (render space is +Y down)");

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
        check(guarded.x == 0.0f && guarded.y == 0.0f, "render->world guards a zero scale");
    }

    /* Heading (radians, CCW positive) -> raylib rotation (degrees, CW positive). */
    check_near((double)units_heading_to_rotation_deg(0.0f), 0.0, 1e-6, "heading 0 -> 0 deg");
    check_near((double)units_heading_to_rotation_deg(CIRCUIT_PI * 0.5f), -90.0, 1e-4,
               "a counterclockwise quarter turn maps to -90 render degrees");
    check_near((double)units_heading_to_rotation_deg(-CIRCUIT_PI), 180.0, 1e-3,
               "heading -PI -> +180 render degrees");
    check_near((double)units_rotation_deg_to_heading(units_heading_to_rotation_deg(0.7f)), 0.7,
               1e-5, "heading <-> rotation round-trips");

    /* The compiled default must be the documented one. */
    check_near((double)PIXELS_PER_METER, 24.0, 1e-6, "PIXELS_PER_METER default is 24 px/m");

    /* ---- pixel-art world pass ------------------------------------------------------ */

    /* The upscale has to be an exact integer division of the window or the final blit is
     * fractional, which is the one thing the whole low-resolution pass exists to prevent. */
    check(PIXEL_ART_UPSCALE >= 1, "the pixel-art upscale is at least 1");
    check(SCREEN_W % PIXEL_ART_UPSCALE == 0 && SCREEN_H % PIXEL_ART_UPSCALE == 0,
          "%dx%d divides exactly by the %dx upscale", SCREEN_W, SCREEN_H, PIXEL_ART_UPSCALE);
    check(PIXEL_ART_TARGET_W * PIXEL_ART_UPSCALE == SCREEN_W &&
              PIXEL_ART_TARGET_H * PIXEL_ART_UPSCALE == SCREEN_H,
          "the low-resolution target enlarges to exactly the window (%dx%d -> %dx%d)",
          PIXEL_ART_TARGET_W, PIXEL_ART_TARGET_H, SCREEN_W, SCREEN_H);

    /* Camera snapping: the arithmetic behind "camera motion does not shimmer". A Camera2D
     * translates by (offset - target * zoom); if that is fractional, every world pixel lands
     * between two target pixels and the grid crawls as the car moves. The shimmer itself
     * needs a moving camera and an eye, but this is the property underneath it, and it is
     * checkable here — including at the awkward values, negative targets and the drift zoom's
     * whole range. */
    {
        const float offsets[] = { 0.0f, 320.0f, 180.0f };
        const float targets[] = { 0.0f, 1.0f, -1.0f, 37.317f, -204.9993f, 1e4f, -1e4f };
        const float zooms[] = { CAMERA_MIN_ZOOM, 0.45f, CAMERA_BASE_ZOOM, 1.0f, 2.0f };
        int unsnapped = 0;
        for (size_t o = 0; o < sizeof(offsets) / sizeof(offsets[0]); o++) {
            for (size_t t = 0; t < sizeof(targets) / sizeof(targets[0]); t++) {
                for (size_t z = 0; z < sizeof(zooms) / sizeof(zooms[0]); z++) {
                    const float snapped =
                        units_snap_camera_offset_axis(offsets[o], targets[t], zooms[z]);
                    const float translation =
                        units_camera_translation_axis(snapped, targets[t], zooms[z]);
                    /* Tolerance, not equality: the snap is float arithmetic on values up to
                     * 1e4, where one ulp is ~1e-3 of a pixel. Anything under a hundredth of a
                     * pixel is invisible; a genuinely unsnapped translation is off by ~0.5. */
                    const float frac = translation - floorf(translation);
                    const float distanceToWhole = (frac < 0.5f) ? frac : (1.0f - frac);
                    if (distanceToWhole > 0.01f) {
                        if (unsnapped == 0) {
                            printf("      camera translation not on a whole pixel:"
                                   " offset %g target %g zoom %g -> %g\n",
                                   (double)offsets[o], (double)targets[t], (double)zooms[z],
                                   (double)translation);
                        }
                        unsnapped++;
                    }
                }
            }
        }
        check(unsnapped == 0,
              "the snapped camera translation always lands on a whole target pixel");
    }
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

        const TimestepResult r =
            timestep_advance(&accumulator, &drops, (float)MAX_PHYSICS_STEPS * FIXED_DT_S,
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

        const TimestepResult r =
            timestep_advance(&accumulator, &drops, 0.2f, counting_adapter, NULL);
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

        const TimestepResult r =
            timestep_advance(&accumulator, &drops, 10.0f, counting_adapter, NULL);
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
            const float frameTime = (i % 3 == 0)   ? 1.0f / 60.0f
                                    : (i % 3 == 1) ? 1.0f / 144.0f
                                                   : 0.09f;
            const TimestepResult r =
                timestep_advance(&accumulator, &drops, frameTime, counting_adapter, NULL);
            if (!(r.interpolationAlpha >= 0.0f && r.interpolationAlpha <= 1.0f))
                bounded = false;
            if (fabsf(r.interpolationAlpha - accumulator / FIXED_DT_S) > 1e-4f) {
                matchesAccumulator = false;
            }
            if (r.substeps > maxSubsteps) maxSubsteps = r.substeps;
        }

        check(bounded, "interpolation alpha stays within [0, 1]");
        check(matchesAccumulator, "interpolation alpha equals accumulator / FIXED_DT_S");
        check(maxSubsteps <= MAX_PHYSICS_STEPS,
              "the substep cap holds over a long varying-frame-rate run (peak %d)",
              maxSubsteps);
        check(drops > 0, "sustained overload accumulates backlog drops (got %d)", drops);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: controller — the output contract every entrant's driving arrives through      */
/* ------------------------------------------------------------------------------------- */

/*
 * What this asserts is the boundary itself, not any driving.
 *
 * A controller may read the world and its own memory and may write nothing but one
 * ControllerOutput; that output is clamped on the way out; the application commands on the
 * input sample are NOT vehicle controls and no controller can reach or consume them; and the
 * private decision memory is explicit enough to be cleared without disturbing who is driving.
 */
static void scenario_controller(void)
{
    /* ---- 1. Conversion, and the single clamping path ---- */
    {
        Input sample;
        input_zero(&sample);
        sample.steer = 4.0f;      /* out of range on purpose */
        sample.throttle = 9.0f;   /* likewise */
        sample.brake = -3.0f;     /* likewise */
        sample.handbrake = 0.50f; /* in range, must pass through untouched */
        sample.shiftUpPressed = true;
        sample.pausePressed = true;
        sample.resetPressed = true;
        sample.debugPressed = true;
        sample.toggleAutoPressed = true;
        const Input beforeUpdate = sample;

        Controller human;
        controller_init(&human, CONTROLLER_KIND_HUMAN);

        ControllerTickView view;
        memset(&view, 0, sizeof(view));
        view.sample = &sample;
        view.dt = FIXED_DT_S;

        ControllerOutput out;
        controller_update(&human, human.kind, &view, &out);

        check(out.steer == 1.0f, "steer is clamped into [-1, +1] (got %.3f)",
              (double)out.steer);
        check(out.throttle == 1.0f, "throttle is clamped into [0, 1] (got %.3f)",
              (double)out.throttle);
        check(out.brake == 0.0f, "a negative brake clamps to zero (got %.3f)",
              (double)out.brake);
        check(out.handbrake == 0.50f, "an in-range control passes through unchanged (got %.3f)",
              (double)out.handbrake);
        check(out.shiftUp && !out.shiftDown,
              "a shift press becomes this entrant's gear request, and only that one");
        check(memcmp(&sample, &beforeUpdate, sizeof(Input)) == 0,
              "a controller reads the tick sample and cannot write to it: pause, reset, debug "
              "and the automatic-transmission toggle are session commands, not vehicle "
              "controls, and are still pending for the session to consume");

        /* A non-finite demand must not reach the integrator. */
        sample.steer = (float)NAN;
        sample.throttle = (float)INFINITY;
        sample.brake = -(float)INFINITY;
        controller_update(&human, human.kind, &view, &out);
        check(out.steer == 0.0f && out.throttle == 0.0f && out.brake == 0.0f,
              "a non-finite demand is replaced by zero rather than passed through");

        /* No sample at all is "hands off", not last tick's controls. */
        view.sample = NULL;
        controller_update(&human, human.kind, &view, &out);
        check(out.steer == 0.0f && out.throttle == 0.0f && out.brake == 0.0f &&
                  out.handbrake == 0.0f && !out.shiftUp && !out.shiftDown,
              "a missing sample yields a zeroed output, never a stale one");
    }

    /* ---- 2. Kind selects the source, and playback overrides the entrant's own kind ---- */
    {
        Input recorded;
        input_zero(&recorded);
        recorded.throttle = 0.25f;
        recorded.shiftDownPressed = true;

        ControllerTickView view;
        memset(&view, 0, sizeof(view));
        view.sample = &recorded;
        view.dt = FIXED_DT_S;

        Controller entrant;
        controller_init(&entrant, CONTROLLER_KIND_AI);

        ControllerOutput out;
        controller_update(&entrant, CONTROLLER_KIND_REPLAY, &view, &out);
        check(out.throttle == 0.25f,
              "playback overrides the entrant's own kind: the recorded output is authoritative "
              "and the AI does not re-decide (got %.3f)",
              (double)out.throttle);
        check(out.shiftDown, "and the recorded gear request travels with it");
        check(entrant.memory.ai.planLayerCount == 0,
              "an overridden controller does not run, so its private memory is untouched");
    }

    /* ---- 3. Private memory is explicit and resettable ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        track_load_chicane(&game->trackDef);
        check(game_spawn_on_track(game), "the car was placed on the start line");
        game->autoTrans.enabled = true;
        game->autoTrans.forwardOnly = true;
        game->state = STATE_PLAYING;
        controller_init(&game->controller, CONTROLLER_KIND_AI);

        const AiDriverConfig configAtStart = game->controller.config.ai;

        bool everShifted = false;
        bool everBothPedals = false;
        for (int t = 0; t < 240; t++) {
            game_fixed_update(game, FIXED_DT_S);
            if (game->controllerOutput.shiftUp || game->controllerOutput.shiftDown)
                everShifted = true;
            if (game->controllerOutput.throttle > 0.0f && game->controllerOutput.brake > 0.0f)
                everBothPedals = true;
        }

        check(!everShifted,
              "the AI controller asks for no gear change: shifting is the automatic box's job");
        check(!everBothPedals, "and it never presses both pedals in the same tick");
        check(game->controller.memory.ai.planLayerCount > 0,
              "the AI controller accumulated private decision memory (%d plan layers)",
              game->controller.memory.ai.planLayerCount);

        controller_reset(&game->controller);
        Controller blank;
        memset(&blank, 0, sizeof(blank));
        check(memcmp(&game->controller.memory, &blank.memory, sizeof(blank.memory)) == 0,
              "controller_reset clears every byte of the private memory");
        check(memcmp(&game->controller.config.ai, &configAtStart, sizeof(configAtStart)) == 0 &&
                  game->controller.kind == CONTROLLER_KIND_AI,
              "and preserves the kind and the frozen configuration: a reset changes the "
              "situation, not who is driving");

        memset(&game->controller.memory, 0xA5, sizeof(game->controller.memory));
        check(game_spawn_on_track(game),
              "the running AI car can be returned to the start line");
        check(memcmp(&game->controller.memory, &blank.memory, sizeof(blank.memory)) == 0,
              "spawning an existing car clears its controller's old plan and axis history");
        check(memcmp(&game->controller.config.ai, &configAtStart, sizeof(configAtStart)) == 0 &&
                  game->controller.kind == CONTROLLER_KIND_AI,
              "spawning preserves the controller kind and frozen configuration");

        const VehicleSpec replacement = game->spec;
        memset(&game->controller.memory, 0xA5, sizeof(game->controller.memory));
        game_apply_spec(game, &replacement);
        check(memcmp(&game->controller.memory, &blank.memory, sizeof(blank.memory)) == 0,
              "replacing a vehicle clears its controller's old plan and axis history");
        check(memcmp(&game->controller.config.ai, &configAtStart, sizeof(configAtStart)) == 0 &&
                  game->controller.kind == CONTROLLER_KIND_AI,
              "replacing a vehicle preserves the controller kind and frozen configuration");

        track_free(&game->trackDef);
        free(game);
    }

    /* ---- 4. A gear request reaches the gearbox only if a CONTROLLER made it ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        track_load_chicane(&game->trackDef);
        check(game_spawn_on_track(game), "the car was placed on the start line");
        game->state = STATE_PLAYING;
        game->autoTrans.enabled = false; /* gear requests are only live in manual */
        game->vehicle.selectedGear = 2;
        controller_init(&game->controller, CONTROLLER_KIND_AI);

        /* A stray shift press on the session's input source, alongside a genuine session
         * command. The AI controller asks for no shift, so the gearbox must not move — while
         * the session command on the very same sample is still consumed exactly once. */
        game->input.shiftUpPressed = true;
        game->input.pausePressed = true;
        game_fixed_update(game, FIXED_DT_S);

        check(game->sim.shiftUpCount == 0u && game->vehicle.selectedGear == 2,
              "a shift nobody's controller asked for cannot reach the gearbox (count %u, "
              "gear %d)",
              game->sim.shiftUpCount, game->vehicle.selectedGear);
        check(game->sim.pauseToggleCount == 1u,
              "while the session command on the same sample is consumed exactly once (got %u)",
              game->sim.pauseToggleCount);

        track_free(&game->trackDef);
        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: controller-conflict — two controllers, conflicting one-shots, one tick        */
/* ------------------------------------------------------------------------------------- */

/*
 * The failure this exists to catch is the one the old design invited: two entrants sharing a
 * single global Input, where whichever one is updated second sees — or clears — the first
 * one's one-shot. Issue 10 introduces the second entrant; this scenario asserts the property
 * now, at the boundary, so the property exists before the storage that could break it.
 *
 * Two things must hold on the same tick. A gear request belongs to ONE entrant and cannot
 * cross to another, and an application command belongs to the SESSION and is consumed exactly
 * once no matter how many controllers ran or how many fixed substeps the render frame drove.
 */
static void scenario_controller_conflict(void)
{
    /* ---- 1. Two controllers, opposite gear requests, the same tick ---- */
    {
        Input sampleUp;
        input_zero(&sampleUp);
        sampleUp.throttle = 1.0f;
        sampleUp.shiftUpPressed = true;
        sampleUp.pausePressed = true;

        Input sampleDown;
        input_zero(&sampleDown);
        sampleDown.brake = 1.0f;
        sampleDown.shiftDownPressed = true;

        const Input beforeUp = sampleUp;
        const Input beforeDown = sampleDown;

        Controller first;
        Controller second;
        controller_init(&first, CONTROLLER_KIND_HUMAN);
        controller_init(&second, CONTROLLER_KIND_SCRIPT);

        ControllerTickView viewUp;
        ControllerTickView viewDown;
        memset(&viewUp, 0, sizeof(viewUp));
        memset(&viewDown, 0, sizeof(viewDown));
        viewUp.sample = &sampleUp;
        viewUp.dt = FIXED_DT_S;
        viewDown.sample = &sampleDown;
        viewDown.dt = FIXED_DT_S;

        ControllerOutput outUp;
        ControllerOutput outDown;
        controller_update(&first, first.kind, &viewUp, &outUp);
        controller_update(&second, second.kind, &viewDown, &outDown);

        check(outUp.shiftUp && !outUp.shiftDown,
              "the first controller emits its own upshift and nothing else");
        check(outDown.shiftDown && !outDown.shiftUp,
              "the second controller emits its own downshift, unaffected by the first");
        check(outUp.throttle == 1.0f && outUp.brake == 0.0f && outDown.throttle == 0.0f &&
                  outDown.brake == 1.0f,
              "and the held controls stay with the entrant that asked for them");
        check(memcmp(&sampleUp, &beforeUp, sizeof(Input)) == 0 &&
                  memcmp(&sampleDown, &beforeDown, sizeof(Input)) == 0,
              "neither controller consumed or cleared an input sample: consumption is the "
              "session's job, so one entrant cannot swallow another's command");
    }

    /* ---- 2. The same conflict driven through two one-entrant sessions ---- */
    {
        Game *up = alloc_game();
        Game *down = alloc_game();
        game_init(up);
        game_init(down);
        up->state = STATE_PLAYING;
        down->state = STATE_PLAYING;
        up->autoTrans.enabled = false; /* gear requests are only live in manual */
        down->autoTrans.enabled = false;
        up->vehicle.selectedGear = 2;
        down->vehicle.selectedGear = 2;

        /* One render frame, latched exactly as input_sample() latches a press, driving the
         * maximum number of fixed substeps. */
        up->input.shiftUpPressed = true;
        down->input.shiftDownPressed = true;

        (void)timestep_advance(&up->accumulatorS, &up->physicsBacklogDrops,
                               (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter, up);
        (void)timestep_advance(&down->accumulatorS, &down->physicsBacklogDrops,
                               (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter,
                               down);

        check(up->sim.shiftUpCount == 1u && up->sim.shiftDownCount == 0u,
              "the upshift entrant shifted up exactly once across %d substeps (up %u, down %u)",
              MAX_PHYSICS_STEPS, up->sim.shiftUpCount, up->sim.shiftDownCount);
        check(down->sim.shiftDownCount == 1u && down->sim.shiftUpCount == 0u,
              "the downshift entrant shifted down exactly once (up %u, down %u)",
              down->sim.shiftUpCount, down->sim.shiftDownCount);
        check(up->vehicle.selectedGear == 3 && down->vehicle.selectedGear == 1,
              "the two conflicting requests moved their own gearboxes and only their own "
              "(up %d, down %d)",
              up->vehicle.selectedGear, down->vehicle.selectedGear);

        /* A second frame with nothing newly latched must not repeat either request. */
        (void)timestep_advance(&up->accumulatorS, &up->physicsBacklogDrops,
                               (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter, up);
        (void)timestep_advance(&down->accumulatorS, &down->physicsBacklogDrops,
                               (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter,
                               down);
        check(up->sim.shiftUpCount == 1u && down->sim.shiftDownCount == 1u,
              "and neither is re-consumed on the next render frame (up %u, down %u)",
              up->sim.shiftUpCount, down->sim.shiftDownCount);

        free(down);
        free(up);
    }

    /* ---- 3. A session command is consumed once, whoever is driving ---- */
    {
        Game *game = alloc_game();
        game_init(game);
        game->state = STATE_PLAYING;
        game->autoTrans.enabled = false;

        /* Pause and an upshift latched together: one is the session's, one is the entrant's,
         * and each must fire exactly once even though the frame runs many substeps. */
        game->input.pausePressed = true;
        game->input.shiftUpPressed = true;
        (void)timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                               (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter,
                               game);

        check(game->sim.pauseToggleCount == 1u,
              "the session command fired exactly once (got %u)", game->sim.pauseToggleCount);
        check(game->sim.shiftUpCount == 1u,
              "the entrant's gear request fired exactly once on the same tick (got %u)",
              game->sim.shiftUpCount);
        check(!input_has_oneshot(&game->input),
              "and the latch is empty afterwards, so nothing can be consumed twice");

        free(game);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: controller-stream — a recorded controller stream reproduces the checksum      */
/* ------------------------------------------------------------------------------------- */

/*
 * The determinism claim the whole boundary rests on: what is recorded is the entrant's
 * ControllerOutput, and feeding that stream back reproduces the run bit-for-bit.
 *
 * Three runs of one scripted stream. The first drives it live through a human controller and
 * records; the second replays what the first recorded; the third replays a timeline BUILT
 * DIRECTLY from the same intended stream, never having been near the live run. All three must
 * agree, and the recorded frames must equal the intended stream — otherwise "the recording
 * reproduces the run" could be true of a recording of the wrong thing.
 */
#define CONTROLLER_STREAM_TICKS 900

static ControllerOutput controller_stream_at(int tick)
{
    ControllerOutput out;
    memset(&out, 0, sizeof(out));
    const int phase = tick % 300;
    out.throttle = (phase < 180) ? 1.0f : 0.0f;
    out.brake = (phase >= 220 && phase < 260) ? 0.85f : 0.0f;
    out.steer = (phase < 90) ? 0.0f : ((phase < 200) ? 0.6f : -0.45f);
    out.handbrake = (phase >= 280) ? 1.0f : 0.0f;
    /* Gear requests are controller output too, so the stream has to carry some. */
    out.shiftUp = (phase == 60);
    out.shiftDown = (phase == 240);
    return out;
}

static void controller_stream_prepare(Game *game)
{
    game_init(game);
    game->state = STATE_PLAYING;
    game->autoTrans.enabled = false; /* the stream drives the gearbox itself */
}

static void scenario_controller_stream(void)
{
    /* ---- Run 1: live through the human controller, recording as it goes ---- */
    Game *live = alloc_game();
    controller_stream_prepare(live);
    for (int t = 0; t < CONTROLLER_STREAM_TICKS; t++) {
        const ControllerOutput want = controller_stream_at(t);
        input_zero(&live->input);
        live->input.steer = want.steer;
        live->input.throttle = want.throttle;
        live->input.brake = want.brake;
        live->input.handbrake = want.handbrake;
        live->input.shiftUpPressed = want.shiftUp;
        live->input.shiftDownPressed = want.shiftDown;
        game_fixed_update(live, FIXED_DT_S);
    }
    const uint32_t liveChecksum = live->stateChecksum;

    check(live->replay.count == CONTROLLER_STREAM_TICKS,
          "one timeline entry was recorded per fixed tick (%d of %d)", live->replay.count,
          CONTROLLER_STREAM_TICKS);

    /* What was recorded must BE the controller output, entry by entry. */
    int frameMismatches = 0;
    for (int t = 0; t < live->replay.count; t++) {
        const ControllerOutput want = controller_stream_at(t);
        const ReplayFrame *frame =
            &live->replay.frames[(live->replay.head + t) % REPLAY_CAPACITY_TICKS];
        if (frame->steer != want.steer || frame->throttle != want.throttle ||
            frame->brake != want.brake || frame->handbrake != want.handbrake ||
            ((frame->oneshotBits & REPLAY_BIT_SHIFT_UP) != 0u) != want.shiftUp ||
            ((frame->oneshotBits & REPLAY_BIT_SHIFT_DOWN) != 0u) != want.shiftDown)
            frameMismatches++;
    }
    check(frameMismatches == 0,
          "the timeline records the entrant's ControllerOutput exactly (%d entries differ)",
          frameMismatches);
    check(live->sim.shiftUpCount > 0u && live->sim.shiftDownCount > 0u,
          "the stream actually exercised gear requests (up %u, down %u)",
          live->sim.shiftUpCount, live->sim.shiftDownCount);

    /* ---- Run 2: replay what run 1 recorded ---- */
    Game *replayed = alloc_game();
    controller_stream_prepare(replayed);
    replayed->replay = live->replay;
    check(replay_begin_playback(&replayed->replay), "the recorded stream begins playback");
    for (int t = 0; t < CONTROLLER_STREAM_TICKS; t++) {
        input_zero(&replayed->input); /* nothing live: the stream is the only authority */
        game_fixed_update(replayed, FIXED_DT_S);
    }
    check(replayed->stateChecksum == liveChecksum,
          "replaying the recorded controller stream reproduces the checksum (%08x vs %08x)",
          replayed->stateChecksum, liveChecksum);

    /* ---- Run 3: replay a timeline built straight from the intended stream ---- */
    ReplayBuffer *authored = (ReplayBuffer *)calloc(1, sizeof(ReplayBuffer));
    if (authored == NULL) {
        fprintf(stderr, "FATAL: could not allocate a ReplayBuffer\n");
        exit(126);
    }
    replay_begin_recording(authored, 0);
    for (int t = 0; t < CONTROLLER_STREAM_TICKS; t++) {
        const ControllerOutput want = controller_stream_at(t);
        Input frame;
        input_zero(&frame);
        frame.steer = want.steer;
        frame.throttle = want.throttle;
        frame.brake = want.brake;
        frame.handbrake = want.handbrake;
        frame.shiftUpPressed = want.shiftUp;
        frame.shiftDownPressed = want.shiftDown;
        replay_record(authored, &frame);
    }

    Game *authoredRun = alloc_game();
    controller_stream_prepare(authoredRun);
    authoredRun->replay = *authored;
    check(replay_begin_playback(&authoredRun->replay), "the authored stream begins playback");
    for (int t = 0; t < CONTROLLER_STREAM_TICKS; t++) {
        input_zero(&authoredRun->input);
        game_fixed_update(authoredRun, FIXED_DT_S);
    }
    check(authoredRun->stateChecksum == liveChecksum,
          "the same stream authored as controller output, never recorded from the live run, "
          "reproduces the identical checksum (%08x vs %08x)",
          authoredRun->stateChecksum, liveChecksum);
    check(memcmp(&authoredRun->vehicle, &live->vehicle, sizeof(VehicleState)) == 0,
          "and the full vehicle state with it");

    printf("    checksum: live %08x  recorded-replay %08x  authored-stream %08x\n",
           liveChecksum, replayed->stateChecksum, authoredRun->stateChecksum);

    free(authoredRun);
    free(authored);
    free(replayed);
    free(live);
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: oneshot                                                                       */
/* ------------------------------------------------------------------------------------- */

static void scenario_oneshot(void)
{
    Game *game = alloc_game();
    game_init(game);
    game->autoTrans.enabled = false; /* the shift keys are only live in manual */

    /* One render frame that runs the maximum number of substeps, with one press of each
     * one-shot command and held controls applied throughout. */
    game->input.steer = 0.0f;
    game->input.throttle = 1.0f;
    game->input.resetPressed = true;
    game->input.debugPressed = true;
    game->input.shiftUpPressed = true;

    const bool debugBefore = game->debugOverlay;

    TimestepResult r =
        timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
                         (float)MAX_PHYSICS_STEPS * FIXED_DT_S, fixed_update_adapter, game);

    check(r.substeps == MAX_PHYSICS_STEPS, "the frame ran %d substeps (got %d)",
          MAX_PHYSICS_STEPS, r.substeps);
    check(game->sim.resetCount == 1u,
          "a one-frame reset press resets exactly once across %d substeps (got %u)",
          MAX_PHYSICS_STEPS, game->sim.resetCount);
    check(game->sim.debugToggleCount == 1u, "the debug toggle fires exactly once (got %u)",
          game->sim.debugToggleCount);
    check(game->debugOverlay != debugBefore,
          "the debug overlay ends up toggled, not toggled 8 times back to where it started");
    check(game->sim.shiftUpCount == 1u, "shift up fires exactly once (got %u)",
          game->sim.shiftUpCount);
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
    (void)timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
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

    (void)timestep_advance(&game->accumulatorS, &game->physicsBacklogDrops,
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
    Game *first = alloc_game();
    Game *second = alloc_game();

    const uint32_t firstChecksum =
        run_playback(first, &recorder->replay, frames, SCRIPT_FRAMES, PIXELS_PER_METER);
    const uint32_t secondChecksum =
        run_playback(second, &recorder->replay, frames, SCRIPT_FRAMES, PIXELS_PER_METER);

    check(firstChecksum == recordedChecksum,
          "replay reproduces the recorded final-state checksum (%08x vs %08x)", firstChecksum,
          recordedChecksum);
    check(firstChecksum == secondChecksum,
          "two replays of the same timeline agree (%08x vs %08x)", firstChecksum,
          secondChecksum);
    check(first->sim.tick == recordedTicks && second->sim.tick == recordedTicks,
          "both replays executed the same number of fixed ticks");
    check(first->sim.resetCount == recorder->sim.resetCount &&
              first->sim.pauseToggleCount == recorder->sim.pauseToggleCount &&
              first->sim.shiftUpCount == recorder->sim.shiftUpCount,
          "one-shot commands are reproduced exactly by the timeline");
    check(memcmp(&first->sim, &second->sim, sizeof(SimState)) == 0 &&
              memcmp(&first->vehicle, &second->vehicle, sizeof(VehicleState)) == 0,
          "the full vehicle simulation state matches between the two replays");

    printf("    checksum: recorded %08x  replay#1 %08x  replay#2 %08x\n", recordedChecksum,
           firstChecksum, secondChecksum);

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
            in.steer = (float)i; /* a unique, identifiable value per tick */
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

        replay_stop(ring);
        ring->initialVehicle.valid = true;
        check(!replay_begin_playback(ring) && ring->mode == REPLAY_MODE_IDLE,
              "a wrapped timeline rejects a stale vehicle snapshot instead of diverging");

        free(ring);
    }
}

/* ------------------------------------------------------------------------------------- */
/* Scenario: renderscale                                                                   */
/* ------------------------------------------------------------------------------------- */

static void scenario_renderscale(void)
{
    /* Thick line segments have flat ends. A bend therefore needs an explicit join between
     * the incoming and outgoing edge sections or the grass background shows through. */
    {
        TrackNode nodes[3] = {
            { .centerM = { 0.0f, 0.0f }, .halfWidthM = 2.0f },
            { .centerM = { 4.0f, 0.0f }, .halfWidthM = 2.0f },
            { .centerM = { 4.0f, 4.0f }, .halfWidthM = 2.0f },
        };
        const TrackDefinition track = { .nodes = nodes, .count = 3 };
        TrackRibbonJoin join;
        const bool built = track_ribbon_join_build(&track, 1, &join);

        check(built,
              "a bent asphalt ribbon builds a join between adjacent flat-ended segments");
        if (built) {
            check_near(join.centerM.x, 4.0, 1e-6, "ribbon join stays on the authored node X");
            check_near(join.centerM.y, 0.0, 1e-6, "ribbon join stays on the authored node Y");
            check_near(join.previousLeftM.x, 4.0, 1e-6,
                       "incoming left edge reaches the node cross-section");
            check_near(join.previousLeftM.y, 2.0, 1e-6,
                       "incoming left edge uses the segment half-width");
            check_near(join.nextLeftM.x, 2.0, 1e-6,
                       "outgoing left edge rotates with the next segment");
            check_near(join.nextLeftM.y, 0.0, 1e-6,
                       "outgoing left edge reaches the node cross-section");
            check_near(join.previousRightM.x, 4.0, 1e-6,
                       "incoming right edge reaches the node cross-section");
            check_near(join.previousRightM.y, -2.0, 1e-6,
                       "incoming right edge uses the segment half-width");
            check_near(join.nextRightM.x, 6.0, 1e-6,
                       "outgoing right edge rotates with the next segment");
            check_near(join.nextRightM.y, 0.0, 1e-6,
                       "outgoing right edge reaches the node cross-section");
            check(join.previousLeftM.x != join.nextLeftM.x ||
                      join.previousLeftM.y != join.nextLeftM.y,
                  "the regression fixture has a real wedge for the join to fill");
        }
    }

    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    Game *baseline = alloc_game();
    Game *doubled = alloc_game();

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
        const Vector2 b =
            units_world_to_render_px(doubled->vehicle.positionM, doubled->renderPixelsPerMeter);
        check(fabsf(a.x) > 1e-4f || fabsf(a.y) > 1e-4f,
              "the vehicle ended away from the origin, so the pixel comparison is meaningful");
        check(fabsf(b.x - a.x) > 1e-5f || fabsf(b.y - a.y) > 1e-5f,
              "render pixel output does change with the scale");
    }

    free(doubled);
    free(baseline);
    free(frames);
}

/*
 * scenario_metamorphic_steer_mirror_symmetry — metamorphic test: run a
 * deterministic script twice, once as-is and once with steer negated.
 * Asserts |yaw|, |beta|, and speed magnitudes match within tolerance
 * across the full run. The sign relation (left-positive convention) is
 * already validated by scenario_steer_sign; this test checks that the
 * full multi-second trajectory preserves magnitude symmetry.
 */
static void scenario_metamorphic_steer_mirror_symmetry(void)
{
    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (!frames) return;
    script_build(frames, SCRIPT_FRAMES);

    ScriptFrame *negated = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (!negated) {
        free(frames);
        return;
    }
    for (int i = 0; i < SCRIPT_FRAMES; i++) {
        negated[i] = frames[i];
        negated[i].steer = -frames[i].steer;
    }

    Game *pos = alloc_game();
    Game *neg = alloc_game();
    game_init(pos);
    game_init(neg);
    pos->state = STATE_PLAYING;
    neg->state = STATE_PLAYING;

    float maxYawDiff = 0.0f, maxBetaDiff = 0.0f, maxSpeedDiff = 0.0f;
    bool allFinite = true;

    for (int i = 0; i < SCRIPT_FRAMES; i++) {
        apply_live_input(pos, &frames[i]);
        apply_live_input(neg, &negated[i]);

        timestep_advance(&pos->accumulatorS, &pos->physicsBacklogDrops, frames[i].frameTimeS,
                         fixed_update_adapter, pos);
        timestep_advance(&neg->accumulatorS, &neg->physicsBacklogDrops, negated[i].frameTimeS,
                         fixed_update_adapter, neg);

        const float yawDiff =
            fabsf(fabsf(pos->vehicle.yawRateRadS) - fabsf(neg->vehicle.yawRateRadS));
        const float betaDiff =
            fabsf(fabsf(pos->derived.bodySideslipRad) - fabsf(neg->derived.bodySideslipRad));
        const float speedDiff = fabsf(pos->derived.speedMps - neg->derived.speedMps);
        if (yawDiff > maxYawDiff) maxYawDiff = yawDiff;
        if (betaDiff > maxBetaDiff) maxBetaDiff = betaDiff;
        if (speedDiff > maxSpeedDiff) maxSpeedDiff = speedDiff;

        if (!physics_state_is_valid(&pos->spec, &pos->vehicle, &pos->derived) ||
            !physics_state_is_valid(&neg->spec, &neg->vehicle, &neg->derived))
            allFinite = false;
    }

    check(allFinite, "metamorphic: both runs finite");
    check(maxYawDiff < 0.05f, "metamorphic: |yaw| matches (max diff %.4f rad/s)",
          (double)maxYawDiff);
    check(maxBetaDiff < 0.05f, "metamorphic: |beta| matches (max diff %.4f rad)",
          (double)maxBetaDiff);
    check(maxSpeedDiff < 0.1f, "metamorphic: speed matches (max diff %.4f m/s)",
          (double)maxSpeedDiff);

    free(neg);
    free(pos);
    free(negated);
    free(frames);
}

/*
 * scenario_replay_bundle_seed_reproduction — asserts that a failure bundle written by a
 * failing scenario can be reloaded and replayed to reproduce the identical failing check and
 * stateChecksum, closing the "is this bundle actually reproducible" question the current
 * failure_bundle machinery leaves open.
 *
 * Inherited from the testing-overhaul plan, Track F1/F2 (not a fresh arXiv finding this round):
 * "F1: extend failure_bundle to record the seeded random state and entire input timeline...
 * F2: new `circuit_tests --replay-bundle <dir>` mode... A known-bad bundle on main fails with
 * the same offending check text; a known-good bundle passes." Placed in core_tests.c (not
 * gameplay, where round 2's plan-inherited scaffolds landed) because reproducibility is a
 * property of the record/replay infrastructure in scenario_shared.h and dev/failure_bundle.h,
 * the same infrastructure the `replay` scenario in this file already exercises.
 *
 * WHAT THIS ASSERTS. The round trip is the whole point, so it goes through the filesystem
 * rather than comparing two in-memory ReplayBuffers: record a scripted run, treat a
 * deliberately-failing check as the trigger, write a real bundle with failure_bundle_write(),
 * then reload only what the bundle contains (replay.bin via dev_replay_load, failure text via
 * failure.txt) and re-run the recorded timeline into a fresh Game. The reloaded run must
 * reproduce the recorded stateChecksum exactly, and the bundle must name the same failing
 * check. That is the "is this bundle actually reproducible" question, answered against the
 * real writer rather than a mock.
 *
 * There is no `--replay-bundle <dir>` flag in test_main.c today; this scenario is the
 * in-process equivalent, and is what such a flag would have to call.
 *
 * The scenario creates and removes its own bundle directory rather than depending on another
 * scenario's failure output, so the ordinary suite stays green and leaves no artifacts.
 */
static void scenario_replay_bundle_seed_reproduction(void)
{
    ScriptFrame *frames = (ScriptFrame *)calloc(SCRIPT_FRAMES, sizeof(ScriptFrame));
    if (frames == NULL) {
        fprintf(stderr, "FATAL: could not allocate the input script\n");
        exit(126);
    }
    script_build(frames, SCRIPT_FRAMES);

    /* ---- 1. Record a run and manufacture a known failing check from its real state. ---- */
    Game *recorder = alloc_game();
    const uint32_t recordedChecksum =
        run_recording(recorder, frames, SCRIPT_FRAMES, PIXELS_PER_METER, NULL);
    const uint64_t recordedTicks = recorder->sim.tick;

    check(recordedTicks > 0u && recorder->replay.count == (int)recordedTicks,
          "bundle-repro: recorded %d timeline entries over %llu ticks", recorder->replay.count,
          (unsigned long long)recordedTicks);
    check(recorder->replay.overwrittenTicks == 0u,
          "bundle-repro: the script fits the ring without overwriting (%llu overwritten)",
          (unsigned long long)recorder->replay.overwrittenTicks);

    /* The trigger. Phrased exactly as a real failing check would be, and carrying the run's
     * own numbers so a wrong reload cannot accidentally produce matching text. */
    char failureText[256];
    snprintf(failureText, sizeof(failureText),
             "bundle-repro: deliberate trigger at tick %llu (checksum %08x, speed %.4f m/s)",
             (unsigned long long)recordedTicks, recordedChecksum,
             (double)recorder->derived.speedMps);

    /* ---- 2. Write a real bundle. ---- */
    const uint32_t seed = 20260731u;
    FailureBundle bundle;
    memset(&bundle, 0, sizeof(bundle));
    bundle.scenario = "replay-bundle-repro";
    bundle.failureText = failureText;
    bundle.replay = &recorder->replay;
    bundle.spec = &recorder->spec;
    bundle.activeProfile = "default";
    bundle.failingTick = recordedTicks;
    bundle.checksum = recordedChecksum;
    bundle.seed = seed;
    bundle.checksRun = 1;
    bundle.checksFailed = 1;

    const char *root = "artifacts/bundle-repro-test";
    char directory[512];
    const bool written = failure_bundle_write(root, &bundle, directory, sizeof(directory));
    check(written, "bundle-repro: bundle written to disk");

    if (written) {
        char path[640];

        /* ---- 3. Reload the timeline from replay.bin alone. ---- */
        snprintf(path, sizeof(path), "%s/replay.bin", directory);
        ReplayBuffer *loaded = (ReplayBuffer *)calloc(1, sizeof(ReplayBuffer));
        DevReplayInfo info;
        memset(&info, 0, sizeof(info));
        if (loaded == NULL) {
            fprintf(stderr, "FATAL: could not allocate a ReplayBuffer\n");
            exit(126);
        }

        const bool reloaded = dev_replay_load(loaded, path, &info);
        check(reloaded, "bundle-repro: replay.bin reloads from the bundle");

        if (reloaded) {
            check(info.seed == seed, "bundle-repro: bundle preserves the seed (%u vs %u)",
                  info.seed, seed);
            check(info.finalChecksum == recordedChecksum,
                  "bundle-repro: bundle records the failing checksum (%08x vs %08x)",
                  info.finalChecksum, recordedChecksum);
            check(info.frameCount == (uint32_t)recorder->replay.count,
                  "bundle-repro: every recorded frame survived the round trip (%u vs %d)",
                  info.frameCount, recorder->replay.count);
            check(info.firstTick == recorder->replay.firstTick,
                  "bundle-repro: the timeline's absolute start tick round-trips");
            check(info.fixedHz == (uint32_t)FIXED_HZ,
                  "bundle-repro: the timeline records its tick rate (%u Hz)", info.fixedHz);

            /* ---- 4. Re-run the reloaded timeline and compare. ---- */
            Game *replayed = alloc_game();
            const uint32_t replayedChecksum =
                run_playback(replayed, loaded, frames, SCRIPT_FRAMES, PIXELS_PER_METER);

            check(replayedChecksum == recordedChecksum,
                  "bundle-repro: the reloaded bundle reproduces the failing checksum "
                  "(%08x vs %08x)",
                  replayedChecksum, recordedChecksum);
            check(replayed->sim.tick == recordedTicks,
                  "bundle-repro: the reloaded run executes the same tick count (%llu vs %llu)",
                  (unsigned long long)replayed->sim.tick, (unsigned long long)recordedTicks);
            check(memcmp(&replayed->vehicle, &recorder->vehicle, sizeof(VehicleState)) == 0,
                  "bundle-repro: the reloaded run reproduces the full vehicle state");

            /* The failing check must be reproducible too, not just the physics. */
            char reproducedText[256];
            snprintf(reproducedText, sizeof(reproducedText),
                     "bundle-repro: deliberate trigger at tick %llu (checksum %08x, "
                     "speed %.4f m/s)",
                     (unsigned long long)replayed->sim.tick, replayedChecksum,
                     (double)replayed->derived.speedMps);
            check(
                strcmp(reproducedText, failureText) == 0,
                "bundle-repro: the reloaded run regenerates the identical failing-check text");

            free(replayed);
        }
        free(loaded);

        /* ---- 5. failure.txt names the same check. ---- */
        snprintf(path, sizeof(path), "%s/failure.txt", directory);
        {
            FILE *file = fopen(path, "rb");
            check(file != NULL, "bundle-repro: failure.txt exists");
            if (file != NULL) {
                char buffer[1024];
                const size_t read = fread(buffer, 1, sizeof(buffer) - 1u, file);
                fclose(file);
                buffer[read] = '\0';
                check(strstr(buffer, failureText) != NULL,
                      "bundle-repro: failure.txt names the original failing check");
            }
        }

        static const char *const files[] = { "failure.txt", "git_info.txt",
                                             "config_snapshot.txt", "replay.bin",
                                             "summary.json" };
        for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); i++) {
            snprintf(path, sizeof(path), "%s/%s", directory, files[i]);
            remove(path);
        }
        CIRCUIT_RMDIR(directory);
        CIRCUIT_RMDIR(root);
    }

    free(recorder);
    free(frames);
}

/*
 * scenario_multi_failure_bundle_capture — writes a FailureBundle with 3 distinct
 * failure messages, verifies failure.txt contains all 3, then cleans up.
 * Exercises the multi-message FailureBundle extension.
 */
static void scenario_multi_failure_bundle_capture(void)
{
    const char *messages[] = {
        "FAIL check 1: deliberate failure for multi-capture test",
        "FAIL check 2: second distinct failure message",
        "FAIL check 3: third distinct failure message",
    };
    const int msgCount = 3;

    Game *game = alloc_game();
    game_init(game);
    replay_begin_recording(&game->replay, game->sim.tick);

    FailureBundle bundle;
    memset(&bundle, 0, sizeof(bundle));
    bundle.scenario = "multi-failure-test";
    bundle.failureText = messages[0];
    bundle.failureCheckMessages = messages;
    bundle.failureMessageCount = msgCount;
    bundle.replay = &game->replay;
    bundle.spec = &game->spec;
    bundle.activeProfile = "test";
    bundle.failingTick = 42;
    bundle.checksum = 0xDEADu;
    bundle.seed = 42u;
    bundle.checksRun = msgCount;
    bundle.checksFailed = msgCount;

    const char *root = "artifacts/bundle-multi-test";
    char directory[512];
    bool ok = failure_bundle_write(root, &bundle, directory, sizeof(directory));
    check(ok, "multi-failure: bundle written");

    if (ok) {
        char path[640];
        snprintf(path, sizeof(path), "%s/failure.txt", directory);
        FILE *file = fopen(path, "rb");
        check(file != NULL, "multi-failure: failure.txt exists");
        if (file != NULL) {
            char buf[2048];
            size_t n = fread(buf, 1, sizeof(buf) - 1, file);
            fclose(file);
            buf[n] = '\0';
            for (int i = 0; i < msgCount; i++) {
                check(strstr(buf, messages[i]) != NULL, "multi-failure: message %d present",
                      i + 1);
            }
        }
        /* Cleanup */
        remove(path);
        snprintf(path, sizeof(path), "%s/replay.bin", directory);
        remove(path);
        snprintf(path, sizeof(path), "%s/summary.json", directory);
        remove(path);
        snprintf(path, sizeof(path), "%s/config_snapshot.txt", directory);
        remove(path);
        snprintf(path, sizeof(path), "%s/git_info.txt", directory);
        remove(path);
        CIRCUIT_RMDIR(directory);
        CIRCUIT_RMDIR(root);
    }

    free(game);
}

static const TestScenario kCoreScenarios[] = {
    { "math", "clampf, lerpf, smooth_to, wrap_angle, smoothstep, lerp_angle", scenario_math },
    { "units", "world<->render conversion and the heading sign convention", scenario_units },
    { "timestep", "substep cap, backlog drops, frame clamp, interpolation alpha",
      scenario_timestep },
    { "oneshot", "one-shot commands consumed exactly once per press", scenario_oneshot },
    { "controller", "ControllerOutput conversion, clamping, kind dispatch, resettable memory",
      scenario_controller },
    { "controller-conflict",
      "two controllers issuing conflicting one-shots on the same tick stay independent",
      scenario_controller_conflict },
    { "controller-stream", "a recorded controller stream reproduces an identical checksum",
      scenario_controller_stream },
    { "replay", "deterministic recording, repeatable playback, ring overflow",
      scenario_replay },
    { "renderscale", "simulation state is independent of PIXELS_PER_METER",
      scenario_renderscale },
    { "metamorphic-steer-mirror-symmetry",
      "steer-negation mirror relation: |yaw|, |beta|, speed magnitudes match",
      scenario_metamorphic_steer_mirror_symmetry },
    { "replay-bundle-seed-reproduction",
      "a written failure bundle reloads and reproduces its failing checksum and text",
      scenario_replay_bundle_seed_reproduction },
    { "multi-failure-bundle-capture",
      "a bundle records every failing check in one run, not just the first",
      scenario_multi_failure_bundle_capture },
};

TestScenarioGroup test_core_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kCoreScenarios;
    group.count = sizeof(kCoreScenarios) / sizeof(kCoreScenarios[0]);
    return group;
}
