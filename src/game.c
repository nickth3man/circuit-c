/*
 * game.c — the reloadable game module's entry points.
 *
 * Phase 0 responsibilities: consume input for one fixed tick, act on one-shot commands
 * exactly once, advance the deterministic placeholder transform, maintain the state
 * checksum, record the input timeline, and draw a debug HUD that makes the loop's behaviour
 * visible.
 *
 * No vehicle physics exists here. physics.c takes ownership of the fixed update order in
 * Phase 1 and the placeholder below is deleted at that point.
 *
 * Everything except game_draw() is free of raylib calls, so this file compiles into the
 * headless test executable with -DDRIFTY_HEADLESS.
 */
#include "game.h"

#include <math.h>
#include <string.h>

#include "math_utils.h"
#include "units.h"

#if !defined(DRIFTY_HEADLESS)
#include "raylib.h"
#endif

/* ------------------------------------------------------------------------------------- */
/* Checksum                                                                                */
/* ------------------------------------------------------------------------------------- */

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME        16777619u

static uint32_t hash_bytes(uint32_t h, const void *data, size_t length)
{
    const unsigned char *bytes = (const unsigned char *)data;
    for (size_t i = 0; i < length; i++) {
        h ^= (uint32_t)bytes[i];
        h *= FNV1A_PRIME;
    }
    return h;
}

/* Hash the IEEE-754 bit pattern rather than the value, so the checksum is exact and does
 * not depend on how a comparison would round. */
static uint32_t hash_f32(uint32_t h, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return hash_bytes(h, &bits, sizeof(bits));
}

static uint32_t hash_u32(uint32_t h, uint32_t value)
{
    return hash_bytes(h, &value, sizeof(value));
}

static uint32_t hash_u64(uint32_t h, uint64_t value)
{
    return hash_bytes(h, &value, sizeof(value));
}

GAME_API uint32_t game_state_checksum(const Game *game)
{
    if (game == NULL) return 0u;

    uint32_t h = FNV1A_OFFSET_BASIS;
    h = hash_u32(h, (uint32_t)game->state);
    h = hash_u64(h, game->sim.tick);
    h = hash_f32(h, game->sim.markerPositionM.x);
    h = hash_f32(h, game->sim.markerPositionM.y);
    h = hash_f32(h, game->sim.markerHeadingRad);
    h = hash_u32(h, game->sim.resetCount);
    h = hash_u32(h, game->sim.pauseToggleCount);
    h = hash_u32(h, game->sim.debugToggleCount);
    h = hash_u32(h, game->sim.shiftUpCount);
    h = hash_u32(h, game->sim.shiftDownCount);
    return h;
}

/* ------------------------------------------------------------------------------------- */
/* Phase 0 placeholder transform                                                           */
/* ------------------------------------------------------------------------------------- */

GAME_API void game_reset_sim(Game *game)
{
    if (game == NULL) return;

    game->sim.markerPositionM.x = 0.0f;
    game->sim.markerPositionM.y = 0.0f;
    game->sim.markerHeadingRad  = 0.0f;

    game->renderState.currPositionM  = game->sim.markerPositionM;
    game->renderState.currHeadingRad = game->sim.markerHeadingRad;
    game->renderState.prevPositionM  = game->sim.markerPositionM;
    game->renderState.prevHeadingRad = game->sim.markerHeadingRad;
}

/*
 * PHASE 0 PLACEHOLDER — a deterministic kinematic marker, not a vehicle.
 *
 * Heading integrates the steer axis at a fixed rate; position integrates a fixed speed
 * along that heading. There is no mass, no tire, no slip, and no drivetrain here, and none
 * is implied. Deleted in Phase 1 when physics.c arrives.
 */
static void sim_advance_marker(Game *game, const Input *in, float dt)
{
    const float steer     = clampf(in->steer, -1.0f, 1.0f);
    const float drive     = clampf(in->throttle - in->brake, -1.0f, 1.0f);
    const float handbrake = clampf(in->handbrake, 0.0f, 1.0f);

    const float speedScale = lerpf(1.0f, MARKER_HANDBRAKE_SCALE, handbrake);
    const float speedMps   = drive * MARKER_SPEED_MPS * speedScale;

    game->sim.markerHeadingRad =
        wrap_angle(game->sim.markerHeadingRad + steer * MARKER_TURN_RATE_RAD_S * dt);

    game->sim.markerPositionM.x += cosf(game->sim.markerHeadingRad) * speedMps * dt;
    game->sim.markerPositionM.y += sinf(game->sim.markerHeadingRad) * speedMps * dt;
}

static void apply_oneshots(Game *game, const Input *in)
{
    if (in->pausePressed) {
        if (game->state == STATE_PLAYING)      game->state = STATE_PAUSED;
        else if (game->state == STATE_PAUSED)  game->state = STATE_PLAYING;
        game->sim.pauseToggleCount++;
    }
    if (in->resetPressed) {
        game_reset_sim(game);
        game->sim.resetCount++;
    }
    if (in->debugPressed) {
        game->debugOverlay = !game->debugOverlay;
        game->sim.debugToggleCount++;
    }
    /* Gear commands have no drivetrain to act on until Phase 2. Counting them here proves
     * the one-shot path carries them exactly once. */
    if (in->shiftUpPressed)   game->sim.shiftUpCount++;
    if (in->shiftDownPressed) game->sim.shiftDownCount++;
}

/* ------------------------------------------------------------------------------------- */
/* Entry points                                                                            */
/* ------------------------------------------------------------------------------------- */

GAME_API void game_init(Game *game)
{
    if (game == NULL) return;

    input_zero(&game->input);

    memset(&game->sim, 0, sizeof(game->sim));
    game_reset_sim(game);

    game->state                = STATE_PLAYING;
    game->accumulatorS         = 0.0f;
    game->lastSubstepCount     = 0;
    game->physicsBacklogDrops  = 0;
    game->debugOverlay         = false;
    game->reloadCount          = 0;
    game->reloadFlashTimerS    = 0.0f;
    game->renderPixelsPerMeter = PIXELS_PER_METER;

    /* Record from the first tick. The ring keeps the most recent
     * REPLAY_CAPACITY_TICKS / FIXED_HZ seconds, which is the window a restart replays. */
    replay_begin_recording(&game->replay, 0);

    game->stateChecksum = game_state_checksum(game);
    game->initialized   = true;

#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: initialised (Phase 0 - no vehicle physics)");
#endif
}

GAME_API void game_pre_reload(Game *game)
{
    if (game == NULL) return;

    /* Phase 0 owns no raylib resources: no textures, sounds, or audio stream callbacks
     * have been acquired by this module. Recording is suspended so the swap itself cannot
     * be mistaken for a recorded tick, and resumed in game_post_reload(). */
    if (game->replay.mode == REPLAY_MODE_RECORDING) replay_stop(&game->replay);

#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: pre-reload (tick %llu)", (unsigned long long)game->sim.tick);
#endif
}

GAME_API void game_post_reload(Game *game)
{
    if (game == NULL) return;

    /* Re-acquire what pre-reload released. Nothing module-owned exists yet in Phase 0, so
     * this is limited to resuming the recording and flagging the swap in the HUD. */
    if (game->replay.mode == REPLAY_MODE_IDLE) {
        game->replay.mode = REPLAY_MODE_RECORDING;
    }

    game->reloadCount++;
    game->reloadFlashTimerS = RELOAD_FLASH_S;

#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: post-reload #%d (tick %llu, checksum %08x)",
             game->reloadCount, (unsigned long long)game->sim.tick, game->stateChecksum);
#endif
}

GAME_API void game_fixed_update(Game *game, float dt)
{
    if (game == NULL) return;

    /* 1. Render transform history, before anything integrates. */
    game->renderState.prevPositionM  = game->renderState.currPositionM;
    game->renderState.prevHeadingRad = game->renderState.currHeadingRad;

    /* 2. Pick this tick's input. Playback substitutes the recorded timeline for the live
     *    controls, which is what lets the headless harness drive the same code path. */
    Input tickInput;
    input_zero(&tickInput);

    bool fromPlayback = false;
    if (game->replay.mode == REPLAY_MODE_PLAYBACK) {
        if (replay_next(&game->replay, &tickInput)) {
            fromPlayback = true;
        } else {
            replay_stop(&game->replay);     /* timeline exhausted */
        }
    }
    if (!fromPlayback) tickInput = game->input;

    /* 3. Consume the live one-shot commands. This happens in the FIRST fixed update that
     *    observes them, so the remaining substeps of the same render frame see nothing and
     *    a single-frame press is acted on exactly once. Held controls are untouched and
     *    stay valid for every substep. */
    input_clear_oneshots(&game->input);

    /* 4. Record what this tick actually used. */
    replay_record(&game->replay, &tickInput);

    /* 5. Act on the one-shot commands. */
    apply_oneshots(game, &tickInput);

    /* 6. Advance the deterministic placeholder transform. */
    if (game->state == STATE_PLAYING) sim_advance_marker(game, &tickInput, dt);

    game->sim.tick++;

    game->renderState.currPositionM  = game->sim.markerPositionM;
    game->renderState.currHeadingRad = game->sim.markerHeadingRad;

    game->stateChecksum = game_state_checksum(game);
}

#if defined(DRIFTY_HEADLESS)

GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    /* The headless build links no rendering code and opens no window. */
    (void)game;
    (void)interpolationAlpha;
}

GAME_API void game_shutdown(Game *game)
{
    (void)game;
}

#else

static const char *state_name(GameStateId state)
{
    switch (state) {
        case STATE_MENU:    return "MENU";
        case STATE_PLAYING: return "PLAYING";
        case STATE_PAUSED:  return "PAUSED";
        case STATE_RESULTS: return "RESULTS";
        case STATE_COUNT:   break;
    }
    return "UNKNOWN";
}

static const char *replay_mode_name(ReplayMode mode)
{
    switch (mode) {
        case REPLAY_MODE_IDLE:      return "idle";
        case REPLAY_MODE_RECORDING: return "recording";
        case REPLAY_MODE_PLAYBACK:  return "playback";
    }
    return "unknown";
}

/* Draw one HUD line and advance the cursor. */
static void hud_line(int x, int *y, const char *text, Color color)
{
    DrawText(text, x, *y, 18, color);
    *y += 22;
}

GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    if (game == NULL) return;

    const float renderDt = GetFrameTime();
    if (game->reloadFlashTimerS > 0.0f) {
        game->reloadFlashTimerS -= renderDt;
        if (game->reloadFlashTimerS < 0.0f) game->reloadFlashTimerS = 0.0f;
    }

    const float alpha = clampf(interpolationAlpha, 0.0f, 1.0f);

    /* Interpolated render transform. lerp_angle takes the shortest wrapped path, so a
     * heading crossing +-PI does not spin the marker through a full rotation. */
    Vector2 drawPosM;
    drawPosM.x = lerpf(game->renderState.prevPositionM.x, game->renderState.currPositionM.x, alpha);
    drawPosM.y = lerpf(game->renderState.prevPositionM.y, game->renderState.currPositionM.y, alpha);
    const float drawHeadingRad = lerp_angle(game->renderState.prevHeadingRad,
                                            game->renderState.currHeadingRad, alpha);

    const float ppm = game->renderPixelsPerMeter;
    const Vector2 posPx = units_world_to_render_px(drawPosM, ppm);

    BeginDrawing();
    ClearBackground((Color){ 22, 24, 28, 255 });

    /* World: a fixed origin cross plus the interpolated marker. The view is centred on the
     * screen; a real Camera2D arrives with the renderer in Phase 1. */
    const float originX = (float)SCREEN_W * 0.5f;
    const float originY = (float)SCREEN_H * 0.5f;

    DrawLine((int)originX - 40, (int)originY, (int)originX + 40, (int)originY, (Color){ 60, 64, 72, 255 });
    DrawLine((int)originX, (int)originY - 40, (int)originX, (int)originY + 40, (Color){ 60, 64, 72, 255 });

    const Rectangle marker = {
        originX + posPx.x,
        originY + posPx.y,
        units_meters_to_pixels(4.0f, ppm),
        units_meters_to_pixels(1.8f, ppm)
    };
    const Vector2 markerOrigin = { marker.width * 0.5f, marker.height * 0.5f };

    DrawRectanglePro(marker, markerOrigin, units_heading_to_rotation_deg(drawHeadingRad),
                     (Color){ 220, 90, 60, 255 });

    /* Nose marker, so the heading convention is readable at a glance. */
    const Vector2 nosePx = {
        marker.x + cosf(drawHeadingRad) * units_meters_to_pixels(2.4f, ppm),
        marker.y - sinf(drawHeadingRad) * units_meters_to_pixels(2.4f, ppm)
    };
    DrawCircleV(nosePx, 5.0f, (Color){ 250, 220, 120, 255 });

    /* HUD. */
    int x = 16;
    int y = 14;
    const Color label = (Color){ 235, 235, 235, 255 };
    const Color dim   = (Color){ 150, 155, 165, 255 };

    hud_line(x, &y, "DRIFTY  Phase 0 - foundations only, no vehicle physics", label);
    hud_line(x, &y, TextFormat("state %s   fps %d   render dt %.4f s",
                               state_name(game->state), GetFPS(), (double)renderDt), dim);
    hud_line(x, &y, TextFormat("substeps %d / %d   backlog drops %d   alpha %.3f",
                               game->lastSubstepCount, MAX_PHYSICS_STEPS,
                               game->physicsBacklogDrops, (double)alpha), label);
    hud_line(x, &y, TextFormat("tick %llu   sim time %.3f s   checksum %08x",
                               (unsigned long long)game->sim.tick,
                               (double)game->sim.tick * (double)FIXED_DT_S,
                               game->stateChecksum), dim);

    y += 8;
    hud_line(x, &y, "one-shot commands, counted once per press:", label);
    hud_line(x, &y, TextFormat("  reset(R) %u   pause(P) %u   debug(F1) %u   shift up(E) %u   down(Q) %u",
                               game->sim.resetCount, game->sim.pauseToggleCount,
                               game->sim.debugToggleCount, game->sim.shiftUpCount,
                               game->sim.shiftDownCount), dim);

    if (game->reloadFlashTimerS > 0.0f) {
        y += 8;
        hud_line(x, &y, TextFormat("MODULE RELOADED  (#%d, state preserved)", game->reloadCount),
                 (Color){ 120, 230, 140, 255 });
    }

    if (game->debugOverlay) {
        y += 8;
        hud_line(x, &y, "-- debug overlay (F1) --", label);
        hud_line(x, &y, TextFormat("held: steer %+.2f  throttle %.2f  brake %.2f  handbrake %.2f",
                                   (double)game->input.steer, (double)game->input.throttle,
                                   (double)game->input.brake, (double)game->input.handbrake), dim);
        hud_line(x, &y, TextFormat("accumulator %.6f s   fixed dt %.6f s   %d Hz",
                                   (double)game->accumulatorS, (double)FIXED_DT_S, FIXED_HZ), dim);
        hud_line(x, &y, TextFormat("marker  pos (%+.3f, %+.3f) m   heading %+.4f rad",
                                   (double)game->sim.markerPositionM.x,
                                   (double)game->sim.markerPositionM.y,
                                   (double)game->sim.markerHeadingRad), dim);
        hud_line(x, &y, TextFormat("render scale %.1f px/m  (rendering only)", (double)ppm), dim);
        hud_line(x, &y, TextFormat("replay %s  %d/%d ticks  overwritten %llu",
                                   replay_mode_name(game->replay.mode),
                                   game->replay.count, REPLAY_CAPACITY_TICKS,
                                   (unsigned long long)game->replay.overwrittenTicks), dim);
        hud_line(x, &y, TextFormat("module reloads %d", game->reloadCount), dim);
    }

    DrawText("WASD / arrows steer+drive   space handbrake   P pause   R reset   F1 debug",
             16, SCREEN_H - 30, 18, dim);

    EndDrawing();
}

GAME_API void game_shutdown(Game *game)
{
    if (game == NULL) return;
    TRACELOG(LOG_INFO, "GAME: shutdown after %llu fixed ticks (checksum %08x)",
             (unsigned long long)game->sim.tick, game->stateChecksum);
}

#endif /* DRIFTY_HEADLESS */
