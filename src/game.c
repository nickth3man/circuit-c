/*
 * game.c — reloadable game entry points and deterministic Phase 1 dispatch.
 *
 * physics.c is the sole owner of vehicle integration. This file retains Phase 0's input,
 * one-shot, replay, checksum, and module-lifecycle guarantees.
 */
#include "game.h"

#include <string.h>

#include "physics.h"
#include "render.h"

#if !defined(DRIFTY_HEADLESS)
#include "raylib.h"
#endif

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
    h = hash_u32(h, game->sim.resetCount);
    h = hash_u32(h, game->sim.pauseToggleCount);
    h = hash_u32(h, game->sim.debugToggleCount);
    h = hash_u32(h, game->sim.shiftUpCount);
    h = hash_u32(h, game->sim.shiftDownCount);

    const VehicleState *v = &game->vehicle;
    h = hash_f32(h, v->positionM.x);
    h = hash_f32(h, v->positionM.y);
    h = hash_f32(h, v->headingRad);
    h = hash_f32(h, v->velocityLongitudinalMps);
    h = hash_f32(h, v->velocityLateralMps);
    h = hash_f32(h, v->yawRateRadS);
    h = hash_f32(h, v->frontRoadWheelAngleRad);
    h = hash_f32(h, v->engineRpm);
    h = hash_u32(h, (uint32_t)v->selectedGear);
    h = hash_f32(h, v->filteredLongAccelMps2);
    h = hash_f32(h, v->prevLongAccelMps2);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const WheelState *wheel = &v->wheels[i];
        h = hash_f32(h, wheel->localPositionM.x);
        h = hash_f32(h, wheel->localPositionM.y);
        h = hash_f32(h, wheel->steerAngleRad);
        h = hash_f32(h, wheel->angularVelocityRadS);
        h = hash_f32(h, wheel->normalLoadN);
        h = hash_f32(h, wheel->slipAngleRad);
        h = hash_f32(h, wheel->slipRatio);
        h = hash_f32(h, wheel->forceLongitudinalN);
        h = hash_f32(h, wheel->forceLateralN);
        h = hash_f32(h, wheel->frictionUsage);
        h = hash_u32(h, wheel->locked ? 1u : 0u);
        h = hash_u32(h, (uint32_t)wheel->surfaceId);
    }
    return h;
}

GAME_API void game_reset_sim(Game *game)
{
    if (game == NULL) return;
    vehicle_state_reset(&game->spec, &game->vehicle, &game->derived, &game->renderState);
}

static void apply_oneshots(Game *game, const Input *input)
{
    if (input->pausePressed) {
        if (game->state == STATE_PLAYING) game->state = STATE_PAUSED;
        else if (game->state == STATE_PAUSED) game->state = STATE_PLAYING;
        game->sim.pauseToggleCount++;
    }
    if (input->resetPressed) {
        game_reset_sim(game);
        game->sim.resetCount++;
    }
    if (input->debugPressed) {
        game->debugOverlay = !game->debugOverlay;
        game->sim.debugToggleCount++;
    }
    /* Until Phase 2, these choose direction only; there are no ratios or engine behavior. */
    if (input->shiftUpPressed) {
        game->vehicle.selectedGear = 1;
        game->sim.shiftUpCount++;
    }
    if (input->shiftDownPressed) {
        game->vehicle.selectedGear = -1;
        game->sim.shiftDownCount++;
    }
}

GAME_API void game_init(Game *game)
{
    if (game == NULL) return;
    input_zero(&game->input);
    memset(&game->sim, 0, sizeof(game->sim));
    vehicle_spec_set_default(&game->spec);
    game_reset_sim(game);
    game->state = STATE_PLAYING;
    game->accumulatorS = 0.0f;
    game->lastSubstepCount = 0;
    game->physicsBacklogDrops = 0;
    game->debugOverlay = false;
    game->reloadCount = 0;
    game->reloadFlashTimerS = 0.0f;
    game->renderPixelsPerMeter = PIXELS_PER_METER;
    game->camera = (Camera2D){
        .offset = { SCREEN_W * 0.5f, SCREEN_H * 0.5f },
        .target = { 0.0f, 0.0f },
        .rotation = 0.0f,
        .zoom = 1.0f
    };
    replay_begin_recording(&game->replay, 0);
    game->stateChecksum = game_state_checksum(game);
    game->initialized = true;
#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: initialised (Phase 1 rigid-body vehicle)");
#endif
}

GAME_API void game_pre_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_RECORDING) replay_stop(&game->replay);
#if !defined(DRIFTY_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: pre-reload (tick %llu)",
             (unsigned long long)game->sim.tick);
#endif
}

GAME_API void game_post_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_IDLE) game->replay.mode = REPLAY_MODE_RECORDING;
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
    Input tickInput;
    input_zero(&tickInput);
    bool fromPlayback = false;
    if (game->replay.mode == REPLAY_MODE_PLAYBACK) {
        if (replay_next(&game->replay, &tickInput)) fromPlayback = true;
        else replay_stop(&game->replay);
    }
    if (!fromPlayback) tickInput = game->input;
    input_clear_oneshots(&game->input);
    replay_record(&game->replay, &tickInput);
    apply_oneshots(game, &tickInput);
    if (game->state == STATE_PLAYING) {
        physics_fixed_update(&game->spec, &game->vehicle, &game->derived,
                             &game->renderState, &tickInput, dt);
    }
    game->sim.tick++;
    game->stateChecksum = game_state_checksum(game);
}

#if defined(DRIFTY_HEADLESS)
GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}

GAME_API void game_shutdown(Game *game)
{
    (void)game;
}
#else
GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    render_draw_game(game, interpolationAlpha);
}

GAME_API void game_shutdown(Game *game)
{
    if (game == NULL) return;
    TRACELOG(LOG_INFO, "GAME: shutdown after %llu fixed ticks (checksum %08x)",
             (unsigned long long)game->sim.tick, game->stateChecksum);
}
#endif
