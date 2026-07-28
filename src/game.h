/*
 * game.h — the platform-owned persistent Game block.
 *
 * OWNERSHIP. A single Game structure, allocated once by the platform layer (main.c) with
 * calloc and passed to the game module by pointer on every entry point. It is deliberately
 * NOT a `static Game game;` inside the module: BSS belongs to whichever module declares it,
 * so a module-owned Game would be destroyed on every hot reload.
 *
 * RELOAD SAFETY. Nothing reachable from Game may point into the game module's code or
 * static data, and no function pointers may be stored here. See the invariant spelled out
 * in hotreload.h. Everything below is plain value data, which is what lets the block
 * survive a module swap unchanged.
 *
 * Phase 2 extends the embedded canonical vehicle structures. This layout change requires
 * one platform restart; subsequent game-module-only edits preserve the block normally.
 */
#ifndef DRIFTY_GAME_H
#define DRIFTY_GAME_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"     /* Vector2 only; game.c is the only TU here that calls raylib */

#include "config.h"
#include "dev_state.h"
#include "hotreload.h"
#include "input.h"
#include "particle.h"
#include "replay.h"
#include "track.h"
#include "vehicle.h"
#include "auto_transmission.h"

typedef enum {
    STATE_MENU = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_RESULTS,
    STATE_COUNT
} GameStateId;

typedef struct {
    uint64_t tick;
    uint32_t resetCount;
    uint32_t pauseToggleCount;
    uint32_t debugToggleCount;
    uint32_t shiftUpCount;
    uint32_t shiftDownCount;
} SimState;

struct Game {
    GameStateId    state;
    Input          input;
    SimState       sim;
    VehicleSpec        spec;
    VehicleState       vehicle;
    VehicleDerived     derived;
    VehicleRenderState renderState;
    Track               track;
    ParticlePool        particles;
    Camera2D            camera;

    /* Fixed-timestep bookkeeping, written by the platform loop via timestep_advance(). */
    float accumulatorS;
    int   lastSubstepCount;
    int   physicsBacklogDrops;

    /* Deterministic input recording and playback. Fixed capacity, no allocation. */
    ReplayBuffer replay;

    /* Rolling checksum of the deterministic simulation state, recomputed every fixed
     * update. Two runs of the same input timeline must agree on this value. */
    uint32_t stateChecksum;

    /* Render-only. Initialised from PIXELS_PER_METER and consumed exclusively by units.h
     * helpers in game_draw(). No simulation quantity may read it; the "renderscale"
     * scenario in tests/physics_tests.c asserts that by running the same timeline at two
     * different scales and comparing checksums. */
    float renderPixelsPerMeter;

    /* Presentation and diagnostics. */
    float driftScore;
    float bestScore;
    float driftTimeS;
    float comboMultiplier;
    float comboTimerS;
    float crashLockoutTimerS;   /* seconds remaining in the post-impact scoring lockout */
    bool  debugOverlay;
    int   reloadCount;
    float reloadFlashTimerS;
    bool  initialized;

    /* Automatic transmission mode (toggle with T). */
    AutoTransmission autoTrans;

    /* Development tooling: Physics Lab, scope history, trajectory, invariant monitor, and
     * the time controls the platform loop reads. Plain value data like everything else here,
     * and present in every build configuration so that drifty.exe and build/game.dll cannot
     * disagree about the layout of this struct. See src/dev_state.h. */
    DevState dev;
};

/*
 * Helpers exposed to the platform layer and the headless harness. These are ordinary
 * module functions, not reloadable entry points: the entry-point list in hotreload.h stays
 * exactly as docs/SPEC.md defines it.
 */

/* FNV-1a over the deterministic simulation fields only. Explicitly excludes the
 * accumulator, the substep and backlog counters, the render scale, and every presentation
 * field, so the checksum depends on the input timeline and nothing else. */
GAME_API uint32_t game_state_checksum(const Game *game);

/* Reset the vehicle and resynchronise render history. Counters and tick are preserved. */
GAME_API void game_reset_sim(Game *game);

#endif /* DRIFTY_GAME_H */
