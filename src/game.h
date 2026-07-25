/*
 * game.h — the persistent Game block and the Phase 0 state it holds.
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
 * PHASE 0 SCOPE. This is the minimum state the Phase 0 systems need. VehicleSpec,
 * VehicleState, VehicleDerived, VehicleRenderState, Track, ParticlePool, Camera2D, and the
 * scoring fields are defined in docs/SPEC.md "Canonical Data Structures" and are added by
 * the phases that own them. Adding them changes this struct's layout, which requires
 * restarting drifty.exe — expected, and documented in README.md.
 */
#ifndef DRIFTY_GAME_H
#define DRIFTY_GAME_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"     /* Vector2 only; game.c is the only TU here that calls raylib */

#include "config.h"
#include "hotreload.h"
#include "input.h"
#include "replay.h"

typedef enum {
    STATE_MENU = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_RESULTS,
    STATE_COUNT
} GameStateId;

/*
 * Phase 0 deterministic placeholder transform.
 *
 * This is NOT a vehicle model and contains no physics: heading integrates the steer axis at
 * a constant rate and position integrates a constant speed along that heading. Its only
 * jobs are to give the fixed-timestep loop something observable to advance, to give the
 * renderer something to interpolate, and to give the replay harness something to checksum.
 * physics.c replaces it wholesale in Phase 1 and owns the fixed update order from then on.
 */
typedef struct {
    uint64_t tick;                  /* fixed updates executed since init */
    Vector2  markerPositionM;       /* world meters */
    float    markerHeadingRad;      /* radians, counterclockwise positive, wrapped */

    /* One-shot command counters. Each increments by exactly one per press, no matter how
     * many substeps run in the frame that observed it. Surfaced in the debug HUD. */
    uint32_t resetCount;
    uint32_t pauseToggleCount;
    uint32_t debugToggleCount;
    uint32_t shiftUpCount;
    uint32_t shiftDownCount;
} SimState;

/* prev* is copied from curr* at the start of every fixed update, before integration. */
typedef struct {
    Vector2 prevPositionM;
    float   prevHeadingRad;
    Vector2 currPositionM;
    float   currHeadingRad;
} SimRenderState;

struct Game {
    GameStateId    state;
    Input          input;
    SimState       sim;
    SimRenderState renderState;

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
    bool  debugOverlay;
    int   reloadCount;
    float reloadFlashTimerS;
    bool  initialized;
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

/* Return the placeholder transform to the origin and resynchronise the render history so
 * the reset does not smear across one interpolated frame. Counters are preserved. */
GAME_API void game_reset_sim(Game *game);

#endif /* DRIFTY_GAME_H */
