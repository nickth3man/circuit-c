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
 *
 * Splitting Track into TrackDefinition/TrackRuntime/RacerProgress is likewise a layout
 * change, so it too costs one platform restart.
 *
 * Giving the entrant its own Controller (issue 9) is a layout change for the same reason and
 * costs the same one restart. It stores a kind, a frozen config and private decision memory —
 * plain values, no function pointer, so the reload invariant above still holds.
 *
 * Moving that entrant into a RaceRoster (issue 10) is the last of those layout changes, and the
 * roster is plain value storage like everything else here, so the invariant is unchanged.
 */
#ifndef CIRCUIT_GAME_H
#define CIRCUIT_GAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "raylib.h" /* Vector2 only; game.c is the only TU here that calls raylib */

#include "core/config.h"
#include "dev/dev_state.h"
#include "platform/hotreload.h"
#include "game/controller.h"
#include "game/input.h"
#include "game/particle.h"
#include "game/race_entrant.h"
#include "game/replay.h"
#include "world/track.h"
#include "physics/vehicle.h"
#include "physics/auto_transmission.h"
#include "game/telemetry.h"

typedef enum {
    STATE_MENU = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_RESULTS,
    STATE_COUNT
} GameStateId;

typedef enum {
    GAME_TRACK_KEEP = 0, /* leave whatever game_init() loaded */
    GAME_TRACK_PARKING_LOT,
    GAME_TRACK_CHICANE,
    GAME_TRACK_SPRINT,
    GAME_TRACK_TECHNICAL,
    GAME_TRACK_COUNT
} GameTrackId;

/*
 * What a bounded run should be set up with. Plain value data passed by pointer from the
 * platform layer, which cannot reach track or vehicle code directly — those live in the
 * reloadable module. Nothing here is retained: game_configure_run() reads it and returns.
 */
struct GameRunConfig {
    GameTrackId track;
    float cameraZoomOverride; /* 0 leaves the follow camera's own choice alone */
    /* Laps to complete before the run ends and the car stops being simulated. 0 means the
     * gameplay default, RESULTS_TARGET_LAPS. This is a run property rather than a constant
     * because a validation run and a race are not the same length: a validation run wants an
     * out-lap it can throw away plus enough timed laps to average, and a race wants a race. */
    int targetLaps;
};

typedef struct {
    uint64_t tick;
    uint32_t resetCount;
    uint32_t pauseToggleCount;
    uint32_t debugToggleCount;
    uint32_t shiftUpCount;
    uint32_t shiftDownCount;
} SimState;

struct Game {
    GameStateId state;
    /* The platform's frame-latched device sample and the session's application commands. It is
     * an input SOURCE, not a vehicle control: the controller stage below converts it into one
     * entrant's ControllerOutput once per fixed tick. See src/game/controller.h. */
    Input input;
    SimState sim;
    /*
     * Every entrant in the session. This is the storage — there is no separate "the player's
     * car" any more, only `roster.entrants[i]`, walked in ascending EntrantId order.
     *
     * The anonymous view beside it is the temporary one-entrant compatibility spelling. It
     * overlays `entrants[0]`, so `game->vehicle`, `game->progress` and the rest keep naming
     * that entrant's members while the subsystems that still assume a single car migrate to
     * entrant iteration. The _Static_asserts below this struct hold the two layouts together,
     * so reordering RaceEntrant is a compile error rather than a silent misread. New code
     * should go through race_roster_local() / race_roster_find(); the view is deleted once
     * nothing reads it. See docs/SIMULATION_OWNERSHIP.md.
     */
    union {
        RaceRoster roster;
        struct {
            /* Issue #8's view, one level down: these five values are physically owned by one
             * VehicleInstance and spelled here as the fields callers already use. */
            union {
                VehicleInstance vehicleInstance;
                struct {
                    VehicleSpec spec;
                    VehicleState vehicle;
                    VehicleDerived derived;
                    VehicleRenderState renderState;
                    AutoTransmission autoTrans;
                    VehicleControlState vehicleControls;
                    float fuelKg;
                    VehicleTireState tireState[WHEEL_COUNT];
                    float damage;
                    float crashLockoutTimerS;
                };
            };
            /* This entrant's controller: kind, frozen config, and private decision memory. */
            Controller controller;
            /* What that controller asked for on the most recent fixed tick, before the
             * pre-physics gating stage (countdown rules, automatic transmission) rewrote it.
             * Excluded from the state checksum: it is recomputed every tick from the tick's
             * input source and no later tick reads it. The gated controls physics actually
             * consumed are `vehicleControls`. */
            ControllerOutput controllerOutput;
            /* This entrant's lap cursor around the shared track definition. */
            RacerProgress progress;
            VehicleDefinition vehicleDefinition;
            VehicleSetup vehicleSetup;
        };
    };
    /* Track ownership is split three ways: `trackDef` is immutable authored content shared by
     * every entrant, `trackRuntime` is session-wide mutable track state, and each entrant's
     * lap cursor is the RacerProgress inside its own roster slot.
     * See docs/SIMULATION_OWNERSHIP.md. */
    TrackDefinition trackDef;
    TrackRuntime trackRuntime;
    /* What the checkpoint test saw on the most recent fixed tick. Plain value data, rewritten
     * every tick, and excluded from the state checksum because it is a report about the
     * simulation rather than part of it. */
    TrackCheckpointEvent lastCheckpointEvent;
    TrackCheckpointEvent pendingTelemetryCheckpointEvent;
    ParticlePool particles;
    Camera2D camera;

    /* Fixed-timestep bookkeeping, written by the platform loop via timestep_advance(). */
    float accumulatorS;
    int lastSubstepCount;
    int physicsBacklogDrops;

    /* Deterministic input recording and playback. Fixed capacity, no allocation. */
    ReplayBuffer replay;

    /* Rolling checksum of the deterministic simulation state, recomputed every fixed
     * update. Two runs of the same input timeline must agree on this value. */
    uint32_t stateChecksum;

    /* Render-only. Initialised from PIXELS_PER_METER and consumed exclusively by units.h
     * helpers in game_draw(). No simulation quantity may read it; the "renderscale"
     * scenario in tests/scenarios/core_tests.c asserts that by running the same timeline at two
     * different scales and comparing checksums. */
    float renderPixelsPerMeter;

    /* Laps this run ends after. Set by game_configure_run() from GameRunConfig.targetLaps and
     * defaulted to RESULTS_TARGET_LAPS by game_init(), so a caller that never configures a run
     * gets exactly the behaviour the constant always gave. */
    int targetLaps;

    /* Presentation and diagnostics. */
    bool debugOverlay;
    int reloadCount;
    float reloadFlashTimerS;
    bool initialized;

    /* Development tooling: Physics Lab, scope history, trajectory, invariant monitor, and
     * the time controls the platform loop reads. Plain value data like everything else here,
     * and present in every build configuration so that circuit.exe and build/game.dll cannot
     * disagree about the layout of this struct. See src/dev/dev_state.h. */
    DevState dev;
};

/*
 * The one-entrant compatibility view must land exactly on `roster.entrants[0]`. These
 * assertions are the whole reason the view is safe to keep: add, remove or reorder a member of
 * RaceEntrant and the build stops here instead of quietly aliasing the wrong bytes.
 */
_Static_assert(offsetof(RaceRoster, entrants) == 0,
               "the entrant array must start the roster: Game overlays entrants[0]");
_Static_assert(offsetof(RaceEntrant, instance) == 0,
               "VehicleInstance must start RaceEntrant: Game overlays it as spec/vehicle/...");
_Static_assert(offsetof(Game, vehicleInstance) == offsetof(Game, roster),
               "the compatibility view must alias entrants[0]");
_Static_assert(offsetof(Game, controller) ==
                   offsetof(Game, roster) + offsetof(RaceEntrant, controller),
               "Game.controller must alias entrants[0].controller");
_Static_assert(offsetof(Game, controllerOutput) ==
                   offsetof(Game, roster) + offsetof(RaceEntrant, controllerOutput),
               "Game.controllerOutput must alias entrants[0].controllerOutput");
_Static_assert(offsetof(Game, progress) ==
                   offsetof(Game, roster) + offsetof(RaceEntrant, progress),
               "Game.progress must alias entrants[0].progress");
_Static_assert(offsetof(Game, vehicleDefinition) ==
                   offsetof(Game, roster) + offsetof(RaceEntrant, definition),
               "Game.vehicleDefinition must alias entrants[0].definition");
_Static_assert(offsetof(Game, vehicleSetup) ==
                   offsetof(Game, roster) + offsetof(RaceEntrant, setup),
               "Game.vehicleSetup must alias entrants[0].setup");

/*
 * Helpers exposed to the platform layer and the headless harness. These are ordinary
 * module functions, not reloadable entry points: the entry-point list in hotreload.h stays
 * exactly as the GAME_ENTRY_POINTS X-macro in src/platform/hotreload.h defines it.
 */

/* FNV-1a over the deterministic simulation fields only. Explicitly excludes the
 * accumulator, the substep and backlog counters, the render scale, and every presentation
 * field, so the checksum depends on the input timeline and nothing else. */
GAME_API uint32_t game_state_checksum(const Game *game);

/* Reset the vehicle and resynchronise render history. Counters and tick are preserved. */
GAME_API void game_reset_sim(Game *game);

#endif /* CIRCUIT_GAME_H */
