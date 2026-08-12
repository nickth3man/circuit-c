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
 * Moving that entrant into a RaceRoster (issue 10), and the roster into a RaceSession (issue
 * 11), are layout changes for the same reason and cost the same one restart. Both are plain
 * value storage — no function pointers, nothing pointing into the reloadable module — so the
 * invariant above is unchanged.
 *
 * Embedding the CollisionWorld in TrackRuntime (issue 26) is a layout change for the same
 * reason and costs the same one restart. It is plain value storage (~180 KB of fixed arrays,
 * no heap, no function pointers), so the reload invariant above is unchanged.
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
#include "game/car_selection.h"
#include "game/controller.h"
#include "game/setup_editor.h"
#include "game/input.h"
#include "game/particle.h"
#include "game/race_session.h"
#include "game/replay.h"
#include "world/track.h"
#include "physics/vehicle.h"
#include "physics/auto_transmission.h"
#include "game/telemetry.h"
#include "game/session_config.h"

typedef enum {
    STATE_MENU = 0,
    STATE_PLAYING,
    STATE_PAUSED,
    STATE_RESULTS,
    STATE_COUNT
} GameStateId;
/*
 * What a bounded run should be set up with. Plain value data passed by pointer from the
 * platform layer, which cannot reach track or vehicle code directly — those live in the
 * reloadable module. Nothing here is retained: game_configure_run() reads it and returns.
 *
 * `trackId` is a stable content id (see track_manifest.h). An empty string means
 * "leave whatever is loaded" (the GAME_TRACK_KEEP sentinel in earlier issues). When non-empty
 * it must be a valid track id and the corresponding file must exist under data/tracks/.
 */
struct GameRunConfig {
    char trackId[TRACK_ID_CHARS];
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
    /*
     * The menu's car selection: an index into the car_selection_* enumeration (id-sorted
     * roster order — see car_selection.h) plus the stable id of that entry. -1 with an
     * empty id when the catalog holds nothing selectable. game_init() resolves the
     * persisted choice, the menu HUD renders and cycles it (LEFT/RIGHT), and
     * restart_session() spawns entrants[0] from it. Deliberately excluded from the state
     * checksum: a menu choice must not perturb the determinism of a running session — the
     * entrant's definition and setup are already hashed.
     */
    int selectedCarIndex;
    char selectedCarId[CAR_SELECTION_ID_CHARS];
    /*
     * The setup editor for the selected car (issue #33). `setupEditor` holds a working copy
     * of the selected car's default setup; the menu HUD shows it while `setupEditing` is on.
     * On start, when `setupCustomized` is true, that working setup is applied to the player
     * entrant instead of the manifest default, so a tuned setup reaches the race without ever
     * mutating the shared definition. Excluded from the state checksum for the same reason as
     * the selection above. Interactive menu only; headless never enters STATE_MENU.
     */
    bool setupEditing;
    bool setupCustomized;
    int setupCursor;
    SetupEditor setupEditor;
    /* Why the last start attempt was refused, or empty when nothing has been refused. Written
     * by the menu's start path from game_can_start_race() and shown by the menu HUD, so a
     * blocked start explains itself instead of looking like an unresponsive key. */
    char startBlockedReason[96];
    /* The platform's frame-latched device sample and the session's application commands. It is
     * an input SOURCE, not a vehicle control: the controller stage below converts it into one
     * entrant's ControllerOutput once per fixed tick. See src/game/controller.h. */
    Input input;
    SimState sim;
    /*
     * The active race: its roster, rules, phase, clock, events and results. Gameplay authority
     * lives here rather than in `state` above, which now only chooses a screen.
     *
     * The anonymous view beside it is the temporary one-entrant compatibility spelling. The
     * session's first member is its roster, whose first member is `entrants[0]`, so this view
     * lands on that entrant and `game->vehicle`, `game->progress` and the rest keep naming its
     * members while the subsystems that still assume a single car migrate to entrant
     * iteration. The _Static_asserts below this struct hold the two layouts together, so
     * reordering RaceSession or RaceEntrant is a compile error rather than a silent misread.
     * New code should go through race_roster_local() / race_roster_find(); the view is deleted
     * once nothing reads it. See docs/SIMULATION_OWNERSHIP.md.
     *
     * PRECONDITION. The staged fixed update drives exactly one car — the one this view names,
     * `entrants[0]` — because per-entrant iteration of physics, collision and progress is the
     * subject of the collision and localization issues (#26/#27, #38), not of the session
     * skeleton. A Game whose roster holds more than one entrant would therefore leave every
     * other slot unsimulated, and a local entrant that sorts after some other id would be
     * presented but never driven. game_init() spawns exactly one entrant, so no supported path
     * reaches that state. Rosters and sessions used directly, as the "race-entrant" and
     * "race-session" scenarios do, are storage and are not subject to this.
     */
    union {
        RaceSession session;
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
_Static_assert(offsetof(RaceSession, roster) == 0,
               "the roster must start the session: Game overlays entrants[0] through it");
_Static_assert(offsetof(RaceRoster, entrants) == 0,
               "the entrant array must start the roster: Game overlays entrants[0]");
_Static_assert(offsetof(RaceEntrant, instance) == 0,
               "VehicleInstance must start RaceEntrant: Game overlays it as spec/vehicle/...");
_Static_assert(offsetof(Game, vehicleInstance) == offsetof(Game, session),
               "the compatibility view must alias entrants[0]");
_Static_assert(offsetof(Game, controller) ==
                   offsetof(Game, session) + offsetof(RaceEntrant, controller),
               "Game.controller must alias entrants[0].controller");
_Static_assert(offsetof(Game, controllerOutput) ==
                   offsetof(Game, session) + offsetof(RaceEntrant, controllerOutput),
               "Game.controllerOutput must alias entrants[0].controllerOutput");
_Static_assert(offsetof(Game, progress) ==
                   offsetof(Game, session) + offsetof(RaceEntrant, progress),
               "Game.progress must alias entrants[0].progress");
_Static_assert(offsetof(Game, vehicleDefinition) ==
                   offsetof(Game, session) + offsetof(RaceEntrant, definition),
               "Game.vehicleDefinition must alias entrants[0].definition");
_Static_assert(offsetof(Game, vehicleSetup) ==
                   offsetof(Game, session) + offsetof(RaceEntrant, setup),
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

/* One entrant's authoritative checksum contribution (issue #44): hashes exactly the fields
 * game_state_checksum folds into the session digest for this entrant, so a divergence report
 * can name the offending entrant by comparing per-entrant hashes. Returns 0 for a missing id.
 */
GAME_API uint32_t game_entrant_state_checksum(const Game *game, EntrantId id);

/* First-divergence diagnostic (issue #44): walks two games' authoritative state in the same
 * field order as the checksum and writes the first difference found into `out` as a compact
 * line naming the tick, entrant, and field (e.g. "tick 42 entrant 2 vehicle.velocityLateralMps
 * 1.234 vs 1.000"). Returns true when the games differ, false when they match. Never writes
 * past `cap`. */
GAME_API bool game_divergence_report(const Game *a, const Game *b, char *out, size_t cap);

/* Returns false with a human reason when the current menu selection/setup cannot start a
 * session (no car, setup outside vehicle bounds, missing class tag, or non-player-selectable
 * kind). Headless tests call this directly to prove invalid setups are rejected before a
 * session starts. */
GAME_API bool game_configure_session(Game *game, const SessionConfig *cfg, char *reason,
                                     size_t reasonCap);

/* Reset the vehicle and resynchronise render history. Counters and tick are preserved. */
GAME_API void game_reset_sim(Game *game);

#endif /* CIRCUIT_GAME_H */
