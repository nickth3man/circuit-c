/*
 * game.c — reloadable game entry points and deterministic Phase 2 dispatch.
 *
 * physics.c is the sole owner of vehicle integration. This file retains Phase 0's input,
 * one-shot, replay, checksum, and module-lifecycle guarantees.
 */
#include "game/game.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dev/dev_lab.h"
#include "dev/dev_scenario.h"
#include "dev/dev_state.h"
#include "game/audio.h"
#include "game/controller.h"
#include "physics/auto_transmission.h"
#include "world/collision.h"
#include "physics/physics.h"
#include "game/profile.h"
#include "render/render.h"
#include "world/track.h"

#if !defined(CIRCUIT_HEADLESS)
#include "raylib.h"
#endif

#if defined(_WIN32)
#include <direct.h>
#define CIRCUIT_MKDIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#define CIRCUIT_MKDIR(path) mkdir(path, 0755)
#endif

#define FNV1A_OFFSET_BASIS 2166136261u
#define FNV1A_PRIME 16777619u

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

/*
 * One entrant's authoritative mutable state, in a fixed field order.
 *
 * Identity is hashed first because ordering and membership are part of the result: two rosters
 * holding the same cars under different ids, or one that lost an entrant, must not agree.
 *
 * The private controller memory is the one piece of persistent entrant state deliberately left
 * out, and it is not an oversight. Playback calls controller_update() with
 * CONTROLLER_KIND_REPLAY, so a recorded run's AI memory is never rebuilt during replay while a
 * live run updates it every tick. Hashing it would make a live AI lap and its own replay
 * disagree on every tick — the "ai-no-privilege" scenario measures exactly that parity — which
 * would report a divergence where there is none. It is added when replay drives controllers
 * through their own memory rather than around it; see docs/SIMULATION_OWNERSHIP.md.
 */
static uint32_t hash_entrant(uint32_t h, const RaceEntrant *entrant)
{
    h = hash_u32(h, entrant->id);
    h = hash_u32(h, (uint32_t)entrant->gridSlot);
    h = hash_u32(h, entrant->result.finished ? 1u : 0u);
    h = hash_u32(h, (uint32_t)entrant->result.finishPosition);
    h = hash_f32(h, entrant->result.finishTimeS);

    const VehicleSetup *setup = &entrant->setup;
    h = hash_f32(h, setup->tirePressureFrontKpa);
    h = hash_f32(h, setup->tirePressureRearKpa);
    h = hash_f32(h, setup->suspCamberFrontRad);
    h = hash_f32(h, setup->suspCamberRearRad);
    h = hash_f32(h, setup->suspToeFrontRad);
    h = hash_f32(h, setup->suspToeRearRad);
    h = hash_f32(h, setup->suspCasterFrontRad);
    h = hash_f32(h, setup->suspCasterRearRad);
    h = hash_u32(h, (uint32_t)setup->gearCount);
    for (int i = 0; i < MAX_GEARS; i++) h = hash_f32(h, setup->gearRatios[i]);
    h = hash_f32(h, setup->reverseGearRatio);
    h = hash_f32(h, setup->finalDriveRatio);
    h = hash_f32(h, setup->brakeBiasFront);
    h = hash_f32(h, setup->differentialMode);
    h = hash_f32(h, setup->differentialBiasRatio);
    h = hash_f32(h, setup->differentialPreloadNm);

    const VehicleState *v = &entrant->instance.vehicle;
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
    h = hash_f32(h, v->filteredLatAccelMps2);
    h = hash_f32(h, v->prevLatAccelMps2);
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
        h = hash_f32(h, wheel->forceLateralRelaxedN);
        h = hash_f32(h, wheel->frictionUsage);
        h = hash_u32(h, wheel->locked ? 1u : 0u);
        h = hash_u32(h, (uint32_t)wheel->surfaceId);
    }

    const VehicleInstance *instance = &entrant->instance;
    h = hash_f32(h, instance->renderState.prevPositionM.x);
    h = hash_f32(h, instance->renderState.prevPositionM.y);
    h = hash_f32(h, instance->renderState.prevHeadingRad);
    h = hash_f32(h, instance->renderState.currPositionM.x);
    h = hash_f32(h, instance->renderState.currPositionM.y);
    h = hash_f32(h, instance->renderState.currHeadingRad);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        h = hash_f32(h, instance->renderState.prevWheelAngleRad[i]);
        h = hash_f32(h, instance->renderState.currWheelAngleRad[i]);
    }
    h = hash_u32(h, instance->autoTrans.enabled ? 1u : 0u);
    h = hash_u32(h, instance->autoTrans.forwardOnly ? 1u : 0u);
    h = hash_u32(h, (uint32_t)instance->autoTrans.driveState);
    h = hash_f32(h, instance->autoTrans.neutralTimer);
    h = hash_f32(h, instance->vehicleControls.steer);
    h = hash_f32(h, instance->vehicleControls.throttle);
    h = hash_f32(h, instance->vehicleControls.brake);
    h = hash_f32(h, instance->vehicleControls.handbrake);
    h = hash_f32(h, instance->fuelKg);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        h = hash_f32(h, instance->tireState[i].pressureKpa);
        h = hash_f32(h, instance->tireState[i].temperatureC);
        h = hash_f32(h, instance->tireState[i].wear);
    }
    h = hash_f32(h, instance->damage);
    h = hash_f32(h, instance->crashLockoutTimerS);

    /* Route progress is authoritative simulation state: it decides when a lap closes and when
     * the run ends, so a replay that diverged on it was previously undetectable. The bound
     * definition hash is deliberately NOT hashed here — an immutable input belongs to the
     * session compatibility digest, not to the rolling checksum. */
    const RacerProgress *p = &entrant->progress;
    h = hash_u32(h, (uint32_t)p->nextCheckpoint);
    h = hash_u32(h, (uint32_t)p->lap);
    h = hash_u32(h, (uint32_t)p->lapStartCheckpoint);
    h = hash_f32(h, p->lapTimerS);
    h = hash_f32(h, p->lastLapTimeS);
    return h;
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

    /* Ascending EntrantId, which is the roster's storage order.
     *
     * The two id cursors are hashed with the count because they decide what a LATER spawn is
     * called: a session that spawned and despawned an entrant and one that never spawned it
     * can hold identical entrants today and still name their next car differently, and a
     * divergence the checksum cannot see is the kind this project exists to catch.
     *
     * Which entrant the local presentation follows is deliberately absent: that is a
     * camera/audio decision, and a checksum that moved when the view changed would stop
     * meaning "the simulation diverged". */
    h = hash_u32(h, (uint32_t)game->session.roster.count);
    h = hash_u32(h, game->session.roster.nextId);
    h = hash_u32(h, game->session.roster.reuseFloorId);
    for (int i = 0; i < game->session.roster.count; i++) {
        h = hash_entrant(h, &game->session.roster.entrants[i]);
    }

    /* Session authority. Phase, race clock and countdown all decide whether a later tick
     * simulates at all, and classification decides what the next finisher is called, so a
     * divergence in any of them changes the future and belongs here.
     *
     * The event ring is summarised by its append count rather than hashed entry by entry: the
     * entries are a report that nothing reads back into the simulation, but two runs that
     * raised different numbers of events did not do the same thing. The results snapshot is
     * likewise derived — it is written from state already hashed above. */
    const RaceSession *session = &game->session;
    h = hash_u32(h, (uint32_t)session->phase);
    h = hash_u32(h, (uint32_t)session->resumePhase);
    h = hash_u32(h, (uint32_t)session->rules.mode);
    h = hash_u32(h, (uint32_t)session->rules.targetLaps);
    h = hash_f32(h, session->rules.countdownS);
    h = hash_u32(h, (uint32_t)session->trackId);
    h = hash_u64(h, session->tick);
    h = hash_f32(h, session->clockS);
    h = hash_u32(h, (uint32_t)session->countdownTicksRemaining);
    h = hash_u32(h, (uint32_t)session->classifiedCount);
    h = hash_u32(h, session->events.totalAppended);
    return h;
}

GAME_API void game_reset_sim(Game *game)
{
    if (game == NULL) return;
    /* Every entrant, in roster order. A car put back on the grid must not steer to a plan
     * computed for where it used to be, so each entrant's private controller memory goes with
     * its vehicle state; kind and frozen configuration survive, because the reset changes the
     * situation rather than who is driving. */
    race_roster_reset(&game->session.roster);
}

GAME_API void game_apply_spec(Game *game, const VehicleSpec *spec)
{
    if (game == NULL || spec == NULL) return;
    RaceEntrant *entrant = race_roster_local(&game->session.roster);
    if (entrant == NULL) return;

    VehicleDefinition definition;
    if (!vehicle_definition_init(&definition, "runtime/profile", "runtime/profile", 1u, spec))
        return;
    entrant->definition = definition;
    vehicle_setup_set_default(&entrant->definition, &entrant->setup);
    (void)vehicle_instance_init(&entrant->instance, &entrant->definition, &entrant->setup);
    controller_reset(&entrant->controller);
}

GAME_API bool game_configure_run(Game *game, const GameRunConfig *config)
{
    if (game == NULL || config == NULL) return false;

    switch (config->track) {
        case GAME_TRACK_PARKING_LOT: track_init(&game->trackDef); break;
        case GAME_TRACK_CHICANE: track_load_chicane(&game->trackDef); break;
        case GAME_TRACK_SPRINT: track_load_sprint(&game->trackDef); break;
        case GAME_TRACK_TECHNICAL: track_load_technical(&game->trackDef); break;
        case GAME_TRACK_KEEP: break;
        default: return false;
    }
    if (config->track != GAME_TRACK_KEEP)
        track_runtime_bind(&game->trackRuntime, &game->trackDef);

    game->dev.cameraZoomOverride = config->cameraZoomOverride;

    /* Freeze this run's rules and start the session on them. The lap target is the only rule a
     * GameRunConfig carries today; mode and countdown take their defaults, which is what keeps
     * a configured validation run behaving exactly as it did before sessions existed. */
    RaceRules rules;
    race_rules_set_default(&rules);
    rules.targetLaps = (config->targetLaps > 0) ? config->targetLaps : RESULTS_TARGET_LAPS;
    /* GAME_TRACK_KEEP means "leave the loaded track alone", so it must leave the session's
     * idea of which track is loaded alone too — writing the sentinel would report and checksum
     * a kept chicane as track 0. */
    if (config->track != GAME_TRACK_KEEP) game->session.trackId = (int)config->track;
    race_session_start(&game->session, &rules);

    if (config->track != GAME_TRACK_KEEP) return game_spawn_on_track(game);
    return true;
}

GAME_API bool game_spawn_on_track(Game *game)
{
    return game_spawn_on_track_at(game, 0);
}

GAME_API bool game_spawn_on_track_at(Game *game, int checkpointIndex)
{
    if (game == NULL) return false;

    Vector2 startM = { 0.0f, 0.0f };
    float headingRad = 0.0f;
    if (!track_start_pose_at(&game->trackDef, checkpointIndex, &startM, &headingRad))
        return false;

    /* Reset first, then place: vehicle_instance_reset() puts the car at the world origin, so
     * doing it the other way round would throw the pose away. */
    vehicle_instance_reset(&game->vehicleInstance);
    controller_reset(&game->controller);
    game->vehicle.positionM = startM;
    game->vehicle.headingRad = headingRad;

    /* Render history must agree with the new pose or the first frame interpolates from the
     * origin, and the first checkpoint test sweeps a segment spanning the whole track. */
    game->renderState.prevPositionM = startM;
    game->renderState.currPositionM = startM;
    game->renderState.prevHeadingRad = headingRad;
    game->renderState.currHeadingRad = headingRad;

    track_reset_progress_at(&game->progress, &game->trackDef, checkpointIndex);
    memset(&game->lastCheckpointEvent, 0, sizeof(game->lastCheckpointEvent));
    game->lastCheckpointEvent.index = -1;

    game->autoTrans.driveState = AUTO_DRIVE;
    game->autoTrans.neutralTimer = 0.0f;
    return true;
}

/*
 * Put the cars back on the grid and start the race again from tick zero.
 *
 * `game_reset_sim()` is the vehicle half of this; the session half is what makes a restart a
 * restart rather than a teleport — clock, countdown, classification, events and results all go
 * back to their starting values, so nothing from the previous run can be read afterwards.
 */
static void restart_session(Game *game)
{
    game_reset_sim(game);
    /* Route progress goes back with the cars. Without it a restart taken on or after the final
     * lap re-satisfies the finish condition on its very next tick and drops straight back to
     * the results screen, which is the opposite of what the player asked for. It is reset here
     * rather than inside game_reset_sim() because that entry point is the vehicle half on its
     * own, and physics scenarios call it mid-run without wanting their lap cursor moved.
     *
     * Deliberately NOT shared with the playback rewind below: a recording does not necessarily
     * begin at gate zero, so a replay that forced one would measure the timeline against the
     * wrong gate. Restoring a recording's starting progress needs it captured with the
     * recording, which is the replay snapshot work in issue 44. */
    track_reset_progress_at(&game->progress, &game->trackDef, 0);
    race_session_start(&game->session, NULL);
}

/*
 * Application/session commands from `input`, gear requests from `output`.
 *
 * The split is the point: pause, reset, the debug overlay and the automatic-transmission
 * toggle belong to the session and are consumed exactly once per tick no matter how many
 * entrants exist, whereas a gear shift is something one driver asked its own car for and
 * arrives on that entrant's controller output.
 *
 * The two axes move together here and only here. `game->state` picks the screen, which is what
 * the HUD switches on; the RaceSession call beside it moves the race. Pausing the screen
 * without pausing the race would leave the race clock running behind a paused overlay.
 */
static void apply_oneshots(Game *game, const Input *input, const ControllerOutput *output)
{
    if (input->pausePressed) {
        switch (game->state) {
            case STATE_MENU:
                restart_session(game);
                game->state = STATE_PLAYING;
                break;
            case STATE_PLAYING:
                race_session_pause(&game->session);
                game->state = STATE_PAUSED;
                break;
            case STATE_PAUSED:
                race_session_resume(&game->session);
                game->state = STATE_PLAYING;
                break;
            case STATE_RESULTS:
                restart_session(game);
                game->state = STATE_PLAYING;
                break;
            default: break;
        }
        game->sim.pauseToggleCount++;
    }
    if (input->resetPressed) {
        switch (game->state) {
            case STATE_PLAYING: restart_session(game); break;
            case STATE_PAUSED:
                restart_session(game);
                game->state = STATE_PLAYING;
                break;
            case STATE_RESULTS:
                /* Leaving the results screen returns to the menu without restarting anything;
                 * the next thing the player does decides what runs. A classified session is
                 * left classified — race_session_abort() refuses it — so its results survive
                 * for the results screen to keep showing. A race abandoned before it finished
                 * is a different matter and is abandoned properly. */
                race_session_abort(&game->session);
                game->state = STATE_MENU;
                break;
            default: break;
        }
        game->sim.resetCount++;
    }
    if (input->debugPressed) {
        game->debugOverlay = !game->debugOverlay;
        game->sim.debugToggleCount++;
    }
    if (input->toggleAutoPressed) {
        game->autoTrans.enabled = !game->autoTrans.enabled;
        if (game->autoTrans.enabled) {
            game->autoTrans.driveState = AUTO_DRIVE;
            game->autoTrans.neutralTimer = 0.0f;
            game->vehicle.selectedGear = 1;
        }
    }
    if (!game->autoTrans.enabled) {
        if (output->shiftUp) {
            if (game->vehicle.selectedGear < game->spec.gearCount) {
                game->vehicle.selectedGear++;
            }
            game->sim.shiftUpCount++;
        }
        if (output->shiftDown) {
            if (game->vehicle.selectedGear > -1) game->vehicle.selectedGear--;
            game->sim.shiftDownCount++;
        }
    }
}

GAME_API void game_init(Game *game)
{
    if (game == NULL) return;
    input_zero(&game->input);
    memset(&game->sim, 0, sizeof(game->sim));

    /* One entrant, spawned through the same roster path a full grid uses: the default car,
     * driven by the human at the keyboard, in grid slot 0. Everything downstream reads it
     * either through race_roster_local() or through the compatibility view in game.h. */
    race_session_init(&game->session);
    const RaceEntrantSpawn playerSpawn = { .controllerKind = CONTROLLER_KIND_HUMAN,
                                           .localPlayer = true,
                                           .gridSlot = 0 };
    (void)race_roster_spawn(&game->session.roster, &playerSpawn, NULL);
    /* Released straight to green with the default rules. A caller that wants a countdown, a
     * different distance or a multi-entrant race calls game_configure_run() (or drives the
     * session directly), and the default keeps every existing caller on the timeline it had. */
    race_session_start(&game->session, NULL);
#if defined(CIRCUIT_HEADLESS)
    game->state = STATE_PLAYING; /* headless: no menus, simulate immediately */
#else
    game->state = STATE_MENU;
#endif
    game->accumulatorS = 0.0f;
    game->lastSubstepCount = 0;
    game->physicsBacklogDrops = 0;
    game->debugOverlay = false;
    game->reloadCount = 0;
    game->reloadFlashTimerS = 0.0f;
    game->crashLockoutTimerS = 0.0f;
    particle_pool_init(&game->particles);
    game->renderPixelsPerMeter = PIXELS_PER_METER;
    game->camera = (Camera2D){ .offset = { SCREEN_W * 0.5f, SCREEN_H * 0.5f },
                               .target = { 0.0f, 0.0f },
                               .rotation = 0.0f,
                               .zoom = CAMERA_BASE_ZOOM };
    replay_begin_recording(&game->replay, 0);
    dev_state_init(&game->dev);
#if !defined(CIRCUIT_HEADLESS)
    /* Load the community gamepad mapping database so non-XInput controllers
     * (DualSense, DualShock 4, Switch Pro, etc.) are recognized by raylib on
     * Windows. The file is optional — if missing, built-in GLFW mappings
     * (which cover XInput/Xbox controllers) still work. */
    char *gamepadMappings = LoadFileText("data/input/gamecontrollerdb.txt");
    if (gamepadMappings != NULL) {
        SetGamepadMappings(gamepadMappings);
        UnloadFileText(gamepadMappings);
    }
    track_init(&game->trackDef);
    track_runtime_bind(&game->trackRuntime, &game->trackDef);
    audio_init();
#endif
    game->stateChecksum = game_state_checksum(game);
    game->initialized = true;
#if !defined(CIRCUIT_HEADLESS)
    TRACELOG(LOG_INFO, "GAME: initialised (particles, camera, state machine)");
#endif
}

GAME_API void game_pre_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_RECORDING) replay_stop(&game->replay);
#if !defined(CIRCUIT_HEADLESS)
    audio_pre_reload();
    /* Same contract as the sounds above: the baked vehicle textures are raylib-tracked GPU
     * resources held in the render module's statics, and a handle from the outgoing module
     * is a dangling GPU name in the incoming one. */
    render_pre_reload();
    TRACELOG(LOG_INFO, "GAME: pre-reload (tick %llu)", (unsigned long long)game->sim.tick);
#endif
}

GAME_API void game_post_reload(Game *game)
{
    if (game == NULL) return;
    if (game->replay.mode == REPLAY_MODE_IDLE) game->replay.mode = REPLAY_MODE_RECORDING;

    /* raygui's style lives in the module's static data, which the swap threw away. */
    dev_lab_reload_style();

    game->reloadCount++;
    game->reloadFlashTimerS = RELOAD_FLASH_S;
#if !defined(CIRCUIT_HEADLESS)
    audio_post_reload();
    render_post_reload();
    TRACELOG(LOG_INFO, "GAME: post-reload #%d (tick %llu, checksum %08x)", game->reloadCount,
             (unsigned long long)game->sim.tick, game->stateChecksum);
#endif
}

/* ------------------------------------------------------------------ fixed-update stages --
 *
 * One tick is a fixed sequence of named stages. Each one below documents what it reads and
 * what it may write, and the order they run in is the contract in docs/SIMULATION_OWNERSHIP.md.
 * Splitting them up is not decoration: it is what lets a reader answer "who wrote this value,
 * and when" without reading a hundred-and-fifty-line function, and it is where per-entrant
 * iteration goes when collision and localization arrive.
 *
 * No stage reads a frame rate, a wall clock, or a render quantity. Everything that varies with
 * time takes the `dt` it is given.
 */

/*
 * What one tick carries between its stages. It lives on the stack and dies with the tick — a
 * stage that needs to remember something across ticks must write it to an owner (the entrant,
 * the session, the track runtime), not here. That restriction is the reason this struct is
 * safe to keep out of the checksum.
 */
typedef struct {
    Input sample;              /* the input source this tick resolved to */
    ControllerKind sourceKind; /* who authored `sample`: human, script, or a recording */
    bool fromPlayback;
    bool fromScript;
    ControllerOutput applied; /* controller output after pre-physics gating */
    Vector2 startPosM;        /* authoritative pose at the top of the physics stage */
    bool trackLoaded;         /* a track with geometry is bound, so track queries are legal */
} TickContext;

/*
 * Stage 1 — acquire tick inputs.
 * Reads: replay buffer, latched Game.input, dev scenario. Writes: ctx.sample/sourceKind, and
 * the replay cursor. Restores the recorded initial vehicle on the first playback tick.
 */
static void stage_acquire_inputs(Game *game, TickContext *ctx)
{
    input_zero(&ctx->sample);
    ctx->fromPlayback = false;
    if (game->replay.mode == REPLAY_MODE_PLAYBACK) {
        if (game->replay.playbackCursor == 0) {
            /* Rewind the session alongside the vehicle. The phase is sticky in a way the
             * screen state never was: leaving it at whatever the live run reached means
             * replaying into a race that is already classified, and a classified race does not
             * simulate — so the playback would silently do nothing at all.
             *
             * Route progress is deliberately left alone. It is not ours to guess: a recording
             * need not have started at gate zero, and restoring the real starting cursor means
             * capturing it with the recording, alongside the vehicle snapshot (issue 44). */
            race_session_start(&game->session, NULL);
            if (game->replay.initialVehicle.valid &&
                !replay_restore_initial_vehicle(&game->replay, &game->vehicleDefinition,
                                                &game->vehicleSetup, &game->vehicleInstance)) {
                replay_stop(&game->replay);
            }
        }
        if (replay_next(&game->replay, &ctx->sample))
            ctx->fromPlayback = true;
        else
            replay_stop(&game->replay);
    }
    if (!ctx->fromPlayback) ctx->sample = game->input;

    /* A running scripted scenario replaces live input, but only when we are not already
     * replaying a timeline: playback must never be second-guessed. The substituted input is
     * recorded like any other, so the scenario itself is replayable. */
    ctx->fromScript = (!ctx->fromPlayback && game->dev.scenarioRunning);
    if (ctx->fromScript) {
        dev_scenario_input(game->dev.scenario, game->sim.tick - game->dev.scenarioStartTick,
                           &ctx->sample);
    }

    /* Playback overrides the entrant's own kind: a recorded stream is authoritative, so an AI
     * entrant replays as recorded instead of re-deciding. */
    ctx->sourceKind = ctx->fromPlayback ? CONTROLLER_KIND_REPLAY
                      : ctx->fromScript ? CONTROLLER_KIND_SCRIPT
                                        : game->controller.kind;
}

/*
 * Stage 2 — controller decisions.
 * Reads: tick-start world and vehicle state, ctx.sample. Writes: the entrant's private
 * controller memory and its ControllerOutput, and nothing else.
 *
 * It runs BEFORE the command stage so that a controller observes the state at the start of the
 * tick and not a state a pause or reset has already changed; the validation AI used to be
 * called by the platform loop immediately before this function, which is the same instant.
 */
static void stage_controllers(Game *game, const TickContext *ctx, float dt)
{
    const ControllerTickView view = { .sample = &ctx->sample,
                                      .track = &game->trackDef,
                                      .runtime = &game->trackRuntime,
                                      .vehicle = &game->vehicle,
                                      .derived = &game->derived,
                                      .spec = &game->spec,
                                      .dt = dt };
    controller_update(&game->controller, ctx->sourceKind, &view, &game->controllerOutput);
}

/*
 * Stage 3 — record the timeline and consume application commands.
 * Reads: ctx.sample, the entrant's ControllerOutput. Writes: the replay buffer, Game.input's
 * one-shot latches, the screen state, the session phase, and the entrant's gearbox.
 *
 * Recording the OUTPUT rather than the raw sample is what makes an AI or scripted run
 * replayable through the same path a human run takes.
 */
static void stage_record_and_commands(Game *game, const TickContext *ctx)
{
    if (game->replay.mode == REPLAY_MODE_RECORDING) {
        replay_capture_initial_vehicle(&game->replay, &game->vehicleDefinition,
                                       &game->vehicleSetup, &game->vehicleInstance);
    }
    input_clear_oneshots(&game->input);

    Input recordedFrame = ctx->sample;
    recordedFrame.steer = game->controllerOutput.steer;
    recordedFrame.throttle = game->controllerOutput.throttle;
    recordedFrame.brake = game->controllerOutput.brake;
    recordedFrame.handbrake = game->controllerOutput.handbrake;
    recordedFrame.shiftUpPressed = game->controllerOutput.shiftUp;
    recordedFrame.shiftDownPressed = game->controllerOutput.shiftDown;
    replay_record(&game->replay, &recordedFrame);
    apply_oneshots(game, &ctx->sample, &game->controllerOutput);
}

/*
 * Stage 4 — pre-physics gating.
 * Reads: the session phase, the entrant's vehicle/spec/derived. Writes: ctx.applied and the
 * entrant's automatic-transmission and committed control state.
 *
 * The controller's own output is never rewritten here; the gated copy is what physics
 * consumes. A countdown gates by zeroing the copy, which is why a held grid needs no separate
 * code path through the solver.
 */
static void stage_pre_physics(Game *game, TickContext *ctx, float dt)
{
    if (game->session.phase == RACE_PHASE_COUNTDOWN) {
        controller_output_zero(&ctx->applied);
    }
    auto_transmission_update(&game->autoTrans, &game->vehicle, &game->spec, &game->derived,
                             &ctx->applied, dt);
    game->vehicleControls.steer = ctx->applied.steer;
    game->vehicleControls.throttle = ctx->applied.throttle;
    game->vehicleControls.brake = ctx->applied.brake;
    game->vehicleControls.handbrake = ctx->applied.handbrake;
}

/*
 * Stage 5 — vehicle physics.
 * Reads: the frozen spec and ctx.applied, plus the track for per-wheel surfaces. Writes: the
 * entrant's vehicle state, derived diagnostics, and authoritative poses. Resolves no contact.
 */
static void stage_physics(Game *game, TickContext *ctx, float dt)
{
    CIRCUIT_ZONE_BEGIN(physics, "Physics");
    /* Start-of-tick position, for the checkpoint crossing test in the progress stage.
     *
     * It has to be currPositionM read HERE, before the physics step. physics_fixed_update
     * shifts curr into prev on entry and writes the new position into curr on exit, so
     * reading prevPositionM afterwards yields the position two ticks ago — and a two-tick
     * sweep overlaps the next tick's sweep, which makes every gate crossing get detected
     * twice. That was invisible while only the expected gate was tested (the second detection
     * simply failed to advance); testing all gates reports it as an out-of-order crossing,
     * which is exactly the false accusation a lap validator must not make. */
    ctx->startPosM = game->renderState.currPositionM;

    /* Per-wheel surface query: each contact point tests the track independently so the car can
     * straddle two surfaces. This lives in game.c, not physics.c, keeping the physics TU free
     * of track knowledge.
     *
     * Guarded: skip when no track is loaded so tests and scenarios that explicitly set
     * per-wheel surfaceId are not overwritten. */
    if (ctx->trackLoaded) {
        for (int i = 0; i < WHEEL_COUNT; i++) {
            const Vector2 worldContact =
                physics_wheel_world_position(&game->vehicle, (WheelId)i);
            game->vehicle.wheels[i].surfaceId =
                Track_SurfaceAt(&game->trackDef, &game->trackRuntime, worldContact);
        }
    }
    physics_fixed_update(&game->spec, &game->vehicle, &game->derived, &game->renderState,
                         &ctx->applied, dt);
    CIRCUIT_ZONE_END(physics);
}

/*
 * Stage 6 — track localization and progress.
 * Reads: the immutable track and the entrant's swept pose. Writes: that entrant's
 * RacerProgress and the tick's checkpoint event reports. The definition is never written, so
 * two entrants may run this against the same track without interacting.
 *
 * ORDER NOTE. This runs before the collision stage, which is where it has always run.
 * docs/SIMULATION_OWNERSHIP.md's target order puts progress after collision, so that a gate is
 * judged on the pose a car actually ended the tick at. That is a behaviour change rather than
 * a refactor: a barrier strike on the same tick as a gate crossing would be judged
 * differently.
 *
 * The swap was measured on this branch — all 46 telemetry scenarios and the full suite are
 * unchanged by it, because no current scenario strikes a barrier across a gate on one tick. So
 * it is safe to make, and it is deliberately not made here: the contract for the session
 * extraction is that every recorded trace survives it, and a semantic change that no test can
 * catch is one that should land beside the collision work it exists for (#26/#27), where a
 * multi-car contact makes it observable and testable.
 */
static void stage_progress(Game *game, const TickContext *ctx, float dt)
{
    if (ctx->trackLoaded) {
        TrackCheckpointEvent ev = track_update_checkpoints(
            &game->trackDef, &game->progress, ctx->startPosM, game->renderState.currPositionM);
        game->lastCheckpointEvent = ev;
        if (ev.crossed) {
            game->pendingTelemetryCheckpointEvent = ev;
            if (ev.lapCompleted && game->session.roster.count > 0) {
                race_session_log_event(&game->session, RACE_EVENT_LAP_COMPLETED,
                                       game->session.roster.entrants[0].id,
                                       (int32_t)game->progress.lap);
            }
        }
        game->progress.lapTimerS += dt;
    } else {
        memset(&game->lastCheckpointEvent, 0, sizeof(game->lastCheckpointEvent));
        game->lastCheckpointEvent.index = -1;
        memset(&game->pendingTelemetryCheckpointEvent, 0,
               sizeof(game->pendingTelemetryCheckpointEvent));
        game->pendingTelemetryCheckpointEvent.index = -1;
    }
}

/*
 * Stage 7 — collision.
 * Reads: the immutable track and the entrant's post-physics pose. Writes: the entrant's pose
 * and its crash lockout. Vehicle-to-vehicle contact is not resolved here yet (#26/#27).
 */
static void stage_collision(Game *game, const TickContext *ctx, float dt)
{
    /* Crash lockout timer: count down toward zero. It gates later contacts, so it is entrant
     * state and is counted before this tick's contact is resolved. */
    if (game->crashLockoutTimerS > 0.0f) {
        game->crashLockoutTimerS -= dt;
        if (game->crashLockoutTimerS < 0.0f) game->crashLockoutTimerS = 0.0f;
    }

    if (ctx->trackLoaded) {
        const float oldLockout = game->crashLockoutTimerS;
        collision_resolve_track(&game->spec, &game->vehicle, &game->renderState,
                                &game->trackDef, &game->trackRuntime,
                                &game->crashLockoutTimerS);
        if (game->crashLockoutTimerS > oldLockout) {
            audio_play_collision_thud();
        }
    }
}

/*
 * Stage 8 — rules and classification.
 * Reads: every entrant's RacerProgress and the frozen rules. Writes: the session clock, phase,
 * countdown, per-entrant results, the event log, and the screen the classified race hands to
 * the player.
 */
static void stage_rules(Game *game)
{
    race_session_update_rules(&game->session);
    if (game->session.phase == RACE_PHASE_CLASSIFIED) {
        game->state = STATE_RESULTS;
    }
}

/*
 * Stage 9 — presentation.
 * Reads: the local entrant and the tick's applied controls. Writes: audio, particles, and
 * development history — nothing a later tick can read back into the simulation, which is why
 * this runs after the checksum has already been taken.
 *
 * Audio and tire smoke describe what the player is driving, so they read the entrant the
 * roster designates as local. A session with no local entrant — a headless AI-only field —
 * produces neither, which is the partition that keeps presentation from depending on
 * simulation storage order.
 */
static void stage_presentation(Game *game, float dt)
{
    const RaceEntrant *localEntrant = race_roster_local_const(&game->session.roster);
    if (localEntrant != NULL) {
        const VehicleInstance *localCar = &localEntrant->instance;
        audio_update(localCar->vehicle.engineRpm, localCar->spec.engineIdleRpm,
                     localCar->spec.engineRedlineRpm, localCar->derived.physicallySliding,
                     localCar->derived.speedMps, dt);
    }

    /* Tire smoke from the rear wheels while physically sliding. Two spawns per rear wheel per
     * tick at 120 Hz is about 480 / s while sliding, which the 512-slot pool sustains over the
     * 0.8 s particle life. */
    if (localEntrant != NULL && localEntrant->instance.derived.physicallySliding &&
        localEntrant->instance.derived.speedMps > 5.0f) {
        const float heading = localEntrant->instance.vehicle.headingRad;
        const float speedMps = fminf(localEntrant->instance.derived.speedMps, 50.0f);

        /* Base velocity: rearward, opposite to the car's heading. */
        const float baseBackX = -cosf(heading) * speedMps * 0.25f;
        const float baseBackY = -sinf(heading) * speedMps * 0.25f;

        for (int w = 0; w < 2; w++) {
            const WheelId wheelId = (w == 0) ? WHEEL_REAR_LEFT : WHEEL_REAR_RIGHT;
            const Vector2 wheelWorldM =
                physics_wheel_world_position(&localEntrant->instance.vehicle, wheelId);

            for (int s = 0; s < 2; s++) {
                /* Deterministic spread that varies per-tick for visual variety. */
                const float hashX = (float)((int)(game->sim.tick * 7 + s * 13 + w * 31) & 0xFF);
                const float hashY =
                    (float)((int)(game->sim.tick * 11 + s * 17 + w * 23) & 0xFF);
                const float spreadX = (hashX / 255.0f - 0.5f) * 1.2f;
                const float spreadY = (hashY / 255.0f - 0.5f) * 1.2f + 0.6f;
                const Vector2 vel = { baseBackX + spreadX, baseBackY + spreadY };

                particle_spawn(&game->particles, wheelWorldM, vel, 0.30f,
                               (Color){ 200, 200, 200, 180 });
            }
        }
    }
}

GAME_API void game_fixed_update(Game *game, float dt)
{
    if (game == NULL) return;
    CIRCUIT_ZONE_BEGIN(fixedUpdate, "FixedUpdate");

    TickContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.trackLoaded = (game->trackDef.nodes != NULL && game->trackDef.count > 0);

    stage_acquire_inputs(game, &ctx);
    stage_controllers(game, &ctx, dt);
    stage_record_and_commands(game, &ctx);

    /* The gated copy: pre-physics assists rewrite this, never the controller's own output. */
    ctx.applied = game->controllerOutput;

    /* Particles age on every tick, whatever the session is doing, so a paused or classified
     * race still lets the smoke it already made fade out. Spawning happens in the presentation
     * stage, which only runs when the session simulated. */
    particle_pool_update(&game->particles, dt);

    /* The simulation gate reads both axes: the screen has to be the playing screen, and the
     * race has to be in a phase that runs. Either one alone would be a lie — a paused overlay
     * over a running clock, or a race stepping forward behind the menu. */
    if (game->state == STATE_PLAYING && race_session_is_simulating(&game->session)) {
        /* Open the tick before anything can raise an event, so a lap crossing logged in the
         * progress stage and the finish it triggers in the rules stage agree about when they
         * happened. */
        race_session_begin_tick(&game->session, dt);

        stage_pre_physics(game, &ctx, dt);
        stage_physics(game, &ctx, dt);
        stage_progress(game, &ctx, dt);
        stage_collision(game, &ctx, dt);
        stage_rules(game);
        stage_presentation(game, dt);
    }

    /* Stage 10 — finalize. The application tick and the rolling checksum advance once per tick
     * whether or not the race ran, because a paused session is still a tick of the application
     * that a replay has to reproduce.
     *
     * The presentation stage deliberately runs BEFORE this rather than after, as the ordering
     * contract would otherwise have it: the deterministic tire-smoke spread is seeded from
     * `sim.tick`, so advancing the counter first would move every particle ever spawned for no
     * gain. Presentation still cannot reach the simulation — it writes audio and the particle
     * pool, neither of which any stage reads. */
    game->sim.tick++;
    game->stateChecksum = game_state_checksum(game);

    /* Development history: scope channels, trajectory, and the invariant monitor. It records
     * the finalized checksum, which is why it is the one consumer that runs after it. Reads
     * the state, writes only to game->dev, and is excluded from the checksum, so it cannot
     * influence the simulation. */
    dev_state_record(game, &ctx.applied);
    CIRCUIT_ZONE_END(fixedUpdate);
}
#if defined(CIRCUIT_HEADLESS)
GAME_API void game_draw(Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}

GAME_API void game_shutdown(Game *game)
{
    if (game != NULL) {
        track_free(&game->trackDef);
    }
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
    track_free(&game->trackDef);
    audio_shutdown();
    render_shutdown();
    TRACELOG(LOG_INFO, "GAME: shutdown after %llu fixed ticks (checksum %08x)",
             (unsigned long long)game->sim.tick, game->stateChecksum);

    /* Every instrumented zone lives in this module, so this is where the table is. Compiles
     * to nothing unless the build asked for a profiling backend. */
    CIRCUIT_PROFILE_REPORT(stdout);
}
#endif

/* Encode the most recent checkpoint event as the telemetry column's closed set:
 * 0 none, 1 in-order crossing, 2 out-of-order crossing, 3 lap-completing crossing. */
static int encode_checkpoint_event(const TrackCheckpointEvent *ev)
{
    if (!ev->crossed) return 0;
    if (ev->lapCompleted) return 3;
    if (ev->outOfOrder) return 2;
    return 1;
}

GAME_API TelemetryRow game_telemetry_row(const Game *game, int substepCount)
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
    row.frontSlipRatio = 0.5f * (game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio +
                                 game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio);
    row.rearSlipRatio = 0.5f * (game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio +
                                game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio);
    row.frontWheelOmegaRadS =
        0.5f * (game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS +
                game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS);
    row.rearWheelOmegaRadS =
        0.5f * (game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS +
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
    row.frontFrictionUsage = fmaxf(game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                                   game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage);
    row.rearFrictionUsage = fmaxf(game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                                  game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage);
    row.frontLocked = game->vehicle.wheels[WHEEL_FRONT_LEFT].locked ||
                      game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked;
    row.rearLocked = game->vehicle.wheels[WHEEL_REAR_LEFT].locked ||
                     game->vehicle.wheels[WHEEL_REAR_RIGHT].locked;
    row.driveTorqueNm = game->derived.driveTorqueNm[WHEEL_FRONT_LEFT] +
                        game->derived.driveTorqueNm[WHEEL_FRONT_RIGHT] +
                        game->derived.driveTorqueNm[WHEEL_REAR_LEFT] +
                        game->derived.driveTorqueNm[WHEEL_REAR_RIGHT];
    row.frontBrakeTorqueNm = game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                             game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT];
    row.rearBrakeTorqueNm = game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                            game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT];
    row.handbrakeTorqueNm = game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
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

    row.steeringInput = game->dev.appliedInput.steer;
    row.throttleInput = game->dev.appliedInput.throttle;
    row.brakeInput = game->dev.appliedInput.brake;
    row.handbrakeInput = game->dev.appliedInput.handbrake;
    row.surfaceFrontLeft = game->vehicle.wheels[WHEEL_FRONT_LEFT].surfaceId;
    row.surfaceFrontRight = game->vehicle.wheels[WHEEL_FRONT_RIGHT].surfaceId;
    row.surfaceRearLeft = game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId;
    row.surfaceRearRight = game->vehicle.wheels[WHEEL_REAR_RIGHT].surfaceId;

    /* Phase 5 lap-validation columns. */
    row.checkpointIndex = game->progress.nextCheckpoint;
    row.lapIndex = game->progress.lap;
    row.lapState = (game->progress.lap < 1) ? 0 : 1;
    row.checkpointEvent = encode_checkpoint_event(&game->pendingTelemetryCheckpointEvent);
    /* Consume the 60 Hz telemetry event latch so it is sampled exactly once into CSV/metrics. */
    ((Game *)game)->pendingTelemetryCheckpointEvent.crossed = false;
    row.crashLockoutS = game->crashLockoutTimerS;
    row.distanceToCenterlineM =
        track_distance_to_centerline_m(&game->trackDef, game->vehicle.positionM, NULL);
    row.onTrack = (game->vehicle.wheels[WHEEL_FRONT_LEFT].surfaceId == SURFACE_ASPHALT &&
                   game->vehicle.wheels[WHEEL_FRONT_RIGHT].surfaceId == SURFACE_ASPHALT &&
                   game->vehicle.wheels[WHEEL_REAR_LEFT].surfaceId == SURFACE_ASPHALT &&
                   game->vehicle.wheels[WHEEL_REAR_RIGHT].surfaceId == SURFACE_ASPHALT)
                      ? 1
                      : 0;

    /* Per-wheel slip, straight off the wheel states the tire model was evaluated at. */
    row.slipAngleFrontLeftRad = game->vehicle.wheels[WHEEL_FRONT_LEFT].slipAngleRad;
    row.slipAngleFrontRightRad = game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipAngleRad;
    row.slipAngleRearLeftRad = game->vehicle.wheels[WHEEL_REAR_LEFT].slipAngleRad;
    row.slipAngleRearRightRad = game->vehicle.wheels[WHEEL_REAR_RIGHT].slipAngleRad;
    row.slipRatioFrontLeft = game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio;
    row.slipRatioFrontRight = game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio;
    row.slipRatioRearLeft = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    row.slipRatioRearRight = game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio;
    return row;
}
