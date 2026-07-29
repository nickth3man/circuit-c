/*
 * main.c — platform layer (Windows only).
 *
 * Owns the window, the raylib context, the Game allocation, and the fixed-timestep loop.
 * It is not hot-reloadable: editing this file requires restarting drifty.exe.
 *
 * The Game block is allocated here, once, and handed to the game module by pointer on every
 * entry point. That is what lets the module be swapped while the game keeps its state.
 *
 * Normal interactive runs loop until the window closes. Pass --smoke-test for a bounded
 * run that exercises the Phase 3 vehicle and debug HUD, writes
 * artifacts/screenshots/phase3_smoke.png,
 * and exits without leaving a background process.
 *
 * Two other bounded modes exist for tooling:
 *
 *   --capture-scene NAME [--width W] [--height H] [--seed N] [--output PATH] [--ticks N]
 *       Steps the simulation with an exact fixed dt (never GetFrameTime), draws one frame,
 *       writes a PNG, and exits. Frame-rate independence is the point: the same command
 *       produces the same image, which is what makes screenshot comparison meaningful.
 *
 *   --list-scenes
 *       Prints the capture scene names and exits.
 *
 * Both terminate on their own. Nothing in this file ever waits for a developer.
 */
#ifndef _WIN32
#error Drifty currently supports Windows only.
#endif

#include <direct.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "core/config.h"
#include "dev/dev_scenario.h"
#include "game/game.h"
#include "platform/hotreload.h"
#include "game/input.h"
#include "platform/timestep.h"

/* Bounded smoke-test duration: enough frames for the fixed-step loop and HUD to settle. */
#define SMOKE_TEST_FRAMES 120

/* The smoke test's evidence image. It is run evidence, not telemetry, so it lives beside the
 * other captures under artifacts/ rather than in the CSV output directory. */
#define SMOKE_TEST_SCREENSHOT "artifacts/screenshots/phase3_smoke.png"

/* --------------------------------------------------------------------- capture scenes -- */

typedef struct {
    const char *name;
    const char *scenario;   /* dev_scenario key driving the sim before the capture */
    int         ticks;      /* fixed ticks to run before drawing */
    bool        debugOverlay;
    bool        lab;
    int         scopePreset;
    bool        showLoads;
    bool        showResistance;
} CaptureScene;

static const CaptureScene g_captureScenes[] = {
    { "debug_overlay", "accel",           240, true,  false, DEV_SCOPE_PRESET_HANDLING, false, false },
    { "tire_curves",   "skidpad",         600, true,  false, DEV_SCOPE_PRESET_GRIP,     false, false },
    { "drift_hud",     "handbrake-entry", 500, true,  false, DEV_SCOPE_PRESET_HANDLING, false, false },
    { "physics_lab",   "skidpad",         480, false, true,  DEV_SCOPE_PRESET_HANDLING, false, false },
    { "accel_load",    "accel-load",      600, false, true,  DEV_SCOPE_PRESET_LOAD,     true,  true  },
    { "brake_load",    "brake-load",      760, false, true,  DEV_SCOPE_PRESET_LOAD,     true,  true  },
    { "skidpad_p3",    "skidpad",        1500, false, true,  DEV_SCOPE_PRESET_GRIP,     true,  true  },
    { "lift_off",      "lift-off",        850, false, true,  DEV_SCOPE_PRESET_LOAD,     true,  true  },
    { "transition_p3", "transition",      900, false, true,  DEV_SCOPE_PRESET_GRIP,     true,  true  },
    { "catchable",     "catchable-drift", 820, false, true,  DEV_SCOPE_PRESET_GRIP,     true,  true  },
};

#define CAPTURE_SCENE_COUNT ((int)(sizeof(g_captureScenes) / sizeof(g_captureScenes[0])))

static const CaptureScene *find_capture_scene(const char *name)
{
    for (int i = 0; i < CAPTURE_SCENE_COUNT; i++) {
        if (strcmp(g_captureScenes[i].name, name) == 0) return &g_captureScenes[i];
    }
    return NULL;
}

/* Create every directory component of path, so --output artifacts/screenshots/x.png works on
 * a clean checkout. Failure is not fatal: raylib reports the export failure itself. */
static void ensure_parent_directory(const char *path)
{
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "%s", path);

    char *lastSeparator = NULL;
    for (char *c = buffer; *c != '\0'; c++) {
        if (*c == '/' || *c == '\\') lastSeparator = c;
    }
    if (lastSeparator == NULL) return;
    *lastSeparator = '\0';

    /* Walk the path creating one level at a time; EEXIST is the normal case. */
    for (char *c = buffer; ; c++) {
        const bool atSeparator = (*c == '/' || *c == '\\');
        if (!atSeparator && *c != '\0') continue;

        const char saved = *c;
        *c = '\0';
        if (buffer[0] != '\0' && _mkdir(buffer) != 0 && errno != EEXIST) {
            fprintf(stderr, "CAPTURE: could not create directory '%s': %s\n",
                    buffer, strerror(errno));
        }
        *c = saved;
        if (saved == '\0') break;
    }
}

/* Transient adapter so the accumulator does not need to know about Game. It is a stack
 * value passed as an argument and is never stored in persistent state, so the
 * "no function pointers in Game" rule is untouched. */
static void platform_fixed_update(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

typedef struct {
    bool        smokeTest;
    const char *captureScene;
    const char *outputPath;
    int         width;
    int         height;
    int         ticks;          /* -1: use the scene's own tick count */
    uint32_t    seed;
    int         galleryPage;    /* 0: not a gallery capture; >=1: the 1-based page */
} Options;

static void print_usage(const char *argv0)
{
    printf("usage: %s [--smoke-test]\n", argv0);
    printf("       %s --capture-scene NAME [--width W] [--height H] [--ticks N]\n", argv0);
    printf("               [--seed N] [--output PATH]\n");
    printf("       %s --list-scenes\n", argv0);
}

/* Returns false when the caller should exit immediately with *statusOut. */
static bool parse_options(int argc, char **argv, Options *options, int *statusOut)
{
    options->smokeTest = false;
    options->captureScene = NULL;
    options->outputPath = NULL;
    options->width = SCREEN_W;
    options->height = SCREEN_H;
    options->ticks = -1;
    options->seed = 0u;
    options->galleryPage = 0;
    *statusOut = 0;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const bool hasValue = (i + 1 < argc);

        if (strcmp(arg, "--smoke-test") == 0) {
            options->smokeTest = true;
        } else if (strcmp(arg, "--list-scenes") == 0) {
            for (int s = 0; s < CAPTURE_SCENE_COUNT; s++) {
                printf("%-16s scenario=%-16s ticks=%d\n", g_captureScenes[s].name,
                       g_captureScenes[s].scenario, g_captureScenes[s].ticks);
            }
            return false;
        } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
            print_usage(argv[0]);
            return false;
        } else if (strcmp(arg, "--capture-scene") == 0 && hasValue) {
            options->captureScene = argv[++i];
        } else if (strcmp(arg, "--output") == 0 && hasValue) {
            options->outputPath = argv[++i];
        } else if (strcmp(arg, "--width") == 0 && hasValue) {
            options->width = atoi(argv[++i]);
        } else if (strcmp(arg, "--height") == 0 && hasValue) {
            options->height = atoi(argv[++i]);
        } else if (strcmp(arg, "--ticks") == 0 && hasValue) {
            options->ticks = atoi(argv[++i]);
        } else if (strcmp(arg, "--seed") == 0 && hasValue) {
            options->seed = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (strcmp(arg, "--gallery-page") == 0 && hasValue) {
            options->galleryPage = atoi(argv[++i]);
        } else {
            fprintf(stderr, "error: unrecognised argument '%s'\n", arg);
            print_usage(argv[0]);
            *statusOut = 2;
            return false;
        }
    }

    if (options->width < 320 || options->height < 240) {
        fprintf(stderr, "error: --width/--height must be at least 320x240\n");
        *statusOut = 2;
        return false;
    }
    return true;
}

/*
 * Deterministic capture. The simulation is advanced with an exact fixed dt rather than
 * GetFrameTime(), so the image depends only on the scene, the tick count, and the build —
 * never on how fast the machine happened to be.
 */
static int run_capture(Game *game, const Options *options)
{
    const CaptureScene *scene = find_capture_scene(options->captureScene);
    if (scene == NULL) {
        fprintf(stderr, "error: no capture scene named '%s' (try --list-scenes)\n",
                options->captureScene);
        return 2;
    }

    const int scenarioIndex = dev_scenario_find(scene->scenario);
    const int ticks = (options->ticks >= 0) ? options->ticks : scene->ticks;
    const char *output = (options->outputPath != NULL)
                       ? options->outputPath : "artifacts/screenshots/capture.png";

    game_init(game);
    game->debugOverlay = scene->debugOverlay;
    game->dev.labVisible = scene->lab;
    game->dev.uiDeterministic = true;
    game->dev.scopePreset = scene->scopePreset;
    game->dev.showLoads = scene->showLoads;
    game->dev.showResistance = scene->showResistance;

    /* The platform layer drives the scenario by writing plain data into the Game block; the
     * module's fixed update is what actually applies the script. dev_state_scenario_start()
     * is not reachable from here — it lives inside the reloadable module — and does not need
     * to be: game_init() has already put the simulation in its reset state. */
    if (scenarioIndex > 0) {
        const DevScenario *scenario = dev_scenario_at(scenarioIndex);
        game->dev.scenario = scenarioIndex;
        game->dev.scenarioRunning = true;
        game->dev.scenarioStartTick = game->sim.tick;
        game->dev.seed = (options->seed != 0u) ? options->seed : scenario->seed;
    } else if (options->seed != 0u) {
        game->dev.seed = options->seed;
    }

    for (int i = 0; i < ticks; i++) {
        game_fixed_update(game, FIXED_DT_S);
    }

    /* Two frames: the first lets raylib settle its own per-frame state, the second is the
     * one that gets captured. Both use interpolation alpha 0 so the drawn pose is exactly
     * the last simulated one. */
    game_draw(game, 0.0f);
    game_draw(game, 0.0f);

    ensure_parent_directory(output);
    TakeScreenshot(output);

    printf("CAPTURE: scene=%s ticks=%d size=%dx%d checksum=%08x -> %s\n",
           scene->name, ticks, options->width, options->height,
           game->stateChecksum, output);
    return 0;
}

/*
 * Bounded gallery capture: no simulation, one page of the vehicle corpus drawn through the
 * production texture path, one screenshot, exit. Selected through the DevState field the
 * module already owns, so this needs no new entry point.
 */
static int run_gallery(Game *game, const Options *options)
{
    const char *output = (options->outputPath != NULL)
                       ? options->outputPath : "artifacts/gallery-ingame/page.png";

    game_init(game);
    game->dev.uiDeterministic = true;
    game->dev.galleryPage = options->galleryPage;

    /* Two frames for the same reason a scene capture takes two: the first lets raylib settle
     * its own per-frame state, the second is the one that gets written. */
    game_draw(game, 0.0f);
    game_draw(game, 0.0f);

    ensure_parent_directory(output);
    TakeScreenshot(output);

    printf("GALLERY: page=%d size=%dx%d -> %s\n",
           options->galleryPage, options->width, options->height, output);
    return 0;
}

int main(int argc, char **argv)
{
    Options options;
    int parseStatus = 0;
    if (!parse_options(argc, argv, &options, &parseStatus)) return parseStatus;

    const bool smoke_test = options.smokeTest;
    const bool capture = (options.captureScene != NULL);
    const bool gallery = (options.galleryPage > 0);

    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "FATAL: could not allocate the Game block (%zu bytes)\n", sizeof(Game));
        return 1;
    }

    if (!Game_LoadModule()) {
        fprintf(stderr, "FATAL: could not load the game module '%s'. Run build.bat first.\n",
                GAME_MODULE_NAME);
        free(game);
        return 1;
    }

    const char *title = "Drifty - Phase 2";
    if (smoke_test) title = "Drifty - Phase 2 smoke test";
    if (capture) title = "Drifty - scene capture";
    if (gallery) title = "Drifty - vehicle gallery";

    InitWindow(options.width, options.height, title);
    if (!IsWindowReady()) {
        fprintf(stderr, "FATAL: InitWindow failed (%dx%d)\n", options.width, options.height);
        Game_UnloadModule();
        free(game);
        return 1;
    }

    SetTargetFPS(TARGET_FPS);

    if (capture || gallery) {
        const int status = capture ? run_capture(game, &options)
                                   : run_gallery(game, &options);
        game_shutdown(game);
        CloseWindow();
        Game_UnloadModule();
        free(game);
        return status;
    }

    game_init(game);
    if (smoke_test) {
        /* Force the debug overlay so the screenshot captures reload status and held-input
         * lines without requiring an F1 press. */
        game->debugOverlay = true;
    }

    int smoke_frames = 0;
    int exit_status = 0;

    while (!WindowShouldClose()) {
        /* Development builds only; compiles to nothing in a release build. */
        Game_MaybeHotReload(game);

        /* Sampled once per render frame; held controls stay valid for every substep. */
        input_sample(&game->input);

        /*
         * Physics Lab time control. The lab writes these three fields; the platform loop is
         * the only thing that reads them, so pausing and stepping cannot perturb the
         * simulation itself — a stepped tick is bit-identical to a free-running one.
         *
         * While paused, the accumulator is zeroed before a step so that exactly the
         * requested number of fixed updates run, and stepping more than the substep cap
         * simply spreads across the next frames instead of being counted as backlog.
         */
        float frameTimeS = GetFrameTime() * game->dev.timeScale;
        if (game->dev.paused) {
            if (game->dev.stepTicks > 0) {
                const int stepping = (game->dev.stepTicks < MAX_PHYSICS_STEPS)
                                   ? game->dev.stepTicks : MAX_PHYSICS_STEPS;
                game->dev.stepTicks -= stepping;
                game->accumulatorS = 0.0f;
                frameTimeS = FIXED_DT_S * (float)stepping;
            } else {
                frameTimeS = 0.0f;
            }
        }

        const TimestepResult step = timestep_advance(&game->accumulatorS,
                                                     &game->physicsBacklogDrops,
                                                     frameTimeS,
                                                     platform_fixed_update,
                                                     game);
        game->lastSubstepCount = step.substeps;

        game_draw(game, step.interpolationAlpha);

        if (smoke_test) {
            smoke_frames++;
            if (smoke_frames >= SMOKE_TEST_FRAMES) {
                ensure_parent_directory(SMOKE_TEST_SCREENSHOT);
                TakeScreenshot(SMOKE_TEST_SCREENSHOT);
                TRACELOG(LOG_INFO,
                         "SMOKE: completed %d frames (substeps=%d backlog=%d alpha=%.3f checksum=%08x reloads=%d)",
                         smoke_frames, game->lastSubstepCount, game->physicsBacklogDrops,
                         (double)step.interpolationAlpha, game->stateChecksum, game->reloadCount);
                break;
            }
        }
    }

    if (smoke_test && smoke_frames < SMOKE_TEST_FRAMES) {
        fprintf(stderr, "SMOKE: window closed early after %d / %d frames\n",
                smoke_frames, SMOKE_TEST_FRAMES);
        exit_status = 1;
    }

    game_shutdown(game);

    CloseWindow();
    Game_UnloadModule();
    free(game);

    if (smoke_test && exit_status == 0) {
        printf("SMOKE: ok (%d frames) -> %s\n", smoke_frames, SMOKE_TEST_SCREENSHOT);
    }
    return exit_status;
}
