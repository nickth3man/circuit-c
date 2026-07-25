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
 * run that exercises the Phase 1 vehicle and debug HUD, writes telemetry/phase1_smoke.png,
 * and exits without leaving a background process.
 */
#ifndef _WIN32
#error Drifty currently supports Windows only.
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "raylib.h"

#include "config.h"
#include "game.h"
#include "hotreload.h"
#include "input.h"
#include "timestep.h"

/* Bounded smoke-test duration: enough frames for the fixed-step loop and HUD to settle. */
#define SMOKE_TEST_FRAMES 120

/* Transient adapter so the accumulator does not need to know about Game. It is a stack
 * value passed as an argument and is never stored in persistent state, so the
 * "no function pointers in Game" rule is untouched. */
static void platform_fixed_update(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

static int parse_smoke_test(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--smoke-test") == 0) return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    const int smoke_test = parse_smoke_test(argc, argv);

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

    InitWindow(SCREEN_W, SCREEN_H, smoke_test ? "Drifty - Phase 1 smoke test" : "Drifty - Phase 1");
    if (!IsWindowReady()) {
        fprintf(stderr, "FATAL: InitWindow failed (%dx%d)\n", SCREEN_W, SCREEN_H);
        Game_UnloadModule();
        free(game);
        return 1;
    }

    SetTargetFPS(TARGET_FPS);

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

        const TimestepResult step = timestep_advance(&game->accumulatorS,
                                                     &game->physicsBacklogDrops,
                                                     GetFrameTime(),
                                                     platform_fixed_update,
                                                     game);
        game->lastSubstepCount = step.substeps;

        game_draw(game, step.interpolationAlpha);

        if (smoke_test) {
            smoke_frames++;
            if (smoke_frames >= SMOKE_TEST_FRAMES) {
                TakeScreenshot("telemetry/phase1_smoke.png");
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
        printf("SMOKE: ok (%d frames) -> telemetry/phase1_smoke.png\n", smoke_frames);
    }
    return exit_status;
}
