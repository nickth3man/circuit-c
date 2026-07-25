/*
 * main.c — platform layer.
 *
 * Owns the window, the raylib context, the Game allocation, and the fixed-timestep loop.
 * It is not hot-reloadable: editing this file requires restarting drifty.exe.
 *
 * The Game block is allocated here, once, and handed to the game module by pointer on every
 * entry point. That is what lets the module be swapped while the game keeps its state.
 */
#include <stdio.h>
#include <stdlib.h>

#include "raylib.h"

#include "config.h"
#include "game.h"
#include "hotreload.h"
#include "input.h"
#include "timestep.h"

/* Transient adapter so the accumulator does not need to know about Game. It is a stack
 * value passed as an argument and is never stored in persistent state, so the
 * "no function pointers in Game" rule is untouched. */
static void platform_fixed_update(void *ctx, float dt)
{
    game_fixed_update((Game *)ctx, dt);
}

int main(void)
{
    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "FATAL: could not allocate the Game block (%zu bytes)\n", sizeof(Game));
        return 1;
    }

    if (!Game_LoadModule()) {
        fprintf(stderr, "FATAL: could not load the game module '%s'. Run ./build.sh first.\n",
                GAME_MODULE_NAME);
        free(game);
        return 1;
    }

    InitWindow(SCREEN_W, SCREEN_H, "Drifty - Phase 0");
    SetTargetFPS(TARGET_FPS);

    game_init(game);

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
    }

    game_shutdown(game);

    CloseWindow();
    Game_UnloadModule();
    free(game);
    return 0;
}
