/*
 * timestep.h — the fixed-timestep accumulator, isolated so it is testable without a window.
 *
 * This is platform-layer code: main.c owns the loop and calls timestep_advance() once per
 * render frame. It is a separate translation unit from main.c only because the headless
 * harness must be able to assert the substep cap, the backlog-drop counter, and the
 * interpolation alpha directly. There is exactly one implementation; main.c and
 * drifty_tests share it, so the tested behaviour is the shipped behaviour.
 *
 * It is NOT compiled into the hot-reloadable game module: the loop lives with the platform
 * layer, as docs/SPEC.md requires.
 *
 * The fixed-update callback is passed in as an argument. It is never stored anywhere
 * persistent, so this does not violate the reload-safety rule against function pointers in
 * Game.
 */
#ifndef DRIFTY_TIMESTEP_H
#define DRIFTY_TIMESTEP_H

#include <stdbool.h>

#include "core/config.h"

/* One fixed physics step of exactly FIXED_DT_S seconds. */
typedef void (*TimestepFixedUpdateFn)(void *ctx, float dt);

typedef struct {
    int   substeps;             /* fixed updates executed this frame, 0 .. MAX_PHYSICS_STEPS */
    bool  droppedBacklog;       /* excess accumulator was discarded this frame */
    float interpolationAlpha;   /* leftover accumulator / FIXED_DT_S, in [0, 1] */
} TimestepResult;

/*
 * Advance the accumulator by one render frame.
 *
 *   - frameTimeS is clamped to MAX_FRAME_TIME_S (and to >= 0), so a debugger pause or a
 *     stalled frame cannot inject an unbounded backlog.
 *   - At most MAX_PHYSICS_STEPS fixed updates run. The loop is never unbounded.
 *   - If the cap is hit and a full step of backlog still remains, the excess is discarded
 *     (the accumulator is reduced modulo FIXED_DT_S so sub-step phase is preserved) and
 *     *backlogDrops is incremented.
 *
 * accumulatorS and backlogDrops are in/out and must be non-NULL. fixedUpdate may be NULL,
 * in which case the accumulator is still drained and counted; that is what lets a test
 * measure pacing on its own.
 */
TimestepResult timestep_advance(float *accumulatorS,
                                int *backlogDrops,
                                float frameTimeS,
                                TimestepFixedUpdateFn fixedUpdate,
                                void *ctx);

#endif /* DRIFTY_TIMESTEP_H */
