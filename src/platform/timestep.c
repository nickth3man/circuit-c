#include "platform/timestep.h"

#include <math.h>
#include <stddef.h>

#include "core/math_utils.h"

TimestepResult timestep_advance(float *accumulatorS,
                                int *backlogDrops,
                                float frameTimeS,
                                TimestepFixedUpdateFn fixedUpdate,
                                void *ctx)
{
    TimestepResult result;
    result.substeps           = 0;
    result.droppedBacklog     = false;
    result.interpolationAlpha = 0.0f;

    if (accumulatorS == NULL || backlogDrops == NULL) return result;

    /* Clamp an extreme or nonsensical frame time before it reaches the accumulator. */
    if (!(frameTimeS > 0.0f)) frameTimeS = 0.0f;              /* also catches NaN */
    if (frameTimeS > MAX_FRAME_TIME_S) frameTimeS = MAX_FRAME_TIME_S;

    *accumulatorS += frameTimeS;

    int steps = 0;
    while (*accumulatorS >= FIXED_DT_S && steps < MAX_PHYSICS_STEPS) {
        if (fixedUpdate != NULL) fixedUpdate(ctx, FIXED_DT_S);
        *accumulatorS -= FIXED_DT_S;
        steps++;
    }

    /* Clamping frame time alone is not a sufficient guard: 0.25 s still buys 30 steps at
     * 120 Hz. The substep cap is what bounds the loop, and whatever it leaves behind is
     * dropped rather than carried into the next frame as a growing debt. */
    if (steps == MAX_PHYSICS_STEPS && *accumulatorS >= FIXED_DT_S) {
        *accumulatorS = fmodf(*accumulatorS, FIXED_DT_S);
        (*backlogDrops)++;
        result.droppedBacklog = true;
    }

    result.substeps           = steps;
    result.interpolationAlpha = clampf(*accumulatorS / FIXED_DT_S, 0.0f, 1.0f);
    return result;
}
