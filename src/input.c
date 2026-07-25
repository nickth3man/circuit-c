#include "input.h"

#include <string.h>

#include "math_utils.h"

void input_zero(Input *in)
{
    if (in == NULL) return;
    memset(in, 0, sizeof(*in));
}

void input_clear_oneshots(Input *in)
{
    if (in == NULL) return;
    in->pausePressed     = false;
    in->resetPressed     = false;
    in->debugPressed     = false;
    in->shiftUpPressed   = false;
    in->shiftDownPressed = false;
}

bool input_has_oneshot(const Input *in)
{
    if (in == NULL) return false;
    return in->pausePressed || in->resetPressed || in->debugPressed ||
           in->shiftUpPressed || in->shiftDownPressed;
}

#if !defined(DRIFTY_HEADLESS)

#include "raylib.h"

void input_sample(Input *in)
{
    if (in == NULL) return;

    /* Held controls: overwritten every render frame. Left is positive. */
    float steer = 0.0f;
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  steer += 1.0f;
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) steer -= 1.0f;

    in->steer     = clampf(steer, -1.0f, 1.0f);
    in->throttle  = (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))   ? 1.0f : 0.0f;
    in->brake     = (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) ? 1.0f : 0.0f;
    in->handbrake = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;

    /* One-shot commands: latched, never overwritten, so a press cannot be lost in a
     * render frame that happens to run no fixed substeps. */
    if (IsKeyPressed(KEY_P))  in->pausePressed     = true;
    if (IsKeyPressed(KEY_R))  in->resetPressed     = true;
    if (IsKeyPressed(KEY_F1)) in->debugPressed     = true;
    if (IsKeyPressed(KEY_E))  in->shiftUpPressed   = true;
    if (IsKeyPressed(KEY_Q))  in->shiftDownPressed = true;
}

#endif /* !DRIFTY_HEADLESS */
