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
    in->toggleAutoPressed = false;
}

bool input_has_oneshot(const Input *in)
{
    if (in == NULL) return false;
    return in->pausePressed || in->resetPressed || in->debugPressed ||
           in->shiftUpPressed || in->shiftDownPressed ||
           in->toggleAutoPressed;
}

#if !defined(DRIFTY_HEADLESS)

#include <math.h>

#include "raylib.h"

/* Radial deadzone for the left stick. Returns the deadzone-corrected x deflection.
 * The y is passed only to compute the radial magnitude; it is not returned because
 * Drifty only uses the X axis for steering. Values outside the deadzone are rescaled
 * from [deadzone..1] to [0..1] so full deflection still reaches 1.0. */
static float gamepad_radial_deadzone_x(float x, float y, float deadzone)
{
    const float mag = sqrtf(x * x + y * y);
    if (mag <= deadzone) return 0.0f;
    const float scale = (mag - deadzone) / (1.0f - deadzone);
    return (x / mag) * scale;
}

/* Normalize a raylib trigger axis from [-1..+1] to [0..1], with a deadzone at the
 * resting end. raylib triggers rest at -1.0 (not 0.0), so raw -1 -> 0.0 output,
 * raw +1 -> 1.0 output. The deadzone (e.g. 0.05) clamps the first 5% of travel. */
static float gamepad_normalize_trigger(float raw, float deadzone)
{
    if (raw < -1.0f + deadzone) return 0.0f;
    return (raw + 1.0f) / 2.0f;
}

void input_sample(Input *in)
{
    if (in == NULL) return;

    /* --- Keyboard sample --- */
    float kb_steer = 0.0f;
    if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  kb_steer += 1.0f;
    if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) kb_steer -= 1.0f;
    kb_steer = clampf(kb_steer, -1.0f, 1.0f);
    const float kb_throttle  = (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))   ? 1.0f : 0.0f;
    const float kb_brake     = (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) ? 1.0f : 0.0f;
    const float kb_handbrake = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;

    /* --- Gamepad sample (gamepad 0 if available) --- */
    float gp_steer = 0.0f, gp_throttle = 0.0f, gp_brake = 0.0f, gp_handbrake = 0.0f;
    if (IsGamepadAvailable(0)) {
        /* Left stick X: raylib reports -1=left, +1=right. Drifty steer is +1=left,
         * so negate. Apply a radial deadzone using both X and Y so diagonal stick
         * noise does not bleed into steering. */
        const float lx = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
        const float ly = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
        gp_steer = -gamepad_radial_deadzone_x(lx, ly, 0.10f);

        /* Triggers rest at -1.0; normalize to 0..1 with a small deadzone.
         * Right trigger = throttle, left trigger = brake. */
        const float rt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER);
        const float lt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER);
        gp_throttle = gamepad_normalize_trigger(rt, 0.05f);
        gp_brake    = gamepad_normalize_trigger(lt, 0.05f);

        /* Handbrake: right bumper (R1) or Circle. */
        gp_handbrake = (IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) ||
                        IsGamepadButtonDown(0, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT))
                       ? 1.0f : 0.0f;
    }

    /* --- Merge held controls: per-axis max magnitude wins. ---
     * This lets keyboard steering (always full ±1) override a partially-deflected
     * stick, while analog triggers blend naturally with digital keys via fmaxf. */
    in->steer     = (fabsf(kb_steer) >= fabsf(gp_steer)) ? kb_steer : gp_steer;
    in->throttle  = fmaxf(kb_throttle, gp_throttle);
    in->brake     = fmaxf(kb_brake, gp_brake);
    in->handbrake = fmaxf(kb_handbrake, gp_handbrake);

    /* --- One-shot commands: latched, OR'd from keyboard and gamepad. --- */
    if (IsKeyPressed(KEY_P))                                   in->pausePressed     = true;
    if (IsKeyPressed(KEY_R))                                   in->resetPressed     = true;
    if (IsKeyPressed(KEY_F1))                                  in->debugPressed     = true;
    if (IsKeyPressed(KEY_E))                                   in->shiftUpPressed   = true;
    if (IsKeyPressed(KEY_Q))                                   in->shiftDownPressed = true;
    if (IsKeyPressed(KEY_T))                                   in->toggleAutoPressed = true;

    if (IsGamepadAvailable(0)) {
        if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_RIGHT))      in->pausePressed      = true;  /* Options */
        if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_MIDDLE_LEFT))       in->resetPressed      = true;  /* Create/Select */
        if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_UP))     in->shiftUpPressed    = true;  /* Triangle/Y */
        if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_FACE_LEFT))   in->shiftDownPressed  = true;  /* Square/X */
        if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_THUMB))        in->toggleAutoPressed = true;  /* L3 */
    }
}

#endif /* !DRIFTY_HEADLESS */
