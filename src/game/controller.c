/*
 * controller.c — kind dispatch for the control boundary described in controller.h.
 *
 * The dispatch is a plain switch rather than a stored callback: Game must stay free of
 * function pointers so the block survives a hot reload (see game.h).
 */
#include "game/controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "core/math_utils.h"

/* A finite value, or 0. Guards the clamp against a NaN that would otherwise pass straight
 * through fminf/fmaxf and poison the integrator. */
static float finite_or_zero(float value)
{
    return isfinite(value) ? value : 0.0f;
}

void controller_init(Controller *controller, ControllerKind kind)
{
    if (controller == NULL) return;
    memset(controller, 0, sizeof(*controller));
    controller->kind =
        (kind >= 0 && kind < CONTROLLER_KIND_COUNT) ? kind : CONTROLLER_KIND_HUMAN;
    ai_driver_config_default(&controller->config.ai);
}

void controller_reset(Controller *controller)
{
    if (controller == NULL) return;
    memset(&controller->memory, 0, sizeof(controller->memory));
}

void controller_output_zero(ControllerOutput *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
}

void controller_output_from_input(const Input *in, ControllerOutput *out)
{
    if (out == NULL) return;
    controller_output_zero(out);
    if (in == NULL) return;
    out->steer = in->steer;
    out->throttle = in->throttle;
    out->brake = in->brake;
    out->handbrake = in->handbrake;
    out->shiftUp = in->shiftUpPressed;
    out->shiftDown = in->shiftDownPressed;
}

void controller_output_clamp(ControllerOutput *out)
{
    if (out == NULL) return;
    out->steer = clampf(finite_or_zero(out->steer), -1.0f, 1.0f);
    out->throttle = clampf(finite_or_zero(out->throttle), 0.0f, 1.0f);
    out->brake = clampf(finite_or_zero(out->brake), 0.0f, 1.0f);
    out->handbrake = clampf(finite_or_zero(out->handbrake), 0.0f, 1.0f);
}

void controller_update(Controller *controller, ControllerKind sourceKind,
                       const ControllerTickView *view, ControllerOutput *out)
{
    if (out == NULL) return;
    controller_output_zero(out);
    if (controller == NULL || view == NULL) return;

    switch (sourceKind) {
        case CONTROLLER_KIND_HUMAN:
        case CONTROLLER_KIND_SCRIPT:
        case CONTROLLER_KIND_REPLAY:
            /* Three sources, one conversion. What differs between them is where the session
             * acquired the sample, not what the car is then asked to do with it. */
            controller_output_from_input(view->sample, out);
            break;

        case CONTROLLER_KIND_AI:
            /* The driver writes four held controls and nothing else; the shift requests stay
             * false because it drives with the automatic box, exactly as before. */
            ai_driver_update(&controller->config.ai, &controller->memory.ai, view->track,
                             view->runtime, view->vehicle, view->derived, view->spec, out,
                             view->dt);
            break;

        default: break;
    }

    controller_output_clamp(out);
}
