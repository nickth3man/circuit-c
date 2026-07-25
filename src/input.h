/*
 * input.h — held controls and one-shot commands.
 *
 * This header is raylib-free so the headless harness can build and drive Input timelines
 * without a window. Keyboard polling lives in input.c behind !DRIFTY_HEADLESS.
 *
 * Lifetime contract (docs/SPEC.md, "One-shot input consumption"):
 *
 *   - Held controls are sampled once per render frame and stay valid for every fixed
 *     substep of that frame.
 *   - One-shot commands are edge-triggered. input_sample() LATCHES them (|=) rather than
 *     overwriting, so a press during a render frame that runs zero fixed substeps survives
 *     to the next frame. game_fixed_update() reads them and calls input_clear_oneshots()
 *     before returning, so a single-frame press is acted on exactly once even when eight
 *     substeps run in that frame.
 *
 * Steering follows the left-positive convention: left key -> +1, right key -> -1.
 */
#ifndef DRIFTY_INPUT_H
#define DRIFTY_INPUT_H

#include <stdbool.h>

typedef struct {
    /* Held controls, sampled every render frame, valid for every substep */
    float steer;        /* -1 (right) .. +1 (left), dimensionless */
    float throttle;     /* 0 .. 1, dimensionless */
    float brake;        /* 0 .. 1, dimensionless */
    float handbrake;    /* 0 .. 1, dimensionless */

    /* One-shot commands, edge-triggered, cleared after the first fixed update */
    bool pausePressed;
    bool resetPressed;
    bool debugPressed;
    bool shiftUpPressed;
    bool shiftDownPressed;
} Input;

/* Zero every field. */
void input_zero(Input *in);

/* Clear only the one-shot commands; held controls are untouched. */
void input_clear_oneshots(Input *in);

/* True while any one-shot command is still pending consumption. */
bool input_has_oneshot(const Input *in);

/* Poll the keyboard for one render frame. Held controls are overwritten; one-shot
 * commands are latched on top of whatever is still pending.
 * Defined only in builds that link raylib (not in DRIFTY_HEADLESS builds). */
void input_sample(Input *in);

#endif /* DRIFTY_INPUT_H */
