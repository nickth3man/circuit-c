/*
 * input.h — held controls and one-shot commands.
 *
 * This header is raylib-free so the headless harness can build and drive Input timelines
 * without a window. Keyboard polling lives in input.c behind !CIRCUIT_HEADLESS.
 *
 * Lifetime contract:
 *
 *   - Held controls are sampled once per render frame and stay valid for every fixed
 *     substep of that frame.
 *   - One-shot commands are edge-triggered. input_sample() LATCHES them (|=) rather than
 *     overwriting, so a press during a render frame that runs zero fixed substeps survives
 *     to the next frame. game_fixed_update() reads them and calls input_clear_oneshots()
 *     before returning, so a single-frame press is acted on exactly once even when eight
 *     substeps run in that frame.
 *
 * `game_fixed_update` reads the one-shot flags, acts on them, and clears them before
 * returning. A single-frame reset press therefore resets exactly once even when eight
 * substeps run in that frame. Held controls (`steer`, `throttle`, `brake`, `handbrake`)
 * remain valid for every substep of the frame.
 *
 * Steering follows the left-positive convention: left key -> +1, right key -> -1.
 */
#ifndef CIRCUIT_INPUT_H
#define CIRCUIT_INPUT_H

#include <stdbool.h>

typedef struct {
    /* Held controls, sampled every render frame, valid for every substep */
    float steer;     /* -1 (right) .. +1 (left), dimensionless */
    float throttle;  /* 0 .. 1, dimensionless */
    float brake;     /* 0 .. 1, dimensionless */
    float handbrake; /* 0 .. 1, dimensionless */

    /* One-shot commands, edge-triggered, cleared after the first fixed update */
    bool pausePressed;
    bool resetPressed;
    bool debugPressed;
    bool shiftUpPressed;
    bool shiftDownPressed;
    bool toggleAutoPressed;
    /*
     * Menu-only commands, and deliberately not part of the replay frame.
     *
     * A recording reproduces a race, not the menu session that configured it: the car identity
     * (id, content version, content hash) and the setup are captured as a snapshot alongside
     * frame zero (see replay_capture_initial_vehicle) rather than reconstructed by re-running
     * keystrokes. That is the stronger guarantee — playback restores the recorded setup exactly,
     * and a recording whose snapshot does not match the live car is refused outright instead of
     * being replayed against the wrong vehicle. Serialising these bits would add a second,
     * weaker path to the same state and let the two disagree.
     */
    /* Menu navigation: previous / next selectable car (LEFT/RIGHT, or gamepad d-pad). */
    bool leftPressed;
    bool rightPressed;
    /* Setup editor (menu only): toggle the setup screen, move cursor, adjust, reset. */
    bool upPressed;
    bool downPressed;
    bool setupTogglePressed;
    bool resetSetupPressed;
} Input;

/* Zero every field. */
void input_zero(Input *in);

/* Clear only the one-shot commands; held controls are untouched. */
void input_clear_oneshots(Input *in);

/* True while any one-shot command is still pending consumption. */
bool input_has_oneshot(const Input *in);

/* Poll the keyboard for one render frame. Held controls are overwritten; one-shot
 * commands are latched on top of whatever is still pending.
 * Defined only in builds that link raylib (not in CIRCUIT_HEADLESS builds). */
void input_sample(Input *in);

#endif /* CIRCUIT_INPUT_H */
