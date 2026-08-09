/*
 * controller_output.h — the one value a controller may produce, and the one path it is
 * validated through.
 *
 * Human, scripted, replayed and AI entrants used to reach the fixed update by three different
 * routes: the keyboard and the validation AI both wrote `Game.input`, a running dev scenario
 * substituted its own `Input`, and playback unpacked a `ReplayFrame`. Three routes means three
 * places a second entrant could accidentally read or clobber the first one's controls. This
 * struct is what replaced them.
 *
 * WHAT IS AND IS NOT A VEHICLE CONTROL. Pause, reset, the debug overlay and the automatic
 * transmission toggle are application/session commands: they belong to one session, not to one
 * car, so they stay on `Input` and are consumed once per tick by the session. A gear shift is
 * something a driver asks the car for, so it is controller output and every entrant owns its
 * own.
 *
 * This header is deliberately tiny and dependency-free apart from Input, so that physics.h,
 * auto_transmission.h and ai_driver.h can speak the vocabulary without seeing the controller
 * aggregate in controller.h.
 */
#ifndef CIRCUIT_CONTROLLER_OUTPUT_H
#define CIRCUIT_CONTROLLER_OUTPUT_H

#include <stdbool.h>

#include "game/input.h"

/*
 * One tick of driving for one entrant.
 *
 * Units and signs match Input's held controls exactly, so a recorded human timeline and a
 * planned AI lap are the same kind of value and can be compared, recorded and replayed
 * interchangeably.
 */
typedef struct {
    float steer;     /* -1 (right) .. +1 (left), dimensionless */
    float throttle;  /* 0 .. 1, dimensionless */
    float brake;     /* 0 .. 1, dimensionless */
    float handbrake; /* 0 .. 1, dimensionless */
    bool shiftUp;    /* request one gear up on this tick */
    bool shiftDown;  /* request one gear down on this tick */
} ControllerOutput;

/* Zero every field. A zeroed output is "hands off everything", which is a safe default. */
void controller_output_zero(ControllerOutput *out);

/* Convert one input sample into controller output. Held controls map across unchanged and the
 * shift one-shots become this tick's gear requests; the application one-shots are deliberately
 * dropped, because they are not vehicle controls. A NULL sample yields a zeroed output. */
void controller_output_from_input(const Input *in, ControllerOutput *out);

/*
 * The single validation/clamping path every entrant's output leaves through: steer into
 * [-1, +1], the three pedals into [0, 1], and any non-finite value to zero, so no controller
 * can hand physics a demand the keyboard could not have produced.
 */
void controller_output_clamp(ControllerOutput *out);

#endif /* CIRCUIT_CONTROLLER_OUTPUT_H */
