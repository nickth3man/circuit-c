/*
 * controller.h — the one control boundary every entrant's driving arrives through.
 *
 * `ControllerOutput` (controller_output.h) is the value; this header is the thing that
 * produces one per entrant per fixed tick, whether the driver is a person, a script, a
 * recording, or the baseline AI.
 *
 * SAMPLING IS NOT CONSUMPTION. The platform samples devices once per rendered FRAME into
 * `Game.input` (see input.h). A controller consumes a sample once per fixed TICK and may not
 * write to it — `ControllerTickView` is const throughout for exactly that reason.
 *
 * HOT RELOAD. `Controller` is a discriminated plain-data union selected by `ControllerKind`
 * and dispatched by a switch in controller.c. It stores no function pointer, so it survives a
 * module swap inside the persistent Game block like everything else there.
 *
 * Raylib-free apart from Vector2 (through ai_driver.h), so the headless harness links it.
 */
#ifndef CIRCUIT_CONTROLLER_H
#define CIRCUIT_CONTROLLER_H

#include "game/ai_driver.h"
#include "game/controller_output.h"
#include "game/input.h"
#include "physics/vehicle.h"
#include "world/track.h"

typedef enum {
    CONTROLLER_KIND_HUMAN = 0, /* the frame-latched device sample */
    CONTROLLER_KIND_SCRIPT,    /* a scripted maneuver or test timeline */
    CONTROLLER_KIND_REPLAY,    /* a recorded controller stream */
    CONTROLLER_KIND_AI,        /* the baseline lap driver in ai_driver.c */
    CONTROLLER_KIND_COUNT
} ControllerKind;

/*
 * The read-only tick-start view. Every pointer is const: a controller can look at the world
 * and at its own memory, and it can write nothing but its ControllerOutput. That is the same
 * audit boundary ai_driver.h has always described, now applied to every kind.
 *
 * `sample` is whatever the session acquired for this tick — the latched device sample, a
 * scripted frame, or an unpacked replay frame. It may be NULL for a controller that needs no
 * sample.
 */
typedef struct {
    const Input *sample;
    const TrackDefinition *track;
    const TrackRuntime *runtime;
    const VehicleState *vehicle;
    const VehicleDerived *derived;
    const VehicleSpec *spec;
    float dt;
} ControllerTickView;

/*
 * One entrant's controller. `kind` and `config` are frozen when the entrant is created;
 * `memory` is the private decision history, mutable only during the controller stage and
 * cleared by controller_reset().
 */
typedef struct {
    ControllerKind kind;
    union {
        AiDriverConfig ai;
    } config;
    union {
        AiDriverState ai;
    } memory;
} Controller;

/* Select a kind, install that kind's default configuration, and clear the private memory. */
void controller_init(Controller *controller, ControllerKind kind);

/* Clear the private decision memory. Kind and configuration are preserved, so a car returned
 * to the grid drives from a blank plan rather than one computed for where it used to be. */
void controller_reset(Controller *controller);

/*
 * Produce this tick's output for one entrant.
 *
 * `sourceKind` is normally `controller->kind`. A session replaying a recorded timeline passes
 * CONTROLLER_KIND_REPLAY instead, so the recorded output is authoritative and no controller
 * re-decides during playback — the same rule that already stops a scripted scenario from
 * second-guessing a timeline. Whatever the kind, the result leaves through
 * controller_output_clamp().
 */
void controller_update(Controller *controller, ControllerKind sourceKind,
                       const ControllerTickView *view, ControllerOutput *out);

#endif /* CIRCUIT_CONTROLLER_H */
