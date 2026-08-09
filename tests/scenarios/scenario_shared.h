/*
 * scenario_shared.h — the few helpers that genuinely span scenario groups.
 *
 * This is a private declaration header for tests/scenarios/, not a support module: each helper
 * is still implemented by the scenario file that owns it. The scripted input timeline is
 * implemented in core_tests.c (the replay, render-scale and one-shot scenarios are its primary
 * users) and read by the physics telemetry scenario; set_vehicle_rolling_speed is implemented
 * in physics_tests.c and used by the physics, handling and gameplay groups.
 */
#ifndef CIRCUIT_SCENARIO_SHARED_H
#define CIRCUIT_SCENARIO_SHARED_H

#include <stdbool.h>
#include <stdint.h>

#include "game/game.h"
#include "game/replay.h"
#include "game/telemetry.h"

#define SCRIPT_FRAMES 600

typedef struct {
    float steer;
    float throttle;
    float brake;
    float handbrake;
    bool pause;
    bool reset;
    bool debug;
    bool shiftUp;
    bool shiftDown;
    float frameTimeS;
} ScriptFrame;

void script_build(ScriptFrame *frames, int count);
void fixed_update_adapter(void *ctx, float dt);
void apply_live_input(Game *game, const ScriptFrame *f);
uint32_t run_recording(Game *game, const ScriptFrame *frames, int count, float pixelsPerMeter,
                       TelemetryWriter *writer);
uint32_t run_playback(Game *game, const ReplayBuffer *timeline, const ScriptFrame *frames,
                      int count, float pixelsPerMeter);

/* Put the vehicle at a steady speed with every wheel already rolling at it. */
void set_vehicle_rolling_speed(Game *game, float velocityLongitudinalMps);

#endif /* CIRCUIT_SCENARIO_SHARED_H */
