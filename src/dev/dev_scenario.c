#include "dev/dev_scenario.h"

#include <string.h>

#include "core/config.h"

/* Scenario scripts are written in seconds and converted here, so a change to FIXED_HZ moves
 * the tick boundaries without rewriting a single scenario. */
#define AT(seconds) ((uint64_t)((seconds) * (double)FIXED_HZ))

typedef enum {
    SCENARIO_FREE = 0,
    SCENARIO_ACCEL,
    SCENARIO_SKIDPAD,
    SCENARIO_STEP_STEER,
    SCENARIO_LIFT_OFF,
    SCENARIO_POWER_OVERSTEER,
    SCENARIO_HANDBRAKE_ENTRY,
    SCENARIO_TRANSITION,
    SCENARIO_BRAKE_CORNER,
    SCENARIO_COAST_DOWN,
    SCENARIO_ACCEL_LOAD,
    SCENARIO_BRAKE_LOAD,
    SCENARIO_CATCHABLE_DRIFT
} ScenarioIndex;

static const DevScenario g_scenarios[] = {
    { "free",            "no scripted input; drive it yourself",                    0,    0u },
    { "accel",           "standing start at full throttle, straight ahead",         1200, 1001u },
    { "skidpad",         "constant radius: settle, then hold steady steer and throttle", 2400, 1002u },
    { "step-steer",      "build speed, step the steering, hold it, then return to centre", 960, 1003u },
    { "lift-off",        "steady cornering interrupted by a throttle lift",         1200, 1004u },
    { "power-oversteer", "steady steer, then full throttle until the rear breaks",  1200, 1005u },
    { "handbrake-entry", "handbrake pull into a corner, release, then counter-steer", 1200, 1006u },
    { "transition",      "left/right transitions at a constant period",             1800, 1007u },
    { "brake-corner",    "braking while already turning",                           1200, 1008u },
    { "coast-down",      "accelerate, then coast with every control released",      2400, 1009u },
    { "accel-load",      "straight full-throttle launch: load transfers rearward",  720,  1010u },
    { "brake-load",      "accelerate, then brake to a stop: load transfers forward", 1080, 1011u },
    { "catchable-drift", "initiate, hold, countersteer, and recover a slide",        1200, 1012u },
};

#define SCENARIO_COUNT ((int)(sizeof(g_scenarios) / sizeof(g_scenarios[0])))

int dev_scenario_count(void)
{
    return SCENARIO_COUNT;
}

const DevScenario *dev_scenario_at(int index)
{
    if (index < 0 || index >= SCENARIO_COUNT) return NULL;
    return &g_scenarios[index];
}

int dev_scenario_find(const char *name)
{
    if (name == NULL) return -1;
    for (int i = 0; i < SCENARIO_COUNT; i++) {
        if (strcmp(g_scenarios[i].name, name) == 0) return i;
    }
    return -1;
}

void dev_scenario_input(int index, uint64_t tick, Input *out)
{
    if (out == NULL) return;
    input_zero(out);
    if (index <= SCENARIO_FREE || index >= SCENARIO_COUNT) return;

    switch (index) {
        case SCENARIO_ACCEL:
            out->throttle = 1.0f;
            break;

        case SCENARIO_SKIDPAD:
            /* Two seconds of straight-line acceleration, then steering wound on over a
             * second and held with partial throttle for the remaining seventeen. The ramp
             * matters: stepping to full lock would measure a transient, and this scenario
             * exists to measure a steady state. Steady state is whatever the model settles
             * on — the script states no target. */
            if (tick < AT(2.0)) {
                out->throttle = 1.0f;
            } else {
                const uint64_t sinceEntry = tick - AT(2.0);
                const float ramp = (sinceEntry >= AT(1.0))
                    ? 1.0f : (float)sinceEntry / (float)AT(1.0);
                out->steer = 0.25f * ramp;
                out->throttle = 0.30f;
            }
            break;

        case SCENARIO_STEP_STEER:
            /* Three seconds to reach a settled speed, a step held for three and a half, then
             * back to centre so the return-to-straight half of the response is measurable
             * too. Rise time, overshoot, and settling are read off the hold; recovery is
             * read off the release. */
            out->throttle = (tick < AT(3.0)) ? 0.60f : 0.30f;
            if (tick >= AT(3.0) && tick < AT(6.5)) out->steer = 0.20f;
            break;

        case SCENARIO_LIFT_OFF:
            /* The corner has to be STABLE before the lift, or the transient is unreadable:
             * a car already past the rear tires' peak has nowhere further to rotate. */
            out->steer = (tick < AT(2.0)) ? 0.0f : 0.22f;
            out->throttle = (tick < AT(6.0)) ? 0.45f : 0.0f;
            break;

        case SCENARIO_POWER_OVERSTEER:
            out->steer = 0.40f;
            out->throttle = (tick < AT(2.0)) ? 0.40f : 1.0f;
            break;

        case SCENARIO_HANDBRAKE_ENTRY:
            out->throttle = (tick < AT(3.0)) ? 1.0f : 0.0f;
            if (tick >= AT(3.0) && tick < AT(3.75)) {
                out->steer = 0.50f;
                out->handbrake = 1.0f;
            } else if (tick >= AT(3.75) && tick < AT(6.0)) {
                out->steer = -0.30f;         /* counter-steer: right is negative */
                out->throttle = 0.35f;
            }
            break;

        case SCENARIO_TRANSITION: {
            out->throttle = (tick < AT(2.0)) ? 0.80f : 0.35f;
            if (tick >= AT(2.0)) {
                /* 1.5 s per half-period, alternating sign. */
                const uint64_t phase = (tick - AT(2.0)) / AT(1.5);
                out->steer = ((phase % 2u) == 0u) ? 0.35f : -0.35f;
            }
            break;
        }

        case SCENARIO_BRAKE_CORNER:
            out->throttle = (tick < AT(4.0)) ? 1.0f : 0.0f;
            if (tick >= AT(4.0)) {
                out->steer = 0.30f;
                out->brake = 0.80f;
            }
            break;

        case SCENARIO_COAST_DOWN:
            out->throttle = (tick < AT(6.0)) ? 1.0f : 0.0f;
            break;

        case SCENARIO_ACCEL_LOAD:
            /* Five seconds of straight full throttle, then one of coast. No steering at all:
             * the only thing that may move the axle loads is longitudinal acceleration. */
            out->throttle = (tick < AT(5.0)) ? 1.0f : 0.0f;
            break;

        case SCENARIO_BRAKE_LOAD:
            /* Four seconds of acceleration, then full service braking to a standstill. */
            out->throttle = (tick < AT(4.0)) ? 1.0f : 0.0f;
            out->brake = (tick >= AT(4.0)) ? 1.0f : 0.0f;
            break;

        case SCENARIO_CATCHABLE_DRIFT:
            /* Five stages, all through ordinary controls: build speed, break the rear loose
             * with the handbrake, hold the slide on throttle, catch it on countersteer, then
             * unwind to straight travel. Nothing here reaches into the physics. */
            if (tick < AT(2.5)) {                       /* 1. build speed */
                out->throttle = 1.0f;
            } else if (tick < AT(3.2)) {                /* 2. initiate */
                out->steer = 0.60f;
                out->handbrake = 1.0f;
            } else if (tick < AT(4.6)) {                /* 3. hold the slide */
                out->steer = 0.25f;
                out->throttle = 0.55f;
            } else if (tick < AT(6.6)) {                /* 4. countersteer */
                out->steer = -0.55f;
                out->throttle = 0.30f;
            } else {                                    /* 5. recover */
                out->steer = 0.0f;
                out->throttle = 0.25f;
            }
            break;

        default:
            break;
    }
}
