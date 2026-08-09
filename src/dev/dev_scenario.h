/*
 * dev_scenario.h — the scripted maneuvers, defined once for every consumer.
 *
 * A scenario is a pure function from "ticks since the scenario started" to an Input. That is
 * all it is: no state, no randomness, no wall-clock. The same table therefore drives
 *
 *   - the Physics Lab's scenario selector (src/dev/dev_lab.c),
 *   - the headless maneuver runs (`circuit_tests --scenario skidpad`),
 *   - the physics-regression workflow, which replays them and diffs telemetry.
 *
 * Because the input timeline is a deterministic function of the tick index, two runs of the
 * same scenario on the same binary produce identical telemetry, which is what makes the
 * regression comparison meaningful.
 *
 * These scripts describe *what the driver does*, never what the car should do. They contain
 * no tuning targets, so adding one cannot smuggle in a handling change.
 */
#ifndef CIRCUIT_DEV_SCENARIO_H
#define CIRCUIT_DEV_SCENARIO_H

#include <stdbool.h>
#include <stdint.h>

#include "game/input.h"

typedef struct {
    const char *name; /* stable key, e.g. "skidpad" */
    const char *description;
    int durationTicks; /* recommended run length at FIXED_HZ */
    uint32_t seed;     /* recorded in telemetry and failure bundles */
} DevScenario;

int dev_scenario_count(void);
const DevScenario *dev_scenario_at(int index);

/* Index of the scenario with this name, or -1. */
int dev_scenario_find(const char *name);

/* Fill *out with the input this scenario applies at `tick` ticks after its start.
 * Out-of-range indices produce a zeroed Input, which is a safe free-drive default. */
void dev_scenario_input(int index, uint64_t tick, Input *out);

#endif /* CIRCUIT_DEV_SCENARIO_H */
