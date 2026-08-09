/*
 * simulation_fixture.h — one Game -> TelemetryRow projection, shared by every writer.
 *
 * The handling scenarios, the determinism scenario and the failure-bundle verifier all have
 * to emit byte-identical telemetry, so the projection lives in exactly one place.
 */
#ifndef CIRCUIT_SIMULATION_FIXTURE_H
#define CIRCUIT_SIMULATION_FIXTURE_H

#include "game/game.h"
#include "game/telemetry.h"

TelemetryRow test_telemetry_row_from_game(const Game *game, int substepCount);

#endif /* CIRCUIT_SIMULATION_FIXTURE_H */
