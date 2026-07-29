/*
 * scoring.h — drift detection, scoring classification, and score accumulation.
 *
 * PURE GAMEPLAY COMPUTATION. This translation unit calls no raylib function and uses
 * only raylib's Vector2 type. It may include <math.h> and the project's math_utils.h.
 *
 * Physical state and scoring state are strictly separate. scoringDrift describes the
 * game rules; physicallySliding describes the tires. Scoring state never modifies
 * forces, and nothing in the force path may read either classification. That invariant
 * is asserted by the "scoring-determinism" scenario in tests/scenarios/gameplay_tests.c.
 *
 * Phase 6 chunk [6a]: drift scoring + high-score persistence.
 */
#ifndef DRIFTY_SCORING_H
#define DRIFTY_SCORING_H

#include <stdbool.h>

#include "physics/vehicle.h"

/* Forward-declared; actual type in game.h. */
struct Game;

/*
 * Classify the current physical state as a scoring drift or not.
 *
 * Reads speed, body sideslip, rear slip angle, yaw rate, wheel surface, and the
 * crash-lockout timer. Writes derived->scoringDrift. Returns the same bool.
 *
 * All thresholds are compile-time constants in config.h:
 *   MIN_DRIFT_SPEED_MPS, MIN_DRIFT_ANGLE_RAD, MIN_REAR_SLIP_RAD,
 *   MIN_DRIFT_YAW_RATE_RADS, SPIN_CUTOFF_RAD.
 *
 * A scoring drift requires ALL seven conditions to be true. See docs/SPEC.md for the
 * full list.
 */
bool scoring_classify(const VehicleState *state, VehicleDerived *derived,
                      float crashLockoutTimerS);

/*
 * Accumulate score and combo for one fixed tick.
 *
 * Reads derived->scoringDrift (set by the preceding scoring_classify call).
 * Mutates game->driftScore, driftTimeS, comboMultiplier, comboTimerS.
 * Does not touch any field that enters the state checksum.
 */
void scoring_update(struct Game *game, float dt);

#endif /* DRIFTY_SCORING_H */
