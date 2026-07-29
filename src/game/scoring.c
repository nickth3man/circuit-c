/*
 * scoring.c — drift detection, scoring classification, and score accumulation.
 *
 * PURE GAMEPLAY COMPUTATION. No raylib functions called. Only Vector2 and <math.h>.
 *
 * Phase 6 chunk [6a]: drift scoring + high-score persistence.
 */
#include "game/scoring.h"

#include <math.h>
#include <stddef.h>

#include "core/config.h"
#include "game/game.h"
#include "core/math_utils.h"

/* -------------------------------------------------------------------------------------
 * Classification
 * ------------------------------------------------------------------------------------- */

bool scoring_classify(const VehicleState *state,
                      VehicleDerived *derived,
                      float crashLockoutTimerS)
{
    if (state == NULL || derived == NULL) {
        if (derived != NULL) derived->scoringDrift = false;
        return false;
    }

    /* Condition 1: forward speed above the low-speed gate. */
    const bool movingForward = state->velocityLongitudinalMps > 0.0f;

    /* Condition 2: speed above the minimum drift speed. */
    const bool fastEnough = derived->speedMps >= MIN_DRIFT_SPEED_MPS;

    /* Condition 3: body sideslip exceeds the minimum angle. */
    const bool sideslipEnough = fabsf(derived->bodySideslipRad) >= MIN_DRIFT_ANGLE_RAD;

    /* Condition 4: rear-axle slip meets the minimum. */
    const bool rearSlipEnough = fabsf(derived->rearSlipAngleRad) >= MIN_REAR_SLIP_RAD;

    /* Condition 5: yaw rate meets the minimum. */
    const bool yawEnough = fabsf(state->yawRateRadS) >= MIN_DRIFT_YAW_RATE_RADS;

    /* Condition 6: sideslip is not past the spin cutoff — this is a drift, not a spin. */
    const bool notSpun = fabsf(derived->bodySideslipRad) <= SPIN_CUTOFF_RAD;

    /* Condition 7: not in the post-crash lockout period. */
    const bool notInLockout = crashLockoutTimerS <= 0.0f;

    /* Condition 8: at least one rear wheel is on a scoring surface (asphalt). */
    const bool onScoringSurface =
        state->wheels[WHEEL_REAR_LEFT].surfaceId == SURFACE_ASPHALT ||
        state->wheels[WHEEL_REAR_RIGHT].surfaceId == SURFACE_ASPHALT;

    const bool drift = movingForward && fastEnough && sideslipEnough &&
                       rearSlipEnough && yawEnough && notSpun &&
                       notInLockout && onScoringSurface;

    derived->scoringDrift = drift;
    return drift;
}

/* -------------------------------------------------------------------------------------
 * Score and combo accumulation
 * ------------------------------------------------------------------------------------- */

void scoring_update(struct Game *game, float dt)
{
    if (game == NULL) return;

    if (game->derived.scoringDrift) {
        const float bodySideslipRad = fabsf(game->derived.bodySideslipRad);
        const float speedMps = game->derived.speedMps;

        /* angleFactor: 0 at MIN_DRIFT_ANGLE_RAD, 1 at SPIN_CUTOFF_RAD. */
        const float angleFactor = clampf(
            (bodySideslipRad - MIN_DRIFT_ANGLE_RAD) /
                (SPIN_CUTOFF_RAD - MIN_DRIFT_ANGLE_RAD),
            0.0f, 1.0f);

        /* speedFactor: 0 at MIN_DRIFT_SPEED_MPS, 1 at SCORE_SPEED_REF_MPS. */
        const float speedFactor = clampf(
            (speedMps - MIN_DRIFT_SPEED_MPS) /
                (SCORE_SPEED_REF_MPS - MIN_DRIFT_SPEED_MPS),
            0.0f, 1.0f);

        /* lineFactor is a placeholder for racing-line quality; always 1.0 for now. */
        const float lineFactor = 1.0f;

        game->driftTimeS += dt;

        /* While actively drifting, the combo grace timer never advances. */
        game->comboTimerS = 0.0f;

        /* Combo multiplier rises over time, capped at 4.0. */
        game->comboMultiplier = clampf(1.0f + game->driftTimeS * 0.5f, 1.0f, 4.0f);

        game->driftScore += SCORE_BASE_RATE * angleFactor *
                            speedFactor * lineFactor *
                            game->comboMultiplier * dt;
        game->driftScore = fminf(game->driftScore, (float)MAX_VALID_SCORE);
    } else {
        game->comboTimerS += dt;
        if (game->comboTimerS >= COMBO_GRACE_S) {
            game->comboMultiplier = 1.0f;
            game->driftTimeS = 0.0f;
        }
    }
}
