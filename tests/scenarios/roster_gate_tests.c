/*
 * roster_gate_tests.c — Layer A: the graded, classifier-driven roster gate.
 *
 * Runs each roster car through the shared AI driver (same setup as the strict milestone gate
 * ai-roster-laps), but reduces each run with the #78 classifier (validation_metrics_compute +
 * validation_classify) and asserts on the graded outcome + named reason instead of bare lap
 * count. It COEXISTS with ai-roster-laps and does not weaken it: the five well-behaved cars
 * must still classify PASS, and awd_rally — the documented stress car (issue #77) — is asserted
 * only on structural invariants (finite, shared config unmodified), so a future fix that turns
 * it green does not break this gate.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "support/test_harness.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "core/config.h"
#include "game/game.h"
#include "game/input.h"
#include "game/car_roster.h"
#include "game/ai_driver.h"
#include "game/telemetry.h"
#include "game/validation_metrics.h"
#include "game/validation_classifier.h"

static void scenario_ai_roster_graded(void)
{
    const int rosterCount = car_roster_count();
    check(rosterCount == 6, "car roster holds 6 cars (got %d)", rosterCount);

    AiDriverConfig cfgAtStart;
    ai_driver_config_default(&cfgAtStart);

    const int budgetTicks = REPLAY_CAPACITY_TICKS;
    TelemetryRow *rows = (TelemetryRow *)malloc((size_t)budgetTicks * sizeof(TelemetryRow));
    check(rows != NULL, "graded gate allocated its per-car telemetry buffer");

    for (int i = 0; i < rosterCount; i++) {
        VehicleSpec spec;
        check(car_roster_spec(i, &spec), "roster spec %d built successfully", i);

        char carId[64];
        car_roster_id(i, carId, sizeof(carId));
        const bool isStress = (strcmp(carId, "awd_rally") == 0);

        Game *game = alloc_game();
        game_init(game);
        game_apply_spec(game, &spec);
        track_load_chicane(&game->trackDef);
        game_spawn_on_track(game);

        game->autoTrans.enabled = true;
        game->autoTrans.forwardOnly = true;
        game->state = STATE_PLAYING;
        game->session.rules.targetLaps = VALIDATION_RUN_LAPS;
        controller_init(&game->controller, CONTROLLER_KIND_AI);

        bool allFinite = true;
        int rowCount = 0;
        for (int t = 0; t < budgetTicks && game->progress.lap < VALIDATION_RUN_LAPS; t++) {
            game_fixed_update(game, FIXED_DT_S);
            if (rowCount < budgetTicks) rows[rowCount++] = game_telemetry_row(game, 1);
            if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y))
                allFinite = false;
        }

        ValidationMetrics metrics;
        validation_metrics_compute(rows, rowCount, &metrics);

        ClassificationInputs clsIn;
        validation_classification_inputs_default(&clsIn);
        clsIn.checkpointCount = game->trackDef.checkpointCount;
        clsIn.startCheckpointIndex = 0;
        clsIn.targetLaps = VALIDATION_RUN_LAPS;
        clsIn.tickBudget = budgetTicks;
        clsIn.ticksRun = rowCount;
        clsIn.fixedDtS = FIXED_DT_S;

        ValidationClassification cls;
        validation_classify(rows, rowCount, &metrics, &clsIn, &cls);

        printf(
            "    ai-roster-graded %-10s %-22s laps %d  off-track %.1fs  peak-friction %.3f\n",
            carId, failure_class_reason(cls.primary), game->progress.lap, metrics.offTrackTimeS,
            (double)metrics.maxFrictionUsage);

        check(allFinite, "car '%s' stayed finite", carId);
        check(memcmp(&game->controller.config.ai, &cfgAtStart, sizeof(cfgAtStart)) == 0,
              "car '%s' was driven with the unmodified shared AiDriverConfig", carId);

        if (!isStress) {
            /* The five well-behaved cars must complete cleanly under the graded budget. */
            check(cls.primary == RUN_CLASS_PASS, "car '%s' classified PASS (got %s)", carId,
                  failure_class_reason(cls.primary));
        }
        /* awd_rally is excluded from the pass assertion: it is the documented stress car
         * (issue #77, chaotic ai-roster-laps gate). Its classification is printed for
         * diagnosis above; asserting on its outcome would make a future fix (#28 / AI v2)
         * break this gate. */

        track_free(&game->trackDef);
        free(game);
    }

    free(rows);
}

static const TestScenario kRosterGateScenarios[] = {
    { "ai-roster-graded",
      "Layer A: graded classifier-budget gate over the roster (coexists with ai-roster-laps)",
      scenario_ai_roster_graded },
};

TestScenarioGroup test_roster_gate_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kRosterGateScenarios;
    group.count = sizeof(kRosterGateScenarios) / sizeof(kRosterGateScenarios[0]);
    return group;
}
