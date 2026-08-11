/*
 * robustness_tests.c — Layer B: the statistical (chaos-tolerant) roster gate.
 *
 * A single deterministic run of a chaotic car (awd_rally, issue #77) is unassertable; the
 * DISTRIBUTION over small perturbations is. This runs a stable car (rwd_grip) and the stress
 * car (awd_rally) for N trials each with a tiny deterministic initial-heading offset, and
 * asserts: the stable car completes its lap in every trial (N-of-N robust), and the stress
 * car stays finite in every trial (chaotic but bounded — never a NaN/crash). The stress car's
 * completion rate is reported, not asserted, so a future fix does not break the gate.
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "support/test_harness.h"
#include "test_scenarios.h"
#include "scenario_shared.h"

#include "core/config.h"
#include "game/game.h"
#include "game/input.h"
#include "game/car_roster.h"
#include "game/ai_driver.h"

/* Find a roster car by id; returns its index or -1. */
static int find_roster_car(const char *id)
{
    for (int i = 0; i < car_roster_count(); i++) {
        char carId[64];
        car_roster_id(i, carId, sizeof(carId));
        if (strcmp(carId, id) == 0) return i;
    }
    return -1;
}

static void scenario_ai_roster_robustness(void)
{
    AiDriverConfig cfgAtStart;
    ai_driver_config_default(&cfgAtStart);

    const int N = 5;
    const int targetLaps = 1;
    /* One lap is ~5000 ticks at FIXED_HZ on the chicane; 8000 gives the stable car margin
     * while keeping the stress car's full-budget trials bounded in runtime. */
    const int budgetTicks = 8000;

    const int stableIdx = find_roster_car("rwd_grip");
    const int stressIdx = find_roster_car("awd_rally");
    check(stableIdx >= 0, "rwd_grip is in the roster");
    check(stressIdx >= 0, "awd_rally is in the roster");

    struct {
        const char *id;
        int index;
        int completes;
        int finiteTrials;
    } cars[2] = {
        { "rwd_grip", stableIdx, 0, 0 },
        { "awd_rally", stressIdx, 0, 0 },
    };

    for (int c = 0; c < 2; c++) {
        VehicleSpec spec;
        check(car_roster_spec(cars[c].index, &spec), "roster spec for %s built", cars[c].id);

        for (int k = 0; k < N; k++) {
            Game *game = alloc_game();
            game_init(game);
            game_apply_spec(game, &spec);
            track_load_chicane(&game->trackDef);
            game_spawn_on_track(game);

            /* Deterministic micro-perturbation: a tiny initial-heading offset per trial. The
             * physics is deterministic, so without this every trial is bit-identical; the
             * offset is what exposes the outcome distribution. */
            game->vehicle.headingRad += (float)(k - N / 2) * 0.0005f;

            game->autoTrans.enabled = true;
            game->autoTrans.forwardOnly = true;
            game->state = STATE_PLAYING;
            game->session.rules.targetLaps = targetLaps;
            controller_init(&game->controller, CONTROLLER_KIND_AI);

            bool allFinite = true;
            for (int t = 0; t < budgetTicks && game->progress.lap < targetLaps; t++) {
                game_fixed_update(game, FIXED_DT_S);
                if (!isfinite(game->vehicle.positionM.x) ||
                    !isfinite(game->vehicle.positionM.y))
                    allFinite = false;
            }

            if (allFinite) cars[c].finiteTrials++;
            if (game->progress.lap >= targetLaps) cars[c].completes++;
            check(memcmp(&game->controller.config.ai, &cfgAtStart, sizeof(cfgAtStart)) == 0,
                  "%s trial %d used the unmodified shared AiDriverConfig", cars[c].id, k);

            track_free(&game->trackDef);
            free(game);
        }

        printf("    robustness %-10s completed %d/%d  finite %d/%d\n", cars[c].id,
               cars[c].completes, N, cars[c].finiteTrials, N);
    }

    /* The stable car is robust (completes every perturbed trial); the stress car's failures
     * stay bounded (always finite — chaotic, never a crash/NaN). */
    check(cars[0].completes == N,
          "rwd_grip completes 1 lap in all %d perturbed trials (got %d)", N, cars[0].completes);
    check(cars[1].finiteTrials == N,
          "awd_rally stays finite in all %d perturbed trials (got %d) — chaotic but bounded", N,
          cars[1].finiteTrials);
}

static const TestScenario kRobustnessScenarios[] = {
    { "ai-roster-robustness",
      "Layer B: statistical (chaos-tolerant) roster gate over N perturbed trials",
      scenario_ai_roster_robustness },
};

TestScenarioGroup test_robustness_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kRobustnessScenarios;
    group.count = sizeof(kRobustnessScenarios) / sizeof(kRobustnessScenarios[0]);
    return group;
}
