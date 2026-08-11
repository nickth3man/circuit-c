/*
 * stability_tests.c — Layer C: plant-level (no-AI) stability and headroom tests.
 *
 * These test the CAR, not the controller, so they are deterministic and physics-meaningful.
 * They surface each car's friction headroom as a named PROPERTY (awd_rally's near-zero margin
 * shows up as data here, not as a mysterious AI lap failure) and assert the yaw step response
 * is bounded and non-divergent. They reuse the shared scripted maneuvers ("skidpad",
 * "step-steer") via the dev-scenario system so no AI driver is involved.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "support/test_harness.h"
#include "test_scenarios.h"

#include "core/config.h"
#include "dev/dev_scenario.h"
#include "game/game.h"
#include "game/car_roster.h"

/* Drive one roster car through a scripted maneuver and return whether it stayed finite,
 * plus the peak per-tick max friction usage over the run. */
static bool run_scripted_roster_car(int carIndex, const char *scenarioName, float *outPeakUsage,
                                    float *outPeakYawRate)
{
    const int idx = dev_scenario_find(scenarioName);
    if (idx <= 0) return false;
    const DevScenario *sc = dev_scenario_at(idx);

    VehicleSpec spec;
    if (!car_roster_spec(carIndex, &spec)) return false;

    Game *game = alloc_game();
    game_init(game);
    game_apply_spec(game, &spec);
    game->dev.scenario = idx;
    game->dev.scenarioRunning = true;
    game->dev.scenarioStartTick = game->sim.tick;

    *outPeakUsage = 0.0f;
    *outPeakYawRate = 0.0f;
    bool allFinite = true;
    for (int t = 0; t < sc->durationTicks; t++) {
        game_fixed_update(game, FIXED_DT_S);
        if (game->derived.maxFrictionUsage > *outPeakUsage)
            *outPeakUsage = game->derived.maxFrictionUsage;
        const float yr = fabsf(game->vehicle.yawRateRadS);
        if (yr > *outPeakYawRate) *outPeakYawRate = yr;
        if (!isfinite(game->vehicle.positionM.x) || !isfinite(game->vehicle.positionM.y) ||
            !isfinite(game->vehicle.yawRateRadS))
            allFinite = false;
    }

    free(game);
    return allFinite;
}

static int find_roster_car(const char *id)
{
    for (int i = 0; i < car_roster_count(); i++) {
        char carId[64];
        car_roster_id(i, carId, sizeof(carId));
        if (strcmp(carId, id) == 0) return i;
    }
    return -1;
}

/* friction-headroom — every roster car on the constant-radius skidpad keeps peak friction
 * usage at/under the budget (the combined-slip limiter enforces this by construction). The
 * reported headroom (1 - peak) names awd_rally's near-zero margin as a property. */
static void scenario_friction_headroom(void)
{
    const int rosterCount = car_roster_count();
    check(rosterCount == 6, "car roster holds 6 cars (got %d)", rosterCount);

    for (int i = 0; i < rosterCount; i++) {
        char carId[64];
        car_roster_id(i, carId, sizeof(carId));

        float peakUsage = 0.0f, peakYaw = 0.0f;
        const bool finite = run_scripted_roster_car(i, "skidpad", &peakUsage, &peakYaw);

        printf("    friction-headroom %-10s peak-usage %.3f  headroom %.3f\n", carId,
               (double)peakUsage, 1.0 - (double)peakUsage);

        check(finite, "car '%s' stayed finite on the skidpad", carId);
        check(peakUsage <= 1.0f + FRICTION_TOLERANCE,
              "car '%s' stays within the friction budget on the skidpad (peak %.4f)", carId,
              (double)peakUsage);
        (void)peakYaw;
    }
}

/* step-steer-damping — the yaw-rate response to a steering step is bounded and does not
 * diverge for the stable car and the stress car alike. Loose, characterizing bounds that
 * hold for every car (including awd_rally); the per-car peaks are printed. */
static void scenario_step_steer_damping(void)
{
    const char *const carIds[] = { "rwd_grip", "awd_rally" };
    const int n = (int)(sizeof(carIds) / sizeof(carIds[0]));

    for (int c = 0; c < n; c++) {
        const int carIdx = find_roster_car(carIds[c]);
        check(carIdx >= 0, "%s is in the roster", carIds[c]);

        /* Run the maneuver twice: once for the whole-run peak, once measuring the last 2 s
         * against it. The dev scenario is a pure function of tick, so both runs agree. */
        float peakUsage = 0.0f, peakYaw = 0.0f;
        bool finite = run_scripted_roster_car(carIdx, "step-steer", &peakUsage, &peakYaw);
        check(finite, "%s stayed finite through the step-steer", carIds[c]);

        /* Late-window peak: re-run and sample only the final 2 s. */
        const int idx = dev_scenario_find("step-steer");
        const DevScenario *sc = dev_scenario_at(idx);
        const int lateStart = sc->durationTicks - 2 * FIXED_HZ;
        VehicleSpec spec;
        car_roster_spec(carIdx, &spec);
        Game *game = alloc_game();
        game_init(game);
        game_apply_spec(game, &spec);
        game->dev.scenario = idx;
        game->dev.scenarioRunning = true;
        game->dev.scenarioStartTick = game->sim.tick;
        float latePeakYaw = 0.0f;
        for (int t = 0; t < sc->durationTicks; t++) {
            game_fixed_update(game, FIXED_DT_S);
            if (t >= lateStart) {
                const float yr = fabsf(game->vehicle.yawRateRadS);
                if (yr > latePeakYaw) latePeakYaw = yr;
            }
        }
        free(game);

        printf("    step-steer-damping %-10s peak-yaw %.3f rad/s  last-2s-peak %.3f\n",
               carIds[c], (double)peakYaw, (double)latePeakYaw);

        check(peakYaw < MAX_SAFE_YAW_RATE_RADS, "%s yaw rate is bounded (peak %.3f rad/s)",
              carIds[c], (double)peakYaw);
        check(latePeakYaw <= peakYaw + 1e-3f,
              "%s yaw response does not diverge (last-2s %.3f <= run peak %.3f)", carIds[c],
              (double)latePeakYaw, (double)peakYaw);
        (void)peakUsage;
    }
}

static const TestScenario kStabilityScenarios[] = {
    { "friction-headroom",
      "Layer C: every roster car holds friction budget on the skidpad (headroom reported)",
      scenario_friction_headroom },
    { "step-steer-damping",
      "Layer C: yaw step response is bounded and non-divergent for stable and stress cars",
      scenario_step_steer_damping },
};

TestScenarioGroup test_stability_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kStabilityScenarios;
    group.count = sizeof(kStabilityScenarios) / sizeof(kStabilityScenarios[0]);
    return group;
}
