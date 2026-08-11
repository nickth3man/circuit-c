/*
 * invariant_tests.c — Layer D: baseline-independent physics invariants.
 *
 * These assert LAWS, not "matches last week's recording", so they stay green through every
 * physics iteration. Two scenarios:
 *   friction-ellipse  — a pure-function sweep proving the combined-slip resultant never
 *                       exceeds mu*Fz, and that each tire curve saturates at the friction peak.
 *   camber-invariants — via the #15 diagnostic derived.camberThrustN: zero camber (caster
 *                       zeroed) produces no thrust, nonzero camber produces thrust, and the
 *                       two signs are symmetric. Defends issue #15's acceptance criteria.
 * (Energy bookkeeping was considered and deferred: its sign conventions are easy to get
 *  wrong and a flaky invariant is worse than none.)
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
#include "physics/tire.h"
#include "physics/vehicle.h"
#include "dev/dev_scenario.h"
#include "game/game.h"
#include "game/car_roster.h"

/* friction-ellipse — the combined-slip resultant stays inside mu*Fz for every (slip, load,
 * grip) combination, and each pure curve saturates at its friction peak. Pure functions, no
 * sim: the law holds regardless of the car or the controller. */
static void scenario_friction_ellipse(void)
{
    const float FzSet[] = { 2000.0f, 4000.0f, 6000.0f };
    const float muSet[] = { 0.55f, 1.0f, 1.3f };
    const float B = 10.0f;
    const float C = 1.45f;

    bool withinEllipse = true;
    float worstUsage = 0.0f;

    for (size_t fi = 0; fi < sizeof(FzSet) / sizeof(FzSet[0]); fi++) {
        for (size_t mi = 0; mi < sizeof(muSet) / sizeof(muSet[0]); mi++) {
            const float Fz = FzSet[fi];
            const float mu = muSet[mi];
            const float limit = mu * Fz;

            for (int ai = 0; ai <= 12; ai++) {
                for (int ri = -12; ri <= 12; ri++) {
                    const float slipAngle = (float)ai * 0.025f; /* 0 .. 0.30 rad */
                    const float slipRatio = (float)ri * 0.025f; /* -0.30 .. 0.30 */

                    const float Fy = tire_lateral_force_n(slipAngle, Fz, B, C, mu);
                    const float Fx = tire_longitudinal_force_n(slipRatio, Fz, B, C, mu);

                    float fxLim, fyLim, usage;
                    tire_apply_combined_limit(Fx, Fy, limit, limit, &fxLim, &fyLim, &usage);

                    if (usage > worstUsage) worstUsage = usage;
                    if (usage > 1.0f + 1e-4f) withinEllipse = false;
                    const float resultant = sqrtf(fxLim * fxLim + fyLim * fyLim);
                    if (resultant > limit * (1.0f + 1e-4f)) withinEllipse = false;
                }
            }
        }
    }
    check(withinEllipse,
          "combined-slip resultant stays inside the friction ellipse (worst usage %.4f)",
          (double)worstUsage);

    /* Each pure curve saturates at its friction peak: scanning slip, the peak |force|
     * approaches mu*Fz (the tire cannot exceed the road's grip) without overshooting. */
    bool latSaturates = true;
    bool longSaturates = true;
    for (size_t mi = 0; mi < sizeof(muSet) / sizeof(muSet[0]); mi++) {
        const float mu = muSet[mi];
        const float Fz = 4000.0f;
        const float limit = mu * Fz;
        float maxLat = 0.0f;
        float maxLong = 0.0f;
        for (int s = 0; s <= 40; s++) {
            const float slip = (float)s * 0.01f; /* 0 .. 0.40 rad / ratio */
            const float fy = fabsf(tire_lateral_force_n(slip, Fz, B, C, mu));
            const float fx = fabsf(tire_longitudinal_force_n(slip, Fz, B, C, mu));
            if (fy > maxLat) maxLat = fy;
            if (fx > maxLong) maxLong = fx;
        }
        /* The Pacejka peak is exactly mu*Fz (sin peaks at 1.0); allow a tiny tolerance. */
        if (maxLat < 0.95f * limit || maxLat > limit + 1.0f) latSaturates = false;
        if (maxLong < 0.95f * limit || maxLong > limit + 1.0f) longSaturates = false;
    }
    check(latSaturates, "lateral tire curve saturates near mu*Fz at its peak");
    check(longSaturates, "longitudinal tire curve saturates near mu*Fz at its peak");
}

/* Peak |camberThrustN| over a scripted skidpad run with static camber set to `camberRad` and
 * caster zeroed (to isolate static-camber thrust from caster-induced steering gain). */
static float camber_thrust_peak(float camberRad)
{
    const int idx = dev_scenario_find("skidpad");
    if (idx <= 0) return -1.0f;
    const DevScenario *sc = dev_scenario_at(idx);

    VehicleSpec spec;
    if (!car_roster_spec(0, &spec)) return -1.0f;
    spec.suspCamberFrontRad = camberRad;
    spec.suspCamberRearRad = camberRad;
    spec.suspCasterFrontRad = 0.0f;
    spec.suspCasterRearRad = 0.0f;

    Game *game = alloc_game();
    game_init(game);
    game_apply_spec(game, &spec);
    game->dev.scenario = idx;
    game->dev.scenarioRunning = true;
    game->dev.scenarioStartTick = game->sim.tick;

    float peak = 0.0f;
    for (int t = 0; t < sc->durationTicks; t++) {
        game_fixed_update(game, FIXED_DT_S);
        for (int w = 0; w < WHEEL_COUNT; w++) {
            const float a = fabsf(game->derived.camberThrustN[w]);
            if (a > peak) peak = a;
        }
    }

    free(game);
    return peak;
}

/* camber-invariants — defends issue #15: zero camber is neutral (the slip shift is then zero,
 * so camberThrustN is identically zero), nonzero camber produces real thrust, and the two
 * signs are symmetric. */
static void scenario_camber_invariants(void)
{
    const float zeroPeak = camber_thrust_peak(0.0f);
    const float posPeak = camber_thrust_peak(0.05f);
    const float negPeak = camber_thrust_peak(-0.05f);
    check(zeroPeak >= 0.0f && posPeak >= 0.0f && negPeak >= 0.0f,
          "camber-invariants ran the skidpad for all three camber settings");

    printf("    camber-invariants peak |camberThrustN|: 0-rad %.4f N, +0.05 %.4f N, -0.05 %.4f "
           "N\n",
           (double)zeroPeak, (double)posPeak, (double)negPeak);

    check(zeroPeak < 0.5f, "zero camber (caster zeroed) produces negligible thrust (%.4f N)",
          (double)zeroPeak);
    check(posPeak > zeroPeak, "positive camber produces more thrust than zero (%.4f > %.4f N)",
          (double)posPeak, (double)zeroPeak);
    check(negPeak > zeroPeak, "negative camber produces more thrust than zero (%.4f > %.4f N)",
          (double)negPeak, (double)zeroPeak);
    check_near((double)posPeak, (double)negPeak,
               (double)(fmaxf(posPeak, negPeak) * 0.25f + 0.5f),
               "camber thrust magnitude is symmetric across sign");
}

static const TestScenario kInvariantScenarios[] = {
    { "friction-ellipse",
      "Layer D: combined-slip resultant never exceeds mu*Fz; curves saturate at the peak",
      scenario_friction_ellipse },
    { "camber-invariants",
      "Layer D: zero camber is neutral; nonzero camber thrust is sign-symmetric (issue #15)",
      scenario_camber_invariants },
};

TestScenarioGroup test_invariant_scenarios(void)
{
    TestScenarioGroup group;
    group.items = kInvariantScenarios;
    group.count = sizeof(kInvariantScenarios) / sizeof(kInvariantScenarios[0]);
    return group;
}
