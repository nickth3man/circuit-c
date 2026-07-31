/*
 * test_harness.c — the tiny check framework, its counters, and the failure-bundle context.
 *
 * test_harness_clear_scenario() is what the runner calls between scenarios; it is the only
 * way to reset the first-failure text and the bundle context.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "test_harness.h"

#include "core/config.h"
#include "core/math_utils.h"

static int g_checks = 0;
static int g_failures = 0;
static bool g_verbose = false;

/*
 * Failure-bundle context.
 *
 * A scenario that has something reproducible to offer — a Game, a telemetry file, a seed —
 * registers it here. When the scenario ends with failures, the runner writes
 * artifacts/failure-<scenario>-<timestamp>/ from whatever was registered, so the failure
 * arrives with its input timeline and tunables attached instead of as a line of text.
 */
static char g_firstFailureText[512];
static const Game *g_bundleGame = NULL;
static char g_bundleTelemetryPath[256];
static bool g_bundleHasTelemetry = false;
static uint32_t g_bundleSeed = 0u;
static bool g_bundlesEnabled = true;

/* The path is copied, not referenced: the runner writes the bundle after the scenario
 * function has returned, when a caller's stack buffer no longer exists. */
void bundle_context(const Game *game, const char *telemetryPath, uint32_t seed)
{
    g_bundleGame = game;
    g_bundleHasTelemetry = (telemetryPath != NULL);
    if (g_bundleHasTelemetry) {
        snprintf(g_bundleTelemetryPath, sizeof(g_bundleTelemetryPath), "%s", telemetryPath);
    } else {
        g_bundleTelemetryPath[0] = '\0';
    }
    g_bundleSeed = seed;
}

void test_harness_clear_scenario(void)
{
    g_firstFailureText[0] = '\0';
    bundle_context(NULL, NULL, 0u);
}

void check(bool ok, const char *fmt, ...)
{
    va_list args;

    g_checks++;
    if (ok) {
        if (g_verbose) {
            printf("    ok   ");
            va_start(args, fmt);
            vprintf(fmt, args);
            va_end(args);
            printf("\n");
        }
        return;
    }

    g_failures++;

    /* Keep the first failure verbatim: later ones are usually consequences of it. */
    if (g_firstFailureText[0] == '\0') {
        va_start(args, fmt);
        vsnprintf(g_firstFailureText, sizeof(g_firstFailureText), fmt, args);
        va_end(args);
    }

    printf("    FAIL ");
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");
}

void check_near(double actual, double expected, double tolerance, const char *what)
{
    const double delta = fabs(actual - expected);
    check(delta <= tolerance, "%s (got %.9g, expected %.9g, |delta| %.3g > %.3g)", what, actual,
          expected, delta, tolerance);
}

/* Angular comparison that treats -PI and +PI as equal. */
void check_near_angle(float actual, float expected, float tolerance, const char *what)
{
    const float delta = fabsf(wrap_angle(actual - expected));
    check(delta <= tolerance, "%s (got %.9g rad, expected %.9g rad, |delta| %.3g)", what,
          (double)actual, (double)expected, (double)delta);
}

Game *alloc_game(void)
{
    Game *game = (Game *)calloc(1, sizeof(Game));
    if (game == NULL) {
        fprintf(stderr, "FATAL: could not allocate Game (%zu bytes)\n", sizeof(Game));
        exit(126);
    }
    return game;
}

void test_harness_set_verbose(bool verbose)
{
    g_verbose = verbose;
}

void test_harness_set_bundles_enabled(bool enabled)
{
    g_bundlesEnabled = enabled;
}

bool test_harness_bundles_enabled(void)
{
    return g_bundlesEnabled;
}

TestHarnessSnapshot test_harness_snapshot(void)
{
    TestHarnessSnapshot snapshot;
    snapshot.checks = g_checks;
    snapshot.failures = g_failures;
    snapshot.firstFailureText = g_firstFailureText;
    snapshot.bundleGame = g_bundleGame;
    snapshot.bundleTelemetryPath = g_bundleTelemetryPath;
    snapshot.bundleHasTelemetry = g_bundleHasTelemetry;
    snapshot.bundleSeed = g_bundleSeed;
    return snapshot;
}

void check_run_invariants(const Game *game, const char *name, bool allFinite,
                          float peakFrictionUsage, float peakSpeedMps)
{
    check(allFinite, "'%s' keeps every state variable finite", name);
    check(!game->dev.invariantFailed, "'%s' violates no invariant%s%s", name,
          game->dev.invariantFailed ? ": " : "",
          game->dev.invariantFailed ? game->dev.invariantText : "");
    check(peakFrictionUsage <= 1.0f + FRICTION_TOLERANCE,
          "'%s' never exceeds the friction budget (peak %.4f)", name,
          (double)peakFrictionUsage);
    check(peakSpeedMps <= MAX_SAFE_SPEED_MPS,
          "'%s' stays below MAX_SAFE_SPEED_MPS (peak %.2f m/s)", name, (double)peakSpeedMps);
}
