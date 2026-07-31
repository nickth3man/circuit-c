/*
 * test_harness.h — the check framework the headless scenarios are written against.
 *
 * Counters, verbosity, first-failure text and failure-bundle context live here, behind
 * functions rather than writable globals: only the runner is allowed to read the accumulated
 * state, and it does so through one snapshot so a scenario cannot reach in and edit a total.
 */
#ifndef DRIFTY_TEST_HARNESS_H
#define DRIFTY_TEST_HARNESS_H

#include <stdbool.h>
#include <stdint.h>

#include "game/game.h"

/*
 * The whole of the harness's accumulated state, as of one instant.
 *
 * `firstFailureText` points at the harness's fixed buffer and stays valid until the next
 * test_harness_clear_scenario(). `bundleTelemetryPath` points at a copy the harness owns, so
 * it outlives the caller's stack buffer. `bundleGame` is BORROWED, never copied: it is only
 * valid until the owning scenario releases its Game.
 */
typedef struct {
    int checks;
    int failures;
    const char *firstFailureText;
    const Game *bundleGame;
    const char *bundleTelemetryPath;
    bool bundleHasTelemetry;
    uint32_t bundleSeed;
} TestHarnessSnapshot;

void test_harness_set_verbose(bool verbose);
void test_harness_set_bundles_enabled(bool enabled);
bool test_harness_bundles_enabled(void);
void test_harness_clear_scenario(void);
TestHarnessSnapshot test_harness_snapshot(void);

void bundle_context(const Game *game, const char *telemetryPath, uint32_t seed);
void check(bool ok, const char *fmt, ...);
void check_near(double actual, double expected, double tolerance, const char *what);
void check_near_angle(float actual, float expected, float tolerance, const char *what);
Game *alloc_game(void);

/*
 * check_run_invariants — the four structural checks every scripted scenario runs.
 *
 * Centralises: allFinite, !invariantFailed, friction budget (1+FRICTION_TOLERANCE),
 * and MAX_SAFE_SPEED_MPS. The caller computes the peaks during its loop so the
 * helper does not need tick-level state or a setup/teardown API.
 */
void check_run_invariants(const Game *game, const char *name, bool allFinite,
                          float peakFrictionUsage, float peakSpeedMps);

#endif /* DRIFTY_TEST_HARNESS_H */
