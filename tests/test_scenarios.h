/*
 * test_scenarios.h — the registry contract between the runner and the scenario groups.
 *
 * Each group returns a span over its own file-static const table, so nothing is allocated,
 * nothing registers itself at load time, and the order the runner reports is the order written
 * in the source rather than whatever the linker chose.
 */
#ifndef DRIFTY_TEST_SCENARIOS_H
#define DRIFTY_TEST_SCENARIOS_H

#include <stddef.h>

/* Where every scenario that writes telemetry writes it. */
#define TELEMETRY_DIR "artifacts/telemetry"

typedef void (*TestScenarioFn)(void);

typedef struct {
    const char *name;
    const char *description;
    TestScenarioFn run;
} TestScenario;

typedef struct {
    const TestScenario *items;
    size_t count;
} TestScenarioGroup;

TestScenarioGroup test_core_scenarios(void);
TestScenarioGroup test_appearance_scenarios(void);
TestScenarioGroup test_physics_scenarios(void);
TestScenarioGroup test_handling_scenarios(void);
TestScenarioGroup test_gameplay_scenarios(void);

/* Releases the Game the last scripted handling run kept alive for failure bundles. */
void test_handling_cleanup(void);

#endif /* DRIFTY_TEST_SCENARIOS_H */
