# Baseline Scenario Inventory (pre-Track-A)

Captured **2026-07-30** against commit `9d252d00ac19` on `main`.

This snapshot exists to prove that Track A made **no behavioural change** to the
existing 59 scenarios. After Track A ships, this file is immutable for the
remainder of the overhaul; new scenarios added by Tracks B/C/D add to it.

## Test runner binary

`build/tests/drifty_tests.exe` (Linux CI: `build/tests/drifty_tests`).
Built from `TEST_RUNNER_SRCS` plus `src/platform/timestep.c`, `GAME_SRCS`,
`SHARED_SRCS`. See `Makefile` lines 144–152.

## Scenario groups

| Group | Source file | Count |
|---|---:|---:|
| `core` | `tests/scenarios/core_tests.c` | 6 |
| `appearance` | `tests/scenarios/appearance_tests.c` | 2 |
| `physics` | `tests/scenarios/physics_tests.c` | 24 |
| `handling` | `tests/scenarios/handling_tests.c` | 18 |
| `gameplay` | `tests/scenarios/gameplay_tests.c` | 9 |
| **Total** | | **59** |

## Naming

Each scenario has:
- a stable, lower-kebab `name` (e.g. `skidpad`, `scoring-accumulation`)
- a one-line `description` shown in `--list`
- a static `(void)` fn referenced by the table

Names are matched by `--scenario NAME` in `tests/test_main.c`. Existing
invocations like `make scenario NAME=skidpad` rely on **exact name match**;
Track A adds `--filter PATTERN` (additive) without changing that contract.

## Source manifests (file SHA256)

```
325d40fda19341436ecf2323a0b287e4a8409e95bdc3ba20a9220bce1eec25e3  tests/scenarios/core_tests.c
d3adad19c309c624ed7fa70f5fcb5a08f41779bd1a140bcc7aa0fb08b563b750  tests/scenarios/handling_tests.c
214c19726412f31670a8218dbc93a89d9f4762ccd47100f4be542f6d70a82b80  tests/scenarios/physics_tests.c
baefa15e7f707b138f4af49fbaf6efa4629870b63a156038079767ebf3a61857  tests/scenarios/gameplay_tests.c
be2cc78b44b59a3ded5397753bfd2f55f87404e62e484eb2f5aac5b827b42deb  tests/scenarios/appearance_tests.c
```

## Known uncommitted worktree drift

Three files were already modified (from prior sessions) at the time of capture:

- `tests/scenarios/gameplay_tests.c` — scoring diagnostics tightened
- `tests/scenarios/handling_tests.c` — `locked-diff` scenario added
- `tests/scenarios/physics_tests.c` — `auto-trans` scenario added

These are **preserved as-is** for Track A so no diff churn is introduced.
