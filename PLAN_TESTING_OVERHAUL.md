# Drifty — Scenarios & Testing Overhaul Plan

**Status:** Research-first proposal, not yet implemented
**Scope:** Headless test infrastructure for `build/tests/drifty_tests.exe`
**Constraint envelope:** C-first, no new MSYS2 packages, vendored into `tests/support/`, headless only (no raylib call, no GPU pixel comparisons in CI), must not break the existing hot-reload, sanitizer, coverage, or fuzz builds.

---

## 1. Baseline — what already exists

### 1.1 Scenario table

Five groups declared in `tests/test_main.c`; each returns a static `const TestScenario[]` span. Runner iterates declaration order, prints `[ name ] description → N checks, M failed`, and on failure writes a `failure_bundle` to `artifacts/failure-<scenario>-<timestamp>/`.

| Group | Scenarios | What it covers |
|---|---|---|
| `core` | `math`, `units`, `timestep`, `oneshot`, `replay`, `renderscale` | Pure functions, fixed-timestep accumulator, replay round-trip, pixel/meter independence |
| `appearance` | `car-visual`, `corpus` | Grammar purity/totality, per-corpus-pair distinctness, sweep-key sensitivity |
| `physics` | `telemetry`, `vehicle`, `tire`, `drivetrain`, `accel-filter`, `load-transfer`, `resistance`, `rest`, `launch-stop`, `coast-down`, `brake-corner`, `power-oversteer`, `handbrake`, `low-speed`, `reverse`, `steer-sign`, `lever-arm`, `integration`, `fixed-rate`, `params`, `presets`, `auto-trans`, `dev-state`, `devreplay` | Component-level physics + the dev registry |
| `handling` | `accel-load`, `brake-load`, `coast-down-run`, `skidpad`, `skidpad-sweep`, `step-steer`, `transition`, `lift-off`, `catchable-drift`, `lat-load-transfer`, `surface-asymmetry`, `open-diff`, `lsd-diff`, `locked-diff`, `ackermann`, `load-sensitivity`, `tire-relaxation`, `steer-speed-feel` | Scripted maneuvers graded on derived metrics |
| `gameplay` | `track-surface`, `collision-barrier`, `scoring-accumulation`, `scoring-rejection`, `scoring-determinism`, `highscore-persistence`, `checkpoint-lap`, `particle-pool`, `state-machine` | Track, scoring, persistence, particles, state machine |

Total: **~58 scenarios**. Each is a C function returning void, registered by hand in a `static const TestScenario[]` table.

### 1.2 Tooling already wired into CI

| Tool | Where | What it does |
|---|---|---|
| `tests/support/test_harness.c` | `TEST_RUNNER_SRCS` | `check()`, `check_near()`, `check_near_angle()`, `bundle_context()`, snapshot of pass/fail counts |
| `tests/support/simulation_fixture.c` | `TEST_RUNNER_SRCS` | `test_telemetry_row_from_game()` + the `simulation_fixture` Game constructor |
| `tests/support/appearance_metrics.c` | `TEST_RUNNER_SRCS` | Per-axis signature L∞, L2, pixel-diff vs shared canvas |
| `tests/support/car_sheet.c` | `TEST_RUNNER_SRCS` | `car_sheet_write()`, `car_sheet_write_cards()`, baked PNG/label map writer |
| `tests/hotreload/hotreload_harness.c` | `HOTRELOAD_HARNESS_SRCS` | Windowless DLL load/copy/atomic-rename smoke |
| `fuzz/fuzz_tire.c`, `fuzz_profile.c`, `fuzz_replay.c` | `FUZZ_SUPPORT_SRCS` | libFuzzer entry points, no custom mutator yet |
| `tools/telemetry/compare_telemetry.py` | invoked by `make regression` | Tolerance-aware CSV diff (exact columns, per-column tolerances, derived-metric tolerances) |
| `tools/telemetry/make_report.py` | invoked by `make report` | Self-contained HTML with inline SVG, no external deps |
| `tools/telemetry/summarize_run.py`, `plot_telemetry.py` | developer | JSON metric dump, per-chart SVGs |
| `tools/visual/tests/{grammar,capture}.spec.js` | invoked by `make visual-diagnose` | Playwright assertions on the corpus grammar (already in `tools/visual/`) |
| `Makefile` | `make ci` | format-check → lint → analyze → test-physics → regression → sanitize → coverage |

### 1.3 Gaps the plan must close

| # | Gap | Evidence |
|---|---|---|
| G1 | **No parameterised scenario runs.** Every scenario is a single hard-coded maneuver; no sweep harness exists for "step-steer at amplitudes {0.05, 0.1, 0.2} rad". The one existing sweep (`skidpad-sweep`) is a bespoke scenario. | grep of `TestScenario` tables — no `for (amplitude ...)` anywhere except `skidpad-sweep`. |
| G2 | **No "first-failure mode" for handling scenarios.** The runner stops at the first failed check and the bundle records only that. Multi-failure scenarios (`--no-bundle`) print but lose artifact coverage. | `failure_bundle.h` doc: "records only the first failed check." |
| G3 | **No libFuzzer custom mutator.** The three fuzz targets consume raw bytes and call their own deserialisers; `ScriptFrame` is a known structured input that has never been structure-aware fuzzed. | `fuzz/fuzz_*.c` — no `LLVMFuzzerCustomMutator` in the tree. |
| G4 | **Telemetry is the only oracle for physics.** Invariant-based assertions are local to each scenario; there is no shared `assert_invariants(game, ctx)` that all scenarios call on every tick to catch "the simulation didn't NaN, but it produced garbage". | grep for `invariant` in `tests/support/` — empty. |
| G5 | **No mutation testing of physics invariants.** A change that *deliberately* makes an invariant looser passes every test. There is no "did any test actually depend on the bound it claims to depend on" check. | `fuzz/` exists; no `mull`/`mull-c` or equivalent. |
| G6 | **No ISO 3888 / NHTSA fishhook / J-turn standardized maneuvers.** The handling group is bespoke; the only reference standards implicit in the code are "skidpad" and "step-steer" by name. | grep `ISO 3888`, `fishhook`, `J-turn`, `moose` — none in tree. |
| G7 | **Replay buffer is the only persistence target.** No "save/load every scenario's first failing tick" replay corpus; the failure bundle reuses the live game. A regression can't be reproduced from the bundle alone unless a developer re-runs the binary. | `failure_bundle.h` — replays a live `Game` snapshot, not a re-loadable record. |
| G8 | **No shared unit-test harness for pure functions.** `math`, `units`, `timestep` are scenario functions in `core_tests.c`; they print a check count and exit. A library-style `TEST_F(scenario)` with per-test skip/filter/junit output is not present. | All test entry points are void-fn, no test ID metadata beyond name. |
| G9 | **No coverage diff between baseline and PR.** `make coverage` writes text/HTML/Cobertura for the current tree; it does not compare the PR's branch coverage to the merge base. | `Makefile` — no `--diff-base` invocation. |
| G10 | **No machine-readable CI summary.** The runner prints `PASS`/`FAIL` and a failure-bundle path. A JUnit-XML export would let a downstream dashboard grade the run. | `test_main.c` — no `--junit` mode. |

### 1.4 What the project already rejected (don't re-propose)

- **C++ test frameworks** (Catch2, doctest, GoogleTest). Project is C11 with a `gcc` UCRT64 toolchain. CI explicitly runs cppcheck + clang --analyze, not C++ compilers.
- **GPU pixel comparisons in CI.** Already documented in `tools/visual/`: Playwright is a *bounded* diagnostic, not a regression gate. `mk visual-diagnose` is wrapped in `|| true` for that reason.
- **MSYS2 packages beyond the current `setup_windows.ps1` list.** Adding `valgrind`, `llvm`, `mull`, `cmake`, `python-pytest` would require modifying the bootstrap script and CI matrix.
- **Replacing the existing scenario runner with a third-party one.** Every test call already routes through `check()` + `bundle_context()`; ripping them out would orphan 58 scenarios.

---

## 2. Prior Art
> **UPDATE (audit remediation F4):** greatest (P2) and fff (P4) were vendored into
> `tests/support/` but **evaluated and removed** — neither had a single `#include` or caller
> anywhere in `tests/` or `src/` (confirmed by grep and the codebase-memory graph). The
> project's own `check()` harness remains the single supported assertion system. P1 (Unity)
> was never vendored. The entries below are retained as historical design context only.


| # | Source | What we adopt | Fit |
|---|---|---|---|
| P1 | **ThrowTheSwitch/Unity** — https://github.com/throwtheswitch/unity — single-file C test framework, MIT, no deps, used by ESP32, mbed, etc. | Adopt as the *pure-function* harness underneath existing scenarios. Vendored as `tests/support/unity/{unity.c,unity.h,unity_internals.h}`. Adds `TEST_ASSERT_*` macros, per-test setup/teardown, and `--list-tests` mode that the current `check()` cannot express. | adopt |
| P2 | **silentbicycle/greatest** — https://github.com/silentbicycle/greatest — 1 file, <1k LOC, no malloc. | Adopt *as an alternative* if Unity's footprint is too large for one support TU. Greatest is ~800 LOC and would fit as a single .c/.h pair. Decision deferred until P1 footprint is measured. | adopt (alt) |
| P3 | **libFuzzer + `LLVMFuzzerCustomMutator`** — https://llvm.org/docs/LibFuzzer.html#custom-mutator, https://github.com/google/fuzzing/blob/master/docs/structure-aware-fuzzing.md | Extend the existing `fuzz/` targets with a custom mutator that flips one `ScriptFrame` field at a time. The struct is already flat (`ScriptFrame` in `tests/support/scenario_shared.h`), so a hand-rolled mutator is ~30 LOC. Plugs into the existing `make fuzz` target — no new Makefile rule. | extend |
| P4 | **fff (Fake Function Framework)** — https://github.com/meekrosoft/fff — single header. | Adopt for the gameplay scenarios that today reach into `physics_state_is_valid` indirectly. Lets a `collision-barrier` scenario inject a "sensor says contact at t=42" without rebuilding the whole physics state. Use sparingly — most scenarios should still exercise real physics. | extend |
| P5 | **Awesome-C** curated list — https://github.com/oz123/awesome-c — and **r-lyeh/single_file_libs** — https://github.com/r-lyeh/single_file_libs. | Cite as the index for vetting future test utilities. No code adopted. | compose |
| P6 | **GAFFER ON GAMES — Floating Point Determinism** — https://gafferongames.com/post/floating_point_determinism/ | Cite as the rationale for *why* the determinism scenarios (`replay`, `telemetry`, `scoring-determinism`) are non-negotiable. The repo's choice of `-O2` for tests + `-O0` for the module is *consistent* with this guidance. | compose |
| P7 | **Carla scenario_runner** — https://github.com/carla-simulator/scenario_runner — open-source scenario definition & execution engine for ADAS testing. | Reference architecture for *parameterised scenario definitions*. The DSL their scenarios use is overkill for Drifty, but their pattern of "one scenario XML/JSON → N parameter sets → N runs" is exactly G1. Adopt the *shape* (a `SweepFrame` next to `ScriptFrame`), not their toolchain. | compose |
| P8 | **ISO 3888-1 / ISO 3888-2** double lane-change — https://www.iso.org/standard/57253.html, **NHTSA Fishhook** — https://www.nhtsa.gov/document/1light-vehicle-dynamic-rollover-propensity-phases-iv-v-and-vi, **ISO 7975 braking in a turn**. | Reference for the missing standardized maneuvers (G6). Drifty is 2D top-down, so full 3D versions are out of scope, but a *projected* version of each is well-defined: lane change = a sequence of step-steers; fishhook = ramp-steer to peak then return; braking-in-a-turn = `brake-corner` already exists but should grow a parameterised sweep. | compose |
| P9 | **Hypothesis (Python)** — https://hypothesis.readthedocs.io — property-based testing framework, shrinks failing inputs. | Reference for the *shape* of property-based scenarios, but C++-only without a port. The port we can do in C is much smaller: a `for_seed(uint32_t s)` helper that generates valid `ScriptFrame` arrays for an arbitrary deterministic seed, then asserts invariants. Adopt the *idea* (random valid inputs + invariant), not the library. | build |
| P10 | **Glenn Fiedler "Fix Your Timestep"** — https://gafferongames.com/post/fix_your_timestep/ | Already implicit in the existing `timestep` and `fixed-rate` scenarios. Cite as the design rationale for keeping the accumulator / substep cap as first-class testable units. | compose |
| P11 | **gcovr** — already in `setup_windows.ps1`. | Extend with `--diff-base` to compare PR branch coverage against the merge base. The diff is a single `subprocess` call; no new code. | extend |
| P12 | **CARLA Real Traffic Scenarios** — https://ml4ad.github.io/files/papers2020/CARLA%20Real%20Traffic%20Scenarios%20%E2%80%93%20a%20novel%20training%20ground%20and%20benchmark%20for%20driver%20systems.pdf | Cite for the *parameter-sweep* matrix design: actor × maneuver × environment × duration. Drifty's `ScriptFrame` already encodes the first three; we add `SweepFrame` to layer the fourth. | compose |

**Outcome matrix**

| Verdict | Items |
|---|---|
| adopt | P1, P2 (alt) |
| extend | P3, P4, P11 |
| compose | P5, P6, P7, P8, P10, P12 |
| build | P9 (port the *idea*, not the lib) |

**Out of scope after evaluation:** ANTLR/Grammarinator, libprotobuf-mutator, Catch2, GoogleTest, GoogleMock, valgrind, CMake, pytest — all rejected for the reasons in §1.4.

---

## 3. The plan

Five tracks, ordered so each track's deliverables feed the next. Every track fits inside the existing build, runs headless, and is gated on green CI.

### Track A — Foundation (no new files, no new Makefile entries)

The lowest-risk improvements. Pure additions to the existing `TestScenario` table and helper consolidation.

| ID | Change | Files | Acceptance |
|---|---|---|---|
| A1 | Add a `assert_invariants(Game *g, TickRange r, const char *scenario)` helper in `tests/support/test_harness.h` + `.c`. Centralises the four checks every handling scenario already duplicates: `isfinite(state)`, `peak friction usage ≤ 1+ε`, `peak speed ≤ MAX_SAFE_SPEED_MPS`, `peak speed ≥ 0`, plus the existing `stateChecksum` mismatch detection. | `tests/support/test_harness.{h,c}` | Every `handling_tests.c` scenario that today hand-codes these four calls reduces to a single `assert_invariants(game, full, name)`; net line count drops; failure text still names the offending check. |
| A2 | Add `tests/scenarios/handling_tests.c::scenario_<name>_bundle` companion: re-runs a failing scenario with `--no-bundle` and writes *all* failed checks (not just the first) into `summary.json`. The current `--no-bundle` path loses artifact coverage (G2). | `tests/scenarios/handling_tests.c`, `src/dev/failure_bundle.{h,c}` (extend with optional second-failure pass) | A scenario that fails 3 checks in one run drops a bundle whose `summary.json` lists all 3, not 1. |
| A3 | Add a `--junit <path>` mode to `tests/test_main.c` that writes a JUnit-XML report of pass/fail/skip per scenario, plus a `time` attribute per check. No new dependency. | `tests/test_main.c` | `drifty_tests --junit artifacts/junit.xml` produces a file parseable by GitHub Actions test reporting and the project README's CI badge. |
| A4 | Add `--filter PATTERN` to `test_main.c` so a developer can run `drifty_tests --filter drift` to exercise only the drift-related scenarios. Pattern matches the scenario `name` field. | `tests/test_main.c` | `drifty_tests --filter accel` runs `accel-filter`, `accel-load`, and nothing else. |
| A5 | Consolidate the four `script_build`/`run_recording`/`run_playback` helpers in `tests/scenarios/core_tests.c` and the `set_vehicle_rolling_speed` / `set_rolling_wheels` helpers in `physics_tests.c` into a single `tests/support/scenario_input.h` API. Reduces cross-file duplication; lets the new fuzz targets (Track C) reuse the same deterministic timeline. | `tests/support/scenario_input.h` (new), edits to `core_tests.c` + `physics_tests.c` | Both files call `scenario_input_init(seed)`, `scenario_input_step(...)`, etc.; identical checksum between callers on the same seed. |

**Risk / verification:** after Track A, `make test-physics` and `make regression` produce identical telemetry hashes to the current `main`. No gameplay change.

### Track B — Parameterised scenario engine (closes G1, G6)

The most impactful track. Replaces one-off `for (int i=0; i<4; i++)` loops with a declared sweep, and adds the three standard maneuvers the handling group is missing.

| ID | Change | Files | Acceptance |
|---|---|---|---|
| B1 | New `tests/support/scenario_sweep.h` API: `scenario_sweep_run(name, params, kCount, body)` where `params` is a `const SweepParam[]` of `{name, value}` tuples and `body` is a `void(Game *g, const SweepCtx *ctx)` callback. Each invocation registers as its own named scenario (`name.0`, `name.1`, …) so the existing runner, telemetry, and baseline machinery all work unchanged. | `tests/support/scenario_sweep.{h,c}` | `skidpad-sweep` collapses from one bespoke scenario to four `skidpad.s0`…`skidpad.s3` entries; existing baseline `tests/baselines/scenario_skidpad-sweep.csv` regenerates identically (or the migration is documented in `mk baselines`). |
| B2 | Add three standard maneuvers (G6), each as a `SweepParam[]` driver: **Lane change** (ISO 3888-1 projected — step-steer to +0.1 rad, hold 0.5 s, return; then −0.1 rad, hold, return). **Fishhook** (NHTSA: ramp-steer to +0.2 rad over 0.3 s, hold 0.5 s, return; mirror). **Brake in turn** (extend the existing `brake-corner` with a sweep of brake pressure at three cornering radii). | `tests/scenarios/handling_tests.c` (new scenarios: `lane-change`, `fishhook`, `brake-turn-sweep`) | Each new scenario fails CI today (intentional — closes the missing-standard gap), passes once the model is tuned, and the report HTML includes the standard's name in the chart title. |
| B3 | Add `scenario_lap_average` to `gameplay_tests.c`: drive a full track loop with `replay` recording, replay 10 times, and assert the per-lap time and total energy stay within a tight tolerance. This is the *one* end-to-end "the simulation actually drives around a track" scenario. | `tests/scenarios/gameplay_tests.c` | New scenario passes; baseline `tests/baselines/scenario_lap_average.csv` recorded. |
| B4 | Reorganise the `kHandlingScenarios` table so derived/integration tests (lift-off, catchable-drift, transition) come *after* the unit-style tests (load-transfer, surface-asymmetry) and are marked with a `requires = { "load-transfer", "surface-asymmetry" }` field. The runner runs the depends-on set first. | `tests/scenarios/handling_tests.c`, `tests/test_scenarios.h` (struct extension), `tests/test_main.c` (topological sort) | Reordering does not break existing baselines; a developer who breaks `load-transfer` sees the dependent scenario skipped, not falsely failed. |
| B5 | Parameterise `scoring-accumulation` with `SweepParam[]` over three drift durations (3 s, 6 s, 12 s) and three entry speeds (40, 80, 120 km/h). Asserts the combo multiplier reaches the documented `MAX_COMBO` within the long-duration runs. | `tests/scenarios/gameplay_tests.c` | New named scenarios `scoring-accum.d3`, `scoring-accum.d6`, `scoring-accum.d12` etc. all pass on the current tree. |

**Risk / verification:** Track B is the largest visible change. Gates:
1. `make test` still passes after B1 (no behavioural change, only reorganisation).
2. After B2 lands, `make regression` shows the new baselines are physically reasonable (compare against published ISO/NHTSA envelopes in the HTML report).
3. `make coverage` shows every new scenario exercises at least one line in `src/physics/`.

### Track C — Fuzzing (closes G3)

Custom `LLVMFuzzerCustomMutator` for `ScriptFrame`, integrated into the existing `fuzz/` directory and the existing `make fuzz` target. No new build target; the existing `FUZZ_SUPPORT_SRCS` group already covers the right sources.

| ID | Change | Files | Acceptance |
|---|---|---|---|
| C1 | Extract the `ScriptFrame` layout from `tests/support/scenario_shared.h` into `tests/support/script_frame.h` so `fuzz/` doesn't need to pull in scenario headers. | `tests/support/script_frame.h` (new), `tests/support/scenario_shared.h` (re-export) | No source file outside `tests/` includes `scenario_shared.h`; `script_frame.h` compiles in isolation under `-DDRIFTY_HEADLESS`. |
| C2 | New `fuzz/fuzz_script_timeline.c` target: `LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)` deserialises a `ScriptFrame[]` using the existing `apply_live_input` path, runs `game_fixed_update` for the implied duration, and asserts the four Track-A invariants. `LLVMFuzzerCustomMutator` flips one field of one frame (steer, throttle, brake, gear, handbrake) at a time using the structured-mutator pattern from libFuzzer docs. | `fuzz/fuzz_script_timeline.c` (new), `Makefile` (add to `FUZZ_SUPPORT_SRCS` or to a new `FUZZ_SRCS` group), `fuzz/fuzz_tire.c` / `fuzz_profile.c` / `fuzz_replay.c` (extend with the same custom-mutator template). | `make fuzz` runs the new target for 30 s and exercises at least 5 distinct paths in `src/physics/`. Existing `fuzz_tire`, `fuzz_profile`, `fuzz_replay` continue to pass. |
| C3 | New `fuzz/fuzz_grammar.c` target: structure-aware mutator over `VehicleSpec` rather than `ScriptFrame`. Mutations stay within the registry ranges (`dev_param_min/max`). | `fuzz/fuzz_grammar.c` (new), reuses C2's mutator template. | A 30-s run finds at least one "failing invariant" path — the whole point of structure-aware fuzzing is to surface inputs the random-byte path misses. |
| C4 | Wire a `--fuzz <name> [--seconds N]` mode into `tests/test_main.c` so a developer can run `drifty_tests --fuzz script_timeline --seconds 10` against the same harness the CI uses, without needing clang's libFuzzer driver on their dev box. (libFuzzer itself is still clang-only.) | `tests/test_main.c` (new op), `tests/support/scenario_input.h` (from A5). | Non-fuzz scenarios remain available alongside the fuzz op; the fuzz op never affects their ordering. |

**Risk / verification:**
- `make fuzz` finds at least one new invariant breach on a clean tree within 60 s of run time (recorded as a known seed in `fuzz/corpus/`). If it doesn't, the fuzzer is structurally blind to the interesting paths and we know to extend it.
- ASan/UBSan remains green; no leak in `fuzz_*` drivers.

### Track D — Mutation testing (closes G5)

Lightweight, build-only, no new packages. C++ mutation tools (Mull, PIT) need clang++ and a C++ build of the SUT. For a C project, a hand-rolled "flip one operator / boundary / branch" driver is the right shape.

| ID | Change | Files | Acceptance |
|---|---|---|---|
| D1 | New `tools/mutate/` Python script (3 files, stdlib only — uses `clang -E` and the existing `make` artifacts to identify syntactically valid mutation sites). Mutates the C source, rebuilds a single TU, runs `drifty_tests --filter <subset>`, records pass/fail. The script follows the patterns of `cosmic-rays` and `mull-c` (lightweight C mutation testing) but is C-only and reuses the existing build graph. | `tools/mutate/{mutate.py,analyze.py,README.md}` | `python tools/mutate/mutate.py --target src/physics/tire.c --scenarios tire,brake-corner,power-oversteer` runs in <2 minutes, reports a mutation score, and lists the 3 survivors (mutations that didn't cause any test to fail) for human review. |
| D2 | Document the mutation-score target (≥ 80% on `src/physics/`) in `AGENTS.md` next to the existing accuracy contract. A score below threshold is a CI failure. | `AGENTS.md` (single new section) | A failing mutation score is reported by `make mutate` with a clear pointer to the surviving mutants. |

**Risk / verification:**
- Mutation score is computed in CI, not locally, because it takes ~10 minutes per PR.
- The first run will *find* survivors — that's the point. Each PR is expected to drive the score up until it plateaus.

### Track E — Coverage regression (closes G9)

| ID | Change | Files | Acceptance |
|---|---|---|---|
| E1 | Extend `make coverage` to also emit `artifacts/coverage-diff.txt` comparing PR branch coverage against `origin/main` via `gcovr --diff-base`. The diff is plain text so a reviewer can read it in a PR comment. | `Makefile` (new target `coverage-diff`), `tools/coverage/diff.py` (~50 LOC, stdlib + existing `gcovr` invocation) | `make coverage-diff` exits 0 when PR coverage ≥ base, exits 1 otherwise with a per-file table. |
| E2 | Add a `--coverage-gate MIN_PCT` flag to `drifty_tests` itself: the runner reads `artifacts/coverage.json` (written by gcovr in E1) and fails if any `src/*.c` file's branch coverage drops below the threshold. | `tests/test_main.c` (new op), `Makefile` | `drifty_tests --coverage-gate 60` is invoked by `make ci`; a coverage drop in any file fails the gate. |

**Risk / verification:** coverage gate threshold is set deliberately low on first run (e.g. 30%) and raised as the suite grows; never retroactively lowered to make a PR green (same rule as the existing baselines).

### Track F — Persistence and reproduction (closes G7)

| ID | Change | Files | Acceptance |
|---|---|---|---|
| F1 | Extend `failure_bundle` to record the *seeded* random state and the entire input timeline (already half-done: replay buffer is included; add the RNG state). | `src/dev/failure_bundle.{h,c}` | A bundle dropped by `--seed 42` reproduces the same `stateChecksum` when the developer runs `drifty_tests --replay-bundle artifacts/failure-...`. |
| F2 | New `drifty_tests --replay-bundle <dir>` mode: reads the bundle, replays the input, asserts the same `stateChecksum`, and prints PASS/FAIL. | `tests/test_main.c` (new op) | A known-bad bundle on `main` fails with the same offending check text; a known-good bundle passes. |

**Risk / verification:** the bundle remains self-contained (no raylib, no windowing). Same compile manifest as the rest of the runner.

---

## 4. Sequencing and CI gates

| Week | Tracks | New commands | CI delta |
|---|---|---|---|
| 1 | A1–A5 | `drifty_tests --junit <path>`, `drifty_tests --filter PATTERN` | None — purely additive |
| 2 | B1, B4 | `drifty_tests --list` shows the new `skidpad.s0…s3` etc. | None — existing baselines unchanged |
| 3 | B2, B3, B5 | New `lane-change`, `fishhook`, `brake-turn-sweep`, `lap-average` scenarios | New baselines recorded; `make regression` covers them |
| 4 | C1–C4 | `make fuzz` exercises the new targets; `drifty_tests --fuzz` is a developer convenience | Fuzz job added to CI matrix (clang-only, gated on tool availability) |
| 5 | D1–D2, E1, F1–F2 | `make mutate`, `make coverage-diff`, `drifty_tests --replay-bundle` | Mutation + coverage-diff jobs added to CI |
| 6 | stabilisation | threshold tuning, doc updates to `AGENTS.md` and `docs/DEVTOOLS.md` | Final coverage gate percentage locked |

**No track ever deletes a scenario.** New scenarios are added next to existing ones; old ones are deprecated only after the replacement has shipped and been green for one release.

**No track modifies `Makefile`'s source-manifest semantics** (`TEST_RUNNER_SRCS`, `FUZZ_SUPPORT_SRCS`, etc.). New `tests/support/*.c` files are added to `TEST_RUNNER_SRCS`; new `fuzz/*.c` files are added to `FUZZ_SUPPORT_SRCS` (or a new `FUZZ_SRCS` group that depends on `FUZZ_SUPPORT_SRCS`).

**No track requires a new MSYS2 package.** ~~Unity, greatest, fff are vendored.~~ greatest and fff were removed (zero callers); the existing `check()` harness suffices. gcovr, clang, cppcheck are already in `setup_windows.ps1`.

---

## 6. Resolved defaults (applied during execution)

The five open questions are settled with the following defaults. Any change to these
requires a new ADR-style note appended below.

| # | Question | Default | Rationale |
|---|---|---|---|
| 1 | Mutation + coverage thresholds | **Mutation ≥ 75% on `src/physics/`**, **branch coverage ≥ 40% on the whole tree** | Below the plan's preferred 80/60 — first-run scores will be low; the floor is ratchet-only. Same rule as `mk baselines`: never lower to make a PR green. |
| 2 | Unity vs Greatest | ~~**Greatest** (`tests/support/greatest/greatest.h`)~~ **Neither** — both evaluated and removed (zero callers); `check()` harness is the single system. | The original rationale (smaller surface area) was sound, but in practice no scenario needed a second framework. |
| 3 | JUnit default path | **`artifacts/junit.xml` only on `--junit <path>` or `--junit`** (no implicit write) | Avoids the runner silently writing files in local dev. Path defaults to `artifacts/junit.xml` only when `--junit` is passed without an argument. |
| 4 | ISO 3888 / NHTSA scope | **Steer-only, no `body_y`** | Drifty is 2D top-down; lateral excursion is the *result* of the maneuver, not an input. The scenarios assert peak yaw rate, peak slip angle, and trajectory deviation against an analytical envelope (computed from steer input + speed + wheelbase). |
| 5 | Coverage gate location | **Every PR** (cheap: gcovr --diff-base is one subprocess, no test rerun) | The repo's CI already runs `make coverage` in `make ci`; adding a diff step is a few-hundred-millisecond overhead. Every-PR is the only defensible answer for a regression contract. |

## 7. Execution constraints discovered at run time

- **No UCRT64 toolchain on the active shell.** `command -v gcc make clang cppcheck gcovr` are all empty on this shell. The implementation cannot be smoke-tested by `make` here. Every code change ships with the *exact* `bash` invocation the user must run on their MSYS2 UCRT64 shell (collected in §8) to verify.
- **Working tree carries 3 modified test files from prior sessions** (`gameplay_tests.c`, `handling_tests.c`, `physics_tests.c`). Each is preserved; new scenarios are appended next to existing ones.
- ~~**Third-party vendored files** (greatest, fff, android-fff)~~ — **removed**: neither had a single caller. See the F4 update at the top of §2.
- **No plan delivery without a verification recipe.** §8 lists, per track, the build + run commands and the expected pass/fail criteria.

---

## 8. Verification recipes (run on the user's UCRT64 shell)

These are the exact commands required to green-light each track. The runner is `build/tests/drifty_tests.exe` produced by `build.bat --tests` (Windows) or `make tests` (Linux CI). All scenarios reachable via `--scenario NAME` or `--list`.

```bash
# Build prerequisite
build.bat --tests          # Windows; or `make tests` from Linux CI

# Track A
./build/tests/drifty_tests.exe --list                                  # smoke: 58+ entries
./build/tests/drifty_tests.exe --filter accel                           # 2 entries expected: accel-filter, accel-load
./build/tests/drifty_tests.exe --junit artifacts/junit.xml              # XML exists, parses
./build/tests/drifty_tests.exe --scenario handling-cleanup              # pre-existing scenarios still green

# Track B
./build/tests/drifty_tests.exe --list | grep -E "lane-change|fishhook|brake-turn-sweep|lap-average"
./build/tests/drifty_tests.exe --scenario skidpad                       # baseline parity
./build/tests/drifty_tests.exe --scenario scoring-accumulation          # passing

# Track C (clang-only)
make fuzz                                                                # existing + new fuzz targets

# Track D
python tools/mutate/mutate.py --target src/physics/tire.c --scenarios tire,brake-corner,power-oversteer

# Track E
make coverage && make coverage-diff                                       # produces coverage-diff.txt

# Track F
./build/tests/drifty_tests.exe --scenario launch-stop                   # produce a real failure bundle
ls artifacts/failure-launch-stop-*/
./build/tests/drifty_tests.exe --replay-bundle artifacts/failure-launch-stop-*   # same checksum
```
