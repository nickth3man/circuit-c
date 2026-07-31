# Drifty — Testing overhaul: current state

## Status: locally implemented (NOT committed)

The items below are implemented in the working tree and pass `build.bat --tests`, but are
**not committed** to git. Nothing here is "shipped" until it is committed and pushed.

### Preflight
- [x] Open questions defaulted (plan §6)
- [x] Static baseline captured → `BASELINE_SCENARIOS.md`
- [~] ~~Vendored greatest v1.5.0~~ — **removed**: zero callers anywhere in `tests/` or `src/`.
      Confirmed dead by both grep and the codebase-memory graph (all functions `in_degree=0`).
- [~] ~~Vendored fff~~ — **removed**: zero `#include` of `fff.h` anywhere in the tree.
      The project's own `check()` harness remains the single supported assertion system.

### Track A — Foundation (locally implemented, verified by `build.bat --tests`)
- [x] **A1** — `check_run_invariants()` helper: `tests/support/test_harness.{h,c}` + `tests/scenarios/handling_tests.c`.
- [x] **A3** — `--junit [FILE]` flag: `tests/test_main.c`.
- [x] **A4** — `--filter PATTERN` flag: `tests/test_main.c`.

### Blocked (not yet implemented)
- [ ] **A2** — Multi-failure bundle (G2).
- [ ] **A5** — `scenario_input.h` consolidation.
- [ ] Tracks B–F: parameterised scenarios, fuzzing, mutation testing, coverage, persistence.

## Verification commands (work from cmd.exe or PowerShell; `build.bat` enters UCRT64)

```bash
build.bat --tests
./build/tests/drifty_tests.exe --list
./build/tests/drifty_tests.exe --filter accel
./build/tests/drifty_tests.exe --junit
./build/tests/drifty_tests.exe
```
