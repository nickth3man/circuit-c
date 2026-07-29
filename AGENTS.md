# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

The interactive game and DLL hot reload are Windows-only and use MSYS2 UCRT64. The Makefile
supports headless scenarios, corpus tools, telemetry, sanitizers, coverage, and fuzzing on Linux
and macOS; hosted CI validates the Linux path, not macOS.

## Project Structure

| Document | What it is |
|---|---|
| [docs/SOURCES.md](docs/SOURCES.md) | Technical reference index. |
| [docs/DEVTOOLS.md](docs/DEVTOOLS.md) | Physics Lab, replay inspector, telemetry, corpus, gallery, and command reference. |
| [docs/CI.md](docs/CI.md) | Hosted workflows, required checks, and gate rationale. |
| [docs/CAR_VISUAL.md](docs/CAR_VISUAL.md) | Authoritative vehicle-appearance contract. |
| [docs/generated/CORPUS.md](docs/generated/CORPUS.md) | Generated 100-vehicle corpus index. |
| [docs/generated/PARAMETERS.md](docs/generated/PARAMETERS.md) | Generated tunable registry. |

Physics/gameplay and vehicle appearance number their phases independently. Always name the
workstream when referring to a phase. Do not duplicate fast-changing phase status here; reconcile
against the code before changing scope.

## Platform and Toolchain

| Capability | Windows MSYS2 UCRT64 | Linux CI | macOS Make |
|---|---:|---:|---:|
| Interactive game | Supported | No | No |
| DLL hot reload | Supported | No | No |
| Headless scenarios and corpus tools | Supported | Supported | Supported |
| Sanitizers, coverage, and fuzzing | Tool-dependent | Supported | Tool-dependent |

- Install or refresh with `powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1`.
- From cmd.exe or PowerShell use `build.bat` / `.\build.bat`; from UCRT64 use `./build.sh`.
- On Windows, `build.bat` and `mk.bat` enter UCRT64; bare `make` requires an existing UCRT64
  shell. Linux and macOS use GNU Make for headless targets; hosted CI validates Linux.
- Windows raylib 6.0 comes only from `mingw-w64-ucrt-x86_64-raylib`; never build it from source
  for this project.
- `Makefile` source groups are the single compilation manifest. Adding, moving, or deleting a
  `.c` file requires updating the correct group. Do not duplicate source lists in `build.sh`,
  wrappers, tools, or workflows.

Development links shared raylib (`libraylib.dll`). Release links `libraylib.a` statically and,
with the current package, still needs `glfw3.dll`. Verify imports instead of assuming:

```bash
objdump -p build/dev/game.dll | grep -i "DLL Name"
```

`libraylib.dll` must appear for `build/dev/game.dll`. It must not appear for
`build/tests/drifty_tests.exe` or `build/release/drifty_release.exe`.

## Development Workflow — Hot Reload

The platform layer is `build/dev/drifty.exe`; reloadable game code is `build/dev/game.dll`.
`build.bat` always rebuilds the module and rebuilds the executable only when needed. A failed build
leaves a running game on the previous module.

Launch the interactive `build/dev/drifty.exe` from the repository root; Explorer double-click may
prevent it from finding the repository-relative `build/dev/game.dll`. A normal executable session,
`mk run`, and `mk inspect` remain active until a person closes or stops them.

Bounded, self-exiting modes are `build.bat --smoke-test`,
`build/dev/drifty.exe --smoke-test`, `build/dev/drifty.exe --capture-scene NAME`,
`build/dev/drifty.exe --gallery-page N`, `mk screenshots`, `mk visual-test`, `mk gallery`,
`mk cards`, and `mk visual-diagnose`. See `docs/DEVTOOLS.md` and `--help` for their arguments.

### Core commands

```bash
mk test                  # fast scenarios; mk.bat enters UCRT64 from Windows shells
mk scenario NAME=skidpad
mk report NAME=skidpad   # scenario plus artifacts/report_skidpad.html
mk verify                # static analysis, scenarios, and regression comparison
mk ci                    # core local checks; inspect SKIP lines; hosted CI is authoritative
mk cards                 # headless per-car PNGs, label maps, and cards.json
mk visual-diagnose       # bounded appearance measurements in artifacts/visual/
```

Missing local tools such as clang, cppcheck, gcovr, or clang-format print `SKIP` rather than
failing. A successful `mk ci` is insufficient when a required tool was skipped: report every skip.
Hosted CI additionally exercises its compiler matrix, workflow lint, Windows builds, hot-reload
harness, linkage inspection, and CodeQL.

`./build/tests/drifty_tests.exe --list` prints the current scenario table. Use
`--scenario NAME` for focused feedback and `--no-bundle` to print all failures without writing a
new bundle.

A failing scenario writes `artifacts/failure-<scenario>-<timestamp>/` with its input timeline,
telemetry, tunables, first failing check, and build commit. Read it before rerunning. Because the
bundle records only the first failed check, rerun a multi-failure scenario as:

```bash
./build/tests/drifty_tests.exe --scenario corpus --no-bundle
```

When physics telemetry changes intentionally, `mk baselines` rerecords `tests/baselines/`. Never
use it merely to make a regression green; review the numbers and explain the model change.

### Restart requirements

Restart `build/dev/drifty.exe` after:

- changing the layout of `Game` or anything embedded in it, including `DevState`;
- adding a field to `VehicleSpec`, which is embedded in `Game`;
- changing `src/platform/main.c` or `src/platform/hotreload_windows.c`;
- changing `GAME_ENTRY_POINTS`.

One restart is sufficient. Ordinary module-only reloads then preserve body state, wheel speeds,
engine RPM, and gear.

### Reload-safety constraints

- No pointer stored in `Game`, or reachable from it, may point into module code or static data.
  Store identifiers such as `SurfaceId` and resolve them at point of use.
- Never store function pointers in persistent state.
- The platform owns the sole `Game` allocation; never declare `static Game game` in the module.
- Release raylib-tracked textures, sounds, and callbacks in `game_pre_reload`; reacquire them in
  `game_post_reload`.
- Do not allocate heap memory inside `game_fixed_update()` or the physics step. Initialization,
  profile loading, corpus generation, and texture rebakes may allocate at existing lifecycle
  boundaries.

## Generated Artifacts

Generated files are part of the change, not optional bookkeeping.

| Trigger | Required update |
|---|---|
| Parameter registry/default/range change | `mk params-doc` → `docs/generated/PARAMETERS.md` |
| Corpus archetype, generation/sweep logic, or registry default/range consumed by the corpus | Regenerate `data/vehicles/corpus/**` and `docs/generated/CORPUS.md` |
| Intentional physics-model output change | Review and, only when justified, update `tests/baselines/**` with `mk baselines` |
| Appearance grammar or raster change | Run `car-visual`, `corpus`, and `mk visual-diagnose`; regenerate corpus outputs only when corpus specs changed |

Corpus regeneration commands:

```bash
./build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus
./build/tests/drifty_tests.exe --dump-corpus-index docs/generated/CORPUS.md
```

Corpus output depends on `src/dev/car_corpus.c`, `src/dev/car_corpus_archetypes.c`, and relevant
registry defaults/ranges in `src/dev/dev_params.c`. The `corpus` scenario checks profile
round-trips; Linux CI regenerates and diffs `docs/generated/CORPUS.md`.

## Vehicle Appearance

A car's appearance is a pure, total, deterministic function of its physics parameters. There is no
hand-authored per-vehicle art. Read `docs/CAR_VISUAL.md` before modifying
`src/render/car_visual.c`, `src/render/car_visual_raster.c`, `src/dev/car_corpus.c`,
`src/dev/car_corpus_archetypes.c`, or `render_vehicle_draw()` in
`src/render/render_vehicle.c`.

Required invariants:

- Geometry may not come from a hash of raw spec data. Colour is the documented exception and is
  excluded from shape distinctness.
- No `body.type` enum, per-archetype drawing branch, or per-car art asset. Vehicle classes are
  regions of parameter space.
- Styling decisions live only in `src/render/car_visual.c`. The grammar and rasterizer remain
  raylib-call-free so headless tests can link them.
- `CarVisual` remains stack-local and derived per bake; never store it in `Game` or `DevState`.
- Cache keys use canonical field serialization, never raw struct bytes or `memcmp` across padded
  structs. Hashes may invalidate caches and seed colour, never generate geometry.
- Float determinism is within one binary only (`game.dll` is `-O0`, tests are `-O2`). Never compare
  module-computed rasters with test-computed rasters, and never add `-ffast-math`.
- `resources/sprite_examples/` is reference material, not shipped assets.

Focused gates:

```bash
./build/tests/drifty_tests.exe --scenario car-visual
./build/tests/drifty_tests.exe --scenario corpus
mk visual-diagnose
```

Every corpus pair must clear all three floors from `tests/support/appearance_metrics.h`:

| Floor | Constant |
|---|---|
| At least 3.0% of the union silhouette differs | `CV_MIN_PIXEL_DIFF` |
| Signature L∞ at least 0.08 m | `CV_MIN_LINF` |
| Signature L2 at least 0.25 | `CV_MIN_L2` |

Never lower a corpus-wide threshold for one row. A key that cannot carry five distinct sweep steps
belongs in `kVisualDrivers[]`; read the measured exclusions in `src/dev/car_corpus.c` first.

Gates catch similarity, not visual plausibility. `mk cards` writes derived quantities and per-feature
pixel histograms; use them to find overactive or zero-coverage features. `mk visual-diagnose` records
the bounded diagnostic evidence. `mk gallery` renders the production texture path for human review;
it is not a GPU regression baseline.

## Physics Conventions

Physics uses SI units: metres, seconds, kilograms, newtons, and radians. Body X is forward, body Y
is left, heading and yaw rate are counterclockwise-positive, and steering is left-positive. Pixels
exist only in the render layer through `PIXELS_PER_METER`. Physics translation units call no raylib
function.

## Tunables

Every tunable is registered once in `src/dev/dev_params.c` with its default, unit, range, and
description. The registry drives Physics Lab sliders, profiles, telemetry metadata, and generated
parameter documentation. The `params` scenario checks registry defaults against
`vehicle_spec_set_default()`.

Adding a tunable has eleven touch points; use `body.cowl_x` as the template:

1. Add the default constant in `src/core/config.h`.
2. Add the `VehicleSpec` field in `src/physics/vehicle.h`; this requires a restart.
3. Set and validate it in `src/physics/vehicle.c`.
4. Add its `g_params[]` row in `src/dev/dev_params.c`; enums use float storage and step `1.0f`.
5. Add a migration alias only when replacing a previously derived key.
6. Add a `CarVisual` field in `src/render/car_visual.h` when it affects drawn geometry.
7. Consume it in `car_visual_derive()` with the documented `[identity]` or `[rule]` taxonomy.
8. Add it to `car_visual_bake_key()`.
9. Add a `CAR_SIGNATURE_COMPONENTS` component when it contributes to distinctness.
10. Draw it in `src/render/car_visual_raster.c`, adding a `CarRasterLabel` for a new feature.
11. Add it to `kVisualDrivers[]` and the mapping table in `docs/CAR_VISUAL.md`; run
    `mk params-doc`.

Steps 8 and 11 protect each other: omitting both lets a slider change without rebaking and leaves no
assertion to detect it.

## Validation After Build-System Changes

After changing build scripts, source manifests, wrappers, or linkage:

1. `build.bat --clean`
2. `build.bat --tests` and `build/tests/drifty_tests.exe`
3. `build.bat`
4. `build.bat --release`
5. Inspect imports with `objdump -p` on `build/dev/drifty.exe`, `build/dev/game.dll`,
   `build/tests/drifty_tests.exe`, and `build/release/drifty_release.exe`.
6. `./scripts/validate_hotreload.sh`
7. `build.bat --smoke-test`

## Cloned Dependency Source

`.slim/clonedeps/repos/` contains read-only dependency source. Never edit it:

- `raysan5__raylib/` — raylib 6.0 headers, layouts, and examples.
- `unconv__racer/` — rendering references only, not the physics model.
- `alexliniger__MPCC/` — bicycle-model and Pacejka sanity references.
