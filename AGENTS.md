# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

**Windows only. MSYS2 UCRT64 is the only supported build environment.**

## Project Structure

See `docs/SPEC.md` for the full specification, physics model, data structures, and
incremental build plan. See `docs/SOURCES.md` for the technical references it cites.

## Current phase

**Phases 0–3 are complete.** The mandatory physics foundation now includes filtered
previous-step longitudinal acceleration, static-plus-dynamic axle loads, per-wheel load
propagation, quadratic aerodynamic drag, load-dependent rolling resistance, objective
handling maneuvers, and reviewed local baselines.

Phase 4 is optional. Do **not** begin four-wheel fidelity, track/gameplay work, or
presentation work outside a deliberate task for that phase.

## Toolchain

- Install / refresh with `powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1`
- Build from cmd.exe with `build.bat`, or from an MSYS2 UCRT64 shell with `./build.sh`
- `build.sh` refuses non-UCRT64 environments. Do not probe Chocolatey GCC or `vendor/raylib`
- raylib 6.0 comes from the MSYS2 package `mingw-w64-ucrt-x86_64-raylib` only
- Do not build raylib from source for this project

Development links shared raylib (`libraylib.dll`). Release links `libraylib.a` statically
(still needs `glfw3.dll` with the current MSYS2 package). Verify rather than assuming:

```bash
objdump -p build/game.dll | grep -i "DLL Name"
```

`libraylib.dll` must appear for development artifacts. The same check on `drifty_tests.exe`
and `drifty_release.exe` must show no `libraylib.dll` entry.

## Development Workflow — Hot Reload

The game runs as a thin platform layer (`drifty.exe`) plus a hot-reloadable game module
(`build/game.dll`). The developer starts the executable once and leaves it running; agents
rebuild the module, and the running game swaps it in without losing state.

**The only build command you need for game edits:**

```bat
build.bat
```

It rebuilds `build/game.dll` always, and rebuilds `drifty.exe` only when the executable is
not already running. **It always terminates in under a second.**

### Rules

- **Never start, launch, or supervise `drifty.exe` yourself** for interactive sessions. The
  developer owns that process. Launching it would block indefinitely.
- **Exception:** the bounded `build.bat --smoke-test` / `drifty.exe --smoke-test` path is
  allowed because it exits on its own after a fixed frame budget.
- **Never run a file watcher** (`watchexec`, `nodemon`, `--watch` flags) or any command that
  does not return on its own.
- After editing game code, run `build.bat` and report the compiler result. The visual
  outcome appears in the window the developer already has open — ask them what they see
  rather than trying to observe it yourself.
- A failed build leaves the running game alive on the previous module, so a compile error is
  safe. Report it and fix it.
- There is no POSIX hot-reload path and no expectation of Linux/macOS validation.

### The development shell

[docs/DEVTOOLS.md](docs/DEVTOOLS.md) documents the tooling around the game: the in-game
Physics Lab (F2), the replay inspector (F3), failure bundles, telemetry reports, and one
command per operation. [docs/CI.md](docs/CI.md) documents the workflows.

```bash
mk test                  # fast scenarios; mk.bat enters UCRT64 for you from cmd.exe
mk scenario NAME=skidpad
mk report NAME=skidpad   # runs it, then writes artifacts/report_skidpad.html
mk verify                # static analysis + every scenario + the regression comparison
mk ci                    # the local equivalent of the required CI checks
```

**`mk run` is the one target an agent must never invoke** — it launches `drifty.exe` and does
not return. Rebuild with `build.bat` (or `mk dev`) and let the running game pick the module
up, exactly as before.

Tools that are not installed (clang, cppcheck, gcovr, clang-format) print a `SKIP` line with
the install command instead of failing. That is expected locally; CI installs all of them.

### For physics work, prefer the headless loop

```bash
./build.sh --tests && ./drifty_tests.exe
```

This terminates, writes CSV telemetry to `telemetry/`, and needs no window. It is the better
feedback loop for equations and tuning; use hot reload for feel, camera, and presentation
work. Run it from the repository root — the telemetry path is relative.

`./drifty_tests.exe --list` prints the scenario table; the suite currently runs 54 scenarios
and 1109 checks. Alongside the physics coverage — acceleration filter, load transfer,
resistance, acceleration and braking load, skidpad sweep, step steer, lift-off, transition,
catchable drift — the `car-visual` and `corpus` scenarios gate the vehicle appearance system
(see [Vehicle appearance](#vehicle-appearance)). Generated telemetry is written under
`telemetry/`; `tests/baselines/` holds the reviewed baselines that `mk regression` compares
against.

Other modes of the same executable:

```bash
./drifty_tests.exe --scenario skidpad      # one scenario
./drifty_tests.exe --dump-params docs/PARAMETERS.md
./drifty_tests.exe --benchmark 240000      # fixed-update throughput
./drifty_tests.exe --no-bundle             # do not write artifacts/failure-* on failure
```

A failing scenario writes `artifacts/failure-<scenario>-<timestamp>/` containing the input
timeline, the telemetry, the tunables, the failing check, and the commit the binary was built
from. Read that directory before re-running anything.

### When a restart is required

Tell the developer to restart `drifty.exe` after any of these — hot reload cannot handle
them:

- A change to the layout of `Game` or anything it contains (the existing memory block
  becomes invalid). **The development-tool state (`DevState`, src/dev_state.h) is part of
  `Game` in every build configuration**, deliberately: making it conditional would make two
  separately compiled binaries disagree about the layout of the block they share. Adding a
  field to it is therefore a restart, like any other layout change.
- A change to `main.c` or `hotreload_windows.c` (the platform layer is not reloadable).
- A change to `GAME_ENTRY_POINTS`.

### Reload-safety constraints on game code

These are specification requirements, not suggestions — violating them causes crashes that
only appear after a reload:

- No pointer stored in `Game`, or reachable from it, may point into the game module's code
  or static data. Surfaces are referenced by `SurfaceId` and resolved through
  `Surface_Get()` at point of use, never cached as `const SurfaceSpec *`.
- No function pointers in persistent state.
- The `Game` block is allocated and owned by the platform layer. Never declare
  `static Game game;` inside the game module.
- Anything raylib tracks (textures, sounds, audio stream callbacks) is released in
  `game_pre_reload` and re-acquired in `game_post_reload`.

## Vehicle appearance

A car's appearance is a **pure, total, deterministic function of its physics parameters**.
There is no hand-authored art for any vehicle. [docs/CAR_VISUAL.md](docs/CAR_VISUAL.md) is the
full contract — what every drawn feature reads, where the render-only gains are and why, the
raster layer order and pivots, and the scale chain. Read it before touching
`src/car_visual.c`, `src/car_visual_raster.c`, `src/car_corpus.c`, or `draw_vehicle()`.

These are the rules that a change must not break, and each one exists because breaking it
would silently destroy a property a test is protecting:

- **No geometric feature may be generated from a hash of raw spec data.** Byte-hashing would
  trivially satisfy the distinctness test while destroying the property that test exists to
  protect. Every geometric feature cites the parameters it reads. Colour is the single stated
  exception: it is explicitly arbitrary, stable per car, and excluded from the distinctness
  metric so that shape has to carry the result.
- **No `body.type` enum, no per-archetype drawing branch, no per-car art asset.** A pickup, a
  bus and an open-wheel car are regions of parameter space, not cases in a switch.
- **No styling decision outside `src/car_visual.c`.** `render.c` stubs its whole draw path out
  under `DRIFTY_HEADLESS`, so anything decided there is unreachable from `drifty_tests.exe`
  and unverifiable. `car_visual.c` and `car_visual_raster.c` are raylib-*free* — they use the
  `Color`/`Vector2` types and call no raylib function — which is what keeps them linkable into
  the headless test binary.
- **`CarVisual` is a stack local**, derived per bake. Never stored in `Game` or `DevState`.
- **Cache keys are canonical field serialization**, never raw struct bytes and never `memcmp`:
  both structs contain padding whose contents are unspecified. A hash may invalidate a cache
  and may seed colour; it may never produce geometry.
- **Float determinism holds only within one binary** (`game.dll` at `-O0`, `drifty_tests` at
  `-O2`). Never compare a module-computed raster against a test-computed one, and do not add
  `-ffast-math`.
- **`resources/sprite_examples/` is reference, not assets.** Those sprites are there to show
  how appearance maps from proportions. They are never shipped.

The gates, all headless and all bounded:

```bash
./drifty_tests.exe --scenario car-visual   # purity, sensitivity, monotonicity, scale independence
./drifty_tests.exe --scenario corpus       # validity, profile round-trip, all-pairs distinctness
./drifty_tests.exe --dump-corpus-sheet artifacts/gallery   # then open index.html and look
```

`mk gallery` renders the same fleet through the production texture path. It is a human-review
artifact, deliberately not a GPU regression baseline.

## Validation after build-system edits

After changing build scripts, wrappers, or linkage:

1. `build.bat --clean`
2. `build.bat --tests` && `drifty_tests.exe`
3. `build.bat`
4. `build.bat --release`
5. Import inspection with `objdump -p` on `drifty.exe`, `build/game.dll`,
   `drifty_tests.exe`, `drifty_release.exe`
6. `./scripts/validate_hotreload.sh` (UCRT64)
7. `build.bat --smoke-test`

## Physics Conventions

Physics is SI units throughout — meters, seconds, kilograms, newtons, radians. Pixels exist
only in the render layer via `PIXELS_PER_METER`. Body X is forward, body Y is left, heading
and yaw rate are counterclockwise-positive, steering is left-positive. Physics translation
units must not call any raylib function, which is what keeps `drifty_tests` headless.

`docs/SPEC.md` is authoritative for all of this.

## Tunables

Every tunable physical parameter is registered once in `src/dev_params.c` with its default,
unit, range, and description, and that registry generates the Physics Lab sliders, the tuning
profile format, the telemetry metadata, and `docs/PARAMETERS.md`. Changing a constant in
`config.h` without updating the registry fails the `params` scenario, on purpose.

After changing the registry, run `mk params-doc` to regenerate the documentation table.

## Cloned Dependency Source

Read-only dependency source repositories are available under
`.slim/clonedeps/repos/` for inspection. Do not edit these clones.

- `.slim/clonedeps/repos/raysan5__raylib/` — raylib at `6.0`; header/type reference, struct layout verification, 215 examples.
- `.slim/clonedeps/repos/unconv__racer/` — raylib drift game at `5d938cf`; skidmark rendering and Camera2D follow patterns (rendering only, not the physics model).
- `.slim/clonedeps/repos/alexliniger__MPCC/` — MPC bicycle model at `bd33162`; slip-angle and Magic Formula tire force reference for tire.c/vehicle.c, and Pacejka parameters in model.json for sanity checks.
