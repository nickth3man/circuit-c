# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

**Windows only. MSYS2 UCRT64 is the only supported build environment.**

## Project Structure

| Document | What it is |
|---|---|
| [docs/SPEC.md](docs/SPEC.md) | The full specification: physics model, data structures, incremental build plan. Authoritative. |
| [docs/SOURCES.md](docs/SOURCES.md) | The reference index SPEC.md cites. |
| [docs/PHASE3_VALIDATION.md](docs/PHASE3_VALIDATION.md) | Accepted handling metrics, baseline policy, and the Phase 1–3 acceptance evidence. |
| [docs/DEVTOOLS.md](docs/DEVTOOLS.md) | The development shell: Physics Lab, replay inspector, failure bundles, telemetry reports, the vehicle corpus and gallery, and the one-command make targets. |
| [docs/CI.md](docs/CI.md) | The workflows, the required checks, and why the gates are shaped as they are. |
| [docs/CAR_VISUAL.md](docs/CAR_VISUAL.md) | The vehicle-appearance contract — what every drawn feature reads, the render-only gains, the raster layer order, and the rules a change must not break. |
| [docs/CORPUS.md](docs/CORPUS.md) | The 100 demonstration vehicles, generated from `src/car_corpus.c`. |
| [docs/PARAMETERS.md](docs/PARAMETERS.md) | The tunable registry, generated. |

## Current phase

**Two workstreams number their phases independently, so "phase 4" is ambiguous on its own.
Always say which.**

**Physics phases (`docs/SPEC.md`) — 0–3 complete.** The mandatory foundation includes
filtered previous-step longitudinal acceleration, static-plus-dynamic axle loads, per-wheel
load propagation, quadratic aerodynamic drag, load-dependent rolling resistance, objective
handling maneuvers, and reviewed local baselines.

Physics phase 4 (four-wheel fidelity) is optional. Do **not** begin it, or track/gameplay
work, outside a deliberate task for that phase.

**Vehicle-appearance phases — 0–6 complete**, merged as `vehicle-visuals/phase0..phase6`.
That workstream built the parameter registry expansion, the appearance grammar, the corpus,
the production texture path, the in-game gallery, and the CI wiring. It is *active*, not
off-limits: see [Vehicle appearance](#vehicle-appearance) for the contract it must satisfy.
Ongoing work there is parameter and grammar refinement, not new subsystems.

## Toolchain

- Install / refresh with `powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1`
- Build from cmd.exe **or PowerShell** with `build.bat` / `.\build.bat`, or from an MSYS2
  UCRT64 shell with `./build.sh`
- `build.bat` and `mk.bat` both enter UCRT64 for you, so they work from any Windows shell.
  Bare `make` does **not**: it aborts unless `MSYSTEM=UCRT64` is already set. Use `mk` rather
  than `make` unless you are inside a UCRT64 shell
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
- **Exceptions — bounded, self-exiting, and allowed:**
  - `build.bat --smoke-test` / `drifty.exe --smoke-test` — a fixed frame budget, then exit.
  - `drifty.exe --capture-scene NAME` — renders one deterministic scene and exits.
    `mk screenshots` and `mk visual-test` are built on it.
  - `drifty.exe --gallery-page N` — draws one corpus page and exits. `mk gallery` uses it.
  - `mk cards` and `mk visual-diagnose` — fully headless; `visual-diagnose` starts and stops
    its own server.
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
mk test                  # fast scenarios; mk.bat enters UCRT64 for you from any shell
mk scenario NAME=skidpad
mk report NAME=skidpad   # runs it, then writes artifacts/report_skidpad.html
mk verify                # static analysis + every scenario + the regression comparison
mk ci                    # the local equivalent of the required CI checks
mk cards                 # per-car PNGs, feature-label maps and cards.json (headless)
mk visual-diagnose       # appearance measurements + evidence into artifacts/visual/
```

**Two targets an agent must never invoke**, both because they block rather than because they
are dangerous:

- **`mk run`** launches `drifty.exe` and does not return. Rebuild with `build.bat` (or
  `mk dev`) and let the running game pick the module up.
- **`mk inspect`** serves the browser inspector and does not return. Use `mk visual-diagnose`
  instead: it starts and stops its own server, exits on its own, and writes the same evidence
  to disk.

When telemetry numbers change **legitimately** — you altered the model on purpose and the
regression comparison is now correctly red — `mk baselines` re-records `tests/baselines/`
from the current build. It is not a way to make a failing test green: the target says so
itself, and a PR that touches those files has to explain in words why the new numbers are
right.

Tools that are not installed (clang, cppcheck, gcovr, clang-format) print a `SKIP` line with
the install command instead of failing. That is expected locally; CI installs all of them.

### For physics work, prefer the headless loop

```bash
./build.sh --tests && ./drifty_tests.exe
```

This terminates, writes CSV telemetry to `telemetry/`, and needs no window. It is the better
feedback loop for equations and tuning. Run it from the repository root — the telemetry path
is relative.

Vehicle appearance has its own headless loop — `mk visual-diagnose`, see
[Measuring appearance](#measuring-appearance-not-just-gating-it). Reserve hot reload for the
things that genuinely need a running window: feel, camera, and HUD.

`./drifty_tests.exe --list` prints the scenario table; the suite currently runs 54 scenarios.
The check count is deliberately not quoted here — it moves with every parameter added, and a
hand-maintained copy of a generated number is only ever a stale number.

Alongside the physics coverage — acceleration filter, load transfer, resistance, acceleration
and braking load, skidpad sweep, step steer, lift-off, transition, catchable drift — the
`car-visual` and `corpus` scenarios gate the vehicle appearance system (see
[Vehicle appearance](#vehicle-appearance)). Generated telemetry is written under `telemetry/`;
`tests/baselines/` holds the reviewed baselines that `mk regression` compares against.

Other modes of the same executable:

```bash
./drifty_tests.exe --scenario skidpad      # one scenario
./drifty_tests.exe --dump-params docs/PARAMETERS.md
./drifty_tests.exe --benchmark 240000      # fixed-update throughput
./drifty_tests.exe --no-bundle             # do not write artifacts/failure-* on failure

./drifty_tests.exe --generate-corpus tuning/corpus          # REQUIRED after car_corpus.c edits
./drifty_tests.exe --dump-corpus-cards artifacts/corpus-cards  # per-car PNG + label map + JSON
./drifty_tests.exe --dump-corpus-metrics artifacts/metrics.csv # latents + signatures as CSV
./drifty_tests.exe --dump-corpus-index docs/CORPUS.md       # the corpus table
```

A failing scenario writes `artifacts/failure-<scenario>-<timestamp>/` containing the input
timeline, the telemetry, the tunables, the failing check, and the commit the binary was built
from. Read that directory before re-running anything.

**The bundle records the first failing check, not all of them.** When a scenario reports
several failures at once, `failure.txt` names only the first, which is often not the
informative one. Re-run that scenario with `--no-bundle` and read stdout to see the rest:

```bash
./drifty_tests.exe --scenario corpus --no-bundle
```

### When a restart is required

Tell the developer to restart `drifty.exe` after any of these — hot reload cannot handle
them:

- A change to the layout of `Game` or anything it contains (the existing memory block
  becomes invalid). **The development-tool state (`DevState`, src/dev_state.h) is part of
  `Game` in every build configuration**, deliberately: making it conditional would make two
  separately compiled binaries disagree about the layout of the block they share. Adding a
  field to it is therefore a restart, like any other layout change.
- **Adding a field to `VehicleSpec` (src/vehicle.h)** — it lives inside `Game`, so this is
  the same layout change as above. Called out by name because it is the most frequently
  edited struct in the project: every new tunable parameter is a field on it, and every one
  of them costs the developer one restart.
- A change to `main.c` or `hotreload_windows.c` (the platform layer is not reloadable).
- A change to `GAME_ENTRY_POINTS`.

One restart clears it. After that, ordinary module-only hot reload preserves body state,
wheel speeds, engine RPM, and gear — so the cost is a single interruption per layout change,
not a degraded workflow.

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

### After changing `src/car_corpus.c`, regenerate the profiles

```bash
./drifty_tests.exe --generate-corpus tuning/corpus
```

`tuning/corpus/**.txt` is **checked in**, and the `corpus` scenario asserts that every
profile round-trips to the spec the code produces. Edit an archetype or a sweep axis without
regenerating and you get:

```
FAIL every checked-in corpus profile round-trips to its spec
```

which names the symptom and not the fix. This is the same obligation as `mk params-doc`
after a registry change — the export is not optional bookkeeping, it is part of the edit.

### The three distinctness floors

Every **pair** of corpus vehicles must clear all three of these at once
(`tests/physics_tests.c`):

| floor | constant | meaning |
|---|---|---|
| ≥ 3.0% of the union silhouette differs | `CV_MIN_PIXEL_DIFF` | the authoritative, colour-blind label-map comparison |
| signature L∞ ≥ 0.08 m | `CV_MIN_LINF` | one visible pixel at 13.2 px/m, literally |
| signature L2 ≥ 0.25 | `CV_MIN_L2` | many small differences count as much as one large one |

Adjacent steps of a five-step sweep are the tightest pairs in the fleet, so **a sweep axis
has to move roughly a fifth of its range past all three floors, four times over.** Most
parameters cannot. That is arithmetic, not tuning.

**Never lower a corpus-wide threshold to accommodate one row.** When a key cannot carry a
sweep, it goes in `kVisualDrivers[]` instead, where the `car-visual` sensitivity test
perturbs it across its whole declared registry range and the bake-key assertion proves the
sprite rebakes for it. `car_corpus.c` records three worked examples of this decision
(`tire.aspect_*`, `body.backlight_x`, `body.bed_length`) with the measured numbers that
forced each one; read them before adding or removing an axis.

### Measuring appearance, not just gating it

The gates answer "is any pair too similar". They do **not** answer "does this look like a
car", and several long-lived defects passed every gate for their whole lifetime — a pickup
bed drawn on 78 of 100 vehicles, three style axes frozen at a constant, a three-way hue split
against a reference median of one. All of those were invisible until they were *measured*.

```bash
mk cards                 # per-car PNG + feature-label map + cards.json for all 100
mk visual-diagnose       # runs the measurements, writes artifacts/visual/diagnostics.txt
```

`cards.json` carries every derived quantity per vehicle plus a **per-feature pixel
histogram** — how many pixels each `CarRasterLabel` actually owns at game scale. That
histogram is the tool that finds "this rule is firing on cars it should not" and "this
feature computes correctly and covers zero pixels", neither of which any pass/fail check
reports. Reach for it before changing a rule, not after.

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

### Adding a parameter: the eleven touch points

Adding a tunable is more than a registry row. Use `body.cowl_x` as the worked template — it
is a plain metre-valued geometry key that exercises every step.

1. **`src/config.h`** — the default constant.
2. **`src/vehicle.h`** — the `float` field on `VehicleSpec`. *(Restart required; see above.)*
3. **`src/vehicle.c`** — set it in `vehicle_spec_set_default()`, and validate it in
   `vehicle_spec_is_valid()` if it must be finite or non-negative.
4. **`src/dev_params.c`** — the `g_params[]` row. Enums ride as floats with `step = 1.0f`,
   exactly like `drive.diff_mode` and `drive.layout`.
5. **Migration alias** — only when the key *replaces* something previously derived, so old
   profiles and presets still load.
6. **`src/car_visual.h`** — a `CarVisual` field, if the parameter produces drawn geometry.
7. **`src/car_visual.c`** — consume it in `car_visual_derive()`, labelled `[identity]` or
   `[rule]` per the taxonomy in `docs/CAR_VISUAL.md`.
8. **`car_visual_bake_key()`** — add the field.
9. **`CAR_SIGNATURE_COMPONENTS`** — add a component if it should count toward distinctness.
10. **`src/car_visual_raster.c`** — draw it, with a `CarRasterLabel` if it is a genuinely new
    feature, so it shows up in the label map and the pixel histogram.
11. **`kVisualDrivers[]`** (`tests/physics_tests.c`) and the feature-mapping table in
    `docs/CAR_VISUAL.md`; then `mk params-doc`.

**Step 8 fails silently.** Omit it and everything still compiles, the slider still moves, the
value still reaches the grammar — and the sprite never rebakes, because the cache key did not
change. Nothing turns red. The only thing that catches it is the bake-key assertion driven by
`kVisualDrivers[]` in step 11, so **8 and 11 protect each other and skipping both is
undetected.** If you add only one of the two, add 11.

If the parameter feeds the appearance system at all, finish with
`./drifty_tests.exe --generate-corpus tuning/corpus` when you also touched `car_corpus.c`.

## Cloned Dependency Source

Read-only dependency source repositories are available under
`.slim/clonedeps/repos/` for inspection. Do not edit these clones.

- `.slim/clonedeps/repos/raysan5__raylib/` — raylib at `6.0`; header/type reference, struct layout verification, 215 examples.
- `.slim/clonedeps/repos/unconv__racer/` — raylib drift game at `5d938cf`; skidmark rendering and Camera2D follow patterns (rendering only, not the physics model).
- `.slim/clonedeps/repos/alexliniger__MPCC/` — MPC bicycle model at `bd33162`; slip-angle and Magic Formula tire force reference for tire.c/vehicle.c, and Pacejka parameters in model.json for sanity checks.
