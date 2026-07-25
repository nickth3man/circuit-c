# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

**Windows only. MSYS2 UCRT64 is the only supported build environment.**

## Project Structure

See `docs/SPEC.md` for the full specification, physics model, data structures, and
incremental build plan. See `docs/SOURCES.md` for the technical references it cites.

## Current phase

**Phase 0 and Phase 1 are complete. Phase 2 has not started.** The game has a planar
rigid-body vehicle, a temporary saturated linear lateral tire model, an isolated temporary
body-level longitudinal command, interpolation, diagnostics, and headless coverage.

Do **not** begin nonlinear tires, drivetrain, wheel rotation/slip ratio, physical braking or
handbrake torque, combined slip, or load transfer outside a deliberate Phase 2/3 task.

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

### For physics work, prefer the headless loop

```bash
./build.sh --tests && ./drifty_tests.exe
```

This terminates, writes CSV telemetry to `telemetry/`, and needs no window. It is the better
feedback loop for equations and tuning; use hot reload for feel, camera, and presentation
work. Run it from the repository root — the telemetry path is relative.

`./drifty_tests.exe --list` prints 16 scenarios: the seven Phase 0 infrastructure scenarios
plus `vehicle`, `rest`, `launch-stop`, `low-speed`, `reverse`, `steer-sign`, `lever-arm`,
`integration`, and `fixed-rate`. The suite currently has 217 passing checks. Stable Phase 1
telemetry is compared with `tests/baselines/phase1_launch_stop.csv`.

### When a restart is required

Tell the developer to restart `drifty.exe` after any of these — hot reload cannot handle
them:

- A change to the layout of `Game` or anything it contains (the existing memory block
  becomes invalid).
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

## Cloned Dependency Source

Read-only dependency source repositories are available under
`.slim/clonedeps/repos/` for inspection. Do not edit these clones.

- `.slim/clonedeps/repos/raysan5__raylib/` — raylib at `6.0`; header/type reference, struct layout verification, 215 examples.
- `.slim/clonedeps/repos/unconv__racer/` — raylib drift game at `5d938cf`; skidmark rendering and Camera2D follow patterns (rendering only, not the physics model).
- `.slim/clonedeps/repos/alexliniger__MPCC/` — MPC bicycle model at `bd33162`; slip-angle and Magic Formula tire force reference for tire.c/vehicle.c, and Pacejka parameters in model.json for sanity checks.
