# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

## Project Structure

See `docs/SPEC.md` for the full specification, physics model, data structures, and
incremental build plan. See `docs/SOURCES.md` for the technical references it cites.

## Current phase

**Phase 0 — Foundations and Test Harness is complete. Phase 1 has not started.** There is no
vehicle, tire, drivetrain, load-transfer, track, particle, or scoring code yet, and none
should be added outside a deliberate Phase 1 task. What the window draws is a labelled
deterministic placeholder marker, not a car. `README.md` lists exactly what exists.

## Toolchain

raylib must be linked as a **shared** library for hot reload to work. If `pkg-config` cannot
find raylib, the build falls back to `$RAYLIB_DIR` (default `vendor/raylib`, gitignored);
`README.md` documents how to populate it. `build.sh` and `build.bat` also probe the usual
MinGW-w64 install locations when `gcc` is not on `PATH`.

Verify the linkage rather than assuming it:

```bash
objdump -p build/game.dll | grep -i "DLL Name"
```

`raylib.dll` must appear. The same check on `drifty_tests` must show no raylib entry at all.

## Development Workflow — Hot Reload

The game runs as a thin platform layer (`drifty.exe`) plus a hot-reloadable game module
(`build/game.dll`). The developer starts the executable once and leaves it running; agents
rebuild the module, and the running game swaps it in without losing state.

**The only build command you need:**

```bash
./build.sh
```

It rebuilds `build/game.dll` always, and rebuilds `drifty.exe` only when the executable is
not already running. **It always terminates in under a second.**

### Rules

- **Never start, launch, or supervise `drifty.exe` yourself.** The developer owns the
  running process. Launching it would block indefinitely and produce no useful output.
- **Never run a file watcher** (`watchexec`, `nodemon`, `--watch` flags) or any command that
  does not return on its own.
- After editing game code, run `./build.sh` and report the compiler result. The visual
  outcome appears in the window the developer already has open — ask them what they see
  rather than trying to observe it yourself.
- A failed build leaves the running game alive on the previous module, so a compile error is
  safe. Report it and fix it.

### For physics work, prefer the headless loop

```bash
./build.sh --tests && ./drifty_tests
```

This terminates, writes CSV telemetry to `telemetry/`, and needs no window. It is the better
feedback loop for equations and tuning; use hot reload for feel, camera, and presentation
work. Run it from the repository root — the telemetry path is relative.

`./drifty_tests --list` prints the scenarios. In Phase 0 they are infrastructure only
(`math`, `units`, `timestep`, `oneshot`, `replay`, `renderscale`, `telemetry`); the physics
scenarios named in `docs/SPEC.md` — `skidpad` and the rest — arrive with the code they
exercise, from Phase 1 onward, together with the committed baselines in `tests/baselines/`.

### When a restart is required

Tell the developer to restart `drifty.exe` after any of these — hot reload cannot handle
them:

- A change to the layout of `Game` or anything it contains (the existing memory block
  becomes invalid).
- A change to `main.c` or the `hotreload_*` files (the platform layer is not reloadable).
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
