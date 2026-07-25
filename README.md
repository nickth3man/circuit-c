# Drifty

A top-down 2D drift driving simulator in C11 with raylib 6.0. The design goal is a
physically coherent vehicle simulation underneath an arcade presentation layer: the car
initiates, holds, transitions, and recovers a drift because of tire, drivetrain, and
load-transfer behaviour, not because a state machine reaches in and changes forces.

**Windows only.** The supported development environment is **MSYS2 UCRT64**.

- Full specification: [docs/SPEC.md](docs/SPEC.md)
- Reference index: [docs/SOURCES.md](docs/SOURCES.md)
- Agent-facing workflow rules: [AGENTS.md](AGENTS.md)

## Current phase: Phase 1 — Rigid-Body Vehicle

Phase 0 and Phase 1 are complete. The running game now uses a deterministic planar
rigid-body vehicle in SI units:

| System | State |
|--------|-------|
| SI units, coordinate and sign convention | `src/units.h`, `src/config.h` |
| Math helpers (`clampf`, `lerpf`, `smooth_to`, `wrap_angle`, `smoothstep`, `lerp_angle`) | `src/math_utils.h/.c` |
| Fixed 120 Hz timestep with substep cap and backlog-drop counter | `src/timestep.h/.c`, driven by `src/main.c` |
| Held controls vs one-shot commands, consumed exactly once | `src/input.h/.c` |
| Deterministic fixed-tick input recording and playback | `src/replay.h/.c` |
| CSV telemetry writer | `src/telemetry.h/.c` |
| Platform-owned `Game` block, hot-reloadable game module | `src/main.c`, `src/hotreload_windows.c`, `src/game.h/.c` |
| Headless test executable | `tests/physics_tests.c` |
| Windowless hot-reload harness | `tests/hotreload_harness.c` |
| Bounded visual smoke test | `drifty.exe --smoke-test` |
| Canonical vehicle specification/state/diagnostics | `src/vehicle.h/.c` |
| Steering, contact kinematics, tire forces, body integration | `src/physics.h/.c` |
| Interpolated body, four wheels, HUD, debug vectors | `src/render.h/.c` |

Phase 1 uses distinct front/rear slip angles, static axle loads, and
`Fy = -C_alpha * alpha` saturated at `mu * Fz`. Front force rotates through the road-wheel
angle, and yaw comes only from tire-force torque. A kinematic derivative model blends into
the dynamic model from 1.5 to 3.0 m/s before semi-implicit Euler integration.

The temporary longitudinal command applies a body-level force in newtons. Q selects reverse,
E selects forward, throttle applies force in that direction, and brake opposes current
travel without pushing through zero. It is isolated in `physics.c` and is not an engine,
drivetrain, longitudinal tire, or physical brake model.

The test runner covers 16 scenarios and 217 checks. The reviewed launch/stop telemetry
baseline is `tests/baselines/phase1_launch_stop.csv`; generated CSV and the bounded
smoke-test screenshot are written under `telemetry/`.

Phase 2 is next: nonlinear lateral tires, rear-wheel drivetrain torque, wheel angular
velocity and longitudinal slip, physical braking/handbrake torque, and combined grip.

## Prerequisites (Windows / MSYS2 UCRT64)

1. Install MSYS2 (default root `C:\msys64`):

```bat
winget install -e --id MSYS2.MSYS2 --accept-package-agreements --accept-source-agreements
```

2. Install the project toolchain and raylib 6.0 from the MSYS2 package manager:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1
```

That script is idempotent. It installs (when missing):

- `mingw-w64-ucrt-x86_64-gcc`
- `mingw-w64-ucrt-x86_64-raylib`
- `mingw-w64-ucrt-x86_64-pkgconf`
- `mingw-w64-ucrt-x86_64-binutils`
- `make`

There is **no** Chocolatey GCC path, **no** vendored `vendor/raylib` build, and **no**
manual raylib source compile. raylib comes only from the MSYS2 package.

## Building

`build.bat` is the Windows entry point: it enters MSYS2 UCRT64 and runs `build.sh`.
`build.sh` is the canonical implementation and refuses to run outside UCRT64.
The `Makefile` exposes the same configurations and the same flags.

```bat
build.bat                 rem hot-reload development build
build.bat --release       rem single executable, static raylib, no game.dll
build.bat --tests         rem headless test executable
build.bat --hotreload-harness
build.bat --smoke-test    rem build, then bounded visual smoke test (exits alone)
build.bat --clean
```

Equivalent inside an MSYS2 UCRT64 shell:

```bash
./build.sh
./build.sh --release
./build.sh --tests
./build.sh --hotreload-harness
./build.sh --smoke-test
./build.sh --clean
```

```bash
make debug
make release
make tests
make hotreload-harness
make run-tests
make smoke-test
make clean
make info
```

Every normal build command terminates immediately. Nothing starts a watcher or leaves a
persistent game process running. `--smoke-test` launches the real window, runs a fixed
frame budget, writes a screenshot, and exits.

### Running

```bat
drifty.exe                 rem development build; start once and leave it running
drifty.exe --smoke-test    rem bounded visual verification; exits on its own
drifty_release.exe         rem release build
drifty_tests.exe           rem headless tests; run from the repository root
drifty_hotreload_harness.exe
```

`drifty_tests.exe` writes CSV telemetry to `telemetry/` relative to the working directory,
so run it from the repository root. It accepts `--list`, `--scenario NAME`, and `-v`.

## Linkage

### Development (hot reload)

`drifty.exe` and `build/game.dll` both link the MSYS2 **shared** raylib import library and
therefore both import `libraylib.dll` (MSYS2's DLL name). The build copies:

- `libraylib.dll`
- `glfw3.dll` (required by `libraylib.dll`)

next to the executables so launching from Explorer or a normal terminal does not depend on
a hand-edited `PATH`. Those DLLs are generated and gitignored.

Never compile raylib sources into `game.dll`.

### Release

`drifty_release.exe` compiles platform + game into one executable with `DRIFTY_HOT_RELOAD`
undefined. It links `libraylib.a` statically and does **not** import `libraylib.dll` or
`game.dll`. With the current MSYS2 raylib package, the static archive still references
shared GLFW, so `glfw3.dll` is copied next to the release executable. That is a package
limitation, not a project DLL.

### Verify imports

```bash
objdump -p drifty.exe | grep -i "DLL Name"
objdump -p build/game.dll | grep -i "DLL Name"
objdump -p drifty_tests.exe | grep -i "DLL Name"
objdump -p drifty_release.exe | grep -i "DLL Name"
```

Expected: development artifacts import `libraylib.dll`; tests and release do not.

## Hot-reload workflow

The game is a thin platform layer (`drifty.exe`) plus a hot-reloadable game module
(`build/game.dll`). The platform layer owns the window, the raylib context, the `Game`
allocation, and the fixed-timestep loop. Everything else lives in the module.

1. Run `drifty.exe` once and leave it open.
2. Edit game code.
3. Run `build.bat` (or `./build.sh` in UCRT64). It rebuilds the module always, rebuilds the
   executable only when it is not already running, and returns in well under a second.
4. The running game notices the new module and swaps it in, keeping its position, counters,
   and checksum.

The loader never unloads a working module until a replacement has been proven good. A
compile error cannot close the running game.

Automated validation without leaving `drifty.exe` running:

```bat
build.bat --hotreload-harness
drifty_hotreload_harness.exe
```

Or the fuller script (harness + failed-compile preservation):

```bash
# inside MSYS2 UCRT64, from the repo root
./scripts/validate_hotreload.sh
```

### When a restart is required

Hot reload cannot handle these. Restart `drifty.exe` after:

- A change to the layout of `Game` or anything it contains.
- A change to `src/main.c`, `src/timestep.c`, or `src/hotreload_windows.c`.
- A change to `GAME_ENTRY_POINTS`.

### Reload-safety rules for game code

- No pointer stored in `Game`, or reachable from it, may point into the module's code or
  static data.
- No function pointers in persistent state.
- The `Game` block is allocated and owned by the platform layer. Never declare
  `static Game game;` inside the module.
- Anything raylib tracks is released in `game_pre_reload` and re-acquired in
  `game_post_reload`.

## Known limitations

- **Changing the `Game` struct layout requires a restart.** Inherent to the technique.
- **Restart `drifty.exe` once for the Phase 1 layout.** `Game` now embeds the canonical
  vehicle structures; normal code-only hot reload works after that restart.
- **Phase 1 longitudinal force is intentionally temporary.** It provides launch, stopping,
  and explicit reverse without pretending the Phase 2 drivetrain exists.
- **Release still needs `glfw3.dll`.** The MSYS2 `libraylib.a` was built against shared
  GLFW (`__imp_glfw*`), so a fully static single-file release without any third-party DLL
  is blocked by that package layout. `libraylib.dll` and `game.dll` are not required.

## Layout

```
Makefile, build.sh, build.bat   build entry points; all terminate immediately
scripts/setup_windows.ps1       idempotent MSYS2 UCRT64 bootstrap
scripts/validate_hotreload.sh   harness + failed-compile preservation
docs/SPEC.md, docs/SOURCES.md   specification and reference index
src/main.c                      platform layer: window, Game allocation, fixed-timestep loop
src/timestep.h/.c               the accumulator, isolated so the harness can assert it
src/hotreload.h                 GAME_ENTRY_POINTS, the one authoritative entry-point list
src/hotreload_windows.c         LoadLibrary / GetProcAddress loader
src/game.h/.c                   the Game block and the reloadable entry points
src/config.h                    Phase 0/1 constants, every physical value unit-bearing
src/units.h                     world<->render conversion and the coordinate convention
src/math_utils.h/.c             scalar helpers raymath.h does not provide
src/input.h/.c                  held controls and one-shot commands
src/replay.h/.c                 deterministic fixed-tick input timeline
src/telemetry.h/.c              CSV row writer, no raylib dependency
src/vehicle.h/.c                canonical vehicle data and initialization
src/physics.h/.c                pure Phase 1 physics and fixed-update owner
src/render.h/.c                 interpolation, vehicle rendering, HUD, debug vectors
tests/physics_tests.c           headless scenario runner
tests/hotreload_harness.c       windowless hot-reload validation
tests/baselines/                reviewed deterministic scenario CSV baselines
telemetry/                      generated CSV / smoke screenshot (gitignored)
```
