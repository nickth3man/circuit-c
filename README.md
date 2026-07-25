# Drifty

A top-down 2D drift driving simulator in C11 with raylib 6.0. The design goal is a
physically coherent vehicle simulation underneath an arcade presentation layer: the car
initiates, holds, transitions, and recovers a drift because of tire, drivetrain, and
load-transfer behaviour, not because a state machine reaches in and changes forces.

- Full specification: [docs/SPEC.md](docs/SPEC.md)
- Reference index: [docs/SOURCES.md](docs/SOURCES.md)
- Agent-facing workflow rules: [AGENTS.md](AGENTS.md)

## Current phase: Phase 0 — Foundations and Test Harness

**No vehicle physics is implemented yet.** Phase 0 builds the foundation the later phases
sit on, and nothing else. What exists today:

| System | State |
|--------|-------|
| SI units, coordinate and sign convention | `src/units.h`, `src/config.h` |
| Math helpers (`clampf`, `lerpf`, `smooth_to`, `wrap_angle`, `smoothstep`, `lerp_angle`) | `src/math_utils.h/.c` |
| Fixed 120 Hz timestep with substep cap and backlog-drop counter | `src/timestep.h/.c`, driven by `src/main.c` |
| Held controls vs one-shot commands, consumed exactly once | `src/input.h/.c` |
| Deterministic fixed-tick input recording and playback | `src/replay.h/.c` |
| CSV telemetry writer | `src/telemetry.h/.c` |
| Platform-owned `Game` block, hot-reloadable game module | `src/main.c`, `src/hotreload*.{h,c}`, `src/game.h/.c` |
| Headless test executable | `tests/physics_tests.c` |

What the running window shows is a **deterministic placeholder marker**, not a car: its
heading integrates the steer axis at a constant rate and its position integrates a constant
speed. It exists so the loop, the interpolation, the debug HUD, and the replay checksum have
something to act on. `src/physics.c` replaces it in Phase 1 and owns the fixed update order
from that point onward.

Phase 1 begins with `vehicle.h/.c` and `physics.h/.c`: the canonical `VehicleSpec`,
`VehicleState`, `VehicleDerived`, `VehicleRenderState`, and `WheelState[WHEEL_COUNT]`
structures from docs/SPEC.md, per-axle slip angles, and semi-implicit Euler integration.

## Prerequisites

- A C11 compiler. MinGW-w64 GCC is the primary target on Windows; Clang and GCC work
  elsewhere.
- **raylib 6.0, linked as a shared library.** See [raylib linkage](#raylib-linkage) below —
  this is a hard requirement for the hot-reload configuration, not a preference.
- GNU Make, if you use the `Makefile` rather than `build.sh`. `build.sh` and `build.bat`
  need no make at all.

### MSYS2 (the documented Windows setup)

```bash
pacman -S mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-raylib mingw-w64-ucrt-x86_64-pkg-config make
```

With that in place, `pkg-config raylib` resolves and both the Makefile and `build.sh` use it
automatically. Nothing further is needed.

### Windows without MSYS2 (the documented fallback)

When `pkg-config` cannot find raylib, the build falls back to `$RAYLIB_DIR`, default
`vendor/raylib`, laid out as:

```
vendor/raylib/
├── include/   raylib.h, raymath.h, rlgl.h, rcamera.h, rgestures.h
├── lib/       libraylib.dll.a   (the DLL import library, NOT the static libraylib.a)
└── bin/       raylib.dll
```

To produce it from the raylib 6.0 sources with a MinGW-w64 toolchain:

```bash
make -C /path/to/raylib/src PLATFORM=PLATFORM_DESKTOP RAYLIB_LIBTYPE=SHARED RAYLIB_RELEASE_PATH=.
```

That emits `raylib.dll` and `libraylibdll.a`. Copy the headers into
`vendor/raylib/include/`, `raylib.dll` into `vendor/raylib/bin/`, and `libraylibdll.a` into
`vendor/raylib/lib/` **renamed to `libraylib.dll.a`** — MinGW's linker searches
`libraylib.dll.a` before `libraylib.a`, so that name is what makes `-lraylib` resolve to the
import library rather than a static archive.

Point the build somewhere else with `RAYLIB_DIR=/some/path ./build.sh` if you prefer.

`vendor/` is gitignored: it is a local build artifact, not source.

The build scripts also probe common MinGW-w64 install locations if `gcc` is not on `PATH`
(chocolatey's `mingw`, `C:\msys64\ucrt64`, `C:\msys64\mingw64`, `C:\mingw64`), so a
toolchain that was installed without a `PATH` entry still works.

## Building

`build.sh` (POSIX shell, including Git Bash) and `build.bat` (cmd.exe) are equivalent. The
`Makefile` exposes the same configurations as named targets.

```bash
./build.sh              # hot-reload development build
./build.sh --release    # single executable, no hot reload
./build.sh --tests      # headless test executable
./build.sh --clean
```

```bash
make debug              # build/game.dll + drifty.exe
make release            # drifty_release.exe, DRIFTY_HOT_RELOAD undefined, no game module
make tests              # drifty_tests.exe
make run-tests          # build and run them
make clean
make info               # print the resolved compiler and raylib linkage
```

Every one of these terminates immediately. Nothing in this repository starts a watcher, a
daemon, or a long-lived process.

### Running

```bash
./drifty.exe            # development build; start it once and leave it running
./drifty_release.exe    # release build
./drifty_tests          # headless tests; run from the repository root
```

`drifty_tests` writes CSV telemetry to `telemetry/` relative to the working directory, so
run it from the repository root. It accepts `--list`, `--scenario NAME`, and `-v`.

The release build produces a single executable with no `game.dll`. It still links raylib,
which with the vendored fallback means `raylib.dll` must sit next to the executable; the
build scripts copy it there for you.

## Hot-reload workflow

The game is a thin platform layer (`drifty.exe`) plus a hot-reloadable game module
(`build/game.dll`). The platform layer owns the window, the raylib context, the `Game`
allocation, and the fixed-timestep loop. Everything else lives in the module.

1. Run `./drifty.exe` once and leave it open.
2. Edit game code.
3. Run `./build.sh`. It rebuilds the module always, rebuilds the executable only when it is
   not already running, and returns in well under a second.
4. The running game notices the new module and swaps it in, keeping its position, counters,
   and checksum.

The loader never unloads a working module until a replacement has been proven good: it
copies the new module to a uniquely named load target, loads it, resolves every entry point
into a temporary table, and only then calls the old module's `game_pre_reload`, publishes
the new table, and calls `game_post_reload`. **A compile error cannot close the running
game** — the build script links to a temporary filename and only renames it into place on
success, so a failed build leaves the previous module untouched, and a module that fails to
load or is missing a symbol is rejected with the previous one still active.

### When a restart is required

Hot reload cannot handle these. Restart `drifty.exe` after:

- A change to the layout of `Game` or anything it contains. The existing memory block
  becomes invalid and the reloaded module reads garbage.
- A change to `src/main.c`, `src/timestep.c`, or the `hotreload_*` files — the platform
  layer is not reloadable.
- A change to `GAME_ENTRY_POINTS`.

The deterministic input recording exists partly to make restarts cheap: the ring buffer in
`Game` holds the last 60 seconds of input at 120 Hz and can be replayed to return to the
moment of interest.

### Reload-safety rules for game code

These are correctness requirements, not style. Violating them produces crashes that only
appear after a reload.

- No pointer stored in `Game`, or reachable from it, may point into the module's code or
  static data. Module-static tables are referenced by id and resolved through an accessor at
  point of use, never cached as a pointer.
- No function pointers in persistent state. Rebuild any needed table in `game_post_reload`.
- The `Game` block is allocated and owned by the platform layer. Never declare
  `static Game game;` inside the module.
- Anything raylib tracks — textures, sounds, audio stream callbacks — is released in
  `game_pre_reload` and re-acquired in `game_post_reload`.

## raylib linkage

raylib keeps its state in **global variables**. If `game.dll` links raylib statically, the
module owns that state and reloading the module destroys it; the next raylib call crashes.
The hot-reload configuration therefore requires a shared raylib, and the MSYS2 package ships
both a static `libraylib.a` and a DLL import library, so which one `-lraylib` resolved to is
worth checking rather than assuming:

```bash
objdump -p build/game.dll | grep -i "DLL Name"
```

`raylib.dll` must appear in that list. If it does not, `-lraylib` found the static archive
and hot reload will crash on the first raylib call after a swap. The same check on
`drifty_tests` must show **no** raylib entry at all: the headless harness reaches raylib.h
for the `Vector2` type and links none of the library.

Release builds are unaffected by this and may link raylib statically.

## Known limitations

- **The POSIX loader (`src/hotreload_posix.c`) is compile-ready but unverified.** Development
  happens on Windows. It mirrors the Windows loader exactly, but the first Linux or macOS run
  should be treated as a bring-up task rather than a regression.
- **Changing the `Game` struct layout requires a restart.** This is inherent to the
  technique, not a defect.
- **`drifty_tests` validates infrastructure only.** No tire, vehicle, drivetrain, or
  load-transfer behaviour is asserted, because none exists yet. The twelve physics scenarios
  in docs/SPEC.md arrive with the code they exercise, from Phase 1 onward.
- **`tests/baselines/` is intentionally empty.** Phase 0 telemetry has no regression value,
  so it is not committed as a baseline.
- The `Makefile` is exercised with GNU Make 3.76 and 4.4.1. Very old releases print harmless
  `$(shell)` noise on stderr.

## Layout

```
Makefile, build.sh, build.bat   build entry points; all terminate immediately
docs/SPEC.md, docs/SOURCES.md   specification and reference index
src/main.c                      platform layer: window, Game allocation, fixed-timestep loop
src/timestep.h/.c               the accumulator, isolated so the harness can assert it
src/hotreload.h                 GAME_ENTRY_POINTS, the one authoritative entry-point list
src/hotreload_windows.c         LoadLibrary / GetProcAddress loader (primary target)
src/hotreload_posix.c           dlopen / dlsym loader (unverified)
src/game.h/.c                   the Game block and the reloadable entry points
src/config.h                    Phase 0 constants, every one unit-bearing
src/units.h                     world<->render conversion and the coordinate convention
src/math_utils.h/.c             scalar helpers raymath.h does not provide
src/input.h/.c                  held controls and one-shot commands
src/replay.h/.c                 deterministic fixed-tick input timeline
src/telemetry.h/.c              CSV row writer, no raylib dependency
tests/physics_tests.c           headless scenario runner
tests/baselines/                committed CSV baselines (empty until Phase 1)
telemetry/                      generated CSV output (gitignored)
vendor/                         locally built raylib (gitignored)
```
