# Development tooling

The shipped game stays plain C and raylib. Everything in this document is the *shell* around
it: tuning, replay, analysis, regression comparison, profiling, and verification. None of it
changes how the car drives.

Three pieces do most of the work, and they are designed to be used together:

- the **Physics Lab** gives you fast subjective tuning,
- the **replay inspector** gives you exact objective evidence,
- the **HTML telemetry report** gives you an artifact a human or an agent can read later.

---

## 1. The Physics Lab

Press **F2** in the running game.

```
F2  show/hide the lab          F6  single step (Shift+F6 for ten)
F3  show/hide the inspector    F7  cycle time scale
F5  pause / resume             F8  capture the current run as the baseline ghost
```

The lab is built with [raygui](../third_party/README.md), vendored from the raylib source
tree this project already builds against. It is compiled into `build/dev/game.dll` only when
`DRIFTY_DEV_TOOLS` is defined — the default for development builds, never for
`build.sh --release` or the headless tests.

What it contains:

- **Scenario selector** — the same table the headless runner uses (`src/dev/dev_scenario.c`):
  free drive, accel, skidpad, step-steer, lift-off, power-oversteer, handbrake-entry,
  transition, brake-corner, coast-down. Start resets the simulation, rewinds the recording,
  and clears the scope, so a run always begins from a known state.
- **Time control** — pause, single step, ten steps, and a 0.05x–4x time scale. The lab writes
  three fields; the platform loop in `main.c` is the only thing that reads them, so a stepped
  tick is bit-identical to a free-running one.
- **Live sliders** for every tunable, generated from the registry, with the current value,
  the default, the unit, the allowed range, and a per-parameter reset button. A modified
  parameter is marked with `*`.
- **Tuning profiles** — named, saved to `data/vehicles/<name>.txt`, loaded back, or reset wholesale.
- **Overlay toggles** — forces, velocity, slip, loads, contact points, trajectory, ghost,
  scope.
- **A scrolling scope** with eight channels: body sideslip, yaw rate, steering, throttle,
  front and rear slip angle, friction usage, and speed. The captured baseline is drawn behind
  the live trace.
- **The seed, tick, checksum, and modified-parameter count** on one line — the run's identity.
- **A red invariant panel** that latches the first violation with the failing value in it.

### The parameter registry

Every tunable is described exactly once, in `src/dev/dev_params.c`, and that one description
generates the sliders, the profile format, the reset behaviour, the telemetry metadata, and
[docs/generated/PARAMETERS.md](generated/PARAMETERS.md).

```c
typedef struct {
    const char *name;       /* stable dotted key, e.g. "tire.lat_front.mu" */
    const char *group;
    const char *unit;
    size_t      offset;     /* byte offset into VehicleSpec — not a pointer */
    float       defaultValue;
    float       minimum;
    float       maximum;
    float       step;
    bool        requiresRestart;
    const char *description;
} DevParameter;
```

Offsets rather than pointers, so the table is `static const` and nothing in it can outlive a
hot reload. The `params` scenario asserts every declared default against
`vehicle_spec_set_default()`, so `config.h` and the registry cannot drift apart.

Regenerate the documentation table after changing the registry:

```bash
mk params-doc
```

---

## 2. The replay inspector

Press **F3**. The input timeline that `src/game/replay.c` already records becomes something you
can scrub.

- a timeline with event markers — throttle, brake, handbrake, shifts, resets, steering
  reversals;
- frame-at-a-time stepping, jump to start, jump to live;
- **jump to the first invariant violation**;
- the input at the cursor, and the live-versus-baseline delta for sideslip and yaw rate;
- save and load `replays/<name>.bin`;
- **export a failure bundle** from the current state.

Replay files are versioned and defensively parsed (`src/dev/dev_replay.c`); `fuzz/fuzz_replay.c`
exists precisely because these files travel between builds.

---

## 3. Failure bundles

When a headless scenario fails, the runner writes everything needed to reproduce it:

```
artifacts/failure-<scenario>-<timestamp>/
├── replay.bin           the exact input timeline
├── telemetry.csv        the run's telemetry, copied verbatim
├── summary.json         scenario, first failing tick, checksum, counts, build facts
├── config_snapshot.txt  every tunable's value at failure time
├── git_info.txt         commit, branch, dirty flag, compiler, flags, platform, build mode
└── failure.txt          the failing check, verbatim
```

`git_info.txt` is the point: a bug is actionable without reconstructing the environment that
produced it. The build scripts pass the commit and dirty flag in as compiler defines, so the
binary knows where it came from.

Disable with `--no-bundle`, relocate with `--artifacts DIR`.

---

## 4. Telemetry tooling

Standard library Python only — no package install, on any machine, in any CI job.

```bash
python tools/telemetry/summarize_run.py artifacts/telemetry/scenario_skidpad.csv
python tools/telemetry/compare_telemetry.py tests/baselines/scenario_skidpad.csv artifacts/telemetry/scenario_skidpad.csv
python tools/telemetry/plot_telemetry.py artifacts/telemetry/scenario_skidpad.csv --out artifacts/plots
python tools/telemetry/make_report.py artifacts/telemetry/scenario_skidpad.csv \
    --baseline tests/baselines/scenario_skidpad.csv --out artifacts/report.html
```

or, the one-liner that runs the scenario first:

```bash
mk report NAME=skidpad
```

The report is a single self-contained HTML page: trajectory, speed, steering and yaw rate,
sideslip, slip angles and ratios, wheel speed versus ground speed, normal loads, tire forces,
friction usage, engine and gear, torques — plus the derived-metric table, the exact
comparisons, and the largest column deviations. Charts are inline SVG; there is no script and
no external request.

### Comparison classes

Formatting changes and last-bit float noise must not fail a run; a real change in the model
must. So the comparison is split:

| Class | What | How |
|---|---|---|
| exact | tick count, gear sequence, wheel-lock transitions | any difference is a difference |
| tolerance | position, velocity, force, load, yaw rate, slip angle | `atol + rtol * |baseline|`, per column |
| derived | 0–100 km/h, peak sideslip, peak yaw rate, steady radius, transition duration, recovery time | per-metric thresholds |

The state checksum is compared but never gates — it is only meaningful between two runs of
the same binary on the same toolchain.

---

## 5. One command per operation

```
mk dev              build the hot-reload module        mk test            fast scenarios
mk test-physics     every scenario, with telemetry     mk scenario NAME=skidpad
mk report NAME=skidpad                                 mk regression      compare to baselines
mk baselines        re-record baselines (explain it)   mk verify-fast     format + tests
mk verify           analysis + tests + regression      mk ci              the required CI set
mk sanitize         ASan + UBSan (clang)               mk coverage        gcovr text/HTML/XML
mk screenshots      deterministic scene captures       mk visual-test     compare to baselines
mk gallery          the in-game vehicle corpus pages   mk profile         build with profiling
mk benchmark        throughput
mk params-doc       regenerate PARAMETERS.md           mk compile-commands  for clangd
mk format           apply .clang-format                mk lint / mk analyze
mk fuzz             build and briefly run the fuzzers  mk release
```

`mk.bat` enters MSYS2 UCRT64 for you from cmd.exe or PowerShell; from a UCRT64 shell run
`make <target>` directly. Every target terminates on its own **except `mk run`**, which
launches the game — that one is the developer's, never an agent's.

Tools that are not installed produce a `SKIP` line with the install command rather than a
shell error. CI installs all of them, so nothing is skipped where it matters.

---

## 6. Editor setup

```bash
mk compile-commands
```

`compile_commands.json` is generated from the same source lists and flags the build scripts
use, because Bear cannot intercept the MSYS2 build from a Windows shell and a *wrong*
compilation database is worse than none. Each translation unit gets the flags of the
configuration it is really built in.

`.vscode/` carries shared `tasks.json`, `launch.json`, `settings.json`, and
`extensions.json`: build, test, run one scenario, generate a report, full verification, and
five debug configurations — including **attach to the running game**, which is the one that
matters when the developer has left `build/dev/drifty.exe` open.

`.clang-format` encodes the style the codebase already uses. Read the adoption note at the
top of it before running `mk format` over the tree: the CI check starts advisory on purpose.

---

## 7. Profiling

```bash
mk profile
```

Zones are declared once and serve both backends:

```c
DRIFTY_ZONE_BEGIN(physics, "Physics");
physics_fixed_update(...);
DRIFTY_ZONE_END(physics);
```

- **default** — the macros compile to nothing.
- **`-DDRIFTY_PROFILE`** — the built-in accumulating timers in `src/game/profile.c`; a table of
  calls, total, mean, and worst is printed at shutdown. No dependency at all.
- **`-DDRIFTY_TRACY`** — the same call sites forwarded to Tracy. Vendor the distribution into
  `third_party/tracy/` (so that `third_party/tracy/public/TracyClient.cpp` exists) and
  `mk profile` picks it up automatically. Tracy is never part of a release build.

Currently instrumented: `FixedUpdate`, `Physics`, `Render`, `PhysicsLab`, and the frame mark.
Finer zones inside `physics_fixed_update` — steering, load transfer, tire forces, combined
slip, integration — are one macro pair each, added where you need them.

---

## 8. Deterministic screenshots

```bash
mk screenshots     # artifacts/screenshots/*.png
mk visual-test     # RMSE comparison against tests/visual/baseline/
```

`build/dev/drifty.exe --capture-scene NAME` does not run the normal frame loop: it steps the simulation
with an exact fixed dt, draws one frame at interpolation alpha 0, writes a PNG, and exits.
raygui controls are locked during a capture so the image does not depend on where the mouse
happens to be. The four scenes are `debug_overlay`, `tire_curves`, `drift_hud`, and
`physics_lab`; `--list-scenes` prints them.

This gate is local rather than CI — see `tests/visual/README.md` for why.

---

## 9. The vehicle corpus and gallery

Appearance is derived from physics parameters alone — see
[CAR_VISUAL.md](CAR_VISUAL.md) for the grammar and [CORPUS.md](generated/CORPUS.md) for the fleet.

The headless path needs no GPU, no window, and no `build/dev/drifty.exe`, so it works on any machine
and in CI:

```bash
./build/tests/drifty_tests.exe --scenario car-visual                 # grammar gates
./build/tests/drifty_tests.exe --scenario corpus                     # fleet gates
./build/tests/drifty_tests.exe --dump-corpus-sheet artifacts/gallery # contact sheet + index.html
./build/tests/drifty_tests.exe --dump-corpus-index docs/generated/CORPUS.md    # the corpus table
./build/tests/drifty_tests.exe --generate-corpus data/vehicles/corpus       # export the fleet as tuning profiles
```

`artifacts/gallery/index.html` is the primary human acceptance check for the appearance
system. Every cell is drawn at ONE metres-to-pixels scale — auto-fitting each car would erase
the size axis, which is the most informative thing the sheet shows.

The in-game path renders the same fleet through the production texture path:

```bash
mk gallery                    # all pages -> artifacts/gallery-ingame/page_N.png
build/dev/drifty.exe --gallery-page 3   # one page; bounded and self-exiting, like every capture here
```

The gallery is a human-review artifact and deliberately **not** a GPU regression baseline: a
hundred cars behind an RMSE gate, on hardware that rasterizes differently per vendor, is a
maintenance sinkhole with no CI value. The headless sheet and the `corpus` scenario are the
gates that matter.

When two corpus vehicles are too similar, the `corpus` scenario writes
`artifacts/car_visual_failures/<pair>/` containing both rasters, a diff image, both specs as
loadable tuning profiles, and a report naming the closest signature components. Suppressed by
`--no-bundle`, like every other failure bundle here.

---

## 10. Fuzzing

```bash
mk fuzz            # clang + libFuzzer, 20 seconds per target
```

Three targets, all over pure deterministic code: `fuzz_profile` (the tuning-profile parser),
`fuzz_replay` (the replay reader), `fuzz_tire` (the tire curves and the friction ellipse).

Physics is fuzzed for **properties**, never for exact values, because the exact value is the
thing under development:

- finite inputs produce finite outputs;
- zero slip produces zero force;
- the limited force pair stays inside the friction ellipse;
- normal loads never become negative grip;
- an accepted profile yields a valid spec, and a rejected one changes nothing.

Each target also builds as a plain executable with a built-in corpus, so it is useful on
Windows where libFuzzer is not available.

---

## What is deliberately absent

- **Dear ImGui / cimgui** — powerful, but raygui is the natural fit for a pure C raylib
  project and needs no C++ toolchain.
- **A telemetry server (Grafana, Prometheus)** — excessive for local deterministic CSV runs.
- **A large C unit-test framework** — the custom headless runner is enough until discovery,
  fixtures, or mocking become painful.
- **A build-system rewrite** — `build.sh` stays canonical; the Makefile delegates to it rather
  than reimplementing the hot-reload-safe link sequence.
- **Strict pixel-perfect tests across platforms** — flaky by construction.
