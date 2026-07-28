# Continuous integration

Everything CI does can be run locally with the same command. That is the design rule: if a
check exists in a workflow, it exists as a make target, and `mk ci` runs the set.

```bash
mk ci
```

## Workflows

| Workflow | Trigger | What it protects |
|---|---|---|
| `ci.yml` | every push to `main`, every pull request | formatting, static analysis, the Linux headless build on two compilers, the real Windows build and its DLL linkage, sanitizers, coverage |
| `physics-regression.yml` | pull requests touching physics, vehicle, tire, drivetrain, config, scenarios, or the comparison tool | the car still behaves the same, compared against the merge base built on the same runner |
| `codeql.yml` | push, pull request, weekly | security and quality queries over the headless build |
| `release.yml` | `v*` tags | clean release build, full suite, packaged archives with SHA-256 checksums |
| `fuzz.yml` | nightly, on demand, and pull requests touching the fuzzed code | the parsers and the pure tire functions |
| `performance.yml` | pull requests touching the simulation | an order-of-magnitude throughput regression |
| `nightly.yml` | nightly | long stress runs, the replay corpus, full static analysis |

## Required checks

Configure these as required in the `main` ruleset:

```
quality (format, static analysis, workflow lint)
linux headless (gcc)
linux headless (clang)
windows (MSYS2 UCRT64)
sanitizers (ASan + UBSan)
scenarios vs merge base
```

`scripts/setup_ruleset.sh` sends exactly that, plus a required pull request, linear history,
blocked force pushes, and blocked branch deletion. It prints the payload by default and only
changes anything with `--apply`. Nothing runs it for you.

Coverage starts **informational**. Make it required once there are enough meaningful tests
that a percentage means something; the useful rule before that is "no decrease in
`physics.c`, `tire.c`, `drivetrain.c`, `vehicle.c`, and every new physics branch has a test".

## Why the physics comparison is against the merge base, not a committed file

Two builds of identical source produce identical telemetry only when the compiler, the libm,
and the flags are identical. The committed baselines in `tests/baselines/` were recorded on
Windows with MSYS2 GCC; comparing a Linux Clang build against them would flag the toolchain,
not the change.

So the gate builds **the pull request head and its merge base on the same runner** and
compares those. The committed baselines remain the local, same-machine reference that
`mk regression` uses, and CI reports them informationally.

The comparison itself is tolerance-aware and split into three classes:

- **exact** — tick count, gear sequence, wheel-lock transitions
- **tolerance** — position, velocity, force, load, yaw rate, slip angle
- **derived** — 0–100 km/h, peak sideslip, peak yaw rate, steady radius, transition duration,
  recovery time

The state checksum is reported but never gates: it is meaningful only between two runs of the
same binary.

Phase 3's reviewed local set is `scenario_accel-load`, `scenario_brake-load`,
`scenario_coast-down`, `scenario_skidpad`, `scenario_step-steer`, `scenario_lift-off`,
`scenario_transition`, and `scenario_catchable-drift`. Their derived-metric tolerances remain
the same tolerance-aware classes above; CI still compares head against merge base on one
runner and does not require cross-compiler float equality.

## Baseline changes

A pull request that changes `tests/baselines/` must explain, in words, which physical change
moved the numbers and why the new ones are correct. Re-recording a baseline is not a way to
make a failing check green, and the pull request template asks for that justification
explicitly.

## Agent workflow

For a solo project using coding agents, a human approval on every pull request is friction
without benefit. The arrangement that works:

1. the agent creates a branch and a pull request;
2. every required check runs;
3. a baseline change requires a written explanation;
4. auto-merge is allowed once all checks pass;
5. the agent responds to CI feedback until green;
6. direct pushes to `main` stay blocked.

## Security posture

Deliberately boring, and all of it is visible in the workflow files:

- explicit minimal `permissions:` on every workflow, raised per job only where needed
  (`security-events: write` for CodeQL, `contents: write` for the release publish job);
- actions pinned to full commit SHAs — the only immutable reference GitHub offers — with
  Dependabot opening weekly updates;
- no third-party publishing action: the release uses the `gh` CLI already on the runner;
- `actionlint` is downloaded by version and verified against a recorded SHA-256 before it
  runs;
- no `pull_request_target`, and no downloading or executing of untrusted pull request
  artifacts;
- no secrets are exposed to pull request code — the workflows use none.

## Downloadable artifacts

Every run uploads what a human would need to diagnose it without reproducing it:

| Artifact | From | Keeps |
|---|---|---|
| `telemetry-linux-<compiler>` | linux headless | scenario CSVs and everything under `artifacts/` | 7 days |
| `vehicle-gallery-<compiler>` | linux headless | the headless vehicle contact sheet, plus any distinctness failure bundle | 14 days |
| `windows-failure` | windows | telemetry and failure bundles, on failure only | — |
| `sanitizer-failure` | sanitizers | failure bundles, on failure only | — |
| `coverage` | coverage | the gcovr HTML and Cobertura report | — |

The vehicle gallery is uploaded separately from the telemetry bundle on purpose: it is the
artifact someone actually downloads to *look* at the fleet, and burying it under a hundred
CSVs makes it useless. It uploads on success and failure alike, because a distinctness
failure is exactly when the pictures are needed.

The same job also regenerates `docs/CORPUS.md` and diffs it against the committed copy, so
the generated corpus table cannot quietly rot away from the code that produces it.

## What is deliberately not in CI

**The in-game vehicle gallery.** `mk gallery` needs a GPU and produces a human-review
artifact, not a gate. A hundred cars behind an RMSE comparison, on hardware that rasterizes
differently per vendor, is a maintenance sinkhole with no CI value. The headless contact sheet
and the `corpus` scenario are the gates, and both run here.

**Visual regression.** raylib renders through OpenGL and rasterisation differs enough between
GPU vendors and drivers to make a pixel comparison flaky; hosted Windows runners have no GPU
at all. `mk visual-test` runs on the developer's machine instead. See `tests/visual/README.md`.

**The interactive game.** Nothing in CI launches `drifty.exe` beyond the bounded
`--capture-scene` and `--smoke-test` modes, which exit on their own.
