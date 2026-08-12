# Release Gate (issue #60)

This file is the core acceptance matrix and release checklist for Circuit. It documents what
"the product works" means in observable, reproducible terms, how to run each check, and the
compatibility policy for artifacts, schemas, and content.

## Identity

Player-facing product: **Circuit** (a deterministic 2D racing simulator). The full rename
audit from Drifty/drift-game identity is documented in `docs/IDENTITY.md`; the only surviving
references are that allowlist (historical docs) and legacy cleanup entries in `.gitignore`,
`Makefile`, and `build.sh` that remove stale old-named artifacts.

## Supported platforms and toolchain

| Platform | Build | Test | Interactive | Notes |
|---|---|---|---|---|
| Windows (MSYS2 UCRT64) | supported | supported | supported | primary dev target; raylib static, glfw3.dll runtime dep |
| Linux | supported (CI) | supported | headless renderer/audio-independent | interactive GLFW path builds where raylib is installed |

Dependency policy: raylib is the only required third-party runtime; GLFW ships as
`glfw3.dll` beside the release executable. Everything else is stdlib. Third-party notices:
`third_party/README.md` (shipped as `THIRD_PARTY_NOTICES.txt` in the bundle).

## Core acceptance matrix

| # | Check | Command | Pass criterion |
|---|---|---|---|
| 1 | Identity audit | `grep -ri drifty docs README.md` (allowlist-aware) | No player-facing Drifty/drift-score language |
| 2 | Clean debug build | `make dev` | Exit 0 |
| 3 | Clean release build | `make release` | Exit 0 |
| 4 | Full test gate | `make CIRCUIT_STRICT=1 verify` | format + lint + 178 scenarios + 46/46 regression baselines |
| 5 | Content validation | `make validate-tracks` | Every shipped track prints `TRACK-VALIDATE ok` |
| 6 | Parameter truthfulness | `make params` | `docs/VEHICLE_PARAMETERS.md` regenerates and the `param-audit` scenario passes |
| 7 | Headless acceptance demo | `make acceptance` | `DEMO-RACE ok` digest line + `DEMO-RACE retry ok`, exit 0 |
| 8 | Multi-car determinism/performance | `make benchmark-multi` | 120 Hz budget with headroom (issue #45 reference: ~177 µs/tick at 8 cars) |
| 9 | Packaging | `make package` | Bundle with exe + DLL + data/ + notices + sha256 manifest |
| 10 | Clean-install smoke | `make package-smoke` | Packaged exe from an unrelated CWD exits 0 |
| 11 | Release evidence | `make release-evidence` | Archived regression report, acceptance digest, track hashes, package manifest |
| 12 | Session modes | `make scenario NAME=session-config` | Config validation, time trial + AI race launch |
| 13 | AI race loop | `make scenario NAME=race-classification` | Live order, finishing window, DNF, fastest lap, immutable results |
| 14 | Pit service | `make scenario NAME=pit-cycle` | Entry→box→service→exit with limiter + penalties |
| 15 | Persistence | `make scenario NAME=championship` + `player-profile` | Versioned save round-trip + version gate |

## Interactive walkthrough

The target walkthrough (a fresh player without console/CLI):

1. Launch `circuit_release.exe` from the bundle.
2. Configure: car (FWD/RWD/AWD), track, race mode, AI field, rules, environment — defaults
   produce a valid session on a clean install (issue #48).
3. Grid + countdown: cars hold on a staggered grid and release on the same fixed tick
   (issue #49).
4. Race multiple AI with contacts, track limits, and optional pit service (issues #27/#55/#57).
5. Finish and classify: results with gaps, fastest lap, DNF/DSQ, penalties (issue #54).
6. Retry or next event; records commit only after finalization (issues #56/#58).
7. Exit and relaunch: the profile persists versioned settings/records (issue #47).

The headless equivalent of this walkthrough is `make acceptance`; the bounded interactive
smoke is `make package-smoke` (120 frames, exits itself).

## Compatibility policy

- **Executable/artifact names**: `circuit[_release|_tests][.exe]` are the only names shipped.
- **Content schemas**: track manifests carry `schema: circuit/track` + `version`;
  vehicle manifests carry their own schema/version; unknown or incompatible versions are
  rejected with the offending id/version named.
- **Saves/profiles**: `player_profile` is versioned with atomic writes and a `.corrupt`
  recovery path; championship saves validate the rule version before accepting.
- **Replays**: recorded against content hashes; a replay whose content hash does not match the
  loaded content is refused.
- **Release artifacts**: every bundle ships `MANIFEST.json` (per-artifact sha256), the build
  commit/branch/dirty flags compiled into the binary, and `THIRD_PARTY_NOTICES.txt`.

## Release checklist

- [ ] `make CIRCUIT_STRICT=1 verify` green (format, lint, 178 scenarios, 46/46 regression)
- [ ] `make validate-tracks` green on every shipped track
- [ ] `make acceptance` prints `DEMO-RACE ok` and `retry ok`
- [ ] `make benchmark-multi` meets the documented 120 Hz entrant budget
- [ ] `make package` + `make package-smoke` pass from an unrelated directory
- [ ] `make release-evidence` produced and reviewed (`artifacts/release-evidence/`)
- [ ] Third-party notices and the sha256 manifest are in the bundle
- [ ] No player-facing Drifty/drift identity outside the documented allowlist
- [ ] All blocking child issues closed or explicitly descoped

## Known limitations (documented, not hidden)

- Physics is a reduced-order 2D model (tires, suspension, drivetrain, aero as documented in
  `docs/SIMULATION_OWNERSHIP.md` and `docs/VEHICLE_PARAMETERS.md`) — not a full 3D
  simulation. Every parameter states its units and truthfulness.
- Interactive Linux support follows the raylib platform; headless mode never depends on a
  renderer or audio device.
- Network multiplayer is post-core (issue #59): the lockstep model is documented in
  `docs/MULTIPLAYER.md`; no network service is a prerequisite for offline play.
