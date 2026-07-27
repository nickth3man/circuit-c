# CLAUDE.md

**Read [AGENTS.md](AGENTS.md) before doing any work in this repository.** It is the
authoritative guide for this project and covers the hot-reload development workflow, the
build commands, the physics conventions, and the constraints that keep hot reload working.

`docs/SPEC.md` is the full specification. `docs/SOURCES.md` is its reference index.
`docs/PHASE3_VALIDATION.md` records the accepted handling metrics, baseline policy, and
Phase 1–3 acceptance evidence.
`docs/DEVTOOLS.md` covers the development shell — the Physics Lab, the replay inspector,
failure bundles, telemetry reports, and the one-command make targets. `docs/CI.md` covers the
workflows and the required checks.

**Windows only — MSYS2 UCRT64.** Use `build.bat` from cmd.exe, or `./build.sh` from an
MSYS2 UCRT64 shell. Phases 0–3 are complete; Phase 4 is an optional, deliberate upgrade.

## Read this before running anything

These rules exist because breaking them hangs the session rather than failing loudly:

- **Never start, launch, or supervise `drifty.exe` for interactive sessions.** The
  developer runs it and leaves it running. Your job is to rebuild the game module with
  `build.bat` / `./build.sh`, which always returns in under a second. The running game
  picks up the change on its own.
- **Exception:** `build.bat --smoke-test` and `drifty.exe --capture-scene NAME` are allowed;
  both are bounded and exit on their own. `mk screenshots` and `mk visual-test` use the
  latter.
- **`mk run` is the same trap under another name.** It launches the game. Never invoke it.
- **Never run a file watcher or any command that does not return on its own** — no
  `watchexec`, no `nodemon`, no `--watch` flags.

For physics and tuning work, prefer the headless loop, which needs no window at all:

```bash
./build.sh --tests && ./drifty_tests.exe
```

or, for one scenario with a report:

```bash
mk report NAME=skidpad
```

The Phase 2 canonical vehicle structures, and then the development-tool state (`DevState`),
each changed the persistent `Game` layout. Restart `drifty.exe` once after updating; ordinary
module-only hot reload preserves body state, wheel speeds, engine RPM, and gear after that.

Everything else — restart triggers, reload-safety constraints on game code, unit and sign
conventions — is in [AGENTS.md](AGENTS.md).
