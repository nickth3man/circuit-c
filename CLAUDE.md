# CLAUDE.md

**Read [AGENTS.md](AGENTS.md) before doing any work in this repository.** It is the
authoritative guide for this project and covers the hot-reload development workflow, the
build commands, the physics conventions, and the constraints that keep hot reload working.

`docs/SPEC.md` is the full specification. `docs/SOURCES.md` is its reference index.

**Windows only — MSYS2 UCRT64.** Use `build.bat` from cmd.exe, or `./build.sh` from an
MSYS2 UCRT64 shell. Phase 1 is complete; do not begin Phase 2 during maintenance tasks.

## Read this before running anything

These rules exist because breaking them hangs the session rather than failing loudly:

- **Never start, launch, or supervise `drifty.exe` for interactive sessions.** The
  developer runs it and leaves it running. Your job is to rebuild the game module with
  `build.bat` / `./build.sh`, which always returns in under a second. The running game
  picks up the change on its own.
- **Exception:** `build.bat --smoke-test` is allowed; it is bounded and exits on its own.
- **Never run a file watcher or any command that does not return on its own** — no
  `watchexec`, no `nodemon`, no `--watch` flags.

For physics and tuning work, prefer the headless loop, which needs no window at all:

```bash
./build.sh --tests && ./drifty_tests.exe
```

The Phase 1 `Game` layout embeds the canonical vehicle structures. Restart `drifty.exe`
once when moving from a Phase 0 executable to this layout; ordinary module-only hot reload
preserves vehicle state after that.

Everything else — restart triggers, reload-safety constraints on game code, unit and sign
conventions — is in [AGENTS.md](AGENTS.md).
