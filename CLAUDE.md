# CLAUDE.md

**Read [AGENTS.md](AGENTS.md) before doing any work in this repository.** It is the
authoritative guide for this project and covers the hot-reload development workflow, the
build commands, the physics conventions, and the constraints that keep hot reload working.

`docs/SPEC.md` is the full specification. `docs/SOURCES.md` is its reference index.

## Read this before running anything

These two rules exist because breaking them hangs the session rather than failing loudly:

- **Never start, launch, or supervise `drifty.exe`.** The developer runs it and leaves it
  running. Your job is to rebuild the game module with `./build.sh`, which always returns in
  under a second. The running game picks up the change on its own.
- **Never run a file watcher or any command that does not return on its own** — no
  `watchexec`, no `nodemon`, no `--watch` flags.

For physics and tuning work, prefer the headless loop, which needs no window at all:

```bash
./build.sh && ./drifty_tests --scenario skidpad
```

Everything else — restart triggers, reload-safety constraints on game code, unit and sign
conventions — is in [AGENTS.md](AGENTS.md).
