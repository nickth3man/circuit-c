# CLAUDE.md

**[AGENTS.md](AGENTS.md) is the authoritative guide for this repository. Read it before
doing any work here.** This file exists only so that tools looking for `CLAUDE.md` find the
pointer; it deliberately duplicates nothing, because two copies of a rule is one copy that
goes stale.

AGENTS.md covers, in order: the document index, the current phase of both workstreams, the
toolchain, the hot-reload development workflow and the commands that must never be run, the
headless test loop, restart triggers, reload-safety constraints, the vehicle-appearance
contract, the parameter-registry procedure, and the physics conventions.

Four things there are worth knowing before you run anything, because breaking them hangs the
session rather than failing loudly:

- **Never launch or supervise `build/dev/drifty.exe`** — the developer keeps it running; you rebuild
  the module with `build.bat` and the running game swaps it in.
- **Never invoke `mk run` or `mk inspect`** — neither returns.
- **Never run a file watcher**, or any command that does not exit on its own.
- **Prefer the headless loops**: `./build/tests/drifty_tests.exe` for physics, `mk visual-diagnose` for
  vehicle appearance.
