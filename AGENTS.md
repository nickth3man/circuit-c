# Drifty — AGENTS.md

Top-down 2D drift driving simulator written in C with raylib 6.0.

## Project Structure

See `SPEC.md` for the full specification, physics model, data structures, and incremental build plan.

## Cloned Dependency Source

Read-only dependency source repositories are available under
`.slim/clonedeps/repos/` for inspection. Do not edit these clones.

- `.slim/clonedeps/repos/raysan5__raylib/` — raylib at `6.0`; header/type reference, struct layout verification, 215 examples.
- `.slim/clonedeps/repos/unconv__racer/` — raylib drift game at `5d938cf`; skidmark rendering and Camera2D follow patterns (rendering only, not the physics model).
