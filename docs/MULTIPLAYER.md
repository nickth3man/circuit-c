# Multiplayer (issue #59)

Multiplayer is **post-core**: the project's target is AI competition, and every multiplayer
change here must preserve offline/headless behaviour bit-for-bit. This document records the
current local-multiplayer support and the selected network synchronization model.

## Status

| Layer | Status |
|---|---|
| Multiple local human entrants with independent input sources | Implemented + tested (`local-multiplayer` scenario) |
| Independent classified results for all human entrants | Implemented (shared session authorities) |
| Network transport, lobby, join/leave | Not implemented (post-core; see the model below) |
| Desync detection over the network | Design specified; determinism already proven headlessly |

## Local multiplayer

Two or more human entrants can race in one session:

- `game_set_entrant_input(game, index, &sample)` binds a live `Input` sample to roster
  entrant `index`. Entrant 0 keeps reading the shared `game->input`; other human entrants read
  their own bound sample, or a zeroed sample when unbound (`game_clear_entrant_input`).
- Each entrant's controller dispatches on its **own** controller kind (issue #59): an AI
  entrant always drives with its own driver regardless of the session's input source; a human
  entrant follows the session source (live/script/replay).
- The roster, grid, timing, classification, penalties, and results are shared session
  authorities, so every local player gets identical rules and independent result rows.

Camera/UI policy: the presentation layer follows the single *local* entrant
(`race_roster_local`). A shared camera is the current 2D-readable default; split view is a
presentation-layer concern and does not touch the simulation.

## Network synchronization model: lockstep

Selected model: **lockstep with deterministic simulation and periodic state hashes**.

Justification from measured numbers:

- A full 8-car tick costs ~177 µs (issue #45, `performance-budget` scenario), so a 60 Hz
  lockstep step is comfortably inside the frame budget even with input round-trips.
- The engine is already bit-deterministic: two runs of the same session and input stream
  produce identical checksums (`replay`/`devreplay`/`ai-no-privilege` scenarios), which is
  exactly the property lockstep needs — no prediction, no rollback, no server authority.
- State is small enough to hash per tick: `game_state_checksum()` folds the authoritative
  fields (the rolling checksum already runs every tick in every build).

Protocol sketch (versioned, from existing authorities):

1. **Session manifest**: `SessionConfig` serialization (issue #48) + track/vehicle content
   hashes (`track_geometry_hash`, `vehicle_content_hash`) + `ChampionshipConfig.version`.
   Peers reject mismatched content/rules with the offending hashes.
2. **Input channel**: each peer's controller stream (the `Input` samples, the same stream the
   replay recorder captures) is broadcast per tick. Ordering is enforced by the tick number;
   inputs are applied only at the start of their tick, so network order cannot change
   authoritative contact/rule ordering.
3. **Desync detection**: peers exchange `game_state_checksum()` every N ticks; a mismatch is
   reported (not silently hidden) and the session can be abandoned with the divergence report
   (`game_divergence_report`).
4. **Join/leave/reconnect**: roster changes are restricted to grid assembly (pre-green) or an
   explicit pause; mid-race join is out of scope.

Offline behaviour is untouched: all of the above is additive and never a prerequisite.

## Tests

- `local-multiplayer` scenario: input independence (bind/unbind), two human entrants racing
  alongside AI with independent classified results.
- Determinism suites (`replay`, `ai-no-privilege`, multi-car checksum scenarios) are the
  lockstep-equivalence evidence for the network model.
