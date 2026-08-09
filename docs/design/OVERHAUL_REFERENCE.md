
> A deterministic 2D racing simulator with physically grounded cars, data-driven tracks, AI competition, and conventional race/session structure.

## 1. Remove the old identity carefully

There are two distinct cleanup jobs.

### Player-facing cleanup

Remove or replace:

- “Drifty” window titles, menu title, executable/module names, screenshots, documentation, and generated reports.
- “drift sandbox,” “arcade drift,” “DRIFT!” and score/combo language.
- Drift-oriented car descriptions where they imply the game’s purpose.
- Documentation that frames oversteer as the main product goal.
- Artifact and command names presented to players or contributors.

The menu currently says “a tiny top-down drift sandbox” at [render_hud.c](C:/Users/nicolas/Documents/GitHub/drift-c/src/render/render_hud.c:80), while the README opens with “2D drift driving simulator.” Those should become neutral racing-simulator language first.

### Technical cleanup

If “all reference” is literal, it also includes:

- `DRIFTY_*` macros and include guards.
- `drifty.exe`, `drifty_tests.exe`, `game.dll` messaging, and build paths.
- Profiling names and generated metadata.
- Repository and documentation references.
- Failure bundles, validation reports, and automation commands.

That mechanical rename should be one isolated change after choosing the replacement project name and C symbol prefix. Mixing it with physics changes would make regressions unnecessarily hard to diagnose.

### Drift-related physics tests

Do not delete useful limit-handling tests merely because their current names say “drift.” Rename them scientifically:

| Current framing | Neutral simulation framing |
|---|---|
| `catchable-drift` | `sideslip-recovery` |
| `constant-radius-drift` | `limit-oversteer-equilibrium` |
| `drift-recovery-envelope` | `sideslip-recovery-envelope` |
| `figure-eight-drift-transition` | `figure-eight-limit-transition` |
| `power-oversteer` | Can remain; this is a real handling phenomenon |
| `physicallySliding` | `atFrictionLimit` or `tireSaturationDetected` |
| “drift car” archetype | “competition RWD” or a concrete vehicle class |

A racing simulator should still prove that a car can enter, sustain, and recover from large sideslip. The distinction is that this becomes one portion of the handling envelope rather than the game’s identity.

# 2. Revised priority order

I would not address every physics gap first. The present data ownership assumes one car and one racer, so some physics work—especially collision and track interaction—would otherwise be implemented twice.

## Phase A: neutral identity and terminology

Observable completion criteria:

- No player-facing “Drifty,” drift-game, score, or combo language.
- Neutral README and window/menu copy.
- Handling scenarios renamed without weakening their checks.
- Replacement project name and technical prefix applied consistently.
- Existing 5,892 checks still pass.

## Phase B: fix the domain model

This is the foundation for a racing game.

### Split track definition from race progress

Current `Track` mixes geometry with one racer’s lap state at [track.h](C:/Users/nicolas/Documents/GitHub/drift-c/src/world/track.h:64).

A better shape is:

```c
typedef struct {
    TrackNode *nodes;
    int nodeCount;
    Checkpoint *checkpoints;
    int checkpointCount;
    SurfaceId offTrackSurfaceId;
    SurfaceId runoffSurfaceId;
    TrackMetadata metadata;
} TrackDefinition;

typedef struct {
    int nextCheckpoint;
    int completedLaps;
    float currentLapTimeS;
    float lastLapTimeS;
    float bestLapTimeS;
    bool finished;
} RacerProgress;
```

Every vehicle/entrant then receives its own `RacerProgress` while sharing one immutable track definition.

### Split vehicle definition from vehicle instance

The current `Game` embeds one `VehicleSpec`, `VehicleState`, `VehicleDerived`, and render state at [game.h](C:/Users/nicolas/Documents/GitHub/drift-c/src/game/game.h:77).

Move toward:

```text
VehicleDefinition
  immutable physics specification and content identity

VehicleInstance
  VehicleState
  VehicleDerived
  VehicleRenderState
  transmission/controller state
  per-race damage or tire state

RaceEntrant
  VehicleInstance
  RacerProgress
  human/AI controller state
  grid position and finishing position
```

This unlocks AI opponents, vehicle collisions, replays, starting grids, and classification without duplicating global state.

### Introduce a race session

`GameStateId` is currently only menu/playing/paused/results. Add a session layer that owns:

- Selected track.
- Entrants.
- Race mode.
- Lap target.
- Countdown.
- Race clock.
- Start grid.
- Running/finished status.
- Classification.
- Penalties.
- Results.

Keep `Game` as the application container, but stop making it the race rules object.

# 3. Physics roadmap for a 2D driving simulator

The existing solver is a good base. The highest-value physics gaps are the fields that already claim physical meaning but do not affect forces.

## Physics 1: parameter truthfulness

Audit every `VehicleSpec` field and classify it:

- Active dynamics input.
- Derived dynamics value.
- Appearance-only input.
- Metadata.
- Unimplemented future physics.

Then make the tuning UI reveal that classification.

This prevents users from changing tire pressure, camber, caster, or aerodynamic lift and incorrectly assuming the handling changed.

## Physics 2: static alignment and tire geometry

Implement the lower-risk missing effects first:

- Add static toe to each wheel’s physical heading.
- Add a bounded camber effect to lateral grip or camber thrust.
- Use tire section width and pressure in stiffness/load-sensitivity derivation.
- Ensure front and rear tire dimensions influence more than wheel radius.
- Add tests showing monotonic and symmetric effects.

These fit the planar model without requiring suspension degrees of freedom.

## Physics 3: aerodynamic loads

The specification already has front/rear lift coefficients and reference areas. Implement:

- Speed-squared front and rear vertical aerodynamic load.
- Resulting per-axle tire-capacity changes.
- Centre-of-pressure contribution.
- Drag and downforce reported separately.
- Tests for zero-speed neutrality, speed-squared scaling, axle balance, and energy behavior.

This is especially relevant once fast circuit cars become a focus.

## Physics 4: suspension-informed load transfer

A full vertical solver is not necessary immediately. Improve the current algebraic model by using existing configuration:

- Wheel rates.
- Anti-roll stiffness.
- Front/rear roll-centre heights.
- Suspension travel.
- Bump-stop/load-transfer limits.
- Roll stiffness derived from physical suspension fields rather than an independent fraction.

The intermediate goal should be a quasi-static suspension model, not a full 3D multibody simulation.

## Physics 5: tire state

For longer races:

- Tire temperature.
- Tire wear.
- Pressure-temperature relationship.
- Grip and stiffness changes.
- Surface-dependent heating/cooling.
- Lockup and wheelspin wear.
- Optional pit restoration.

Dust Racing 2D already uses tire wear and pit stops as gameplay structure, although Drifty’s implementation could be much more physically grounded. [Dust Racing 2D](https://github.com/juzzlin/DustRacing2D)

## Physics 6: powertrain fidelity

Potential additions:

- Engine rotational inertia.
- Clutch state and engagement.
- Shift interruption.
- Engine braking derived from RPM/load.
- Fuel consumption and changing mass.
- Differential behavior on both axles.
- ABS and traction-control assists.
- Mechanical or thermal limits only if races are long enough to expose them.

## Physics 7: collision

Before real opponents:

- Replace the track-only assumption with generic collision shapes.
- Add vehicle-to-vehicle contacts.
- Add a broad phase.
- Separate collision response from gameplay crash events.
- Handle immobilized/stuck recovery.
- Add collision attribution for race rules.
- Preserve deterministic ordering when several vehicles contact simultaneously.

This is likely the hardest prerequisite for convincing racing.

# 4. Car-system roadmap

## Make the six-car roster actual gameplay content

Expose the existing validated roster through:

- Car selection.
- Specifications and class information.
- Drivetrain, mass, power, dimensions, and tire summary.
- A preview generated through the existing procedural renderer.
- Per-car records.

The 100-car appearance corpus should remain test/demo content until each entry has reviewed driving behavior.

## Use data manifests

A car manifest should contain:

```text
id
display name
class/category
physics profile
appearance identity or overrides
default setup
AI eligibility
availability
version
```

The C roster can become a built-in fallback or generated table, but ordinary content creation should not require recompilation.

## Separate setup from identity

Players should be able to change setup values without mutating what car they selected:

- Tire pressure.
- Alignment.
- Brake bias.
- Differential settings.
- Gear ratios.
- Aero balance.
- Suspension settings.

The base definition, current setup, and runtime state should be distinct.

## Add validation per car class

The current uniform AI roster test is excellent. Extend it toward:

- Acceleration and braking envelopes.
- Maximum sustainable lateral acceleration.
- High-speed stability.
- Recovery from perturbations.
- Track completion on every supported circuit.
- AI lap-time sanity within class.
- No unexpected parameter fields that are dynamics-neutral.

# 5. Track-system roadmap

## Introduce a versioned external format

The first track format only needs to represent what exists:

- Metadata and version.
- Closed/open course.
- Centreline nodes.
- Racing width.
- Runoff width.
- Surface identifiers.
- Barrier definitions.
- Checkpoints.
- Starting grid.
- Pit lane/start position.
- Presentation metadata.

Keep the geometry hash and headless validation.

## Separate checkpoints, sectors, and timing lines

These are related but not identical:

- Checkpoints validate route order.
- Sectors measure intermediate time.
- Start/finish controls lap timing.
- Grid positions control race start.
- Pit entry/exit lines control pit behavior.

The current “one checkpoint per centreline node” fallback is useful for tests but too dense and semantically overloaded for authored tracks.

## Improve localization before adding many cars

Track each vehicle’s current segment and search locally before falling back to a full scan. This supports:

- Race progress along the lap.
- Position ordering.
- AI localization.
- Faster surface lookup.
- Faster barrier queries.
- Wrong-way detection.
- Sector deltas.

A spatial index can come later if measured performance requires it.

## Build track tooling around validation

Before a visual editor, create:

- File parser.
- Schema/version validation.
- Geometry continuity checks.
- Checkpoint-order validation.
- Self-intersection reporting.
- Barrier/runoff consistency checks.
- Headless AI completion.
- Preview/contact-sheet generation.

Once the data contract is stable, an editor becomes much safer to build.

# 6. Racing gameplay roadmap

For the new direction, the smallest complete gameplay loop is:

```text
Main menu
→ select track
→ select car
→ configure session
→ starting grid/countdown
→ race against AI
→ finish and classification
→ records/results
→ retry or choose next session
```

The first mode should be conventional circuit racing or time trial.

Recommended order:

1. Time trial with persistent best laps and a player ghost.
2. Single AI opponent without physical contact, if needed as an intermediate slice.
3. Multi-entrant race session and classification.
4. Deterministic vehicle-to-vehicle collision.
5. Difficulty levels and several AI drivers.
6. Championship/event sequence.
7. Tire wear and pits if race duration justifies them.
8. Local multiplayer.
9. Networking only after the entire race state is deterministic and serializable.

# Architectural principle to preserve

Do not turn the project into a generic engine while making these changes.

The current project succeeds because its systems are explicit, testable, and small. The right evolution is:

```text
Game
└── RaceSession
    ├── TrackDefinition
    ├── RaceRules
    └── Entrants[]
        ├── VehicleInstance
        ├── RacerProgress
        └── ControllerState
```

That is enough structure for a serious 2D racing simulator without introducing an ECS, plugin framework, generalized component graph, or premature networking abstraction.

The corrected top priorities are therefore:

1. Neutralize the old drift branding and terminology.
2. Split shared track content from per-racer progress.
3. Split vehicle definitions from mutable instances.
4. Introduce a real race-session owner.
5. Make cars and tracks data-driven.
6. Ensure every advertised physical parameter has a truthful effect or label.
7. Add persistent time-trial records and ghosts.
8. Add multiple entrants and collision.
9. Deepen aero, suspension, tire, and powertrain physics.
10. Expand into full race modes, progression, and broader platform support.
