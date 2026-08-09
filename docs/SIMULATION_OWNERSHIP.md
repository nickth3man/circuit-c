# Simulation ownership and fixed-step contract

## Status

Accepted for the issue 7–11 architecture refactors.

Issue 8 is implemented by `VehicleDefinition`, `VehicleSetup`, and `VehicleInstance`. The
definition owns the stable content ID/version/hash, immutable physical inputs, and a separate
appearance reference. Each entrant owns a validated setup and an instance containing its
compiled physics cache, pose/velocity, controls, drivetrain, tire, fuel, damage, and render-pose
state. `vehicle_instance_derive()` is the only definition/setup-to-runtime recomputation
boundary; reset changes runtime state without writing the shared definition. `Game` temporarily
exposes the prior one-entrant field names through the same `VehicleInstance` storage while
subsystems migrate.

## Context

The current single-car `Game`, `Track`, and `VehicleSpec` aggregates mix immutable content,
derived data, per-vehicle runtime state, per-racer progress, rendering, and session rules. That
ambiguity would let additional entrants accidentally share mutable state. This decision defines
the ownership target without changing runtime behavior. Child refactors retain the existing
`Game` entry points and deterministic tests while migrating incrementally.

## Decision

`Game` remains the platform-owned application container. An active race is owned by one
`RaceSession`; the session owns shared track runtime state and an ordered array of
`RaceEntrant` values. Each entrant owns its mutable vehicle, progress, and controller state.
Immutable vehicle and track content is shared by identity and cannot own race state.

```text
Game (application lifetime; hot-reload-safe storage)
├── content catalog
│   ├── TrackDefinition[stable TrackId]
│   └── VehicleDefinition[stable VehicleId]
├── active RaceSession
│   ├── SessionId, RaceRules, clock, phase, event queue
│   ├── selected TrackDefinition + TrackRuntime
│   └── RaceEntrant[ascending EntrantId]
│       ├── VehicleSetup (frozen session copy)
│       ├── VehicleInstance
│       ├── RacerProgress
│       └── Controller (kind + private state -> ControllerOutput)
└── presentation, platform timing, and development state
```

This is a fixed domain model, not an entity-component system. Storage remains explicit,
bounded where practical, and composed of plain data. Persistent `Game` state may not contain
function pointers or pointers into reloadable-module static storage.

## Rationale

- Immutable content can be validated, hashed, and shared without one entrant changing another.
- Entrant-local runtime ownership gives human, AI, replay, and ghost controllers one simulation
  path while keeping future-affecting state inside the deterministic boundary.
- Stable IDs and ordered stages make multi-entrant checksums independent of pointer values and
  incidental container order.
- Keeping `Game` as the explicit hot-reload-safe application owner preserves the current small C
  architecture and avoids an ECS or plugin framework.

## Type contracts

The checksum column describes the rolling simulation checksum. Immutable inputs are instead
covered by the session compatibility digest described below.

| Type | Owner and lifetime | Mutability | Serialization role | Deterministic identity | Rolling checksum |
| --- | --- | --- | --- | --- | --- |
| `VehicleDefinition` | Content catalog; application lifetime and shared by sessions/entrants | Immutable after validation | Versioned vehicle manifest and replay/session header reference | Stable `VehicleId` plus canonical definition hash | No; its ID/hash is in the compatibility digest |
| `VehicleSetup` | Player/profile while editing; copied into an entrant and frozen at session start | Mutable before a session, immutable during a session | Profile/setup file and canonical setup snapshot in replay/session header | Hash of canonical fields; no process address or array index | No; frozen setup hash is in the compatibility digest |
| `VehicleInstance` | Exactly one `RaceEntrant`; session lifetime | Mutable only during ordered simulation stages | Authoritative snapshot/keyframe state | Parent `EntrantId`; never a content ID | Yes, for all persistent state that can affect a later tick |
| `TrackDefinition` | Content catalog; application lifetime and shared by all entrants in a session | Immutable after validation | Versioned track manifest and replay/session header reference | Stable `TrackId` plus canonical geometry/content hash | No; its ID/hash is in the compatibility digest |
| `TrackRuntime` | `RaceSession`; session lifetime | Mutable session-wide environment, collision/query structures, and deterministic caches | Snapshot only for authoritative values; rebuildable caches are omitted | Parent `SessionId` | Authoritative values yes; pure caches no |
| `RacerProgress` | Exactly one `RaceEntrant`; session lifetime | Mutable only in progress/rules stages | Authoritative snapshot and results input | Parent `EntrantId` | Yes |
| `Controller` | Exactly one `RaceEntrant`; session lifetime | Kind/config frozen at start; private memory mutable only in controller stage | Kind/config in header; persistent controller memory in snapshot when it affects future output | Parent `EntrantId` plus `ControllerKind` | Persistent decision state yes; diagnostics no |
| `RaceEntrant` | `RaceSession`; session lifetime | Mutable through its child owners and classification fields | Ordered entrant record in session snapshot/results | Session-local `EntrantId` | Yes, serialized in ascending `EntrantId` order |
| `RaceSession` | `Game`; from configuration through teardown | Mutable according to its lifecycle and fixed-step stages | Replay/session header, authoritative snapshot, results | Session-local `SessionId` plus compatibility digest | Yes for authoritative mutable state |

### VehicleDefinition

Owns content identity, immutable base physics, physical geometry, appearance identity, and
classification metadata. It may name a separate default setup, but it does not own a mutable
setup object. It never owns fuel remaining, damage, tire condition/temperature, transmission
state, or any other per-race value.

Definition values that are deterministically derived from authored fields may be compiled and
stored beside the authored fields. They remain immutable and are validated against the same
definition hash. Rebuildable diagnostics do not belong here.

### VehicleSetup

Owns the entrant-selectable adjustments permitted by the definition: pressure and alignment,
brake bias, differential settings, gearing, suspension adjustments, and aero adjustments when
those become active. A session copies and validates the selected setup before tick zero.
Changing an editor/profile setup later cannot mutate an active entrant.

### VehicleInstance

Owns `VehicleState`, persistent wheel/tire state, transmission and powertrain state, fuel,
damage, service state, and any other value that evolves for one car. It also owns the previous
and current authoritative poses needed to build a presentation snapshot. Per-tick derived force
diagnostics may live in an entrant-local scratch/output value, but are recomputed and excluded
from snapshots/checksums unless a later tick reads them.

### TrackDefinition and TrackRuntime

`TrackDefinition` owns the authored centreline, surfaces, barriers, checkpoints, metadata, and
stable identity. It contains no lap counter, next checkpoint, racer-local cache, or timing value.

`TrackRuntime` owns mutable state shared by the session rather than by one racer, such as
deterministic weather/wetness and collision-world state. Acceleration structures derived only
from `TrackDefinition` are rebuildable caches. A localization cache that differs per racer
belongs to that entrant's `RacerProgress`, not to `TrackRuntime`.

### RacerProgress

Owns the next expected checkpoint, completed laps, start checkpoint, lap/sector timing,
route-localization cache, wrong-way state, finish state, and entrant-local penalties or validity
flags. Two racers on one `TrackDefinition` therefore cannot advance or reset each other.

### Controller

The controller boundary consumes a read-only tick-start view and emits one plain value:

```c
typedef struct {
    float steer;
    float throttle;
    float brake;
    float handbrake;
    bool shiftUp;
    bool shiftDown;
} ControllerOutput;
```

Human, AI, replay, ghost, and scripted controllers all use that output. Application commands
such as pause, reset, and debug toggles remain application/session commands and are not vehicle
controls. To preserve hot reload, `Controller` is a discriminated plain-data union selected by
`ControllerKind`; dispatch is a normal switch in module code, not a stored function pointer.

AI configuration can be immutable/shared, but `AiDriverState` is private to one controller.
Human frame samples and replay frames are input sources; they cannot write vehicle state.

### RaceEntrant and RaceSession

`RaceEntrant` is the sole aggregate owner for one competitor. In addition to the four child
contracts above, it owns grid slot, classification/finish values, and entrant-scoped rule state.

`RaceSession` owns selected content references, frozen rules, phase/countdown, authoritative
tick and race clock, entrant storage, deterministic event ordering, classification, and results.
It advances entrants in ascending `EntrantId` order. `GameStateId` continues to select
application screens; race lifecycle state belongs to `RaceSession`.

## Stable IDs and compatibility

Content IDs are UTF-8 manifest keys with a restricted, canonical spelling:
`[a-z0-9][a-z0-9._-]{0,62}`. They are stable across versions and are never inferred from a
display name or file path. A version/hash change identifies revised content without changing
the stable ID. Duplicate IDs are a load error.

`EntrantId` is a nonzero fixed-width integer assigned sequentially when the session roster is
frozen. Assignment follows the canonical configured roster order and never uses a pointer,
controller kind, grid position, or mutable array position. IDs are not reused within a session.
All entrant iteration, pair generation, snapshots, events, and checksums use ascending
`EntrantId`; a vehicle-contact pair is ordered `(minId, maxId)`.

`SessionId` identifies one configured run. It is metadata and must not introduce randomness.
Any random seed is an explicit frozen rules field.

Before tick zero the session computes a compatibility digest over the serialization version,
simulation version, fixed-step rate, rules and seed, `TrackId` and track hash, then each entrant
in ascending ID order with controller kind/config hash, `VehicleId` and definition hash, and
setup hash. Replay playback rejects a mismatched digest rather than attempting a best effort.

## Serialization and checksum boundary

There are three deliberately separate products:

1. A **content manifest** serializes immutable definitions and their versions.
2. A **replay header and timeline** serializes the compatibility digest plus one
   `ControllerOutput` per interacting entrant per tick. A non-interacting ghost may instead
   serialize its sampled pose stream, explicitly marked presentation-only.
3. An **authoritative snapshot** serializes mutable session state needed to resume at the next
   tick. It is canonical, versioned, and entrant-ID ordered.

The rolling checksum covers every authoritative mutable value that can influence a later tick:
session phase/tick/clock and rule state, ordered event state that survives the tick, each
entrant's persistent controller state, setup-selected runtime switches, `VehicleInstance`,
`RacerProgress`, and authoritative `TrackRuntime`. Floating values are hashed by their defined
canonical representation, not struct padding.

The rolling checksum excludes immutable definitions/setups already secured by the compatibility
digest, recomputed `VehicleDerived` diagnostics, broadphase/localization caches whose contents
cannot change results, input recording storage, telemetry, audio, particles, camera/interpolation
state, render scale, development tools, platform accumulator/backlog counters, and presentation
snapshots. If a supposed cache changes a result when rebuilt, it is authoritative state and must
be reclassified or fixed.

## Fixed-step ordering contract

The platform samples human devices once per rendered frame and latches application commands.
That sampling is outside the simulation. Every fixed tick then performs these stages in order:

1. **Acquire tick inputs.** Read the next replay frame or the latched human sample for each
   source; record the exact authoritative controller inputs for replay. Consume application
   one-shots once.
2. **Controller decisions.** In ascending `EntrantId`, give each controller the same read-only
   tick-start session/entrant view, update only its private state, and emit `ControllerOutput`.
   No controller observes another entrant's partially updated tick.
3. **Pre-physics/session gating.** Apply countdown/finished restrictions and deterministic
   transmission or assist logic to each output. Resolve per-wheel track/environment queries.
4. **Vehicle physics.** In ascending `EntrantId`, integrate each vehicle from tick-start state
   using its frozen definition/setup and controller output. Store authoritative end poses and
   recomputed diagnostics. Integration does not resolve vehicle pairs.
5. **Collision.** Resolve static track contacts in entrant order, then vehicle pairs in
   lexicographic `(minId, maxId)` order. Any iterative passes use a fixed documented count and
   the same pair order.
6. **Progress and rules.** From final post-collision sweeps, update every `RacerProgress`, then
   session clock/phase, penalties, finish, and classification in entrant order.
7. **Events.** Materialize authoritative events ordered by tick, stage priority, primary
   `EntrantId`, secondary `EntrantId`, then local sequence. Apply state-changing consequences
   here; audio/telemetry only consume copies later.
8. **Finalize.** Advance the session tick, compute the canonical rolling checksum, and append
   replay/checksum records.
9. **Presentation snapshot.** Copy/derive interpolation poses, HUD facts, audio cues, particles,
   telemetry, and developer diagnostics. Presentation cannot feed a later simulation tick.

## Current fixed-update read/write audit

This table accounts for the current `game_fixed_update()` path and assigns each access to its
target owner. It also records intentional ordering changes for the later session refactor.

| Current operation | Current read/write | Target owner and stage |
| --- | --- | --- |
| Playback selection and `replay_next` | Reads/writes `Game.replay`; chooses `tickInput` | Replay input source in acquire-input stage; replay storage excluded from checksum |
| Live/scripted substitution | Reads `Game.input`, `DevState`, `SimState.tick`; writes local `tickInput` | Human/scripted `Controller` source; scripts cannot bypass `ControllerOutput` |
| Clear one-shots and record input | Writes `Game.input` and `ReplayBuffer` | Acquire-input/application-command boundary |
| `apply_oneshots` | Writes `Game.state`, counters, transmission selection, vehicle reset | Pause/debug remain `Game`; race reset/phase commands enter `RaceSession`; gear commands become controller output |
| Particle update | Writes `ParticlePool` before simulation | Presentation only; moves to presentation stage and remains excluded |
| Automatic transmission | Reads setup/vehicle/derived/output; writes transmission, gear, output | Entrant `VehicleInstance` pre-physics stage; controller output is entrant-local |
| Capture start position | Reads `VehicleRenderState.currPositionM` | Read authoritative previous pose from `VehicleInstance`, not presentation state |
| Wheel surface queries | Reads `Track` geometry and vehicle pose; writes wheel `surfaceId` | Reads `TrackDefinition`/`TrackRuntime`; writes entrant `VehicleInstance` in pre-physics stage |
| `physics_fixed_update` | Reads `VehicleSpec` and output; writes `VehicleState`, `VehicleDerived`, `VehicleRenderState` | Reads frozen definition/setup; writes entrant instance and scratch diagnostics in physics stage |
| Checkpoint crossing/timer | Reads geometry and pose; writes `Track.nextCheckpoint`, `lap`, timers, and event reports | Writes only that entrant's `RacerProgress` in progress stage; definition remains immutable |
| Crash lockout countdown | Writes `Game.crashLockoutTimerS` | Entrant collision/damage state in `VehicleInstance`; checksummed if it gates later contacts |
| Track collision | Reads spec/track; writes vehicle/pose/lockout | Collision stage reads definition/runtime and writes only the entrant instance |
| Collision audio | Reads lockout change and calls audio | Presentation event consumer after authoritative events; never mutates simulation |
| Engine/tire audio | Reads vehicle/spec/derived | Presentation snapshot consumer; excluded |
| Results trigger | Reads lap and target; writes `Game.state` | Rules stage reads `RacerProgress`, writes `RaceSession` phase/classification; `Game` selects results screen afterward |
| Tire-smoke spawn | Reads derived/vehicle/tick; writes particles | Presentation snapshot/effect consumer; excluded even though deterministic today |
| Tick increment | Writes `SimState.tick` | `RaceSession.tick` in finalize stage; application diagnostic counters stay in `Game` |
| State checksum | Reads selected game/vehicle fields; writes checksum | Canonical `RaceSession` checksum after all authoritative stages |
| Development history | Reads tick/input/simulation; writes `DevState` | Presentation/development consumer after checksum; excluded |

The current checksum omits track progress, lateral filter state, relaxed lateral tire force,
automatic-transmission memory, and other future-affecting values. Child migrations must expand
the canonical checksum as each owner is introduced and update deterministic baselines in the
same focused change.

## Field-level migration map

No field is removed merely because its final owner changes. A child issue first moves the field,
adds a compatibility accessor/adapter, and then updates callers and tests.

### `Track`

| Current fields | Target |
| --- | --- |
| `nodes`, `count` | `TrackDefinition.nodes`, `nodeCount` |
| `checkpoints`, `checkpointCount` | `TrackDefinition.checkpoints`, `checkpointCount` |
| `offTrackSurfaceId`, `runoffSurfaceId` | `TrackDefinition` |
| `isParkingLot`, `lotMinXM`, `lotMaxXM`, `lotMinYM`, `lotMaxYM` | `TrackDefinition` geometry/mode |
| `id`, `version` | `TrackDefinition.id`, `contentVersion`; geometry hash is stored/verified beside them |
| `nextCheckpoint`, `lapStartCheckpoint` | `RacerProgress` route state |
| `lap`, `lapTimerS`, `lastLapTimeS` | `RacerProgress` timing/progress |

`TrackNode`, `Checkpoint`, and their arrays are definition data. `TrackCheckpointEvent` becomes
an entrant-scoped progress event value; retained telemetry copies remain presentation-only.

### `VehicleSpec`

| Current fields | Target |
| --- | --- |
| `wheelbaseM`, `trackWidthFrontM`, `trackWidthRearM`, `frontOverhangM`, `rearOverhangM`, `widthOverallM`, `heightOverallM`, `rideHeightFrontM`, `rideHeightRearM` | `VehicleDefinition` physical geometry |
| `cowlXM`, `backlightXM`, `bedLengthM`, `noseWidthM`, `tailWidthM`, `shoulderXM`, `fenderFlareFrontM`, `fenderFlareRearM` | `VehicleDefinition` body/appearance geometry |
| `roofStartXM`, `roofEndXM`, `roofWidthM`, `windscreenRakeRad`, `backlightRakeRad`, `sideWindowCount`, `quarterWindowLengthM`, `sunroofLengthM`, `doorCount`, `cabinRows`, `roofType` | `VehicleDefinition` body/appearance geometry |
| `massEngineKg`, `massEngineXM`, `massEngineZM`, `massGearboxKg`, `massGearboxXM`, `massGearboxZM`, `massFuelKg`, `massFuelXM`, `massFuelZM`, `massDriverKg`, `massDriverXM`, `massDriverZM`, `massChassisKg`, `massChassisXM`, `massChassisZM` | `VehicleDefinition` base mass model; current `massFuelKg` is initial/capacity content, while fuel remaining becomes `VehicleInstance` |
| `tireSectionWidthFrontMm`, `tireSectionWidthRearMm`, `tireAspectFrontPct`, `tireAspectRearPct`, `tireRimDiameterFrontIn`, `tireRimDiameterRearIn`, `tireRimWidthFrontIn`, `tireRimWidthRearIn` | `VehicleDefinition` tire/wheel hardware |
| `tirePressureFrontKpa`, `tirePressureRearKpa` | `VehicleSetup`; live pressure later belongs to `VehicleInstance` tire state |
| `suspCamberFrontRad`, `suspCamberRearRad`, `suspToeFrontRad`, `suspToeRearRad`, `suspCasterFrontRad`, `suspCasterRearRad` | `VehicleSetup`, bounded by `VehicleDefinition` hardware limits |
| `suspWheelRateFrontNpm`, `suspWheelRateRearNpm`, `suspAntiRollFrontNpm`, `suspAntiRollRearNpm`, `suspTravelFrontM`, `suspTravelRearM`, `suspRollCentreFrontM`, `suspRollCentreRearM` | `VehicleDefinition` until an explicitly adjustable subset is modeled in `VehicleSetup` |
| `wheelOffsetEtFrontMm`, `wheelOffsetEtRearMm`, `brakeDiscRadiusFrontM`, `brakeDiscRadiusRearM`, `brakePadFriction` | `VehicleDefinition` hardware |
| `aeroLiftCoefFront`, `aeroLiftCoefRear`, `aeroRefAreaFrontM2`, `aeroRefAreaRearM2`, `aeroCentreOfPressureXM` | `VehicleDefinition`; later adjustable aero settings are separate `VehicleSetup` deltas |
| `drivetrainLayout`, `frontTorqueSplit`, `engineCylinders`, `engineDisplacementL` | `VehicleDefinition` hardware |
| `massKg`, `yawInertiaKgM2`, `cgToFrontM`, `cgToRearM`, `cgHeightM`, `lengthOverallM`, `wheelRadiusFrontM`, `wheelRadiusRearM`, `wheelRadiusM`, `wheelInertiaKgM2`, `frontalAreaM2`, `bodyHalfWidthM`, `maxBrakeTorqueNm`, `rollStiffnessFrontFraction`, `tireRelaxationLengthM`, `tireLoadRefPerWheelN` | Immutable compiled/derived values beside `VehicleDefinition`; recompute when definition/setup inputs change |
| `maxRoadWheelAngleRad`, `maxSteerRateRadS`, `steerReturnRateRadS`, `steerSpeedRefMps`, `steerSpeedMinFactor`, `dragCoefficient`, `loadFilterRateHz` | `VehicleDefinition` physics parameters |
| `tireBLatFront`, `tireCLatFront`, `tireMuLatFront`, `tireBLatRear`, `tireCLatRear`, `tireMuLatRear`, `tireBLong`, `tireCLong`, `tireMuLongScale` | `VehicleDefinition` tire model |
| `gearRatios`, `gearCount`, `reverseGearRatio`, `finalDriveRatio` | `VehicleSetup` values validated against `VehicleDefinition` transmission limits/defaults |
| `drivetrainEfficiency`, `engineIdleRpm`, `engineRedlineRpm`, `engineTorqueCurveNm`, `engineBrakingTorqueNm` | `VehicleDefinition` powertrain |
| `brakeBiasFront` | `VehicleSetup` |
| `handbrakeTorqueNm` | `VehicleDefinition` brake hardware |
| `collisionRestitution`, `collisionFriction` | `VehicleDefinition` collision material |
| `tireLoadSensitivityK`, `ackermannPercent` | `VehicleDefinition` physics parameters |
| `differentialMode`, `differentialBiasRatio`, `differentialPreloadNm` | `VehicleSetup`, validated against supported `VehicleDefinition` differential hardware |
| `lateralLoadTransferEnabled` | Temporary development/validation option outside content; remove or freeze in session rules rather than let one entrant mutate shared definition |

`VehicleDefinition` also gains stable `id`, content version/hash, display/class metadata, and a
reference to appearance content. Those are content fields, not runtime state.

Issue 12 records the same assignment field by field, machine-checked: every float in
`VehicleSpec` carries an owner (`definition`, `setup`, `derived`) and a class (`physics`,
`derived`, `appearance`, `inactive`) in the `src/dev/dev_params.c` registry, tabulated in
[VEHICLE_PARAMETERS.md](VEHICLE_PARAMETERS.md). The `param-audit` scenario proves the owner
by compiling a perturbed definition through `vehicle_instance_derive()` and the class by
perturbing the field and comparing simulated trajectories. Ownership says who may write a
field; the class says whether writing it does anything.

### Vehicle runtime and diagnostics

| Current fields | Target |
| --- | --- |
| All `VehicleState` fields, including `wheels[WHEEL_COUNT]` and persistent filter values | `VehicleInstance` authoritative physics state |
| `WheelState.forceLateralRelaxedN` | `VehicleInstance`; it is persistent even though neighboring force fields are per-tick outputs |
| Other per-tick `WheelState` contact/load/force values | Entrant-local physics output unless the next tick consumes them; consumed values remain in `VehicleInstance` and checksum |
| All `VehicleDerived` fields | Entrant-local recomputed physics output/diagnostics, excluded unless a later tick begins to consume a field |
| `VehicleRenderState.prev*`, `VehicleRenderState.curr*` | Authoritative previous/current pose in `VehicleInstance`; copied into a presentation snapshot after checksum |
| `AutoTransmission.enabled`, `forwardOnly` | Frozen entrant setup/assist selection |
| `AutoTransmission.driveState`, `neutralTimer` | `VehicleInstance` transmission state |

### `Game` and adjacent current state

| Current fields | Target |
| --- | --- |
| `GameRunConfig.track` | Stable `TrackId` selection resolved into the new `RaceSession` |
| `GameRunConfig.cameraZoomOverride` | `Game` presentation configuration; never part of `RaceRules` or checksum |
| `GameRunConfig.targetLaps` | Frozen `RaceRules.targetLaps` in `RaceSession` |
| `state` | `Game` screen/application state; active race phase is added to `RaceSession` |
| `input` | `Game` platform-latched human input source; converted to one entrant's `ControllerOutput` per tick |
| `sim.tick` | `RaceSession.tick` while a session is active |
| `sim.resetCount`, `pauseToggleCount`, `debugToggleCount`, `shiftUpCount`, `shiftDownCount` | `Game` diagnostics/telemetry, excluded from authoritative session state unless a rule explicitly consumes one |
| `spec` | Selected immutable `VehicleDefinition` plus frozen `VehicleSetup`, initially through a one-entrant compatibility adapter |
| `vehicle`, `derived`, `renderState`, `autoTrans` | First `RaceEntrant`'s `VehicleInstance` and physics output, initially exposed through compatibility accessors |
| `track` | `RaceSession` selected `TrackDefinition`, `TrackRuntime`, and first entrant's `RacerProgress` |
| `lastCheckpointEvent`, `pendingTelemetryCheckpointEvent` | Entrant-scoped authoritative event then presentation/telemetry copies |
| `particles`, `camera`, `renderPixelsPerMeter` | `Game` presentation state; excluded |
| `accumulatorS`, `lastSubstepCount`, `physicsBacklogDrops` | `Game` platform timing; excluded |
| `replay` | Session replay recorder/source owned beside `RaceSession`; buffer mechanics excluded, replay header/timeline serialized |
| `stateChecksum` | `RaceSession` rolling checksum, mirrored on `Game` temporarily for API compatibility |
| `targetLaps` | Frozen `RaceRules` in `RaceSession` |
| `crashLockoutTimerS` | First entrant's `VehicleInstance` collision/damage state |
| `debugOverlay`, `reloadCount`, `reloadFlashTimerS`, `initialized`, `dev` | `Game` application/development state; excluded |

`ReplayBuffer.frames`, `head`, `count`, `playbackCursor`, `firstTick`, `overwrittenTicks`, and
`mode` remain recorder/source mechanics. `ReplayFrame` evolves from the current `Input` packing
to entrant-ID-keyed `ControllerOutput` records; pause/reset/debug are not entrant controls.

`AiDriverConfig` becomes immutable controller configuration. Persistent `AiDriverState` fields
(`prevCrossTrackErrorM`, `hasPrevError`, `pedalAxis`, `steerAxis`, `gripCut`, planned offsets and
plan indices/ticks) belong to one `Controller` and are checksummed. Its nearest-segment, error,
target, lookahead, and binding diagnostics may be recomputed/excluded once no future decision
reads them; until proven otherwise they remain authoritative controller state.

## Incremental migration order

Each step leaves the headless suite runnable and keeps `game_init`, `game_configure_run`,
`game_fixed_update`, `game_draw`, and checksum reporting available to existing callers.

1. **Issue 7:** introduce `TrackDefinition` and `RacerProgress`; adapt track queries to the
   definition and checkpoint updates to explicit progress. Keep a temporary one-racer `Track`
   facade/accessor for callers, then remove it after all tests use the split API.
2. **Issue 8:** introduce `VehicleDefinition`, `VehicleSetup`, and `VehicleInstance`; first copy
   the existing default/spec into one immutable definition and one setup. Keep one-entrant
   `Game` accessors so physics baselines remain unchanged.
3. **Issue 9:** add the plain-data `Controller` union and `ControllerOutput`; route human,
   scripted, replay, ghost, and existing AI input through it without changing physics input
   semantics.
4. **Issue 10:** introduce `RaceEntrant` and deterministic entrant storage. Start with capacity
   for multiple entrants but configure exactly one in all existing flows; migrate collision,
   progress, telemetry, and rendering to entrant iteration separately.
5. **Issue 11:** introduce `RaceSession`, move rules/tick/track runtime/event ordering into it,
   and make `game_fixed_update` delegate to the ordered session stages. Preserve `Game` as the
   platform/hot-reload owner and presentation container.
6. Expand replay/snapshot/checksum schemas only when their new owners exist. Version formats,
   update golden checksums deliberately, and retain a failing divergence test for each boundary.

No step changes physics equations. A field move must first reproduce the prior one-car checksum
or explain a checksum-schema-only change while the underlying state trajectory remains equal.

## Scenario review

| Scenario | Ownership result |
| --- | --- |
| One human | One human controller consumes the frame-latched sample and can mutate only its output/private state; one entrant owns all vehicle/progress state |
| Multiple AI entrants | AI config may be shared, but every entrant has separate `AiDriverState`, setup, vehicle, progress, and ID; all see the same tick-start snapshot |
| Replay | Recorded outputs are keyed by entrant ID and compatibility digest; playback enters through controllers and cannot write physics directly |
| Ghost | A competitive replay ghost uses a replay controller and normal entrant state; a non-interacting visual ghost is explicitly presentation-only and absent from collision, classification, and checksum |
| Two simultaneous racers on one track | Both read one immutable `TrackDefinition`; neither can mutate it or the other's `RacerProgress`; contacts use stable pair order and session rules classify both |

## Consequences

The model makes shared content safe and gives every deterministic mutation one owner. It also
requires explicit adapters during migration and a broader checksum than the current single-car
implementation. That cost is accepted because the same boundaries support human, AI, replay,
and ghost entrants without duplicate simulation paths.

An ECS, generalized plugin framework, stored behavior pointers, and physics changes are outside
this decision.
