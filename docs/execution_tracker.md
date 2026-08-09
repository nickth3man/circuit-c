# Deterministic 2D Racing Simulator — Execution Tracker

Master roadmap: https://github.com/nickth3man/drift-c/issues/42

## Target

Deliver a deterministic 2D racing simulator with:

- Physically grounded cars
- Data-driven vehicle and track content
- Deterministic AI competition
- Conventional practice, qualifying, race, timing, classification, and results
- Replay/checksum validation
- Cross-platform release packaging

## Status legend

- [ ] Not started
- [x] Completed

If work is in progress, append `— IN PROGRESS` to its line.

## Execution rules

1. Do not start an issue until every issue listed in its `Depends on` field is complete.
2. Tasks in separate lanes may proceed in parallel when their dependencies are complete.
3. Complete the phase gate before advancing to the next phase unless the later issue's dependencies are independently satisfied.
4. Physics behavior changes require focused validation and deterministic regression tests.
5. Content migrations must preserve existing behavior before compiled definitions are removed.
6. Issue #99 is post-core and does not block the AI-racing target.
7. Issue #100 is the final core release gate.

---

# Phase 0 — Identity and Core Architecture

Start the Identity and Architecture lanes in parallel.

## Identity lane

- [x] [#43 — Choose the replacement product identity and migration map](https://github.com/nickth3man/drift-c/issues/43)
  - Depends on: none

- [x] [#44 — Remove drift-game language, scoring remnants, and misleading scenario names](https://github.com/nickth3man/drift-c/issues/44)
  - Depends on: #43

- [x] [#45 — Rename technical symbols, binaries, paths, artifacts, and repository metadata](https://github.com/nickth3man/drift-c/issues/45)
  - Depends on: #43, #44

## Architecture lane

- [ ] [#46 — Define simulation, content, entrant, and session ownership contracts](https://github.com/nickth3man/drift-c/issues/46)
  - Depends on: none

After #46, #47 and #48 may run in parallel.

- [ ] [#47 — Split immutable TrackDefinition from per-entrant RacerProgress](https://github.com/nickth3man/drift-c/issues/47)
  - Depends on: #46

- [ ] [#48 — Split VehicleDefinition, VehicleSetup, and VehicleInstance state](https://github.com/nickth3man/drift-c/issues/48)
  - Depends on: #46

- [ ] [#49 — Unify human, AI, replay, ghost, and scripted control behind Controller output](https://github.com/nickth3man/drift-c/issues/49)
  - Depends on: #48

- [ ] [#50 — Introduce RaceEntrant and deterministic multi-vehicle storage](https://github.com/nickth3man/drift-c/issues/50)
  - Depends on: #47, #48, #49

- [ ] [#51 — Introduce RaceSession lifecycle, rules, and ordered fixed-update stages](https://github.com/nickth3man/drift-c/issues/51)
  - Depends on: #50

## Phase 0 gate

- [x] Replacement identity is fully decided.
- [x] Player-facing drift scoring/game framing has been removed.
- [x] Technical identity rename builds and tests successfully.
- [ ] Track content and racer progress have separate ownership.
- [ ] Vehicle definitions, setups, and runtime instances have separate ownership.
- [ ] All simulated entrants use the same controller contract.
- [ ] A headless RaceSession can own multiple isolated entrants.
- [ ] Single-car behavior remains operational through the new architecture.

---

# Phase 1 — Physics Foundations

These issues establish truthful parameters and safe solver seams before adding physical effects.

- [ ] [#52 — Audit VehicleSpec parameter truthfulness and units](https://github.com/nickth3man/drift-c/issues/52)
  - Depends on: #48

- [ ] [#53 — Decompose the planar solver into deterministic, testable stages](https://github.com/nickth3man/drift-c/issues/53)
  - Depends on: #51, #52

## Phase 1 gate

- [ ] Every vehicle parameter is classified as active, derived, runtime, appearance-only, diagnostic, or inactive/reserved.
- [ ] Every active parameter has documented units and validation bounds.
- [ ] No UI or documentation claims inactive parameters affect handling.
- [ ] Physics stages have explicit inputs, outputs, and ordering.
- [ ] Existing physics baselines and deterministic tests still pass.

---

# Phase 2 — Data-Driven Cars and Track Foundations

The Cars and Tracks lanes can run primarily in parallel.

## Cars lane

- [ ] [#69 — Define a versioned vehicle manifest and loader](https://github.com/nickth3man/drift-c/issues/69)
  - Depends on: #48, #52

- [ ] [#70 — Migrate the six-car validated roster out of hard-coded C](https://github.com/nickth3man/drift-c/issues/70)
  - Depends on: #69

After #70, #71 and #72 may run in parallel.

- [ ] [#71 — Separate the appearance corpus from gameplay roster promotion](https://github.com/nickth3man/drift-c/issues/71)
  - Depends on: #69, #70

- [ ] [#72 — Add player-facing car selection, inspection, and compatibility status](https://github.com/nickth3man/drift-c/issues/72)
  - Depends on: #70, #51

- [ ] [#73 — Implement validated setups, classes, and class-aware performance checks](https://github.com/nickth3man/drift-c/issues/73)
  - Depends on: #48, #52, #69

## Tracks lane

- [ ] [#74 — Define a versioned external track format](https://github.com/nickth3man/drift-c/issues/74)
  - Depends on: #47

- [ ] [#75 — Implement headless track loading, validation, version migration, and hashes](https://github.com/nickth3man/drift-c/issues/75)
  - Depends on: #74

After #75, track migration can proceed while race semantics are developed.

- [ ] [#76 — Migrate built-in tracks from C into external content](https://github.com/nickth3man/drift-c/issues/76)
  - Depends on: #75

- [ ] [#77 — Separate route checkpoints, timing sectors, start/finish, grid, and pit semantics](https://github.com/nickth3man/drift-c/issues/77)
  - Depends on: #51, #74

- [ ] [#78 — Implement per-entrant route localization, progress, ordering, and wrong-way detection](https://github.com/nickth3man/drift-c/issues/78)
  - Depends on: #50, #77

## Phase 2 gate

- [ ] Cars load from versioned external manifests.
- [ ] All six validated vehicles retain approved physical behavior.
- [ ] Visual-only corpus entries cannot silently enter the gameplay roster.
- [ ] Cars are selectable by stable content ID.
- [ ] Setups cannot mutate shared base definitions.
- [ ] Tracks load from versioned external files.
- [ ] All original tracks pass loader, collision, timing, and replay tests.
- [ ] Checkpoints, sectors, start/finish, grid, and pit metadata are distinct.
- [ ] Every entrant has independent route localization and progress.

---

# Phase 3 — Physics Implementation

Multiple lanes may operate in parallel after #53, but dependencies within each lane must be respected.

## Alignment lane

- [ ] [#54 — Apply per-wheel toe/alignment to contact-patch kinematics](https://github.com/nickth3man/drift-c/issues/54)
  - Depends on: #53

- [ ] [#55 — Model bounded camber and caster effects in the planar tire model](https://github.com/nickth3man/drift-c/issues/55)
  - Depends on: #54

## Tire dimensions and transient-force lane

- [ ] [#56 — Make tire width, radius, and pressure affect force generation consistently](https://github.com/nickth3man/drift-c/issues/56)
  - Depends on: #52, #53

- [ ] [#60 — Add longitudinal relaxation and aligning-moment diagnostics](https://github.com/nickth3man/drift-c/issues/60)
  - Depends on: #53, #55, #56

- [ ] [#61 — Add deterministic tire temperature and pressure state](https://github.com/nickth3man/drift-c/issues/61)
  - Depends on: #48, #56, #60

- [ ] [#62 — Add tire wear, grip degradation, and deterministic service hooks](https://github.com/nickth3man/drift-c/issues/62)
  - Depends on: #51, #61

## Aerodynamics lane

- [ ] [#57 — Implement speed-squared aerodynamic loads and balance](https://github.com/nickth3man/drift-c/issues/57)
  - Depends on: #52, #53

## Suspension lane

- [ ] [#58 — Derive quasi-static load transfer from suspension and chassis parameters](https://github.com/nickth3man/drift-c/issues/58)
  - Depends on: #52, #53

- [ ] [#59 — Add suspension travel, bump-stop, and wheel-unloading limits](https://github.com/nickth3man/drift-c/issues/59)
  - Depends on: #58

## Powertrain lane

- [ ] [#63 — Model engine inertia, clutch coupling, and shift engagement](https://github.com/nickth3man/drift-c/issues/63)
  - Depends on: #48, #49, #53

- [ ] [#64 — Add fuel consumption and dynamic mass/CG effects](https://github.com/nickth3man/drift-c/issues/64)
  - Depends on: #48, #63

## Driver-assist lane

- [ ] [#65 — Add deterministic ABS and traction-control controllers](https://github.com/nickth3man/drift-c/issues/65)
  - Depends on: #49, #60, #63

## Collision lane

- [ ] [#66 — Introduce a deterministic collision world and measured broadphase](https://github.com/nickth3man/drift-c/issues/66)
  - Depends on: #50, #51

- [ ] [#67 — Implement deterministic vehicle-to-vehicle contact response](https://github.com/nickth3man/drift-c/issues/67)
  - Depends on: #66

- [ ] [#68 — Add configurable collision damage and deterministic stuck recovery](https://github.com/nickth3man/drift-c/issues/68)
  - Depends on: #51, #67

## Phase 3 gate

- [ ] Toe, camber, caster, dimensions, and pressure have truthful bounded behavior.
- [ ] Aero affects high-speed load and balance correctly.
- [ ] Suspension parameters determine load transfer and finite travel.
- [ ] Tire transient, thermal, pressure, and wear state replay deterministically.
- [ ] Engine, clutch, shifts, fuel, and changing mass are active and validated.
- [ ] ABS and TCS work through normal controller/powertrain paths.
- [ ] Static and vehicle collision ordering is deterministic.
- [ ] Vehicle contacts conserve momentum within accepted tolerances.
- [ ] Damage and recovery cannot mutate shared content or exploit race progress.
- [ ] The complete physics validation suite passes.

---

# Phase 4 — Complete the Track System

- [ ] [#79 — Cache local track queries and add measured spatial acceleration](https://github.com/nickth3man/drift-c/issues/79)
  - Depends on: #66, #78

- [ ] [#80 — Add elevation, grade, banking, kerb, and road-profile effects for the 2.5D solver](https://github.com/nickth3man/drift-c/issues/80)
  - Depends on: #59, #75, #78

- [ ] [#81 — Add deterministic weather, wetness, time, and environmental presentation state](https://github.com/nickth3man/drift-c/issues/81)
  - Depends on: #61, #80

- [ ] [#82 — Build the track authoring, validation, conversion, and preview workflow](https://github.com/nickth3man/drift-c/issues/82)
  - Depends on: #75, #77, #81

- [ ] [#83 — Ship independent circuits, discovery metadata, previews, and track coverage](https://github.com/nickth3man/drift-c/issues/83)
  - Depends on: #76, #82

## Phase 4 gate

- [ ] Accelerated track queries match brute-force reference results.
- [ ] Acceleration structures are justified by benchmarks.
- [ ] Flat tracks preserve the original physical baseline.
- [ ] Grade, banking, crests, dips, and kerbs affect physics predictably.
- [ ] Weather and wetness are deterministic and wheel-local.
- [ ] Track validation can run headlessly in CI.
- [ ] Authors can inspect route, surface, barrier, grid, pit, and profile data.
- [ ] Shipped tracks are independently authored rather than trivial transformations.
- [ ] Every shipped track passes its automated coverage matrix.

---

# Phase 5 — Cross-Cutting Quality and Player Foundations

These tasks may run in parallel once their dependencies are complete.

- [ ] [#84 — Build a multi-car determinism, replay, checksum, and regression matrix](https://github.com/nickth3man/drift-c/issues/84)
  - Depends on: #49, #50, #51, #67, #78

- [ ] [#85 — Establish fixed-step performance budgets and multi-car scale benchmarks](https://github.com/nickth3man/drift-c/issues/85)
  - Depends on: #50, #66, #79

- [ ] [#86 — Deliver interactive cross-platform builds and reproducible release packaging](https://github.com/nickth3man/drift-c/issues/86)
  - Depends on: #45, #69, #75

- [ ] [#87 — Add versioned player profile, settings, controls, and accessibility](https://github.com/nickth3man/drift-c/issues/87)
  - Depends on: #45, #69, #75

## Phase 5 gate

- [ ] Multi-car replay produces identical checksums under the supported determinism contract.
- [ ] Divergence reports identify the first tick, entrant, subsystem, and fields.
- [ ] The target entrant count sustains the fixed 120 Hz simulation budget.
- [ ] Windows and Linux builds, tests, packages, and launch successfully.
- [ ] Release bundles contain all declared runtime dependencies.
- [ ] Settings and input bindings persist and migrate safely.
- [ ] Essential controls are remappable.
- [ ] Accessibility options work throughout menus and sessions.

---

# Phase 6 — Conventional Race and Gameplay Loop

Several lanes can begin in parallel after the phase foundations are complete.

## Session configuration and start lane

- [ ] [#88 — Build the player-facing car, track, and session configuration flow](https://github.com/nickth3man/drift-c/issues/88)
  - Depends on: #51, #72, #83, #87

- [ ] [#89 — Implement grid placement, countdown, start lights, and false starts](https://github.com/nickth3man/drift-c/issues/89)
  - Depends on: #51, #77, #88

## Timing and ghost lane

- [ ] [#90 — Implement authoritative timing, sectors, lap validity, and time-trial records](https://github.com/nickth3man/drift-c/issues/90)
  - Depends on: #51, #77, #78, #87

- [ ] [#91 — Promote replay data into persistent, non-interacting player ghosts](https://github.com/nickth3man/drift-c/issues/91)
  - Depends on: #49, #84, #90

## AI lane

- [ ] [#92 — Integrate the validation AI as a live session Controller](https://github.com/nickth3man/drift-c/issues/92)
  - Depends on: #49, #50, #70, #78

- [ ] [#93 — Add AI racecraft, traffic awareness, and data-driven difficulty](https://github.com/nickth3man/drift-c/issues/93)
  - Depends on: #67, #78, #85, #92

## Classification, rules, and results lane

Start #94 after both the grid/start and basic live-AI requirements are complete.

- [ ] [#94 — Implement multi-entrant race order, finish, and classification](https://github.com/nickth3man/drift-c/issues/94)
  - Depends on: #51, #78, #89, #92

- [ ] [#95 — Implement track limits, cuts, wrong-way, contact, and penalty rules](https://github.com/nickth3man/drift-c/issues/95)
  - Depends on: #68, #78, #94

- [ ] [#96 — Complete race HUD, classified results, records, retry, and next-session flow](https://github.com/nickth3man/drift-c/issues/96)
  - Depends on: #87, #90, #94, #95

## Pits and event structure lane

- [ ] [#97 — Implement pit lane, speed limiter, stops, fuel/tire/damage service, and AI strategy hooks](https://github.com/nickth3man/drift-c/issues/97)
  - Depends on: #62, #64, #68, #77, #95

- [ ] [#98 — Add practice, qualifying, race weekends, championships, points, and progression](https://github.com/nickth3man/drift-c/issues/98)
  - Depends on: #73, #96, #97

## Phase 6 gate

- [ ] A player can choose a car, setup, track, mode, rules, environment, and AI field.
- [ ] Entrants spawn on a valid grid and share an authoritative green-light tick.
- [ ] False starts generate rule events.
- [ ] Time trial timing, sectors, lap validity, records, and ghosts work.
- [ ] AI enters through the normal Controller interface.
- [ ] AI can start, race in traffic, overtake, recover, finish, and use pits.
- [ ] Difficulty does not grant AI hidden grip or power.
- [ ] Live race position and final classification are deterministic.
- [ ] Track limits, cuts, wrong-way driving, contacts, recovery, and pit violations have explicit rules.
- [ ] The player receives an understandable HUD and classified results.
- [ ] Retry completely resets session state.
- [ ] Practice, qualifying, races, and championships use shared session authorities.
- [ ] No drift score, drift combo, or drift objective exists in the gameplay loop.

---

# Phase 7 — Core Target Release Gate

- [ ] [#100 — Define and pass the target acceptance demo, documentation, and release gate](https://github.com/nickth3man/drift-c/issues/100)
  - Depends on: #45, #51, #68, #73, #83, #84, #85, #86, #96, #97, #98

## Final core acceptance checklist

- [ ] A clean packaged build launches without developer tools.
- [ ] A player can configure a valid car, track, environment, ruleset, and AI field.
- [ ] The race proceeds through grid, countdown, green, competition, finish, and classification.
- [ ] AI uses the same physics and rules as the player.
- [ ] Vehicle contacts, damage, recovery, penalties, fuel, tires, and pits behave deterministically.
- [ ] Results and records persist correctly.
- [ ] Retry and next-event flows work.
- [ ] Headless acceptance runs reproduce their expected checksums and results.
- [ ] All cars and tracks load from versioned external content.
- [ ] Content validators pass for every shipped car and track.
- [ ] Physics documentation identifies modeled, approximated, inactive, and out-of-scope behavior.
- [ ] No obsolete Circuit or drift-scoring identity remains.
- [ ] Windows and Linux release packages pass clean-install smoke tests.
- [ ] Performance meets the documented 120 Hz entrant budget.
- [ ] Issue #100 is closed.

---

# Post-Core Work — Multiplayer

This work closes the multiplayer gap but does not block the stated AI-racing target.

- [ ] [#99 — Close the post-core local and network multiplayer gap](https://github.com/nickth3man/drift-c/issues/99)
  - Depends on: #84, #85, #86, #96

## Post-core multiplayer gate

- [ ] Multiple local human controllers can own independent entrants.
- [ ] Local multiplayer can configure, start, race, and classify all players.
- [ ] The synchronization model is selected from measured prototypes.
- [ ] Network peers verify physics, content, setup, and rules compatibility.
- [ ] Desynchronization is detected and reported.
- [ ] Offline AI racing is unchanged when multiplayer is unused.

---

# Completion Summary

- [ ] Phase 0 — Identity and Core Architecture
- [ ] Phase 1 — Physics Foundations
- [ ] Phase 2 — Data-Driven Cars and Track Foundations
- [ ] Phase 3 — Physics Implementation
- [ ] Phase 4 — Complete Track System
- [ ] Phase 5 — Cross-Cutting Quality and Player Foundations
- [ ] Phase 6 — Conventional Race and Gameplay Loop
- [ ] Phase 7 — Core Target Release Gate
- [ ] Post-Core — Multiplayer
