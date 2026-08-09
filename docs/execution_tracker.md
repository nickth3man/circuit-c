# Deterministic 2D Racing Simulator — Execution Tracker

Master roadmap: https://github.com/nickth3man/circuit-c/issues/2

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
6. Issue #59 is post-core and does not block the AI-racing target.
7. Issue #60 is the final core release gate.

---

# Phase 0 — Identity and Core Architecture

Start the Identity and Architecture lanes in parallel.

## Identity lane

- [x] [#3 — Choose the replacement product identity and migration map](https://github.com/nickth3man/circuit-c/issues/3)
  - Depends on: none

- [x] [#4 — Remove drift-game language, scoring remnants, and misleading scenario names](https://github.com/nickth3man/circuit-c/issues/4)
  - Depends on: #3

- [x] [#5 — Rename technical symbols, binaries, paths, artifacts, and repository metadata](https://github.com/nickth3man/circuit-c/issues/5)
  - Depends on: #3, #4

## Architecture lane

- [x] [#6 — Define simulation, content, entrant, and session ownership contracts](https://github.com/nickth3man/circuit-c/issues/6)
  - Depends on: none

After #6, #7 and #8 may run in parallel.

- [ ] [#7 — Split immutable TrackDefinition from per-entrant RacerProgress](https://github.com/nickth3man/circuit-c/issues/7)
  - Depends on: #6

- [x] [#8 — Split VehicleDefinition, VehicleSetup, and VehicleInstance state](https://github.com/nickth3man/circuit-c/issues/8)
  - Depends on: #6

- [x] [#9 — Unify human, AI, replay, ghost, and scripted control behind Controller output](https://github.com/nickth3man/circuit-c/issues/9)
  - Depends on: #8

- [ ] [#10 — Introduce RaceEntrant and deterministic multi-vehicle storage](https://github.com/nickth3man/circuit-c/issues/10)
  - Depends on: #7, #8, #9

- [ ] [#11 — Introduce RaceSession lifecycle, rules, and ordered fixed-update stages](https://github.com/nickth3man/circuit-c/issues/11)
  - Depends on: #10

## Phase 0 gate

- [x] Replacement identity is fully decided.
- [x] Player-facing drift scoring/game framing has been removed.
- [x] Technical identity rename builds and tests successfully.
- [ ] Track content and racer progress have separate ownership.
- [x] Vehicle definitions, setups, and runtime instances have separate ownership.
- [x] All simulated entrants use the same controller contract.
- [ ] A headless RaceSession can own multiple isolated entrants.
- [ ] Single-car behavior remains operational through the new architecture.

---

# Phase 1 — Physics Foundations

These issues establish truthful parameters and safe solver seams before adding physical effects.

- [x] [#12 — Audit VehicleSpec parameter truthfulness and units](https://github.com/nickth3man/circuit-c/issues/12)
  - Depends on: #8

- [ ] [#13 — Decompose the planar solver into deterministic, testable stages](https://github.com/nickth3man/circuit-c/issues/13)
  - Depends on: #11, #12

## Phase 1 gate

- [x] Every vehicle parameter is classified as active, derived, runtime, appearance-only, diagnostic, or inactive/reserved.
- [x] Every active parameter has documented units and validation bounds.
- [x] No UI or documentation claims inactive parameters affect handling.
- [ ] Physics stages have explicit inputs, outputs, and ordering.
- [x] Existing physics baselines and deterministic tests still pass.

---

# Phase 2 — Data-Driven Cars and Track Foundations

The Cars and Tracks lanes can run primarily in parallel.

## Cars lane

- [ ] [#29 — Define a versioned vehicle manifest and loader](https://github.com/nickth3man/circuit-c/issues/29)
  - Depends on: #8, #12

- [ ] [#30 — Migrate the six-car validated roster out of hard-coded C](https://github.com/nickth3man/circuit-c/issues/30)
  - Depends on: #29

After #30, #31 and #32 may run in parallel.

- [ ] [#31 — Separate the appearance corpus from gameplay roster promotion](https://github.com/nickth3man/circuit-c/issues/31)
  - Depends on: #29, #30

- [ ] [#32 — Add player-facing car selection, inspection, and compatibility status](https://github.com/nickth3man/circuit-c/issues/32)
  - Depends on: #30, #11

- [ ] [#33 — Implement validated setups, classes, and class-aware performance checks](https://github.com/nickth3man/circuit-c/issues/33)
  - Depends on: #8, #12, #29

## Tracks lane

- [ ] [#34 — Define a versioned external track format](https://github.com/nickth3man/circuit-c/issues/34)
  - Depends on: #7

- [ ] [#35 — Implement headless track loading, validation, version migration, and hashes](https://github.com/nickth3man/circuit-c/issues/35)
  - Depends on: #34

After #35, track migration can proceed while race semantics are developed.

- [ ] [#36 — Migrate built-in tracks from C into external content](https://github.com/nickth3man/circuit-c/issues/36)
  - Depends on: #35

- [ ] [#37 — Separate route checkpoints, timing sectors, start/finish, grid, and pit semantics](https://github.com/nickth3man/circuit-c/issues/37)
  - Depends on: #11, #34

- [ ] [#38 — Implement per-entrant route localization, progress, ordering, and wrong-way detection](https://github.com/nickth3man/circuit-c/issues/38)
  - Depends on: #10, #37

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

Multiple lanes may operate in parallel after #13, but dependencies within each lane must be respected.

## Alignment lane

- [ ] [#14 — Apply per-wheel toe/alignment to contact-patch kinematics](https://github.com/nickth3man/circuit-c/issues/14)
  - Depends on: #13

- [ ] [#15 — Model bounded camber and caster effects in the planar tire model](https://github.com/nickth3man/circuit-c/issues/15)
  - Depends on: #14

## Tire dimensions and transient-force lane

- [ ] [#16 — Make tire width, radius, and pressure affect force generation consistently](https://github.com/nickth3man/circuit-c/issues/16)
  - Depends on: #12, #13

- [ ] [#20 — Add longitudinal relaxation and aligning-moment diagnostics](https://github.com/nickth3man/circuit-c/issues/20)
  - Depends on: #13, #15, #16

- [ ] [#21 — Add deterministic tire temperature and pressure state](https://github.com/nickth3man/circuit-c/issues/21)
  - Depends on: #8, #16, #20

- [ ] [#22 — Add tire wear, grip degradation, and deterministic service hooks](https://github.com/nickth3man/circuit-c/issues/22)
  - Depends on: #11, #21

## Aerodynamics lane

- [ ] [#17 — Implement speed-squared aerodynamic loads and balance](https://github.com/nickth3man/circuit-c/issues/17)
  - Depends on: #12, #13

## Suspension lane

- [ ] [#18 — Derive quasi-static load transfer from suspension and chassis parameters](https://github.com/nickth3man/circuit-c/issues/18)
  - Depends on: #12, #13

- [ ] [#19 — Add suspension travel, bump-stop, and wheel-unloading limits](https://github.com/nickth3man/circuit-c/issues/19)
  - Depends on: #18

## Powertrain lane

- [ ] [#23 — Model engine inertia, clutch coupling, and shift engagement](https://github.com/nickth3man/circuit-c/issues/23)
  - Depends on: #8, #9, #13

- [ ] [#24 — Add fuel consumption and dynamic mass/CG effects](https://github.com/nickth3man/circuit-c/issues/24)
  - Depends on: #8, #23

## Driver-assist lane

- [ ] [#25 — Add deterministic ABS and traction-control controllers](https://github.com/nickth3man/circuit-c/issues/25)
  - Depends on: #9, #20, #23

## Collision lane

- [ ] [#26 — Introduce a deterministic collision world and measured broadphase](https://github.com/nickth3man/circuit-c/issues/26)
  - Depends on: #10, #11

- [ ] [#27 — Implement deterministic vehicle-to-vehicle contact response](https://github.com/nickth3man/circuit-c/issues/27)
  - Depends on: #26

- [ ] [#28 — Add configurable collision damage and deterministic stuck recovery](https://github.com/nickth3man/circuit-c/issues/28)
  - Depends on: #11, #27

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

- [ ] [#39 — Cache local track queries and add measured spatial acceleration](https://github.com/nickth3man/circuit-c/issues/39)
  - Depends on: #26, #38

- [ ] [#40 — Add elevation, grade, banking, kerb, and road-profile effects for the 2.5D solver](https://github.com/nickth3man/circuit-c/issues/40)
  - Depends on: #19, #35, #38

- [ ] [#41 — Add deterministic weather, wetness, time, and environmental presentation state](https://github.com/nickth3man/circuit-c/issues/41)
  - Depends on: #21, #40

- [ ] [#42 — Build the track authoring, validation, conversion, and preview workflow](https://github.com/nickth3man/circuit-c/issues/42)
  - Depends on: #35, #37, #41

- [ ] [#43 — Ship independent circuits, discovery metadata, previews, and track coverage](https://github.com/nickth3man/circuit-c/issues/43)
  - Depends on: #36, #42

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

- [ ] [#44 — Build a multi-car determinism, replay, checksum, and regression matrix](https://github.com/nickth3man/circuit-c/issues/44)
  - Depends on: #9, #10, #11, #27, #38

- [ ] [#45 — Establish fixed-step performance budgets and multi-car scale benchmarks](https://github.com/nickth3man/circuit-c/issues/45)
  - Depends on: #10, #26, #39

- [ ] [#46 — Deliver interactive cross-platform builds and reproducible release packaging](https://github.com/nickth3man/circuit-c/issues/46)
  - Depends on: #5, #29, #35

- [ ] [#47 — Add versioned player profile, settings, controls, and accessibility](https://github.com/nickth3man/circuit-c/issues/47)
  - Depends on: #5, #29, #35

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

- [ ] [#48 — Build the player-facing car, track, and session configuration flow](https://github.com/nickth3man/circuit-c/issues/48)
  - Depends on: #11, #32, #43, #47

- [ ] [#49 — Implement grid placement, countdown, start lights, and false starts](https://github.com/nickth3man/circuit-c/issues/49)
  - Depends on: #11, #37, #48

## Timing and ghost lane

- [ ] [#50 — Implement authoritative timing, sectors, lap validity, and time-trial records](https://github.com/nickth3man/circuit-c/issues/50)
  - Depends on: #11, #37, #38, #47

- [ ] [#51 — Promote replay data into persistent, non-interacting player ghosts](https://github.com/nickth3man/circuit-c/issues/51)
  - Depends on: #9, #44, #50

## AI lane

- [ ] [#52 — Integrate the validation AI as a live session Controller](https://github.com/nickth3man/circuit-c/issues/52)
  - Depends on: #9, #10, #30, #38

- [ ] [#53 — Add AI racecraft, traffic awareness, and data-driven difficulty](https://github.com/nickth3man/circuit-c/issues/53)
  - Depends on: #27, #38, #45, #52

## Classification, rules, and results lane

Start #54 after both the grid/start and basic live-AI requirements are complete.

- [ ] [#54 — Implement multi-entrant race order, finish, and classification](https://github.com/nickth3man/circuit-c/issues/54)
  - Depends on: #11, #38, #49, #52

- [ ] [#55 — Implement track limits, cuts, wrong-way, contact, and penalty rules](https://github.com/nickth3man/circuit-c/issues/55)
  - Depends on: #28, #38, #54

- [ ] [#56 — Complete race HUD, classified results, records, retry, and next-session flow](https://github.com/nickth3man/circuit-c/issues/56)
  - Depends on: #47, #50, #54, #55

## Pits and event structure lane

- [ ] [#57 — Implement pit lane, speed limiter, stops, fuel/tire/damage service, and AI strategy hooks](https://github.com/nickth3man/circuit-c/issues/57)
  - Depends on: #22, #24, #28, #37, #55

- [ ] [#58 — Add practice, qualifying, race weekends, championships, points, and progression](https://github.com/nickth3man/circuit-c/issues/58)
  - Depends on: #33, #56, #57

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

- [ ] [#60 — Define and pass the target acceptance demo, documentation, and release gate](https://github.com/nickth3man/circuit-c/issues/60)
  - Depends on: #5, #11, #28, #33, #43, #44, #45, #46, #56, #57, #58

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
- [ ] No obsolete Drifty or drift-scoring identity remains.
- [ ] Windows and Linux release packages pass clean-install smoke tests.
- [ ] Performance meets the documented 120 Hz entrant budget.
- [ ] Issue #60 is closed.

---

# Post-Core Work — Multiplayer

This work closes the multiplayer gap but does not block the stated AI-racing target.

- [ ] [#59 — Close the post-core local and network multiplayer gap](https://github.com/nickth3man/circuit-c/issues/59)
  - Depends on: #44, #45, #46, #56

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
