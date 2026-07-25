# Drifty — Roadmap

Phase-by-phase status. [docs/SPEC.md](docs/SPEC.md) is authoritative for *what* each phase
contains and for the completion criteria reproduced below; this file tracks *where the work
actually stands* and is updated as phases close.

**Rule:** do not begin a phase until the previous phase's checklist is fully satisfied.

| Phase | Scope | Status |
|-------|-------|--------|
| [0](#phase-0--foundations-and-test-harness) | Foundations and test harness | ✅ **Complete** |
| [1](#phase-1--rigid-body-vehicle) | Rigid-body vehicle | ⬜ Not started — **next** |
| [2](#phase-2--tire-drivetrain-braking-and-combined-slip) | Tire, drivetrain, braking, combined slip | ⬜ Not started |
| [3](#phase-3--load-transfer-and-handling-validation) | Load transfer and handling validation | ⬜ Not started |
| [4](#phase-4--four-wheel-fidelity) | Four-wheel fidelity | ⬜ Optional upgrade |
| [5](#phase-5--track-surfaces-and-collision) | Track, surfaces, collision | ⬜ Not started |
| [6](#phase-6--scoring-effects-and-presentation) | Scoring, effects, presentation | ⬜ Not started |

Phases 0–3 are the mandatory path to a playable build. Phase 4 is an upgrade, not a
prerequisite: the corrected bicycle model of Phases 1–3 is a valid shipping configuration.

---

## Phase 0 — Foundations and Test Harness

**Status: complete.** Files: `config.h`, `units.h`, `math_utils.h/.c`, `input.h/.c`,
`replay.h/.c`, `telemetry.h/.c`, `timestep.h/.c`, `game.h/.c`, `main.c`, `hotreload.h`,
`hotreload_windows.c`, `hotreload_posix.c`, `Makefile`, `build.sh`, `build.bat`,
`tests/physics_tests.c`.

- [x] `drifty_tests` builds and runs with no display.
- [x] A one-frame reset press resets exactly once even when eight substeps run.
- [x] Substep count and dropped backlog are visible in the debug HUD.
- [x] Recorded input replays reproduce an identical state checksum.
- [x] Editing game code and running the build script swaps in the new module without
      restarting, preserving state.
- [x] A deliberate compile error leaves the running game alive on the previous module.
- [x] The build script exits in under a second in both the running and not-running cases.
- [x] A release build with `DRIFTY_HOT_RELOAD` undefined produces a single executable with
      no DLL and identical behaviour.

Verification notes, so later phases know what was actually exercised:

- 121 headless checks pass across 7 infrastructure scenarios; replay checksum `7cc8e6ee`
  reproduces across runs and across separate processes.
- Shared-raylib linkage confirmed with `objdump -p`: `build/game.dll` imports `raylib.dll`;
  `drifty_tests` imports no raylib at all.
- The safe-swap sequence — reject a bad candidate, keep the old module live, then swap a
  genuinely rebuilt module while preserving state — was exercised through a windowless
  harness driving `hotreload_windows.c` directly.
- **Not verified by an agent:** the visual behaviour of `drifty.exe` itself (HUD legibility,
  60 Hz smoothness). The developer owns that process.
- **Not verified at all:** `hotreload_posix.c`. Compile-ready, never run.

### Deliberate Phase 0 non-goals

No vehicle, tire, drivetrain, load-transfer, track, particle, or scoring code exists. The
window draws a labelled deterministic placeholder marker whose heading integrates the steer
axis at a constant rate — it is not a car and implies no physics.

---

## Phase 1 — Rigid-Body Vehicle

**Next.** Files: `vehicle.h/.c`, `physics.h/.c`, `render.h/.c`.

Starting point: define the canonical `VehicleSpec`, `VehicleState`, `VehicleDerived`,
`VehicleRenderState`, and `WheelState[WHEEL_COUNT]` exactly as docs/SPEC.md specifies, add
them to `Game`, and give `physics.c` ownership of the documented 20-step fixed update order.
`game.c`'s placeholder marker is deleted at that point.

- [ ] Contact-point velocities per wheel from body velocity and yaw rate.
- [ ] Per-axle slip angles with distinct `l_f` / `l_r` lever arms.
- [ ] Linear tire model `Fy = -C_alpha * alpha`, saturated at `mu * Fz`.
- [ ] Front forces rotated through `delta` into the body frame.
- [ ] Body dynamics and semi-implicit Euler in the documented order.
- [ ] Low-speed kinematic blend, 1.5 → 3.0 m/s.
- [ ] Steering rate limit on `frontRoadWheelAngleRad`.
- [ ] Interpolated renderer; front wheels steer, rear wheels do not.
- [ ] Debug overlay: axle velocity vectors, wheel heading vectors, slip angles, yaw rate.

**Complete when:** the car rotates from yaw torque and never from a direct `heading += steer`
term; front and rear slip angles respond differently to yaw rate; left steering from straight
travel produces positive yaw; rest stability, low-speed launch, and reverse scenarios pass;
rendering is smooth at 60 Hz with 120 Hz physics; doubling `PIXELS_PER_METER` changes nothing
but visual size.

Adding these structures changes the layout of `Game`, so `drifty.exe` must be restarted once
when they land. That is expected.

---

## Phase 2 — Tire, Drivetrain, Braking, and Combined Slip

Files: `tire.h/.c`, `drivetrain.h/.c`.

- [ ] Nonlinear lateral curve `Fy = -mu * Fz * sin(C * atan(B * alpha))`; remove the linear
      cornering-stiffness constants.
- [ ] Rear-wheel drive: engine torque curve, gearing, final drive, efficiency, locked rear axle.
- [ ] Per-wheel angular velocity integration with lockup handling.
- [ ] Slip ratio from wheel angular velocity, clamped.
- [ ] Longitudinal tire curve.
- [ ] Combined-friction ellipse with recorded `frictionUsage`.
- [ ] Brakes with `brakeBiasFront`; brake torque cannot reverse rotation.
- [ ] Handbrake as rear brake torque only — never a lateral-grip multiplier.
- [ ] Tire curve debug plot per axle.

**Complete when:** full throttle breaks rear traction without the handbrake; reducing throttle
restores rear lateral authority; the handbrake visibly decelerates or locks the rear wheels;
rear `Fx`/`Fy` never exceed the friction budget; braking while cornering measurably reduces
lateral force; the straight-line, coast-down, power-oversteer, and handbrake-entry scenarios
pass.

---

## Phase 3 — Load Transfer and Handling Validation

- [ ] Longitudinal load transfer from `l_f`, `l_r`, `h`, and filtered `ax`; clamped at
      `MIN_NORMAL_LOAD_N`.
- [ ] First-order filter on the previous step's solved `ax`.
- [ ] Aerodynamic drag opposite the velocity vector; rolling resistance per wheel.
- [ ] Skidpad, step-steer, lift-off, and drift-transition scenarios with committed CSV
      baselines in `tests/baselines/`.
- [ ] Feel tuning: `B`, `C`, `mu` per axle, brake bias, engine curve, steering rate.
- [ ] Reference-behaviour matrix populated.

**Complete when:** accelerating shifts load rearward and braking forward, with static loads
summing to `mass * g`; changing CG position changes static axle loads as expected; lifting
throttle mid-corner rotates the car and the CSV shows the load shift causing it; a slide can
be initiated, held on countersteer, transitioned, and recovered; behaviour is continuous
rather than binary; every scenario in the spec's validation section passes; the physics
acceptance checklist is fully checked.

This is the gate before any scoring or presentation work.

---

## Phase 4 — Four-Wheel Fidelity *(optional upgrade)*

- [ ] Four independent contact patches with track-width yaw lever arms.
- [ ] Lateral load transfer.
- [ ] Tire load sensitivity (sub-linear peak growth with `Fz`).
- [ ] Differential progression: locked → open → torque-bias/clutch LSD.
- [ ] Ackermann steering.
- [ ] Tire relaxation length.
- [ ] Per-surface `tireBScale` and loose-surface drag.

**Complete when:** inside-wheel unloading and snap oversteer are observable in telemetry; one
wheel on grass produces asymmetric yaw torque; differential mode measurably changes power
oversteer; all Phase 3 scenarios still pass with reviewed, re-baselined CSV deltas.

---

## Phase 5 — Track, Surfaces, and Collision

Files: `track.h/.c`, `surface.h/.c`.

- [ ] Hand-authored oval, ~24 `TrackNode` entries in meters.
- [ ] Centreline and offset boundaries via `DrawLineEx` in render pixel space.
- [ ] `Track_SurfaceAt()` per wheel contact point — no global grip multiplier.
- [ ] Capsule collision with swept tests at high speed and impulse response with yaw effect.
- [ ] Checkpoints and lap timing.
- [ ] HUD: lap count, lap timer, checkpoint count.

**Complete when:** a full lap can be completed; straddling asphalt and grass produces
asymmetric behaviour rather than a uniform slowdown; the car cannot tunnel through a barrier
at top speed; the lap timer displays and resets on lap completion.

Surfaces are referenced by `SurfaceId` and resolved through `Surface_Get()` at point of use —
never cached as `const SurfaceSpec *`, which would not survive a hot reload.

---

## Phase 6 — Scoring, Effects, and Presentation

Files: `scoring.h/.c`, `particle.h/.c`, plus `render.h/.c` growth.

- [ ] Full multi-condition `scoringDrift` test, separate from `physicallySliding`.
- [ ] Normalised angle/speed/line factors with a combo multiplier.
- [ ] Validated high-score load/save in a user-writable directory.
- [ ] Smoke particles from a round-robin pool of 512.
- [ ] Drift camera zoom-*out* by decreasing `Camera2D.zoom`, smoothed on render dt.
- [ ] Nose triangle and steered front wheels.
- [ ] `STATE_MENU` → `STATE_PAUSED` → `STATE_RESULTS` polish.
- [ ] HUD: speed in km/h, gear, RPM, score, combo, lap.

**Complete when:** drifting a corner accumulates score with a visible multiplier chaining
across corners; creeping, reversing, spinning, and post-crash sliding do not score; smoke
trails render during drifts; the best score persists and survives a corrupted save file; the
menu → play → results → replay loop works; scoring provably does not alter any physical force,
verified by comparing a scored and an unscored replay checksum.

---

## Deferred beyond Phase 6

Multiple cars and AI opponents; skid-mark decals; tire and engine audio; car texture art;
track file loading; split-screen and ghost replays; in-menu car spec selection; full Magic
Formula MF6.1. Rationale for each is in docs/SPEC.md.

---

## Standing constraints

These hold in every phase and are not renegotiated by a later one:

- SI units internally; pixels exist only at the render boundary, via `units.h`.
- `physics.*`, `vehicle.*`, `tire.*`, `drivetrain.*` call no raylib function — that is what
  keeps `drifty_tests` headless.
- Drift is emergent. The handbrake is brake torque; scoring observes physics and never
  modifies it.
- Nothing reachable from `Game` may point into the game module's code or static data, and no
  function pointers live in persistent state.
- No allocation during gameplay.
- Every build command terminates.
