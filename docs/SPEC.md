# Drifty — Top-Down Drift Simulator Game

## Overview

A top-down 2D drift driving game written in C using **raylib 6.0**. The player controls a
car from a bird's-eye view, building speed and initiating drifts around a track. Scoring
rewards sustained, fast, controlled slides with a combo multiplier.

The design goal is a **physically coherent vehicle simulation** underneath an arcade
presentation layer: the car must be able to initiate, hold, transition, and recover a
drift because of tire, drivetrain, and load-transfer behavior — not because a scoring or
state machine reaches in and changes forces.

See [SOURCES.md](SOURCES.md) for the technical references used by this specification.

---

## Scope and Development Philosophy

The specification separates three tiers of work. Do not require every advanced feature
before the first playable build.

| Tier | Contents | Status |
|------|----------|--------|
| **Required initial physics** | SI units, coordinate convention, planar rigid-body dynamics, per-axle slip angles, nonlinear lateral tire curve, rear-wheel drive with wheel angular velocity and slip ratio, combined-friction limit, brake and handbrake torque, longitudinal load transfer, low-speed blend, fixed timestep with substep cap, physics test harness | Phases 0–3, mandatory |
| **Later realism upgrades** | Four contact patches, lateral load transfer, differential, tire relaxation length, separated aerodynamic and rolling forces, per-surface tire coefficients | Phase 4, optional but architecturally planned for |
| **Presentation and gameplay** | Track geometry, collision, checkpoints, drift scoring, particles, camera effects, audio, menus | Phases 5–6 |

Two layers are kept deliberately distinct:

- **Simulation layer** — physically coherent vehicle behavior. Tuned by physical
  parameters only.
- **Control/feel layer** — input filtering, optional assists, camera, feedback, and
  accessibility. Never distort the tire model to compensate for keyboard input; add a
  separate keyboard steering controller instead.

---

## Technology Stack

| Component | Choice | Rationale |
|-----------|--------|-----------|
| Language | C11 | Simple, fast, zero-overhead |
| Graphics | **raylib 6.0** | Rich 2D primitives, `DrawTexturePro` for rotated sprites, built-in `Vector2`, single-header setup, no dependency hell |
| Math | raylib's `Vector2` + `math_utils.h` | Use raylib's types everywhere; add only missing helpers (`clampf`, `lerpf`, `smooth_to`, `wrap_angle`, `smoothstep`, `lerp_angle`) |
| Build | Makefile / build.bat (MSYS2 UCRT64 on Windows) | `mingw-w64-ucrt-x86_64-raylib` installs raylib 6.0; Windows-only |
| Physics | 120 Hz fixed timestep, SI units | Per-axle nonlinear tire model with driven-wheel dynamics (see [Physics Model](#physics-model)) |

Physics translation units (`physics.*`, `vehicle.*`, `tire.*`, `drivetrain.*`) **must not
call any raylib function**. They may use raylib's `Vector2` type from the header. This is
what allows the physics test harness to run without a window.

---

## Units, Coordinate System, and Conventions

### SI units are mandatory internally

All physical state and all parameters are stored in SI units. There are no pixel-valued
physical quantities anywhere in the simulation.

| Quantity | Unit | Suffix convention |
|----------|------|-------------------|
| Position, length, CG height | meters | `...M` |
| Linear velocity | meters/second | `...Mps` |
| Linear acceleration | meters/second² | `...Mps2` |
| Mass | kilograms | `...Kg` |
| Force | newtons | `...N` |
| Torque | newton-meters | `...Nm` |
| Yaw inertia | kg·m² | `...KgM2` |
| Angle (heading, steering, slip) | radians | `...Rad` |
| Angular velocity (yaw, wheel) | radians/second | `...RadS` |
| Time | seconds | `...S` |

Every physics constant carries its unit in its name or in a trailing comment. Constants
without units are dimensionless ratios and must be documented as such.

### Rendering is the only place pixels exist

```c
#define PIXELS_PER_METER 24.0f   // dimensionless render scale

// world meters -> render pixel space (render pixel space has +Y down)
screen.x =  world_m.x * PIXELS_PER_METER;
screen.y = -world_m.y * PIXELS_PER_METER;
```

`Camera2D` operates entirely in that render pixel space. Because the render layer negates
Y, a counterclockwise heading in world space appears counterclockwise on screen. raylib's
rotation arguments are degrees, positive clockwise in screen space, so pass:

```c
float rotationDeg = -state->headingRad * RAD2DEG;
```

**Requirement:** changing `PIXELS_PER_METER` changes visual size only. It must not alter
speed, acceleration, tire forces, or handling. This is a regression test (see
[Validation](#physics-validation-and-regression-testing)).

### Coordinate and sign convention

This convention governs every equation in this document.

```
Body X          : forward (nose direction)
Body Y          : left
World X / Y     : right-handed simulation plane, +Y is "up" on screen after the render flip
Heading         : angle of body X measured from world X, counterclockwise positive
Yaw rate (r)    : counterclockwise positive
Steering angle  : left positive
Tire slip angle : positive when the contact-point velocity points left of the wheel heading
Lateral force   : always acts opposite the slip angle
Slip ratio      : positive when wheel surface speed exceeds ground speed (driving/spinning)
Longitudinal force : positive forward along the wheel heading
```

Body-frame velocity components are named:

```c
float velocityLongitudinalMps;   // body X, forward positive
float velocityLateralMps;        // body Y, left positive
```

Body-to-world rotation (body X axis is `(cos h, sin h)`, body Y axis is `(-sin h, cos h)`):

```
world_vx = vx * cos(heading) - vy * sin(heading)
world_vy = vx * sin(heading) + vy * cos(heading)
```

Consequences that must hold in the implementation:

- A positive (left) steering input from straight forward travel produces positive yaw rate.
- Lateral tire force always opposes the corresponding slip angle.
- No force is applied in both the body frame and the world frame.

---

## Canonical Data Structures

These are the only definitions of these structures. Every phase, algorithm, and code
example in this document refers to exactly these fields.

### Surface

Surfaces are referenced by **id**, never by pointer. The `SurfaceSpec` table is static data
inside the game module, and a raw pointer into it does not survive a hot reload (see
[Development Workflow](#development-workflow)). Ids are stable across reloads; pointers are
not.

```c
typedef enum {
    SURFACE_ASPHALT,
    SURFACE_GRAVEL,
    SURFACE_GRASS,
    SURFACE_SNOW,
    SURFACE_COUNT
} SurfaceId;

typedef struct {
    const char *name;
    float muLongitudinal;        // dimensionless peak longitudinal friction coefficient
    float muLateral;             // dimensionless peak lateral friction coefficient
    float tireBScale;            // dimensionless; scales tire stiffness B (soft buildup < 1)
    float rollingResistanceCoefficient; // dimensionless
    float looseSurfaceDragN;     // extra drag force, newtons, opposing contact velocity
    float sinkFactor;            // dimensionless 0..1, reserved for terrain resistance
} SurfaceSpec;

// The only way physics code reaches surface data. Resolved fresh each use, never cached.
const SurfaceSpec *Surface_Get(SurfaceId id);
```

Baseline surfaces:

| Surface | muLongitudinal | muLateral | tireBScale | rollingResistanceCoefficient | looseSurfaceDragN |
|---------|---------------|-----------|-----------|------------------------------|-------------------|
| Asphalt | 1.35 | 1.30 | 1.00 | 0.015 | 0 |
| Gravel  | 0.85 | 0.80 | 0.65 | 0.045 | 250 |
| Grass   | 0.65 | 0.60 | 0.70 | 0.080 | 600 |
| Snow    | 0.40 | 0.38 | 0.50 | 0.050 | 200 |

### Wheels

The wheel array is four-wide from the first implementation. The bicycle-model phases fill
only the axle-representative entries and mirror them; Phase 4 populates all four
independently without changing any surrounding structure.

```c
typedef enum {
    WHEEL_FRONT_LEFT,
    WHEEL_FRONT_RIGHT,
    WHEEL_REAR_LEFT,
    WHEEL_REAR_RIGHT,
    WHEEL_COUNT
} WheelId;

typedef struct {
    Vector2 localPositionM;        // contact patch position in body frame (X forward, Y left)
    float   steerAngleRad;         // road-wheel angle; 0 for rear wheels
    float   angularVelocityRadS;   // wheel spin rate; positive = rolling forward
    float   normalLoadN;
    float   slipAngleRad;
    float   slipRatio;             // dimensionless
    float   forceLongitudinalN;    // in the wheel frame, after combined-friction limiting
    float   forceLateralN;         // in the wheel frame, after combined-friction limiting
    float   frictionUsage;         // dimensionless; 1.0 = at the friction limit
    bool    locked;                // brake torque has stopped rotation this step
    SurfaceId surfaceId;           // id, not a pointer; survives hot reload
} WheelState;
```

### Vehicle specification (immutable parameters)

```c
typedef struct {
    // Mass and geometry
    float massKg;
    float yawInertiaKgM2;
    float cgToFrontM;              // l_f
    float cgToRearM;               // l_r
    float wheelbaseM;              // L = cgToFrontM + cgToRearM (derived, stored for convenience)
    float cgHeightM;               // h
    float trackWidthFrontM;
    float trackWidthRearM;

    // Wheels
    float wheelRadiusM;
    float wheelInertiaKgM2;

    // Steering (road-wheel angle, not steering-wheel angle)
    float maxRoadWheelAngleRad;
    float maxSteerRateRadS;
    float steerReturnRateRadS;

    // Resistance
    float dragCoefficient;              // dimensionless Cd
    float frontalAreaM2;
    float rollingResistanceCoefficient; // dimensionless, vehicle baseline; surfaces override

    // Lateral tire curve (normalized: force = mu * Fz * curve)
    float tireBLatFront, tireCLatFront, tireMuLatFront;
    float tireBLatRear,  tireCLatRear,  tireMuLatRear;

    // Longitudinal tire curve
    float tireBLong, tireCLong, tireMuLongScale; // muLongScale multiplies the surface mu

    float tireRelaxationLengthM;        // Phase 4; 0 disables the relaxation filter

    // Drivetrain (rear-wheel drive)
    float gearRatios[MAX_GEARS];
    int   gearCount;
    float finalDriveRatio;
    float drivetrainEfficiency;         // dimensionless 0..1
    float engineIdleRpm;
    float engineRedlineRpm;
    float engineTorqueCurveNm[ENGINE_CURVE_POINTS]; // sampled evenly idle..redline
    float engineBrakingTorqueNm;

    // Brakes
    float maxBrakeTorqueNm;             // total, split by bias
    float brakeBiasFront;               // dimensionless 0..1
    float handbrakeTorqueNm;            // applied to rear wheels only
} VehicleSpec;
```

### Vehicle state (integrated each fixed step)

```c
typedef struct {
    Vector2 positionM;                  // world position of the CG
    float   headingRad;
    float   velocityLongitudinalMps;    // body X
    float   velocityLateralMps;         // body Y
    float   yawRateRadS;

    float   frontRoadWheelAngleRad;     // current, rate-limited

    float   engineRpm;
    int     selectedGear;               // 0 = neutral, 1..gearCount forward, -1 reverse

    float   filteredLongAccelMps2;      // filtered ax used by load transfer
    float   prevLongAccelMps2;          // previous solved ax

    WheelState wheels[WHEEL_COUNT];
} VehicleState;
```

### Derived / diagnostic state (recomputed each step, never integrated)

```c
typedef struct {
    float   bodySideslipRad;
    float   longitudinalAccelerationMps2;
    float   lateralAccelerationMps2;
    float   speedMps;

    float   normalLoadFrontN;
    float   normalLoadRearN;

    Vector2 totalBodyForceN;            // X = longitudinal, Y = lateral, body frame
    float   totalYawTorqueNm;

    float   maxFrictionUsage;           // max over wheels
    float   lowSpeedBlend;              // 0 = kinematic, 1 = fully dynamic

    bool    physicallySliding;          // pure physics classification
    bool    scoringDrift;               // gameplay classification, see Drift Detection
} VehicleDerived;
```

### Render state (interpolation only)

```c
typedef struct {
    Vector2 prevPositionM;
    float   prevHeadingRad;
    float   prevWheelAngleRad[WHEEL_COUNT];
    Vector2 currPositionM;
    float   currHeadingRad;
    float   currWheelAngleRad[WHEEL_COUNT];
} VehicleRenderState;
```

`prev*` is copied from `curr*` at the start of every fixed update, before any integration.

### Input

Held controls and one-shot commands are separate. One-shot commands are consumed by the
first fixed update that observes them.

```c
typedef struct {
    // Held controls, sampled every render frame, valid for every substep
    float steer;        // -1 (right) .. +1 (left), matching the left-positive convention
    float throttle;     // 0 .. 1
    float brake;        // 0 .. 1
    float handbrake;    // 0 .. 1

    // One-shot commands, edge-triggered, cleared after the first fixed update
    bool pausePressed;
    bool resetPressed;
    bool debugPressed;
    bool shiftUpPressed;
    bool shiftDownPressed;
} Input;
```

### Game

```c
typedef struct {
    GameStateId         state;      // STATE_MENU, STATE_PLAYING, STATE_PAUSED, STATE_RESULTS
    VehicleSpec         spec;
    VehicleState        vehicle;
    VehicleDerived      derived;
    VehicleRenderState  renderState;
    Input               input;
    ParticlePool        particles;
    Track               track;
    Camera2D            camera;

    float driftScore;
    float bestScore;
    float driftTimeS;               // seconds in the current scoring drift
    float comboMultiplier;
    float comboTimerS;              // seconds since the last scoring drift tick
    float crashLockoutTimerS;       // seconds remaining in the post-impact scoring lockout

    float accumulatorS;
    int   lastSubstepCount;
    int   physicsBacklogDrops;
    bool  debugOverlay;
} Game;
```

A **single `Game` structure owned by the platform layer** holds all runtime state. It is
allocated once at startup by `main.c` and passed to the game module by pointer on every
entry point. It must not be a `static Game game;` inside the game module: BSS belongs to
whichever module declares it, so a module-owned `Game` would be destroyed on every hot
reload. See [Development Workflow](#development-workflow).

No allocation occurs during gameplay — the only allocations are the `Game` block and the
track nodes, both at load time, both owned by the platform layer.

**Reload-safety invariant:** no field in `Game`, or in anything reachable from it, may be a
pointer into the game module's code or static data. That means no function pointers and no
`const SurfaceSpec *`. Use ids and resolve them through accessors at point of use.

### Particles and track

```c
typedef struct {
    Vector2 positionM;
    Vector2 velocityMps;
    float   lifeS;
    float   maxLifeS;
    float   sizeM;
    Color   color;
    bool    active;
} Particle;

typedef struct {
    Particle particles[MAX_PARTICLES];  // 512
    int      cursor;                    // round-robin
} ParticlePool;

typedef struct {
    Vector2   centerM;                  // centerline point
    float     halfWidthM;
    SurfaceId surfaceId;                // surface inside this segment
} TrackNode;

typedef struct {
    TrackNode *nodes;                   // owned by the platform layer, allocated once at load
    int        count;
    SurfaceId  offTrackSurfaceId;
    int        nextCheckpoint;
    int        lap;
    float      lapTimerS;
} Track;
```

---

## Physics Model

### Architecture

The vehicle is a planar rigid body with three degrees of freedom (world X, world Y, yaw)
plus one rotational degree of freedom per driven/braked wheel.

- **Intermediate model (Phases 1–3):** per-axle bicycle model. Left and right tires of an
  axle are aggregated. This is a correct, validated model in its own right and is what the
  first playable build ships with.
- **Higher-fidelity target (Phase 4):** four independent contact patches. The `WheelState`
  array, per-wheel surface queries, and per-wheel force accumulation exist from Phase 1 so
  that this upgrade changes force *sourcing*, not the surrounding architecture.

Throttle acts through the drivetrain and the driven (rear) wheels. There is no direct
throttle-to-velocity or throttle-to-body-force path anywhere in the model.

### Fixed update order

Every fixed update executes exactly this sequence:

1. Copy current render transform into the previous render transform.
2. Read held controls.
3. Update the front road-wheel angle with a rate limit.
4. Update engine, gear, and brake/handbrake torque commands.
5. Compute the contact-point velocity for every wheel.
6. Compute each wheel's slip angle and slip ratio.
7. Compute static and dynamic normal loads.
8. Query the surface under each wheel.
9. Compute pure longitudinal and pure lateral tire forces.
10. Apply the combined-friction limit per wheel.
11. Rotate steered-wheel forces into the body frame.
12. Sum body forces and yaw torque.
13. Add aerodynamic drag and rolling resistance.
14. Blend dynamic and low-speed derivatives.
15. Integrate body velocity and yaw rate with semi-implicit Euler.
16. Integrate heading and world position.
17. Update wheel angular velocities from drive, brake, and reaction torques.
18. Store derived diagnostics; store `prevLongAccelMps2`.
19. Evaluate drift/scoring state, strictly separate from physical state.
20. Assert all values are finite and within safety bounds; store the current transform for
    render interpolation and replay validation.

### Steering

`frontRoadWheelAngleRad` is the **road-wheel angle**, not a steering-wheel angle. It is
rate-limited rather than exponentially smoothed, so the maximum steering speed is a defined
physical quantity:

```c
float target = input.steer * spec->maxRoadWheelAngleRad;
float error  = target - state->frontRoadWheelAngleRad;

float rate      = (fabsf(target) < fabsf(state->frontRoadWheelAngleRad))
                  ? spec->steerReturnRateRadS      // returning to center is faster
                  : spec->maxSteerRateRadS;
float maxChange = rate * dt;

state->frontRoadWheelAngleRad += clampf(error, -maxChange, maxChange);
```

Both front wheels receive this angle in the bicycle phases. Ackermann differentiation
between the front-left and front-right wheels is a Phase 4 refinement.

`maxRoadWheelAngleRad = 0.70` (≈40°) is the default. A value near 0.90 rad (≈51.6°) is a
**modified drift steering rack**, valid for a dedicated drift-car profile but extreme for a
generic road car; document it as such wherever it is used.

Any speed-sensitive steering assistance is a separate optional controller in the
control/feel layer. It must not be embedded in the physical steering model.

### Contact-point velocities and slip angles

For a wheel whose contact patch sits at body-frame position `(px, py)`:

```
wheel_vx = vx - r * py
wheel_vy = vy + r * px
```

For the axle bicycle model this reduces, with `l_f = cgToFrontM` and `l_r = cgToRearM`, to:

```
axle_vy_front = vy + l_f * r
axle_vy_rear  = vy - l_r * r
axle_vx       = vx                    (py = 0 on the centerline)
```

Slip angles, using a denominator floor to avoid division by zero:

```
vx_safe = max(fabsf(wheel_vx), LOW_SPEED_EPSILON_MPS)

alpha_front = atan2f(axle_vy_front, vx_safe) - frontRoadWheelAngleRad
alpha_rear  = atan2f(axle_vy_rear,  vx_safe)
```

`LOW_SPEED_EPSILON_MPS = 0.5`. The epsilon alone is **not** the low-speed solution; see
[Low-speed behavior](#low-speed-behavior).

The front and rear slip angles respond differently to yaw rate because `l_f` and `l_r` are
distinct lever arms. Changing either distance must measurably change that axle's slip
response — this is a validation check, not just a claim.

### Normal loads and longitudinal load transfer

With `l_f`, `l_r`, `L = l_f + l_r`, `h = cgHeightM`, `g = 9.80665 m/s²`, and `ax` the
longitudinal acceleration:

```
Fz_front = mass * g * l_r / L  -  mass * ax * h / L
Fz_rear  = mass * g * l_f / L  +  mass * ax * h / L
```

Positive `ax` (accelerating forward) transfers load rearward; braking transfers it forward.
These are **forces in newtons**, not masses.

Loads are clamped so a wheel can be unloaded but never generates negative grip:

```c
Fz_front = fmaxf(Fz_front, MIN_NORMAL_LOAD_N);   // MIN_NORMAL_LOAD_N = 50.0f
Fz_rear  = fmaxf(Fz_rear,  MIN_NORMAL_LOAD_N);
```

`ax` must **not** be a finite difference of a velocity that the current step's tire forces
have already modified. Use the previous step's solved longitudinal acceleration, passed
through a first-order filter:

```c
state->filteredLongAccelMps2 +=
    (state->prevLongAccelMps2 - state->filteredLongAccelMps2) *
    (1.0f - expf(-LOAD_FILTER_RATE_HZ * dt));       // LOAD_FILTER_RATE_HZ = 20.0f
```

In the bicycle model each axle load is split evenly between its two `WheelState` entries.
Lateral load transfer is a Phase 4 upgrade.

### Lateral tire force

A single normalized convention is used throughout: the Magic-Formula-style curve is
**dimensionless**, and force comes from multiplying by the friction coefficient and the
normal load.

```
curve(x)   = sinf(C * atanf(B * x))          // dimensionless, peaks near 1 for C ~ 1.3-1.6
Fy_steady  = -mu_lateral * Fz * curve(alpha)
```

The leading minus sign implements "lateral force opposes slip angle". `mu_lateral` is the
product of the vehicle's axle coefficient and the surface's `muLateral`; `B` is scaled by
the surface's `tireBScale`.

Properties this guarantees, all of which are asserted in tests:

- Force is measured in newtons.
- Force at zero slip is zero.
- Force initially opposes slip and rises approximately linearly with slope
  `mu * Fz * B * C`.
- Peak magnitude is approximately `mu * Fz`.
- Past the peak, force falls off but does not go to zero — this is the drift zone, and
  countersteering reduces slip back toward the peak, restoring authority.

`mu` sets the peak magnitude. The slip angle at which the peak occurs is set by `B` and
`C` together — lowering rear `mu` alone lowers rear grip, it does not by itself move the
rear peak to a smaller slip angle. Tune `B` and `C` when the peak location needs to move.

A debug view must be able to plot force versus slip angle for each axle.

### Wheel dynamics, drivetrain, and slip ratio

The first car is **rear-wheel drive** with a locked rear axle (both rear wheels share one
angular velocity in the bicycle phases).

Engine speed is derived from the driven-wheel speed through the transmission:

```
gear_total = gearRatios[selectedGear] * finalDriveRatio
engineRpm  = clamp(|omega_rear| * gear_total * 60 / (2*PI),
                   engineIdleRpm, engineRedlineRpm)
```

Engine torque is a small piecewise curve sampled evenly between idle and redline
(`engineTorqueCurveNm`), interpolated linearly. A full engine simulation is out of scope;
a piecewise curve is sufficient and keeps power delivery physical.

```
engine_torque = throttle * lerp_curve(engineTorqueCurveNm, engineRpm)
                - (1 - throttle) * engineBrakingTorqueNm
drive_torque_per_rear_wheel =
    engine_torque * gear_total * drivetrainEfficiency * 0.5
```

Brake torque:

```
front_brake_torque = brake * maxBrakeTorqueNm * brakeBiasFront * 0.5
rear_brake_torque  = brake * maxBrakeTorqueNm * (1 - brakeBiasFront) * 0.5
                     + handbrake * handbrakeTorqueNm * 0.5
```

Longitudinal slip ratio, per wheel:

```
wheel_surface_speed = angularVelocityRadS * wheelRadiusM

slip_ratio = (wheel_surface_speed - wheel_vx) /
             max(fabsf(wheel_vx), SLIP_SPEED_EPSILON_MPS)     // 1.0 m/s
```

Pure longitudinal force uses the same curve shape:

```
Fx_pure = mu_longitudinal * Fz * curve_long(slip_ratio)
        = mu_longitudinal * Fz * sinf(C_long * atanf(B_long * slip_ratio))
```

Slip ratio is clamped to ±`SLIP_RATIO_CLAMP` (4.0) before evaluation to bound the argument
during wheelspin and lockup.

Wheel angular velocity is integrated **after** the body, from the combined-limited tire
force:

```
net_torque = drive_torque
           - brake_torque * sign(angularVelocityRadS)
           - forceLongitudinalN * wheelRadiusM

angularVelocityRadS += net_torque / wheelInertiaKgM2 * dt
```

Brake torque must not spin a wheel backwards within a step. If applying it would reverse
the sign of `angularVelocityRadS` while no drive torque exceeds it, set the wheel speed to
zero and mark `locked = true`. A locked wheel produces a slip ratio of `-1` at speed and
therefore saturates its longitudinal friction budget.

Wheel inertia is small relative to the timestep, so the wheel equation is the stiffest part
of the model. At 120 Hz with `wheelInertiaKgM2 = 1.2` it is stable; if wheelspin exhibits
oscillation, raise the physics rate to 240 Hz rather than damping the tire curve.

### Combined longitudinal and lateral grip

Longitudinal and lateral forces share a finite friction budget. After computing pure
`Fx_requested` and pure `Fy_pure` for a wheel:

```
nx    = Fx_requested / (mu_longitudinal * Fz)
ny    = Fy_pure      / (mu_lateral      * Fz)
usage = sqrtf(nx*nx + ny*ny)

if (usage > 1.0f) {
    Fx = Fx_requested / usage;
    Fy = Fy_pure      / usage;
} else {
    Fx = Fx_requested;
    Fy = Fy_pure;
}

wheel->frictionUsage = fminf(usage, 1.0f);
```

This friction ellipse is a pragmatic first approximation. A combined-slip Magic Formula or
brush model can replace it later without changing the surrounding architecture, because
the interface — pure forces in, limited forces plus usage out — stays the same.

Direct consequences that the model must exhibit:

- Full throttle can break rear traction without touching the handbrake (power oversteer).
- Reducing throttle frees rear friction budget and restores rear lateral authority.
- Braking while cornering reduces available lateral force.
- Rear `Fx` and `Fy` never exceed the configured budget.

### Handbrake

The handbrake is **rear brake torque**. It is never a lateral-grip multiplier. Its effect
on cornering is entirely emergent:

1. `handbrakeTorqueNm` is applied to the rear wheels.
2. Rear wheel angular speed drops, and typically locks.
3. Large negative rear longitudinal slip develops.
4. The rear friction ellipse is consumed by longitudinal force.
5. Available rear lateral force collapses, and the rear steps out.

Releasing the handbrake lets the rear wheels spin back up, longitudinal slip decays, and
lateral authority returns without any special-case code.

### Force transformation and planar rigid-body dynamics

Front tire forces are computed in the **steered wheel frame** and must be rotated into the
body frame by the road-wheel angle `delta`:

```
Fx_front_body = Fx_front * cosf(delta) - Fy_front * sinf(delta)
Fy_front_body = Fx_front * sinf(delta) + Fy_front * cosf(delta)
```

Rear wheels are aligned with the body:

```
Fx_rear_body = Fx_rear
Fy_rear_body = Fy_rear
```

Totals, with resistance forces added in the body frame:

```
Fx_total = Fx_front_body + Fx_rear_body - drag_body_x - rolling_body_x
Fy_total = Fy_front_body + Fy_rear_body - drag_body_y - rolling_body_y
```

Body-frame accelerations, including the rotational transport terms exactly once:

```
ax_body = Fx_total / mass
ay_body = Fy_total / mass

dvx_dt = ax_body + r * vy
dvy_dt = ay_body - r * vx
```

Yaw torque, using the same lever arms as the slip-angle calculation:

```
yaw_torque = l_f * Fy_front_body - l_r * Fy_rear_body
dr_dt      = yaw_torque / yawInertiaKgM2
```

In the four-wheel model, longitudinal forces also contribute yaw torque through the track
width; in the bicycle model both contact points lie on the centerline, so they do not.

Integration is **semi-implicit (symplectic) Euler**, in this exact documented order:

```c
state->velocityLongitudinalMps += dvx_dt * dt;
state->velocityLateralMps      += dvy_dt * dt;
state->yawRateRadS             += dr_dt  * dt;

state->headingRad = wrap_angle(state->headingRad + state->yawRateRadS * dt);

// world velocity is computed from the UPDATED body velocity and heading
float world_vx = state->velocityLongitudinalMps * cosf(state->headingRad)
               - state->velocityLateralMps      * sinf(state->headingRad);
float world_vy = state->velocityLongitudinalMps * sinf(state->headingRad)
               + state->velocityLateralMps      * cosf(state->headingRad);

state->positionM.x += world_vx * dt;
state->positionM.y += world_vy * dt;

state->prevLongAccelMps2 = ax_body;
```

`ax_body` (not `dvx_dt`) is what feeds next step's load transfer, because load transfer
responds to the longitudinal force on the body, not to the centripetal transport term.

### Low-speed behavior

A dynamic tire model divides by longitudinal speed and is undefined or unstable near zero.
An epsilon alone hides the symptom; a model blend is required.

```
LOW_SPEED_BEGIN_MPS = 1.5
LOW_SPEED_END_MPS   = 3.0

blend = smoothstep(LOW_SPEED_BEGIN_MPS, LOW_SPEED_END_MPS, fabsf(vx))
```

- Below 1.5 m/s: kinematic (heavily damped) model.
- Between 1.5 and 3.0 m/s: blended.
- Above 3.0 m/s: full dynamic tire model.

The kinematic model constrains yaw rate and lateral velocity geometrically:

```
r_kinematic  = vx * tanf(delta) / L
beta         = atan2f(l_r * tanf(delta), L)
vy_kinematic = vx * tanf(beta)
```

The blend is applied to the **derivatives**, not to the state, so the state stays single-valued:

```c
Derivatives dyn = DynamicVehicleModel(spec, state, cmd, dt);
Derivatives kin = KinematicVehicleModel(spec, state, cmd, dt);
Derivatives out = derivatives_lerp(kin, dyn, blend);
```

Reverse driving is defined explicitly: slip angles use `fabsf(vx)` in the denominator and
the sign of `vx` is carried through the `atan2f` numerator, so reversing does not silently
invert the sign of every tire force. Below the low-speed threshold the kinematic model
governs in both directions.

Required behavior:

- The vehicle can start from rest without oscillating.
- Steering while stationary produces no lateral acceleration.
- Braking to zero produces no force spike.
- Slow reversing is stable.
- The transition into the dynamic model is not visually abrupt.

### Aerodynamic drag and rolling resistance

These are separate physical forms, not one arbitrary quadratic constant.

```
speed        = sqrtf(vx*vx + vy*vy)
aero_drag_N  = 0.5 * AIR_DENSITY_KGM3 * dragCoefficient * frontalAreaM2 * speed*speed
```

Aerodynamic drag acts **opposite the velocity vector**, not automatically along the
vehicle's forward axis — a sideways car still experiences drag:

```
drag_body_x = aero_drag_N * (vx / max(speed, epsilon))
drag_body_y = aero_drag_N * (vy / max(speed, epsilon))
```

Rolling resistance is applied per wheel, opposing that wheel's contact velocity, using the
wheel's surface coefficient:

```
const SurfaceSpec *s = Surface_Get(wheel->surfaceId);
rolling_N = s->rollingResistanceCoefficient * wheel->normalLoadN
```

Loose surfaces additionally apply `looseSurfaceDragN` opposing the contact velocity.

`AIR_DENSITY_KGM3 = 1.225`.

### Runtime assertions and safety bounds

Every fixed update ends with these checks, active in debug builds and in the test harness:

```c
assert(isfinite(all state values));
assert(Fz_front >= 0.0f && Fz_rear >= 0.0f);
assert(wheel->frictionUsage <= 1.0f + FRICTION_TOLERANCE);   // 1e-3
assert(fabsf(speedMps)  < MAX_SAFE_SPEED_MPS);               // 120.0
assert(fabsf(yawRateRadS) < MAX_SAFE_YAW_RATE_RADS);         // 20.0
```

In release builds a violation logs through `TRACELOG(LOG_WARNING, ...)` and resets the
vehicle to the last known-good state rather than aborting.

---

## Baseline Vehicle Parameters

Starting values for a small rear-wheel-drive drift car. These are starting ranges for
tuning, not final values.

```c
// config.h — units are stated for every constant

#define PIXELS_PER_METER        24.0f     // render scale only; never used by physics
#define GRAVITY_MPS2            9.80665f
#define AIR_DENSITY_KGM3        1.225f

// Mass and geometry
#define VEH_MASS_KG             1200.0f
#define VEH_YAW_INERTIA_KGM2    1800.0f   // useful range 1500..2200
#define VEH_CG_TO_FRONT_M       1.15f     // l_f
#define VEH_CG_TO_REAR_M        1.40f     // l_r  (L = 2.55 m)
#define VEH_CG_HEIGHT_M         0.50f     // h
#define VEH_TRACK_FRONT_M       1.48f
#define VEH_TRACK_REAR_M        1.46f

// Wheels
#define WHEEL_RADIUS_M          0.31f
#define WHEEL_INERTIA_KGM2      1.20f

// Steering (road-wheel angle; 0.70 rad ~ 40 deg. 0.90 rad ~ 51.6 deg is a drift rack)
#define STEER_MAX_RAD           0.70f
#define STEER_RATE_RAD_S        5.0f
#define STEER_RETURN_RATE_RAD_S 7.0f

// Resistance
#define DRAG_COEFFICIENT        0.32f     // dimensionless
#define FRONTAL_AREA_M2         1.90f
#define ROLLING_RESISTANCE_COEF 0.015f    // dimensionless

// Lateral tire curve (normalized; force = mu * Fz * sin(C * atan(B * alpha)))
#define TIRE_B_LAT_FRONT        10.0f     // dimensionless stiffness
#define TIRE_C_LAT_FRONT        1.45f     // dimensionless shape
#define TIRE_MU_LAT_FRONT       1.30f     // dimensionless peak friction
#define TIRE_B_LAT_REAR         10.0f
#define TIRE_C_LAT_REAR         1.45f
#define TIRE_MU_LAT_REAR        1.20f     // lower than front = oversteer bias

// Longitudinal tire curve
#define TIRE_B_LONG             12.0f
#define TIRE_C_LONG             1.55f
#define TIRE_MU_LONG_SCALE      1.00f     // multiplies the surface longitudinal mu

// Drivetrain (rear-wheel drive, locked rear axle)
#define MAX_GEARS               8         // VehicleSpec.gearRatios array size
#define ENGINE_CURVE_POINTS     7         // VehicleSpec.engineTorqueCurveNm array size
#define GEAR_RATIOS             { 3.55f, 2.05f, 1.38f, 1.00f, 0.82f }
#define GEAR_COUNT              5
#define REVERSE_GEAR_RATIO      3.20f
#define FINAL_DRIVE_RATIO       4.10f
#define DRIVETRAIN_EFFICIENCY   0.90f     // dimensionless
#define ENGINE_IDLE_RPM         900.0f
#define ENGINE_REDLINE_RPM      7000.0f
#define ENGINE_BRAKING_TORQUE_NM 35.0f
// Sampled evenly from idle to redline, newton-meters:
#define ENGINE_TORQUE_CURVE_NM  { 140.0f, 200.0f, 240.0f, 255.0f, 250.0f, 230.0f, 195.0f }

// Brakes
#define MAX_BRAKE_TORQUE_NM     3000.0f
#define BRAKE_BIAS_FRONT        0.62f     // dimensionless
#define HANDBRAKE_TORQUE_NM     1800.0f

// Numerical guards
#define LOW_SPEED_EPSILON_MPS   0.50f
#define SLIP_SPEED_EPSILON_MPS  1.00f
#define SLIP_RATIO_CLAMP        4.0f
#define MIN_NORMAL_LOAD_N       50.0f
#define LOAD_FILTER_RATE_HZ     20.0f
#define LOW_SPEED_BEGIN_MPS     1.5f
#define LOW_SPEED_END_MPS       3.0f
#define FRICTION_TOLERANCE      0.001f
#define MAX_SAFE_SPEED_MPS      120.0f
#define MAX_SAFE_YAW_RATE_RADS  20.0f

// Fixed timestep
#define FIXED_DT_S              (1.0f / 120.0f)
#define MAX_PHYSICS_STEPS       8
#define MAX_FRAME_TIME_S        0.25f

// Presentation and gameplay (Phase 6; no effect on the simulation layer)
#define SCREEN_W                1280      // pixels
#define SCREEN_H                720       // pixels
#define TARGET_FPS              60
#define MAX_PARTICLES           512
#define PARTICLE_LIFE_S         0.80f
#define CAMERA_BASE_ZOOM        1.20f     // dimensionless; >1 magnifies (zooms in)
#define CAMERA_ZOOM_RANGE       0.25f     // dimensionless; subtracted at full drift -> 0.95
#define CAMERA_MIN_ZOOM         0.50f     // dimensionless; must stay > 0
#define CAMERA_ZOOM_RATE        4.0f      // 1/second smoothing rate
#define CAMERA_LOOKAHEAD        0.25f     // seconds of velocity lookahead
#define DRIFT_ZOOM_REF_RAD      0.70f     // sideslip mapped to full zoom-out
#define SLIDE_USAGE_THRESHOLD   0.98f     // dimensionless friction usage = physically sliding
#define SCORE_BASE_RATE         100.0f    // points/second at full factors and combo 1.0
#define SCORE_SPEED_REF_MPS     35.0f     // speed at which speedFactor saturates
#define COMBO_GRACE_S           1.50f
#define MAX_VALID_SCORE         100000000L
```

---

## Game Loop — Fixed Timestep

The accumulator is capped in **substeps**, not only in frame time. Clamping frame time to
0.25 s still permits 30 physics steps at 120 Hz, which is not a sufficient guard.

The loop lives in the platform layer (`main.c`), which owns the `Game` block and calls into
the game module through the reloadable interface described in
[Development Workflow](#development-workflow).

```c
InitWindow(SCREEN_W, SCREEN_H, "Drifty");
SetTargetFPS(TARGET_FPS);

Game *game = calloc(1, sizeof(Game));       // platform-owned; survives hot reload
game_init(game);

while (!WindowShouldClose()) {
    Game_MaybeHotReload(game);                  // dev builds only; no-op in release

    input_sample(&game->input);                 // once per render frame

    float frameTime = GetFrameTime();
    if (frameTime > MAX_FRAME_TIME_S) frameTime = MAX_FRAME_TIME_S;
    game->accumulatorS += frameTime;

    int steps = 0;
    while (game->accumulatorS >= FIXED_DT_S && steps < MAX_PHYSICS_STEPS) {
        game_fixed_update(game, FIXED_DT_S);    // consumes one-shot inputs on first call
        game->accumulatorS -= FIXED_DT_S;
        steps++;
    }

    if (steps == MAX_PHYSICS_STEPS && game->accumulatorS >= FIXED_DT_S) {
        game->accumulatorS = fmodf(game->accumulatorS, FIXED_DT_S);
        game->physicsBacklogDrops++;            // surfaced in the debug HUD
    }
    game->lastSubstepCount = steps;

    float alpha = game->accumulatorS / FIXED_DT_S;
    game_draw(game, alpha);
}

game_shutdown(game);
CloseWindow();
```

### One-shot input consumption

`game_fixed_update` reads the one-shot flags, acts on them, and clears them before
returning. A single-frame reset press therefore resets exactly once even when eight
substeps run in that frame. Held controls (`steer`, `throttle`, `brake`, `handbrake`)
remain valid for every substep of the frame.

### Render interpolation

Render interpolation exists from the first playable physics milestone, not as a later fix.
Each fixed update copies `curr*` into `prev*` before integrating; the renderer blends:

```c
Vector2 drawPosM = Vector2Lerp(rs->prevPositionM, rs->currPositionM, alpha);
float   drawHeadingRad = lerp_angle(rs->prevHeadingRad, rs->currHeadingRad, alpha);
```

`lerp_angle` must take the **shortest wrapped angular path** so a heading crossing ±π does
not spin the sprite through a full rotation. Wheel angles interpolate the same way.

---

## Input Mapping

| Action | Keys | Sampling |
|--------|------|----------|
| Steer left | A / Left Arrow | `IsKeyDown` (held) |
| Steer right | D / Right Arrow | `IsKeyDown` (held) |
| Throttle | W / Up Arrow | `IsKeyDown` (held) |
| Brake | S / Down Arrow | `IsKeyDown` (held) |
| Handbrake | Space | `IsKeyDown` (held) |
| Shift up / down | E / Q | `IsKeyPressed` (one-shot) |
| Pause | P | `IsKeyPressed` (one-shot) |
| Reset | R | `IsKeyPressed` (one-shot) |
| Debug overlay | F1 | `IsKeyPressed` (one-shot) |

Steering input maps to the left-positive convention: left key → `+1`, right key → `-1`.

Keyboard steering is binary, so an optional **keyboard steering controller** in the
control/feel layer may shape the `steer` value (rate shaping, speed-sensitive scaling,
counter-steer assist). It writes only to `Input.steer`; the tire model is never altered to
compensate for input device.

---

## Rendering Pipeline

1. **Camera** — `Camera2D` follows the car's interpolated position in render pixel space,
   smoothed with `smooth_to`, plus velocity-based lookahead (`CAMERA_LOOKAHEAD = 0.25`).
2. **Draw order** (inside `BeginMode2D`): track surface → skid marks → particles → car →
   debug vectors. HUD is drawn after `EndMode2D`.
3. **Car body** — Phase 1: `DrawRectanglePro` with `rotation = -headingRad * RAD2DEG`.
   Phase 6: nose triangle for heading readability, then optional `DrawTexturePro` with
   `assets/car.png`.
4. **Wheels** — four small rectangles at the wheels' `localPositionM`, converted to pixels.
   **Only the front wheels steer**: front wheels are drawn at
   `heading + frontRoadWheelAngleRad`, rear wheels at `heading`. Rear-wheel steering is not
   part of this specification; adding it later requires an explicit rear-steer field in
   `WheelState` and matching physics.
5. **HUD** — screen space: speed (km/h), gear, engine RPM, drift score, combo multiplier,
   lap/checkpoint, best score.
6. **Debug overlay (F1)** — per-wheel slip angle, slip ratio, normal load, friction usage;
   body sideslip; yaw rate; substep count and `physicsBacklogDrops`; axle velocity vectors
   and wheel heading vectors drawn with `DrawLineDashed`; tire force vectors drawn from
   each contact patch. This overlay is the primary tool for catching steering and
   tire-force sign errors.

### Camera zoom

In raylib, `Camera2D.zoom = 1.0` means no scaling and **larger values magnify the world**
(zoom in). To zoom *out* during a drift, the target zoom must decrease:

```c
float driftIntensity = clampf(fabsf(derived->bodySideslipRad) / DRIFT_ZOOM_REF_RAD, 0.0f, 1.0f);
float targetZoom     = CAMERA_BASE_ZOOM - driftIntensity * CAMERA_ZOOM_RANGE;  // 1.20 -> 0.95

targetZoom   = fmaxf(targetZoom, CAMERA_MIN_ZOOM);      // 0.5, always > 0
camera.zoom  = smooth_to(camera.zoom, targetZoom, CAMERA_ZOOM_RATE, renderDt);
```

Zoom is smoothed against **render** delta time and is independent of the physics state
machine, so it never affects simulation determinism.

---

## Track, Surfaces, and Collision

### Geometry

The track is a centerline of `TrackNode` entries (position, half-width, surface id), in
meters. Phase 5 hand-authors an oval of roughly 24 nodes as an array literal. Rendering
draws the centerline and offset boundaries with `DrawLineEx` in render pixel space.

### Surface querying

There is no global off-track grip multiplier. Surfaces are queried **per wheel contact
point**, so the vehicle can straddle two surfaces naturally with one pair of wheels on
asphalt and the other on grass:

```c
SurfaceId Track_SurfaceAt(const Track *track, Vector2 pointM);
```

Each wheel's `surfaceId` is refreshed in step 8 of the fixed update, and the corresponding
`SurfaceSpec` is resolved through `Surface_Get()` at point of use. The surface
contributes `muLongitudinal`, `muLateral`, `tireBScale`, `rollingResistanceCoefficient`,
and `looseSurfaceDragN` to that wheel only. Lateral friction, longitudinal traction,
rolling resistance, and terrain drag are therefore independent quantities rather than one
conflated multiplier.

### Collision

Collision uses a **vehicle shape**, not a single center point:

- Phase 5 baseline: a capsule (two circles plus the connecting body) covering the car's
  footprint.
- High speed requires **swept tests** between the previous and current transform so the
  car cannot tunnel through a barrier in one 1/120 s step.
- Upgrade to an oriented bounding box when barrier interactions become gameplay-relevant.

Barrier response resolves penetration along the contact normal, reflects the normal
velocity component with a restitution coefficient, applies friction to the tangential
component, and applies the resulting impulse at the contact point so the car gains yaw from
a glancing hit.

### Checkpoints and laps

Checkpoint gates are line segments at segment midpoints. `Track_PassCheckpoint(pointM)`
advances the checkpoint index; wrapping from last to first increments `lap` and records the
split time.

---

## Drift Detection and Scoring

Physical state and scoring state are strictly separate. `physicallySliding` describes the
tires; `scoringDrift` describes the game rules. Scoring state never modifies forces.

### Physical classification

```c
derived->physicallySliding = (derived->maxFrictionUsage >= SLIDE_USAGE_THRESHOLD); // 0.98
```

### Scoring classification

Rear slip angle alone would classify creeping, reversing, spinning in place, crashing, and
facing backwards as drifting. A scoring drift requires **all** of:

```
speedMps                    >= MIN_DRIFT_SPEED_MPS
fabsf(bodySideslipRad)      >= MIN_DRIFT_ANGLE_RAD
fabsf(rear slip angle)      >= MIN_REAR_SLIP_RAD
fabsf(yawRateRadS)          >= MIN_DRIFT_YAW_RATE_RADS
vehicle is on a valid (scoring) surface
not crashed (no barrier impact within CRASH_LOCKOUT_S)
not reversing (velocityLongitudinalMps > 0)
fabsf(bodySideslipRad)      <= SPIN_CUTOFF_RAD
```

Body sideslip:

```
beta = atan2f(velocityLateralMps, fmaxf(fabsf(velocityLongitudinalMps), LOW_SPEED_EPSILON_MPS))
```

Initial thresholds, to be tuned after physics behavior is stable:

```c
#define MIN_DRIFT_SPEED_MPS      5.0f
#define MIN_DRIFT_ANGLE_RAD      0.175f   // 10 degrees
#define MIN_REAR_SLIP_RAD        0.12f
#define MIN_DRIFT_YAW_RATE_RADS  0.25f
#define SPIN_CUTOFF_RAD          1.48f    // ~85 degrees
#define CRASH_LOCKOUT_S          1.0f
```

### Score accumulation

Scoring uses normalized factors, never raw radians multiplied by pixel-based speeds:

```
angleFactor = normalized |beta| within [MIN_DRIFT_ANGLE_RAD, SPIN_CUTOFF_RAD]
speedFactor = normalized speed above MIN_DRIFT_SPEED_MPS, saturating at SCORE_SPEED_REF_MPS
lineFactor  = optional racing-line/proximity quality, defaults to 1.0

score += SCORE_BASE_RATE * angleFactor * speedFactor * lineFactor * comboMultiplier * dt
```

```
comboMultiplier = clamp(1.0 + driftTimeS * 0.5, 1.0, 4.0)
combo resets to 1.0 after COMBO_GRACE_S (1.5 s) outside a scoring drift
```

Score accrues per fixed tick while `scoringDrift` is true.

---

## High-Score Persistence

`LoadFileText()` returns allocated memory that must be released with `UnloadFileText()`,
and save data may be missing or corrupt. The required sequence:

```c
// Load
if (FileExists(scorePath)) {
    char *text = LoadFileText(scorePath);
    if (text != NULL) {
        long parsed = strtol(text, &end, 10);
        if (end != text && parsed >= 0 && parsed <= MAX_VALID_SCORE) {
            game->bestScore = (float)parsed;
        }                       // otherwise keep the default; do not trust the file
        UnloadFileText(text);   // required
    }
}

// Save
if (!SaveFileText(scorePath, TextFormat("%d", (int)game->bestScore))) {
    TRACELOG(LOG_WARNING, "Failed to write high score to %s", scorePath);
}
```

Requirements:

- Check `FileExists()` before loading.
- Validate and clamp parsed values; never accept impossible scores.
- Always call `UnloadFileText()`.
- Always check the boolean result of `SaveFileText()`.
- Write to a **user-writable save directory** (for example `%APPDATA%/drifty/` on Windows).
  Do not assume the executable's directory is writable.

The same validated-load pattern applies to any later track or configuration file loading.

---

## Physics Validation and Regression Testing

A separate **headless physics test executable** is built before track work begins. It links
the physics translation units and never calls `InitWindow` — physics code calls no raylib
functions, so no window, GL context, or audio device is required.

```bash
build.bat --tests            # or ./build.sh --tests; sources come from the Makefile manifest
./build/tests/drifty_tests.exe
```

Each scenario drives the vehicle from a scripted input timeline at a fixed 120 Hz and
writes one CSV file per scenario to `artifacts/telemetry/`. Every CSV row carries at minimum:
time, position, heading, body velocities, yaw rate, body sideslip, front/rear slip angle,
front/rear slip ratio, front/rear normal load, per-wheel `Fx`/`Fy`/`frictionUsage`, wheel
angular velocities, engine RPM, gear, and total yaw torque.

### Required scenarios

| # | Scenario | What it verifies |
|---|----------|------------------|
| 1 | **Rest stability** | Vehicle stays at rest with no input; no NaN, no force growth, normal loads sum to `mass * g` |
| 2 | **Straight-line acceleration** | Records speed, wheel speed, RPM, and distance; monotonic speed increase under full throttle |
| 3 | **Coast-down** | Drag and rolling resistance reduce speed monotonically to zero without a spike |
| 4 | **Constant-radius skidpad** | Records steering angle, lateral acceleration, yaw rate, and slip angles at increasing speed; understeer gradient is measurable |
| 5 | **Step-steer response** | Fixed steering input at fixed speed; records yaw-rate rise time and overshoot |
| 6 | **Lift-off transient** | Throttle held in a turn then released; records axle loads and the yaw-rate change (lift-off oversteer) |
| 7 | **Power oversteer** | Steering held while rear drive torque rises; confirms rear combined-slip saturation occurs before front |
| 8 | **Handbrake entry** | Confirms rear wheel speed drops or locks and rear friction usage rises to the limit |
| 9 | **Drift transition** | Steering reversed mid-slide; records front/rear slip, yaw rate, and sideslip through the transition |
| 10 | **Deterministic replay** | The same timestamped input sequence run twice produces an identical final-state checksum |
| 11 | **Render-scale independence** | Running with `PIXELS_PER_METER` doubled produces a bit-identical physics checksum |
| 12 | **Low-speed launch and reverse** | Start from rest, stop, and reverse without oscillation or force spikes across the blend region |

### Required assertions (all scenarios)

```c
isfinite(every state and derived value)
normalLoadFrontN >= 0 && normalLoadRearN >= 0
frictionUsage <= 1.0 + FRICTION_TOLERANCE
fabsf(speedMps) < MAX_SAFE_SPEED_MPS
fabsf(yawRateRadS) < MAX_SAFE_YAW_RATE_RADS
at rest: |Fz_front + Fz_rear - mass * g| < 1 N
accelerating: Fz_rear increases and Fz_front decreases
braking: Fz_front increases and Fz_rear decreases
tire lateral force is zero at zero slip and opposes slip elsewhere
peak lateral force is within tolerance of mu * Fz
```

### Regression workflow

- CSV output is committed as a baseline whenever handling is intentionally changed.
- Every tuning change is diffed against the previous CSV baseline.
- A test failure must distinguish an intentional handling change from a broken equation:
  scenario assertions (invariants) are hard failures; CSV deltas are reviewed.
- The test executable runs in CI and requires no display.

---

## Project Structure

```
drifty/
├── Makefile
├── build.sh / build.bat        # Hot-reload dev build; always terminates immediately
├── mk.bat                      # `make` from any Windows shell (enters UCRT64)
├── scripts/
│   ├── setup_windows.ps1       # Idempotent MSYS2 UCRT64 bootstrap
│   └── validate_hotreload.sh   # Windowless harness + failed-compile preservation
├── README.md
├── docs/
│   ├── SPEC.md                 # This document
│   ├── SOURCES.md              # Technical reference index
│   ├── generated/              # PARAMETERS.md, CORPUS.md — generated, never hand-edited
│   └── plans/                  # PLAN.md, ROADMAP.md
├── src/
│   ├── platform/               # --- platform layer (drifty.exe; not hot-reloadable) ---
│   │   ├── main.c              # Entry point, window init, Game allocation, fixed-timestep loop
│   │   ├── hotreload.h         # GAME_ENTRY_POINTS list, shared by both layers
│   │   ├── hotreload_windows.c # LoadLibrary / GetProcAddress / FreeLibrary
│   │   ├── timestep.h/.c       # The accumulator, isolated so the harness can assert it
│   │   └── build_info.h        # Commit/branch/dirty provenance baked into the binary
│   ├── core/                   # --- conventions every other domain depends on ---
│   │   ├── config.h            # All tunables, with units
│   │   ├── math_utils.h/.c     # clampf, lerpf, smooth_to, wrap_angle, smoothstep, lerp_angle
│   │   └── units.h             # PIXELS_PER_METER and world<->render conversion helpers
│   ├── physics/                # --- the vehicle model; raylib-free, headless-linkable ---
│   │   ├── vehicle.h/.c        # VehicleSpec/State/Derived, spec presets
│   │   ├── physics.h/.c        # Fixed update order, body dynamics, integration, low-speed blend
│   │   ├── tire.h/.c           # Lateral/longitudinal curves, combined-friction limit
│   │   ├── drivetrain.h/.c     # Engine curve, gearing, wheel dynamics, brakes, handbrake
│   │   ├── auto_transmission.h/.c  # Shift scheduling
│   │   └── surface.h/.c        # SurfaceSpec table and Surface_Get lookup
│   ├── world/
│   │   ├── track.h/.c          # Track geometry, surface query, checkpoints
│   │   └── collision.h/.c      # Barrier response
│   ├── game/                   # --- orchestration and per-frame systems ---
│   │   ├── game.c              # Entry point implementations, state machine, update dispatch
│   │   ├── game.h              # Game struct
│   │   ├── input.h/.c          # Held controls + one-shot commands
│   │   ├── replay.h/.c         # Deterministic input recording and playback
│   │   ├── audio.h/.c          # Engine and tire audio
│   │   ├── particle.h/.c       # Fixed-size particle pool, smoke trails
│   │   ├── scoring.h/.c        # Drift classification, score, combo, persistence
│   │   ├── profile.h/.c        # Zone timers
│   │   └── telemetry.h/.c      # CSV writer and the row built from Game
│   ├── render/                 # --- everything drawn; render.c is raylib-only ---
│   │   ├── render.h/.c         # Camera, interpolation, draw order, HUD, debug overlay
│   │   ├── car_visual.h/.c     # VehicleSpec -> CarVisual appearance grammar (raylib-free)
│   │   └── car_visual_raster.h/.c  # CarVisual -> pixels (raylib-free)
│   └── dev/                    # --- development tooling; see docs/DEVTOOLS.md ---
│       ├── dev_params.h/.c     # The tunable registry
│       ├── dev_lab.h/.c        # The raygui Physics Lab (development builds only)
│       ├── dev_presets.h/.c    # Named spec presets
│       ├── dev_replay.h/.c     # Replay inspector
│       ├── dev_scenario.h/.c   # Scripted input timelines
│       ├── dev_state.h/.c      # DevState, part of Game in every configuration
│       ├── car_corpus.h/.c     # The 100 demonstration vehicles
│       └── failure_bundle.h/.c # The inspectable directory a failing scenario writes
├── tests/
│   ├── test_main.c             # Argument parsing, group iteration, summary and exit code
│   ├── test_commands.h/.c      # --benchmark, --dump-*, --generate-corpus, --verify-*
│   ├── test_scenarios.h        # The scenario registry contract
│   ├── scenarios/              # Scenario bodies, one file per domain
│   ├── support/                # Check harness, appearance metrics, fixtures, contact sheet
│   ├── hotreload/              # Windowless hot-reload validation harness
│   ├── baselines/              # Committed CSV baselines
│   └── visual/                 # Deterministic scene baselines
├── data/                       # Reviewed, tracked, non-code inputs
│   ├── input/                  # gamecontrollerdb.txt
│   └── vehicles/               # Reviewed tuning profiles and the corpus export
├── build/                      # Every generated binary (gitignored)
│   ├── dev/                    # drifty.exe, game.dll, runtime DLLs
│   ├── tests/                  # drifty_tests.exe
│   └── release/                # drifty_release.exe
├── artifacts/                  # All ephemeral run evidence (gitignored)
└── assets/                     # Car textures, sounds (Phase 6)
```

`physics.c` is introduced in **Phase 1** and is the owner of the fixed update order from
that point onward. No phase reimplements body dynamics outside it.

Everything except `main.c` and the `hotreload_*` files compiles into the game module. A
release build compiles both layers into one executable with `DRIFTY_HOT_RELOAD` undefined,
calling the entry points directly — no DLL involved.

---

## Development Workflow

The goal is a loop where code changes appear in the running game without restarting it, and
without any build step that blocks or runs indefinitely. This matters for automated or
agent-driven editing: **every command in this workflow terminates in under a second**. There
is no watcher daemon and no server to start or supervise.

### Division of responsibility

| Layer | Contents | Lifetime |
|-------|----------|----------|
| **Platform** (`build/dev/drifty.exe`) | Window, raylib context, `Game` allocation, fixed-timestep loop, DLL loading | Started once by the developer, left running |
| **Game module** (`game.dll`) | Everything else, including all physics | Rebuilt and swapped in freely while the game runs |

The platform layer owns the `Game` memory block. On reload it hands the same pointer back to
the freshly loaded module, so the car keeps its position, velocity, and score across a code
change.

### The reloadable interface

Entry points are declared once with an X-macro so that adding one means editing a single
line, and the platform layer's function-pointer table and symbol lookups stay in sync
automatically.

```c
// src/platform/hotreload.h
#define GAME_ENTRY_POINTS \
    ENTRY(game_init,          void,  Game *)        /* first-time setup */ \
    ENTRY(game_pre_reload,    void,  Game *)        /* release anything DLL-owned */ \
    ENTRY(game_post_reload,   void,  Game *)        /* re-acquire it */ \
    ENTRY(game_fixed_update,  void,  Game *, float) /* one physics step */ \
    ENTRY(game_draw,          void,  Game *, float) /* render with interpolation alpha */ \
    ENTRY(game_shutdown,      void,  Game *)

#define ENTRY(name, ret, ...) typedef ret (name##_t)(__VA_ARGS__);
GAME_ENTRY_POINTS
#undef ENTRY

#define ENTRY(name, ...) extern name##_t *name;
GAME_ENTRY_POINTS
#undef ENTRY

bool Game_ReloadModule(void);
```

The platform side expands the same list into definitions and lookups:

```c
// src/platform/hotreload_windows.c
#define ENTRY(name, ...) name##_t *name = NULL;
GAME_ENTRY_POINTS
#undef ENTRY

bool Game_ReloadModule(void)
{
    if (gameModule != NULL) FreeLibrary(gameModule);

    gameModule = LoadLibrary(GAME_MODULE_NAME);
    if (gameModule == NULL) {
        TRACELOG(LOG_ERROR, "HOTRELOAD: could not load %s", GAME_MODULE_NAME);
        return false;
    }

    #define ENTRY(name, ...) \
        name = (name##_t *)GetProcAddress(gameModule, #name); \
        if (name == NULL) { \
            TRACELOG(LOG_ERROR, "HOTRELOAD: missing symbol %s", #name); \
            return false; \
        }
    GAME_ENTRY_POINTS
    #undef ENTRY

    return true;
}
```

The loop polls the module's modification time and performs the handshake:

```c
void Game_MaybeHotReload(Game *game)
{
    long modTime = GetFileModTime(GAME_MODULE_NAME);
    if (modTime == lastModTime) return;
    lastModTime = modTime;

    game_pre_reload(game);
    if (!Game_ReloadModule()) return;   // keep running on the old module if the build failed
    game_post_reload(game);
    TRACELOG(LOG_INFO, "HOTRELOAD: reloaded %s", GAME_MODULE_NAME);
}
```

A failed reload must leave the game running on the previous module. A compile error should
never close the window.

### The build script

The script rebuilds the game module unconditionally, and rebuilds the executable **only when
it is not already running**. Either way it exits immediately.

```sh
#!/bin/sh
set -e

# Always rebuild the game module. Write to a temp name first: the linker briefly
# leaves a zero-length file in place, and the running game would load that.
gcc -O0 -g -Wall -Wextra -std=c11 -shared -fPIC -DDRIFTY_HOT_RELOAD \
    src/game/game.c src/physics/physics.c src/physics/tire.c src/physics/drivetrain.c src/physics/vehicle.c \
    src/physics/surface.c src/world/track.c src/game/particle.c src/render/render.c src/game/scoring.c \
    src/game/input.c src/core/math_utils.c \
    -o build/dev/game_tmp.tmp $(pkg-config --cflags --libs raylib)
mv build/dev/game_tmp.tmp build/dev/game.dll

# If the game is already running, that is all there is to do.
if tasklist 2>/dev/null | grep -qi drifty.exe; then
    echo "Hot reloading..."
    exit 0
fi

gcc -O0 -g -Wall -Wextra -std=c11 -DDRIFTY_HOT_RELOAD \
    src/platform/main.c src/platform/hotreload_windows.c \
    -o build/dev/drifty.exe $(pkg-config --cflags --libs raylib)
echo "Built build/dev/drifty.exe — run it and leave it running."
```

The resulting loop:

1. The developer runs `./build/dev/drifty.exe` once and leaves it open.
2. Code is edited — by hand or by an agent.
3. `./build.sh` runs. It returns in well under a second.
4. The running game notices the new module and swaps it in, keeping its state.

Step 3 is the only command an automated editor ever needs to run, and it always terminates.
Its exit status and compiler diagnostics are the feedback signal; the visual result appears
in the window the developer already has open.

### Platform requirements and gotchas

**raylib must be linked as a shared library.** raylib keeps its state in global variables. If
`game.dll` links raylib statically, the module owns that state, and reloading the module
destroys it — the next raylib call crashes. Link against `raylib.dll` and confirm the import
library is what actually got used; on MSYS2 the package ships both a static `libraylib.a`
and an import library, and which one the linker picks is worth verifying explicitly rather
than assuming.

**Write to a temporary filename and rename.** The linker creates the output file before
filling it, so a running game polling for changes can load a zero-length module. Building to
`game_tmp.dll` and renaming makes the swap atomic.

**Emit a fresh PDB per build if you debug on Windows.** A debugger attached to the process
holds a lock on the current PDB and the next build fails. Numbering them
(`game_1.pdb`, `game_2.pdb`, …) avoids this; clean them up on a fresh start.

**Reload-safety rules for game code:**

- No pointer stored in `Game` may point into the module's code or static data. This is why
  `WheelState` holds a `SurfaceId` rather than a `const SurfaceSpec *`.
- No function pointers in persistent state. If an indirection is genuinely needed, rebuild
  the table in `game_post_reload`.
- `game_pre_reload` releases anything the module owns that raylib tracks — textures, sounds,
  audio stream callbacks — and `game_post_reload` re-acquires it.
- Changing the layout of `Game` invalidates the existing block. The reload will read garbage.
  Restart the executable after a struct change; this is expected and is the main limitation
  of the technique.

### Recovering state after a restart

Struct layout changes, platform-layer edits, and crashes all require a restart. The
timestamped input recording required by [Phase 0](#phase-0--foundations-and-test-harness-1-day)
doubles as the recovery mechanism: replay the recorded input timeline at maximum speed on
startup to arrive back at the moment of interest in a fraction of a second. This is worth
wiring to a key (replay last N seconds) early, since it makes restarts cheap enough that
hot reload becomes a convenience rather than a necessity.

### The headless loop

For physics work specifically, the faster loop skips the window entirely:

```bash
./build.sh --tests && ./build/tests/drifty_tests.exe --scenario skidpad
```

This terminates, emits CSV telemetry, and diffs against committed baselines. Prefer it for
tuning and equation work; use the hot-reload loop for feel, camera, and presentation.

---

## Implementation Phases

Each phase ends with a completion checklist. Do not begin a later phase until the current
checklist is fully satisfied.

### Phase 0 — Foundations and Test Harness *(~1 day)*

**Files:** `src/core/config.h`, `src/core/units.h`, `src/core/math_utils.h/.c`,
`src/game/input.h/.c`, `src/game/game.h/.c`, `src/platform/main.c`,
`src/platform/hotreload.h`, `src/platform/hotreload_windows.c`, `build.sh`/`build.bat`,
`tests/test_main.c`, `tests/scenarios/core_tests.c`

| Task | Details |
|------|---------|
| Coordinate convention | Encode the body/world/sign convention in header comments and helper functions |
| SI types and constants | `config.h` with every constant unit-suffixed; `units.h` with the `PIXELS_PER_METER` conversion |
| Fixed timestep | Accumulator, `MAX_PHYSICS_STEPS` cap, backlog-drop counter |
| Input split | Held controls vs one-shot commands; one-shot consumption in the first fixed update |
| Input recording | Timestamped input timeline capture/playback, for deterministic replay |
| Test executable | `drifty_tests` target that links no raylib functions and opens no window |
| CSV telemetry | Row writer used by every scenario |
| Platform/module split | `main.c` owns the window, the `Game` allocation, and the loop; everything else compiles into the game module |
| Hot reload | `GAME_ENTRY_POINTS` X-macro, `hotreload_windows.c`, mtime poll, pre/post-reload handshake |
| Build script | `build.sh`/`build.bat` that rebuilds the module always and the exe only when it is not running |

**Complete when:**

- [ ] `drifty_tests` builds and runs with no display.
- [ ] A one-frame reset press resets exactly once even when eight substeps run.
- [ ] Substep count and dropped backlog are visible in the debug HUD.
- [ ] Recorded input replays reproduce an identical state checksum.
- [ ] Editing game code and running the build script swaps in the new module without
      restarting, preserving state.
- [ ] A deliberate compile error leaves the running game alive on the previous module.
- [ ] The build script exits in under a second in both the running and not-running cases.
- [ ] A release build with `DRIFTY_HOT_RELOAD` undefined produces a single executable with
      no DLL and identical behavior.

### Phase 1 — Rigid-Body Vehicle *(~1 day)*

**Files:** `vehicle.h/.c`, `physics.h/.c`, `render.h/.c`

| Task | Details |
|------|---------|
| Canonical structures | `VehicleSpec`, `VehicleState`, `VehicleDerived`, `VehicleRenderState`, `WheelState[WHEEL_COUNT]` |
| Contact-point velocities | Per-wheel `wheel_vx`, `wheel_vy` from body velocity and yaw rate |
| Per-axle slip angles | `l_f`/`l_r` lever arms, distinct front and rear response |
| Linear tire model | `Fy = -C_alpha * alpha`, saturated at `mu * Fz`; the nonlinear curve arrives in Phase 2 |
| Force transformation | Front forces rotated through `delta` into the body frame |
| Body dynamics | `dvx_dt`, `dvy_dt`, yaw torque, semi-implicit Euler in the documented order |
| Low-speed blend | Kinematic model below 1.5 m/s, blended to 3.0 m/s |
| Steering rate limit | `frontRoadWheelAngleRad` with `maxSteerRateRadS` |
| Interpolated renderer | Rotated rectangle plus four wheel rectangles; front wheels steer, rear wheels do not |
| Debug overlay | Axle velocity vectors, wheel heading vectors, slip angles, yaw rate |

**Complete when:**

- [ ] The car rotates from yaw torque, never from a direct `heading += steer` term.
- [ ] Front and rear slip angles respond differently to yaw rate.
- [ ] Left steering input from straight travel produces positive yaw.
- [ ] Rest stability, low-speed launch, and reverse scenarios pass.
- [ ] Rendering is smooth at 60 Hz with 120 Hz physics.
- [ ] Doubling `PIXELS_PER_METER` changes nothing but visual size.

### Phase 2 — Tire, Drivetrain, Braking, and Combined Slip *(~2 days)*

**Files:** `tire.h/.c`, `drivetrain.h/.c`

| Task | Details |
|------|---------|
| Nonlinear lateral curve | Replace the linear model with `Fy = -mu * Fz * sin(C * atan(B * alpha))`; remove the linear cornering-stiffness constants |
| Rear-wheel drive | Piecewise engine torque curve, gearing, final drive, drivetrain efficiency, locked rear axle |
| Wheel angular velocity | Per-wheel integration from drive, brake, and tire reaction torque; lockup handling |
| Slip ratio | `(omega * R - wheel_vx) / max(|wheel_vx|, SLIP_SPEED_EPSILON_MPS)`, clamped |
| Longitudinal tire curve | `Fx = mu * Fz * sin(C_long * atan(B_long * kappa))` |
| Combined-friction ellipse | Normalize, scale both components when usage exceeds 1, record `frictionUsage` |
| Brakes | `brakeBiasFront` split; brake torque opposes rotation and cannot reverse it |
| Handbrake | Rear brake torque only; the slide emerges from the friction budget |
| Tire curve debug plot | Force-versus-slip graph per axle |

**Complete when:**

- [ ] Full throttle breaks rear traction without the handbrake.
- [ ] Reducing throttle restores rear lateral authority.
- [ ] The handbrake visibly decelerates or locks the rear wheels.
- [ ] Rear `Fx` and `Fy` never exceed the friction budget in any scenario.
- [ ] Braking while cornering measurably reduces lateral force.
- [ ] Straight-line acceleration, coast-down, power oversteer, and handbrake-entry scenarios pass.

### Phase 3 — Load Transfer and Handling Validation *(~2 days)*

| Task | Details |
|------|---------|
| Longitudinal load transfer | `Fz_front`/`Fz_rear` from `l_f`, `l_r`, `h`, and filtered `ax`; clamped at `MIN_NORMAL_LOAD_N` |
| Load filtering | First-order filter on the previous step's solved `ax` |
| Separated resistance | Aerodynamic drag opposite the velocity vector, rolling resistance per wheel |
| Objective maneuver tests | Skidpad, step-steer, lift-off, drift transition scenarios with CSV baselines |
| Feel tuning | Iterate `B`, `C`, `mu` per axle, brake bias, engine curve, steering rate |
| Reference-behavior comparison | Populate the handling reference matrix (below) |

**Complete when:**

- [ ] Accelerating shifts load rearward; braking shifts it forward; static loads sum to `mass * g`.
- [ ] Changing CG position changes static axle loads as expected.
- [ ] Lifting throttle mid-corner rotates the car (lift-off oversteer) and the CSV shows the load shift causing it.
- [ ] The drift is catchable: a slide can be initiated, held on countersteer, transitioned, and recovered.
- [ ] Behavior is not binary — there is a continuous range between gripping and spinning.
- [ ] Every scenario in [Validation](#physics-validation-and-regression-testing) passes with committed baselines.
- [ ] Every item in the [Physics acceptance checklist](#physics-acceptance-checklist) is checked.

### Phase 4 — Four-Wheel Fidelity *(optional upgrade)*

This phase is an upgrade path, not a prerequisite for a playable game. The corrected
bicycle model of Phases 1–3 remains a valid shipping configuration; the four-contact-patch
model is the higher-fidelity target.

| Task | Details |
|------|---------|
| Four contact patches | Populate all four `WheelState` entries independently; forces summed per wheel with track-width yaw lever arms |
| Lateral load transfer | Distribute each axle's load left/right from lateral acceleration, CG height, and track width |
| Tire load sensitivity | Peak force grows sub-linearly with load; doubling `Fz` yields less than double peak `Fy` |
| Differential | Progression: locked rear axle → open → torque-bias/clutch LSD → configurable preload and locking under power/coast |
| Ackermann steering | Differentiate front-left and front-right road-wheel angles |
| Tire relaxation length | `dFy/dt = (Fy_steady - Fy_actual) * |wheel_vx| / relaxationLengthM`; keep configurable, since large values make keyboard control feel delayed |
| Per-surface coefficients | Per-wheel surface already queried in Phase 1; extend to `tireBScale` and loose-surface drag |

**Complete when:**

- [ ] Inside-wheel unloading and snap oversteer are observable in telemetry.
- [ ] One wheel on grass produces asymmetric yaw torque.
- [ ] Differential mode changes power-oversteer behavior measurably.
- [ ] All Phase 3 scenarios still pass, with reviewed and re-baselined CSV deltas.

### Phase 5 — Track, Surfaces, and Collision *(~1 day)*

| Task | Details |
|------|---------|
| Track geometry | Hand-authored oval, ~24 `TrackNode` entries in meters |
| Rendering | Centerline and offset boundaries via `DrawLineEx` in render pixel space |
| Surface querying | `Track_SurfaceAt()` per wheel contact point; no global grip multiplier |
| Barriers and collision | Capsule shape, swept tests at high speed, impulse response with yaw effect |
| Checkpoints and lap timing | Gate crossing advances checkpoints; wrap increments the lap and records splits |
| HUD basics | Lap count, lap timer, checkpoint count |

**Complete when:**

- [ ] A full lap can be completed on the oval.
- [ ] Straddling asphalt and grass produces asymmetric behavior, not a uniform slowdown.
- [ ] The car cannot tunnel through a barrier at top speed.
- [ ] Lap timer displays and resets on lap completion.

### Phase 6 — Scoring, Effects, and Presentation *(~2 days)*

| Task | Details |
|------|---------|
| Drift classification | Full multi-condition `scoringDrift` test, separate from `physicallySliding` |
| Scoring | Normalized angle/speed/line factors with combo multiplier |
| Persistence | Validated load/save with `FileExists`/`UnloadFileText`/`SaveFileText` result checking, in a user-writable directory |
| Smoke particles | Spawned at rear wheel contact points during a drift; expand and fade over `PARTICLE_LIFE_S`; round-robin pool of 512 |
| Spawn rate | `spawnTimer` accumulator: sparse at low drift angle, every tick at high angle |
| Camera effects | Drift zoom-*out* by decreasing `Camera2D.zoom`, smoothed on render dt |
| Car visualization | Nose triangle for heading readability; front wheels drawn steered, rear wheels aligned with heading |
| State machine polish | `STATE_MENU` → Enter to start; `STATE_PAUSED` overlay; `STATE_RESULTS` with score and lap times |
| HUD overlay | Speed in km/h, gear, RPM, score, combo, lap |

**Complete when:**

- [ ] Drifting a corner accumulates score with a visible multiplier that chains across corners.
- [ ] Creeping, reversing, spinning, and post-crash sliding do not score.
- [ ] Smoke trails render behind the car during drifts.
- [ ] Best score persists across sessions and survives a corrupted save file.
- [ ] Menu → play → results → replay loop works.
- [ ] Scoring state provably does not alter any physical force (verified by comparing a scored and an unscored replay checksum).

### Deferred beyond Phase 6

| Feature | Why deferred |
|---------|-------------|
| Multiple cars / AI opponents | Multiplies state management; lock single-car feel first |
| Skid mark decals | Needs a dedicated ground-layer draw pass |
| Tire and engine audio | Audio adds platform header complexity |
| Car texture art | `DrawTexturePro` is trivial to add; art is the blocker |
| Track file loading | Hand-authored oval is sufficient for playtesting; when added, use the same validated-load pattern as high scores |
| Split-screen / ghost replays | Entirely new game mode, though deterministic replay already exists |
| In-menu car spec selection | Only once there are ≥2 distinct feel profiles worth choosing between |
| Full Magic Formula MF6.1 | 20+ coefficients; the normalized curve plus friction ellipse is sufficient at this fidelity |

---

## Physics Acceptance Checklist

All of these must be true before advancing from Phase 3 to scoring and presentation work.

- [ ] Physics uses SI units exclusively.
- [ ] Rendering scale does not affect handling.
- [ ] Coordinate and sign conventions are documented and followed by every equation.
- [ ] Axle or wheel slip angles use contact-point velocities with distinct lever arms.
- [ ] Static normal loads sum to `mass * g`.
- [ ] Acceleration and braking shift load in the correct direction.
- [ ] Front tire force is rotated through the steering angle.
- [ ] Throttle acts through the drivetrain and driven wheels.
- [ ] Rear wheel slip ratio is calculated from wheel angular velocity.
- [ ] Combined longitudinal/lateral grip is enforced per wheel.
- [ ] The handbrake applies rear brake torque rather than scaling lateral force.
- [ ] The vehicle is stable from rest through the low-speed blend, including reverse.
- [ ] Fixed updates have a substep cap and a backlog-drop counter.
- [ ] One-shot inputs are consumed exactly once.
- [ ] Physics tests run without rendering.
- [ ] Replays are deterministic for identical inputs and build settings.
- [ ] Debug telemetry shows forces, loads, slip angles, and friction usage.
- [ ] The car can initiate, hold, transition, and recover a drift without any score-state
      logic changing physical forces.

---

## Handling Reference Targets

Art of Rally and Absolute Drift are **handling and feel references only**. Their developers
have publicly described iterative handling refinement, extensive parameter tuning, player
testing, and a shared physics foundation, but they have not published their tire,
drivetrain, or body equations. Nothing in this specification should be justified by a claim
about how those games implement their internals.

Comparison is therefore behavioral. Maintain a reference-behavior matrix and record
observable behavior for each row, using video capture and side-by-side telemetry where
possible:

| Behavior | What to observe |
|----------|-----------------|
| Steering response at low speed | Turn-in delay and yaw buildup below 10 m/s |
| Steering response at high speed | Turn-in sharpness and stability above 25 m/s |
| Throttle sensitivity during a slide | How much drift angle changes per unit throttle |
| Handbrake initiation | Time and steering input needed to break the rear loose |
| Braking initiation | Whether trail braking rotates the car |
| Countersteer speed | Steering rate required to catch a slide |
| Drift-angle stability | Whether a held angle self-sustains or requires constant correction |
| Transition behavior | Left-to-right transition sharpness and any snap |
| Spin recovery | Whether a spin is catchable and at what angle it becomes unrecoverable |
| Surface changes | Behavior stepping onto gravel or grass |
| Collision forgiveness | Speed and yaw lost in a glancing barrier hit |
| Controller vs keyboard | How much input assistance the game applies |

Discrepancies are resolved in the **control/feel layer** — input shaping, assists, camera —
before the tire model is touched.

---

## Platform / Build

### Windows (MSYS2 UCRT64) — only supported target

Drifty is developed and supported exclusively on Windows. The only supported native
toolchain is **MSYS2 UCRT64** at `C:\msys64` (override with `MSYS2_ROOT`).

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1
```

`build.bat` enters that environment and runs `build.sh`. `build.sh` refuses to run when
`MSYSTEM` is not `UCRT64`. Chocolatey GCC and a vendored `vendor/raylib` tree are not used.

Release build — one executable, no `game.dll`, `DRIFTY_HOT_RELOAD` undefined, raylib linked
statically via `libraylib.a` (the MSYS2 static archive still needs `glfw3.dll` at runtime):

```bash
./build.sh --release
```

Development build — see [Development Workflow](#development-workflow):

```bash
./build.sh
# or from cmd.exe:
build.bat
```

Bounded visual smoke test (creates the window, runs a fixed frame budget, writes
`artifacts/screenshots/phase0_smoke.png`, exits):

```bash
./build.sh --smoke-test
```

Include `raylib.h`, and `raymath.h` when `Vector2` helpers are wanted. The only build-time
define is `DRIFTY_HOT_RELOAD`, set for development builds and unset for release.

**Linking raylib for hot-reload builds:** raylib holds its state in global variables, so the
hot-reload configuration must link it as a **shared** library. A statically linked raylib
inside `game.dll` is destroyed on every reload. MSYS2 ships both a static `libraylib.a` and
the import library `libraylib.dll.a` for `libraylib.dll`; development builds link the import
library explicitly. Release builds link `libraylib.a` and must not import `libraylib.dll`.

### raylib 6.0 features used

| Feature | Usage |
|---------|-------|
| `Vector2CrossProduct()` | Determine left/right of heading for drift direction |
| `DrawLineDashed()` | Debug overlays — velocity vectors, force vectors, collision normals |
| `DrawEllipseV()` | Tire marks and skid effects |
| `FileExists()` / `LoadFileText()` / `UnloadFileText()` / `SaveFileText()` | Validated high-score persistence |
| `GetFileModTime()` | Hot-reload polling of the game module |
| `GetKeyName()` | Input rebinding UI display |
| `FLAG_SET` / `FLAG_CLEAR` / `FLAG_IS_SET` | Bitflag state management |

### raylib 6.0 API notes

| Item | Action |
|------|--------|
| `TRACELOGD()` removed | Use `TRACELOG(LOG_DEBUG, ...)` |
| `GetRandomValue()` bias fix | Expect a subtly different distribution if used for procedural generation |
| `DrawCircleGradient()` signature | Now takes `Vector2 center` instead of `(int x, int y)` |
| `config.h` is build-time only | As a consumer, do not define `RAYLIB_STANDALONE` |
| `Camera2D.zoom` | `1.0` is no scaling; **larger values zoom in**, smaller values zoom out |

### Binary size

Passing `SUPPORT_MODULE_RMODELS=0` or `SUPPORT_MODULE_RAUDIO=0` while compiling the game
has no effect: those options are resolved when raylib itself is compiled, and they do not
strip modules from the prebuilt MSYS2 package. To remove modules, build a custom static
raylib library with them disabled.

Measure the final executable size before documenting any percentage reduction. This
optimization stays deferred until executable size is a measured problem.

---

## Key Design Decisions

1. **SI units internally, pixels only at the render boundary.** A single
   `PIXELS_PER_METER` conversion. Physics constants have physical meaning and survive
   changes to art scale.
2. **One canonical structure per concept.** `VehicleSpec` (parameters), `VehicleState`
   (integrated state), `VehicleDerived` (diagnostics), `VehicleRenderState`
   (interpolation), `WheelState` (per contact patch). No parallel or reduced variants.
3. **No custom `Vec2`.** Use raylib's `Vector2` type. Physics code uses the type but calls
   no raylib functions, which is what keeps the test harness headless.
4. **No allocations during gameplay.** A single platform-allocated `Game` block, a
   round-robin particle pool, and track nodes allocated once at load. The platform layer
   owns that memory so it survives a hot reload.
5. **Drift is emergent, never a state machine.** Peak-then-falloff tire curves, per-axle
   slip angles, driven-wheel slip ratio, a shared friction budget, and load transfer
   produce drift. The handbrake is brake torque. Scoring observes physics and never
   modifies it.
6. **Correct the model before tuning it.** Tuning incorrect equations can produce a
   playable car, but the resulting constants have no physical meaning and every later
   improvement forces a full retune.
7. **Validation is infrastructure, not a final step.** The headless scenario runner and CSV
   baselines exist from Phase 0, so a broken equation is always distinguishable from an
   intentional handling change.
8. **Simulation and feel are separate layers.** Input shaping, assists, and camera live in
   the feel layer; the tire model is never bent to accommodate a keyboard.
9. **Single `config.h` for all tunables.** Every magic number lives there with its unit.
   Drift tuning is the majority of development time.
10. **Every build command terminates.** The dev loop is a thin platform layer plus a
    reloadable game module, driven by a build script that exits immediately whether or not
    the game is running. No watcher daemon, no long-lived build process — which is what
    makes the loop usable by an automated editor as well as by hand.
