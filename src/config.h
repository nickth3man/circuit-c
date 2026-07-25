/*
 * config.h — Phase 0 tunables and numerical infrastructure constants.
 *
 * Rules enforced by this file (docs/SPEC.md, "Units, Coordinate System, and Conventions"):
 *
 *   - All physical values are SI. Every unit-bearing constant states its unit in its name
 *     or in a trailing comment. Constants with no unit are dimensionless ratios and say so.
 *   - PIXELS_PER_METER is a render scale. It is consumed only by units.h helpers and by
 *     rendering code. No simulation quantity may be derived from it. This is a regression
 *     test (tests/physics_tests.c, scenario "renderscale").
 *
 * Phase 0 deliberately does NOT define the vehicle, tire, drivetrain, brake, scoring, or
 * particle tunables listed in docs/SPEC.md "Baseline Vehicle Parameters". Those arrive with
 * the code that consumes them, starting in Phase 1.
 */
#ifndef DRIFTY_CONFIG_H
#define DRIFTY_CONFIG_H

/* -------------------------------------------------------------------------------------
 * Render scale (rendering only — never read by simulation code)
 * ------------------------------------------------------------------------------------- */

/* Dimensionless render scale: how many screen pixels one world meter occupies.
 * Overridable at build time (-DPIXELS_PER_METER=48.0f) purely to prove that changing it
 * alters nothing but visual size. It is the default for Game.renderPixelsPerMeter. */
#ifndef PIXELS_PER_METER
#define PIXELS_PER_METER 24.0f
#endif

/* -------------------------------------------------------------------------------------
 * Fixed timestep
 * ------------------------------------------------------------------------------------- */

#define FIXED_HZ                120                  /* fixed updates per second */
#define FIXED_DT_S              (1.0f / 120.0f)      /* seconds per fixed update */
#define MAX_PHYSICS_STEPS       8                    /* substep cap per render frame */
#define MAX_FRAME_TIME_S        0.25f                /* seconds; frame-time clamp */

/* -------------------------------------------------------------------------------------
 * Deterministic input recording
 * ------------------------------------------------------------------------------------- */

/* Fixed-capacity ring buffer, sized in fixed ticks. 7200 ticks = 60.0 s at 120 Hz.
 * Overflow behaviour is a documented ring: the oldest tick is discarded and counted in
 * ReplayBuffer.overwrittenTicks. See src/replay.h. */
#define REPLAY_CAPACITY_TICKS   7200

/* -------------------------------------------------------------------------------------
 * Phase 0 placeholder marker transform
 *
 * These drive the deterministic non-vehicle test transform that game.c integrates until
 * physics.c takes ownership of the fixed update order in Phase 1. They are NOT a vehicle
 * model and carry no physical meaning beyond their stated units.
 * ------------------------------------------------------------------------------------- */

#define MARKER_TURN_RATE_RAD_S  2.0f    /* radians/second of marker heading per unit steer */
#define MARKER_SPEED_MPS        6.0f    /* meters/second of marker travel at full throttle */
#define MARKER_HANDBRAKE_SCALE  0.25f   /* dimensionless speed scale at full handbrake */

/* -------------------------------------------------------------------------------------
 * Window and presentation (no effect on the simulation layer)
 * ------------------------------------------------------------------------------------- */

#define SCREEN_W                1280    /* pixels */
#define SCREEN_H                720     /* pixels */
#define TARGET_FPS              60      /* render frames per second */

#define RELOAD_FLASH_S          2.0f    /* seconds the "module reloaded" HUD notice persists */

#endif /* DRIFTY_CONFIG_H */
