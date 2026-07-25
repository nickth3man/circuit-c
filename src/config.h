/*
 * config.h — simulation, numerical infrastructure, and presentation constants.
 *
 * Rules enforced by this file (docs/SPEC.md, "Units, Coordinate System, and Conventions"):
 *
 *   - All physical values are SI. Every unit-bearing constant states its unit in its name
 *     or in a trailing comment. Constants with no unit are dimensionless ratios and say so.
 *   - PIXELS_PER_METER is a render scale. It is consumed only by units.h helpers and by
 *     rendering code. No simulation quantity may be derived from it. This is a regression
 *     test (tests/physics_tests.c, scenario "renderscale").
 *
 * Phase 1 adds only the rigid-body vehicle and temporary linear-tire/longitudinal constants.
 * Nonlinear tires, drivetrain, wheel rotation, physical brakes, and load transfer remain
 * intentionally absent until their owning phases.
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
 * Phase 1 rigid-body vehicle (SI units)
 * ------------------------------------------------------------------------------------- */

#define GRAVITY_MPS2            9.80665f

#define VEH_MASS_KG             1200.0f
#define VEH_YAW_INERTIA_KGM2    1800.0f
#define VEH_CG_TO_FRONT_M       1.15f
#define VEH_CG_TO_REAR_M        1.40f
#define VEH_CG_HEIGHT_M         0.50f
#define VEH_TRACK_FRONT_M       1.48f
#define VEH_TRACK_REAR_M        1.46f

#define WHEEL_RADIUS_M          0.31f
#define WHEEL_INERTIA_KGM2      1.20f

#define STEER_MAX_RAD           0.70f
#define STEER_RATE_RAD_S        5.0f
#define STEER_RETURN_RATE_RAD_S 7.0f

#define DRAG_COEFFICIENT        0.32f
#define FRONTAL_AREA_M2         1.90f
#define ROLLING_RESISTANCE_COEF 0.015f

/* Canonical later-phase fields are initialized with the specification's neutral baseline,
 * but Phase 1 does not evaluate either nonlinear tire curve. */
#define TIRE_B_LAT_FRONT        10.0f
#define TIRE_C_LAT_FRONT        1.45f
#define TIRE_MU_LAT_FRONT       1.30f
#define TIRE_B_LAT_REAR         10.0f
#define TIRE_C_LAT_REAR         1.45f
#define TIRE_MU_LAT_REAR        1.20f
#define TIRE_B_LONG             12.0f
#define TIRE_C_LONG             1.55f
#define TIRE_MU_LONG_SCALE      1.00f

#define MAX_GEARS               8
#define ENGINE_CURVE_POINTS     7

/* Temporary Phase 1 linear lateral model. Removed when Phase 2 supplies nonlinear tires. */
#define TIRE_CORNERING_STIFFNESS_FRONT_N_PER_RAD 80000.0f
#define TIRE_CORNERING_STIFFNESS_REAR_N_PER_RAD  85000.0f
#define TIRE_MU_LINEAR_FRONT    1.30f
#define TIRE_MU_LINEAR_REAR     1.20f

/* Temporary body-level longitudinal command. This is explicitly not a drivetrain or a
 * longitudinal tire model. selectedGear chooses forward (+1) or reverse (-1) direction. */
#define PHASE1_MAX_DRIVE_FORCE_N         4500.0f
#define PHASE1_MAX_REVERSE_FORCE_N       3000.0f
#define PHASE1_MAX_BRAKE_FORCE_N         8000.0f
#define PHASE1_LINEAR_DRAG_N_PER_MPS      120.0f
#define PHASE1_KINEMATIC_RESPONSE_RATE_HZ  10.0f

#define LOW_SPEED_EPSILON_MPS   0.50f
#define LOW_SPEED_BEGIN_MPS     1.50f
#define LOW_SPEED_END_MPS       3.00f
#define MIN_NORMAL_LOAD_N       50.0f
#define MAX_SAFE_SPEED_MPS      120.0f
#define MAX_SAFE_YAW_RATE_RADS  20.0f

/* -------------------------------------------------------------------------------------
 * Window and presentation (no effect on the simulation layer)
 * ------------------------------------------------------------------------------------- */

#define SCREEN_W                1280    /* pixels */
#define SCREEN_H                720     /* pixels */
#define TARGET_FPS              60      /* render frames per second */

#define RELOAD_FLASH_S          2.0f    /* seconds the "module reloaded" HUD notice persists */

#endif /* DRIFTY_CONFIG_H */
