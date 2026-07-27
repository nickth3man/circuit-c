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
 * Phase 2 added nonlinear tires, rear-wheel drive, wheel rotation, physical brakes, and
 * combined slip. Phase 3 adds longitudinal load transfer from a filtered previous-step
 * acceleration, and uses separated aerodynamic drag and per-wheel rolling resistance.
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
 * Phase 1/2 vehicle physics (SI units)
 * ------------------------------------------------------------------------------------- */

#define GRAVITY_MPS2            9.80665f
#define AIR_DENSITY_KGM3        1.225f    /* kg/m^3, sea-level standard atmosphere */

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

/* First-order corner frequency of the load-transfer acceleration filter, in hertz. It
 * models the suspension's finite pitch response: a step in longitudinal force does not
 * move the axle loads instantaneously. Registered as a tunable. */
#define LOAD_FILTER_RATE_HZ     20.0f

/* Normalized nonlinear tire curves. */
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

#define GEAR_RATIOS             { 3.55f, 2.05f, 1.38f, 1.00f, 0.82f }
#define GEAR_COUNT              5
#define REVERSE_GEAR_RATIO      3.20f
#define FINAL_DRIVE_RATIO       4.10f
#define DRIVETRAIN_EFFICIENCY   0.90f
#define ENGINE_IDLE_RPM         900.0f
#define ENGINE_REDLINE_RPM      7000.0f
#define ENGINE_BRAKING_TORQUE_NM 35.0f
#define ENGINE_TORQUE_CURVE_NM  \
    { 140.0f, 200.0f, 240.0f, 255.0f, 250.0f, 230.0f, 195.0f }

#define MAX_BRAKE_TORQUE_NM     3000.0f
#define BRAKE_BIAS_FRONT        0.62f
#define HANDBRAKE_TORQUE_NM     1800.0f

/* Rate at which the kinematic low-speed model pulls lateral velocity and yaw rate onto
 * their geometric targets, in hertz. Not a resistance term and not a handling tunable: it
 * is the stiffness of the model blend that keeps the vehicle stable below LOW_SPEED_BEGIN,
 * and changing it interactively would change what "kinematic" means mid-run. */
#define LOW_SPEED_RESPONSE_RATE_HZ 10.0f

#define LOW_SPEED_EPSILON_MPS   0.50f
#define SLIP_SPEED_EPSILON_MPS  1.00f
#define SLIP_RATIO_CLAMP        4.00f
#define LOW_SPEED_BEGIN_MPS     1.50f
#define LOW_SPEED_END_MPS       3.00f
#define MIN_NORMAL_LOAD_N       50.0f
#define FRICTION_TOLERANCE      0.001f
#define MAX_SAFE_SPEED_MPS      120.0f
#define MAX_SAFE_YAW_RATE_RADS  20.0f

/* Phase 3 numerical guards. These bound the resistance model rather than shaping it, so
 * they stay compile-time constants: a slider that can make drag point along +v, or let a
 * resistance force reverse the car inside one tick, does not describe a different car —
 * it describes a broken integrator.
 *
 *   RESISTANCE_EPSILON_MPS       direction denominator floor; below it there is no
 *                                well-defined direction to oppose, so the force is zero.
 *   MAX_LOAD_TRANSFER_FRACTION   safety tripwire, as a multiple of mass * g. Physical
 *                                transfer peaks near 0.3 of that; 1.5 means something is
 *                                badly wrong, not merely aggressive.
 *   RESISTANCE_POWER_TOLERANCE_W slack for the "resistance never adds energy" assertion,
 *                                in watts, absorbing float rounding in the dot product.
 */
#define RESISTANCE_EPSILON_MPS       1.0e-4f
#define MAX_LOAD_TRANSFER_FRACTION   1.50f
#define RESISTANCE_POWER_TOLERANCE_W 1.0e-2f

/* -------------------------------------------------------------------------------------
 * Window and presentation (no effect on the simulation layer)
 * ------------------------------------------------------------------------------------- */

#define SCREEN_W                1280    /* pixels */
#define SCREEN_H                720     /* pixels */
#define TARGET_FPS              60      /* render frames per second */

#define RELOAD_FLASH_S          2.0f    /* seconds the "module reloaded" HUD notice persists */

#endif /* DRIFTY_CONFIG_H */
