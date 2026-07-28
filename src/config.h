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

/* Phase 5 collision --------------------------------------------------------- */

/* Capsule covering the car footprint: two circles at front/rear axle plus the connecting
 * body. The radius of each circle is the half-width of the car body, smaller than the track
 * width which is wheel-to-wheel center distance. */
#define VEHICLE_BODY_HALF_WIDTH_M   0.85f   /* m; collision capsule circle radius */
#define COLLISION_RESTITUTION       0.30f   /* dimensionless; barrier bounce (< 1) */
#define COLLISION_FRICTION          0.50f   /* dimensionless; Coulomb friction at impact */

/* Scoring lockout after a barrier impact (Phase 6). Timer starts counting down from this. */
#define CRASH_LOCKOUT_S             1.0f    /* seconds; post-impact scoring suspension */

/* Drift classification thresholds (Phase 6). Gameplay rules, not physical tunables:
 * these are compile-time constants that never feed back into the force model. */
#define MIN_DRIFT_SPEED_MPS         5.0f    /* m/s; below this speed a slide is not scored */
#define MIN_DRIFT_ANGLE_RAD         0.175f  /* rad (~10 deg); minimum body sideslip for scoring */
#define MIN_REAR_SLIP_RAD           0.12f   /* rad; minimum rear-axle slip for scoring */
#define MIN_DRIFT_YAW_RATE_RADS     0.25f   /* rad/s; minimum yaw rate for scoring */
#define SPIN_CUTOFF_RAD             1.48f   /* rad (~85 deg); beyond this the car has spun */

/* Score accumulation (Phase 6). Points are accrued per fixed tick while scoringDrift is true. */
#define SCORE_BASE_RATE             100.0f  /* points/second at full factors and base multiplier */
#define SCORE_SPEED_REF_MPS         35.0f   /* m/s; the speed at which speedFactor saturates */
#define COMBO_GRACE_S               1.50f   /* s; reset multiplier after this long outside a scoring drift */
#define MAX_VALID_SCORE             100000000L  /* upper bound for file-load validation and clamping */

/* Phase 6 results trigger (presentation only; no effect on simulation). */
#define RESULTS_TARGET_LAPS         3       /* laps to complete before entering STATE_RESULTS */

/* Phase 4 four-wheel fidelity ---------------------------------------------- */
#define TIRE_LOAD_SENSITIVITY_K          0.00f   /* dimensionless; realistic ~0.02. 0 disables */
#define TIRE_LOAD_REF_PER_WHEEL_N        2940.0f /* N; = VEH_MASS_KG*g/4 at default mass */
#define TIRE_RELAXATION_LENGTH_M         0.00f   /* m; realistic 0.20..0.50. 0 disables */
#define ACKERMANN_PERCENT                0.00f   /* dimensionless; 0=parallel, 1=true Ackermann */
#define DIFFERENTIAL_MODE_DEFAULT        0       /* 0=LOCKED, 1=OPEN, 2=LSD */
#define DIFFERENTIAL_BIAS_RATIO          2.0f    /* dimensionless; LSD slower/faster cap */
#define DIFFERENTIAL_PRELOAD_NM          60.0f   /* N*m; LSD clutch preload */
#define DIFF_OMEGA_EPSILON_RAD_S         1.0e-3f /* rad/s; LSD omega-difference deadband */
#define ROLL_STIFFNESS_FRONT_FRACTION    0.50f   /* dimensionless 0..1; front axle roll-moment share */
#define SURFACE_REFERENCE_MU_LAT        1.30f   /* asphalt lateral mu; tireMuLat* are absolute vs this */
#define SURFACE_REFERENCE_MU_LONG       1.35f   /* asphalt longitudinal mu; documentation reference only */
#define RELAXATION_VX_FLOOR_MPS         0.50f   /* m/s; floor for relaxation vx denominator */

/* -------------------------------------------------------------------------------------
 * Window and presentation (no effect on the simulation layer)
 * ------------------------------------------------------------------------------------- */

#define SCREEN_W                1280    /* pixels */
#define SCREEN_H                720     /* pixels */
#define TARGET_FPS              60      /* render frames per second */

#define RELOAD_FLASH_S          2.0f    /* seconds the "module reloaded" HUD notice persists */

/* Phase 6 particles --------------------------------------------------------- */

#define MAX_PARTICLES           512           /* total slots in the round-robin particle pool */
#define PARTICLE_LIFE_S         0.80f         /* seconds a smoke particle lives before fading out */

/* Phase 6 camera drift zoom ------------------------------------------------- */

#define CAMERA_BASE_ZOOM        1.20f    /* dimensionless; >1 magnifies (zooms in) */
#define CAMERA_ZOOM_RANGE       0.25f    /* dimensionless; subtracted at full drift -> 0.95 */
#define CAMERA_MIN_ZOOM         0.50f    /* dimensionless; must stay > 0 */
#define CAMERA_ZOOM_RATE        4.0f     /* 1/second smoothing rate */
#define CAMERA_LOOKAHEAD        0.25f    /* seconds of velocity lookahead (reserved) */
#define DRIFT_ZOOM_REF_RAD      0.70f    /* sideslip mapped to full zoom-out */
#define SLIDE_USAGE_THRESHOLD   0.98f    /* dimensionless friction usage = physically sliding */

#endif /* DRIFTY_CONFIG_H */
