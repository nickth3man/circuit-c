/*
 * dev_presets.c — the ten hardcoded driving-profile presets.
 *
 * Each preset is a named bundle of parameter overrides sourced from a research lane:
 *
 *   0  Stock Baseline  — the config.h defaults (no overrides)
 *   1  Touge Hero      — 950 kg AE86 grip-drift (JDM touge)
 *   2  Pro D1GP        — 1320 kg Formula Drift V8 competition car (pro drift + sim drift)
 *   3  Arcade God      — exaggerated NFS / Forza / Burnout arcade drift (arcade drift)
 *   4  Track Predator  — 1220 kg GT3 maximum-grip circuit car (grip racing)
 *   5  Rally Devil     — 1000 kg Group B gravel monster (rally / loose surface)
 *   6  Pony Car        — 1740 kg American muscle (car archetypes)
 *   7  911 Pendulum    — 1520 kg rear-engine Porsche (car archetypes)
 *   8  Shoebox         — 760 kg kei-car momentum machine (car archetypes)
 *   9  Time Machine    — 540 kg 1988 Formula 1 turbo (car archetypes)
 *
 * Application is always: reset to stock defaults, then write the overrides. So every
 * preset is independent of whatever was tuned before — switching to "Touge Hero"
 * always yields the same car.
 *
 * Values are kept inside the registry's declared [minimum, maximum] for every
 * parameter; dev_param_set clamps anyway, but the table is deliberately clean.
 *
 * Raylib-free: linked into the headless test executable.
 */
#include "dev/dev_presets.h"

#include <stdlib.h>
#include <string.h>

#include "dev/dev_params.h"

/* ------------------------------------------------------------------------------ data -- */

/* One (parameter key, value) pair. The key is resolved through dev_param_find at
 * apply time, so the table never holds raw byte offsets that could drift from the
 * registry. Unknown keys are counted and skipped, matching the profile loader. */
typedef struct {
    const char *key;
    float       value;
} PresetOverride;

/* Shorthand to keep the tables readable. */
#define O(key, val) { key, (float)(val) }

/* ------------------------------------------------------------------ 0: Stock Baseline --
 * No overrides. dev_preset_apply() calls dev_params_reset_all() unconditionally, so
 * index 0 with an empty table is exactly "back to stock". */

/* --------------------------------------------------------------------- 1: Touge Hero --
 * AE86 Trueno: 950 kg, 50/50 weight, narrow 195-section tires, 8000 RPM NA redline,
 * 2-way LSD, 4.778 final drive. "Grip drift" — progressive throttle steer, not smoke.
 * Source: lib-2 (JDM touge), Drifted.com Initial D guide, Toyota AE86 Wikipedia. */
static const PresetOverride kTougeHero[] = {
    O("body.mass",                950.0f),
    O("body.yaw_inertia",        1100.0f),
    O("body.cg_to_front",          1.10f),
    O("body.cg_to_rear",           1.30f),   /* wheelbase 2.40 m, ~54F/46R */
    O("body.cg_height",            0.42f),
    O("body.track_front",          1.35f),
    O("body.track_rear",           1.34f),
    O("body.frontal_area",         1.75f),
    O("wheel.radius",              0.29f),   /* 195/50R15 */
    O("wheel.inertia",             0.80f),
    O("steer.max_angle",           0.66f),   /* ~38 deg */
    O("steer.ackermann_percent",   0.70f),   /* tight hairpins */
    O("tire.lat_front.mu",         1.10f),
    O("tire.lat_rear.b",           9.0f),
    O("tire.lat_rear.mu",          1.00f),
    O("tire.relaxation_length",    0.20f),
    O("tire.load_sensitivity_k",   0.008f),
    O("tire.load_ref_per_wheel", 2330.0f),   /* 950*9.81/4 */
    O("engine.redline_rpm",     8000.0f),
    O("engine.torque_p0",         80.0f),    /* high-revving NA, peak late */
    O("engine.torque_p1",        100.0f),
    O("engine.torque_p2",        115.0f),
    O("engine.torque_p3",        130.0f),    /* ~peak at 6000 rpm */
    O("engine.torque_p4",        130.0f),
    O("engine.torque_p5",        125.0f),
    O("engine.torque_p6",        115.0f),
    O("drive.final",               4.78f),   /* TRD 2-way final drive */
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     2.5f),
    O("drive.diff_preload",       80.0f),
    O("brake.max_torque",       2200.0f),
    O("brake.bias_front",          0.53f),
    O("collision.half_width",      0.81f),   /* AE86 is 1625 mm wide */
};

/* ----------------------------------------------------------------------- 2: Pro D1GP --
 * Formula Drift competition car: 1320 kg, locked diff, 65 deg steering lock, flat
 * 550 N*m V8 torque from 2500-7500 rpm, rear tire mu ~30% below front.
 * Source: lib-1 (pro drift), Wisefab, Suspension Secrets, DrivingLine. */
static const PresetOverride kProD1GP[] = {
    O("body.mass",               1320.0f),
    O("body.yaw_inertia",        2200.0f),   /* longer wheelbase, resists snap-spins */
    O("body.cg_to_front",          1.17f),
    O("body.cg_to_rear",           1.43f),   /* wheelbase 2.60 m, ~55F/45R */
    O("body.cg_height",            0.45f),
    O("body.track_front",          1.55f),   /* wide, angle-kit track */
    O("body.track_rear",           1.54f),
    O("body.roll_stiffness_front", 0.65f),
    O("wheel.inertia",             1.50f),   /* big rotors, wide tires */
    O("steer.max_angle",           1.13f),   /* 65 deg — the Wisefab number */
    O("steer.rate",                7.0f),
    O("steer.return_rate",         9.0f),
    O("steer.ackermann_percent",   0.25f),   /* mild positive */
    O("tire.lat_front.b",         11.0f),
    O("tire.lat_front.mu",         1.15f),
    O("tire.lat_rear.b",           8.0f),    /* broad-peak rear */
    O("tire.lat_rear.mu",          0.80f),   /* 30% stagger — the FD tire split */
    O("tire.load_sensitivity_k",   0.012f),
    O("tire.load_ref_per_wheel", 3237.0f),   /* 1320*9.81/4 */
    O("engine.redline_rpm",     8500.0f),
    O("engine.torque_p0",        300.0f),    /* fat flat V8 torque band */
    O("engine.torque_p1",        450.0f),
    O("engine.torque_p2",        530.0f),
    O("engine.torque_p3",        550.0f),    /* peak ~4500 rpm */
    O("engine.torque_p4",        540.0f),
    O("engine.torque_p5",        510.0f),
    O("engine.torque_p6",        450.0f),
    O("drive.final",               4.20f),
    O("drive.diff_mode",           0.0f),    /* LOCKED — the archetypal drift diff */
    O("brake.max_torque",       3500.0f),
    O("brake.bias_front",          0.58f),
    O("brake.handbrake_torque", 2400.0f),
    O("collision.half_width",      0.90f),
};

/* --------------------------------------------------------------------- 3: Arcade God --
 * Exaggerated arcade drift: locked diff, rear mu 0.70 vs front 1.10, 3500 N*m
 * handbrake, 55 deg steer, very low wheel inertia. Every input breaks traction.
 * Source: lib-4 (arcade drift), Forza Horizon drift guides, NFS ProStreet, Burnout. */
static const PresetOverride kArcadeGod[] = {
    O("body.mass",               1150.0f),
    O("body.yaw_inertia",        1400.0f),
    O("body.cg_to_front",          1.25f),
    O("body.cg_to_rear",           1.25f),   /* 50/50 */
    O("body.cg_height",            0.45f),
    O("body.roll_stiffness_front", 0.60f),
    O("wheel.inertia",             0.60f),   /* instant spin-up */
    O("steer.max_angle",           0.96f),   /* ~55 deg */
    O("steer.rate",                9.0f),
    O("steer.return_rate",         5.0f),    /* slow — hold the angle */
    O("steer.ackermann_percent",   0.25f),
    O("tire.lat_front.mu",         1.10f),
    O("tire.lat_rear.b",           7.0f),    /* very broad plateau */
    O("tire.lat_rear.mu",          0.70f),   /* 40% stagger — the arcade cheat */
    O("tire.load_sensitivity_k",   0.003f),  /* almost none — easy slides */
    O("tire.relaxation_length",    0.10f),   /* instant slip response */
    O("tire.load_ref_per_wheel", 2820.0f),
    O("engine.redline_rpm",     7500.0f),
    O("engine.torque_p0",        200.0f),    /* wide plateau 3000-7000 */
    O("engine.torque_p1",        280.0f),
    O("engine.torque_p2",        330.0f),
    O("engine.torque_p3",        350.0f),
    O("engine.torque_p4",        345.0f),
    O("engine.torque_p5",        330.0f),
    O("engine.torque_p6",        300.0f),
    O("engine.braking_torque",    60.0f),    /* strong lift-off initiation */
    O("drive.final",               4.50f),   /* short for wheelspin */
    O("drive.diff_mode",           0.0f),    /* LOCKED */
    O("brake.max_torque",       2500.0f),
    O("brake.bias_front",          0.45f),   /* rear locks first */
    O("brake.handbrake_torque", 3500.0f),    /* the "drift button" */
    O("collision.restitution",     0.15f),   /* soft wall taps */
};

/* ----------------------------------------------------------------- 4: Track Predator --
 * GT3 circuit grip: LSD with 3.0 bias, parallel steering, mu 1.55 balanced front/rear,
 * very low CG, near-zero load sensitivity. Maximum lap time — fights you if you slide.
 * Source: lib-5 (grip racing), iRacing, Forza Motorsport, OptimumG, Racecar Engineering. */
static const PresetOverride kTrackPredator[] = {
    O("body.mass",               1220.0f),
    O("body.yaw_inertia",        1500.0f),
    O("body.cg_to_front",          1.20f),
    O("body.cg_to_rear",           1.35f),
    O("body.cg_height",            0.28f),   /* very low — minimize load transfer */
    O("body.track_front",          1.55f),
    O("body.track_rear",           1.55f),
    O("body.drag_coefficient",     0.28f),
    O("body.frontal_area",         1.80f),
    O("body.load_filter_rate",    30.0f),    /* fast load-transfer response */
    O("body.roll_stiffness_front", 0.52f),
    O("wheel.inertia",             0.80f),
    O("steer.max_angle",           0.45f),   /* ~26 deg — grip racing never needs more */
    O("steer.rate",                8.0f),
    O("steer.ackermann_percent",   0.00f),   /* parallel / anti-Ackermann */
    O("tire.lat_front.b",         14.0f),    /* high stiffness — sharp peak */
    O("tire.lat_front.mu",         1.55f),
    O("tire.lat_rear.b",          14.0f),
    O("tire.lat_rear.mu",          1.50f),   /* nearly balanced */
    O("tire.load_sensitivity_k",   0.001f),  /* near-zero — qualifying compound */
    O("tire.relaxation_length",    0.15f),
    O("tire.load_ref_per_wheel", 2990.0f),
    O("engine.redline_rpm",     7800.0f),
    O("engine.torque_p0",        180.0f),
    O("engine.torque_p1",        240.0f),
    O("engine.torque_p2",        285.0f),
    O("engine.torque_p3",        300.0f),
    O("engine.torque_p4",        295.0f),
    O("engine.torque_p5",        280.0f),
    O("engine.torque_p6",        250.0f),
    O("drive.diff_mode",           2.0f),    /* LSD — never locked for grip racing */
    O("drive.diff_bias_ratio",     3.0f),
    O("drive.diff_preload",      100.0f),
    O("brake.max_torque",       5000.0f),    /* strong deceleration */
    O("brake.bias_front",          0.57f),
    O("brake.handbrake_torque",  800.0f),    /* grip racing rarely uses it */
};

/* ------------------------------------------------------------------- 5: Rally Devil --
 * Group B gravel: 1000 kg, 55 deg steer for hairpin countersteer, gravel mu ~0.58,
 * low B (broad forgiving peak), long relaxation, LSD with 70% rear bias and 200 N*m
 * preload. Turbo-lag-then-surge torque curve.
 * Source: lib-6 (rally), DirtFish, DiRT Rally tuning guide, Lancia Delta S4. */
static const PresetOverride kRallyDevil[] = {
    O("body.mass",               1000.0f),
    O("body.yaw_inertia",        1200.0f),
    O("body.cg_to_front",          1.44f),
    O("body.cg_to_rear",           0.96f),   /* mid-engine rear bias, ~40F/60R */
    O("body.cg_height",            0.50f),
    O("body.track_front",          1.50f),
    O("body.track_rear",           1.50f),
    O("body.drag_coefficient",     0.40f),   /* boxy aero + big wing */
    O("body.frontal_area",         2.10f),
    O("body.load_filter_rate",    10.0f),    /* soft suspension, slow transfer */
    O("body.roll_stiffness_front", 0.35f),   /* rear-biased ARB for rotation */
    O("wheel.radius",              0.27f),   /* 15" gravel */
    O("wheel.inertia",             0.80f),
    O("steer.max_angle",           0.96f),   /* ~55 deg — hairpin countersteer */
    O("steer.rate",                7.0f),
    O("steer.return_rate",         4.0f),    /* driver-controlled */
    O("steer.ackermann_percent",   0.30f),
    O("tire.lat_front.b",          8.0f),    /* low B — broad forgiving peak */
    O("tire.lat_front.mu",         0.58f),   /* gravel grip */
    O("tire.lat_rear.b",           8.0f),
    O("tire.lat_rear.mu",          0.55f),
    O("tire.long.b",               8.0f),
    O("tire.load_sensitivity_k",   0.025f),  /* pronounced — gravel digs then saturates */
    O("tire.relaxation_length",    0.30f),   /* long — smooth on loose surface */
    O("tire.load_ref_per_wheel", 2453.0f),
    O("engine.redline_rpm",     8000.0f),
    O("engine.torque_p0",        100.0f),    /* turbo lag */
    O("engine.torque_p1",        180.0f),
    O("engine.torque_p2",        300.0f),
    O("engine.torque_p3",        420.0f),    /* boost threshold */
    O("engine.torque_p4",        440.0f),    /* surge */
    O("engine.torque_p5",        430.0f),
    O("engine.torque_p6",        400.0f),
    O("engine.braking_torque",    60.0f),
    O("drive.final",               4.50f),
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     4.0f),    /* high rear torque bias */
    O("drive.diff_preload",      200.0f),    /* keeps rear tied on loose surface */
    O("brake.max_torque",       2000.0f),    /* moderate — too strong locks on gravel */
    O("brake.bias_front",          0.55f),
    O("brake.handbrake_torque", 2800.0f),    /* essential rally tool */
    O("collision.half_width",      0.94f),
    O("collision.restitution",     0.10f),
};

/* --------------------------------------------------------------------- 6: Pony Car --
 * Ford Mustang GT: 1740 kg, 54F/46R, big 450 N*m V8 at 4900 rpm, LSD 2.5 bias, 3.55
 * final drive. Heavy nose wants to plow, big torque wants to spin — throttle-steerable.
 * Source: lib-7 (car archetypes), MustangSpecs. */
static const PresetOverride kPonyCar[] = {
    O("body.mass",               1740.0f),
    O("body.yaw_inertia",        2600.0f),   /* heavy, stable */
    O("body.cg_to_front",          1.25f),
    O("body.cg_to_rear",           1.47f),   /* wheelbase 2.72 m, ~54F/46R */
    O("body.cg_height",            0.52f),
    O("body.track_front",          1.60f),
    O("body.track_rear",           1.62f),
    O("body.drag_coefficient",     0.35f),
    O("body.frontal_area",         2.30f),
    O("body.roll_stiffness_front", 0.55f),
    O("wheel.inertia",             1.80f),   /* big rotors, wide tires */
    O("steer.max_angle",           0.66f),   /* ~38 deg */
    O("steer.ackermann_percent",   0.65f),
    O("tire.lat_front.mu",         1.20f),
    O("tire.lat_rear.mu",          1.05f),
    O("tire.load_sensitivity_k",   0.012f),
    O("tire.load_ref_per_wheel", 4267.0f),   /* 1740*9.81/4 */
    O("engine.redline_rpm",     7500.0f),
    O("engine.torque_p0",        250.0f),
    O("engine.torque_p1",        350.0f),
    O("engine.torque_p2",        420.0f),
    O("engine.torque_p3",        450.0f),    /* peak ~4900 rpm */
    O("engine.torque_p4",        445.0f),
    O("engine.torque_p5",        420.0f),
    O("engine.torque_p6",        380.0f),
    O("drive.final",               3.55f),   /* long — GT cruiser gearing */
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     2.5f),
    O("drive.diff_preload",      100.0f),
    O("brake.max_torque",       3800.0f),
    O("brake.bias_front",          0.62f),
    O("collision.half_width",      0.95f),
};

/* ------------------------------------------------------------------ 7: 911 Pendulum --
 * Porsche 911 Carrera: 1520 kg, 38F/62R rear weight bias, grippy rear tires (mu 1.30),
 * moderate front grip. Trail-brake to rotate, power to catch. Lift-off oversteer is
 * the signature move.
 * Source: lib-7 (car archetypes), CarreraFever, FastestLaps. */
static const PresetOverride k911Pendulum[] = {
    O("body.mass",               1520.0f),
    O("body.yaw_inertia",        2000.0f),
    O("body.cg_to_front",          1.52f),
    O("body.cg_to_rear",           0.93f),   /* rear-engine, ~38F/62R */
    O("body.cg_height",            0.40f),   /* low */
    O("body.track_front",          1.50f),
    O("body.track_rear",           1.54f),   /* wider rear */
    O("body.roll_stiffness_front", 0.45f),
    O("wheel.inertia",             1.20f),
    O("steer.max_angle",           0.61f),   /* ~35 deg */
    O("steer.rate",                6.0f),
    O("steer.return_rate",         8.0f),
    O("steer.ackermann_percent",   0.50f),
    O("tire.lat_front.mu",         1.05f),
    O("tire.lat_rear.b",          11.0f),
    O("tire.lat_rear.mu",          1.30f),   /* rear-biased grip — engine over the axle */
    O("tire.load_sensitivity_k",   0.010f),
    O("tire.load_ref_per_wheel", 3728.0f),
    O("engine.redline_rpm",     7500.0f),
    O("engine.torque_p0",        280.0f),    /* flat torque 2000-5000 */
    O("engine.torque_p1",        350.0f),
    O("engine.torque_p2",        380.0f),
    O("engine.torque_p3",        385.0f),
    O("engine.torque_p4",        380.0f),
    O("engine.torque_p5",        360.0f),
    O("engine.torque_p6",        330.0f),
    O("drive.final",               3.80f),
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     2.5f),
    O("drive.diff_preload",      100.0f),
    O("brake.max_torque",       3200.0f),
    O("brake.bias_front",          0.52f),   /* near-neutral — trail-brake rotation */
    O("collision.half_width",      0.88f),
};

/* ----------------------------------------------------------------------- 8: Shoebox --
 * Honda Beat: 760 kg, mid-engine, 8500 RPM, 65 N*m peak torque. You brake for nothing.
 * No power to waste on wheelspin — every slide must come from weight transfer and
 * momentum. Miss a corner entry and you're a passenger.
 * Source: lib-7 (car archetypes), Wikipedia Honda Beat. */
static const PresetOverride kShoebox[] = {
    O("body.mass",                760.0f),
    O("body.yaw_inertia",         700.0f),   /* extremely low — knife-edge agile */
    O("body.cg_to_front",          1.21f),
    O("body.cg_to_rear",           0.99f),   /* mid-engine, ~45F/55R */
    O("body.cg_height",            0.45f),
    O("body.track_front",          1.30f),   /* very narrow */
    O("body.track_rear",           1.30f),
    O("body.drag_coefficient",     0.36f),   /* boxy kei shape */
    O("body.frontal_area",         1.60f),
    O("body.rolling_resistance",   0.018f),
    O("wheel.radius",              0.25f),   /* 13" kei wheels */
    O("wheel.inertia",             0.50f),
    O("steer.max_angle",           0.73f),   /* ~42 deg */
    O("steer.rate",                8.0f),
    O("steer.return_rate",         9.0f),
    O("steer.ackermann_percent",   0.70f),
    O("tire.lat_front.mu",         0.95f),
    O("tire.lat_rear.mu",          0.90f),
    O("tire.load_sensitivity_k",   0.005f),  /* tiny loads */
    O("tire.load_ref_per_wheel", 1864.0f),
    O("engine.redline_rpm",     8500.0f),    /* motorcycle-style screamer */
    O("engine.torque_p0",         35.0f),
    O("engine.torque_p1",         45.0f),
    O("engine.torque_p2",         55.0f),
    O("engine.torque_p3",         65.0f),    /* peak ~6000 rpm */
    O("engine.torque_p4",         64.0f),
    O("engine.torque_p5",         60.0f),
    O("engine.torque_p6",         55.0f),
    O("drive.gear1",               3.30f),   /* close-ratio short box */
    O("drive.gear2",               2.10f),
    O("drive.gear3",               1.60f),
    O("drive.gear4",               1.20f),
    O("drive.gear5",               0.95f),
    O("drive.final",               4.90f),   /* very short — compensate low torque */
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     2.0f),
    O("drive.diff_preload",       40.0f),
    O("brake.max_torque",       1400.0f),
    O("brake.bias_front",          0.55f),
    O("collision.half_width",      0.66f),   /* kei width ~1.33 m */
};

/* ------------------------------------------------------------------ 9: Time Machine --
 * McLaren MP4/4 spirit: 540 kg, 10000 RPM (capped from era 12500), 600 N*m turbo spike
 * at the top of the band, qualifying-tire grip everywhere, parallel steering, no
 * handbrake. Nothing below 7000 rpm, then the turbo detonates.
 * Source: lib-7 (car archetypes), Wikipedia McLaren MP4/4, Honda Global. */
static const PresetOverride kTimeMachine[] = {
    O("body.mass",                540.0f),
    O("body.yaw_inertia",         450.0f),   /* extreme — instant rotation */
    O("body.cg_to_front",          1.62f),
    O("body.cg_to_rear",           1.18f),   /* mid-engine, ~42F/58R */
    O("body.cg_height",            0.25f),   /* ground-effect era, minimal */
    O("body.track_front",          1.80f),   /* wide F1 stance */
    O("body.track_rear",           1.80f),
    O("body.drag_coefficient",     0.25f),
    O("body.frontal_area",         1.50f),
    O("body.load_filter_rate",    40.0f),    /* very stiff, fast transfer */
    O("wheel.radius",              0.33f),
    O("wheel.inertia",             0.60f),
    O("steer.max_angle",           0.38f),   /* ~22 deg — F1 steering */
    O("steer.rate",               12.0f),    /* instant */
    O("steer.return_rate",        12.0f),
    O("steer.ackermann_percent",   0.00f),   /* parallel */
    O("tire.lat_front.b",         18.0f),    /* very high — sharp spike peak */
    O("tire.lat_front.mu",         1.50f),   /* qualifying compound */
    O("tire.lat_rear.b",          18.0f),
    O("tire.lat_rear.mu",          1.45f),
    O("tire.load_sensitivity_k",   0.001f),  /* near-zero — 80s qualifying tires */
    O("tire.relaxation_length",    0.08f),   /* ultra-short response */
    O("tire.load_ref_per_wheel", 1324.0f),
    O("engine.idle_rpm",        2500.0f),    /* high — turbo era */
    O("engine.redline_rpm",    10000.0f),    /* registry max; era cars ran 12500 */
    O("engine.torque_p0",        100.0f),    /* off-boost lag */
    O("engine.torque_p1",        150.0f),
    O("engine.torque_p2",        300.0f),
    O("engine.torque_p3",        480.0f),    /* boost coming in */
    O("engine.torque_p4",        600.0f),    /* spike — qualifying boost */
    O("engine.torque_p5",        590.0f),
    O("engine.torque_p6",        550.0f),
    O("engine.braking_torque",    20.0f),    /* low — no engine braking wanted */
    O("drive.gear1",               3.00f),
    O("drive.gear2",               2.20f),
    O("drive.gear3",               1.70f),
    O("drive.gear4",               1.30f),
    O("drive.gear5",               1.00f),
    O("drive.final",               3.50f),   /* tall — already huge torque */
    O("drive.diff_mode",           2.0f),    /* LSD */
    O("drive.diff_bias_ratio",     3.5f),
    O("drive.diff_preload",      120.0f),
    O("brake.max_torque",       6000.0f),    /* enormous carbon brakes */
    O("brake.bias_front",          0.55f),
    O("brake.handbrake_torque",    0.0f),    /* F1 cars have no handbrake */
    O("collision.half_width",      0.90f),
};

/* --------------------------------------------------------------------------- table -- */

static const DevPreset kPresets[] = {
    { "Stock Baseline", "the Drifty defaults" },
    { "Touge Hero",     "950 kg AE86 grip-drift" },
    { "Pro D1GP",       "1320 kg Formula Drift V8" },
    { "Arcade God",     "exaggerated arcade drift" },
    { "Track Predator", "1220 kg GT3 maximum grip" },
    { "Rally Devil",    "1000 kg Group B gravel" },
    { "Pony Car",       "1740 kg American muscle" },
    { "911 Pendulum",   "1520 kg rear-engine Porsche" },
    { "Shoebox",        "760 kg kei-car momentum" },
    { "Time Machine",   "540 kg 1988 F1 turbo" },
};

static const PresetOverride *const kOverrideTables[] = {
    NULL,           /* Stock Baseline — no overrides, reset only */
    kTougeHero,
    kProD1GP,
    kArcadeGod,
    kTrackPredator,
    kRallyDevil,
    kPonyCar,
    k911Pendulum,
    kShoebox,
    kTimeMachine,
};

static const int kOverrideCounts[] = {
    0,
    (int)(sizeof(kTougeHero)     / sizeof(kTougeHero[0])),
    (int)(sizeof(kProD1GP)       / sizeof(kProD1GP[0])),
    (int)(sizeof(kArcadeGod)     / sizeof(kArcadeGod[0])),
    (int)(sizeof(kTrackPredator) / sizeof(kTrackPredator[0])),
    (int)(sizeof(kRallyDevil)    / sizeof(kRallyDevil[0])),
    (int)(sizeof(kPonyCar)       / sizeof(kPonyCar[0])),
    (int)(sizeof(k911Pendulum)   / sizeof(k911Pendulum[0])),
    (int)(sizeof(kShoebox)       / sizeof(kShoebox[0])),
    (int)(sizeof(kTimeMachine)   / sizeof(kTimeMachine[0])),
};

#define PRESET_COUNT ((int)(sizeof(kPresets) / sizeof(kPresets[0])))

/* ---------------------------------------------------------------------------- access -- */

int dev_preset_count(void)
{
    return PRESET_COUNT;
}

const DevPreset *dev_preset_at(int index)
{
    if (index < 0 || index >= PRESET_COUNT) return NULL;
    return &kPresets[index];
}

int dev_preset_apply(VehicleSpec *spec, int presetIndex)
{
    if (spec == NULL || presetIndex < 0 || presetIndex >= PRESET_COUNT) return 0;

    /* Always start from stock so the result is independent of prior tuning. */
    dev_params_reset_all(spec);

    const PresetOverride *overrides = kOverrideTables[presetIndex];
    const int count = kOverrideCounts[presetIndex];
    if (count <= 0 || overrides == NULL) return 0;

    DevParamAssignment *items = (DevParamAssignment *)malloc((size_t)count * sizeof(*items));
    if (items == NULL) return 0;
    for (int i = 0; i < count; i++) {
        items[i].key = overrides[i].key;
        items[i].value = overrides[i].value;
    }
    const int applied = dev_params_apply_assignments(spec, items, count, NULL, NULL);
    free(items);
    return applied;
}
