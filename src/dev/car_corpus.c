/*
 * car_corpus.c — the demonstration fleet. See the header for the contract.
 *
 * COMPOSITION (Phase 3):
 *   17 archetypes  the stock baseline plus the 16 required hand-designed forms
 *   40 sweeps      8 axes × 5 steps — one registry key varied at a time
 *   43 sampled     deterministic Halton low-discrepancy sequence over a documented box
 *   ───
 *  100 vehicles
 *
 * Sweep bounds are read from the DevParameter registry rather than hardcoded, so widening a
 * parameter's range in Phase 2 automatically widens the sweep that demonstrates it.
 *
 * Sampled generation uses a Halton sequence (bases 2,3,5,7,11,13,17,19,23,29,31,37) over
 * 12 visually-active primary parameters. Candidates are filtered: spec validity, CarVisual
 * derivation, raster bounds, and pairwise similarity against ALL already-accepted vehicles
 * (rejection margin 1.5× the corpus scenario threshold). Generation is bounded at 4096
 * candidates; if 44 cannot be reached, the sampled group is truncated.
 *
 * Raylib-free. Lives in DEV_SRCS rather than SHARED_SRCS because it calls vehicle_spec and
 * raster functions that are linked into the test executable only.
 */
#include "dev/car_corpus.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "dev/dev_params.h"
#include "physics/vehicle.h"

/* --------------------------------------------------------------------------- archetypes --
 *
 * The stock baseline at index 0, then the 16 required forms (kei, compact, coupé, sedan,
 * sports, supercar, muscle, GT3, rally, open-wheel, drift, pickup, van, bus, limousine, box
 * truck) at indices 1..16. Each is a set of parameter overrides on the stock spec, and every
 * value stays within its registry [minimum, maximum]. dev_params_apply_assignments handles
 * migration aliases transparently — presets may use legacy keys like "body.mass" alongside
 * primaries like "tire.section_width_front".
 *
 * Ordered for stability: the index of each archetype is part of the corpus API.
 *
 * WHY EVERY ARCHETYPE SETS cg_height AND BOTH LATERAL mu.
 *
 * car_visual.c derives nine normalised style axes, and three of them read only these fields:
 * low01 <- cgHeightM, grip01 <- max lateral mu, balance01 <- muLatFront - muLatRear. Before
 * these assignments existed no corpus entry touched any of them, so all three were CONSTANT
 * across the whole fleet — measured range exactly 0.000 on 100 vehicles. That froze everything
 * downstream: sport01 is 0.35*grip01 + 0.30*low01 + 0.35*power01, so with two of its three
 * inputs pinned it spanned only 0.142 of 0..1, and the nose taper, tail taper and waist pinch
 * rules that read it could barely move. The grammar was not at fault; it was being fed
 * constants.
 *
 * body.cg_height is a `derived` registry key with a migration alias (dev_params.c ->
 * shift_particle_z_to_cg), which shifts every mass particle's Z by a constant. That shift is
 * NOT clamped, so a target below ~0.25 m would push the lowest particle (fuel, 0.30 m by
 * default) under the 0.05 m registry floor for mass.*_z. The open-wheel car sits at 0.30 m,
 * which is the practical floor with margin.
 *
 * WHY EVERY ARCHETYPE BUT THE BASELINE SETS nose_width, tail_width AND shoulder_x.
 *
 * These three are Phase B [identity] hull stations: the drawn half-width at the foremost and
 * rearmost station is exactly half the declared width, and the shoulder is where the body
 * reaches widthOverallM. They are absolute metres, so one fleet-wide default cannot serve a
 * corpus whose bodies span 1.4 m (kei, open-wheel) to 2.4 m (bus) — the same 0.85 m nose is a
 * blunt face on the kei car and a spearhead on the bus. Every archetype from 1 upward therefore
 * declares its own, proportioned to its own width: roughly 0.58..0.72 of body width at the nose
 * and 0.74..0.88 at the tail for road cars, with the shoulder 0.2..0.36 of a wheelbase behind
 * the axle midpoint (the rear-door station). The commercial forms (van, bus, box truck) are the
 * flattest, because a slab-sided body genuinely is closest to parallel in plan view. Index 0 is
 * the exception by definition: Stock Baseline carries no overrides at all, so it is what
 * exercises the config.h station defaults.
 *
 * body.fender_flare_[front|rear] is declared only where the form has bolt-on arches — the
 * supercar, muscle car, GT3, rally, drift and pickup. It is zero everywhere else, including on
 * the open-wheel car, whose exposed wheels carry no bodywork over them at all.
 *
 * Values are ordinary engineering estimates, not measurements of specific cars.
 */
#define ARCHETYPE_COUNT 17

/* Shorthand for one assignment. */
#define A(key, val) { key, (float)(val) }

/* ----------------------------------------------------------------- 0: Stock Baseline --
 * The config.h defaults with no overrides. Always index 0. */
static const char *const kArchName0  = "Stock Baseline";
static const char *const kArchDesc0  = "the Drifty defaults; identical to a fresh spec";

/* --------------------------------------------------------------------- 1: Kei Car ----
 * Honda Beat spirit: 660 cc, 760 kg, mid-engine, narrow, tall-ish. */
static const char *const kArchName1  = "Kei Car";
static const char *const kArchDesc1  = "660 cc kei car: short, narrow, tall, tiny tires";
static const DevParamAssignment kArch1[] = {
    A("body.mass",                 760.0f),
    A("body.wheelbase",             2.20f),
    A("body.track_front",           1.30f),
    A("body.track_rear",            1.30f),
    A("body.width_overall",         1.40f),
    A("body.height_overall",        1.55f),
    A("body.front_overhang",        0.55f),
    A("body.rear_overhang",         0.50f),
    A("body.ride_height_front",     0.150f),
    A("body.ride_height_rear",      0.155f),
    /* CG height and tire grip: see the note above ARCHETYPE_COUNT. Tall and narrow on skinny
     * economy tires. */
    A("body.cg_height",             0.52f),
    A("tire.lat_front.mu",          0.95f),
    A("tire.lat_rear.mu",           0.90f),
    A("mass.engine_x",              0.90f),   /* mid-front */
    A("tire.section_width_front", 155.0f),
    A("tire.section_width_rear",  155.0f),
    A("tire.aspect_front",         65.0f),
    A("tire.aspect_rear",          65.0f),
    A("tire.rim_diameter_front",   13.0f),
    A("tire.rim_diameter_rear",    13.0f),
    A("engine.cylinders",           3.0f),
    A("engine.displacement",        0.66f),
    A("engine.redline_rpm",      8500.0f),
    A("body.nose_width",         0.87f),
    A("body.tail_width",         1.15f),
    A("body.shoulder_x",        -0.66f),
};

/* --------------------------------------------------------------------- 2: Compact -----
 * Typical C-segment hatch: 1050 kg, FWD-ish proportions, 1.5L. */
static const char *const kArchName2  = "Compact";
static const char *const kArchDesc2  = "C-segment hatch: economical, modest stance";
static const DevParamAssignment kArch2[] = {
    A("body.mass",               1050.0f),
    A("body.wheelbase",            2.40f),
    A("body.track_front",          1.50f),
    A("body.track_rear",           1.48f),
    A("body.width_overall",        1.65f),
    A("body.height_overall",       1.55f),
    A("body.front_overhang",       0.80f),
    A("body.rear_overhang",        0.65f),
    A("body.ride_height_front",    0.140f),
    A("body.ride_height_rear",     0.145f),
    A("body.cg_height",            0.53f),
    A("tire.lat_front.mu",         1.05f),
    A("tire.lat_rear.mu",          1.00f),
    A("mass.engine_x",             1.40f),  /* front transverse */
    A("tire.section_width_front", 185.0f),
    A("tire.section_width_rear",  185.0f),
    A("tire.aspect_front",        60.0f),
    A("tire.aspect_rear",         60.0f),
    A("tire.rim_diameter_front",  15.0f),
    A("tire.rim_diameter_rear",   15.0f),
    A("engine.cylinders",          4.0f),
    A("engine.displacement",       1.5f),
    A("body.nose_width",         0.99f),
    A("body.tail_width",         1.29f),
    A("body.shoulder_x",        -0.72f),
};

/* --------------------------------------------------------------------- 3: Coupé ------
 * Two-door sporting profile: low roof, fast backlight, wider stance. */
static const char *const kArchName3  = "Coupe";
static const char *const kArchDesc3  = "low-slung two-door: fast roofline, planted stance";
static const DevParamAssignment kArch3[] = {
    A("body.mass",               1350.0f),
    A("body.wheelbase",            2.60f),
    A("body.track_front",          1.60f),
    A("body.track_rear",           1.58f),
    A("body.width_overall",        1.78f),
    A("body.height_overall",       1.32f),
    A("body.front_overhang",       0.90f),
    A("body.rear_overhang",        0.85f),
    A("body.ride_height_front",    0.120f),
    A("body.ride_height_rear",     0.125f),
    A("body.cg_height",            0.44f),
    A("tire.lat_front.mu",         1.25f),
    A("tire.lat_rear.mu",          1.20f),
    A("body.cowl_x",               0.60f),
    A("body.backlight_x",         -0.35f),  /* fast roofline */
    A("mass.engine_x",             2.00f),  /* front-mid */
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear",  225.0f),
    A("tire.aspect_front",        45.0f),
    A("tire.aspect_rear",         45.0f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       2.5f),
    A("body.nose_width",         1.00f),
    A("body.tail_width",         1.32f),
    A("body.shoulder_x",        -0.88f),
};

/* --------------------------------------------------------------------- 4: Sedan -------
 * Three-box D-segment: long wheelbase, tall greenhouse, balanced. */
static const char *const kArchName4  = "Sedan";
static const char *const kArchDesc4  = "D-segment three-box: roomy greenhouse, balanced stance";
static const DevParamAssignment kArch4[] = {
    A("body.mass",               1550.0f),
    A("body.wheelbase",            2.85f),
    A("body.track_front",          1.62f),
    A("body.track_rear",           1.60f),
    A("body.width_overall",        1.82f),
    A("body.height_overall",       1.52f),
    A("body.front_overhang",       0.90f),
    A("body.rear_overhang",        1.05f),
    A("body.ride_height_front",    0.140f),
    A("body.ride_height_rear",     0.140f),
    A("body.cg_height",            0.52f),
    A("tire.lat_front.mu",         1.10f),
    A("tire.lat_rear.mu",          1.05f),
    A("body.cowl_x",               0.70f),
    A("body.backlight_x",         -0.55f),
    A("mass.engine_x",             1.60f),  /* front */
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear",  215.0f),
    A("tire.aspect_front",        55.0f),
    A("tire.aspect_rear",         55.0f),
    A("engine.cylinders",          4.0f),
    A("engine.displacement",       2.0f),
    A("body.nose_width",         1.06f),
    A("body.tail_width",         1.38f),
    A("body.shoulder_x",        -0.91f),
};

/* ------------------------------------------------------------------ 5: Sports Car ---
 * Mid-engine two-seater: 1100 kg, balanced, 3.0L flat-six, wide tires. */
static const char *const kArchName5  = "Sports Car";
static const char *const kArchDesc5  = "mid-engine two-seater: light, low, balanced grip";
static const DevParamAssignment kArch5[] = {
    A("body.mass",               1100.0f),
    A("body.wheelbase",            2.55f),
    A("body.track_front",          1.60f),
    A("body.track_rear",           1.62f),
    A("body.width_overall",        1.82f),
    A("body.height_overall",       1.22f),
    A("body.front_overhang",       0.85f),
    A("body.rear_overhang",        0.70f),
    A("body.ride_height_front",    0.110f),
    A("body.ride_height_rear",     0.115f),
    A("body.cg_height",            0.40f),
    A("tire.lat_front.mu",         1.40f),
    A("tire.lat_rear.mu",          1.38f),
    A("body.cowl_x",               0.40f),
    A("body.backlight_x",         -0.40f),
    A("mass.engine_x",             0.20f),  /* mid */
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear",  265.0f),
    A("tire.aspect_front",        40.0f),
    A("tire.aspect_rear",         35.0f),
    A("tire.rim_diameter_front",  18.0f),
    A("tire.rim_diameter_rear",   19.0f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       3.0f),
    A("engine.redline_rpm",      8000.0f),
    A("body.nose_width",         1.06f),
    A("body.tail_width",         1.27f),
    A("body.shoulder_x",        -0.87f),
};

/* ------------------------------------------------------------------- 6: Supercar -----
 * Mid-engine exotic: very low, very wide, huge tires, strong rear aero, V12. */
static const char *const kArchName6  = "Supercar";
static const char *const kArchDesc6  = "mid-engine exotic: extreme low/wide, giant tires, V12 aero";
static const DevParamAssignment kArch6[] = {
    A("body.mass",               1450.0f),
    A("body.wheelbase",            2.75f),
    A("body.track_front",          1.75f),
    A("body.track_rear",           1.72f),
    A("body.width_overall",        2.05f),
    A("body.height_overall",       1.12f),
    A("body.front_overhang",       1.00f),
    A("body.rear_overhang",        0.90f),
    A("body.ride_height_front",    0.080f),
    A("body.ride_height_rear",     0.085f),
    A("body.cg_height",            0.34f),
    A("tire.lat_front.mu",         1.55f),
    A("tire.lat_rear.mu",          1.55f),
    A("body.cowl_x",               0.30f),
    A("body.backlight_x",         -0.55f),
    A("mass.engine_x",            -0.50f),  /* mid-rear */
    A("tire.section_width_front", 255.0f),
    A("tire.section_width_rear",  325.0f),
    A("tire.aspect_front",        35.0f),
    A("tire.aspect_rear",         30.0f),
    A("tire.rim_diameter_front",  19.0f),
    A("tire.rim_diameter_rear",   20.0f),
    A("aero.lift_rear",          -2.50f),  /* strong rear downforce → big wing */
    A("aero.lift_front",         -1.50f),  /* significant front aero */
    A("aero.ref_area_rear",       1.20f),
    A("aero.ref_area_front",      0.80f),
    A("engine.cylinders",        12.0f),
    A("engine.displacement",      6.0f),
    A("engine.redline_rpm",     9000.0f),
    A("body.nose_width",         1.13f),
    A("body.tail_width",         1.39f),
    A("body.shoulder_x",        -0.99f),
    A("body.fender_flare_front", 0.060f),
    A("body.fender_flare_rear",  0.090f),
};

/* ---------------------------------------------------------------- 7: Muscle Car ----
 * American V8 coupe: long hood, wide rear tires, heavy, 6.2L. */
static const char *const kArchName7  = "Muscle Car";
static const char *const kArchDesc7  = "American V8: long hood, wide rear tires, heavy torque";
static const DevParamAssignment kArch7[] = {
    A("body.mass",               1700.0f),
    A("body.wheelbase",            2.85f),
    A("body.track_front",          1.62f),
    A("body.track_rear",           1.68f),  /* wider rear — drag-strip stance */
    A("body.width_overall",        1.95f),
    A("body.height_overall",       1.42f),
    A("body.front_overhang",       0.85f),
    A("body.rear_overhang",        1.10f),
    A("body.ride_height_front",    0.135f),
    A("body.ride_height_rear",     0.140f),
    A("body.cg_height",            0.51f),
    A("tire.lat_front.mu",         1.15f),
    A("tire.lat_rear.mu",          1.10f),
    A("body.cowl_x",               0.60f),
    A("body.backlight_x",         -0.30f),  /* fastback */
    A("mass.engine_x",             2.80f),  /* engine far forward → long hood */
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear",  295.0f),  /* fat rears */
    A("tire.aspect_front",        45.0f),
    A("tire.aspect_rear",         40.0f),
    A("tire.rim_diameter_rear",   18.0f),
    A("engine.cylinders",          8.0f),
    A("engine.displacement",       6.2f),
    A("engine.redline_rpm",      6800.0f),
    A("body.nose_width",         1.17f),
    A("body.tail_width",         1.44f),
    A("body.shoulder_x",        -0.97f),
    A("body.fender_flare_front", 0.040f),
    A("body.fender_flare_rear",  0.070f),
};

/* ---------------------------------------------------------------- 8: GT3 Race Car ---
 * Circuit racer: low, wide, strong aero both ends, 8-cyl, low ride height. */
static const char *const kArchName8  = "GT3 Race Car";
static const char *const kArchDesc8  = "circuit racer: extreme aero, low ride height, wide tires";
static const DevParamAssignment kArch8[] = {
    A("body.mass",               1250.0f),
    A("body.wheelbase",            2.80f),
    A("body.track_front",          1.70f),
    A("body.track_rear",           1.68f),
    A("body.width_overall",        2.00f),
    A("body.height_overall",       1.25f),
    A("body.front_overhang",       1.00f),
    A("body.rear_overhang",        0.90f),
    A("body.ride_height_front",    0.060f),
    A("body.ride_height_rear",     0.065f),
    A("body.cg_height",            0.32f),
    A("tire.lat_front.mu",         1.75f),   /* slicks */
    A("tire.lat_rear.mu",          1.75f),
    A("body.cowl_x",               0.30f),
    A("body.backlight_x",         -0.60f),
    A("mass.engine_x",            -0.40f),  /* mid-rear */
    A("tire.section_width_front", 275.0f),
    A("tire.section_width_rear",  305.0f),
    A("tire.aspect_front",        35.0f),
    A("tire.aspect_rear",         30.0f),
    A("tire.rim_diameter_front",  18.0f),
    A("tire.rim_diameter_rear",   18.0f),
    A("aero.lift_rear",          -2.80f),
    A("aero.lift_front",         -1.80f),
    A("aero.ref_area_rear",       1.50f),
    A("aero.ref_area_front",      1.00f),
    A("engine.cylinders",          8.0f),
    A("engine.displacement",       5.5f),
    A("engine.redline_rpm",      8500.0f),
    A("body.nose_width",         1.12f),
    A("body.tail_width",         1.40f),
    A("body.shoulder_x",        -1.01f),
    A("body.fender_flare_front", 0.080f),
    A("body.fender_flare_rear",  0.110f),
};

/* ----------------------------------------------------------------- 9: Rally Car -----
 * Gravel stage: high ride height, long travel, modest tires, turbo 4. */
static const char *const kArchName9  = "Rally Car";
static const char *const kArchDesc9  = "gravel stage: high ride, long travel, turbo 4-cyl";
static const DevParamAssignment kArch9[] = {
    A("body.mass",               1050.0f),
    A("body.wheelbase",            2.50f),
    A("body.track_front",          1.55f),
    A("body.track_rear",           1.55f),
    A("body.width_overall",        1.75f),
    A("body.height_overall",       1.60f),
    A("body.front_overhang",       0.70f),
    A("body.rear_overhang",        0.75f),
    A("body.ride_height_front",    0.220f),
    A("body.ride_height_rear",     0.230f),
    A("body.cg_height",            0.60f),   /* high ride, tall body */
    A("tire.lat_front.mu",         1.20f),   /* gravel */
    A("tire.lat_rear.mu",          1.18f),
    A("body.cowl_x",               0.55f),
    A("body.backlight_x",         -0.40f),
    A("mass.engine_x",             1.00f),  /* front-mid */
    A("susp.travel_front",         0.200f),
    A("susp.travel_rear",          0.210f),
    A("tire.section_width_front", 195.0f),
    A("tire.section_width_rear",  195.0f),
    A("tire.aspect_front",        60.0f),
    A("tire.aspect_rear",         60.0f),
    A("tire.rim_diameter_front",  15.0f),
    A("tire.rim_diameter_rear",   15.0f),
    A("aero.lift_rear",          -0.80f),
    A("aero.ref_area_rear",       0.70f),
    A("engine.cylinders",          4.0f),
    A("engine.displacement",       2.0f),
    A("engine.redline_rpm",      8000.0f),
    A("body.nose_width",         1.05f),
    A("body.tail_width",         1.36f),
    A("body.shoulder_x",        -0.75f),
    A("body.fender_flare_front", 0.050f),
    A("body.fender_flare_rear",  0.060f),
};

/* -------------------------------------------------------------- 10: Open-Wheel / F1 --
 * Formula-car stance: track >> body width, extremely low, long, light. VISUAL DEMO ONLY
 * (drives poorly — narrow body / wide track gives unstable collision geometry). */
static const char *const kArchName10 = "Open-Wheel";
static const char *const kArchDesc10 = "F1 stance: track far wider than body - VISUAL DEMO ONLY";
static const DevParamAssignment kArch10[] = {
    A("body.mass",                600.0f),
    A("body.wheelbase",            3.20f),
    A("body.track_front",          1.95f),
    A("body.track_rear",           1.90f),
    A("body.width_overall",        1.40f),  /* narrow body → openWheelWeight fires */
    A("body.height_overall",       0.95f),
    A("body.front_overhang",       0.50f),
    A("body.rear_overhang",        0.50f),
    A("body.ride_height_front",    0.040f),
    A("body.ride_height_rear",     0.045f),
    A("body.cg_height",            0.30f),   /* floor of the safe range: shift_particle_z_to_cg
                                              * moves the lowest particle (fuel, 0.30 m) to
                                              * 0.10 m, clear of the 0.05 m registry floor */
    A("tire.lat_front.mu",         1.95f),
    A("tire.lat_rear.mu",          1.95f),
    A("body.cowl_x",               0.10f),
    A("body.backlight_x",         -0.60f),
    A("mass.engine_x",            -0.20f),  /* mid-rear */
    A("tire.section_width_front", 305.0f),
    A("tire.section_width_rear",  355.0f),
    A("tire.aspect_front",        35.0f),
    A("tire.aspect_rear",         30.0f),
    A("tire.rim_diameter_front",  13.0f),
    A("tire.rim_diameter_rear",   13.0f),
    A("aero.lift_rear",          -2.00f),
    A("aero.lift_front",         -1.50f),
    A("aero.ref_area_rear",       1.00f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       1.6f),
    A("engine.redline_rpm",     10000.0f),
    A("body.nose_width",         0.80f),
    A("body.tail_width",         0.95f),
    A("body.shoulder_x",        -0.96f),
};

/* -------------------------------------------------------------- 11: Drift Car ------
 * Competition drift: extreme steering lock, locked diff, rear tire stagger. */
static const char *const kArchName11 = "Drift Car";
static const char *const kArchDesc11 = "competition drift: 65° lock, locked diff, rear stagger";
static const DevParamAssignment kArch11[] = {
    A("body.mass",               1300.0f),
    A("body.wheelbase",            2.65f),
    A("body.track_front",          1.60f),
    A("body.track_rear",           1.58f),
    A("body.width_overall",        1.82f),
    A("body.height_overall",       1.35f),
    A("body.front_overhang",       0.95f),
    A("body.rear_overhang",        0.85f),
    A("body.ride_height_front",    0.090f),
    A("body.ride_height_rear",     0.095f),
    A("body.cg_height",            0.46f),
    /* The rear grip deficit IS the drift setup: balance01 reads muFront - muRear, and this
     * 0.30 spread is the widest in the corpus, which is what makes the axis span its range. */
    A("tire.lat_front.mu",         1.35f),
    A("tire.lat_rear.mu",          1.05f),
    A("body.cowl_x",               0.50f),
    A("body.backlight_x",         -0.35f),
    A("mass.engine_x",             1.50f),  /* front */
    A("steer.max_angle",           1.13f),  /* 65° — the Wisefab number */
    A("steer.ackermann_percent",   0.25f),  /* mild positive */
    A("susp.camber_front",        -0.040f), /* visible negative camber */
    A("susp.camber_rear",         -0.030f),
    A("susp.toe_front",            0.010f),
    A("susp.toe_rear",            -0.005f),
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear",  265.0f),
    A("tire.aspect_front",        40.0f),
    A("tire.aspect_rear",         35.0f),
    A("engine.cylinders",          8.0f),
    A("engine.displacement",       5.0f),
    A("engine.redline_rpm",      8000.0f),
    A("body.nose_width",         1.09f),
    A("body.tail_width",         1.33f),
    A("body.shoulder_x",        -0.87f),
    A("body.fender_flare_front", 0.050f),
    A("body.fender_flare_rear",  0.100f),
};

/* ------------------------------------------------------------------- 12: Pickup ------
 * Light-duty truck: short cabin and a declared 1.45 m open bed. Before body.bed_length
 * existed this had to be faked by shoving the backlight forward, which is exactly why 74
 * non-trucks grew beds too. */
static const char *const kArchName12 = "Pickup";
static const char *const kArchDesc12 = "light truck: short cabin, long bed, high ride height";
static const DevParamAssignment kArch12[] = {
    A("body.mass",               2200.0f),
    A("body.wheelbase",            3.30f),
    A("body.track_front",          1.70f),
    A("body.track_rear",           1.68f),
    A("body.width_overall",        1.95f),
    A("body.height_overall",       1.85f),
    A("body.front_overhang",       0.85f),
    A("body.rear_overhang",        1.35f),  /* long bed behind axle */
    A("body.ride_height_front",    0.200f),
    A("body.ride_height_rear",     0.220f),
    A("body.cg_height",            0.70f),
    A("tire.lat_front.mu",         0.98f),
    A("tire.lat_rear.mu",          0.92f),
    A("body.cowl_x",               0.90f),  /* windscreen far forward */
    A("body.backlight_x",          0.20f),  /* short cabin: rear glass ends early */
    A("body.bed_length",           1.45f),  /* the only vehicle here that has one */
    A("mass.engine_x",             2.50f),  /* engine well forward */
    A("tire.section_width_front", 245.0f),
    A("tire.section_width_rear",  245.0f),
    A("tire.aspect_front",        70.0f),
    A("tire.aspect_rear",         70.0f),
    A("tire.rim_diameter_front",  17.0f),
    A("tire.rim_diameter_rear",   17.0f),
    A("engine.cylinders",          8.0f),
    A("engine.displacement",       5.0f),
    A("body.nose_width",         1.21f),
    A("body.tail_width",         1.72f),
    A("body.shoulder_x",        -0.79f),
    A("body.fender_flare_front", 0.030f),
    A("body.fender_flare_rear",  0.050f),
};

/* ---------------------------------------------------------------------- 13: Van ------
 * Cargo/commercial van: tall, long greenhouse, side windows fire. */
static const char *const kArchName13 = "Van";
static const char *const kArchDesc13 = "cargo van: tall box, long greenhouse, side windows";
static const DevParamAssignment kArch13[] = {
    A("body.mass",               2100.0f),
    A("body.wheelbase",            3.10f),
    A("body.track_front",          1.62f),
    A("body.track_rear",           1.60f),
    A("body.width_overall",        1.95f),
    A("body.height_overall",       2.20f),  /* tall */
    A("body.front_overhang",       0.80f),
    A("body.rear_overhang",        0.80f),
    A("body.ride_height_front",    0.170f),
    A("body.ride_height_rear",     0.175f),
    A("body.cg_height",            0.80f),   /* tall cargo box above the floor */
    A("tire.lat_front.mu",         0.92f),
    A("tire.lat_rear.mu",          0.90f),
    A("body.cowl_x",               1.10f),  /* windscreen well forward */
    A("body.backlight_x",         -0.70f), /* greenhouse stretches far rearward */
    A("mass.engine_x",             1.00f),
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear",  215.0f),
    A("tire.aspect_front",        65.0f),
    A("tire.aspect_rear",         65.0f),
    A("tire.rim_diameter_front",  16.0f),
    A("tire.rim_diameter_rear",   16.0f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       3.5f),
    A("body.nose_width",         1.21f),
    A("body.tail_width",         1.68f),
    A("body.shoulder_x",        -0.68f),
};

/* ---------------------------------------------------------------------- 14: Bus ------
 * City bus: extremely long, very tall, very heavy, long greenhouse. VISUAL DEMO ONLY. */
static const char *const kArchName14 = "Bus";
static const char *const kArchDesc14 = "city bus: extreme length and height - VISUAL DEMO ONLY";
static const DevParamAssignment kArch14[] = {
    /* 8 t, not the ~12 t of a real city bus: vehicle_spec_is_valid() caps the derived
     * tireLoadRefPerWheelN at 20 kN, i.e. mass * g / 4 <= 20000 => mass <= 8155 kg. The bus
     * exists here for its silhouette, and mass is not a visual driver above mass01's 8 t
     * ceiling, so the model limit is respected rather than the validator widened. */
    A("body.mass",               8000.0f),
    A("body.wheelbase",            6.00f),
    A("body.track_front",          2.00f),
    A("body.track_rear",           2.00f),
    A("body.width_overall",        2.40f),
    A("body.height_overall",       2.80f),
    A("body.front_overhang",       1.50f),
    A("body.rear_overhang",        2.00f),
    A("body.ride_height_front",    0.250f),
    A("body.ride_height_rear",     0.250f),
    A("body.cg_height",            1.05f),   /* the fleet's high-water mark for low01 */
    A("tire.lat_front.mu",         0.85f),
    A("tire.lat_rear.mu",          0.85f),
    A("body.cowl_x",               2.00f),  /* driver sits at the very front */
    A("body.backlight_x",         -1.80f), /* glass nearly to the tail */
    A("mass.engine_x",            -1.50f), /* rear-engine bus */
    A("tire.section_width_front", 275.0f),
    A("tire.section_width_rear",  275.0f),
    A("tire.aspect_front",        80.0f),
    A("tire.aspect_rear",         80.0f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       8.0f),
    A("body.nose_width",         1.39f),
    A("body.tail_width",         2.02f),
    A("body.shoulder_x",        -1.44f),
};

/* ---------------------------------------------------------------- 15: Limousine -----
 * Stretch sedan: very long wheelbase, long greenhouse, moderate height. */
static const char *const kArchName15 = "Limousine";
static const char *const kArchDesc15 = "stretch sedan: long wheelbase, long greenhouse";
static const DevParamAssignment kArch15[] = {
    A("body.mass",               2500.0f),
    A("body.wheelbase",            3.80f),
    A("body.track_front",          1.65f),
    A("body.track_rear",           1.63f),
    A("body.width_overall",        1.90f),
    A("body.height_overall",       1.55f),
    A("body.front_overhang",       1.00f),
    A("body.rear_overhang",        1.20f),
    A("body.ride_height_front",    0.140f),
    A("body.ride_height_rear",     0.140f),
    A("body.cg_height",            0.53f),
    A("tire.lat_front.mu",         1.08f),
    A("tire.lat_rear.mu",          1.05f),
    A("body.cowl_x",               0.80f),
    A("body.backlight_x",         -1.00f), /* long greenhouse */
    A("mass.engine_x",             2.00f),
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear",  235.0f),
    A("tire.aspect_front",        55.0f),
    A("tire.aspect_rear",         55.0f),
    A("engine.cylinders",          8.0f),
    A("engine.displacement",       4.5f),
    A("body.nose_width",         1.10f),
    A("body.tail_width",         1.48f),
    A("body.shoulder_x",        -1.06f),
};

/* ---------------------------------------------------------------- 16: Box Truck -----
 * Commercial box truck: tall, boxy, heavy, long greenhouse. VISUAL DEMO ONLY. */
static const char *const kArchName16 = "Box Truck";
static const char *const kArchDesc16 = "commercial box truck: tall, boxy, heavy - VISUAL DEMO ONLY";
static const DevParamAssignment kArch16[] = {
    A("body.mass",               7500.0f),
    A("body.wheelbase",            4.50f),
    A("body.track_front",          1.80f),
    A("body.track_rear",           1.80f),
    A("body.width_overall",        2.20f),
    A("body.height_overall",       2.70f),
    A("body.front_overhang",       1.00f),
    A("body.rear_overhang",        2.00f),
    A("body.ride_height_front",    0.230f),
    A("body.ride_height_rear",     0.230f),
    A("body.cg_height",            0.95f),
    A("tire.lat_front.mu",         0.88f),
    A("tire.lat_rear.mu",          0.85f),
    A("body.cowl_x",               1.60f),  /* cab-forward box truck */
    A("body.backlight_x",         -1.50f), /* greenhouse nearly to cargo box end */
    A("mass.engine_x",             0.50f),  /* cab-under engine */
    A("tire.section_width_front", 255.0f),
    A("tire.section_width_rear",  255.0f),
    A("tire.aspect_front",        80.0f),
    A("tire.aspect_rear",         80.0f),
    A("engine.cylinders",          6.0f),
    A("engine.displacement",       8.0f),
    A("body.nose_width",         1.28f),
    A("body.tail_width",         1.87f),
    A("body.shoulder_x",        -1.08f),
};

/* ---- archetype table ---- */
typedef struct {
    const char *name;
    const char *description;
    const DevParamAssignment *overrides;
    int overrideCount;
} ArchetypeDef;

static const ArchetypeDef kArchetypes[ARCHETYPE_COUNT] = {
    { kArchName0,  kArchDesc0,  NULL,       0 },
    { kArchName1,  kArchDesc1,  kArch1,     (int)(sizeof(kArch1)  / sizeof(kArch1[0])) },
    { kArchName2,  kArchDesc2,  kArch2,     (int)(sizeof(kArch2)  / sizeof(kArch2[0])) },
    { kArchName3,  kArchDesc3,  kArch3,     (int)(sizeof(kArch3)  / sizeof(kArch3[0])) },
    { kArchName4,  kArchDesc4,  kArch4,     (int)(sizeof(kArch4)  / sizeof(kArch4[0])) },
    { kArchName5,  kArchDesc5,  kArch5,     (int)(sizeof(kArch5)  / sizeof(kArch5[0])) },
    { kArchName6,  kArchDesc6,  kArch6,     (int)(sizeof(kArch6)  / sizeof(kArch6[0])) },
    { kArchName7,  kArchDesc7,  kArch7,     (int)(sizeof(kArch7)  / sizeof(kArch7[0])) },
    { kArchName8,  kArchDesc8,  kArch8,     (int)(sizeof(kArch8)  / sizeof(kArch8[0])) },
    { kArchName9,  kArchDesc9,  kArch9,     (int)(sizeof(kArch9)  / sizeof(kArch9[0])) },
    { kArchName10, kArchDesc10, kArch10,    (int)(sizeof(kArch10) / sizeof(kArch10[0])) },
    { kArchName11, kArchDesc11, kArch11,    (int)(sizeof(kArch11) / sizeof(kArch11[0])) },
    { kArchName12, kArchDesc12, kArch12,    (int)(sizeof(kArch12) / sizeof(kArch12[0])) },
    { kArchName13, kArchDesc13, kArch13,    (int)(sizeof(kArch13) / sizeof(kArch13[0])) },
    { kArchName14, kArchDesc14, kArch14,    (int)(sizeof(kArch14) / sizeof(kArch14[0])) },
    { kArchName15, kArchDesc15, kArch15,    (int)(sizeof(kArch15) / sizeof(kArch15[0])) },
    { kArchName16, kArchDesc16, kArch16,    (int)(sizeof(kArch16) / sizeof(kArch16[0])) },
};

/* ------------------------------------------------------------------------------ sweeps --
 *
 * Eight axes, five steps each = 40 vehicles. Each sweep varies EXACTLY ONE registry key;
 * all other parameters stay at stock. The varied key is documented beside each axis.
 */
#define SWEEP_AXES   8
#define SWEEP_STEPS  5

/* One sweep axis. A row is five cars that differ in this key and in nothing else. */
typedef struct {
    const char *key;    /* the ONE registry key this row varies */
    const char *note;   /* the visual dimension the row demonstrates */
} SweepAxis;

/* WHY TIRE ASPECT RATIO IS NOT ONE OF THESE AXES.
 *
 * Tire aspect ratio is an obvious candidate for a sweep axis — it changes the drawn tire
 * diameter directly — and it is deliberately not here. That is a measured decision, not an
 * omission.
 *
 * Every PAIR of corpus vehicles has to clear three floors at once: >= 3% of union-silhouette
 * pixels differing, signature L2 >= 0.25, and signature L-infinity >= 0.08 m. Adjacent steps
 * of a five-step sweep are the tightest pairs in the whole fleet, so a sweep axis has to move
 * roughly a fifth of its range past all three floors, four times over.
 *
 * A tire's drawn diameter is rim + 2 * section * aspect, and aspect touches essentially
 * nothing else: it moves the tire diameter, the sidewall band inside it, and the arch drawn
 * around it — two of four wheels, on a shape seen from above. On the stock 225/45R17 the
 * registry's whole 25%..80% span moves the diameter by 2 * 0.225 * 0.55 = 0.2475 m, i.e.
 * 0.062 m per step: below the 0.08 m one-pixel floor before anything else is considered.
 * Widening the registry to close that would need ~80 points of aspect (25..105), which is not
 * a tire that exists.
 *
 * Building the row on a wide-tire, open-wheel base does clear L-infinity — a 355 mm section
 * gives 0.098 m per step — and was tried. It still fails the other two floors, and this is
 * arithmetic rather than a tuning problem: the pixel metric is a FRACTION of a shared
 * silhouette, and L2 is dominated by how MANY components move. Aspect ratio moves two. Even
 * with the body shrunk to the registry's minimum in every dimension, the measured figures
 * were 2.1% of pixels and L2 = 0.107 against floors of 3% and 0.25.
 *
 * So the axis is covered where it can be honest: `tire.aspect_front` and `tire.aspect_rear`
 * are both designated visual drivers in the `car-visual` sensitivity table, perturbed across
 * their full declared range — a 45% change in tire diameter, which clears the sensitivity
 * floors comfortably and is plainly visible on the contact sheet. What it cannot do is carry
 * five mutually distinct cars, and asserting otherwise would mean lowering a corpus-wide
 * threshold to accommodate one row.
 */
static const SweepAxis kSweepAxes[SWEEP_AXES] = {
    { "mass.engine_x",       "engine station: CG, layout read, hood bulge" },
    { "body.wheelbase",      "axle span and the body length that follows it" },
    { "body.width_overall",  "silhouette width and fender flare" },
    { "body.height_overall", "roof share of the plan area, glass, side windows" },
    { "body.front_overhang", "nose length ahead of the front axle" },
    { "body.track_front",    "front stance: track against body width" },
    { "body.shoulder_x",     "station of maximum width: where the body is widest" },
    { "body.rear_overhang",  "tail length behind the rear axle" },
};

/* WHY body.backlight_x LEFT THIS TABLE, AND WHY body.bed_length DID NOT REPLACE IT.
 *
 * Both are the tire-aspect story above, told twice more.
 *
 * backlight_x used to carry TWO visual jobs: the rear glass station, and — through the old
 * inference — the pickup bed. The second job was a bug (it grew beds on 74 non-trucks) and
 * body.bed_length replaced it. Stripped of the bed, the axis moves only cabin length, roof
 * panel and glass band, and the row stopped clearing the floors: sweep_body_backlight_x_3
 * measured L-infinity 0.0700 m and L2 0.1071 against sweep_mass_engine_x_0, versus floors of
 * 0.08 m and 0.25. The axis genuinely does less than it used to.
 *
 * bed_length was the obvious replacement and does not work either, for the opposite reason.
 * Its four upper steps are strongly distinct (measured bed areas 133 / 209 / 285 / 380 px),
 * but its default is 0 — most vehicles have no bed — so the generator's exclusion window puts
 * the FIRST step at 0.1875 m. At 13.2 px/m that is a 2.5 px lip that renders as nothing, so
 * step 0 came out byte-identical to the stock baseline: 0.00% of pixels differing. Widening
 * the range cannot help, because the exclusion window is half a grid step and scales with it.
 * A bed that short is not a configuration a real vehicle has.
 *
 * body.rear_overhang takes the slot instead: 0.2..2.5 m is 0.575 m per step, and it moves
 * length, both tail hull stations and the tail taper together, so it clears the pixel and L2
 * floors with room to spare.
 *
 * Both displaced keys stay covered where they can be honest — `body.backlight_x` and
 * `body.bed_length` are designated visual drivers in the `car-visual` sensitivity table,
 * perturbed across their whole declared registry range.
 */

/* WHY aero.lift_rear LEFT THIS TABLE IN PHASE B, AND WHY body.shoulder_x TOOK THE SLOT.
 *
 * The same story a third time, caused by a deliberate change rather than by a bug. Before
 * Phase B the tail taper read the SIGNED rear lift coefficient, so this axis moved both the
 * wing and the whole rear silhouette. Phase B made the silhouette endpoints explicit
 * ([identity] body.nose_width / body.tail_width) and retired the derived taper, which is what
 * closed the measured 3x tail-closure gap — and left aero.lift_rear moving only the wing
 * bolted to the deck. Measured with `drifty_tests --measure-sweep aero.lift_rear` after that
 * change, its adjacent steps read:
 *
 *     pair 0-1  pixels 0.0313  L2 0.2447  Linf 0.2337 m   (L2 below the 0.25 floor)
 *     pair 1-2  pixels 0.0552  L2 0.2574  Linf 0.2338 m
 *     pair 2-3  pixels 0.0356  L2 0.2447  Linf 0.2337 m   (L2 below the floor)
 *     pair 3-4  pixels 0.0280  L2 1.2688  Linf 1.2538 m   (pixels below the 0.030 floor)
 *
 * Three of four adjacent pairs miss a floor. A wing is two rectangles over the deck: it cannot
 * carry five mutually distinct cars on its own, and Phase F is where wing span and chord become
 * explicit dimensions rather than a coefficient's shadow.
 *
 * body.shoulder_x takes the slot because it was measured to clear all three floors on every
 * adjacent pair:
 *
 *     pair 0-1  pixels 0.1068  L2 1.7659  Linf 1.7500 m
 *     pair 1-2  pixels 0.0710  L2 0.7686  Linf 0.7500 m
 *     pair 2-3  pixels 0.0573  L2 0.7693  Linf 0.7500 m
 *     pair 3-4  pixels 0.0363  L2 0.7604  Linf 0.7500 m
 *
 * It moves the maximum-width station along the body, so both hull sections change length at
 * once and several stations move together — the property a single appendage dimension lacks.
 *
 * THE OTHER TWO PHASE B CANDIDATES CANNOT BE SWEPT AT ALL, and the reason is the identity
 * itself rather than the fidelity budget. body.nose_width and body.tail_width are hull
 * endpoints, and vehicle_spec_is_valid() rejects an endpoint wider than widthOverallM: an
 * endpoint past the body's maximum width would falsify both the endpoint identity and the
 * shoulder's claim to be the widest station. The stock body is 1.70 m wide while the declared
 * registry ranges reach 2.2 m and 2.4 m, so the generated rows die on validity:
 *
 *     body.nose_width  step 3 = 1.90625 m  -> invalid spec
 *     body.tail_width  step 2 = 1.80000 m  -> invalid spec
 *
 * A sweep row varies exactly one key, so widening body.width_overall alongside is not
 * available, and narrowing the registry range to fit one base would misreport what the
 * parameter can express on a 2.4 m bus. Both keys are therefore designated visual drivers in
 * the `car-visual` sensitivity table instead, where they are perturbed across their whole
 * declared range from stock, and the archetypes exercise them as authored proportions.
 */

/* ---- sweep value computation ----
 *
 * Even spacing over [min, max] is wrong when the stock default sits inside the interval: a
 * step landing on the default would reproduce the stock baseline exactly. Map steps through
 * the complement of an exclusion window around the default so every step is at least one
 * visible tick away from stock, the sequence stays monotonic, and neighbouring steps cannot
 * collapse onto the same value.
 */
static float sweep_compute_value(const DevParameter *param, int step, int totalSteps)
{

    const float span = param->maximum - param->minimum;
    const float stepSpan = (totalSteps > 1) ? (span / (float)(totalSteps - 1)) : span;
    /* Minimum exclusion around stock: half a grid step, but never less than 0.08 m for
     * metre-valued drivers (one screen pixel at ~13.2 px/m). For dimensionless keys like
     * aero coefficients, use at least 10% of the span. */
    const float metreExclude = fmaxf(0.5f * fabsf(stepSpan), 0.08f);
    const float dimExclude = fmaxf(0.5f * fabsf(stepSpan), 0.10f * span);
    const float exclude = fmaxf(metreExclude, dimExclude);

    const float leftHi  = param->defaultValue - exclude;
    const float rightLo = param->defaultValue + exclude;
    const float leftLen  = fmaxf(0.0f, leftHi - param->minimum);
    const float rightLen = fmaxf(0.0f, param->maximum - rightLo);
    const float usable = leftLen + rightLen;

    float value;
    if (!(usable > 0.0f)) {
        /* Degenerate registry range; fall back to plain endpoints. */
        const float t = (totalSteps > 1) ? ((float)step / (float)(totalSteps - 1)) : 0.0f;
        value = param->minimum + span * t;
    } else {
        const float t = (totalSteps > 1) ? ((float)step / (float)(totalSteps - 1)) : 0.0f;
        const float pos = t * usable;
        if (pos <= leftLen) {
            value = param->minimum + fminf(pos, leftLen);
        } else {
            value = rightLo + fminf(pos - leftLen, rightLen);
        }
    }
    return value;
}

/* Build the spec for one sweep slot: the stock car with a single key moved. */
static void sweep_build_spec(int axis, int step, VehicleSpec *out)
{
    vehicle_spec_set_default(out);
    if (axis < 0 || axis >= SWEEP_AXES) return;

    const DevParameter *param = dev_param_find(kSweepAxes[axis].key);
    if (param == NULL) return;
    (void)dev_param_set(out, param, sweep_compute_value(param, step, SWEEP_STEPS));
}

/* Build the spec a sweep row WOULD use for a key that is not on the axis table. Shares
 * sweep_compute_value with the real generator, so a measurement cannot flatter a candidate by
 * spacing its steps differently from the fleet that would ship. */
int car_corpus_sweep_steps(void) { return SWEEP_STEPS; }

bool car_corpus_sweep_probe(const char *key, int step, VehicleSpec *out)
{
    if (key == NULL || out == NULL) return false;
    if (step < 0 || step >= SWEEP_STEPS) return false;

    const DevParameter *param = dev_param_find(key);
    if (param == NULL) return false;

    vehicle_spec_set_default(out);
    return dev_param_set(out, param, sweep_compute_value(param, step, SWEEP_STEPS));
}

/* ---- sampled generation ----
 *
 * 43 vehicles generated from a Halton low-discrepancy sequence over a documented parameter
 * box. Candidates are filtered: must pass vehicle_spec_is_valid(), car_visual_derive(),
 * raster-bounds check, and be sufficiently distinct from ALL already-accepted vehicles
 * (archetypes + sweeps + previously-accepted sampled).
 *
 * HALTON BOX. Twelve primary parameters are varied independently using Halton bases
 * 2,3,5,7,11,13,17,19,23,29,31,37. The box bounds are chosen within the registry range
 * to span the plausible passenger-vehicle space while avoiding degenerate combinations:
 *
 *   Index  Base  Registry key            Box min   Box max   Registry range
 *   ─────  ────  ──────────────────────  ────────  ────────  ──────────────
 *     0      2   body.wheelbase            2.000     5.500    1.80 – 7.00
 *     1      3   body.track_front          1.100     2.300    1.00 – 2.60
 *     2      5   body.width_overall        1.300     2.400    1.20 – 2.60
 *     3      7   body.height_overall       1.100     2.800    1.00 – 3.20
 *     4     11   body.front_overhang       0.300     1.800    0.20 – 2.50
 *     5     13   body.rear_overhang        0.300     1.800    0.20 – 2.50
 *     6     17   mass.engine_x            -2.500     3.000   -4.00 – 4.00
 *     7     19   body.cowl_x              -1.500     1.500   -2.00 – 2.00
 *     8     23   body.backlight_x         -1.500     1.500   -2.00 – 2.00
 *     9     29   tire.section_width_front 155.000   325.000  145.00 – 355.00
 *    10     31   tire.aspect_front         30.000    70.000   25.00 – 80.00
 *    11     37   aero.lift_rear           -2.000     0.500   -3.00 – 1.00
 */
#define SAMPLED_COUNT   43
#define HALTON_DIMS     12
#define MAX_CANDIDATES  4096

/* Rejection thresholds: 1.5× the corpus scenario distinctness floor. */
#define SAMPLED_REJECT_PIXEL   (1.5f * 0.030f)   /* 0.045 */
#define SAMPLED_REJECT_LINF    (1.5f * 0.080f)   /* 0.120 */

static const int kHaltonBases[HALTON_DIMS] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37 };
static const char *const kHaltonKeys[HALTON_DIMS] = {
    "body.wheelbase", "body.track_front", "body.width_overall",
    "body.height_overall", "body.front_overhang", "body.rear_overhang",
    "mass.engine_x", "body.cowl_x", "body.backlight_x",
    "tire.section_width_front", "tire.aspect_front", "aero.lift_rear",
};
static const float kHaltonBoxMin[HALTON_DIMS] = {
    2.000f, 1.100f, 1.300f, 1.100f, 0.300f, 0.300f,
    -2.500f, -1.500f, -1.500f, 155.000f, 30.000f, -2.000f,
};
static const float kHaltonBoxMax[HALTON_DIMS] = {
    5.500f, 2.300f, 2.400f, 2.800f, 1.800f, 1.800f,
    3.000f, 1.500f, 1.500f, 325.000f, 70.000f, 0.500f,
};

/* Cached sampled specs. Populated on first access. */
static VehicleSpec g_sampledSpecs[SAMPLED_COUNT];
static int g_sampledAccepted = -1;  /* -1 = not generated yet */

/* ---- Halton sequence ---- */
static float halton(int index, int base)
{
    float result = 0.0f;
    float f = 1.0f / (float)base;
    int i = index;
    while (i > 0) {
        result += f * (float)(i % base);
        i /= base;
        f /= (float)base;
    }
    return result;
}

/* Map a Halton point [0,1] into the box range [boxMin, boxMax]. */
static float halton_to_box(float t, float boxMin, float boxMax)
{
    return boxMin + t * (boxMax - boxMin);
}

/* Build a label map for a spec on the given canvas. Returns true on success. */
static bool build_label_map(const VehicleSpec *spec, const CarRasterInfo *canvas,
                            unsigned char *labels, size_t bytes)
{
    CarVisual visual;
    car_visual_derive(spec, &visual);
    return car_raster_draw_labels(&visual, *canvas, labels, bytes);
}

/* Pre-compute canvas info from archetypes + sweeps, with generous margin. */
static CarRasterInfo compute_canvas(void)
{
    const float pxPerM = 13.2f;  /* PIXELS_PER_METER * CAMERA_BASE_ZOOM ≈ 13.2 */
    float left = 1.0f, right = 1.0f, up = 1.0f, down = 1.0f;

    for (int i = 0; i < ARCHETYPE_COUNT + SWEEP_AXES * SWEEP_STEPS; i++) {
        VehicleSpec spec;
        vehicle_spec_set_default(&spec);

        if (i < ARCHETYPE_COUNT) {
            const ArchetypeDef *def = &kArchetypes[i];
            if (def->overrides != NULL) {
                dev_params_apply_assignments(&spec, def->overrides, def->overrideCount,
                                             NULL, NULL);
            }
        } else {
            const int si = i - ARCHETYPE_COUNT;
            sweep_build_spec(si / SWEEP_STEPS, si % SWEEP_STEPS, &spec);
        }

        CarVisual visual;
        car_visual_derive(&spec, &visual);
        const CarRasterInfo info = car_raster_info(&visual, pxPerM, 2);
        const float l = info.originXPx;
        const float r = (float)info.width - info.originXPx;
        const float u = info.originYPx;
        const float d = (float)info.height - info.originYPx;
        if (l > left)   left  = l;
        if (r > right)  right = r;
        if (u > up)     up    = u;
        if (d > down)   down  = d;
    }

    /* Add 2.5 m margin on each side for sampled vehicles. */
    const float marginPx = 2.5f * pxPerM;
    CarRasterInfo shared;
    memset(&shared, 0, sizeof(shared));
    shared.pxPerM    = pxPerM;
    shared.width     = (int)ceilf(left + right + 2.0f * marginPx);
    shared.height    = (int)ceilf(up + down + 2.0f * marginPx);
    shared.originXPx = left + marginPx;
    shared.originYPx = up + marginPx;
    return shared;
}

/* Generate every sampled vehicle. Called once on first access. Returns true when the full
 * SAMPLED_COUNT was reached; false means generation ran out of candidates and truncated,
 * which car_corpus_spec() reports rather than papering over. */
static bool generate_sampled(void)
{
    if (g_sampledAccepted >= 0) return (g_sampledAccepted == SAMPLED_COUNT);

    g_sampledAccepted = 0;

    /* Phase 1: compute canvas, rasterize archetypes + sweeps for comparison. */
    const CarRasterInfo canvas = compute_canvas();
    const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
    if (pixels == 0) return false;

    const int baseVehicles = ARCHETYPE_COUNT + SWEEP_AXES * SWEEP_STEPS;
    /* Store label maps for all accepted vehicles (grows as sampled are accepted). */
    const int maxAccepted = baseVehicles + SAMPLED_COUNT;  /* 17 + 40 + 43 = 100 */
    unsigned char **acceptedLabels =
        (unsigned char **)malloc((size_t)maxAccepted * sizeof(unsigned char *));
    VehicleSpec *acceptedSpecs =
        (VehicleSpec *)malloc((size_t)maxAccepted * sizeof(VehicleSpec));
    if (acceptedLabels == NULL || acceptedSpecs == NULL) {
        free(acceptedLabels);
        free(acceptedSpecs);
        return false;
    }

    int acceptedCount = 0;

    /* Build and store archetypes. */
    for (int i = 0; i < ARCHETYPE_COUNT; i++) {
        VehicleSpec spec;
        vehicle_spec_set_default(&spec);
        const ArchetypeDef *def = &kArchetypes[i];
        if (def->overrides != NULL) {
            dev_params_apply_assignments(&spec, def->overrides, def->overrideCount,
                                         NULL, NULL);
        }
        acceptedLabels[acceptedCount] =
            (unsigned char *)calloc(pixels, 1);
        if (acceptedLabels[acceptedCount] == NULL ||
            !build_label_map(&spec, &canvas, acceptedLabels[acceptedCount], pixels)) {
            for (int j = 0; j <= acceptedCount; j++) free(acceptedLabels[j]);
            free(acceptedLabels);
            free(acceptedSpecs);
            return false;
        }
        acceptedSpecs[acceptedCount] = spec;
        acceptedCount++;
    }

    /* Build and store sweeps. */
    for (int si = 0; si < SWEEP_AXES * SWEEP_STEPS; si++) {
        VehicleSpec spec;
        sweep_build_spec(si / SWEEP_STEPS, si % SWEEP_STEPS, &spec);
        acceptedLabels[acceptedCount] =
            (unsigned char *)calloc(pixels, 1);
        if (acceptedLabels[acceptedCount] == NULL ||
            !build_label_map(&spec, &canvas, acceptedLabels[acceptedCount], pixels)) {
            for (int j = 0; j <= acceptedCount; j++) free(acceptedLabels[j]);
            free(acceptedLabels);
            free(acceptedSpecs);
            return false;
        }
        acceptedSpecs[acceptedCount] = spec;
        acceptedCount++;
    }

    /* Phase 2: iterate Halton candidates. */
    int candidatesTried = 0;
    for (int seed = 1; seed <= MAX_CANDIDATES && g_sampledAccepted < SAMPLED_COUNT; seed++) {
        candidatesTried++;

        /* Build candidate spec from Halton point. */
        VehicleSpec candidate;
        vehicle_spec_set_default(&candidate);
        for (int d = 0; d < HALTON_DIMS; d++) {
            const float t = halton(seed, kHaltonBases[d]);
            const float val = halton_to_box(t, kHaltonBoxMin[d], kHaltonBoxMax[d]);
            const DevParameter *param = dev_param_find(kHaltonKeys[d]);
            if (param != NULL) {
                dev_param_set(&candidate, param, val);
            }
        }

        /* Validate. */
        if (!vehicle_spec_is_valid(&candidate)) continue;

        /* Derive visual and check raster. */
        CarVisual candVis;
        car_visual_derive(&candidate, &candVis);
        const CarRasterInfo candInfo = car_raster_info(&candVis, canvas.pxPerM, 0);
        if (candInfo.width <= 0 || candInfo.height <= 0) continue;

        /* Rasterize candidate. */
        unsigned char *candLabels = (unsigned char *)calloc(pixels, 1);
        if (candLabels == NULL) continue;
        if (!car_raster_draw_labels(&candVis, canvas, candLabels, pixels)) {
            free(candLabels);
            continue;
        }

        /* Compute candidate signature. */
        float candSig[CAR_SIGNATURE_MAX];
        const int sigN = car_visual_signature_count();
        if (sigN <= 0 || sigN > (int)(sizeof(candSig) / sizeof(candSig[0]))) {
            free(candLabels);
            continue;
        }
        if (car_visual_signature(&candVis, candSig, sigN) != sigN) {
            free(candLabels);
            continue;
        }

        /* Check against all already-accepted vehicles. */
        bool tooSimilar = false;
        for (int a = 0; a < acceptedCount && !tooSimilar; a++) {
            /* Pixel label-map difference. */
            const float pixDiff = car_raster_difference(candLabels, acceptedLabels[a],
                                                        canvas.width, canvas.height);
            if (pixDiff < SAMPLED_REJECT_PIXEL) {
                tooSimilar = true;
                break;
            }

            /* Signature Linf. */
            CarVisual accVis;
            car_visual_derive(&acceptedSpecs[a], &accVis);
            float accSig[CAR_SIGNATURE_MAX];
            if (car_visual_signature(&accVis, accSig, sigN) == sigN) {
                float worst = 0.0f;
                for (int k = 0; k < sigN; k++) {
                    const float d = fabsf(candSig[k] - accSig[k]);
                    if (d > worst) worst = d;
                }
                if (worst < SAMPLED_REJECT_LINF) {
                    tooSimilar = true;
                    break;
                }
            }
        }

        if (tooSimilar) {
            free(candLabels);
            continue;
        }

        /* Accepted! */
        g_sampledSpecs[g_sampledAccepted] = candidate;
        acceptedLabels[acceptedCount] = candLabels;
        acceptedSpecs[acceptedCount] = candidate;
        acceptedCount++;
        g_sampledAccepted++;
    }

    /* Cleanup. */
    for (int j = 0; j < acceptedCount; j++) free(acceptedLabels[j]);
    free(acceptedLabels);
    free(acceptedSpecs);

    if (g_sampledAccepted < SAMPLED_COUNT) {
        fprintf(stderr,
                "car_corpus: sampled generation reached %d/%d vehicles"
                " (%d candidates tried, max %d)\n",
                g_sampledAccepted, SAMPLED_COUNT, candidatesTried, MAX_CANDIDATES);
    }
    return (g_sampledAccepted == SAMPLED_COUNT);
}

/* -------------------------------------------------------------------- public API ---- */

static int archetype_count(void) { return ARCHETYPE_COUNT; }
static int sweep_count(void)     { return SWEEP_AXES * SWEEP_STEPS; }
static int sampled_count(void)   { return SAMPLED_COUNT; }

int car_corpus_count(void)
{
    return archetype_count() + sweep_count() + sampled_count();
}

CarCorpusGroup car_corpus_group(int index)
{
    if (index < archetype_count()) return CAR_CORPUS_ARCHETYPE;
    if (index < archetype_count() + sweep_count()) return CAR_CORPUS_SWEEP;
    return CAR_CORPUS_SAMPLED;
}

const char *car_corpus_group_name(CarCorpusGroup group)
{
    switch (group) {
        case CAR_CORPUS_ARCHETYPE: return "archetype";
        case CAR_CORPUS_SWEEP:     return "sweep";
        case CAR_CORPUS_SAMPLED:   return "sampled";
        default:                   return "?";
    }
}

/* Decompose a sweep index into axis and step. */
static bool sweep_slot(int index, int *axisOut, int *stepOut)
{
    const int base = archetype_count();
    const int end  = base + sweep_count();
    if (index < base || index >= end) return false;
    const int offset = index - base;
    if (axisOut != NULL) *axisOut = offset / SWEEP_STEPS;
    if (stepOut != NULL) *stepOut = offset % SWEEP_STEPS;
    return true;
}

const char *car_corpus_sweep_key(int index)
{
    int axis = 0;
    if (!sweep_slot(index, &axis, NULL)) return NULL;
    if (axis < 0 || axis >= SWEEP_AXES) return NULL;
    return kSweepAxes[axis].key;
}

bool car_corpus_spec(int index, VehicleSpec *out)
{
    if (out == NULL || index < 0 || index >= car_corpus_count()) return false;

    vehicle_spec_set_default(out);

    /* Archetypes. */
    if (index < archetype_count()) {
        const ArchetypeDef *def = &kArchetypes[index];
        if (def->overrides != NULL && def->overrideCount > 0) {
            (void)dev_params_apply_assignments(out, def->overrides, def->overrideCount,
                                               NULL, NULL);
        }
        return true;
    }

    /* Sweeps. */
    int base = archetype_count();
    if (index < base + sweep_count()) {
        int axis = 0, step = 0;
        if (!sweep_slot(index, &axis, &step)) return false;
        sweep_build_spec(axis, step, out);
        return true;
    }

    /* Sampled. Lazy-generation on first access. */
    if (!generate_sampled()) {
        /* Generation truncated; only the available specs are valid. */
        const int sampledIdx = index - archetype_count() - sweep_count();
        if (sampledIdx < 0 || sampledIdx >= g_sampledAccepted) return false;
        *out = g_sampledSpecs[sampledIdx];
        return true;
    }

    const int sampledIdx = index - archetype_count() - sweep_count();
    if (sampledIdx < 0 || sampledIdx >= SAMPLED_COUNT) return false;
    *out = g_sampledSpecs[sampledIdx];
    return true;
}

/* Lower-case, replace anything awkward with '_'. */
static void slugify(const char *src, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    size_t n = 0;
    for (; src != NULL && *src != '\0' && n + 1 < cap; src++) {
        const char c = *src;
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
            buf[n++] = c;
        } else if (c >= 'A' && c <= 'Z') {
            buf[n++] = (char)(c - 'A' + 'a');
        } else if (n > 0 && buf[n - 1] != '_') {
            buf[n++] = '_';
        }
    }
    while (n > 0 && buf[n - 1] == '_') n--;
    buf[n] = '\0';
}

void car_corpus_id(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    buf[0] = '\0';
    if (index < 0 || index >= car_corpus_count()) return;

    if (index < archetype_count()) {
        const ArchetypeDef *def = &kArchetypes[index];
        char slug[48];
        slugify(def->name, slug, sizeof(slug));
        snprintf(buf, cap, "archetype_%02d_%s", index, slug);
        return;
    }

    int base = archetype_count();
    if (index < base + sweep_count()) {
        int axis = 0, step = 0;
        (void)sweep_slot(index, &axis, &step);
        char slug[48];
        slugify(kSweepAxes[axis].key, slug, sizeof(slug));
        snprintf(buf, cap, "sweep_%s_%d", slug, step);
        return;
    }

    /* Sampled. */
    const int sampledIdx = index - base - sweep_count();
    snprintf(buf, cap, "sampled_%02d", sampledIdx);
}

void car_corpus_describe(int index, char *buf, size_t cap)
{
    if (buf == NULL || cap == 0) return;
    buf[0] = '\0';
    if (index < 0 || index >= car_corpus_count()) return;

    /* Archetypes. */
    if (index < archetype_count()) {
        const ArchetypeDef *def = &kArchetypes[index];
        snprintf(buf, cap, "%s - %s", def->name, def->description);
        return;
    }

    int base = archetype_count();

    /* Sweeps. */
    if (index < base + sweep_count()) {
        int step = 0;
        const DevParameter *param = NULL;
        (void)sweep_slot(index, NULL, &step);

        VehicleSpec spec;
        if (!car_corpus_spec(index, &spec)) return;

        const char *key = car_corpus_sweep_key(index);
        param = (key != NULL) ? dev_param_find(key) : NULL;
        if (param == NULL) return;

        const float value = dev_param_get(&spec, param);
        snprintf(buf, cap, "%s = %.3f %s (step %d/%d)",
                 param->name, (double)value,
                 (param->unit != NULL && param->unit[0] != '\0') ? param->unit : "",
                 step + 1, SWEEP_STEPS);
        return;
    }

    /* Sampled. */
    const int sampledIdx = index - base - sweep_count();
    snprintf(buf, cap, "sampled_%02d - Halton seed %d", sampledIdx, sampledIdx);
}
