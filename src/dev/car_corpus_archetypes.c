/*
 * car_corpus_archetypes.c — the 17 hand-designed archetype forms: names, descriptions,
 * and parameter overrides, plus the ArchetypeDef table that binds them. This translation
 * unit owns the table; the rest of the corpus reaches it through car_corpus_internal.h.
 *
 * Raylib-free, like the rest of the corpus.
 */
#include "dev/car_corpus_internal.h"

#include "dev/dev_params.h"

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
static const char *const kArchName0 = "Stock Baseline";
static const char *const kArchDesc0 = "the Drifty defaults; identical to a fresh spec";

/* --------------------------------------------------------------------- 1: Kei Car ----
 * Honda Beat spirit: 660 cc, 760 kg, mid-engine, narrow, tall-ish. */
static const char *const kArchName1 = "Kei Car";
static const char *const kArchDesc1 = "660 cc kei car: short, narrow, tall, tiny tires";
static const DevParamAssignment kArch1[] = {
    A("body.mass", 760.0f),
    A("body.wheelbase", 2.20f),
    A("body.track_front", 1.30f),
    A("body.track_rear", 1.30f),
    A("body.width_overall", 1.40f),
    A("body.height_overall", 1.55f),
    A("body.front_overhang", 0.55f),
    A("body.rear_overhang", 0.50f),
    A("body.ride_height_front", 0.150f),
    A("body.ride_height_rear", 0.155f),
    /* CG height and tire grip: see the note above ARCHETYPE_COUNT. Tall and narrow on skinny
     * economy tires. */
    A("body.cg_height", 0.52f),
    A("tire.lat_front.mu", 0.95f),
    A("tire.lat_rear.mu", 0.90f),
    A("mass.engine_x", 0.90f), /* mid-front */
    A("tire.section_width_front", 155.0f),
    A("tire.section_width_rear", 155.0f),
    A("tire.aspect_front", 65.0f),
    A("tire.aspect_rear", 65.0f),
    A("tire.rim_diameter_front", 13.0f),
    A("tire.rim_diameter_rear", 13.0f),
    A("engine.cylinders", 3.0f),
    A("engine.displacement", 0.66f),
    A("engine.redline_rpm", 8500.0f),
    A("body.nose_width", 0.87f),
    A("body.tail_width", 1.15f),
    A("body.shoulder_x", -0.66f),
};

/* --------------------------------------------------------------------- 2: Compact -----
 * Typical C-segment hatch: 1050 kg, FWD-ish proportions, 1.5L. */
static const char *const kArchName2 = "Compact";
static const char *const kArchDesc2 = "C-segment hatch: economical, modest stance";
static const DevParamAssignment kArch2[] = {
    A("body.mass", 1050.0f),
    A("body.wheelbase", 2.40f),
    A("body.track_front", 1.50f),
    A("body.track_rear", 1.48f),
    A("body.width_overall", 1.65f),
    A("body.height_overall", 1.55f),
    A("body.front_overhang", 0.80f),
    A("body.rear_overhang", 0.65f),
    A("body.ride_height_front", 0.140f),
    A("body.ride_height_rear", 0.145f),
    A("body.cg_height", 0.53f),
    A("tire.lat_front.mu", 1.05f),
    A("tire.lat_rear.mu", 1.00f),
    A("mass.engine_x", 1.40f), /* front transverse */
    A("tire.section_width_front", 185.0f),
    A("tire.section_width_rear", 185.0f),
    A("tire.aspect_front", 60.0f),
    A("tire.aspect_rear", 60.0f),
    A("tire.rim_diameter_front", 15.0f),
    A("tire.rim_diameter_rear", 15.0f),
    A("engine.cylinders", 4.0f),
    A("engine.displacement", 1.5f),
    A("body.nose_width", 0.99f),
    A("body.tail_width", 1.29f),
    A("body.shoulder_x", -0.72f),
};

/* --------------------------------------------------------------------- 3: Coupé ------
 * Two-door sporting profile: low roof, fast backlight, wider stance. */
static const char *const kArchName3 = "Coupe";
static const char *const kArchDesc3 = "low-slung two-door: fast roofline, planted stance";
static const DevParamAssignment kArch3[] = {
    A("body.mass", 1350.0f),
    A("body.wheelbase", 2.60f),
    A("body.track_front", 1.60f),
    A("body.track_rear", 1.58f),
    A("body.width_overall", 1.78f),
    A("body.height_overall", 1.32f),
    A("body.front_overhang", 0.90f),
    A("body.rear_overhang", 0.85f),
    A("body.ride_height_front", 0.120f),
    A("body.ride_height_rear", 0.125f),
    A("body.cg_height", 0.44f),
    A("tire.lat_front.mu", 1.25f),
    A("tire.lat_rear.mu", 1.20f),
    A("body.cowl_x", 0.60f),
    A("body.backlight_x", -0.35f), /* fast roofline */
    A("mass.engine_x", 2.00f),     /* front-mid */
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear", 225.0f),
    A("tire.aspect_front", 45.0f),
    A("tire.aspect_rear", 45.0f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 2.5f),
    A("body.nose_width", 1.00f),
    A("body.tail_width", 1.32f),
    A("body.shoulder_x", -0.88f),
};

/* --------------------------------------------------------------------- 4: Sedan -------
 * Three-box D-segment: long wheelbase, tall greenhouse, balanced. */
static const char *const kArchName4 = "Sedan";
static const char *const kArchDesc4 = "D-segment three-box: roomy greenhouse, balanced stance";
static const DevParamAssignment kArch4[] = {
    A("body.mass", 1550.0f),
    A("body.wheelbase", 2.85f),
    A("body.track_front", 1.62f),
    A("body.track_rear", 1.60f),
    A("body.width_overall", 1.82f),
    A("body.height_overall", 1.52f),
    A("body.front_overhang", 0.90f),
    A("body.rear_overhang", 1.05f),
    A("body.ride_height_front", 0.140f),
    A("body.ride_height_rear", 0.140f),
    A("body.cg_height", 0.52f),
    A("tire.lat_front.mu", 1.10f),
    A("tire.lat_rear.mu", 1.05f),
    A("body.cowl_x", 0.70f),
    A("body.backlight_x", -0.55f),
    A("mass.engine_x", 1.60f), /* front */
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear", 215.0f),
    A("tire.aspect_front", 55.0f),
    A("tire.aspect_rear", 55.0f),
    A("engine.cylinders", 4.0f),
    A("engine.displacement", 2.0f),
    A("body.nose_width", 1.06f),
    A("body.tail_width", 1.38f),
    A("body.shoulder_x", -0.91f),
};

/* ------------------------------------------------------------------ 5: Sports Car ---
 * Mid-engine two-seater: 1100 kg, balanced, 3.0L flat-six, wide tires. */
static const char *const kArchName5 = "Sports Car";
static const char *const kArchDesc5 = "mid-engine two-seater: light, low, balanced grip";
static const DevParamAssignment kArch5[] = {
    A("body.mass", 1100.0f),
    A("body.wheelbase", 2.55f),
    A("body.track_front", 1.60f),
    A("body.track_rear", 1.62f),
    A("body.width_overall", 1.82f),
    A("body.height_overall", 1.22f),
    A("body.front_overhang", 0.85f),
    A("body.rear_overhang", 0.70f),
    A("body.ride_height_front", 0.110f),
    A("body.ride_height_rear", 0.115f),
    A("body.cg_height", 0.40f),
    A("tire.lat_front.mu", 1.40f),
    A("tire.lat_rear.mu", 1.38f),
    A("body.cowl_x", 0.40f),
    A("body.backlight_x", -0.40f),
    A("mass.engine_x", 0.20f), /* mid */
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear", 265.0f),
    A("tire.aspect_front", 40.0f),
    A("tire.aspect_rear", 35.0f),
    A("tire.rim_diameter_front", 18.0f),
    A("tire.rim_diameter_rear", 19.0f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 3.0f),
    A("engine.redline_rpm", 8000.0f),
    A("body.nose_width", 1.06f),
    A("body.tail_width", 1.27f),
    A("body.shoulder_x", -0.87f),
};

/* ------------------------------------------------------------------- 6: Supercar -----
 * Mid-engine exotic: very low, very wide, huge tires, strong rear aero, V12. */
static const char *const kArchName6 = "Supercar";
static const char *const kArchDesc6 =
    "mid-engine exotic: extreme low/wide, giant tires, V12 aero";
static const DevParamAssignment kArch6[] = {
    A("body.mass", 1450.0f),
    A("body.wheelbase", 2.75f),
    A("body.track_front", 1.75f),
    A("body.track_rear", 1.72f),
    A("body.width_overall", 2.05f),
    A("body.height_overall", 1.12f),
    A("body.front_overhang", 1.00f),
    A("body.rear_overhang", 0.90f),
    A("body.ride_height_front", 0.080f),
    A("body.ride_height_rear", 0.085f),
    A("body.cg_height", 0.34f),
    A("tire.lat_front.mu", 1.55f),
    A("tire.lat_rear.mu", 1.55f),
    A("body.cowl_x", 0.30f),
    A("body.backlight_x", -0.55f),
    A("mass.engine_x", -0.50f), /* mid-rear */
    A("tire.section_width_front", 255.0f),
    A("tire.section_width_rear", 325.0f),
    A("tire.aspect_front", 35.0f),
    A("tire.aspect_rear", 30.0f),
    A("tire.rim_diameter_front", 19.0f),
    A("tire.rim_diameter_rear", 20.0f),
    A("aero.lift_rear", -2.50f),  /* strong rear downforce → big wing */
    A("aero.lift_front", -1.50f), /* significant front aero */
    A("aero.ref_area_rear", 1.20f),
    A("aero.ref_area_front", 0.80f),
    A("engine.cylinders", 12.0f),
    A("engine.displacement", 6.0f),
    A("engine.redline_rpm", 9000.0f),
    A("body.nose_width", 1.13f),
    A("body.tail_width", 1.39f),
    A("body.shoulder_x", -0.99f),
    A("body.fender_flare_front", 0.060f),
    A("body.fender_flare_rear", 0.090f),
};

/* ---------------------------------------------------------------- 7: Muscle Car ----
 * American V8 coupe: long hood, wide rear tires, heavy, 6.2L. */
static const char *const kArchName7 = "Muscle Car";
static const char *const kArchDesc7 = "American V8: long hood, wide rear tires, heavy torque";
static const DevParamAssignment kArch7[] = {
    A("body.mass", 1700.0f),
    A("body.wheelbase", 2.85f),
    A("body.track_front", 1.62f),
    A("body.track_rear", 1.68f), /* wider rear — drag-strip stance */
    A("body.width_overall", 1.95f),
    A("body.height_overall", 1.42f),
    A("body.front_overhang", 0.85f),
    A("body.rear_overhang", 1.10f),
    A("body.ride_height_front", 0.135f),
    A("body.ride_height_rear", 0.140f),
    A("body.cg_height", 0.51f),
    A("tire.lat_front.mu", 1.15f),
    A("tire.lat_rear.mu", 1.10f),
    A("body.cowl_x", 0.60f),
    A("body.backlight_x", -0.30f), /* fastback */
    A("mass.engine_x", 2.80f),     /* engine far forward → long hood */
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear", 295.0f), /* fat rears */
    A("tire.aspect_front", 45.0f),
    A("tire.aspect_rear", 40.0f),
    A("tire.rim_diameter_rear", 18.0f),
    A("engine.cylinders", 8.0f),
    A("engine.displacement", 6.2f),
    A("engine.redline_rpm", 6800.0f),
    A("body.nose_width", 1.17f),
    A("body.tail_width", 1.44f),
    A("body.shoulder_x", -0.97f),
    A("body.fender_flare_front", 0.040f),
    A("body.fender_flare_rear", 0.070f),
};

/* ---------------------------------------------------------------- 8: GT3 Race Car ---
 * Circuit racer: low, wide, strong aero both ends, 8-cyl, low ride height. */
static const char *const kArchName8 = "GT3 Race Car";
static const char *const kArchDesc8 =
    "circuit racer: extreme aero, low ride height, wide tires";
static const DevParamAssignment kArch8[] = {
    A("body.mass", 1250.0f),
    A("body.wheelbase", 2.80f),
    A("body.track_front", 1.70f),
    A("body.track_rear", 1.68f),
    A("body.width_overall", 2.00f),
    A("body.height_overall", 1.25f),
    A("body.front_overhang", 1.00f),
    A("body.rear_overhang", 0.90f),
    A("body.ride_height_front", 0.060f),
    A("body.ride_height_rear", 0.065f),
    A("body.cg_height", 0.32f),
    A("tire.lat_front.mu", 1.75f), /* slicks */
    A("tire.lat_rear.mu", 1.75f),
    A("body.cowl_x", 0.30f),
    A("body.backlight_x", -0.60f),
    A("mass.engine_x", -0.40f), /* mid-rear */
    A("tire.section_width_front", 275.0f),
    A("tire.section_width_rear", 305.0f),
    A("tire.aspect_front", 35.0f),
    A("tire.aspect_rear", 30.0f),
    A("tire.rim_diameter_front", 18.0f),
    A("tire.rim_diameter_rear", 18.0f),
    A("aero.lift_rear", -2.80f),
    A("aero.lift_front", -1.80f),
    A("aero.ref_area_rear", 1.50f),
    A("aero.ref_area_front", 1.00f),
    A("engine.cylinders", 8.0f),
    A("engine.displacement", 5.5f),
    A("engine.redline_rpm", 8500.0f),
    A("body.nose_width", 1.12f),
    A("body.tail_width", 1.40f),
    A("body.shoulder_x", -1.01f),
    A("body.fender_flare_front", 0.080f),
    A("body.fender_flare_rear", 0.110f),
};

/* ----------------------------------------------------------------- 9: Rally Car -----
 * Gravel stage: high ride height, long travel, modest tires, turbo 4. */
static const char *const kArchName9 = "Rally Car";
static const char *const kArchDesc9 = "gravel stage: high ride, long travel, turbo 4-cyl";
static const DevParamAssignment kArch9[] = {
    A("body.mass", 1050.0f),
    A("body.wheelbase", 2.50f),
    A("body.track_front", 1.55f),
    A("body.track_rear", 1.55f),
    A("body.width_overall", 1.75f),
    A("body.height_overall", 1.60f),
    A("body.front_overhang", 0.70f),
    A("body.rear_overhang", 0.75f),
    A("body.ride_height_front", 0.220f),
    A("body.ride_height_rear", 0.230f),
    A("body.cg_height", 0.60f),    /* high ride, tall body */
    A("tire.lat_front.mu", 1.20f), /* gravel */
    A("tire.lat_rear.mu", 1.18f),
    A("body.cowl_x", 0.55f),
    A("body.backlight_x", -0.40f),
    A("mass.engine_x", 1.00f), /* front-mid */
    A("susp.travel_front", 0.200f),
    A("susp.travel_rear", 0.210f),
    A("tire.section_width_front", 195.0f),
    A("tire.section_width_rear", 195.0f),
    A("tire.aspect_front", 60.0f),
    A("tire.aspect_rear", 60.0f),
    A("tire.rim_diameter_front", 15.0f),
    A("tire.rim_diameter_rear", 15.0f),
    A("aero.lift_rear", -0.80f),
    A("aero.ref_area_rear", 0.70f),
    A("engine.cylinders", 4.0f),
    A("engine.displacement", 2.0f),
    A("engine.redline_rpm", 8000.0f),
    A("body.nose_width", 1.05f),
    A("body.tail_width", 1.36f),
    A("body.shoulder_x", -0.75f),
    A("body.fender_flare_front", 0.050f),
    A("body.fender_flare_rear", 0.060f),
};

/* -------------------------------------------------------------- 10: Open-Wheel / F1 --
 * Formula-car stance: track >> body width, extremely low, long, light. VISUAL DEMO ONLY
 * (drives poorly — narrow body / wide track gives unstable collision geometry). */
static const char *const kArchName10 = "Open-Wheel";
static const char *const kArchDesc10 =
    "F1 stance: track far wider than body - VISUAL DEMO ONLY";
static const DevParamAssignment kArch10[] = {
    A("body.mass", 600.0f),
    A("body.wheelbase", 3.20f),
    A("body.track_front", 1.95f),
    A("body.track_rear", 1.90f),
    A("body.width_overall", 1.40f), /* narrow body → openWheelWeight fires */
    A("body.height_overall", 0.95f),
    A("body.front_overhang", 0.50f),
    A("body.rear_overhang", 0.50f),
    A("body.ride_height_front", 0.040f),
    A("body.ride_height_rear", 0.045f),
    A("body.cg_height", 0.30f), /* floor of the safe range: shift_particle_z_to_cg
                                              * moves the lowest particle (fuel, 0.30 m) to
                                              * 0.10 m, clear of the 0.05 m registry floor */
    A("tire.lat_front.mu", 1.95f),
    A("tire.lat_rear.mu", 1.95f),
    A("body.cowl_x", 0.10f),
    A("body.backlight_x", -0.60f),
    A("mass.engine_x", -0.20f), /* mid-rear */
    A("tire.section_width_front", 305.0f),
    A("tire.section_width_rear", 355.0f),
    A("tire.aspect_front", 35.0f),
    A("tire.aspect_rear", 30.0f),
    A("tire.rim_diameter_front", 13.0f),
    A("tire.rim_diameter_rear", 13.0f),
    A("aero.lift_rear", -2.00f),
    A("aero.lift_front", -1.50f),
    A("aero.ref_area_rear", 1.00f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 1.6f),
    A("engine.redline_rpm", 10000.0f),
    A("body.nose_width", 0.80f),
    A("body.tail_width", 0.95f),
    A("body.shoulder_x", -0.96f),
};

/* -------------------------------------------------------------- 11: Drift Car ------
 * Competition drift: extreme steering lock, locked diff, rear tire stagger. */
static const char *const kArchName11 = "Drift Car";
static const char *const kArchDesc11 = "competition drift: 65° lock, locked diff, rear stagger";
static const DevParamAssignment kArch11[] = {
    A("body.mass", 1300.0f),
    A("body.wheelbase", 2.65f),
    A("body.track_front", 1.60f),
    A("body.track_rear", 1.58f),
    A("body.width_overall", 1.82f),
    A("body.height_overall", 1.35f),
    A("body.front_overhang", 0.95f),
    A("body.rear_overhang", 0.85f),
    A("body.ride_height_front", 0.090f),
    A("body.ride_height_rear", 0.095f),
    A("body.cg_height", 0.46f),
    /* The rear grip deficit IS the drift setup: balance01 reads muFront - muRear, and this
     * 0.30 spread is the widest in the corpus, which is what makes the axis span its range. */
    A("tire.lat_front.mu", 1.35f),
    A("tire.lat_rear.mu", 1.05f),
    A("body.cowl_x", 0.50f),
    A("body.backlight_x", -0.35f),
    A("mass.engine_x", 1.50f),           /* front */
    A("steer.max_angle", 1.13f),         /* 65° — the Wisefab number */
    A("steer.ackermann_percent", 0.25f), /* mild positive */
    A("susp.camber_front", -0.040f),     /* visible negative camber */
    A("susp.camber_rear", -0.030f),
    A("susp.toe_front", 0.010f),
    A("susp.toe_rear", -0.005f),
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear", 265.0f),
    A("tire.aspect_front", 40.0f),
    A("tire.aspect_rear", 35.0f),
    A("engine.cylinders", 8.0f),
    A("engine.displacement", 5.0f),
    A("engine.redline_rpm", 8000.0f),
    A("body.nose_width", 1.09f),
    A("body.tail_width", 1.33f),
    A("body.shoulder_x", -0.87f),
    A("body.fender_flare_front", 0.050f),
    A("body.fender_flare_rear", 0.100f),
};

/* ------------------------------------------------------------------- 12: Pickup ------
 * Light-duty truck: short cabin and a declared 1.45 m open bed. Before body.bed_length
 * existed this had to be faked by shoving the backlight forward, which is exactly why 74
 * non-trucks grew beds too. */
static const char *const kArchName12 = "Pickup";
static const char *const kArchDesc12 = "light truck: short cabin, long bed, high ride height";
static const DevParamAssignment kArch12[] = {
    A("body.mass", 2200.0f),
    A("body.wheelbase", 3.30f),
    A("body.track_front", 1.70f),
    A("body.track_rear", 1.68f),
    A("body.width_overall", 1.95f),
    A("body.height_overall", 1.85f),
    A("body.front_overhang", 0.85f),
    A("body.rear_overhang", 1.35f), /* long bed behind axle */
    A("body.ride_height_front", 0.200f),
    A("body.ride_height_rear", 0.220f),
    A("body.cg_height", 0.70f),
    A("tire.lat_front.mu", 0.98f),
    A("tire.lat_rear.mu", 0.92f),
    A("body.cowl_x", 0.90f),      /* windscreen far forward */
    A("body.backlight_x", 0.20f), /* short cabin: rear glass ends early */
    A("body.bed_length", 1.45f),  /* the only vehicle here that has one */
    A("mass.engine_x", 2.50f),    /* engine well forward */
    A("tire.section_width_front", 245.0f),
    A("tire.section_width_rear", 245.0f),
    A("tire.aspect_front", 70.0f),
    A("tire.aspect_rear", 70.0f),
    A("tire.rim_diameter_front", 17.0f),
    A("tire.rim_diameter_rear", 17.0f),
    A("engine.cylinders", 8.0f),
    A("engine.displacement", 5.0f),
    A("body.nose_width", 1.21f),
    A("body.tail_width", 1.72f),
    A("body.shoulder_x", -0.79f),
    A("body.fender_flare_front", 0.030f),
    A("body.fender_flare_rear", 0.050f),
};

/* ---------------------------------------------------------------------- 13: Van ------
 * Cargo/commercial van: tall, long greenhouse, side windows fire. */
static const char *const kArchName13 = "Van";
static const char *const kArchDesc13 = "cargo van: tall box, long greenhouse, side windows";
static const DevParamAssignment kArch13[] = {
    A("body.mass", 2100.0f),
    A("body.wheelbase", 3.10f),
    A("body.track_front", 1.62f),
    A("body.track_rear", 1.60f),
    A("body.width_overall", 1.95f),
    A("body.height_overall", 2.20f), /* tall */
    A("body.front_overhang", 0.80f),
    A("body.rear_overhang", 0.80f),
    A("body.ride_height_front", 0.170f),
    A("body.ride_height_rear", 0.175f),
    A("body.cg_height", 0.80f), /* tall cargo box above the floor */
    A("tire.lat_front.mu", 0.92f),
    A("tire.lat_rear.mu", 0.90f),
    A("body.cowl_x", 1.10f),       /* windscreen well forward */
    A("body.backlight_x", -0.70f), /* greenhouse stretches far rearward */
    A("mass.engine_x", 1.00f),
    A("tire.section_width_front", 215.0f),
    A("tire.section_width_rear", 215.0f),
    A("tire.aspect_front", 65.0f),
    A("tire.aspect_rear", 65.0f),
    A("tire.rim_diameter_front", 16.0f),
    A("tire.rim_diameter_rear", 16.0f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 3.5f),
    A("body.nose_width", 1.21f),
    A("body.tail_width", 1.68f),
    A("body.shoulder_x", -0.68f),
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
    A("body.mass", 8000.0f),
    A("body.wheelbase", 6.00f),
    A("body.track_front", 2.00f),
    A("body.track_rear", 2.00f),
    A("body.width_overall", 2.40f),
    A("body.height_overall", 2.80f),
    A("body.front_overhang", 1.50f),
    A("body.rear_overhang", 2.00f),
    A("body.ride_height_front", 0.250f),
    A("body.ride_height_rear", 0.250f),
    A("body.cg_height", 1.05f), /* the fleet's high-water mark for low01 */
    A("tire.lat_front.mu", 0.85f),
    A("tire.lat_rear.mu", 0.85f),
    A("body.cowl_x", 2.00f),       /* driver sits at the very front */
    A("body.backlight_x", -1.80f), /* glass nearly to the tail */
    A("mass.engine_x", -1.50f),    /* rear-engine bus */
    A("tire.section_width_front", 275.0f),
    A("tire.section_width_rear", 275.0f),
    A("tire.aspect_front", 80.0f),
    A("tire.aspect_rear", 80.0f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 8.0f),
    A("body.nose_width", 1.39f),
    A("body.tail_width", 2.02f),
    A("body.shoulder_x", -1.44f),
};

/* ---------------------------------------------------------------- 15: Limousine -----
 * Stretch sedan: very long wheelbase, long greenhouse, moderate height. */
static const char *const kArchName15 = "Limousine";
static const char *const kArchDesc15 = "stretch sedan: long wheelbase, long greenhouse";
static const DevParamAssignment kArch15[] = {
    A("body.mass", 2500.0f),
    A("body.wheelbase", 3.80f),
    A("body.track_front", 1.65f),
    A("body.track_rear", 1.63f),
    A("body.width_overall", 1.90f),
    A("body.height_overall", 1.55f),
    A("body.front_overhang", 1.00f),
    A("body.rear_overhang", 1.20f),
    A("body.ride_height_front", 0.140f),
    A("body.ride_height_rear", 0.140f),
    A("body.cg_height", 0.53f),
    A("tire.lat_front.mu", 1.08f),
    A("tire.lat_rear.mu", 1.05f),
    A("body.cowl_x", 0.80f),
    A("body.backlight_x", -1.00f), /* long greenhouse */
    A("mass.engine_x", 2.00f),
    A("tire.section_width_front", 235.0f),
    A("tire.section_width_rear", 235.0f),
    A("tire.aspect_front", 55.0f),
    A("tire.aspect_rear", 55.0f),
    A("engine.cylinders", 8.0f),
    A("engine.displacement", 4.5f),
    A("body.nose_width", 1.10f),
    A("body.tail_width", 1.48f),
    A("body.shoulder_x", -1.06f),
};

/* ---------------------------------------------------------------- 16: Box Truck -----
 * Commercial box truck: tall, boxy, heavy, long greenhouse. VISUAL DEMO ONLY. */
static const char *const kArchName16 = "Box Truck";
static const char *const kArchDesc16 =
    "commercial box truck: tall, boxy, heavy - VISUAL DEMO ONLY";
static const DevParamAssignment kArch16[] = {
    A("body.mass", 7500.0f),
    A("body.wheelbase", 4.50f),
    A("body.track_front", 1.80f),
    A("body.track_rear", 1.80f),
    A("body.width_overall", 2.20f),
    A("body.height_overall", 2.70f),
    A("body.front_overhang", 1.00f),
    A("body.rear_overhang", 2.00f),
    A("body.ride_height_front", 0.230f),
    A("body.ride_height_rear", 0.230f),
    A("body.cg_height", 0.95f),
    A("tire.lat_front.mu", 0.88f),
    A("tire.lat_rear.mu", 0.85f),
    A("body.cowl_x", 1.60f),       /* cab-forward box truck */
    A("body.backlight_x", -1.50f), /* greenhouse nearly to cargo box end */
    A("mass.engine_x", 0.50f),     /* cab-under engine */
    A("tire.section_width_front", 255.0f),
    A("tire.section_width_rear", 255.0f),
    A("tire.aspect_front", 80.0f),
    A("tire.aspect_rear", 80.0f),
    A("engine.cylinders", 6.0f),
    A("engine.displacement", 8.0f),
    A("body.nose_width", 1.28f),
    A("body.tail_width", 1.87f),
    A("body.shoulder_x", -1.08f),
};

/* ---- archetype table ---- */
typedef struct {
    const char *name;
    const char *description;
    const DevParamAssignment *overrides;
    int overrideCount;
} ArchetypeDef;

static const ArchetypeDef kArchetypes[ARCHETYPE_COUNT] = {
    { kArchName0, kArchDesc0, NULL, 0 },
    { kArchName1, kArchDesc1, kArch1, (int)(sizeof(kArch1) / sizeof(kArch1[0])) },
    { kArchName2, kArchDesc2, kArch2, (int)(sizeof(kArch2) / sizeof(kArch2[0])) },
    { kArchName3, kArchDesc3, kArch3, (int)(sizeof(kArch3) / sizeof(kArch3[0])) },
    { kArchName4, kArchDesc4, kArch4, (int)(sizeof(kArch4) / sizeof(kArch4[0])) },
    { kArchName5, kArchDesc5, kArch5, (int)(sizeof(kArch5) / sizeof(kArch5[0])) },
    { kArchName6, kArchDesc6, kArch6, (int)(sizeof(kArch6) / sizeof(kArch6[0])) },
    { kArchName7, kArchDesc7, kArch7, (int)(sizeof(kArch7) / sizeof(kArch7[0])) },
    { kArchName8, kArchDesc8, kArch8, (int)(sizeof(kArch8) / sizeof(kArch8[0])) },
    { kArchName9, kArchDesc9, kArch9, (int)(sizeof(kArch9) / sizeof(kArch9[0])) },
    { kArchName10, kArchDesc10, kArch10, (int)(sizeof(kArch10) / sizeof(kArch10[0])) },
    { kArchName11, kArchDesc11, kArch11, (int)(sizeof(kArch11) / sizeof(kArch11[0])) },
    { kArchName12, kArchDesc12, kArch12, (int)(sizeof(kArch12) / sizeof(kArch12[0])) },
    { kArchName13, kArchDesc13, kArch13, (int)(sizeof(kArch13) / sizeof(kArch13[0])) },
    { kArchName14, kArchDesc14, kArch14, (int)(sizeof(kArch14) / sizeof(kArch14[0])) },
    { kArchName15, kArchDesc15, kArch15, (int)(sizeof(kArch15) / sizeof(kArch15[0])) },
    { kArchName16, kArchDesc16, kArch16, (int)(sizeof(kArch16) / sizeof(kArch16[0])) },
};

int car_corpus_archetype_count(void)
{
    return ARCHETYPE_COUNT;
}

bool car_corpus_archetype_build(int index, VehicleSpec *out)
{
    if (out == NULL || index < 0 || index >= ARCHETYPE_COUNT) return false;

    vehicle_spec_set_default(out);
    const ArchetypeDef *def = &kArchetypes[index];
    if (def->overrides != NULL && def->overrideCount > 0) {
        (void)dev_params_apply_assignments(out, def->overrides, def->overrideCount, NULL, NULL);
    }
    return true;
}

const char *car_corpus_archetype_name(int index)
{
    if (index < 0 || index >= ARCHETYPE_COUNT) return NULL;
    return kArchetypes[index].name;
}

const char *car_corpus_archetype_description(int index)
{
    if (index < 0 || index >= ARCHETYPE_COUNT) return NULL;
    return kArchetypes[index].description;
}
