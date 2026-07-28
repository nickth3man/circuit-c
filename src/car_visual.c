/*
 * car_visual.c — the vehicle appearance grammar. See car_visual.h for the contract.
 *
 * Every feature below is labelled [identity], [rule] or [phase-2]:
 *
 *   [identity]  the drawn value IS the physical value
 *   [rule]      a documented styling mapping that cites the parameters it reads
 *   [phase-2]   a constant standing in for a parameter that does not exist yet, identical for
 *               every car so it contributes no false variety
 *
 * Phase 1 deliberately uses [phase-2] constants rather than inventing relationships such as
 * "tire width follows from grip coefficient". A constant is honest; a fabricated correlation
 * is not, and it would make the contact sheet lie about which parameters matter.
 *
 * Raylib-free: linked into drifty_tests.exe.
 */
#include "car_visual.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "math_utils.h"

/* ------------------------------------------------------------------ phase-2 constants --
 *
 * Presentation-only gains that are not VehicleSpec fields. Grep TODO(phase-2) is retired;
 * geometry now reads the Phase 2 primaries. */
/* Presentation gain on the resting wheel angle. Static toe is ~0.15 degrees, which is far
 * below the ~7.6 cm / one-pixel visibility floor at the scale the game draws at, so the cue is
 * amplified to be seen at all. Render-only, exactly like steerVisualGain in render.c; no
 * simulation quantity reads it. */
#define CV_REST_ANGLE_GAIN    6.0f

/* ---------------------------------------------------------------------------- helpers -- */

/* Clamped linear normalise: 0 at lo, 1 at hi, monotonic in between. */
static float u01(float v, float lo, float hi)
{
    if (!(hi > lo)) return 0.0f;
    return clampf((v - lo) / (hi - lo), 0.0f, 1.0f);
}

static float maxf(float a, float b) { return (a > b) ? a : b; }

static unsigned char to_u8(float v)
{
    return (unsigned char)clampf(v * 255.0f + 0.5f, 0.0f, 255.0f);
}

/* HSV -> RGB. Local because raylib's ColorFromHSV is a raylib *call*, and this TU must stay
 * link-free of raylib. h in degrees, s and v in [0,1]. */
static Color hsv_to_color(float h, float s, float v, unsigned char a)
{
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    s = clampf(s, 0.0f, 1.0f);
    v = clampf(v, 0.0f, 1.0f);

    const float c = v * s;
    const float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    const float m = v - c;
    float r = 0.0f, g = 0.0f, b = 0.0f;

    if      (h <  60.0f) { r = c; g = x; }
    else if (h < 120.0f) { r = x; g = c; }
    else if (h < 180.0f) { g = c; b = x; }
    else if (h < 240.0f) { g = x; b = c; }
    else if (h < 300.0f) { r = x; b = c; }
    else                 { r = c; b = x; }

    return (Color){ to_u8(r + m), to_u8(g + m), to_u8(b + m), a };
}

static Color shade(Color c, float factor)
{
    return (Color){ to_u8((float)c.r / 255.0f * factor),
                    to_u8((float)c.g / 255.0f * factor),
                    to_u8((float)c.b / 255.0f * factor),
                    c.a };
}

/* ------------------------------------------------------------------------ colour seed --
 *
 * FNV-1a over an explicitly listed set of fields. Never over the raw struct bytes: those
 * include padding, which is unspecified and would make the colour depend on the compiler.
 * Colour is the ONE stated exception to the no-hashing rule (car_visual.h); no geometry below
 * reads this value. */
static void fnv_mix(uint32_t *h, float value)
{
    /* Quantise before hashing so that two specs which are equal for every practical purpose
     * cannot land on wildly different colours through float noise. */
    const int32_t q = (int32_t)lrintf(value * 1024.0f);
    const unsigned char *p = (const unsigned char *)&q;
    for (size_t i = 0; i < sizeof(q); i++) {
        *h ^= p[i];
        *h *= 16777619u;
    }
}

uint32_t car_visual_colour_seed(const VehicleSpec *spec)
{
    if (spec == NULL) return 0u;
    uint32_t h = 2166136261u;
    fnv_mix(&h, spec->massKg);
    fnv_mix(&h, spec->cgToFrontM);
    fnv_mix(&h, spec->cgToRearM);
    fnv_mix(&h, spec->trackWidthFrontM);
    fnv_mix(&h, spec->trackWidthRearM);
    fnv_mix(&h, spec->wheelRadiusFrontM);
    fnv_mix(&h, spec->wheelRadiusRearM);
    fnv_mix(&h, spec->bodyHalfWidthM);
    fnv_mix(&h, spec->cgHeightM);
    fnv_mix(&h, spec->tireMuLatFront);
    fnv_mix(&h, spec->tireMuLatRear);
    fnv_mix(&h, spec->dragCoefficient);
    fnv_mix(&h, spec->maxBrakeTorqueNm);
    fnv_mix(&h, spec->finalDriveRatio);
    fnv_mix(&h, spec->engineRedlineRpm);
    return h;
}

/* --------------------------------------------------------------------------- latents -- */

static float peak_engine_torque(const VehicleSpec *spec)
{
    float peak = 0.0f;
    for (int i = 0; i < ENGINE_CURVE_POINTS; i++) {
        if (spec->engineTorqueCurveNm[i] > peak) peak = spec->engineTorqueCurveNm[i];
    }
    return peak;
}

CarLatents car_visual_latents(const VehicleSpec *spec)
{
    CarLatents l;
    memset(&l, 0, sizeof(l));
    if (spec == NULL) return l;

    const float wheelbase = spec->cgToFrontM + spec->cgToRearM;
    const float muMax = maxf(spec->tireMuLatFront, spec->tireMuLatRear);
    const float mass = maxf(spec->massKg, 1.0f);
    const float tractive = peak_engine_torque(spec) * maxf(spec->finalDriveRatio, 0.1f) / mass;

    l.mass01    = u01(spec->massKg, 600.0f, 2500.0f);
    l.size01    = u01(wheelbase, 1.90f, 3.20f);
    l.low01     = 1.0f - u01(spec->cgHeightM, 0.20f, 1.00f);
    l.grip01    = u01(muMax, 0.50f, 2.00f);
    l.balance01 = u01(spec->tireMuLatFront - spec->tireMuLatRear, -0.20f, 0.45f);
    l.power01   = u01(tractive, 0.30f, 2.50f);
    l.aero01    = u01(spec->dragCoefficient * spec->frontalAreaM2, 0.35f, 3.00f);
    l.sport01   = clampf(0.35f * l.grip01 + 0.30f * l.low01 + 0.35f * l.power01, 0.0f, 1.0f);
    /* Light for its size reads as a stripped racecar; heavy for its size reads as a road car. */
    l.strip01   = clampf(l.sport01 * (1.0f - u01(spec->massKg / maxf(wheelbase, 0.1f),
                                                 250.0f, 900.0f)), 0.0f, 1.0f);
    return l;
}

/* ------------------------------------------------------------------------ hull profile --
 *
 * [rule] Half-width as a fraction of the body half-width, along t from tail (0) to nose (1).
 * The tail rises from its taper, the middle carries a slight waist pinch, and the nose falls
 * away. Sporty cars get a sharper nose and a tighter waist; heavy cars keep a fuller tail. */
static float hull_profile(float t, float noseTaper, float tailTaper, float waistDepth)
{
    t = clampf(t, 0.0f, 1.0f);

    float frac;
    if (t < 0.18f) {
        frac = lerpf(tailTaper, 1.0f, smoothstep(0.0f, 0.18f, t));
    } else if (t > 0.62f) {
        frac = lerpf(1.0f, noseTaper, smoothstep(0.62f, 1.0f, t));
    } else {
        frac = 1.0f;
    }

    /* A shallow dip centred between the axles, so the car is not a lozenge. */
    const float d = (t - 0.42f) / 0.26f;
    const float bump = expf(-d * d);
    return clampf(frac * (1.0f - waistDepth * bump), 0.02f, 1.30f);
}

/* ---------------------------------------------------------------------------- derive -- */

void car_visual_derive(const VehicleSpec *spec, CarVisual *out)
{
    if (spec == NULL || out == NULL) return;
    memset(out, 0, sizeof(*out));

    const CarLatents l = car_visual_latents(spec);
    out->latents = l;

    /* ---- principal dimensions ---- */

    /* [identity] the wheelbase the simulation uses. */
    const float wheelbase = spec->cgToFrontM + spec->cgToRearM;
    out->wheelbaseM = wheelbase;

    /* [identity] overhangs from the Phase 2 geometry primaries. */
    out->frontOverhangM = spec->frontOverhangM;
    out->rearOverhangM  = spec->rearOverhangM;

    const float noseX = spec->cgToFrontM + out->frontOverhangM;
    const float tailX = -(spec->cgToRearM + out->rearOverhangM);
    out->lengthM = noseX - tailX;

    /* [identity] the collision half-width is the drawn half-width. A wider car both looks
     * wider and hits barriers sooner — collision.c uses this same value as its capsule
     * radius, so the picture cannot disagree with the physics. */
    const float halfW = spec->bodyHalfWidthM;
    out->widthM = 2.0f * halfW;

    /* ---- hull outline ---- */

    /* [rule] a slippery, sporty car gets a sharper nose; a heavy one keeps a fuller tail.
     * The spans are deliberately wide: at ~13 px/m a 10% difference in half-width is well
     * under a pixel, so a timid mapping produces fifty identical lozenges. */
    const float noseTaper  = clampf(0.74f - 0.42f * l.sport01 - 0.20f * (1.0f - l.aero01),
                                    0.14f, 0.95f);
    const float tailTaper  = clampf(0.80f - 0.34f * l.sport01 + 0.20f * l.mass01, 0.20f, 1.00f);
    const float waistDepth = 0.02f + 0.10f * l.sport01;

    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        const float t = (float)i / (float)(CAR_HULL_STATIONS - 1);
        out->hull[i].xM = lerpf(tailX, noseX, t);
        out->hull[i].halfWidthM = halfW * hull_profile(t, noseTaper, tailTaper, waistDepth);
    }

    /* ---- wheels ---- */

    /* [identity] tire width from section width; diameter from per-axle loaded radius. */
    const float tireWidthF = spec->tireSectionWidthFrontMm * 0.001f;
    const float tireWidthR = spec->tireSectionWidthRearMm * 0.001f;
    const float tireDiaF = 2.0f * spec->wheelRadiusFrontM;
    const float tireDiaR = 2.0f * spec->wheelRadiusRearM;
    /* [rule] rim diameter from designation inches, clamped inside the tire. */
    const float rimDiaF = fminf(spec->tireRimDiameterFrontIn * 0.0254f, tireDiaF * 0.92f);
    const float rimDiaR = fminf(spec->tireRimDiameterRearIn * 0.0254f, tireDiaR * 0.92f);

    /* [rule] a heavier wheel reads as fewer, fatter spokes. */
    const float inertia = spec->wheelInertiaKgM2;
    const int spokes = (inertia < 0.70f) ? 10 : (inertia < 1.00f) ? 8
                     : (inertia < 1.40f) ?  6 : (inertia < 2.00f) ? 5 : 4;

    /* [rule] disc diameter scales with the brake torque the axle can actually apply. */
    const float torque01 = u01(spec->maxBrakeTorqueNm, 0.0f, 8000.0f);
    const float biasF = clampf(spec->brakeBiasFront, 0.0f, 1.0f);

    /* [rule] at rest the front wheels sit at a small angle so steering geometry is legible,
     * scaled by the Ackermann setting. Presentation-gained; physics is untouched. */
    const float restAngle = clampf(spec->ackermannPercent, 0.0f, 1.0f)
                          * 0.012f * CV_REST_ANGLE_GAIN;

    const float halfTrackF = 0.5f * spec->trackWidthFrontM;
    const float halfTrackR = 0.5f * spec->trackWidthRearM;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool isFront = (i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT);
        const bool isLeft  = (i == WHEEL_FRONT_LEFT || i == WHEEL_REAR_LEFT);
        CarWheelVisual *w = &out->wheels[i];

        /* [identity] must equal vehicle.c set_wheel_positions(); asserted by `car-visual`. */
        w->centreM.x = isFront ? spec->cgToFrontM : -spec->cgToRearM;
        w->centreM.y = (isLeft ? 1.0f : -1.0f) * (isFront ? halfTrackF : halfTrackR);

        w->diameterM    = isFront ? tireDiaF : tireDiaR;
        w->widthM       = isFront ? tireWidthF : tireWidthR;
        w->rimDiameterM = isFront ? rimDiaF : rimDiaR;
        w->discDiameterM = w->rimDiameterM * (0.70f + 0.22f * torque01)
                         * (isFront ? (0.85f + 0.30f * biasF)
                                    : (0.85f + 0.30f * (1.0f - biasF)));
        w->spokeCount = spokes;
        w->staticAngleRad = isFront ? ((isLeft ? 1.0f : -1.0f) * restAngle) : 0.0f;
    }

    /* [rule] the arch stands proud wherever the track pushes the tire outboard of the hull —
     * a narrow body on a wide track produces bolt-on flares, exactly as it does in reality. */
    const float widestTrack = maxf(spec->trackWidthFrontM, spec->trackWidthRearM);
    const float meanTireWidth = 0.5f * (tireWidthF + tireWidthR);
    const float outboard = maxf(0.0f, 0.5f * widestTrack + 0.5f * meanTireWidth - halfW);
    const float archClearance = 0.5f * (spec->rideHeightFrontM + spec->rideHeightRearM) * 0.32f;
    out->archFlareM = archClearance + outboard;

    /* ---- greenhouse ---- */

    /* [rule] the cabin sits where the weight distribution puts it: a rear-biased CG pushes the
     * greenhouse back and lengthens the bonnet, which is what makes a mid-engine car read as
     * mid-engine. Cites cgToFrontM and cgToRearM. Prefer cowl/backlight when they span a
     * sensible cabin. */
    out->cabinCentreXM = 0.55f * (spec->cgToRearM - spec->cgToFrontM);
    out->cabinLengthM  = out->lengthM * (0.34f + 0.18f * (1.0f - l.sport01));
    /* [rule] taller bodies get a wider cabin fraction of the body half-width. */
    const float cabinFrac = lerpf(0.62f, 0.82f, u01(spec->heightOverallM, 1.10f, 2.20f));
    out->cabinHalfWidthM = halfW * cabinFrac;
    out->windscreenXM = out->cabinCentreXM + 0.5f * out->cabinLengthM;
    out->backlightXM  = out->cabinCentreXM - 0.5f * out->cabinLengthM;
    /* Prefer explicit cowl/backlight stations when they form a forward-to-rear glass band. */
    if (spec->cowlXM > spec->backlightXM + 0.40f) {
        out->windscreenXM = spec->cowlXM;
        out->backlightXM = spec->backlightXM;
        out->cabinCentreXM = 0.5f * (spec->cowlXM + spec->backlightXM);
        out->cabinLengthM = spec->cowlXM - spec->backlightXM;
    }

    /* ---- appendages ---- */

    /* [rule] a draggy, sporty car earns a wing; magnitude ramps so a hair of extra drag never
     * snaps one into existence. Cites dragCoefficient and frontalAreaM2 via aero01. */
    const float wing01 = smoothstep(0.45f, 0.85f, 0.6f * l.aero01 + 0.4f * l.sport01);
    if (wing01 > 0.01f) {
        out->wingSpanM  = out->widthM * (0.86f + 0.16f * wing01);
        out->wingChordM = 0.12f + 0.18f * wing01;
        out->wingXM     = tailX + 0.12f;
    }

    /* [rule] front-biased braking plus aero earns a splitter. */
    const float split01 = smoothstep(0.50f, 0.90f, 0.5f * l.sport01 + 0.5f * biasF);
    if (split01 > 0.01f) {
        out->splitterProtrusionM = 0.05f + 0.18f * split01;
        out->splitterWidthM      = out->widthM * (0.90f + 0.12f * split01);
    }

    /* [rule] a stripped car loses its mirrors and gains a cage. */
    out->hasCage    = (l.strip01 > 0.60f);
    out->hasMirrors = (l.strip01 <= 0.85f);
    out->mirrorOffsetM = out->hasMirrors ? (halfW + 0.10f) : 0.0f;

    /* [rule] exhaust count and bore follow peak engine torque. */
    const float peak = peak_engine_torque(spec);
    out->exhaustCount = (peak < 200.0f) ? 1 : (peak < 400.0f) ? 2 : 4;
    out->exhaustBoreM = 0.05f + 0.05f * u01(peak, 100.0f, 700.0f);

    /* ---- palette: arbitrary but stable; excluded from the signature ---- */

    const uint32_t seed = car_visual_colour_seed(spec);
    const float hue = (float)(seed % 360u);
    const float sat = 0.45f + 0.40f * (float)((seed >> 9) % 256u) / 255.0f;
    const float val = 0.55f + 0.35f * (float)((seed >> 17) % 256u) / 255.0f;

    out->body      = hsv_to_color(hue, sat, val, 255);
    out->bodyShade = shade(out->body, 0.78f);
    out->cabin     = shade(out->body, 0.42f);
    out->glass     = (Color){ 58, 68, 84, 255 };
    out->outline   = (Color){ 22, 16, 14, 255 };
    out->tire      = (Color){ 24, 26, 30, 255 };
    out->rim       = (Color){ 150, 155, 165, 255 };
    out->disc      = (Color){ 96, 100, 110, 255 };
    out->accent    = hsv_to_color(hue + 180.0f, sat * 0.85f, clampf(val * 1.15f, 0.0f, 1.0f), 255);
    out->lamp      = (Color){ 255, 240, 200, 255 };
}

/* ------------------------------------------------------------------------- signature --
 *
 * One list generates the enum, the count and the names, so they cannot drift apart. */
#define CAR_SIGNATURE_COMPONENTS(X)                                                          \
    X(length) X(width) X(wheelbase) X(front_overhang) X(rear_overhang)                       \
    X(hull0) X(hull1) X(hull2) X(hull3) X(hull4) X(hull5) X(hull6) X(hull7) X(hull8)         \
    X(cabin_centre_x) X(cabin_length) X(cabin_half_width) X(windscreen_x) X(backlight_x)     \
    X(arch_flare)                                                                            \
    X(tire_diameter_front) X(tire_diameter_rear) X(tire_width_front) X(tire_width_rear)      \
    X(rim_diameter_front) X(rim_diameter_rear) X(disc_diameter_front) X(disc_diameter_rear)  \
    X(track_front) X(track_rear) X(rest_angle_front)                                         \
    X(wing_span) X(wing_chord) X(splitter_protrusion) X(mirror_offset) X(exhaust_bore)       \
    X(spoke_level) X(exhaust_level) X(cage_flag) X(mirror_flag)

#define X_ENUM(name) CAR_SIG_##name,
enum { CAR_SIGNATURE_COMPONENTS(X_ENUM) CAR_SIG_COUNT };
#undef X_ENUM

#define X_NAME(name) #name,
static const char *const kSignatureNames[] = { CAR_SIGNATURE_COMPONENTS(X_NAME) };
#undef X_NAME

/* One discrete level step equals the minimum per-component threshold exactly, so a change of
 * level is always by itself enough to separate two cars. */
#define CAR_SIG_LEVEL_STEP 0.08f

int car_visual_signature_count(void) { return CAR_SIG_COUNT; }

const char *car_visual_signature_component_name(int index)
{
    if (index < 0 || index >= CAR_SIG_COUNT) return "?";
    return kSignatureNames[index];
}

int car_visual_signature(const CarVisual *v, float *out, int cap)
{
    if (v == NULL || out == NULL || cap < CAR_SIG_COUNT) return 0;

    const CarWheelVisual *fl = &v->wheels[WHEEL_FRONT_LEFT];
    const CarWheelVisual *rl = &v->wheels[WHEEL_REAR_LEFT];

    out[CAR_SIG_length]         = v->lengthM;
    out[CAR_SIG_width]          = v->widthM;
    out[CAR_SIG_wheelbase]      = v->wheelbaseM;
    out[CAR_SIG_front_overhang] = v->frontOverhangM;
    out[CAR_SIG_rear_overhang]  = v->rearOverhangM;

    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        out[CAR_SIG_hull0 + i] = v->hull[i].halfWidthM;
    }

    out[CAR_SIG_cabin_centre_x]   = v->cabinCentreXM;
    out[CAR_SIG_cabin_length]     = v->cabinLengthM;
    out[CAR_SIG_cabin_half_width] = v->cabinHalfWidthM;
    out[CAR_SIG_windscreen_x]     = v->windscreenXM;
    out[CAR_SIG_backlight_x]      = v->backlightXM;
    out[CAR_SIG_arch_flare]       = v->archFlareM;

    out[CAR_SIG_tire_diameter_front] = fl->diameterM;
    out[CAR_SIG_tire_diameter_rear]  = rl->diameterM;
    out[CAR_SIG_tire_width_front]    = fl->widthM;
    out[CAR_SIG_tire_width_rear]     = rl->widthM;
    out[CAR_SIG_rim_diameter_front]  = fl->rimDiameterM;
    out[CAR_SIG_rim_diameter_rear]   = rl->rimDiameterM;
    out[CAR_SIG_disc_diameter_front] = fl->discDiameterM;
    out[CAR_SIG_disc_diameter_rear]  = rl->discDiameterM;

    out[CAR_SIG_track_front] = 2.0f * fabsf(fl->centreM.y);
    out[CAR_SIG_track_rear]  = 2.0f * fabsf(rl->centreM.y);

    /* Converted from radians to the arc it sweeps at the tire's edge, so it is in the same
     * "visible metres" currency as everything else. */
    out[CAR_SIG_rest_angle_front] = fabsf(fl->staticAngleRad) * fl->diameterM;

    out[CAR_SIG_wing_span]            = v->wingSpanM;
    out[CAR_SIG_wing_chord]           = v->wingChordM;
    out[CAR_SIG_splitter_protrusion]  = v->splitterProtrusionM;
    out[CAR_SIG_mirror_offset]        = v->mirrorOffsetM;
    out[CAR_SIG_exhaust_bore]         = v->exhaustBoreM;

    out[CAR_SIG_spoke_level]   = (float)fl->spokeCount * CAR_SIG_LEVEL_STEP;
    out[CAR_SIG_exhaust_level] = (float)v->exhaustCount * CAR_SIG_LEVEL_STEP;
    out[CAR_SIG_cage_flag]     = v->hasCage ? CAR_SIG_LEVEL_STEP : 0.0f;
    out[CAR_SIG_mirror_flag]   = v->hasMirrors ? CAR_SIG_LEVEL_STEP : 0.0f;

    return CAR_SIG_COUNT;
}
