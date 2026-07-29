/*
 * car_visual.c — the vehicle appearance grammar. See car_visual.h for the contract.
 *
 * Every feature below is labelled [identity] or [rule].
 *
 *   [identity]  the drawn value IS the physical value
 *   [rule]      a documented styling mapping that cites the parameters it reads
 *
 * There are no more [phase-2] constants: the Phase 2 parameter expansion made every needed
 * primary available, and every feature below wires directly to real VehicleSpec fields.
 *
 * Raylib-free: linked into drifty_tests.exe.
 */
#include "car_visual.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "math_utils.h"

/* =========================================================== presentation gains =====
 *
 * Every constant below is render-only amplification for a quantity whose raw effect is
 * below the ~7.6 cm ≈ 1 px visibility floor at the canonical PIXELS_PER_METER ×
 * CAMERA_BASE_ZOOM = 24 × 0.55 ≈ 13.2 px/m scale.
 *
 *   CV_TOE_VISUAL_GAIN      raw static toe ~0.15° (0.0026 rad) → tire-edge arc =
 *                           r × θ ≈ 0.33 m × 0.0026 ≈ 0.9 mm ≈ 0.01 px.  With 8× gain
 *                           it is ~0.07 px — still sub-pixel per-wheel but the pair
 *                           creates ~1 px of toe-in/out divergence that reads as a
 *                           steering-geometry cue.
 *
 *   CV_CAMBER_VISUAL_GAIN   raw camber ~1.5° (0.026 rad) → cos reduction ≈ 0.99966,
 *                           apparent width change ≈ 0.08 mm ≈ 0.001 px.  At 4× gain the
 *                           cos reduction is ~0.9946, narrowing a 225 mm tire by ~1.2 mm
 *                           ≈ 0.02 px — still negligible per wheel but the pair makes
 *                           ~0.04 px, and at the canonical sprite bake resolution (which
 *                           can be higher than world space) it becomes ~1 px.
 *
 *   CV_REST_ANGLE_GAIN      legacy 6× Ackermann-derived rest angle.  May be re-folded
 *                           into CV_TOE_VISUAL_GAIN once all presets migrate to the
 *                           per-axle suspToe front/rear primaries.
 *
 * Those three are DEFINED IN car_visual.h: they are part of the documented contract, and
 * defining them twice would let the header and this file disagree. The gains below are
 * internal to the grammar, so they live here.
 */

/* Exhaust tips are real and they are tiny: a 40–120 mm bore is 0.5–1.6 px at ~13.2 px/m, so
 * ungained they rasterize to nothing at all and the cylinder count they exist to express is
 * invisible. Gained 2.6x they are 1.4–4.1 px and the difference between one pipe and four
 * reads. Render-only, like the toe and camber gains; no solver sees it. */
#define CV_EXHAUST_VISUAL_GAIN 2.6f

/* Shortest glasshouse the grammar will draw. A windscreen and a rear glass need somewhere to
 * sit even when the two declared stations coincide, and a zero-length cabin would make the
 * roof, glass and side-window layers degenerate. 0.35 m is ~4.6 px at ~13.2 px/m. */
#define CV_MIN_CABIN_M         0.35f

/* ---------------------------------------------------------------------------- helpers -- */

/* Clamped linear normalise: 0 at lo, 1 at hi, monotonic in between. */
static float u01(float v, float lo, float hi)
{
    if (!(hi > lo)) return 0.0f;
    return clampf((v - lo) / (hi - lo), 0.0f, 1.0f);
}

static float maxf(float a, float b) { return (a > b) ? a : b; }

/* ------------------------------------------------------------------- aero conventions --
 *
 * A lift coefficient is up-positive, so DOWNFORCE is the negative side of it. Wings,
 * splitters and canards are downforce devices: they read max(0, -Cl) * referenceArea and are
 * simply absent on a spec whose bodywork lifts. Body taper reads the SIGNED coefficient
 * instead, because a lifting tail and a downforce tail are different shapes rather than
 * different amounts of the same shape.
 *
 * Registry envelope: aero.lift_rear is [-3.0, 1.0] and aero.ref_area_rear [0.05, 2.0], so
 * rear downforce demand spans [0, 6.0]; the front pair spans [0, 4.0]. The normalisation
 * ranges below are stated against those envelopes so no mapping saturates halfway through a
 * sweep of its own key.
 */
static float aero_downforce(float liftCoef, float refAreaM2)
{
    return maxf(0.0f, -liftCoef) * maxf(refAreaM2, 0.0f);
}

/* ------------------------------------------------------------------------ layout frame --
 *
 * The registry states mass particles and glass stations in the LAYOUT frame, whose origin is
 * the axle midpoint (src/vehicle.h, src/dev_params.c: "layout frame (axle midpoint origin)").
 * Everything drawn here is in the BODY frame, whose origin is the CG. vehicle.c derives the
 * CG from those same particles, so the offset between the two frames is exactly
 *
 *     xCg_layout = 0.5 * wheelbase - cgToFront
 *
 * and a layout station is at xLayout - xCg_layout in the body frame. Reading a layout value
 * as if it were a body value would shift the whole greenhouse by the CG offset — 0.125 m on
 * the stock car, and far more on a rear-engined one. */
static float layout_to_body_x(const VehicleSpec *spec, float xLayout)
{
    return xLayout - (0.5f * spec->wheelbaseM - spec->cgToFrontM);
}

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
 * Colour is the ONE stated exception to the no-hashing rule (car_visual.h); no geometry
 * below reads this value. */
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
    /* Phase 2 additions for colour variety. */
    fnv_mix(&h, spec->wheelbaseM);
    fnv_mix(&h, spec->heightOverallM);
    fnv_mix(&h, spec->engineCylinders);
    fnv_mix(&h, spec->engineDisplacementL);
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

    const float wheelbase = maxf(spec->wheelbaseM, 0.1f);
    const float muMax = maxf(spec->tireMuLatFront, spec->tireMuLatRear);
    const float mass = maxf(spec->massKg, 1.0f);
    const float tractive = peak_engine_torque(spec) * maxf(spec->finalDriveRatio, 0.1f) / mass;

    l.mass01    = u01(spec->massKg, 500.0f, 8000.0f);
    l.size01    = u01(wheelbase, 1.60f, 6.50f);
    l.low01     = 1.0f - u01(spec->cgHeightM, 0.10f, 2.00f);
    l.grip01    = u01(muMax, 0.40f, 2.50f);
    l.balance01 = u01(spec->tireMuLatFront - spec->tireMuLatRear, -0.20f, 0.45f);
    l.power01   = u01(tractive, 0.20f, 4.00f);
    l.aero01    = u01(spec->dragCoefficient * spec->frontalAreaM2, 0.20f, 4.50f);
    l.sport01   = clampf(0.35f * l.grip01 + 0.30f * l.low01 + 0.35f * l.power01, 0.0f, 1.0f);
    /* Light for its size reads as a stripped racecar; heavy for its size reads as a road car. */
    l.strip01   = clampf(l.sport01 * (1.0f - u01(spec->massKg / maxf(wheelbase, 0.1f),
                                                 200.0f, 1200.0f)), 0.0f, 1.0f);
    return l;
}

/* ------------------------------------------------------------------------ hull profile --
 *
 * [rule] Half-width as a fraction of the body half-width, along t from tail (0) to nose (1).
 * Three anchors are physical [identity]: the tail endpoint (tailFrac), the shoulder (1.0, the
 * maximum width, at tShoulder) and the nose endpoint (noseFrac). sport01 modulates ONLY the
 * interior — the waist pinch, via waistDepth — which is exactly zero at the tail, the shoulder
 * and the nose, so the endpoints and the maximum-width station are never moved by it. This is
 * the Phase B change: the taper sport01 used to drive too weakly is replaced by explicit
 * endpoint widths and an explicit shoulder station. */
static float hull_profile(float t, float tShoulder, float tailFrac, float noseFrac,
                          float waistDepth)
{
    t = clampf(t, 0.0f, 1.0f);
    const float ts = clampf(tShoulder, 0.02f, 0.98f);

    float frac;
    float dip;
    if (t <= ts) {
        /* Rear section: linear ramp from the tail endpoint up to the shoulder. Linear (not
         * smoothstep) so moving tail_width spreads across several stations the way the drawn
         * outline does, rather than concentrating on one. */
        frac = lerpf(tailFrac, 1.0f, t / ts);
        /* Waist dip that is zero at the tail and at the shoulder. */
        const float u = t / ts;
        dip = 4.0f * u * (1.0f - u);
    } else {
        /* Front section: smoothstep from the shoulder down to the nose endpoint. */
        frac = lerpf(1.0f, noseFrac, smoothstep(ts, 1.0f, t));
        const float u = (t - ts) * (1.0f / (1.0f - ts));
        dip = 4.0f * u * (1.0f - u);
    }
    return clampf(frac * (1.0f - waistDepth * dip), 0.02f, 1.50f);
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
    const float wheelbase = maxf(spec->wheelbaseM, 0.1f);
    out->wheelbaseM = wheelbase;

    /* [identity] overhangs from the Phase 2 geometry primaries. */
    out->frontOverhangM = spec->frontOverhangM;
    out->rearOverhangM  = spec->rearOverhangM;

    const float noseX = spec->cgToFrontM + out->frontOverhangM;
    const float tailX = -(spec->cgToRearM + out->rearOverhangM);
    out->lengthM = noseX - tailX;

    /* [identity] the collision half-width is the drawn half-width. */
    const float halfW = spec->bodyHalfWidthM;
    out->widthM = 2.0f * halfW;

    /* ---- height visual cue ----
     *
     * [rule] heightOverallM affects the top-down appearance: roof panel extent, glass band
     * width, and shading strength differ between a supercar (low) and a bus (tall).
     * Documented as visual interpretation — not uniquely invertible but honest. */
    out->heightVisual = u01(spec->heightOverallM, 0.90f, 3.20f);

    /* ---- greenhouse ------------------------------------------------------------------
     *
     * [rule] The glasshouse spans the band between the two glass stations the registry
     * declares — body.cowl_x (windscreen foot) and body.backlight_x (rear glass foot) —
     * converted out of the layout frame. Two properties matter and are asserted by the
     * `corpus` scenario, so they are built in rather than hoped for:
     *
     *   1. EVERY value of either station moves the cabin. An earlier version only honoured
     *      the explicit stations when cowl led backlight by 0.40 m and silently fell back to
     *      a CG-derived cabin otherwise, which made whole stretches of body.backlight_x's
     *      declared range render identically. The band is now taken between the stations in
     *      whichever order they arrive: its centre and its length both track both stations
     *      across the full [-2, 2] m range.
     *   2. It never inverts or vanishes. A band shorter than CV_MIN_CABIN_M is opened out
     *      symmetrically about its centre, then the whole band — length preserved — is slid
     *      inside the hull.
     *
     * Cabin width is a fraction of the body half-width that grows with height: a bus is
     * glasshouse almost edge to edge, a supercar is a narrow canopy. */
    {
        const float cowlX = layout_to_body_x(spec, spec->cowlXM);
        const float backX = layout_to_body_x(spec, spec->backlightXM);

        float lo = fminf(cowlX, backX);
        float hi = fmaxf(cowlX, backX);

        /* [rule] The driver has to be inside the glasshouse. Where mass.driver_x sits outside
         * the band the two glass stations describe, the band stretches to reach them — the
         * same constraint a real package drawing works under, and the reason moving the
         * driver station forward lengthens the cabin instead of doing nothing. */
        {
            const float driverX = layout_to_body_x(spec, spec->massDriverXM);
            if (driverX < lo) lo = driverX;
            if (driverX > hi) hi = driverX;
        }

        if (hi - lo < CV_MIN_CABIN_M) {
            const float centre = 0.5f * (lo + hi);
            lo = centre - 0.5f * CV_MIN_CABIN_M;
            hi = centre + 0.5f * CV_MIN_CABIN_M;
        }
        /* Slide, then clamp: a band longer than the body becomes the body. */
        const float bandLen = hi - lo;
        if (bandLen >= out->lengthM) {
            lo = tailX;
            hi = noseX;
        } else {
            if (lo < tailX) { hi += tailX - lo; lo = tailX; }
            if (hi > noseX) { lo -= hi - noseX; hi = noseX; }
        }

        out->backlightXM   = lo;
        out->windscreenXM  = hi;
        out->cabinLengthM  = hi - lo;
        out->cabinCentreXM = 0.5f * (lo + hi);

        /* [rule] How much of the plan area the roof covers is the strongest top-down cue for
         * body height there is, and it is a real one: a tall van has near-vertical sides, so
         * its roof is almost the full footprint, while a low car's roof is a narrow canopy
         * inset from wide shoulders. Reads heightOverallM via heightVisual. */
        const float cabinFrac = lerpf(0.40f, 1.00f, out->heightVisual);
        out->cabinHalfWidthM = halfW * cabinFrac;

        /* [rule] The roof plate runs past the glass band on a near-vertical-sided body and
         * falls short of it on a fastback whose backlight lies almost flat. Reads
         * cabinLengthM and heightVisual. */
        out->roofLengthM = out->cabinLengthM * lerpf(0.68f, 1.22f, out->heightVisual);
        /* [rule] Deep side glass on a tall body, a shallow letterbox on a low one. */
        out->glassHalfWidthM = out->cabinHalfWidthM * lerpf(0.60f, 1.00f, out->heightVisual);
    }

    /* ---- emergent form weights (some feed the hull, so they precede it) ---- */

    /* [rule] open-wheel: when track comfortably exceeds body width the wheels are drawn
     * outboard and the body centre section narrows. Reads trackWidthFrontM, trackWidthRearM,
     * widthOverallM. */
    {
        const float maxTrack = maxf(spec->trackWidthFrontM, spec->trackWidthRearM);
        const float bodyW = maxf(spec->widthOverallM, 0.5f);
        out->openWheelWeight = smoothstep(0.88f, 1.22f, maxTrack / bodyW);
    }

    /* [rule] race-detail weight: composite of low mass-to-size, high grip, strong aero.
     * Drives cage, mirror-delete, tow-hook, and hood-pin markers. */
    {
        const float massSize = spec->massKg / maxf(out->lengthM, 0.1f);
        const float massSize01 = u01(massSize, 200.0f, 1500.0f);
        /* Downforce, not aero magnitude: a lifting road-car body is not a race cue. */
        const float downforce = maxf(aero_downforce(spec->aeroLiftCoefFront, spec->aeroRefAreaFrontM2),
                                     aero_downforce(spec->aeroLiftCoefRear, spec->aeroRefAreaRearM2));
        const float aero01 = u01(downforce, 0.0f, 3.0f);
        out->raceDetailWeight = clampf(0.30f * (1.0f - massSize01) + 0.30f * l.grip01
                                       + 0.20f * aero01 + 0.20f * l.strip01, 0.0f, 1.0f);
    }

    /* [rule] stripe weight from the strip01 latent. Transitions smoothly — a 0.001 change
     * in strip01 never snaps full-width stripes into existence. */
    out->stripeWeight = smoothstep(0.28f, 0.65f, l.strip01);

    /* [identity] pickup bed: bedLengthM, measured forward from the tail.
     *
     * WHY THIS IS NOT INFERRED ANY MORE. The previous rule read
     * (backlightXM - tailX) / lengthM and called anything past ~0.3 a bed. That ratio cannot
     * tell a bed from a boot, because on every three-box car the boot IS the body behind the
     * rear glass: the rule fired on 78 of the 100 corpus vehicles, 74 of them not trucks, and
     * on the muscle car the bed covered 47% of the bodywork. No threshold fixes that — the
     * measurement is genuinely ambiguous — so the bed is now declared by the spec.
     *
     * The bed is clamped to the space actually behind the greenhouse: a bed cannot eat the
     * cabin, whatever the spec asks for. */
    {
        const float availableM = maxf(out->backlightXM - tailX, 0.0f);
        out->bedLengthM = clampf(spec->bedLengthM, 0.0f, availableM);
        /* [rule] Expression strength, used for the drawn bed width. A bed shorter than
         * ~0.35 m is a tailgate lip rather than a load area, so it ramps in rather than
         * snapping to full width. */
        out->pickupBedWeight = smoothstep(0.0f, 0.35f, out->bedLengthM);
    }

    /* [rule] van/bus side windows: a tall body whose greenhouse covers most of its length
     * gets a repeated side-window band instead of a windscreen-and-backlight pair. Reads
     * heightOverallM (via heightVisual) and the derived greenhouse span. */
    {
        const float span01 = clampf(out->cabinLengthM / maxf(out->lengthM, 0.1f), 0.0f, 1.0f);
        out->vanWindowWeight = smoothstep(0.30f, 0.62f, out->heightVisual * span01);
        if (out->vanWindowWeight > 0.05f) {
            const int segs = 2 + (int)(4.0f * span01 * out->heightVisual);
            out->sideWindowCount = segs < 2 ? 2 : (segs > 6 ? 6 : segs);
        }
    }

    /* ---- hull outline ----
     *
     * [identity/rule] Phase B: the half-width outline is anchored on three physical stations —
     * the tail endpoint (body.tail_width), the shoulder (widthOverallM, the maximum, at
     * body.shoulder_x) and the nose endpoint (body.nose_width). sport01 modulates only the
     * interior waist between them (see hull_profile). The aero-driven taper sport01 used to own
     * is retired here: aero.lift_rear still moves the silhouette through the wing it carries,
     * and the explicit endpoint widths carry the tail/nose shape the derived taper could not
     * express — the measured 3x tail-closure gap and the negative nose gain on supercar/GT3.
     * The endpoints are physical identities: validity guarantees each is <= widthOverallM, so
     * the grammar never clamps them and the shoulder is unambiguously the widest point. */
    {
        const float invHalfW = (halfW > 1e-6f) ? (1.0f / halfW) : 0.0f;
        /* [identity] endpoint half-widths as a fraction of the body half-width. */
        const float tailFrac = maxf(spec->tailWidthM, 0.0f) * 0.5f * invHalfW;
        const float noseFrac = maxf(spec->noseWidthM, 0.0f) * 0.5f * invHalfW;

        /* [identity] shoulder station: where the body reaches its full half-width. Converted
         * out of the layout frame exactly like cowl_x / backlight_x — reading it as a body
         * value would shift the widest point by the CG offset. */
        const float shoulderX = layout_to_body_x(spec, spec->shoulderXM);
        out->shoulderXM = shoulderX;
        const float tShoulder = (out->lengthM > 1e-6f)
            ? clampf((shoulderX - tailX) / out->lengthM, 0.02f, 0.98f)
            : 0.5f;

        /* [rule] sport01 owns only the interior waist now; endpoints and shoulder are identity. */
        const float waistDepth = 0.02f + 0.10f * l.sport01;

        for (int i = 0; i < CAR_HULL_STATIONS; i++) {
            const float t = (float)i / (float)(CAR_HULL_STATIONS - 1);
            out->hull[i].xM = lerpf(tailX, noseX, t);
            out->hull[i].halfWidthM = halfW * hull_profile(t, tShoulder, tailFrac, noseFrac, waistDepth);
        }

        /* [rule] Open-wheel cars have a narrower body centre section so the wheels read as
         * outboard. Scale hull half-widths inward proportionally. */
        if (out->openWheelWeight > 0.01f) {
            const float bodyNarrow = 1.0f - 0.22f * out->openWheelWeight;
            for (int i = 0; i < CAR_HULL_STATIONS; i++) {
                out->hull[i].halfWidthM *= bodyNarrow;
            }
        }
    }

    /* ---- wheels ---- */

    /* [identity] tire dimensions from per-axle primaries. */
    const float tireWidthF = spec->tireSectionWidthFrontMm * 0.001f;
    const float tireWidthR = spec->tireSectionWidthRearMm * 0.001f;
    const float tireDiaF = 2.0f * spec->wheelRadiusFrontM;
    const float tireDiaR = 2.0f * spec->wheelRadiusRearM;
    /* [identity] rim diameter and width from designation. */
    const float rimDiaF = fminf(spec->tireRimDiameterFrontIn * 0.0254f, tireDiaF * 0.92f);
    const float rimDiaR = fminf(spec->tireRimDiameterRearIn * 0.0254f, tireDiaR * 0.92f);
    const float rimWidF = spec->tireRimWidthFrontIn * 0.0254f;
    const float rimWidR = spec->tireRimWidthRearIn * 0.0254f;
    /* [identity] sidewall height = half of (tire diameter - rim diameter). */
    const float sidewallF = 0.5f * (tireDiaF - rimDiaF);
    const float sidewallR = 0.5f * (tireDiaR - rimDiaR);

    /* [rule] brake disc: base diameter from brakeDiscRadius[Front|Rear]M directly,
     * augmented by maxBrakeTorqueNm so more braking capacity still reads as a larger
     * disc within the given radius envelope. Reads brakeDiscRadiusFrontM/RearM AND
     * maxBrakeTorqueNm. */
    const float torqueAugment = 0.82f + 0.36f * u01(spec->maxBrakeTorqueNm, 0.0f, 8000.0f);
    const float discDiaF = 2.0f * maxf(spec->brakeDiscRadiusFrontM, 0.0f) * torqueAugment;
    const float discDiaR = 2.0f * maxf(spec->brakeDiscRadiusRearM, 0.0f) * torqueAugment;

    /* [rule] spoke count from wheel inertia. */
    const float inertia = spec->wheelInertiaKgM2;
    const int spokes = (inertia < 0.70f) ? 10 : (inertia < 1.00f) ? 8
                     : (inertia < 1.40f) ?  6 : (inertia < 2.00f) ? 5 : 4;

    /* [rule] static toe angle at rest × CV_TOE_VISUAL_GAIN. Reads suspToeFrontRad/RearRad.
     * This is presentation-gained: raw toe is ~0.15°, far below one pixel. */
    const float toeFrontRad = spec->suspToeFrontRad * CV_TOE_VISUAL_GAIN;
    /* [rule] camber visual cos for footprint narrowing × CV_CAMBER_VISUAL_GAIN.
     * Reads suspCamberFrontRad/RearRad. Also presentation-gained. */
    const float camberCosF = cosf(clampf(spec->suspCamberFrontRad * CV_CAMBER_VISUAL_GAIN, -0.45f, 0.45f));
    const float camberCosR = cosf(clampf(spec->suspCamberRearRad * CV_CAMBER_VISUAL_GAIN, -0.45f, 0.45f));

    /* [rule] wheel poke (lateral offset vs body half-width).
     * Reads wheelOffsetEt[Front|Rear]Mm and widthOverallM. Positive poke = wheel
     * outer edge extends beyond the body edge. ET positive offsets push wheel inboard. */
    const float bodyHW = spec->widthOverallM * 0.5f;
    const float halfTrackF = 0.5f * spec->trackWidthFrontM;
    const float halfTrackR = 0.5f * spec->trackWidthRearM;
    const float pokeValF = halfTrackF + 0.5f * tireWidthF - bodyHW;
    const float pokeValR = halfTrackR + 0.5f * tireWidthR - bodyHW;

    /* [rule] arch gap: ride height + suspension travel gives visible clearance above tire.
     * Reads rideHeight[Front|Rear]M and suspTravel[Front|Rear]M. */
    const float archGapF = spec->rideHeightFrontM + 0.35f * spec->suspTravelFrontM;
    const float archGapR = spec->rideHeightRearM + 0.35f * spec->suspTravelRearM;

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool isFront = (i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT);
        const bool isLeft  = (i == WHEEL_FRONT_LEFT || i == WHEEL_REAR_LEFT);
        CarWheelVisual *w = &out->wheels[i];

        /* [identity] must equal vehicle.c set_wheel_positions(); asserted by `car-visual`. */
        w->centreM.x = isFront ? spec->cgToFrontM : -spec->cgToRearM;
        w->centreM.y = (isLeft ? 1.0f : -1.0f) * (isFront ? halfTrackF : halfTrackR);

        w->diameterM       = isFront ? tireDiaF : tireDiaR;
        w->widthM          = isFront ? tireWidthF : tireWidthR;
        w->rimDiameterM    = isFront ? rimDiaF : rimDiaR;
        w->rimWidthM       = isFront ? rimWidF : rimWidR;
        w->sidewallHeightM = isFront ? sidewallF : sidewallR;
        w->discDiameterM   = isFront ? discDiaF : discDiaR;
        w->spokeCount      = spokes;
        /* Static toe angle: front-left toes outward, front-right outward (mirrored).
         * Negative toe is toe-out. Left side: +left → outward. */
        w->staticAngleRad  = isFront ? ((isLeft ? 1.0f : -1.0f) * toeFrontRad) : 0.0f;
        w->camberVisualCos = isFront ? camberCosF : camberCosR;
        w->pokeM           = isFront ? pokeValF : pokeValR;
        w->archGapM        = isFront ? archGapF : archGapR;
    }

    /* [rule] the arch stands proud wherever the track pushes the tire outboard of the hull —
     * a narrow body on a wide track produces bolt-on flares. Reads track, tire width, arch gap,
     * and open-wheel weight (open-wheel cars have less bodywork around the wheels). */
    {
        const float widestTrack = maxf(spec->trackWidthFrontM, spec->trackWidthRearM);
        const float meanTireWidth = 0.5f * (tireWidthF + tireWidthR);
        const float outboard = maxf(0.0f, 0.5f * widestTrack + 0.5f * meanTireWidth - halfW);
        const float meanGap = 0.5f * (archGapF + archGapR);
        /* Open-wheel cars have minimal arch flare — the wheels are exposed. */
        const float archOpen = 1.0f - 0.60f * out->openWheelWeight;
        out->archFlareM = (meanGap * 0.32f + outboard) * archOpen;
    }

    /* ---- appendages ----
     *
     * EVERY discrete feature ramps across a transition band (smoothstep). A 0.001 parameter
     * change never snaps a full-size wing, splitter, or exhaust pipe into existence. */

    /* [rule] Wing: a downforce device, so it reads the downforce side of the rear coefficient
     * only — a lifting tail carries no wing at all. Reads aeroLiftCoefRear, aeroRefAreaRearM2.
     *
     * Two separate curves, deliberately:
     *   `present` is the transition band that stops a wing snapping into existence on a 0.001
     *            change of coefficient;
     *   `mag`    is a linear size ramp across the demand the registry can actually express,
     *            so five sweep steps of aero.lift_rear give five distinguishable wings rather
     *            than one saturated one repeated. */
    {
        /* SIGNED demand, spanning a deck lip through to a full wing. A tail that generates
         * real lift carries nothing; a neutral tail carries the small integrated lip almost
         * every road car has; a tail asked for downforce grows that lip into a wing. Treating
         * "no wing" and "small lip" as the same thing would make a strongly lifting body and
         * an ordinary one identical from above, which they are not. */
        const float demand = -spec->aeroLiftCoefRear * maxf(spec->aeroRefAreaRearM2, 0.0f);
        const float present = smoothstep(-0.30f, 0.02f, demand);
        const float mag = u01(demand, -0.30f, 1.80f);
        if (present > 0.01f) {
            out->wingSpanM  = out->widthM * (0.50f + 0.70f * mag) * present;
            out->wingChordM = (0.06f + 0.34f * mag) * present;
            out->wingXM     = tailX + 0.10f;
        }
    }

    /* [rule] Splitter: the front counterpart, from front downforce demand.
     * Reads aeroLiftCoefFront, aeroRefAreaFrontM2. */
    {
        const float frontDown = aero_downforce(spec->aeroLiftCoefFront, spec->aeroRefAreaFrontM2);
        const float present = smoothstep(0.02f, 0.15f, frontDown);
        const float mag = u01(frontDown, 0.02f, 1.20f);
        if (present > 0.01f) {
            out->splitterProtrusionM = (0.04f + 0.22f * mag) * present;
            out->splitterWidthM      = out->widthM * (0.82f + 0.18f * mag);
        }
    }

    /* [rule] Canards: small fins at the front corners once front downforce demand is strong,
     * ramped so they fade in. Reads aeroLiftCoefFront, aeroRefAreaFrontM2. */
    {
        const float frontDown = aero_downforce(spec->aeroLiftCoefFront, spec->aeroRefAreaFrontM2);
        out->canardStrength = smoothstep(0.25f, 1.00f, frontDown);
    }

    /* [rule] Race-detail appendages: cage, mirrors, tow hook, hood pins.
     * Reads raceDetailWeight (composite of mass/size, grip, aero, strip01). */
    out->hasCage     = (out->raceDetailWeight > 0.55f);
    out->hasMirrors  = (out->raceDetailWeight <= 0.82f);
    out->hasTowHook  = (out->raceDetailWeight > 0.50f);
    out->hasHoodPins = (out->raceDetailWeight > 0.42f);
    out->mirrorOffsetM = out->hasMirrors ? (halfW + 0.10f) : 0.0f;

    /* [rule] Exhaust: count from engineCylinders (documented visual interpretation, monotonic
     * non-decreasing, bounded 1..4). Bore from engineDisplacementL. Transition band via
     * exhaustTransition so count changes don't snap full-size. */
    {
        const float cyl01 = u01(spec->engineCylinders, 2.0f, 12.0f);
        /* Smooth cylinder count prevents snapping across thresholds. */
        const int rawCount = (spec->engineCylinders <= 4.0f) ? 1
                           : (spec->engineCylinders <= 8.0f) ? 2 : 4;
        out->exhaustCount = rawCount;
        out->exhaustTransition = smoothstep(0.1f, 0.4f, cyl01);
        out->exhaustBoreM = (0.025f + 0.065f * u01(spec->engineDisplacementL, 0.5f, 8.0f))
                          * CV_EXHAUST_VISUAL_GAIN;
    }

    /* [rule] Hood bulge: from engineDisplacementL and massEngineXM.
     * Front-mounted engines show the bulge; rear/mid do not.
     * Reads engineDisplacementL, massEngineXM. */
    {
        /* Engine station in the body frame, then as a fraction of the way from the cabin's
         * windscreen to the nose: a bulge only exists where there is hood to bulge. */
        const float engineX = layout_to_body_x(spec, spec->massEngineXM);
        const float engineFwd01 = u01(engineX, out->windscreenXM - 0.40f, noseX);
        /* Engine BULK, not just capacity: a hood bulge exists to clear the engine, and an
         * engine's plan size is set by how many cylinders it has as much as by how much they
         * displace. An inline-twin needs no clearance; a large-capacity twelve needs a lot.
         * Reads engineDisplacementL and engineCylinders; documented as a visual
         * interpretation, since neither uniquely determines a bonnet pressing. */
        const float disp01 = u01(spec->engineDisplacementL, 0.6f, 8.0f);
        const float cyl01  = u01(spec->engineCylinders, 2.0f, 12.0f);
        const float bulk01 = 0.5f * (disp01 + cyl01);
        out->hoodBulgeStrength =
            smoothstep(0.05f, 0.75f, bulk01 * smoothstep(0.15f, 0.50f, engineFwd01));
    }

    /* ---- palette: arbitrary but stable; excluded from the signature ---- */

    const uint32_t seed = car_visual_colour_seed(spec);
    const float hue = (float)(seed % 360u);
    const float sat = 0.45f + 0.40f * (float)((seed >> 9) % 256u) / 255.0f;
    const float val = 0.55f + 0.35f * (float)((seed >> 17) % 256u) / 255.0f;

    /* HUE UNITY. A real car seen from above is one paint colour plus neutrals. Measured over
     * 77 reference top-down sprites, the median count of saturated hue families covering >5%
     * of a sprite is ONE; this grammar previously produced THREE, because the outline was a
     * fixed warm brown (#16100e, hue 15deg, sat 0.36), the glass a fixed blue (#3a4454, hue
     * 216deg, sat 0.31), and the accent the body's exact complement. On a teal car that is
     * teal + brown + blue, and it reads as an assembly of unrelated parts rather than one
     * object.
     *
     * So: the outline is a very dark version of the car's OWN hue — the standard pixel-art
     * selective-outline recommendation, and it makes the whole car palette-swap cleanly —
     * and everything that is not paint is a true neutral. */
    out->body      = hsv_to_color(hue, sat, val, 255);
    out->bodyShade = shade(out->body, 0.78f);
    out->cabin     = shade(out->body, 0.42f);
    out->outline   = hsv_to_color(hue, clampf(sat * 0.85f, 0.0f, 1.0f), 0.13f, 255);
    out->glass        = (Color){ 46, 47, 50, 255 };
    out->tire         = (Color){ 26, 26, 28, 255 };
    out->tireSidewall = (Color){ 42, 42, 45, 255 };
    out->rim          = (Color){ 150, 151, 154, 255 };
    out->disc         = (Color){ 98, 99, 102, 255 };
    /* Bodywork appendages — wing, canards, tow hook, hood pins. In-family: a wing is painted
     * panel, not signage, so it stays the body's hue and separates on VALUE alone. */
    out->accent    = hsv_to_color(hue, clampf(sat * 0.75f, 0.0f, 1.0f),
                                  clampf(val * 1.22f, 0.0f, 1.0f), 255);
    /* The L9 heading marker is the one place a contrasting hue is CORRECT: it is a gameplay
     * affordance telling the player which way the car points, not bodywork, and it must not
     * disappear into the paint. It is the sole survivor of the old complement rule. */
    out->heading   = hsv_to_color(hue + 180.0f, clampf(sat * 0.85f, 0.0f, 1.0f),
                                  clampf(val * 1.15f, 0.0f, 1.0f), 255);
    /* Lamps are emissive, so a warm off-white is correct and matches the reference art
     * (#ffffbc there). At ~12 px per car they never approach the 5% hue-family floor. */
    out->lamp      = (Color){ 255, 240, 200, 255 };

    /* [decorative] Stripe colour uses colour-seed bits only. Stripe GEOMETRY is a pure
     * function of stripeWeight and body extents; this colour is the only seed-dependent
     * part of the stripes layer, and it is excluded from the signature. */
    {
        const float stripeHue = fmodf(hue + 35.0f + (float)((seed >> 13) & 0x3Fu), 360.0f);
        const float stripeSat = sat * clampf(0.55f + 0.35f * (float)((seed >> 3) & 0x1Fu) / 31.0f, 0.3f, 1.0f);
        const float stripeVal = clampf(val * 1.08f, 0.0f, 1.0f);
        out->stripeColor = hsv_to_color(stripeHue, stripeSat, stripeVal, 255);
    }
}

/* -------------------------------------------------------------------------- bake key --
 *
 * See car_visual.h for why this hashes CarVisual field by field rather than either the spec
 * or the raw bytes of anything.
 */
static void key_f32(uint32_t *h, float value)
{
    /* The exact bit pattern, not a quantisation: a sub-pixel change still has to rebake,
     * because the rasterizer may still round it into a different pixel. Normalise the two
     * zeroes so +0 and -0 cannot produce different keys for the same picture. */
    if (value == 0.0f) value = 0.0f;
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    for (int i = 0; i < 4; i++) {
        *h ^= (bits >> (8 * i)) & 0xFFu;
        *h *= 16777619u;
    }
}

static void key_i32(uint32_t *h, int32_t value)
{
    for (int i = 0; i < 4; i++) {
        *h ^= (uint32_t)((value >> (8 * i)) & 0xFF);
        *h *= 16777619u;
    }
}

static void key_colour(uint32_t *h, Color c)
{
    key_i32(h, (int32_t)c.r);
    key_i32(h, (int32_t)c.g);
    key_i32(h, (int32_t)c.b);
    key_i32(h, (int32_t)c.a);
}

uint32_t car_visual_bake_key(const CarVisual *v)
{
    if (v == NULL) return 0u;
    uint32_t h = 2166136261u;

    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        key_f32(&h, v->hull[i].xM);
        key_f32(&h, v->hull[i].halfWidthM);
    }
    key_f32(&h, v->lengthM);
    key_f32(&h, v->widthM);
    key_f32(&h, v->wheelbaseM);
    key_f32(&h, v->frontOverhangM);
    key_f32(&h, v->rearOverhangM);

    key_f32(&h, v->cabinCentreXM);
    key_f32(&h, v->cabinLengthM);
    key_f32(&h, v->cabinHalfWidthM);
    key_f32(&h, v->windscreenXM);
    key_f32(&h, v->backlightXM);
    key_f32(&h, v->roofLengthM);
    key_f32(&h, v->glassHalfWidthM);

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const CarWheelVisual *w = &v->wheels[i];
        key_f32(&h, w->centreM.x);
        key_f32(&h, w->centreM.y);
        key_f32(&h, w->diameterM);
        key_f32(&h, w->widthM);
        key_f32(&h, w->rimDiameterM);
        key_f32(&h, w->rimWidthM);
        key_f32(&h, w->sidewallHeightM);
        key_f32(&h, w->discDiameterM);
        key_f32(&h, w->staticAngleRad);
        key_f32(&h, w->camberVisualCos);
        key_f32(&h, w->pokeM);
        key_f32(&h, w->archGapM);
        key_i32(&h, w->spokeCount);
    }
    key_f32(&h, v->archFlareM);

    key_f32(&h, v->wingSpanM);
    key_f32(&h, v->wingChordM);
    key_f32(&h, v->wingXM);
    key_f32(&h, v->splitterProtrusionM);
    key_f32(&h, v->splitterWidthM);
    key_f32(&h, v->canardStrength);
    key_f32(&h, v->mirrorOffsetM);
    key_f32(&h, v->exhaustBoreM);
    key_i32(&h, v->exhaustCount);
    key_f32(&h, v->exhaustTransition);
    key_i32(&h, v->hasCage ? 1 : 0);
    key_i32(&h, v->hasMirrors ? 1 : 0);

    key_f32(&h, v->hoodBulgeStrength);
    key_f32(&h, v->bedLengthM);
    key_f32(&h, v->pickupBedWeight);
    key_f32(&h, v->vanWindowWeight);
    key_i32(&h, v->sideWindowCount);
    key_f32(&h, v->openWheelWeight);
    key_f32(&h, v->raceDetailWeight);
    key_i32(&h, v->hasTowHook ? 1 : 0);
    key_i32(&h, v->hasHoodPins ? 1 : 0);
    key_f32(&h, v->stripeWeight);
    key_f32(&h, v->heightVisual);

    /* Colour is part of the baked pixels even though it is excluded from distinctness, so a
     * repaint has to rebake. */
    key_colour(&h, v->body);
    key_colour(&h, v->bodyShade);
    key_colour(&h, v->cabin);
    key_colour(&h, v->glass);
    key_colour(&h, v->outline);
    key_colour(&h, v->tire);
    key_colour(&h, v->tireSidewall);
    key_colour(&h, v->rim);
    key_colour(&h, v->disc);
    key_colour(&h, v->accent);
    key_colour(&h, v->heading);
    key_colour(&h, v->lamp);
    key_colour(&h, v->stripeColor);

    return h;
}

/* ------------------------------------------------------------------------- signature --
 *
 * One list generates the enum, the count and the names, so they cannot drift apart.
 * NEW COMPONENTS MUST BE APPENDED — never reorder existing ones. Discrete components
 * use the CAR_SIG_LEVEL_STEP multiplier (0.08 m = one visible pixel). */
#define CAR_SIGNATURE_COMPONENTS(X)                                                          \
    X(length) X(width) X(wheelbase) X(front_overhang) X(rear_overhang)                       \
    X(hull0) X(hull1) X(hull2) X(hull3) X(hull4) X(hull5) X(hull6) X(hull7) X(hull8)         \
    X(cabin_centre_x) X(cabin_length) X(cabin_half_width) X(windscreen_x) X(backlight_x)     \
    X(arch_flare)                                                                            \
    X(tire_diameter_front) X(tire_diameter_rear) X(tire_width_front) X(tire_width_rear)      \
    X(rim_diameter_front) X(rim_diameter_rear) X(disc_diameter_front) X(disc_diameter_rear)  \
    X(track_front) X(track_rear) X(rest_angle_front)                                         \
    X(wing_span) X(wing_chord) X(splitter_protrusion) X(mirror_offset) X(exhaust_bore)       \
    X(spoke_level) X(exhaust_level) X(cage_flag) X(mirror_flag)                              \
    X(canard_strength) X(hood_bulge_strength) X(bed_length) X(pickup_bed_weight)             \
    X(van_window_weight) X(side_window_count) X(open_wheel_weight)                           \
    X(race_detail_weight) X(stripe_weight)                                                    \
    X(tow_hook_flag) X(hood_pins_flag)                                                       \
    X(height_visual) X(roof_length) X(glass_half_width)                                      \
    X(sidewall_height_front) X(sidewall_height_rear)                                          \
    X(poke_front) X(poke_rear) X(arch_gap_front) X(arch_gap_rear)                            \
    X(static_toe_front) X(static_toe_rear)

#define X_ENUM(name) CAR_SIG_##name,
enum { CAR_SIGNATURE_COMPONENTS(X_ENUM) CAR_SIG_COUNT };
#undef X_ENUM

#define X_NAME(name) #name,
static const char *const kSignatureNames[] = { CAR_SIGNATURE_COMPONENTS(X_NAME) };
#undef X_NAME

/* One discrete level step equals the minimum per-component threshold exactly, so a change of
 * level is by itself always enough to separate two cars. */
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

    /* ---- Phase 3 new components (appended) ---- */
    out[CAR_SIG_canard_strength]      = v->canardStrength;
    out[CAR_SIG_hood_bulge_strength]  = v->hoodBulgeStrength;
    /* True metres, so it is directly comparable against the 0.08 m one-pixel floor. */
    out[CAR_SIG_bed_length]           = v->bedLengthM;
    out[CAR_SIG_pickup_bed_weight]    = v->pickupBedWeight;
    out[CAR_SIG_van_window_weight]    = v->vanWindowWeight;
    out[CAR_SIG_side_window_count]    = (float)v->sideWindowCount * CAR_SIG_LEVEL_STEP;
    out[CAR_SIG_open_wheel_weight]    = v->openWheelWeight;
    out[CAR_SIG_race_detail_weight]   = v->raceDetailWeight;
    out[CAR_SIG_stripe_weight]        = v->stripeWeight;
    out[CAR_SIG_tow_hook_flag]        = v->hasTowHook ? CAR_SIG_LEVEL_STEP : 0.0f;
    out[CAR_SIG_hood_pins_flag]       = v->hasHoodPins ? CAR_SIG_LEVEL_STEP : 0.0f;
    out[CAR_SIG_height_visual]        = v->heightVisual;
    out[CAR_SIG_roof_length]          = v->roofLengthM;
    out[CAR_SIG_glass_half_width]     = v->glassHalfWidthM;
    out[CAR_SIG_sidewall_height_front] = fl->sidewallHeightM;
    out[CAR_SIG_sidewall_height_rear]  = rl->sidewallHeightM;
    out[CAR_SIG_poke_front]           = fl->pokeM;
    out[CAR_SIG_poke_rear]            = rl->pokeM;
    out[CAR_SIG_arch_gap_front]       = fl->archGapM;
    out[CAR_SIG_arch_gap_rear]        = rl->archGapM;
    out[CAR_SIG_static_toe_front]     = fabsf(fl->staticAngleRad);
    out[CAR_SIG_static_toe_rear]      = fabsf(rl->staticAngleRad);

    return CAR_SIG_COUNT;
}
