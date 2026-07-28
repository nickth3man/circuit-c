/*
 * car_visual.h — the vehicle appearance grammar: VehicleSpec -> CarVisual.
 *
 * THE ONE RULE. A car's appearance is a pure, total, deterministic function of its physics
 * parameters. This translation unit is the only place in the project where a styling decision
 * may live. src/render.c and the contact-sheet writer are dumb consumers of CarVisual; neither
 * may invent geometry of its own.
 *
 * WHY IT LIVES HERE AND NOT IN render.c. render.c stubs its whole draw path out under
 * DRIFTY_HEADLESS (src/render.c), so anything decided there is unreachable from
 * drifty_tests.exe and therefore unverifiable. car_visual.c is raylib-free and sits in
 * SHARED_SRCS, so the grammar is linked into the headless test binary and is covered by the
 * `car-visual` and `corpus` scenarios.
 *
 * RAYLIB-FREE. This header includes raylib.h for the Vector2 and Color *types* only. Neither
 * it nor car_visual.c calls a raylib function, exactly like src/units.h. That is what keeps
 * drifty_tests linkable without a window.
 *
 * TAXONOMY OF A FEATURE. Every field below is produced by exactly one of three kinds of rule,
 * and car_visual.c labels each one:
 *
 *   [identity]  The drawn value IS the physical value. Wheel centres, wheelbase, tire
 *               diameter, body width. These may never be fudged: the `car-visual` scenario
 *               asserts the wheel centres equal vehicle.c's set_wheel_positions() exactly.
 *   [rule]      A documented, deterministic styling mapping from named parameters — e.g. the
 *               greenhouse sits where the CG bias puts it. Not a law of physics, but it cites
 *               its inputs and is stable.
 *   [phase-2]   A constant standing in for a parameter that does not exist yet. It is the same
 *               for every car, so it adds no false variety. Phase 2 replaces it with a real
 *               field. Marked TODO(phase-2) at the site.
 *
 * NO BYTE HASHING FOR GEOMETRY. car_visual.c must not derive any geometric feature from a hash
 * of the spec. Hashing would trivially satisfy the distinctness test while destroying the
 * property that test exists to protect. Colour is the single, stated exception: it is
 * explicitly arbitrary (see car_visual_colour_seed) and is excluded from the signature.
 *
 * UNITS AND FRAME. Everything is in metres in the body frame: +X forward (nose), +Y left,
 * origin at the CG — the convention in docs/SPEC.md and src/units.h. Consumers apply their own
 * metres-to-pixels scale.
 */
#ifndef DRIFTY_CAR_VISUAL_H
#define DRIFTY_CAR_VISUAL_H

#include <stdbool.h>
#include <stdint.h>

#include "raylib.h"   /* Vector2 and Color types only; no raylib call is made */

#include "vehicle.h"

/* Longitudinal stations of the hull half-outline, tail (index 0) to nose (last). The outline
 * is mirrored across the centreline, so a car is always laterally symmetric. Nine stations is
 * enough to express a wedge, a brick, a teardrop and a hipped muscle car while staying legible
 * at the ~13 px/m the game actually draws at. */
#define CAR_HULL_STATIONS 9

typedef struct {
    float xM;          /* body-space X, +X forward */
    float halfWidthM;  /* hull half-width at this station, always >= 0 */
} CarHullStation;

typedef struct {
    Vector2 centreM;        /* [identity] equals VehicleState.wheels[i].localPositionM */
    float   diameterM;      /* [identity] 2 * wheelRadiusM */
    float   widthM;
    float   rimDiameterM;
    float   discDiameterM;
    float   staticAngleRad; /* rest toe/steer cue, presentation-gained; never fed to physics */
    int     spokeCount;
} CarWheelVisual;

/* The normalised style axes. Derived first so that ~30 features move together instead of
 * independently: a heavy, tall, soft spec should read as van-like across silhouette, arches,
 * wing and exhaust at once. Exposed for test diagnostics and for docs/CAR_VISUAL.md. */
typedef struct {
    float mass01;     /* massKg */
    float size01;     /* wheelbase */
    float low01;      /* 1 - cgHeight; "how planted" */
    float grip01;     /* max lateral mu */
    float balance01;  /* muFront - muRear; oversteer bias */
    float power01;    /* peak engine torque * final drive / mass */
    float aero01;     /* dragCoefficient * frontalArea */
    float sport01;    /* composite of grip/low/power */
    float strip01;    /* light-for-its-size => racecar */
} CarLatents;

typedef struct {
    /* ---- silhouette ---- */
    CarHullStation hull[CAR_HULL_STATIONS];
    float lengthM;          /* nose to tail */
    float widthM;           /* widest point, both sides */
    float wheelbaseM;       /* [identity] cgToFrontM + cgToRearM */
    float frontOverhangM;   /* body ahead of the front axle */
    float rearOverhangM;    /* body behind the rear axle */

    /* ---- greenhouse ---- */
    float cabinCentreXM;
    float cabinLengthM;
    float cabinHalfWidthM;
    float windscreenXM;     /* forward edge of the glass */
    float backlightXM;      /* rear edge of the glass */

    /* ---- wheels ---- */
    CarWheelVisual wheels[WHEEL_COUNT];
    float archFlareM;       /* how far the arch stands proud of the hull */

    /* ---- appendages; a zero magnitude means absent ---- */
    float wingSpanM;
    float wingChordM;
    float wingXM;
    float splitterProtrusionM;
    float splitterWidthM;
    float mirrorOffsetM;
    float exhaustBoreM;
    int   exhaustCount;
    bool  hasCage;
    bool  hasMirrors;

    /* ---- palette; arbitrary but stable per spec, excluded from the signature ---- */
    Color body;
    Color bodyShade;
    Color cabin;
    Color glass;
    Color outline;
    Color tire;
    Color rim;
    Color disc;
    Color accent;
    Color lamp;

    CarLatents latents;
} CarVisual;

/* The grammar. Pure and total: any spec that passes vehicle_spec_is_valid() yields finite,
 * non-negative dimensions. A NULL spec or out pointer is a no-op. Deriving twice from the same
 * spec produces a bit-identical result, which the `car-visual` scenario asserts. */
void car_visual_derive(const VehicleSpec *spec, CarVisual *out);

/* The style axes on their own, for diagnostics and documentation. */
CarLatents car_visual_latents(const VehicleSpec *spec);

/* ------------------------------------------------------------------------- signature ----
 *
 * A feature vector used by the `corpus` scenario to report WHICH feature makes two cars too
 * similar. The authoritative distinctness test is a pixel comparison of two rasters; this
 * vector exists because a pixel count cannot name the culprit and this can.
 *
 * Every component is normalised to approximate VISIBLE METRES, so the threshold has a physical
 * meaning: at the ~13.2 px/m the game draws at (PIXELS_PER_METER 24 * CAMERA_BASE_ZOOM 0.55),
 * 0.08 m is almost exactly one screen pixel. Colour contributes nothing, by design — shape has
 * to carry the difference.
 */
int         car_visual_signature_count(void);
const char *car_visual_signature_component_name(int index);

/* Writes car_visual_signature_count() floats into out. Returns the number written, or 0 if
 * out is NULL or cap is too small. */
int car_visual_signature(const CarVisual *visual, float *out, int cap);

/* The colour seed: an FNV-1a hash over a fixed, explicitly listed set of spec fields — never
 * over the raw struct bytes, which would fold in padding and make the result depend on the
 * compiler. Exposed so tests can assert that colour and only colour depends on it. */
uint32_t car_visual_colour_seed(const VehicleSpec *spec);

#endif /* DRIFTY_CAR_VISUAL_H */
