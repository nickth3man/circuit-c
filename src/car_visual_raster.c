/*
 * car_visual_raster.c — CPU rasterizer for CarVisual. See the header for the contract.
 *
 * One geometry pass writes both an RGBA colour buffer and a feature-label buffer (either may
 * be omitted), so the label map used by the distinctness test can never describe a different
 * shape from the one actually drawn.
 *
 * Hard-edged fills only: a pixel centre is inside a shape or it is not. That is what the
 * reference sprites in resources/sprite_examples/ do, and it keeps the output exactly
 * reproducible.
 *
 * Raylib-free: linked into drifty_tests.exe.
 */
#include "car_visual_raster.h"

#include <math.h>
#include <string.h>

#include "math_utils.h"

#define MAX_POLY_POINTS (2 * CAR_HULL_STATIONS + 8)

/* Where a fill writes. Either channel may be NULL. */
typedef struct {
    unsigned char *rgba;
    unsigned char *labels;
    int width;
    int height;
    float pxPerM;
    float originXPx;
    float originYPx;
} RasterTarget;

/* ------------------------------------------------------------------------- primitives -- */

static void put_px(const RasterTarget *t, int x, int y, Color c, unsigned char label)
{
    if (x < 0 || y < 0 || x >= t->width || y >= t->height) return;
    const size_t index = (size_t)y * (size_t)t->width + (size_t)x;

    if (t->rgba != NULL) {
        unsigned char *p = t->rgba + index * CAR_RASTER_BPP;
        if (c.a >= 255) {
            p[0] = c.r; p[1] = c.g; p[2] = c.b; p[3] = 255;
        } else if (c.a > 0) {
            /* Straight-alpha source-over. */
            const float sa = (float)c.a / 255.0f;
            const float da = (float)p[3] / 255.0f;
            const float oa = sa + da * (1.0f - sa);
            if (oa > 0.0f) {
                for (int k = 0; k < 3; k++) {
                    const float sc = (k == 0) ? (float)c.r : (k == 1) ? (float)c.g : (float)c.b;
                    const float dc = (float)p[k];
                    p[k] = (unsigned char)clampf((sc * sa + dc * da * (1.0f - sa)) / oa,
                                                 0.0f, 255.0f);
                }
                p[3] = (unsigned char)clampf(oa * 255.0f, 0.0f, 255.0f);
            }
        }
    }

    /* A label is identity, not paint: a translucent wash does not change what a pixel IS. */
    if (t->labels != NULL && c.a >= 128) t->labels[index] = label;
}

static Vector2 to_px(const RasterTarget *t, float xM, float yM)
{
    /* +X forward -> +px; +Y left -> -py, matching src/units.h. */
    Vector2 p;
    p.x = t->originXPx + xM * t->pxPerM;
    p.y = t->originYPx - yM * t->pxPerM;
    return p;
}

/* Even-odd scanline fill. Handles convex and mildly concave outlines alike. */
static void fill_polygon_px(const RasterTarget *t, const Vector2 *pts, int count,
                            Color c, unsigned char label)
{
    if (count < 3) return;

    float minY = pts[0].y, maxY = pts[0].y;
    for (int i = 1; i < count; i++) {
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }

    int y0 = (int)floorf(minY);
    int y1 = (int)ceilf(maxY);
    if (y0 < 0) y0 = 0;
    if (y1 > t->height) y1 = t->height;

    for (int y = y0; y < y1; y++) {
        const float sy = (float)y + 0.5f;
        float xs[MAX_POLY_POINTS * 2];
        int n = 0;

        for (int i = 0, j = count - 1; i < count; j = i++) {
            const float ay = pts[j].y, by = pts[i].y;
            if ((ay <= sy && by > sy) || (by <= sy && ay > sy)) {
                const float tt = (sy - ay) / (by - ay);
                if (n < (int)(sizeof(xs) / sizeof(xs[0]))) {
                    xs[n++] = pts[j].x + tt * (pts[i].x - pts[j].x);
                }
            }
        }
        if (n < 2) continue;

        /* Insertion sort: n is tiny. */
        for (int i = 1; i < n; i++) {
            const float key = xs[i];
            int k = i - 1;
            while (k >= 0 && xs[k] > key) { xs[k + 1] = xs[k]; k--; }
            xs[k + 1] = key;
        }

        for (int i = 0; i + 1 < n; i += 2) {
            int x0 = (int)ceilf(xs[i] - 0.5f);
            int x1 = (int)ceilf(xs[i + 1] - 0.5f);
            if (x0 < 0) x0 = 0;
            if (x1 > t->width) x1 = t->width;
            for (int x = x0; x < x1; x++) put_px(t, x, y, c, label);
        }
    }
}

/* An oriented rectangle in body space: centre, length along its own +X, width across. */
static void fill_oriented_rect(const RasterTarget *t, float cxM, float cyM,
                               float lengthM, float widthM, float angleRad,
                               Color c, unsigned char label)
{
    if (!(lengthM > 0.0f) || !(widthM > 0.0f)) return;
    const float hl = 0.5f * lengthM, hw = 0.5f * widthM;
    const float ca = cosf(angleRad), sa = sinf(angleRad);
    const float ox[4] = { +hl, +hl, -hl, -hl };
    const float oy[4] = { +hw, -hw, -hw, +hw };

    Vector2 pts[4];
    for (int i = 0; i < 4; i++) {
        pts[i] = to_px(t, cxM + ox[i] * ca - oy[i] * sa, cyM + ox[i] * sa + oy[i] * ca);
    }
    fill_polygon_px(t, pts, 4, c, label);
}

static void fill_disc(const RasterTarget *t, float cxM, float cyM, float diameterM,
                      Color c, unsigned char label)
{
    if (!(diameterM > 0.0f)) return;
    const Vector2 centre = to_px(t, cxM, cyM);
    const float r = 0.5f * diameterM * t->pxPerM;
    const float r2 = r * r;

    int y0 = (int)floorf(centre.y - r), y1 = (int)ceilf(centre.y + r);
    int x0 = (int)floorf(centre.x - r), x1 = (int)ceilf(centre.x + r);
    if (y0 < 0) y0 = 0;
    if (x0 < 0) x0 = 0;
    if (y1 > t->height) y1 = t->height;
    if (x1 > t->width) x1 = t->width;

    for (int y = y0; y < y1; y++) {
        const float dy = ((float)y + 0.5f) - centre.y;
        for (int x = x0; x < x1; x++) {
            const float dx = ((float)x + 0.5f) - centre.x;
            if (dx * dx + dy * dy <= r2) put_px(t, x, y, c, label);
        }
    }
}

/* ------------------------------------------------------------------------ hull outline -- */

/* Build the closed outline: up the left flank tail-to-nose, back down the right flank.
 * `expandM` grows it outward, which is how the dark outline underneath is produced. */
static int build_hull(const RasterTarget *t, const CarVisual *v, float expandM, Vector2 *pts)
{
    const int last = CAR_HULL_STATIONS - 1;
    int n = 0;
    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        float x = v->hull[i].xM;
        if (i == 0) x -= expandM;
        if (i == last) x += expandM;
        pts[n++] = to_px(t, x, v->hull[i].halfWidthM + expandM);
    }
    for (int i = last; i >= 0; i--) {
        float x = v->hull[i].xM;
        if (i == 0) x -= expandM;
        if (i == last) x += expandM;
        pts[n++] = to_px(t, x, -(v->hull[i].halfWidthM + expandM));
    }
    return n;
}

/* ----------------------------------------------------------------------------- extents -- */

static void grow(float *minX, float *maxX, float *minY, float *maxY, float x, float y)
{
    if (x < *minX) *minX = x;
    if (x > *maxX) *maxX = x;
    if (y < *minY) *minY = y;
    if (y > *maxY) *maxY = y;
}

CarRasterInfo car_raster_info(const CarVisual *visual, float pxPerM, int padPx)
{
    CarRasterInfo info;
    memset(&info, 0, sizeof(info));
    if (visual == NULL || !(pxPerM > 0.0f)) return info;
    if (padPx < 0) padPx = 0;

    float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;

    for (int i = 0; i < CAR_HULL_STATIONS; i++) {
        grow(&minX, &maxX, &minY, &maxY, visual->hull[i].xM,  visual->hull[i].halfWidthM);
        grow(&minX, &maxX, &minY, &maxY, visual->hull[i].xM, -visual->hull[i].halfWidthM);
    }

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const CarWheelVisual *w = &visual->wheels[i];
        const float hl = 0.5f * w->diameterM + visual->archFlareM;
        const float hw = 0.5f * w->widthM + visual->archFlareM;
        grow(&minX, &maxX, &minY, &maxY, w->centreM.x + hl, w->centreM.y + hw);
        grow(&minX, &maxX, &minY, &maxY, w->centreM.x - hl, w->centreM.y - hw);
    }

    if (visual->wingSpanM > 0.0f) {
        grow(&minX, &maxX, &minY, &maxY,
             visual->wingXM - 0.5f * visual->wingChordM, +0.5f * visual->wingSpanM);
        grow(&minX, &maxX, &minY, &maxY,
             visual->wingXM + 0.5f * visual->wingChordM, -0.5f * visual->wingSpanM);
    }
    if (visual->splitterProtrusionM > 0.0f) {
        const float noseX = visual->hull[CAR_HULL_STATIONS - 1].xM;
        grow(&minX, &maxX, &minY, &maxY,
             noseX + visual->splitterProtrusionM, +0.5f * visual->splitterWidthM);
        grow(&minX, &maxX, &minY, &maxY, noseX, -0.5f * visual->splitterWidthM);
    }
    if (visual->mirrorOffsetM > 0.0f) {
        grow(&minX, &maxX, &minY, &maxY, visual->windscreenXM, +visual->mirrorOffsetM + 0.06f);
        grow(&minX, &maxX, &minY, &maxY, visual->windscreenXM, -visual->mirrorOffsetM - 0.06f);
    }

    /* One extra pixel of body space for the dark outline drawn under the hull. */
    const float outlineM = 1.0f / pxPerM;
    minX -= outlineM; maxX += outlineM;
    minY -= outlineM; maxY += outlineM;

    info.pxPerM = pxPerM;
    info.width  = (int)ceilf((maxX - minX) * pxPerM) + 2 * padPx;
    info.height = (int)ceilf((maxY - minY) * pxPerM) + 2 * padPx;
    if (info.width  < 1) info.width  = 1;
    if (info.height < 1) info.height = 1;
    info.originXPx = (float)padPx - minX * pxPerM;
    info.originYPx = (float)padPx + maxY * pxPerM;
    return info;
}

size_t car_raster_bytes(CarRasterInfo info)
{
    if (info.width <= 0 || info.height <= 0) return 0;
    return (size_t)info.width * (size_t)info.height * CAR_RASTER_BPP;
}

/* ------------------------------------------------------------------------------ render -- */

static void render(const CarVisual *v, RasterTarget *t)
{
    Vector2 poly[MAX_POLY_POINTS];
    const float onePx = 1.0f / t->pxPerM;
    const float noseX = v->hull[CAR_HULL_STATIONS - 1].xM;
    const float tailX = v->hull[0].xM;

    /* L7a: splitter first, so the body edge stays clean over the top of it. Drawn dark, not in
     * the accent colour: a splitter is a shadowed lip under the nose, and painting it as an
     * accent made it read as a front wing. */
    if (v->splitterProtrusionM > 0.0f) {
        fill_oriented_rect(t, noseX + 0.5f * v->splitterProtrusionM - onePx, 0.0f,
                           v->splitterProtrusionM + 2.0f * onePx, v->splitterWidthM,
                           0.0f, v->outline, CAR_LABEL_SPLITTER);
    }

    /* L6: wheels under the body. Where the track is wider than the hull they stay visible,
     * which is how a wide-track car gets its stance without any special case. */
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const CarWheelVisual *w = &v->wheels[i];
        fill_oriented_rect(t, w->centreM.x, w->centreM.y, w->diameterM, w->widthM,
                           w->staticAngleRad, v->tire, CAR_LABEL_TIRE);
        fill_oriented_rect(t, w->centreM.x, w->centreM.y, w->rimDiameterM,
                           w->widthM * 0.62f, w->staticAngleRad, v->rim, CAR_LABEL_RIM);
        if (w->discDiameterM > 0.0f) {
            fill_oriented_rect(t, w->centreM.x, w->centreM.y, w->discDiameterM,
                               w->widthM * 0.34f, w->staticAngleRad, v->disc, CAR_LABEL_DISC);
        }
    }

    /* L1: dark outline plate, then the body on top — the same silhouette trick render.c
     * already used, so the car reads against both asphalt and grass. */
    fill_polygon_px(t, poly, build_hull(t, v, onePx, poly), v->outline, CAR_LABEL_OUTLINE);
    fill_polygon_px(t, poly, build_hull(t, v, 0.0f, poly), v->body, CAR_LABEL_BODY);

    /* L2: rear half a shade darker, so front and rear read apart at a glance. */
    {
        const float rearLen = 0.42f * (noseX - tailX);
        fill_oriented_rect(t, tailX + 0.5f * rearLen, 0.0f, rearLen,
                           2.0f * v->hull[1].halfWidthM, 0.0f,
                           (Color){ v->bodyShade.r, v->bodyShade.g, v->bodyShade.b, 150 },
                           CAR_LABEL_BODY_SHADE);
    }

    /* L6b: arch flares over the body, around each wheel. */
    if (v->archFlareM > 0.0f) {
        for (int i = 0; i < WHEEL_COUNT; i++) {
            const CarWheelVisual *w = &v->wheels[i];
            const float inboard = (w->centreM.y > 0.0f) ? -1.0f : 1.0f;
            fill_oriented_rect(t, w->centreM.x,
                               w->centreM.y + inboard * (0.5f * w->widthM + 0.5f * onePx),
                               w->diameterM + 2.0f * v->archFlareM, onePx * 2.0f, 0.0f,
                               v->bodyShade, CAR_LABEL_ARCH);
        }
    }

    /* L3/L4: greenhouse, then the glass bands inside it. */
    if (v->cabinLengthM > 0.0f) {
        fill_oriented_rect(t, v->cabinCentreXM, 0.0f, v->cabinLengthM,
                           2.0f * v->cabinHalfWidthM, 0.0f, v->cabin, CAR_LABEL_CABIN);

        const float bandLen = 0.26f * v->cabinLengthM;
        fill_oriented_rect(t, v->windscreenXM - 0.5f * bandLen, 0.0f, bandLen,
                           2.0f * v->cabinHalfWidthM * 0.90f, 0.0f, v->glass, CAR_LABEL_GLASS);
        fill_oriented_rect(t, v->backlightXM + 0.5f * bandLen, 0.0f, bandLen,
                           2.0f * v->cabinHalfWidthM * 0.84f, 0.0f, v->glass, CAR_LABEL_GLASS);
    }

    /* L4b: cage bars drawn through the glass on a stripped car. */
    if (v->hasCage) {
        fill_oriented_rect(t, v->cabinCentreXM, 0.0f, v->cabinLengthM, 2.0f * onePx, 0.0f,
                           v->outline, CAR_LABEL_CAGE);
        fill_oriented_rect(t, v->cabinCentreXM, 0.0f, 2.0f * onePx,
                           2.0f * v->cabinHalfWidthM, 0.0f, v->outline, CAR_LABEL_CAGE);
    }

    /* L5: lights. */
    {
        const float lampLen = 0.10f, lampWid = 0.26f;
        const float noseHalf = v->hull[CAR_HULL_STATIONS - 1].halfWidthM;
        const float tailHalf = v->hull[0].halfWidthM;
        for (int s = -1; s <= 1; s += 2) {
            fill_oriented_rect(t, noseX - 0.5f * lampLen, (float)s * noseHalf * 0.55f,
                               lampLen, lampWid, 0.0f, v->lamp, CAR_LABEL_LAMP);
            fill_oriented_rect(t, tailX + 0.5f * lampLen, (float)s * tailHalf * 0.55f,
                               lampLen, lampWid, 0.0f, (Color){ 200, 48, 40, 255 },
                               CAR_LABEL_LAMP);
        }
    }

    /* L7b: wing over the deck. */
    if (v->wingSpanM > 0.0f) {
        fill_oriented_rect(t, v->wingXM, 0.0f, v->wingChordM, v->wingSpanM, 0.0f,
                           v->accent, CAR_LABEL_WING);
        fill_oriented_rect(t, v->wingXM, 0.0f, v->wingChordM * 0.34f, v->wingSpanM, 0.0f,
                           v->outline, CAR_LABEL_WING);
    }

    /* L7c: mirrors on their stalks. */
    if (v->hasMirrors && v->mirrorOffsetM > 0.0f) {
        for (int s = -1; s <= 1; s += 2) {
            fill_oriented_rect(t, v->windscreenXM, (float)s * v->mirrorOffsetM,
                               0.14f, 0.10f, 0.0f, v->bodyShade, CAR_LABEL_MIRROR);
        }
    }

    /* L7d: exhaust tips at the tail, spaced across the centreline. */
    if (v->exhaustCount > 0 && v->exhaustBoreM > 0.0f) {
        const float spacing = v->exhaustBoreM * 1.7f;
        const float first = -0.5f * spacing * (float)(v->exhaustCount - 1);
        for (int i = 0; i < v->exhaustCount; i++) {
            fill_disc(t, tailX + 0.5f * v->exhaustBoreM, first + spacing * (float)i,
                      v->exhaustBoreM, (Color){ 70, 74, 82, 255 }, CAR_LABEL_EXHAUST);
        }
    }
}

static bool prepare(const CarVisual *visual, CarRasterInfo info, RasterTarget *t)
{
    if (visual == NULL || info.width <= 0 || info.height <= 0 || !(info.pxPerM > 0.0f)) {
        return false;
    }
    t->width = info.width;
    t->height = info.height;
    t->pxPerM = info.pxPerM;
    t->originXPx = info.originXPx;
    t->originYPx = info.originYPx;
    return true;
}

bool car_raster_draw(const CarVisual *visual, CarRasterInfo info,
                     unsigned char *rgba, size_t bytes)
{
    RasterTarget t;
    memset(&t, 0, sizeof(t));
    if (rgba == NULL || bytes < car_raster_bytes(info)) return false;
    if (!prepare(visual, info, &t)) return false;

    memset(rgba, 0, car_raster_bytes(info));
    t.rgba = rgba;
    t.labels = NULL;
    render(visual, &t);
    return true;
}

bool car_raster_draw_labels(const CarVisual *visual, CarRasterInfo info,
                            unsigned char *labels, size_t bytes)
{
    RasterTarget t;
    memset(&t, 0, sizeof(t));
    const size_t need = (size_t)info.width * (size_t)info.height;
    if (labels == NULL || info.width <= 0 || info.height <= 0 || bytes < need) return false;
    if (!prepare(visual, info, &t)) return false;

    memset(labels, CAR_LABEL_EMPTY, need);
    t.rgba = NULL;
    t.labels = labels;
    render(visual, &t);
    return true;
}

bool car_raster_rotate_nose_up(const unsigned char *src, int srcW, int srcH,
                               unsigned char *dst, size_t dstBytes)
{
    if (src == NULL || dst == NULL || srcW <= 0 || srcH <= 0) return false;
    const size_t need = (size_t)srcW * (size_t)srcH * CAR_RASTER_BPP;
    if (dstBytes < need) return false;

    /* Rotate CCW: the nose (+X, at large source x) ends up at small destination y. */
    const int dstW = srcH;
    for (int y = 0; y < srcH; y++) {
        for (int x = 0; x < srcW; x++) {
            const int dx = y;
            const int dy = srcW - 1 - x;
            const size_t s = ((size_t)y * (size_t)srcW + (size_t)x) * CAR_RASTER_BPP;
            const size_t d = ((size_t)dy * (size_t)dstW + (size_t)dx) * CAR_RASTER_BPP;
            dst[d + 0] = src[s + 0];
            dst[d + 1] = src[s + 1];
            dst[d + 2] = src[s + 2];
            dst[d + 3] = src[s + 3];
        }
    }
    return true;
}

float car_raster_difference(const unsigned char *labelsA, const unsigned char *labelsB,
                            int width, int height)
{
    if (labelsA == NULL || labelsB == NULL || width <= 0 || height <= 0) return 0.0f;

    const size_t count = (size_t)width * (size_t)height;
    size_t unionPx = 0, differing = 0;
    for (size_t i = 0; i < count; i++) {
        const unsigned char a = labelsA[i];
        const unsigned char b = labelsB[i];
        if (a == CAR_LABEL_EMPTY && b == CAR_LABEL_EMPTY) continue;
        unionPx++;
        if (a != b) differing++;
    }
    if (unionPx == 0) return 0.0f;
    return (float)differing / (float)unionPx;
}
