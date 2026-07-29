#include "render/render.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "dev/car_corpus.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"
#include "dev/dev_lab.h"
#include "game/game.h"
#include "core/math_utils.h"
#include "game/profile.h"
#include "physics/tire.h"
#include "core/units.h"

VehicleDrawState render_interpolate_vehicle(const VehicleRenderState *state, float alpha)
{
    VehicleDrawState out;
    memset(&out, 0, sizeof(out));
    if (state == NULL) return out;
    const float t = clampf(alpha, 0.0f, 1.0f);
    out.positionM.x = lerpf(state->prevPositionM.x, state->currPositionM.x, t);
    out.positionM.y = lerpf(state->prevPositionM.y, state->currPositionM.y, t);
    out.headingRad = lerp_angle(state->prevHeadingRad, state->currHeadingRad, t);
    for (int i = 0; i < WHEEL_COUNT; i++) {
        out.wheelAngleRad[i] = lerp_angle(state->prevWheelAngleRad[i],
                                         state->currWheelAngleRad[i], t);
    }
    return out;
}

#if defined(DRIFTY_HEADLESS)
void render_draw_game(struct Game *game, float interpolationAlpha)
{
    (void)game;
    (void)interpolationAlpha;
}
void render_pre_reload(void)  {}
void render_post_reload(void) {}
void render_shutdown(void)    {}
int  render_gallery_page_count(void) { return 0; }
void render_draw_gallery(struct Game *game, int page) { (void)game; (void)page; }
#else

#include "raylib.h"

static Vector2 body_point_to_world(Vector2 bodyPointM, Vector2 positionM, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){
        positionM.x + bodyPointM.x * c - bodyPointM.y * s,
        positionM.y + bodyPointM.x * s + bodyPointM.y * c
    };
}

static Vector2 body_vector_to_world(Vector2 bodyVector, float headingRad)
{
    const float c = cosf(headingRad);
    const float s = sinf(headingRad);
    return (Vector2){
        bodyVector.x * c - bodyVector.y * s,
        bodyVector.x * s + bodyVector.y * c
    };
}

static void draw_world_vector(Vector2 startM, Vector2 vectorM, float ppm,
                              Color color, const char *label)
{
    if (vectorM.x * vectorM.x + vectorM.y * vectorM.y < 0.0004f) return;
    const Vector2 endM = { startM.x + vectorM.x, startM.y + vectorM.y };
    const Vector2 startPx = units_world_to_render_px(startM, ppm);
    const Vector2 endPx = units_world_to_render_px(endM, ppm);
    DrawLineEx(startPx, endPx, 2.0f, color);
    DrawCircleV(endPx, 3.0f, color);
    if (label != NULL && label[0] != '\0') {
        DrawText(label, (int)endPx.x + 4, (int)endPx.y - 8, 12, color);
    }
}

/* ---- track rendering --------------------------------------------------------------- */

static void render_draw_track(const Track *track, float ppm)
{
    if (track == NULL || track->nodes == NULL || track->count < 2) return;

    /* Parking lot mode: wide open rectangular area with parking-space line grid. */
    if (track->isParkingLot) {
        const float lotL = track->lotMinXM;
        const float lotR = track->lotMaxXM;
        const float lotB = track->lotMinYM;
        const float lotT = track->lotMaxYM;

        /* ---- Grass surround (behind everything) ---- */
        {
            const float marginM = 60.0f;
            const Vector2 blPx = units_world_to_render_px(
                (Vector2){ lotL - marginM, lotB - marginM }, ppm);
            const Vector2 trPx = units_world_to_render_px(
                (Vector2){ lotR + marginM, lotT + marginM }, ppm);
            DrawRectangle((int)blPx.x, (int)trPx.y,
                          (int)(trPx.x - blPx.x), (int)(blPx.y - trPx.y),
                          (Color){ 76, 117, 67, 255 });
        }

        /* ---- Asphalt lot ---- */
        {
            const Vector2 blPx = units_world_to_render_px(
                (Vector2){ lotL, lotB }, ppm);
            const Vector2 trPx = units_world_to_render_px(
                (Vector2){ lotR, lotT }, ppm);
            DrawRectangle((int)blPx.x, (int)trPx.y,
                          (int)(trPx.x - blPx.x), (int)(blPx.y - trPx.y),
                          (Color){ 45, 45, 50, 255 });
        }

        /* ---- Parking-space lines (vertical & horizontal grid) ---- */
        {
            const float spacingM  = 6.0f;   /* distance between line centres */
            const float lineLenM  = 5.0f;   /* length of each painted line */
            const float lineW     = 3.0f;   /* pixel thickness */
            const Color colWhite  = { 210, 210, 215, 200 };  /* semi-transparent white */

            /* Vertical parking strips: rows of short east-west lines at regular Y intervals */
            for (float yy = lotB + 3.0f; yy <= lotT - 3.0f; yy += spacingM) {
                for (float xx = lotL + 3.0f; xx <= lotR - 3.0f; xx += spacingM) {
                    const Vector2 aPx = units_world_to_render_px(
                        (Vector2){ xx - lineLenM * 0.5f, yy }, ppm);
                    const Vector2 bPx = units_world_to_render_px(
                        (Vector2){ xx + lineLenM * 0.5f, yy }, ppm);
                    DrawLineEx(aPx, bPx, lineW, colWhite);
                }
            }

            /* Perimeter double-line (curb) */
            const Color colCurb = { 180, 180, 185, 230 };
            const float curbThick = 4.0f;
            {
                const Vector2 bl = units_world_to_render_px((Vector2){ lotL, lotB }, ppm);
                const Vector2 br = units_world_to_render_px((Vector2){ lotR, lotB }, ppm);
                const Vector2 tr = units_world_to_render_px((Vector2){ lotR, lotT }, ppm);
                const Vector2 tl = units_world_to_render_px((Vector2){ lotL, lotT }, ppm);
                DrawLineEx(bl, br, curbThick, colCurb);
                DrawLineEx(br, tr, curbThick, colCurb);
                DrawLineEx(tr, tl, curbThick, colCurb);
                DrawLineEx(tl, bl, curbThick, colCurb);
            }

            /* Inner curb offset line (2m inset) */
            const float inset = 2.0f;
            const Color colInnerCurb = { 140, 140, 145, 120 };
            {
                const Vector2 bl = units_world_to_render_px(
                    (Vector2){ lotL + inset, lotB + inset }, ppm);
                const Vector2 br = units_world_to_render_px(
                    (Vector2){ lotR - inset, lotB + inset }, ppm);
                const Vector2 tr = units_world_to_render_px(
                    (Vector2){ lotR - inset, lotT - inset }, ppm);
                const Vector2 tl = units_world_to_render_px(
                    (Vector2){ lotL + inset, lotT - inset }, ppm);
                DrawLineEx(bl, br, 2.0f, colInnerCurb);
                DrawLineEx(br, tr, 2.0f, colInnerCurb);
                DrawLineEx(tr, tl, 2.0f, colInnerCurb);
                DrawLineEx(tl, bl, 2.0f, colInnerCurb);
            }
        }

        return;
    }

    /* ---- Original stadium oval rendering (below) ---- */

    const int n = track->count;
    /* All current nodes share the same half-width; sample the first. */
    const float hwM = track->nodes[0].halfWidthM;
    const float ribbonThicknessPx = 2.0f * hwM * ppm;

    /* --- grass surround (drawn first, behind everything) --- */
    {
        float minXM = track->nodes[0].centerM.x;
        float maxXM = track->nodes[0].centerM.x;
        float minYM = track->nodes[0].centerM.y;
        float maxYM = track->nodes[0].centerM.y;
        for (int i = 1; i < n; i++) {
            const Vector2 c = track->nodes[i].centerM;
            if (c.x < minXM) minXM = c.x;
            if (c.x > maxXM) maxXM = c.x;
            if (c.y < minYM) minYM = c.y;
            if (c.y > maxYM) maxYM = c.y;
        }
        const float marginM = 60.0f;
        minXM -= marginM; maxXM += marginM;
        minYM -= marginM; maxYM += marginM;

        /* Convert corners.  Y is negated by units_world_to_render_px, so the
         * bottom-left world corner produces the largest render Y. */
        const Vector2 blPx = units_world_to_render_px((Vector2){minXM, minYM}, ppm);
        const Vector2 trPx = units_world_to_render_px((Vector2){maxXM, maxYM}, ppm);
        DrawRectangle((int)blPx.x, (int)trPx.y,
                      (int)(trPx.x - blPx.x), (int)(blPx.y - trPx.y),
                      (Color){ 76, 117, 67, 255 });
    }

    /* --- asphalt ribbon (thick filled centreline) --- */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const Vector2 aPx = units_world_to_render_px(track->nodes[i].centerM, ppm);
        const Vector2 bPx = units_world_to_render_px(track->nodes[j].centerM, ppm);
        DrawLineEx(aPx, bPx, ribbonThicknessPx, (Color){ 40, 40, 45, 255 });
    }

    /* --- track boundaries (two offset polylines) --- */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const Vector2 a = track->nodes[i].centerM;
        const Vector2 b = track->nodes[j].centerM;

        const float dx = b.x - a.x;
        const float dy = b.y - a.y;
        const float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-6f) continue;

        const float invLen = 1.0f / len;
        const float ux = dx * invLen;
        const float uy = dy * invLen;

        /* Perpendicular in world space: (-uy, ux) = left side of forward direction. */
        const float perpX = -uy * hwM;
        const float perpY =  ux * hwM;

        const Vector2 leftA  = units_world_to_render_px((Vector2){a.x + perpX, a.y + perpY}, ppm);
        const Vector2 leftB  = units_world_to_render_px((Vector2){b.x + perpX, b.y + perpY}, ppm);
        const Vector2 rightA = units_world_to_render_px((Vector2){a.x - perpX, a.y - perpY}, ppm);
        const Vector2 rightB = units_world_to_render_px((Vector2){b.x - perpX, b.y - perpY}, ppm);

        DrawLineEx(leftA,  leftB,  2.5f, (Color){ 220, 220, 225, 255 });
        DrawLineEx(rightA, rightB, 2.5f, (Color){ 220, 220, 225, 255 });
    }

    /* --- centreline hint (faint guide) --- */
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const Vector2 aPx = units_world_to_render_px(track->nodes[i].centerM, ppm);
        const Vector2 bPx = units_world_to_render_px(track->nodes[j].centerM, ppm);
        DrawLineEx(aPx, bPx, 1.5f, (Color){ 180, 180, 190, 80 });
    }
}

/* ---- presentation palette, type scale, and screen-space helpers -----------------------
 *
 * Arcade-drift palette (documented for the design review):
 *   COL_ACCENT       hot gold    - score, combo, DRIFT! callouts, the car nose marker
 *   COL_ACCENT_WARM  warm orange - NEW BEST flash and other "payoff" moments
 *   COL_COOL         steel cyan  - secondary info (best score, rpm bar fill)
 *   COL_TEXT         near-white  - primary HUD text
 *   COL_TEXT_DIM     slate       - secondary and hint text
 *   COL_PANEL        translucent charcoal - backing panels behind HUD clusters
 *   COL_PANEL_EDGE   faint white          - panel outline, separates panel from track
 *   COL_DIM_SCREEN   heavy translucent charcoal - full-screen dim behind overlays
 *
 * Type scale (one scale, used everywhere below):
 *   title 64 | overlay heading 40-48 | results figure 56 | cluster figure 34-46 |
 *   body 18-20 | labels/hints 14-16 | micro 12.
 */
static const Color COL_ACCENT       = { 255, 198,  64, 255 };
static const Color COL_ACCENT_WARM  = { 255, 120,  72, 255 };
static const Color COL_COOL         = { 110, 205, 235, 255 };
static const Color COL_TEXT         = { 236, 238, 242, 255 };
static const Color COL_TEXT_DIM     = { 152, 158, 170, 255 };
static const Color COL_PANEL        = {  12,  14,  18, 170 };
static const Color COL_PANEL_EDGE   = { 255, 255, 255,  26 };
static const Color COL_DIM_SCREEN   = {   8,  10,  14, 185 };

/* The car's own palette used to live here as COL_CAR_BODY / COL_CAR_OUTLINE / COL_CAR_CABIN /
 * COL_TIRE / COL_RIM, next to a hardcoded 4.2 x 1.82 m box. Both are gone: the vehicle's
 * colours and its geometry are now derived in src/render/car_visual.c and rasterized by
 * src/render/car_visual_raster.c, and this file only uploads and draws what they produce. Adding a
 * styling decision back here would put it where drifty_tests cannot reach it. */

static void draw_text_centered(const char *text, int y, int fontSize, Color color)
{
    DrawText(text, (SCREEN_W - MeasureText(text, fontSize)) / 2, y, fontSize, color);
}

static void draw_text_centered_shadow(const char *text, int y, int fontSize, Color color)
{
    const int x = (SCREEN_W - MeasureText(text, fontSize)) / 2;
    DrawText(text, x + 3, y + 3, fontSize, (Color){ 0, 0, 0, 170 });
    DrawText(text, x, y, fontSize, color);
}

static void draw_hud_panel(Rectangle rec)
{
    DrawRectangleRounded(rec, 0.16f, 6, COL_PANEL);
    DrawRectangleRoundedLines(rec, 0.16f, 6, COL_PANEL_EDGE);
}

/* Slow pulse between two alpha levels, for "press a key" prompts and the DRIFT! callout.
 * Render-only time source; nothing here feeds the simulation. */
static unsigned char pulse_alpha(float cyclesPerSecond, unsigned char lo, unsigned char hi)
{
    const float s = 0.5f + 0.5f * sinf((float)GetTime() * 6.2831853f * cyclesPerSecond);
    return (unsigned char)(lo + (unsigned char)((float)(hi - lo) * s));
}

/* ------------------------------------------------------------- baked vehicle sprites ----
 *
 * The car is drawn from textures baked by the SAME CPU rasterizer the headless contact sheet
 * uses (src/render/car_visual_raster.c), so there is exactly one geometry grammar in the project and
 * the gallery cannot drift away from the game.
 *
 * BAKE, DO NOT REDRAW. A car's pixels change only when its spec changes, which for a running
 * game is almost never — but the Physics Lab writes game->spec live, so it does happen. Every
 * frame derives the CarVisual (cheap, no allocation) and hashes it with car_visual_bake_key();
 * the textures are rebuilt only when that key or the metres-to-pixels scale differs from what
 * is already on the GPU. A still car costs one derive and one integer compare per frame.
 *
 * SCALE. Baked at the WORLD scale, PIXELS_PER_METER * CAMERA_BASE_ZOOM, so one texel is
 * exactly one pixel of the low-resolution world target at rest and the sprite is never
 * resampled — see the scale chain in src/core/config.h. The drift zoom still moves continuously
 * around that resting point; at rest, which is where a still image is judged, the mapping is
 * exact.
 *
 * COMPOSITION. Body first, then the four wheels on top, matching the L6-after-L1 order the
 * rasterizer draws them in. The front wheels rotate to heading + steer + their derived static
 * toe; the rears keep their derived static alignment. The wheels are NOT baked into the body
 * texture — a wheel that cannot turn would make the steering unreadable at exactly the moment
 * it matters most.
 *
 * HOT RELOAD. Texture2D handles are raylib-tracked GPU resources living in module statics, so
 * render_pre_reload() releases them and render_post_reload() drops the cache key so the next
 * frame rebakes. This is the audio.c contract applied to textures; see render.h.
 */
#define CAR_TEX_PAD_PX 1

/* One car's sprites. The running game keeps exactly one of these in a module static; the
 * gallery builds and discards one per cell, so reviewing a hundred vehicles never holds more
 * than a single car's textures on the GPU. */
typedef struct {
    bool      ready;
    float     pxPerM;          /* texels per metre these were baked at */
    Texture2D body;
    Vector2   bodyOriginPx;
    Texture2D wheel[WHEEL_COUNT];
    Vector2   wheelOriginPx[WHEEL_COUNT];
} CarSprites;

static void car_sprites_unload(CarSprites *s)
{
    if (s == NULL || !s->ready) return;
    UnloadTexture(s->body);
    for (int i = 0; i < WHEEL_COUNT; i++) UnloadTexture(s->wheel[i]);
    memset(s, 0, sizeof(*s));
}

/* Upload one rasterized part. Returns false on any failure, which the caller treats as a
 * failed bake rather than drawing a garbage handle. */
static bool upload_part(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                        float pxPerM, Texture2D *texOut, Vector2 *originOut)
{
    const CarRasterInfo info =
        car_raster_part_info(visual, part, wheelIndex, pxPerM, CAR_TEX_PAD_PX);
    const size_t bytes = car_raster_bytes(info);
    if (bytes == 0) return false;

    unsigned char *rgba = (unsigned char *)malloc(bytes);
    if (rgba == NULL) return false;
    if (!car_raster_draw_part(visual, part, wheelIndex, info, rgba, bytes)) {
        free(rgba);
        return false;
    }

    /* Wrap the buffer raylib expects without handing ownership over: LoadTextureFromImage
     * copies to the GPU and does not keep the pointer, so this frees its own memory rather
     * than calling UnloadImage on something raylib never allocated. */
    Image image;
    memset(&image, 0, sizeof(image));
    image.data = rgba;
    image.width = info.width;
    image.height = info.height;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;

    *texOut = LoadTextureFromImage(image);
    free(rgba);

    if (texOut->id == 0) return false;
    /* Pixel art: nearest neighbour, never a smoothed upscale. */
    SetTextureFilter(*texOut, TEXTURE_FILTER_POINT);
    originOut->x = info.originXPx;
    originOut->y = info.originYPx;
    return true;
}

/* Bake one car's five sprites. Leaves `out` not-ready and holding nothing on failure. */
static bool car_sprites_bake(CarSprites *out, const CarVisual *visual, float pxPerM)
{
    if (out == NULL || visual == NULL) return false;
    car_sprites_unload(out);

    bool ok = upload_part(visual, CAR_RASTER_PART_BODY, 0, pxPerM,
                          &out->body, &out->bodyOriginPx);
    for (int i = 0; i < WHEEL_COUNT && ok; i++) {
        ok = upload_part(visual, CAR_RASTER_PART_WHEEL, i, pxPerM,
                         &out->wheel[i], &out->wheelOriginPx[i]);
    }
    if (!ok) {
        car_sprites_unload(out);
        return false;
    }
    out->pxPerM = pxPerM;
    out->ready = true;
    return true;
}

/* Draw a baked part about its documented pivot, `scale` destination pixels per texel.
 * `rotationRad` is a heading in world terms; units_heading_to_rotation_deg handles the
 * screen-space Y flip. */
static void draw_car_part(Texture2D tex, Vector2 originPx, Vector2 centrePx,
                          float rotationRad, float scale)
{
    if (tex.id == 0) return;
    const Rectangle src = { 0.0f, 0.0f, (float)tex.width, (float)tex.height };
    const Rectangle dst = { centrePx.x, centrePx.y,
                            (float)tex.width * scale, (float)tex.height * scale };
    /* DrawTexturePro measures `origin` in DESTINATION units, so the pivot scales with the
     * sprite. Getting this wrong shifts the car off its own centre of mass as the scale
     * changes, which is exactly the kind of bug a still screenshot hides. */
    const Vector2 origin = { originPx.x * scale, originPx.y * scale };
    DrawTexturePro(tex, src, dst, origin, units_heading_to_rotation_deg(rotationRad), WHITE);
}

/* The one place a car is composited, used by the running game and by the gallery alike.
 * `steerRad` is per wheel and is added to the derived static alignment; pass NULL for a
 * straight-ahead pose. `scale` is destination pixels per texel. */
static void car_sprites_draw(const CarSprites *s, const CarVisual *visual,
                             Vector2 centrePx, float headingRad,
                             const float *steerRad, float scale)
{
    if (s == NULL || !s->ready || visual == NULL) return;

    draw_car_part(s->body, s->bodyOriginPx, centrePx, headingRad, scale);

    /* Destination pixels per metre follows from what the sprites were baked at and what they
     * are being drawn at, so a caller never has to restate the world scale. */
    const float destPxPerM = s->pxPerM * scale;
    const float c = cosf(headingRad);
    const float sn = sinf(headingRad);

    for (int i = 0; i < WHEEL_COUNT; i++) {
        const Vector2 hubM = visual->wheels[i].centreM;
        /* Body frame -> world offset in metres, then metres -> destination pixels. +Y is
         * left in body and world space and up the screen, hence the negated Y. */
        const float wx = hubM.x * c - hubM.y * sn;
        const float wy = hubM.x * sn + hubM.y * c;
        const Vector2 hubPx = { centrePx.x + wx * destPxPerM,
                                centrePx.y - wy * destPxPerM };

        const float steer = (steerRad != NULL) ? steerRad[i] : 0.0f;
        draw_car_part(s->wheel[i], s->wheelOriginPx[i], hubPx,
                      headingRad + steer + visual->wheels[i].staticAngleRad, scale);
    }
}

/* ---- the running game's single cached car ---- */

static CarSprites s_car;
static uint32_t   s_carTexKey = 0u;
static float      s_carTexPxPerM = 0.0f;

/* Rebake if and only if the picture or the scale changed. */
static void ensure_car_textures(const CarVisual *visual, float pxPerM)
{
    const uint32_t key = car_visual_bake_key(visual);
    if (s_car.ready && key == s_carTexKey && pxPerM == s_carTexPxPerM) return;

    if (!car_sprites_bake(&s_car, visual, pxPerM)) {
        TRACELOG(LOG_WARNING, "RENDER: vehicle sprite bake failed; retrying next frame");
        s_carTexKey = 0u;
        s_carTexPxPerM = 0.0f;
        return;
    }
    s_carTexKey = key;
    s_carTexPxPerM = pxPerM;
}

/* Destination pixels per texel inside the world camera. Sprites are baked at the world scale
 * (PIXELS_PER_METER * CAMERA_BASE_ZOOM) but drawn in render-pixel space, which the camera
 * then multiplies by its zoom — so a texel becomes exactly one target pixel when the zoom is
 * at rest, and the drift zoom moves smoothly around that. */
#define CAR_WORLD_TEXEL_SCALE (1.0f / CAMERA_BASE_ZOOM)

static float car_bake_px_per_m(float renderPixelsPerMeter)
{
    return renderPixelsPerMeter * CAMERA_BASE_ZOOM;
}

static void draw_vehicle(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;

    CarVisual visual;                       /* stack-local by design: never stored in Game */
    car_visual_derive(&game->spec, &visual);
    ensure_car_textures(&visual, car_bake_px_per_m(ppm));
    if (!s_car.ready) return;

    /* Front wheels draw at heading + steerAngleRad with a presentation-only gain so the steer
     * is unmistakable at top-down scale; the physics angle itself is untouched. Rear wheels
     * carry no steer, but they do carry whatever static alignment the grammar derived for
     * them, which car_sprites_draw adds for every wheel. */
    const float steerVisualGain = 1.25f;  /* render-only amplification, documented above */
    float steer[WHEEL_COUNT];
    for (int i = 0; i < WHEEL_COUNT; i++) {
        const bool isFront = (i == WHEEL_FRONT_LEFT || i == WHEEL_FRONT_RIGHT);
        steer[i] = isFront ? draw->wheelAngleRad[i] * steerVisualGain : 0.0f;
    }

    car_sprites_draw(&s_car, &visual,
                     units_world_to_render_px(draw->positionM, ppm),
                     draw->headingRad, steer, CAR_WORLD_TEXEL_SCALE);
}

/* ------------------------------------------------------------- pixel-art world target ----
 *
 * The world is rasterized into a low-resolution RenderTexture2D and blown up once by an exact
 * integer factor with nearest-neighbour filtering; the HUD, raygui and the Physics Lab draw
 * afterwards at native resolution. src/core/config.h carries the whole scale chain and the reason
 * for each number in it.
 */
static RenderTexture2D s_worldTarget;
static bool s_worldTargetReady = false;

static void unload_world_target(void)
{
    if (!s_worldTargetReady) return;
    UnloadRenderTexture(s_worldTarget);
    memset(&s_worldTarget, 0, sizeof(s_worldTarget));
    s_worldTargetReady = false;
}

static void ensure_world_target(void)
{
    if (s_worldTargetReady) return;
    s_worldTarget = LoadRenderTexture(PIXEL_ART_TARGET_W, PIXEL_ART_TARGET_H);
    if (s_worldTarget.id == 0) return;
    SetTextureFilter(s_worldTarget.texture, TEXTURE_FILTER_POINT);
    s_worldTargetReady = true;
}

/* The world camera, adjusted for the low-resolution target.
 *
 * Two changes from game->camera, both load-bearing:
 *   - the offset centres on the TARGET, which is half the window in each axis;
 *   - the camera translation is snapped to whole target pixels. A Camera2D maps world to
 *     screen as (world - target) * zoom + offset, so a fractional (-target * zoom) makes
 *     every world pixel land between two target pixels and the whole grid crawls a pixel at a
 *     time as the car moves. Snapping costs sub-pixel camera smoothness, which nobody can
 *     see, and buys a stable pixel grid, which everybody can.
 */
static Camera2D world_camera_for_target(Camera2D camera)
{
    Camera2D cam = camera;
    cam.offset = (Vector2){ (float)PIXEL_ART_TARGET_W * 0.5f,
                            (float)PIXEL_ART_TARGET_H * 0.5f };

    cam.offset.x = units_snap_camera_offset_axis(cam.offset.x, cam.target.x, cam.zoom);
    cam.offset.y = units_snap_camera_offset_axis(cam.offset.y, cam.target.y, cam.zoom);
    return cam;
}

/* Blit the finished world target over the whole window. The source height is negative because
 * a RenderTexture2D is stored bottom-up; the destination is exactly SCREEN_W x SCREEN_H, so
 * the enlargement is PIXEL_ART_UPSCALE and nothing else. */
static void blit_world_target(void)
{
    if (!s_worldTargetReady) return;
    const Rectangle src = { 0.0f, 0.0f,
                            (float)s_worldTarget.texture.width,
                            -(float)s_worldTarget.texture.height };
    const Rectangle dst = { 0.0f, 0.0f, (float)SCREEN_W, (float)SCREEN_H };
    DrawTexturePro(s_worldTarget.texture, src, dst, (Vector2){ 0.0f, 0.0f }, 0.0f, WHITE);
}

void render_pre_reload(void)
{
    /* Release before the module is swapped: the handles belong to this module's statics and
     * would be dangling GPU names after the reload. */
    car_sprites_unload(&s_car);
    unload_world_target();
}

void render_post_reload(void)
{
    /* Nothing to re-acquire eagerly. Clearing the key makes the next draw rebake, which is
     * both simpler and safer than baking here without a spec in hand. */
    s_carTexKey = 0u;
    s_carTexPxPerM = 0.0f;
}

void render_shutdown(void)
{
    car_sprites_unload(&s_car);
    unload_world_target();
}

/* ------------------------------------------------------------------------- gallery ----
 *
 * See render.h for the contract. Layout numbers, and why they are what they are:
 *
 *   GALLERY_COLS/ROWS   4 x 4 cells, so a cell is exactly 320 x 180 native pixels and the
 *                       grid divides SCREEN_W and SCREEN_H without a remainder.
 *   GALLERY_SCALE       2 destination pixels per texel — integer, like the world pass, so
 *                       the cars stay pixel art and are not smeared by a fractional resize.
 *   bake scale          the world scale, PIXELS_PER_METER * CAMERA_BASE_ZOOM, ONE common
 *                       value for every car. Fitting each cell to its own car would erase
 *                       the size axis, which is the single most informative thing the
 *                       gallery shows: a kei car and a bus have to look different sizes.
 *
 * At that scale the longest vehicle in the corpus (the bus, 12 m) is about 316 px, which is
 * what sets the cell width. A car wider than its cell overhangs rather than being rescaled —
 * a visible, honest overflow rather than a quiet lie about scale.
 */
#define GALLERY_COLS      4
#define GALLERY_ROWS      4
#define GALLERY_PER_PAGE  (GALLERY_COLS * GALLERY_ROWS)
#define GALLERY_SCALE     2.0f
/* Front wheels turned so lock, Ackermann and static toe are all legible in a still image. */
#define GALLERY_STEER_RAD (8.0f * DRIFTY_DEG2RAD)

int render_gallery_page_count(void)
{
    const int count = car_corpus_count();
    if (count <= 0) return 0;
    return (count + GALLERY_PER_PAGE - 1) / GALLERY_PER_PAGE;
}

void render_draw_gallery(struct Game *game, int page)
{
    const int pageCount = render_gallery_page_count();
    const float ppm = (game != NULL) ? game->renderPixelsPerMeter : (float)PIXELS_PER_METER;
    const float bakePxPerM = car_bake_px_per_m(ppm);

    if (page < 1 || page > pageCount) {
        BeginDrawing();
        ClearBackground((Color){ 21, 24, 29, 255 });
        DrawText(TextFormat("gallery page %d is out of range (1..%d)", page, pageCount),
                 24, 24, 20, COL_TEXT);
        EndDrawing();
        return;
    }

    /* ONE PAGE of sprites is uploaded at a time — sixteen cars, not the whole corpus.
     *
     * Bake-draw-discard PER CELL was the obvious shape and is wrong: raylib batches draw
     * calls and only resolves texture ids when the batch is flushed, so unloading a texture
     * between DrawTexturePro and the end of the frame leaves the batch pointing at a freed
     * id — which renders as whatever happens to be bound, in practice the default font atlas.
     * Every car came out as a block of glyphs. So: bake the page, draw the page, then release
     * once the frame has actually been submitted. */
    static CarSprites cells[GALLERY_PER_PAGE];
    static CarVisual  visuals[GALLERY_PER_PAGE];
    bool              baked[GALLERY_PER_PAGE];

    const int first = (page - 1) * GALLERY_PER_PAGE;
    const int count = car_corpus_count();

    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        const int index = first + slot;
        baked[slot] = false;
        memset(&cells[slot], 0, sizeof(cells[slot]));
        if (index >= count) continue;

        VehicleSpec spec;
        if (!car_corpus_spec(index, &spec)) continue;
        car_visual_derive(&spec, &visuals[slot]);
        baked[slot] = car_sprites_bake(&cells[slot], &visuals[slot], bakePxPerM);
    }

    const int cellW = SCREEN_W / GALLERY_COLS;
    const int cellH = SCREEN_H / GALLERY_ROWS;

    float steer[WHEEL_COUNT];
    for (int w = 0; w < WHEEL_COUNT; w++) {
        const bool isFront = (w == WHEEL_FRONT_LEFT || w == WHEEL_FRONT_RIGHT);
        steer[w] = isFront ? GALLERY_STEER_RAD : 0.0f;
    }

    BeginDrawing();
    ClearBackground((Color){ 21, 24, 29, 255 });

    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        const int index = first + slot;
        if (index >= count) break;

        const int col = slot % GALLERY_COLS;
        const int row = slot / GALLERY_COLS;
        const int cellX = col * cellW;
        const int cellY = row * cellH;

        /* Cell plate, so a car with a dark palette still reads against the background. */
        DrawRectangle(cellX + 2, cellY + 2, cellW - 4, cellH - 4, (Color){ 29, 33, 40, 255 });

        if (baked[slot]) {
            const Vector2 centrePx = { (float)cellX + (float)cellW * 0.5f,
                                       (float)cellY + (float)cellH * 0.5f - 8.0f };
            car_sprites_draw(&cells[slot], &visuals[slot], centrePx, 0.0f,
                             steer, GALLERY_SCALE);
        }

        char id[128], note[192];
        car_corpus_id(index, id, sizeof(id));
        car_corpus_describe(index, note, sizeof(note));
        DrawText(id, cellX + 8, cellY + cellH - 34, 10, COL_COOL);
        DrawText(note, cellX + 8, cellY + cellH - 20, 10, COL_TEXT_DIM);
    }

    DrawText(TextFormat("vehicle corpus - page %d / %d - %d vehicles - %.1f px/m x%d",
                        page, pageCount, count,
                        (double)bakePxPerM, (int)GALLERY_SCALE),
             12, 8, 12, COL_TEXT);

    EndDrawing();

    /* The frame is submitted; the ids are resolved and these are safe to release. */
    for (int slot = 0; slot < GALLERY_PER_PAGE; slot++) {
        if (baked[slot]) car_sprites_unload(&cells[slot]);
    }
}

static void draw_debug_vectors(const Game *game, const VehicleDrawState *draw)
{
    const float ppm = game->renderPixelsPerMeter;
    const float heading = draw->headingRad;
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 2.0f, 0.0f }, heading),
                      ppm, YELLOW, "");
    draw_world_vector(draw->positionM, body_vector_to_world((Vector2){ 0.0f, 1.5f }, heading),
                      ppm, SKYBLUE, "");
    const Vector2 bodyVelocity = {
        game->vehicle.velocityLongitudinalMps * 0.30f,
        game->vehicle.velocityLateralMps * 0.30f
    };
    draw_world_vector(draw->positionM, body_vector_to_world(bodyVelocity, heading),
                      ppm, GREEN, "velocity");

    const Vector2 frontM = body_point_to_world(
        (Vector2){ game->spec.cgToFrontM, 0.0f }, draw->positionM, heading);
    const Vector2 rearM = body_point_to_world(
        (Vector2){ -game->spec.cgToRearM, 0.0f }, draw->positionM, heading);
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ game->derived.frontAxleContactVelocityBodyMps.x * 0.20f,
                   game->derived.frontAxleContactVelocityBodyMps.y * 0.20f }, heading),
        ppm, ORANGE, "front v");
    draw_world_vector(rearM, body_vector_to_world(
        (Vector2){ game->derived.rearAxleContactVelocityBodyMps.x * 0.20f,
                   game->derived.rearAxleContactVelocityBodyMps.y * 0.20f }, heading),
        ppm, PURPLE, "rear v");
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ cosf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f,
                   sinf(draw->wheelAngleRad[WHEEL_FRONT_LEFT]) * 1.4f }, heading),
        ppm, GOLD, "front heading");
    draw_world_vector(frontM, body_vector_to_world(
        (Vector2){ game->derived.frontBodyForceN.x * 0.00008f,
                   game->derived.frontBodyForceN.y * 0.00008f }, heading),
        ppm, RED, "front force");
    draw_world_vector(rearM, body_vector_to_world(
        (Vector2){ game->derived.rearBodyForceN.x * 0.00008f,
                   game->derived.rearBodyForceN.y * 0.00008f }, heading),
        ppm, MAROON, "rear force");
}

static const char *gear_label(int selectedGear);

/* ---- tire smoke ------------------------------------------------------------------
 * Each active particle renders as a soft puff: three overlapping translucent circles
 * whose offsets come from a deterministic per-slot wobble (no per-frame randomness).
 * Colour drifts from a dark rubber-grey when fresh at the tire to near-white as it
 * disperses, and alpha falls off quadratically with age so the trail dissolves softly.
 * Cost: 3 DrawCircleV per active particle, pool cap 512 — comfortably cheap.
 */
static void draw_smoke_particles(const Game *game)
{
    const float ppm = game->renderPixelsPerMeter;
    for (int i = 0; i < MAX_PARTICLES; i++) {
        const Particle *p = &game->particles.particles[i];
        if (!p->active) continue;

        const float maxLifeS = (p->maxLifeS > 0.0f) ? p->maxLifeS : 1.0f;
        const float t = clampf(1.0f - p->lifeS / maxLifeS, 0.0f, 1.0f);  /* 0 fresh -> 1 gone */

        const unsigned char cr = (unsigned char)lerpf(104.0f, 236.0f, t);
        const unsigned char cg = (unsigned char)lerpf( 96.0f, 236.0f, t);
        const unsigned char cb = (unsigned char)lerpf( 88.0f, 240.0f, t);
        const float fade = (1.0f - t) * (1.0f - t);
        const float baseA = (float)p->color.a * fade;

        const Vector2 px = units_world_to_render_px(p->positionM, ppm);
        const float radiusPx = p->sizeM * ppm * 0.5f * (1.0f + t * 1.1f);

        /* Two fixed wobble directions per pool slot, slowly swirling as the puff ages. */
        const float wobA = (float)(((unsigned)i * 2654435761u) >> 16 & 0xFF) / 255.0f *
                           6.2831853f + t * 1.7f;
        const float wobB = wobA + 2.4f;
        const Vector2 offA = { cosf(wobA) * radiusPx * 0.45f, sinf(wobA) * radiusPx * 0.45f };
        const Vector2 offB = { cosf(wobB) * radiusPx * 0.50f, sinf(wobB) * radiusPx * 0.50f };

        /* Outer puffs first (larger, fainter), core last. */
        DrawCircleV((Vector2){ px.x + offA.x, px.y + offA.y }, radiusPx * 1.35f,
                    (Color){ cr, cg, cb, (unsigned char)(baseA * 0.36f) });
        DrawCircleV((Vector2){ px.x + offB.x, px.y + offB.y }, radiusPx * 1.05f,
                    (Color){ cr, cg, cb, (unsigned char)(baseA * 0.28f) });
        DrawCircleV(px, radiusPx,
                    (Color){ cr, cg, cb, (unsigned char)(baseA * 0.85f) });
    }
}

/* ---- full-screen overlays (STATE_MENU / STATE_PAUSED / STATE_RESULTS) ----------------
 * Pure screen space: called after EndMode2D so the camera transform never touches them.
 * Copy is deliberately short and direct; wording is up for review.
 */
static void draw_overlay_menu(const Game *game)
{
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);

    draw_text_centered_shadow("DRIFTY", 226, 64, COL_ACCENT);
    draw_text_centered("a tiny top-down drift sandbox", 306, 20, COL_TEXT_DIM);

    draw_text_centered("PRESS P TO START", 414, 24,
                       (Color){ COL_TEXT.r, COL_TEXT.g, COL_TEXT.b,
                                pulse_alpha(0.6f, 90, 255) });
    draw_text_centered("W/S throttle & brake    A/D steer    SPACE handbrake    "
                       "Q/E shift    P pause    R reset",
                       466, 16, COL_TEXT_DIM);

    if (game->bestScore > 0.0f) {
        draw_text_centered(TextFormat("BEST  %.0f", (double)game->bestScore),
                           528, 18, COL_COOL);
    }
}

static void draw_overlay_paused(const Game *game)
{
    (void)game;
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, (Color){ COL_DIM_SCREEN.r, COL_DIM_SCREEN.g,
                                                     COL_DIM_SCREEN.b, 150 });
    draw_text_centered_shadow("PAUSED", 300, 48, COL_TEXT);
    draw_text_centered("P resume    R reset", 372, 18, COL_TEXT_DIM);
}

static void draw_overlay_results(const Game *game)
{
    DrawRectangle(0, 0, SCREEN_W, SCREEN_H, COL_DIM_SCREEN);

    /* bestScore is raised to driftScore on entering STATE_RESULTS, so equality here
     * means this run set (or matched) the best; guard against the 0/0 first-boot case. */
    const bool newBest = game->driftScore >= game->bestScore && game->driftScore > 0.0f;

    draw_text_centered_shadow("RUN COMPLETE", 196, 40, COL_TEXT);

    /* The payoff: the final drift score, biggest thing on the screen. */
    draw_text_centered_shadow(TextFormat("%.0f", (double)game->driftScore),
                              286, 56, COL_ACCENT);
    draw_text_centered("DRIFT SCORE", 352, 16, COL_TEXT_DIM);

    draw_text_centered(TextFormat("BEST  %.0f", (double)game->bestScore),
                       398, 20, COL_COOL);
    if (newBest) {
        draw_text_centered("NEW BEST!", 436, 24,
                           (Color){ COL_ACCENT_WARM.r, COL_ACCENT_WARM.g, COL_ACCENT_WARM.b,
                                    pulse_alpha(0.9f, 120, 255) });
    }

    draw_text_centered("P drive again    R menu", 504, 18, COL_TEXT_DIM);
}

/* ---- arcade HUD clusters -------------------------------------------------------------
 * Three clusters with clear hierarchy, each on a translucent panel so it reads against
 * any track background:
 *   speed  - bottom-left: km/h large, gear, rpm bar. The most-glanced readout.
 *   score  - top-right:   drift score prominent, combo in accent gold (larger while a
 *                         drift is scoring), DRIFT! callout, best score small.
 *   lap    - top-center:  lap count, running timer, checkpoint progress.
 */
static void draw_hud_clusters(const Game *game)
{
    /* ---- speed cluster (bottom-left) ---- */
    {
        const Rectangle panel = { 18.0f, SCREEN_H - 168.0f, 244.0f, 140.0f };
        draw_hud_panel(panel);

        const float kmh = game->derived.speedMps * 3.6f;
        const char *kmhText = TextFormat("%.0f", (double)kmh);
        DrawText(kmhText, (int)panel.x + 16, (int)panel.y + 12, 46, COL_TEXT);
        DrawText("KM/H", (int)panel.x + 20 + MeasureText(kmhText, 46),
                 (int)panel.y + 40, 16, COL_TEXT_DIM);

        const char *modeLabel = game->autoTrans.enabled ? "AUTO " : "";
        DrawText(TextFormat("%sGEAR %s", modeLabel, gear_label(game->vehicle.selectedGear)),
                 (int)panel.x + 16, (int)panel.y + 66, 18, COL_TEXT);

        /* RPM bar: cool cyan, flipping to accent gold near the redline. */
        const float idleRpm = game->spec.engineIdleRpm;
        const float redlineRpm = game->spec.engineRedlineRpm;
        const float rpmFrac = clampf((game->vehicle.engineRpm - idleRpm) /
                                     (redlineRpm - idleRpm), 0.0f, 1.0f);
        const Rectangle barBg = { panel.x + 16.0f, panel.y + 98.0f,
                                  panel.width - 32.0f, 10.0f };
        DrawRectangleRec(barBg, (Color){ 255, 255, 255, 22 });
        DrawRectangleRec((Rectangle){ barBg.x, barBg.y, barBg.width * rpmFrac, barBg.height },
                         (rpmFrac > 0.85f) ? COL_ACCENT : COL_COOL);
        DrawText(TextFormat("%.0f RPM", (double)game->vehicle.engineRpm),
                 (int)panel.x + 16, (int)panel.y + 114, 12, COL_TEXT_DIM);
    }

    /* ---- score cluster (top-right) ---- */
    {
        const Rectangle panel = { SCREEN_W - 266.0f, 16.0f, 248.0f, 122.0f };
        const int right = (int)(panel.x + panel.width) - 16;
        draw_hud_panel(panel);

        DrawText("SCORE", (int)panel.x + 16, (int)panel.y + 10, 14, COL_TEXT_DIM);
        DrawText(TextFormat("%.0f", (double)game->driftScore),
                 (int)panel.x + 16, (int)panel.y + 26, 34, COL_TEXT);
        DrawText(TextFormat("BEST %.0f", (double)game->bestScore),
                 (int)panel.x + 16, (int)panel.y + 68, 14, COL_COOL);

        /* Combo: prominent in gold while a drift is scoring, quiet otherwise. */
        const bool drifting = game->derived.scoringDrift;
        const char *comboText = TextFormat("x%.1f", (double)game->comboMultiplier);
        const int comboSize = drifting ? 30 : 18;
        DrawText(comboText, right - MeasureText(comboText, comboSize),
                 (int)panel.y + (drifting ? 22 : 30), comboSize,
                 drifting ? COL_ACCENT : COL_TEXT_DIM);
        if (drifting) {
            const char *callout = "DRIFT!";
            DrawText(callout, right - MeasureText(callout, 16),
                     (int)panel.y + 58, 16,
                     (Color){ COL_ACCENT_WARM.r, COL_ACCENT_WARM.g, COL_ACCENT_WARM.b,
                              pulse_alpha(1.2f, 110, 255) });
        }
    }

    /* ---- lap cluster (top-center) ---- */
    {
        const Rectangle panel = { (SCREEN_W - 360.0f) * 0.5f, 16.0f, 360.0f, 56.0f };
        draw_hud_panel(panel);

        int shownLap = game->track.lap + 1;
        if (shownLap > RESULTS_TARGET_LAPS) shownLap = RESULTS_TARGET_LAPS;
        DrawText(TextFormat("LAP %d/%d", shownLap, RESULTS_TARGET_LAPS),
                 (int)panel.x + 16, (int)panel.y + 18, 18, COL_TEXT);

        const int minutes = (int)(game->track.lapTimerS / 60.0f);
        const float seconds = game->track.lapTimerS - (float)minutes * 60.0f;
        const char *timerText = TextFormat("%d:%05.2f", minutes, (double)seconds);
        DrawText(timerText,
                 (int)(panel.x + (panel.width - (float)MeasureText(timerText, 22)) * 0.5f),
                 (int)panel.y + 16, 22, COL_TEXT);

        const char *cpText = TextFormat("CP %d/%d", game->track.nextCheckpoint,
                                        game->track.count);
        DrawText(cpText, (int)(panel.x + panel.width) - 16 - MeasureText(cpText, 18),
                 (int)panel.y + 18, 18, COL_TEXT_DIM);
    }

    /* Hot-reload notice, top-left: preserves the information the old HUD line carried. */
    if (game->reloadFlashTimerS > 0.0f) {
        DrawText("module reloaded - state preserved", 18, 18, 14, COL_ACCENT);
    }

    DrawText("W throttle  S brake  Space handbrake  Q/E shift  A/D steer  R reset  "
             "F1 diagnostics  F2 physics lab",
             (SCREEN_W - MeasureText("W throttle  S brake  Space handbrake  Q/E shift  "
                                     "A/D steer  R reset  F1 diagnostics  F2 physics lab",
                                     14)) / 2,
             SCREEN_H - 26, 14, COL_TEXT_DIM);
}

static void hud_line(int x, int *y, const char *text, Color color)
{
    DrawText(text, x, *y, 16, color);
    *y += 19;
}

static Vector2 plot_point(Rectangle bounds, float x, float xMin, float xMax,
                          float y, float yMin, float yMax)
{
    return (Vector2){
        bounds.x + (x - xMin) / (xMax - xMin) * bounds.width,
        bounds.y + bounds.height - (y - yMin) / (yMax - yMin) * bounds.height
    };
}

static void draw_curve_axes(Rectangle bounds, float xMin, float xMax,
                            float yMin, float yMax)
{
    DrawRectangleRec(bounds, (Color){ 16, 18, 22, 235 });
    DrawRectangleLinesEx(bounds, 1.0f, (Color){ 100, 106, 116, 255 });
    DrawLineV(plot_point(bounds, xMin, xMin, xMax, 0.0f, yMin, yMax),
              plot_point(bounds, xMax, xMin, xMax, 0.0f, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
    DrawLineV(plot_point(bounds, 0.0f, xMin, xMax, yMin, yMin, yMax),
              plot_point(bounds, 0.0f, xMin, xMax, yMax, yMin, yMax),
              (Color){ 80, 84, 92, 255 });
}

static void draw_tire_curve_panel(const Game *game)
{
    const Rectangle lateral = { SCREEN_W - 390.0f, 26.0f, 365.0f, 145.0f };
    const Rectangle longitudinal = { SCREEN_W - 390.0f, 205.0f, 365.0f, 145.0f };
    const float latMin = -0.55f;
    const float latMax = 0.55f;
    const float forceMin = -1.4f;
    const float forceMax = 1.4f;
    draw_curve_axes(lateral, latMin, latMax, forceMin, forceMax);
    DrawText("LATERAL  normalized force / wheel load", (int)lateral.x,
             (int)lateral.y - 19, 14, RAYWHITE);
    DrawText("front", (int)lateral.x + 6, (int)lateral.y + 5, 12, ORANGE);
    DrawText("rear", (int)lateral.x + 52, (int)lateral.y + 5, 12, SKYBLUE);
    Vector2 prevFront = { 0.0f, 0.0f };
    Vector2 prevRear = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(latMin, latMax, (float)i / 120.0f);
        const float front = -game->spec.tireMuLatFront * tire_normalized_curve(
            game->spec.tireBLatFront, game->spec.tireCLatFront, slip);
        const float rear = -game->spec.tireMuLatRear * tire_normalized_curve(
            game->spec.tireBLatRear, game->spec.tireCLatRear, slip);
        const Vector2 pFront = plot_point(
            lateral, slip, latMin, latMax, front, forceMin, forceMax);
        const Vector2 pRear = plot_point(
            lateral, slip, latMin, latMax, rear, forceMin, forceMax);
        if (i > 0) {
            DrawLineV(prevFront, pFront, ORANGE);
            DrawLineV(prevRear, pRear, SKYBLUE);
        }
        prevFront = pFront;
        prevRear = pRear;
    }
    const float currentFront = -game->spec.tireMuLatFront * tire_normalized_curve(
        game->spec.tireBLatFront, game->spec.tireCLatFront,
        game->derived.frontSlipAngleRad);
    const float currentRear = -game->spec.tireMuLatRear * tire_normalized_curve(
        game->spec.tireBLatRear, game->spec.tireCLatRear,
        game->derived.rearSlipAngleRad);
    DrawCircleV(plot_point(lateral, game->derived.frontSlipAngleRad, latMin, latMax,
                           currentFront, forceMin, forceMax), 4.0f, ORANGE);
    DrawCircleV(plot_point(lateral, game->derived.rearSlipAngleRad, latMin, latMax,
                           currentRear, forceMin, forceMax), 4.0f, SKYBLUE);

    const float longMin = -1.25f;
    const float longMax = 1.25f;
    draw_curve_axes(longitudinal, longMin, longMax, -1.1f, 1.1f);
    DrawText("LONGITUDINAL  normalized force / wheel load",
             (int)longitudinal.x, (int)longitudinal.y - 19, 14, RAYWHITE);
    Vector2 previous = { 0.0f, 0.0f };
    for (int i = 0; i <= 120; i++) {
        const float slip = lerpf(longMin, longMax, (float)i / 120.0f);
        const float force = game->spec.tireMuLongScale * tire_normalized_curve(
            game->spec.tireBLong, game->spec.tireCLong, slip);
        const Vector2 point = plot_point(
            longitudinal, slip, longMin, longMax, force, -1.1f, 1.1f);
        if (i > 0) DrawLineV(previous, point, LIME);
        previous = point;
    }
    const float rearSlip = game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio;
    const float rearLong = game->spec.tireMuLongScale * tire_normalized_curve(
        game->spec.tireBLong, game->spec.tireCLong, rearSlip);
    DrawCircleV(plot_point(longitudinal, rearSlip, longMin, longMax,
                           rearLong, -1.1f, 1.1f), 4.0f, YELLOW);
    DrawText("zero axes; curve peaks are the configured friction references",
             (int)longitudinal.x, (int)longitudinal.y + (int)longitudinal.height + 5,
             11, (Color){ 155, 160, 170, 255 });
}

static const char *gear_label(int selectedGear)
{
    if (selectedGear < 0) return "R";
    if (selectedGear == 0) return "N";
    static const char *labels[MAX_GEARS] = { "1", "2", "3", "4", "5", "6", "7", "8" };
    if (selectedGear <= MAX_GEARS) return labels[selectedGear - 1];
    return "?";
}

void render_draw_game(struct Game *game, float interpolationAlpha)
{
    if (game == NULL) return;

    /* The gallery is a whole-screen mode, not an overlay: no world, no HUD, no simulation.
     * It is selected through the DevState field Phase 2 reserved for it rather than a new
     * GAME_ENTRY_POINTS function, so the platform layer can ask for a page without changing
     * the module's ABI or the layout of anything persistent. */
    if (game->dev.galleryPage > 0) {
        render_draw_gallery(game, game->dev.galleryPage);
        return;
    }

    /* Development shortcuts and the status-line timer, once per render frame. Compiles to
     * nothing when the dev tools are not built in. */
    dev_lab_update(game);

    DRIFTY_ZONE_BEGIN(render, "Render");
    const float renderDt = GetFrameTime();
    if (game->reloadFlashTimerS > 0.0f) {
        game->reloadFlashTimerS = fmaxf(0.0f, game->reloadFlashTimerS - renderDt);
    }
    const float alpha = clampf(interpolationAlpha, 0.0f, 1.0f);
    const VehicleDrawState draw = render_interpolate_vehicle(&game->renderState, alpha);
    game->camera.target = units_world_to_render_px(draw.positionM, game->renderPixelsPerMeter);

    /* Camera drift zoom: zoom out during a drift proportional to body sideslip.
     * Smoothed against render delta time and independent of physics determinism. */
    {
        const float driftIntensity = clampf(
            fabsf(game->derived.bodySideslipRad) / DRIFT_ZOOM_REF_RAD, 0.0f, 1.0f);
        float targetZoom = CAMERA_BASE_ZOOM - driftIntensity * CAMERA_ZOOM_RANGE;
        targetZoom = fmaxf(targetZoom, CAMERA_MIN_ZOOM);
        game->camera.zoom = smooth_to(game->camera.zoom, targetZoom,
                                      CAMERA_ZOOM_RATE, renderDt);
    }

    ensure_world_target();

    /* ---- the world, at pixel-art resolution -------------------------------------------
     *
     * Everything inside the camera goes into the low-resolution target and is enlarged once
     * by an exact integer factor. Everything after the blit — HUD, raygui, Physics Lab — is
     * drawn at native resolution, because text through a nearest-neighbour upscale is
     * unreadable. If the target could not be created the world draws straight to the window;
     * a missing render texture should cost the pixel-art look, not the game. */
    const bool pixelArt = s_worldTargetReady;
    const Camera2D worldCam = pixelArt ? world_camera_for_target(game->camera) : game->camera;

    if (pixelArt) {
        BeginTextureMode(s_worldTarget);
        ClearBackground((Color){ 22, 24, 28, 255 });
    } else {
        BeginDrawing();
        ClearBackground((Color){ 22, 24, 28, 255 });
    }

    BeginMode2D(worldCam);
    render_draw_track(&game->track, game->renderPixelsPerMeter);

    /* ---- particles (between track and car, per the spec draw order) ---- */
    draw_smoke_particles(game);

    draw_vehicle(game, &draw);
    if (game->debugOverlay) draw_debug_vectors(game, &draw);
    dev_lab_draw_world(game, &draw);
    EndMode2D();

    if (pixelArt) {
        EndTextureMode();
        BeginDrawing();
        ClearBackground((Color){ 22, 24, 28, 255 });
        blit_world_target();
    }

    /* ---- raw physics diagnostics (F1) -------------------------------------------------
     * The development readout. Everything from here to the tire-curve panel draws only
     * when the debug overlay is on; the always-on presentation is the arcade HUD below.
     * Lines are unchanged from the previous always-on stack, just gated. */
    if (game->debugOverlay) {
    const Color label = (Color){ 235, 235, 235, 255 };
    const Color dim = (Color){ 155, 160, 170, 255 };
    int y = 12;
    hud_line(14, &y, "DRIFTY diagnostics", label);
    hud_line(14, &y, TextFormat("pos (%+.2f,%+.2f) m  heading %+.3f rad",
             (double)game->vehicle.positionM.x, (double)game->vehicle.positionM.y,
             (double)game->vehicle.headingRad), dim);
    hud_line(14, &y, TextFormat("vx %+.3f m/s  vy %+.3f m/s  speed %.3f m/s  yaw %+.3f rad/s",
             (double)game->vehicle.velocityLongitudinalMps,
             (double)game->vehicle.velocityLateralMps, (double)game->derived.speedMps,
             (double)game->vehicle.yawRateRadS), label);
    hud_line(14, &y, TextFormat("steer %+.3f  slip F/R %+.3f / %+.3f rad  sideslip %+.3f",
             (double)game->vehicle.frontRoadWheelAngleRad,
             (double)game->derived.frontSlipAngleRad,
             (double)game->derived.rearSlipAngleRad,
             (double)game->derived.bodySideslipRad), dim);
    hud_line(14, &y, TextFormat("gear %s  engine %.0f rpm  rear drive %+.1f Nm",
             gear_label(game->vehicle.selectedGear), (double)game->vehicle.engineRpm,
             (double)game->derived.drivelineTorqueNm), label);
    hud_line(14, &y, TextFormat("omega FL/FR/R %.2f / %.2f / %.2f rad/s  surface R %+.2f m/s",
             (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].angularVelocityRadS,
             (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].angularVelocityRadS,
             (double)game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS,
             (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].angularVelocityRadS *
                      vehicle_wheel_radius_m(&game->spec, WHEEL_REAR_LEFT))), dim);
    hud_line(14, &y, TextFormat("kappa FL/FR/RL/RR %+.3f %+.3f %+.3f %+.3f",
             (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_REAR_LEFT].slipRatio,
             (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].slipRatio), dim);
    hud_line(14, &y, TextFormat("load F/R %.1f / %.1f N  transfer %+.1f N  lateral F/R %+.1f / %+.1f N",
             (double)game->derived.normalLoadFrontN,
             (double)game->derived.normalLoadRearN,
             (double)game->derived.loadTransferN,
             (double)game->derived.frontLateralForceN,
             (double)game->derived.rearLateralForceN), dim);
    hud_line(14, &y, TextFormat("body force (%+.1f,%+.1f) N  yaw torque %+.1f Nm  blend %.3f",
             (double)game->derived.totalBodyForceN.x,
             (double)game->derived.totalBodyForceN.y,
             (double)game->derived.totalYawTorqueNm,
             (double)game->derived.lowSpeedBlend), dim);
    hud_line(14, &y, TextFormat("substeps %d  backlog %d  alpha %.3f  tick %llu  checksum %08x",
             game->lastSubstepCount, game->physicsBacklogDrops, (double)alpha,
             (unsigned long long)game->sim.tick, game->stateChecksum), label);
    hud_line(14, &y, TextFormat("reloads %d%s  gear %s  render %.1f px/m",
             game->reloadCount, game->reloadFlashTimerS > 0.0f ? " (state preserved)" : "",
             gear_label(game->vehicle.selectedGear),
             (double)game->renderPixelsPerMeter), dim);
    hud_line(14, &y, TextFormat("Lap: %d  Timer: %d:%05.2f  Checkpoint: %d / %d",
             game->track.lap, (int)(game->track.lapTimerS / 60.0f),
             (double)(game->track.lapTimerS -
                      (float)(int)(game->track.lapTimerS / 60.0f) * 60.0f),
             game->track.nextCheckpoint, game->track.count), label);
    hud_line(14, &y, TextFormat("Score: %.0f  Best: %.0f  Combo: x%.1f  Drift: %.2fs%s",
             (double)game->driftScore, (double)game->bestScore,
             (double)game->comboMultiplier, (double)game->driftTimeS,
             game->derived.scoringDrift ? "  DRIFT!" : ""), dim);

    {
        hud_line(14, &y, TextFormat("pure Fx F/R %+.0f/%+.0f  pure Fy F/R %+.0f/%+.0f N",
                 (double)(game->derived.pureLongitudinalForceN[WHEEL_FRONT_LEFT] +
                          game->derived.pureLongitudinalForceN[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.pureLongitudinalForceN[WHEEL_REAR_LEFT] +
                          game->derived.pureLongitudinalForceN[WHEEL_REAR_RIGHT]),
                 (double)(game->derived.pureLateralForceN[WHEEL_FRONT_LEFT] +
                          game->derived.pureLateralForceN[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.pureLateralForceN[WHEEL_REAR_LEFT] +
                          game->derived.pureLateralForceN[WHEEL_REAR_RIGHT])), dim);
        hud_line(14, &y, TextFormat("limited Fx F/R %+.0f/%+.0f  limited Fy F/R %+.0f/%+.0f N",
                 (double)(game->vehicle.wheels[WHEEL_FRONT_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_FRONT_RIGHT].forceLongitudinalN),
                 (double)(game->vehicle.wheels[WHEEL_REAR_LEFT].forceLongitudinalN +
                          game->vehicle.wheels[WHEEL_REAR_RIGHT].forceLongitudinalN),
                 (double)game->derived.frontLateralForceN,
                 (double)game->derived.rearLateralForceN), dim);
        hud_line(14, &y, TextFormat("usage FL/FR/RL/RR %.2f %.2f %.2f %.2f  lock %d%d%d%d",
                 (double)game->vehicle.wheels[WHEEL_FRONT_LEFT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_FRONT_RIGHT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_REAR_LEFT].frictionUsage,
                 (double)game->vehicle.wheels[WHEEL_REAR_RIGHT].frictionUsage,
                 game->vehicle.wheels[WHEEL_FRONT_LEFT].locked,
                 game->vehicle.wheels[WHEEL_FRONT_RIGHT].locked,
                 game->vehicle.wheels[WHEEL_REAR_LEFT].locked,
                 game->vehicle.wheels[WHEEL_REAR_RIGHT].locked), dim);
        hud_line(14, &y, TextFormat("brake F/R %.0f/%.0f Nm  handbrake R %.0f Nm",
                 (double)(game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_LEFT] +
                          game->derived.serviceBrakeTorqueNm[WHEEL_FRONT_RIGHT]),
                 (double)(game->derived.serviceBrakeTorqueNm[WHEEL_REAR_LEFT] +
                          game->derived.serviceBrakeTorqueNm[WHEEL_REAR_RIGHT]),
                 (double)(game->derived.handbrakeTorqueNm[WHEEL_REAR_LEFT] +
                          game->derived.handbrakeTorqueNm[WHEEL_REAR_RIGHT])), dim);
        hud_line(14, &y, TextFormat("ax prev/filt/solved %+.2f/%+.2f/%+.2f m/s^2  "
                 "drag %.0f N  rolling %.0f N",
                 (double)game->derived.previousLongAccelMps2,
                 (double)game->derived.filteredLongAccelMps2,
                 (double)game->derived.solvedLongAccelMps2,
                 (double)game->derived.aeroDragMagnitudeN,
                 (double)game->derived.rollingResistanceMagnitudeN), dim);
        draw_tire_curve_panel(game);
    }
    }

    /* ---- arcade HUD + state overlays (screen space, after EndMode2D) ----------------- */
    if (game->state == STATE_PLAYING || game->state == STATE_PAUSED) {
        draw_hud_clusters(game);
    }
    switch (game->state) {
        case STATE_MENU:    draw_overlay_menu(game);    break;
        case STATE_PAUSED:  draw_overlay_paused(game);  break;
        case STATE_RESULTS: draw_overlay_results(game); break;
        default: break;
    }

    /* The lab paints over the HUD deliberately: when it is open it is what you are reading. */
    DRIFTY_ZONE_BEGIN(lab, "PhysicsLab");
    dev_lab_draw_ui(game);
    DRIFTY_ZONE_END(lab);

    EndDrawing();
    DRIFTY_ZONE_END(render);
    DRIFTY_FRAME_MARK();
}
#endif
