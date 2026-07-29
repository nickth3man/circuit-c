/*
 * car_visual_raster.h — CPU rasterizer: CarVisual -> RGBA8 pixels.
 *
 * ONE RASTERIZER, TWO CONSUMERS. drifty_tests composites these buffers into the vehicle
 * contact sheet with no GPU and no window; src/render/render.c uploads the same buffer as a texture
 * and draws it rotated with point filtering. Because both read the identical pixels, the
 * gallery cannot drift away from what the game shows.
 *
 * Raylib-free, like car_visual.c: raylib.h is included for the Color and Vector2 types only.
 *
 * PIXEL ART, NOT VECTOR ART. Fills are hard-edged — a pixel is inside a shape or it is not.
 * No anti-aliasing, matching resources/sprite_examples/. Sub-pixel features are therefore
 * genuinely invisible, which is why car_visual.c amplifies the ones that matter rather than
 * relying on the rasterizer to hint them.
 *
 * FRAME. Body space, +X forward drawn toward increasing pixel X, +Y left drawn toward
 * decreasing pixel Y — the same mapping as src/core/units.h. A consumer that wants the nose-up
 * orientation of the reference sheets rotates the finished buffer by an exact 90 degrees,
 * which is lossless.
 */
#ifndef DRIFTY_CAR_VISUAL_RASTER_H
#define DRIFTY_CAR_VISUAL_RASTER_H

#include <stdbool.h>
#include <stddef.h>

#include "render/car_visual.h"

/* Geometry of a rasterization: the buffer size, the scale, and where body-space (0,0) lands.
 * Produced by car_raster_info() so a caller can allocate before drawing. */
typedef struct {
    int width;
    int height;
    float pxPerM;
    float originXPx;
    float originYPx;
} CarRasterInfo;

/* Bytes per pixel of every buffer this module writes. */
#define CAR_RASTER_BPP 4

/* ------------------------------------------------------------------ composable parts ----
 *
 * The contact sheet wants one finished picture; the game wants the body and each wheel as
 * separate sprites, so the front wheels can be rotated to the steer angle every frame while
 * the body is baked once. Both come out of the SAME geometry pass, so there is no second
 * grammar and no way for the two to drift apart.
 *
 * The split is at the joint that actually moves. Wheel ARCHES are bodywork — a fender does
 * not steer — so they stay with the body; the tire, sidewall, rim and disc travel with the
 * wheel. Composited body-then-wheels at zero steer, the result is the same picture
 * CAR_RASTER_PART_ALL draws, in the same order.
 *
 * PIVOTS, which the caller must respect:
 *   ALL / BODY   body-space (0,0) — the CG — sits at (originXPx, originYPx). Rotate about
 *                that point by the vehicle heading.
 *   WHEEL        the wheel's own hub sits at (originXPx, originYPx), and the sprite is drawn
 *                axis-aligned: the derived static toe/camber angle is NOT baked in. Rotate
 *                about the hub by heading + steer + CarWheelVisual.staticAngleRad. Baking the
 *                static angle in would make it impossible to steer the wheel afterwards
 *                without double-counting it.
 */
typedef enum {
    CAR_RASTER_PART_ALL = 0, /* the whole car, wheels in place at their static angles */
    CAR_RASTER_PART_BODY,    /* everything except the tires: hull, glass, arches, appendages */
    CAR_RASTER_PART_WHEEL    /* one wheel, about its hub, unrotated */
} CarRasterPart;

/* Size the buffer for one part. For CAR_RASTER_PART_WHEEL, wheelIndex selects the wheel;
 * it is ignored otherwise. BODY deliberately uses the same bounds as ALL: the wheel arches
 * are body geometry and reach past the hull wherever the track does, so the car's bounds are
 * the body's bounds. */
CarRasterInfo car_raster_part_info(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                                   float pxPerM, int padPx);

/* Draw one part. Same contract as car_raster_draw otherwise. */
bool car_raster_draw_part(const CarVisual *visual, CarRasterPart part, int wheelIndex,
                          CarRasterInfo info, unsigned char *rgba, size_t bytes);

/* Size the buffer needed to draw `visual` at `pxPerM`, with `padPx` pixels of margin on every
 * side. Accounts for everything actually drawn, including wheels outboard of the hull, the
 * wing, the splitter and the mirrors. Returns a zero-sized info for a NULL or degenerate
 * input, which every other function here treats as a no-op. */
CarRasterInfo car_raster_info(const CarVisual *visual, float pxPerM, int padPx);

/* Bytes required for an info: width * height * CAR_RASTER_BPP. */
size_t car_raster_bytes(CarRasterInfo info);

/* Draw `visual` into `rgba`, which must hold at least car_raster_bytes(info). The buffer is
 * cleared to fully transparent first. Returns false if the arguments do not agree.
 *
 * Deterministic: the same visual and info always produce byte-identical output within a
 * binary. Do not compare buffers produced by two DIFFERENT binaries — game.dll builds at -O0
 * and drifty_tests at -O2, so float results may differ in the last bit. */
bool car_raster_draw(const CarVisual *visual, CarRasterInfo info, unsigned char *rgba,
                     size_t bytes);

/* Rotate a buffer 90 degrees counter-clockwise into `dst`, so the nose points up as in
 * resources/sprite_examples/. dst must hold height*width*CAR_RASTER_BPP bytes; its dimensions
 * are the source's transposed. Exact and lossless — this is an index permutation, not a
 * resample. */
bool car_raster_rotate_nose_up(const unsigned char *src, int srcW, int srcH, unsigned char *dst,
                               size_t dstBytes);

/* ------------------------------------------------------------------- feature labels ----
 *
 * A second rasterization mode that writes one byte of FEATURE IDENTITY per pixel instead of a
 * colour. This is what the distinctness test compares.
 *
 * Why not compare colours or luminance: colour in this project is arbitrary (see
 * car_visual_colour_seed), so two cars of identical shape in different paint would "differ"
 * on any colour-based metric and pass a distinctness test they should fail. A label map is
 * colour-blind by construction — it changes only when the geometry or the arrangement of
 * features changes, which is exactly the property being asserted. */
typedef enum {
    CAR_LABEL_EMPTY = 0,
    CAR_LABEL_BODY,
    CAR_LABEL_BODY_SHADE,
    CAR_LABEL_CABIN,
    CAR_LABEL_GLASS,
    CAR_LABEL_QUARTER_WINDOW,
    CAR_LABEL_SUNROOF,
    CAR_LABEL_TIRE,
    CAR_LABEL_TIRE_SIDEWALL, /* sidewall ring between tread and rim */
    CAR_LABEL_RIM,
    CAR_LABEL_DISC,
    CAR_LABEL_ARCH,
    CAR_LABEL_WING,
    CAR_LABEL_SPLITTER,
    CAR_LABEL_CANARD, /* small front-corner fins */
    CAR_LABEL_MIRROR,
    CAR_LABEL_EXHAUST,
    CAR_LABEL_BED,        /* pickup bed floor and rails */
    CAR_LABEL_HOOD_BULGE, /* engine hood bulge */
    CAR_LABEL_TOW_HOOK,   /* race-detail tow-hook marker */
    CAR_LABEL_HOOD_PINS,  /* race-detail hood-pin markers */
    CAR_LABEL_LAMP,
    CAR_LABEL_CAGE,
    CAR_LABEL_STRIPE,  /* livery stripes */
    CAR_LABEL_HEADING, /* L9 gameplay heading marker */
    CAR_LABEL_OUTLINE,
    CAR_LABEL_SHADOW, /* L0 shadow; excluded from distinctness */
    CAR_LABEL_COUNT
} CarRasterLabel;

/* One byte per pixel. `labels` must hold at least info.width * info.height bytes. */
bool car_raster_draw_labels(const CarVisual *visual, CarRasterInfo info, unsigned char *labels,
                            size_t bytes);

/* Fraction of the union of two label maps whose labels disagree, in [0,1]. The authoritative
 * "are these two cars visibly different" measure, used by the `corpus` scenario. Both maps
 * must share dimensions. Returns 0 when the inputs do not agree, which fails a distinctness
 * assertion safely rather than passing it by accident. */
float car_raster_difference(const unsigned char *labelsA, const unsigned char *labelsB,
                            int width, int height);

#endif /* DRIFTY_CAR_VISUAL_RASTER_H */
