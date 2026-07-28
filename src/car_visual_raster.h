/*
 * car_visual_raster.h — CPU rasterizer: CarVisual -> RGBA8 pixels.
 *
 * ONE RASTERIZER, TWO CONSUMERS. drifty_tests composites these buffers into the vehicle
 * contact sheet with no GPU and no window; src/render.c uploads the same buffer as a texture
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
 * decreasing pixel Y — the same mapping as src/units.h. A consumer that wants the nose-up
 * orientation of the reference sheets rotates the finished buffer by an exact 90 degrees,
 * which is lossless.
 */
#ifndef DRIFTY_CAR_VISUAL_RASTER_H
#define DRIFTY_CAR_VISUAL_RASTER_H

#include <stdbool.h>
#include <stddef.h>

#include "car_visual.h"

/* Geometry of a rasterization: the buffer size, the scale, and where body-space (0,0) lands.
 * Produced by car_raster_info() so a caller can allocate before drawing. */
typedef struct {
    int   width;
    int   height;
    float pxPerM;
    float originXPx;
    float originYPx;
} CarRasterInfo;

/* Bytes per pixel of every buffer this module writes. */
#define CAR_RASTER_BPP 4

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
bool car_raster_draw(const CarVisual *visual, CarRasterInfo info,
                     unsigned char *rgba, size_t bytes);

/* Rotate a buffer 90 degrees counter-clockwise into `dst`, so the nose points up as in
 * resources/sprite_examples/. dst must hold height*width*CAR_RASTER_BPP bytes; its dimensions
 * are the source's transposed. Exact and lossless — this is an index permutation, not a
 * resample. */
bool car_raster_rotate_nose_up(const unsigned char *src, int srcW, int srcH,
                               unsigned char *dst, size_t dstBytes);

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
    CAR_LABEL_TIRE,
    CAR_LABEL_RIM,
    CAR_LABEL_DISC,
    CAR_LABEL_ARCH,
    CAR_LABEL_WING,
    CAR_LABEL_SPLITTER,
    CAR_LABEL_MIRROR,
    CAR_LABEL_EXHAUST,
    CAR_LABEL_LAMP,
    CAR_LABEL_CAGE,
    CAR_LABEL_OUTLINE,
    CAR_LABEL_COUNT
} CarRasterLabel;

/* One byte per pixel. `labels` must hold at least info.width * info.height bytes. */
bool car_raster_draw_labels(const CarVisual *visual, CarRasterInfo info,
                            unsigned char *labels, size_t bytes);

/* Fraction of the union of two label maps whose labels disagree, in [0,1]. The authoritative
 * "are these two cars visibly different" measure, used by the `corpus` scenario. Both maps
 * must share dimensions. Returns 0 when the inputs do not agree, which fails a distinctness
 * assertion safely rather than passing it by accident. */
float car_raster_difference(const unsigned char *labelsA, const unsigned char *labelsB,
                            int width, int height);

#endif /* DRIFTY_CAR_VISUAL_RASTER_H */
