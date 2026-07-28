/*
 * car_sheet.c — headless paginated vehicle contact sheet. See the header for the contract.
 *
 * This is the only translation unit that defines STB_IMAGE_WRITE_IMPLEMENTATION, and it is
 * compiled into drifty_tests.exe alone: neither the game module, drifty.exe, nor the release
 * build links stb_image_write (see third_party/README.md).
 */
#include "car_sheet.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_corpus.h"
#include "car_visual.h"
#include "car_visual_raster.h"
#include "config.h"
#include "dev_params.h"
#include "telemetry.h"   /* telemetry_ensure_dir — the project's mkdir helper */
#include "vehicle.h"

/* stb triggers a handful of the project's stricter warnings. It is vendored verbatim and is
 * not ours to fix (third_party/README.md), so the diagnostics are silenced for this include
 * only, not for the file. */
#if defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wmissing-prototypes"
#  pragma GCC diagnostic ignored "-Wsign-compare"
#  pragma GCC diagnostic ignored "-Wshadow"
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO_DEPRECATION
#include "stb/stb_image_write.h"
#if defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif

#define SHEET_PAD_PX     2
#define SHEET_COLS       5
#define SHEET_ROWS       4
#define SHEET_GAP_PX     6
#define SHEET_MARGIN_PX  10
#define SHEET_FONT_W     5
#define SHEET_FONT_H     7
#define SHEET_LABEL_H    22
/* Characters a caption must be able to show. The cell is otherwise sized by the largest car
 * in the corpus, which is narrow enough that ids and sweep values were being cut mid-word —
 * and an unreadable caption defeats the point of a sheet a human is meant to judge. */
#define SHEET_MIN_CHARS  30

/* Opaque page / cell colours (RGBA). */
static const unsigned char kPageBg[4]  = { 0x15, 0x18, 0x1d, 0xff };
static const unsigned char kCellBg[4]  = { 0x1d, 0x21, 0x28, 0xff };
static const unsigned char kNoteFg[4]  = { 0x98, 0xa0, 0xae, 0xff };
static const unsigned char kGroupFg[4] = { 0x6e, 0xcd, 0xeb, 0xff };

float car_sheet_default_px_per_m(void)
{
    /* What a player actually sees: the world scale times the camera's resting zoom. */
    return PIXELS_PER_METER * CAMERA_BASE_ZOOM;
}

/* Minimal 5x7 glyphs for printable ASCII. Each glyph is five column bytes; bit 0 is the top
 * row. Missing code points render as a hollow box so a bad caption cannot silently vanish. */
static const unsigned char kFont5x7[95][5] = {
    /* 32 space */ {0x00,0x00,0x00,0x00,0x00},
    /* 33 !     */ {0x00,0x00,0x5F,0x00,0x00},
    /* 34 "     */ {0x00,0x07,0x00,0x07,0x00},
    /* 35 #     */ {0x14,0x7F,0x14,0x7F,0x14},
    /* 36 $     */ {0x24,0x2A,0x7F,0x2A,0x12},
    /* 37 %     */ {0x23,0x13,0x08,0x64,0x62},
    /* 38 &     */ {0x36,0x49,0x55,0x22,0x50},
    /* 39 '     */ {0x00,0x05,0x03,0x00,0x00},
    /* 40 (     */ {0x00,0x1C,0x22,0x41,0x00},
    /* 41 )     */ {0x00,0x41,0x22,0x1C,0x00},
    /* 42 *     */ {0x14,0x08,0x3E,0x08,0x14},
    /* 43 +     */ {0x08,0x08,0x3E,0x08,0x08},
    /* 44 ,     */ {0x00,0x50,0x30,0x00,0x00},
    /* 45 -     */ {0x08,0x08,0x08,0x08,0x08},
    /* 46 .     */ {0x00,0x60,0x60,0x00,0x00},
    /* 47 /     */ {0x20,0x10,0x08,0x04,0x02},
    /* 48 0     */ {0x3E,0x51,0x49,0x45,0x3E},
    /* 49 1     */ {0x00,0x42,0x7F,0x40,0x00},
    /* 50 2     */ {0x42,0x61,0x51,0x49,0x46},
    /* 51 3     */ {0x21,0x41,0x45,0x4B,0x31},
    /* 52 4     */ {0x18,0x14,0x12,0x7F,0x10},
    /* 53 5     */ {0x27,0x45,0x45,0x45,0x39},
    /* 54 6     */ {0x3C,0x4A,0x49,0x49,0x30},
    /* 55 7     */ {0x01,0x71,0x09,0x05,0x03},
    /* 56 8     */ {0x36,0x49,0x49,0x49,0x36},
    /* 57 9     */ {0x06,0x49,0x49,0x29,0x1E},
    /* 58 :     */ {0x00,0x36,0x36,0x00,0x00},
    /* 59 ;     */ {0x00,0x56,0x36,0x00,0x00},
    /* 60 <     */ {0x08,0x14,0x22,0x41,0x00},
    /* 61 =     */ {0x14,0x14,0x14,0x14,0x14},
    /* 62 >     */ {0x00,0x41,0x22,0x14,0x08},
    /* 63 ?     */ {0x02,0x01,0x51,0x09,0x06},
    /* 64 @     */ {0x32,0x49,0x79,0x41,0x3E},
    /* 65 A     */ {0x7E,0x11,0x11,0x11,0x7E},
    /* 66 B     */ {0x7F,0x49,0x49,0x49,0x36},
    /* 67 C     */ {0x3E,0x41,0x41,0x41,0x22},
    /* 68 D     */ {0x7F,0x41,0x41,0x22,0x1C},
    /* 69 E     */ {0x7F,0x49,0x49,0x49,0x41},
    /* 70 F     */ {0x7F,0x09,0x09,0x09,0x01},
    /* 71 G     */ {0x3E,0x41,0x49,0x49,0x7A},
    /* 72 H     */ {0x7F,0x08,0x08,0x08,0x7F},
    /* 73 I     */ {0x00,0x41,0x7F,0x41,0x00},
    /* 74 J     */ {0x20,0x40,0x41,0x3F,0x01},
    /* 75 K     */ {0x7F,0x08,0x14,0x22,0x41},
    /* 76 L     */ {0x7F,0x40,0x40,0x40,0x40},
    /* 77 M     */ {0x7F,0x02,0x0C,0x02,0x7F},
    /* 78 N     */ {0x7F,0x04,0x08,0x10,0x7F},
    /* 79 O     */ {0x3E,0x41,0x41,0x41,0x3E},
    /* 80 P     */ {0x7F,0x09,0x09,0x09,0x06},
    /* 81 Q     */ {0x3E,0x41,0x51,0x21,0x5E},
    /* 82 R     */ {0x7F,0x09,0x19,0x29,0x46},
    /* 83 S     */ {0x46,0x49,0x49,0x49,0x31},
    /* 84 T     */ {0x01,0x01,0x7F,0x01,0x01},
    /* 85 U     */ {0x3F,0x40,0x40,0x40,0x3F},
    /* 86 V     */ {0x1F,0x20,0x40,0x20,0x1F},
    /* 87 W     */ {0x3F,0x40,0x38,0x40,0x3F},
    /* 88 X     */ {0x63,0x14,0x08,0x14,0x63},
    /* 89 Y     */ {0x07,0x08,0x70,0x08,0x07},
    /* 90 Z     */ {0x61,0x51,0x49,0x45,0x43},
    /* 91 [     */ {0x00,0x7F,0x41,0x41,0x00},
    /* 92 \     */ {0x02,0x04,0x08,0x10,0x20},
    /* 93 ]     */ {0x00,0x41,0x41,0x7F,0x00},
    /* 94 ^     */ {0x04,0x02,0x01,0x02,0x04},
    /* 95 _     */ {0x40,0x40,0x40,0x40,0x40},
    /* 96 `     */ {0x00,0x01,0x02,0x04,0x00},
    /* 97 a     */ {0x20,0x54,0x54,0x54,0x78},
    /* 98 b     */ {0x7F,0x48,0x44,0x44,0x38},
    /* 99 c     */ {0x38,0x44,0x44,0x44,0x20},
    /* 100 d    */ {0x38,0x44,0x44,0x48,0x7F},
    /* 101 e    */ {0x38,0x54,0x54,0x54,0x18},
    /* 102 f    */ {0x08,0x7E,0x09,0x01,0x02},
    /* 103 g    */ {0x0C,0x52,0x52,0x52,0x3E},
    /* 104 h    */ {0x7F,0x08,0x04,0x04,0x78},
    /* 105 i    */ {0x00,0x44,0x7D,0x40,0x00},
    /* 106 j    */ {0x20,0x40,0x44,0x3D,0x00},
    /* 107 k    */ {0x7F,0x10,0x28,0x44,0x00},
    /* 108 l    */ {0x00,0x41,0x7F,0x40,0x00},
    /* 109 m    */ {0x7C,0x04,0x18,0x04,0x78},
    /* 110 n    */ {0x7C,0x08,0x04,0x04,0x78},
    /* 111 o    */ {0x38,0x44,0x44,0x44,0x38},
    /* 112 p    */ {0x7C,0x14,0x14,0x14,0x08},
    /* 113 q    */ {0x08,0x14,0x14,0x18,0x7C},
    /* 114 r    */ {0x7C,0x08,0x04,0x04,0x08},
    /* 115 s    */ {0x48,0x54,0x54,0x54,0x24},
    /* 116 t    */ {0x04,0x3F,0x44,0x40,0x20},
    /* 117 u    */ {0x3C,0x40,0x40,0x20,0x7C},
    /* 118 v    */ {0x1C,0x20,0x40,0x20,0x1C},
    /* 119 w    */ {0x3C,0x40,0x30,0x40,0x3C},
    /* 120 x    */ {0x44,0x28,0x10,0x28,0x44},
    /* 121 y    */ {0x0C,0x50,0x50,0x50,0x3C},
    /* 122 z    */ {0x44,0x64,0x54,0x4C,0x44},
    /* 123 {    */ {0x00,0x08,0x36,0x41,0x00},
    /* 124 |    */ {0x00,0x00,0x7F,0x00,0x00},
    /* 125 }    */ {0x00,0x41,0x36,0x08,0x00},
    /* 126 ~    */ {0x08,0x04,0x08,0x10,0x08},
};

static void put_px(unsigned char *rgba, int stride, int x, int y, int w, int h,
                   const unsigned char color[4])
{
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    unsigned char *p = rgba + ((size_t)y * (size_t)stride + (size_t)x) * CAR_RASTER_BPP;
    p[0] = color[0];
    p[1] = color[1];
    p[2] = color[2];
    p[3] = color[3];
}

static void fill_rect(unsigned char *rgba, int stride, int w, int h,
                      int x0, int y0, int rw, int rh, const unsigned char color[4])
{
    for (int y = y0; y < y0 + rh; y++) {
        for (int x = x0; x < x0 + rw; x++) {
            put_px(rgba, stride, x, y, w, h, color);
        }
    }
}

static void blit_rgba(unsigned char *dst, int dstStride, int dstW, int dstH,
                      int dx, int dy,
                      const unsigned char *src, int srcW, int srcH, int srcStride)
{
    for (int y = 0; y < srcH; y++) {
        const int ty = dy + y;
        if (ty < 0 || ty >= dstH) continue;
        for (int x = 0; x < srcW; x++) {
            const int tx = dx + x;
            if (tx < 0 || tx >= dstW) continue;
            const unsigned char *s = src + ((size_t)y * (size_t)srcStride + (size_t)x) * CAR_RASTER_BPP;
            if (s[3] == 0) continue;
            unsigned char *d = dst + ((size_t)ty * (size_t)dstStride + (size_t)tx) * CAR_RASTER_BPP;
            /* Source cars are drawn on a cleared buffer; overwrite rather than blend. */
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
            d[3] = s[3];
        }
    }
}

static void draw_char(unsigned char *rgba, int stride, int w, int h,
                      int x, int y, char c, const unsigned char color[4])
{
    const unsigned char *glyph;
    if (c < 32 || c > 126) {
        static const unsigned char box[5] = {0x7F, 0x41, 0x41, 0x41, 0x7F};
        glyph = box;
    } else {
        glyph = kFont5x7[c - 32];
    }
    for (int col = 0; col < SHEET_FONT_W; col++) {
        const unsigned char bits = glyph[col];
        for (int row = 0; row < SHEET_FONT_H; row++) {
            if (bits & (1u << row)) {
                put_px(rgba, stride, x + col, y + row, w, h, color);
            }
        }
    }
}

static void draw_text(unsigned char *rgba, int stride, int w, int h,
                      int x, int y, const char *text, int maxChars,
                      const unsigned char color[4])
{
    int drawn = 0;
    for (const char *p = text; p != NULL && *p != '\0' && drawn < maxChars; p++, drawn++) {
        draw_char(rgba, stride, w, h, x + drawn * (SHEET_FONT_W + 1), y, *p, color);
    }
}

static void write_index(FILE *out, int pageCount, int zoom, float pxPerM,
                        int pageW, int pageH, int count)
{
    fprintf(out,
        "<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
        "<title>Drifty — vehicle corpus</title>\n<style>\n"
        "  body { background:#15181d; color:#e8eaee; font:14px/1.5 system-ui,sans-serif;"
        " margin:24px; }\n"
        "  h1 { font-size:20px; font-weight:600; margin:0 0 4px; }\n"
        "  p.sub { color:#98a0ae; margin:0 0 20px; }\n"
        "  h2 { font-size:14px; color:#6ecdeb; margin:28px 0 10px; font-weight:600; }\n"
        "  img.page { image-rendering:pixelated; background:#15181d;"
        " border:1px solid #2a303a; display:block; max-width:100%%; height:auto; }\n"
        "</style>\n</head>\n<body>\n"
        "<h1>Drifty — vehicle corpus</h1>\n"
        "<p class=\"sub\">%d vehicles as paginated contact sheets."
        " Rasterized at %.2f px/m — the scale the game actually draws at — with one shared"
        " cell size (no per-car auto-fit). Pages shown at %d&times; nearest-neighbour.</p>\n",
        count, (double)pxPerM, zoom);

    for (int p = 1; p <= pageCount; p++) {
        fprintf(out, "<h2>Page %d / %d</h2>\n"
                     "<img class=\"page\" src=\"page_%d.png\" width=\"%d\" height=\"%d\""
                     " alt=\"corpus contact sheet page %d\">\n",
                p, pageCount, p, pageW * zoom, pageH * zoom, p);
    }
    fputs("</body>\n</html>\n", out);
}

bool car_sheet_write(const char *outDir, float pxPerM, int zoom)
{
    if (outDir == NULL || outDir[0] == '\0') return false;
    if (!(pxPerM > 0.0f)) pxPerM = car_sheet_default_px_per_m();
    if (zoom <= 0) zoom = 5;
    if (!telemetry_ensure_dir(outDir)) return false;

    const int count = car_corpus_count();
    if (count <= 0) return false;

    /* First pass: the common cell art size. Every car is drawn at one scale, so the cell must
     * fit the largest of them; sizing per car would hide the size axis entirely. */
    int artW = 1, artH = 1;
    for (int i = 0; i < count; i++) {
        VehicleSpec spec;
        CarVisual visual;
        if (!car_corpus_spec(i, &spec)) return false;
        car_visual_derive(&spec, &visual);
        const CarRasterInfo info = car_raster_info(&visual, pxPerM, SHEET_PAD_PX);
        /* Rotated nose-up, so width and height swap. */
        if (info.height > artW) artW = info.height;
        if (info.width  > artH) artH = info.width;
    }

    const int minCellW = 4 + SHEET_MIN_CHARS * (SHEET_FONT_W + 1);
    const int cellW = (artW > minCellW) ? artW : minCellW;
    const int cellH = artH + SHEET_LABEL_H;
    const int pageW = SHEET_MARGIN_PX * 2 + SHEET_COLS * cellW + (SHEET_COLS - 1) * SHEET_GAP_PX;
    const int pageH = SHEET_MARGIN_PX * 2 + SHEET_ROWS * cellH + (SHEET_ROWS - 1) * SHEET_GAP_PX;
    const int perPage = SHEET_COLS * SHEET_ROWS;
    const int pageCount = (count + perPage - 1) / perPage;

    const size_t pageBytes = (size_t)pageW * (size_t)pageH * CAR_RASTER_BPP;
    unsigned char *page = (unsigned char *)malloc(pageBytes);
    if (page == NULL) {
        fprintf(stderr, "CAR-SHEET: out of memory for page buffer (%dx%d)\n", pageW, pageH);
        return false;
    }

    char indexPath[512];
    snprintf(indexPath, sizeof(indexPath), "%s/index.html", outDir);
    FILE *index = fopen(indexPath, "wb");
    if (index == NULL) {
        fprintf(stderr, "CAR-SHEET: could not open '%s'\n", indexPath);
        free(page);
        return false;
    }

    bool ok = true;
    int pageIndex = 0;
    int slot = 0;

    for (int i = 0; i < count && ok; i++) {
        if (slot == 0) {
            /* Opaque fill — viewers disagree about checkerboarding under translucent pages. */
            for (size_t b = 0; b < pageBytes; b += CAR_RASTER_BPP) {
                page[b + 0] = kPageBg[0];
                page[b + 1] = kPageBg[1];
                page[b + 2] = kPageBg[2];
                page[b + 3] = kPageBg[3];
            }
            pageIndex++;
        }

        VehicleSpec spec;
        CarVisual visual;
        char id[128], note[192];
        if (!car_corpus_spec(i, &spec)) { ok = false; break; }
        car_visual_derive(&spec, &visual);
        car_corpus_id(i, id, sizeof(id));
        car_corpus_describe(i, note, sizeof(note));
        const char *group = car_corpus_group_name(car_corpus_group(i));

        const CarRasterInfo info = car_raster_info(&visual, pxPerM, SHEET_PAD_PX);
        const size_t bytes = car_raster_bytes(info);
        unsigned char *body = (unsigned char *)malloc(bytes);
        unsigned char *upright = (unsigned char *)malloc(bytes);
        if (body == NULL || upright == NULL) {
            fprintf(stderr, "CAR-SHEET: out of memory for '%s'\n", id);
            free(body);
            free(upright);
            ok = false;
            break;
        }

        if (!car_raster_draw(&visual, info, body, bytes) ||
            !car_raster_rotate_nose_up(body, info.width, info.height, upright, bytes)) {
            fprintf(stderr, "CAR-SHEET: could not rasterize '%s'\n", id);
            free(body);
            free(upright);
            ok = false;
            break;
        }

        const int col = slot % SHEET_COLS;
        const int row = slot / SHEET_COLS;
        const int cellX = SHEET_MARGIN_PX + col * (cellW + SHEET_GAP_PX);
        const int cellY = SHEET_MARGIN_PX + row * (cellH + SHEET_GAP_PX);

        fill_rect(page, pageW, pageW, pageH, cellX, cellY, cellW, cellH, kCellBg);

        /* Nose-up: source height becomes displayed width. Centre inside the art area. */
        const int carW = info.height, carH = info.width;
        const int ox = cellX + (cellW - carW) / 2;
        const int oy = cellY + (artH - carH) / 2;
        blit_rgba(page, pageW, pageW, pageH, ox, oy, upright, carW, carH, carW);

        /* The id already begins with its group, so printing the group again only ate
         * characters the caption did not have to spare. */
        (void)group;
        const int labelY0 = cellY + artH + 2;
        const int maxChars = (cellW - 4) / (SHEET_FONT_W + 1);
        draw_text(page, pageW, pageW, pageH, cellX + 2, labelY0, id, maxChars, kGroupFg);
        draw_text(page, pageW, pageW, pageH, cellX + 2, labelY0 + SHEET_FONT_H + 2,
                  note, maxChars, kNoteFg);

        free(body);
        free(upright);

        slot++;
        const bool pageFull = (slot >= perPage);
        const bool lastCar = (i + 1 >= count);
        if (pageFull || lastCar) {
            char pngPath[512];
            snprintf(pngPath, sizeof(pngPath), "%s/page_%d.png", outDir, pageIndex);
            if (!stbi_write_png(pngPath, pageW, pageH, CAR_RASTER_BPP, page,
                                pageW * CAR_RASTER_BPP)) {
                fprintf(stderr, "CAR-SHEET: could not write '%s'\n", pngPath);
                ok = false;
                break;
            }
            slot = 0;
        }
    }

    if (ok) {
        write_index(index, pageCount, zoom, pxPerM, pageW, pageH, count);
    }
    if (fclose(index) != 0) ok = false;
    free(page);

    if (ok) {
        printf("CAR-SHEET: wrote %d vehicles across %d page(s) to %s (page_N.png + index.html)\n",
               count, pageCount, outDir);
    }
    return ok;
}

/* ---------------------------------------------------------- distinctness failure bundle -- */

/* The canvas every corpus comparison happens on: one buffer big enough for the largest car,
 * with body-space (0,0) at a fixed point, so a difference in position cannot masquerade as a
 * difference in shape. Built the same way as the `corpus` scenario builds it, so the ratio in
 * the report is the same number the assertion failed on. */
static CarRasterInfo pair_shared_canvas(float pxPerM)
{
    float left = 1.0f, right = 1.0f, up = 1.0f, down = 1.0f;

    for (int i = 0; i < car_corpus_count(); i++) {
        VehicleSpec spec;
        CarVisual visual;
        if (!car_corpus_spec(i, &spec)) continue;
        car_visual_derive(&spec, &visual);

        const CarRasterInfo info = car_raster_info(&visual, pxPerM, 2);
        const float l = info.originXPx;
        const float r = (float)info.width - info.originXPx;
        const float u = info.originYPx;
        const float d = (float)info.height - info.originYPx;
        if (l > left)  left  = l;
        if (r > right) right = r;
        if (u > up)    up    = u;
        if (d > down)  down  = d;
    }

    CarRasterInfo shared;
    memset(&shared, 0, sizeof(shared));
    shared.pxPerM    = pxPerM;
    shared.width     = (int)ceilf(left + right);
    shared.height    = (int)ceilf(up + down);
    shared.originXPx = left;
    shared.originYPx = up;
    return shared;
}

/* Rotate nose-up and write. Keeps every artifact in the same orientation as the contact
 * sheet, so a failure bundle can be compared against it directly. */
static bool write_upright_png(const char *path, const unsigned char *rgba, int w, int h)
{
    const size_t bytes = (size_t)w * (size_t)h * CAR_RASTER_BPP;
    unsigned char *upright = (unsigned char *)malloc(bytes);
    if (upright == NULL) return false;
    bool ok = car_raster_rotate_nose_up(rgba, w, h, upright, bytes);
    if (ok) {
        ok = stbi_write_png(path, h, w, CAR_RASTER_BPP, upright, h * CAR_RASTER_BPP) != 0;
    }
    free(upright);
    return ok;
}

static bool write_profile(const char *path, const VehicleSpec *spec)
{
    return dev_params_save(spec, path);
}

bool car_sheet_write_pair_failure(const char *outDir, int indexA, int indexB, float pxPerM)
{
    if (outDir == NULL || outDir[0] == '\0') return false;
    if (indexA < 0 || indexB < 0) return false;
    if (indexA >= car_corpus_count() || indexB >= car_corpus_count()) return false;
    if (!(pxPerM > 0.0f)) pxPerM = car_sheet_default_px_per_m();

    char idA[128], idB[128];
    car_corpus_id(indexA, idA, sizeof(idA));
    car_corpus_id(indexB, idB, sizeof(idB));

    VehicleSpec specA, specB;
    if (!car_corpus_spec(indexA, &specA) || !car_corpus_spec(indexB, &specB)) return false;

    CarVisual visA, visB;
    car_visual_derive(&specA, &visA);
    car_visual_derive(&specB, &visB);

    if (!telemetry_ensure_dir(outDir)) return false;
    char dir[512];
    snprintf(dir, sizeof(dir), "%s/%s__%s", outDir, idA, idB);
    if (!telemetry_ensure_dir(dir)) return false;

    const CarRasterInfo canvas = pair_shared_canvas(pxPerM);
    const size_t pixels = (size_t)canvas.width * (size_t)canvas.height;
    const size_t rgbaBytes = car_raster_bytes(canvas);
    if (pixels == 0 || rgbaBytes == 0) return false;

    unsigned char *rgbaA = (unsigned char *)malloc(rgbaBytes);
    unsigned char *rgbaB = (unsigned char *)malloc(rgbaBytes);
    unsigned char *rgbaD = (unsigned char *)malloc(rgbaBytes);
    unsigned char *labA  = (unsigned char *)malloc(pixels);
    unsigned char *labB  = (unsigned char *)malloc(pixels);
    bool ok = (rgbaA != NULL && rgbaB != NULL && rgbaD != NULL && labA != NULL && labB != NULL);

    if (ok) {
        ok = car_raster_draw(&visA, canvas, rgbaA, rgbaBytes) &&
             car_raster_draw(&visB, canvas, rgbaB, rgbaBytes) &&
             car_raster_draw_labels(&visA, canvas, labA, pixels) &&
             car_raster_draw_labels(&visB, canvas, labB, pixels);
    }

    size_t unionPx = 0, differing = 0;
    if (ok) {
        /* diff.png: shared silhouette dimmed, disagreeing pixels in warning red. Same rule as
         * car_raster_difference so the picture and the number cannot disagree. */
        for (size_t i = 0; i < pixels; i++) {
            const unsigned char a = labA[i], b = labB[i];
            unsigned char *p = rgbaD + i * CAR_RASTER_BPP;
            if (a == CAR_LABEL_EMPTY && b == CAR_LABEL_EMPTY) {
                p[0] = 0x15; p[1] = 0x18; p[2] = 0x1d; p[3] = 0xff;
                continue;
            }
            unionPx++;
            if (a != b) {
                differing++;
                p[0] = 0xe0; p[1] = 0x48; p[2] = 0x3c; p[3] = 0xff;
            } else {
                p[0] = 0x38; p[1] = 0x3e; p[2] = 0x48; p[3] = 0xff;
            }
        }
    }

    char path[640];
    if (ok) {
        snprintf(path, sizeof(path), "%s/car_a.png", dir);
        ok = write_upright_png(path, rgbaA, canvas.width, canvas.height);
    }
    if (ok) {
        snprintf(path, sizeof(path), "%s/car_b.png", dir);
        ok = write_upright_png(path, rgbaB, canvas.width, canvas.height);
    }
    if (ok) {
        snprintf(path, sizeof(path), "%s/diff.png", dir);
        ok = write_upright_png(path, rgbaD, canvas.width, canvas.height);
    }
    if (ok) {
        snprintf(path, sizeof(path), "%s/car_a.txt", dir);
        ok = write_profile(path, &specA);
    }
    if (ok) {
        snprintf(path, sizeof(path), "%s/car_b.txt", dir);
        ok = write_profile(path, &specB);
    }

    if (ok) {
        /* Signature metrics: L2, Linf, and the three components that differ most, by name. */
        float sigA[64], sigB[64];
        const int n = car_visual_signature_count();
        const int cap = (int)(sizeof(sigA) / sizeof(sigA[0]));
        float l2 = 0.0f, linf = 0.0f;
        int top[3] = { -1, -1, -1 };

        if (n > 0 && n <= cap &&
            car_visual_signature(&visA, sigA, n) == n &&
            car_visual_signature(&visB, sigB, n) == n) {
            for (int i = 0; i < n; i++) {
                const float d = (sigA[i] > sigB[i]) ? (sigA[i] - sigB[i]) : (sigB[i] - sigA[i]);
                l2 += d * d;
                if (d > linf) linf = d;
                for (int k = 0; k < 3; k++) {
                    const float best = (top[k] >= 0)
                        ? ((sigA[top[k]] > sigB[top[k]]) ? (sigA[top[k]] - sigB[top[k]])
                                                         : (sigB[top[k]] - sigA[top[k]]))
                        : -1.0f;
                    if (d > best) {
                        for (int m = 2; m > k; m--) top[m] = top[m - 1];
                        top[k] = i;
                        break;
                    }
                }
            }
            l2 = (float)sqrt((double)l2);
        }

        snprintf(path, sizeof(path), "%s/report.txt", dir);
        FILE *out = fopen(path, "wb");
        if (out == NULL) {
            ok = false;
        } else {
            char noteA[192], noteB[192];
            car_corpus_describe(indexA, noteA, sizeof(noteA));
            car_corpus_describe(indexB, noteB, sizeof(noteB));

            fprintf(out, "corpus distinctness failure\n");
            fprintf(out, "===========================\n\n");
            fprintf(out, "car a : [%d] %s\n        %s\n", indexA, idA, noteA);
            fprintf(out, "car b : [%d] %s\n        %s\n\n", indexB, idB, noteB);
            fprintf(out, "scale            : %.3f px/m (%d x %d canvas)\n",
                    (double)pxPerM, canvas.width, canvas.height);
            fprintf(out, "union silhouette : %llu px\n", (unsigned long long)unionPx);
            fprintf(out, "differing        : %llu px\n", (unsigned long long)differing);
            fprintf(out, "pixel ratio      : %.4f\n",
                    (unionPx > 0) ? (double)differing / (double)unionPx : 0.0);
            fprintf(out, "signature L2     : %.4f\n", (double)l2);
            fprintf(out, "signature Linf   : %.4f m\n\n", (double)linf);

            fprintf(out, "largest signature gaps\n----------------------\n");
            for (int k = 0; k < 3; k++) {
                if (top[k] < 0) continue;
                const float d = (sigA[top[k]] > sigB[top[k]]) ? (sigA[top[k]] - sigB[top[k]])
                                                             : (sigB[top[k]] - sigA[top[k]]);
                fprintf(out, "  %-24s %10.4f vs %10.4f   (gap %.4f)\n",
                        car_visual_signature_component_name(top[k]),
                        (double)sigA[top[k]], (double)sigB[top[k]], (double)d);
            }

            fprintf(out, "\nnon-default overrides — car a\n-----------------------------\n");
            (void)dev_params_write_overrides(out, &specA);
            fprintf(out, "\nnon-default overrides — car b\n-----------------------------\n");
            (void)dev_params_write_overrides(out, &specB);

            if (fclose(out) != 0) ok = false;
        }
    }

    free(rgbaA);
    free(rgbaB);
    free(rgbaD);
    free(labA);
    free(labB);

    if (ok) {
        printf("CAR-SHEET: wrote distinctness failure bundle to %s\n", dir);
    } else {
        fprintf(stderr, "CAR-SHEET: could not write failure bundle to %s\n", dir);
    }
    return ok;
}
