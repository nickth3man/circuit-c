/*
 * car_sheet.c — headless vehicle contact sheet. See the header for the contract.
 *
 * This is the only translation unit that defines STB_IMAGE_WRITE_IMPLEMENTATION, and it is
 * compiled into drifty_tests.exe alone: neither the game module, drifty.exe, nor the release
 * build links stb_image_write (see third_party/README.md).
 */
#include "car_sheet.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_corpus.h"
#include "car_visual.h"
#include "car_visual_raster.h"
#include "config.h"
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

#define SHEET_PAD_PX 2

float car_sheet_default_px_per_m(void)
{
    /* What a player actually sees: the world scale times the camera's resting zoom. */
    return PIXELS_PER_METER * CAMERA_BASE_ZOOM;
}

/* Minimal HTML escaping for captions, which carry em dashes and parameter names. */
static void write_escaped(FILE *out, const char *text)
{
    for (const char *p = text; p != NULL && *p != '\0'; p++) {
        switch (*p) {
            case '&':  fputs("&amp;",  out); break;
            case '<':  fputs("&lt;",   out); break;
            case '>':  fputs("&gt;",   out); break;
            case '"':  fputs("&quot;", out); break;
            default:   fputc(*p, out);       break;
        }
    }
}

static void write_index_head(FILE *out, int zoom, float pxPerM, int cellW, int cellH)
{
    fprintf(out,
        "<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
        "<title>Drifty — vehicle corpus</title>\n<style>\n"
        "  body { background:#15181d; color:#e8eaee; font:14px/1.5 system-ui,sans-serif;"
        " margin:24px; }\n"
        "  h1 { font-size:20px; font-weight:600; margin:0 0 4px; }\n"
        "  p.sub { color:#98a0ae; margin:0 0 20px; }\n"
        "  .grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(%dpx,1fr));"
        " gap:18px; }\n"
        "  .cell { background:#1d2128; border:1px solid #2a303a; border-radius:8px;"
        " padding:10px; }\n"
        "  .art { height:%dpx; display:flex; align-items:center; justify-content:center; }\n"
        "  img { image-rendering:pixelated; }\n"
        "  .id { font:12px/1.4 ui-monospace,monospace; color:#ffc640; margin-top:8px;"
        " word-break:break-all; }\n"
        "  .note { font-size:12px; color:#98a0ae; }\n"
        "  .grp { font-size:11px; text-transform:uppercase; letter-spacing:.06em;"
        " color:#6ecdeb; }\n"
        "</style>\n</head>\n<body>\n"
        "<h1>Drifty — vehicle corpus</h1>\n"
        "<p class=\"sub\">Every car below is a pure function of its physics parameters."
        " Rasterized at %.2f px/m — the scale the game actually draws at — then shown at"
        " %d&times; with nearest-neighbour, so nothing here is more legible than it is"
        " in play.</p>\n<div class=\"grid\">\n",
        cellW + 24, cellH, (double)pxPerM, zoom);
}

bool car_sheet_write(const char *outDir, float pxPerM, int zoom)
{
    if (outDir == NULL || outDir[0] == '\0') return false;
    if (!(pxPerM > 0.0f)) pxPerM = car_sheet_default_px_per_m();
    if (zoom <= 0) zoom = 5;
    if (!telemetry_ensure_dir(outDir)) return false;

    const int count = car_corpus_count();
    if (count <= 0) return false;

    /* First pass: the common cell size. Every car is drawn at one scale, so the cell must fit
     * the largest of them; sizing per car would hide the size axis entirely. */
    int cellW = 1, cellH = 1;
    for (int i = 0; i < count; i++) {
        VehicleSpec spec;
        CarVisual visual;
        if (!car_corpus_spec(i, &spec)) return false;
        car_visual_derive(&spec, &visual);
        const CarRasterInfo info = car_raster_info(&visual, pxPerM, SHEET_PAD_PX);
        /* Rotated nose-up, so width and height swap. */
        if (info.height > cellW) cellW = info.height;
        if (info.width  > cellH) cellH = info.width;
    }

    char indexPath[512];
    snprintf(indexPath, sizeof(indexPath), "%s/index.html", outDir);
    FILE *index = fopen(indexPath, "wb");
    if (index == NULL) {
        fprintf(stderr, "CAR-SHEET: could not open '%s'\n", indexPath);
        return false;
    }
    write_index_head(index, zoom, pxPerM, cellW * zoom, cellH * zoom);

    bool ok = true;
    for (int i = 0; i < count && ok; i++) {
        VehicleSpec spec;
        CarVisual visual;
        char id[128], note[192];

        if (!car_corpus_spec(i, &spec)) { ok = false; break; }
        car_visual_derive(&spec, &visual);
        car_corpus_id(i, id, sizeof(id));
        car_corpus_describe(i, note, sizeof(note));

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
            ok = false;
        } else {
            char pngPath[512];
            snprintf(pngPath, sizeof(pngPath), "%s/%s.png", outDir, id);
            /* Nose-up: the source's height becomes the width and vice versa. */
            const int w = info.height, h = info.width;
            if (!stbi_write_png(pngPath, w, h, CAR_RASTER_BPP, upright, w * CAR_RASTER_BPP)) {
                fprintf(stderr, "CAR-SHEET: could not write '%s'\n", pngPath);
                ok = false;
            } else {
                fprintf(index, "<div class=\"cell\"><div class=\"art\">"
                               "<img src=\"%s.png\" width=\"%d\" height=\"%d\" alt=\"",
                        id, w * zoom, h * zoom);
                write_escaped(index, id);
                fputs("\"></div>\n<div class=\"grp\">", index);
                write_escaped(index, car_corpus_group_name(car_corpus_group(i)));
                fputs("</div>\n<div class=\"id\">", index);
                write_escaped(index, id);
                fputs("</div>\n<div class=\"note\">", index);
                write_escaped(index, note);
                fputs("</div></div>\n", index);
            }
        }

        free(body);
        free(upright);
    }

    fputs("</div>\n</body>\n</html>\n", index);
    if (fclose(index) != 0) ok = false;

    if (ok) printf("CAR-SHEET: wrote %d vehicles to %s (index.html)\n", count, outDir);
    return ok;
}
