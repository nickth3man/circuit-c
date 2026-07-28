/*
 * car_sheet.h — the vehicle contact sheet, written headlessly from drifty_tests.
 *
 * No GPU, no window, no raylib call: the cars are rasterized on the CPU by
 * src/car_visual_raster.c and written as PNGs by the vendored stb_image_write. That is what
 * lets the whole fleet be reviewed on any machine, including CI, and — crucially — before
 * src/render.c has been touched at all.
 *
 * OUTPUT. Paginated composite PNGs (page_1.png, page_2.png, …) plus index.html. Every cell
 * shares one metres-to-pixels scale; cells are not individually auto-fitted. Each cell is
 * labelled with corpus group, id, and description (sweep steps name the varied parameter).
 *
 * SCALE HONESTY. Two rules, both load-bearing:
 *   - Every car is rasterized at the SAME metres-to-pixels scale. Auto-fitting each cell to
 *     its car would erase the size axis and make the sheet lie about the biggest visual
 *     difference there is.
 *   - That scale defaults to what the game actually draws at (PIXELS_PER_METER *
 *     CAMERA_BASE_ZOOM), so the sheet cannot flatter the grammar with detail the player will
 *     never see. The HTML then upscales page images by an integer factor with
 *     nearest-neighbour so a human can inspect them without the pixels being a lie.
 */
#ifndef DRIFTY_CAR_SHEET_H
#define DRIFTY_CAR_SHEET_H

#include <stdbool.h>

/* The scale the game genuinely renders at. */
float car_sheet_default_px_per_m(void);

/* Write paginated contact sheets into outDir as page_N.png plus index.html.
 * pxPerM <= 0 selects car_sheet_default_px_per_m(); zoom <= 0 selects 5 (HTML display only).
 * Returns false if the directory could not be created or any file could not be written. */
bool car_sheet_write(const char *outDir, float pxPerM, int zoom);

#endif /* DRIFTY_CAR_SHEET_H */
