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

/* ------------------------------------------------------------------ inspector cards ----
 *
 * One directory of per-car evidence for the browser inspector in tools/visual/. The contact
 * sheet answers "does the fleet look varied"; this answers "what is this ONE car made of, and
 * which of its features survive the trip to the screen".
 *
 * Written into outDir:
 *   <id>.png          the car, nose-up, at pxPerM — the same pixels the sheet shows
 *   <id>_labels.png   the feature-label map, one flat colour per CarRasterLabel
 *   cards.json        every derived quantity per car: latents, named signature components,
 *                     hull stations, per-wheel visuals, appendages, AND the label histogram
 *
 * THE HISTOGRAM IS THE POINT. A rule can be perfectly implemented and still cover zero pixels
 * once it is quantised at 13.2 px/m. Counting the pixels each feature actually occupies is the
 * only way to tell a grammar bug from a fidelity-budget loss, and no pass/fail check reports
 * it. pxPerM <= 0 selects car_sheet_default_px_per_m(). */
bool car_sheet_write_cards(const char *outDir, float pxPerM);

/* ------------------------------------------------------------- distinctness failures ----
 *
 * When two corpus vehicles are too similar, a pass/fail line is useless on its own: the whole
 * point of the corpus is that a human can look at the pair and decide whether the grammar or
 * the corpus is at fault. This writes everything needed to do that into
 *
 *     <outDir>/<idA>__<idB>/
 *         car_a.png  car_b.png   the two cars, nose-up, at the comparison scale
 *         diff.png                per-pixel label agreement: differing pixels highlighted
 *         car_a.txt  car_b.txt    each spec as a tuning profile, loadable in the Physics Lab
 *         report.txt              ids, pixel ratio, union/differing counts, L2, Linf, and the
 *                                 three largest signature-component gaps by name
 *
 * pxPerM <= 0 selects car_sheet_default_px_per_m(). Returns false if anything could not be
 * written; the caller reports that rather than losing the failure silently. */
bool car_sheet_write_pair_failure(const char *outDir, int indexA, int indexB, float pxPerM);

#endif /* DRIFTY_CAR_SHEET_H */
