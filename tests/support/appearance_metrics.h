/*
 * appearance_metrics.h — the one contract the appearance gates and --measure-sweep share.
 *
 * Both the `car-visual`/`corpus` scenarios and the sweep-measurement command have to agree on
 * the canvas, the metrics and the floors, or a key could pass one and fail the other for
 * reasons that have nothing to do with the vehicle.
 */
#ifndef DRIFTY_APPEARANCE_METRICS_H
#define DRIFTY_APPEARANCE_METRICS_H

#include <stdbool.h>
#include <stddef.h>

#include "core/config.h"
#include "physics/vehicle.h"
#include "render/car_visual.h"
#include "render/car_visual_raster.h"

/* The scale everything visual is asserted at: what the game actually draws. Asserting at a
 * higher scale would let differences pass that no player could ever see. */
#define CV_TEST_PX_PER_M   (PIXELS_PER_METER * CAMERA_BASE_ZOOM)

/* Fraction of the union silhouette that must differ for two cars to count as distinguishable.
 * The authoritative metric — a colour-blind comparison of feature-label maps. */
#define CV_MIN_PIXEL_DIFF  0.030f

/* Per-component floor on the diagnostic feature vector: 0.08 m is almost exactly one screen
 * pixel at CV_TEST_PX_PER_M, so "one visible pixel somewhere" is a literal reading of it. */
#define CV_MIN_LINF        0.080f

/* Companion floor on the whole diagnostic vector. Two cars that differ a little in many
 * features are as distinguishable as two that differ a lot in one, and L2 is what says so. */
#define CV_MIN_L2          0.250f

/* The sensitivity floor is deliberately lower than the pairwise one, and the two ask
 * different questions.
 *
 * CV_MIN_PIXEL_DIFF separates two DIFFERENT VEHICLES inside a hundred-car fleet: it has to be
 * high enough that "these are the same car" is never the honest reading.
 *
 * The sensitivity test asks something weaker and more specific — is this key wired to the
 * picture at all? A mis-wired or deleted rule scores 0.0000, not 0.0200, so the floor only
 * has to be clear of quantisation noise. The real anti-cheat here is the companion
 * CV_MIN_LINF assertion, which additionally demands the change reach a NAMED feature by at
 * least one screen pixel — a pixel count alone cannot say that.
 *
 * The number matters because the plan's own fidelity budget is binding: at ~13.2 px/m a stock
 * car is ~1100 silhouette pixels, so 1.5% is ~17 px. Tire aspect ratio swung across its whole
 * 25%..80% registry range repaints ~2.5% of the car — unmistakable on the contact sheet, a
 * 45% change in tire diameter — yet would fail a 3% bar simply because two of four wheels are
 * a small share of a car seen from above. Failing that would be the test lying, not the
 * grammar. */
#define CV_MIN_SENSITIVITY_DIFF  0.015f

/* Where a distinctness failure writes its inspectable bundle. Suppressed by --no-bundle,
 * like every other failure bundle in this suite. */
#define CV_FAILURE_DIR     "artifacts/car_visual_failures"

CarRasterInfo test_car_shared_canvas(float pxPerM);
bool test_car_labels_for_spec(const VehicleSpec *spec, CarRasterInfo canvas,
                             unsigned char *labels, size_t bytes);
float test_car_signature_linf(const CarVisual *a, const CarVisual *b, int *worstOut);
float test_car_signature_l2(const CarVisual *a, const CarVisual *b);
int test_car_primary_diff_count(const VehicleSpec *a, const VehicleSpec *b,
                                const char **firstName);

#endif /* DRIFTY_APPEARANCE_METRICS_H */
