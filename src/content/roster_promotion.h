/*
 * roster_promotion.h — issue #31 promotion checklist gate for vehicle content.
 *
 * WHY THIS EXISTS. A manifest that parses is not yet race content. The demonstration fleet
 * in src/dev/car_corpus.c (100 vehicles: 17 archetypes + 40 sweeps + 43 sampled) exists to
 * exercise parametric rendering, not to populate the roster: those shapes were never given a
 * reviewed setup, a confirmed collision envelope, or a human sign-off, and the corpus is not
 * part of the gameplay roster for exactly that reason. Promotion must therefore be an
 * explicit, reviewed data change — a manifest only becomes race content when an author runs
 * the checklist below and every item passes. The engine never promotes implicitly: until
 * then the manifest stays visible to review tools but invisible to car_roster.
 *
 * THE CHECKLIST (fixed order; each row carries evidence a reviewer can verify, not just a
 * boolean):
 *   1. identity-provenance    stable content id, non-empty display name, recorded
 *                             provenance source and author.
 *   2. physical-data-complete spec validates and a non-zero content hash proves the
 *                             physics were actually authored.
 *   3. class-assignment       at least one class tag, so the roster UI can group cars.
 *   4. default-setup-valid    the authored baseline setup validates against the definition.
 *   5. review-status          contentKind is validated or player-selectable — an explicit
 *                             human review recorded the promotion.
 *   6. collision-dimensions   derived collision half-width and overall length sit inside
 *                             the envelopes the collision system was tuned for.
 *   7. ai-compatibility       AI eligibility is on; a promoted car must be drivable by
 *                             every entrant kind the game ships.
 *
 * DETERMINISM. vehicle_promotion_evaluate is a pure function of one manifest: identical
 * input, identical report, on every platform, on every call. All rows are always filled in
 * the fixed order above (count == PROMOTION_CHECK_COUNT), never pruned, so a reviewer sees
 * every outstanding item at once and a test can assert each check's independence. Every
 * pass/fail decision is an integer, boolean, or string comparison — never a floating-point
 * equality — so no epsilon policy is needed; floats appear only in the human-readable
 * evidence.
 *
 * This translation unit calls no raylib function and links into the headless test executable.
 */
#ifndef CIRCUIT_ROSTER_PROMOTION_H
#define CIRCUIT_ROSTER_PROMOTION_H

#include <stdbool.h>

#include "content/vehicle_manifest.h"

#define PROMOTION_CHECK_NAME_CHARS 48
#define PROMOTION_CHECK_DETAIL_CHARS 128
#define PROMOTION_MAX_CHECKS 8
#define PROMOTION_CHECK_COUNT \
    7 /* the fixed checklist; the report always fills this many rows */

/* One row of the promotion report: the check name, whether it passed, and the evidence a
 * reviewer (or a failing test) reads to see why. Fixed buffers, so a report is plain data
 * that survives a hot reload. */
typedef struct {
    char name[PROMOTION_CHECK_NAME_CHARS];
    bool pass;
    char detail[PROMOTION_CHECK_DETAIL_CHARS];
} PromotionCheck;

typedef struct {
    PromotionCheck checks[PROMOTION_MAX_CHECKS];
    int count;
} VehiclePromotionReport;

/* Evaluates the promotion checklist; returns true when every check passes. Always fills `out`
 * with every row in the fixed order above (count == PROMOTION_CHECK_COUNT), regardless of
 * pass/fail, so callers can show all outstanding work instead of just the first failure.
 * Returns false with an empty report when `out` or `manifest` is NULL. */
bool vehicle_promotion_evaluate(const VehicleManifest *manifest, VehiclePromotionReport *out);

#endif /* CIRCUIT_ROSTER_PROMOTION_H */
