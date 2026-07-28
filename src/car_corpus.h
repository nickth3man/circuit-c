/*
 * car_corpus.h — the demonstration fleet: N vehicles as pure functions of an index.
 *
 * WHY C AND NOT FILES. The corpus is the source of truth for the distinctness test and the
 * contact sheet, so it must be reachable without file I/O, without a working directory
 * assumption, and without enumerating a directory from inside the hot-reloadable game module.
 * `drifty_tests --generate-corpus` exports these specs into tuning/corpus as .txt files in the
 * existing profile format for humans to read and diff; a test asserts the export round-trips,
 * so the files cannot silently rot away from the code.
 *
 * COMPOSITION. Two groups in Phase 1:
 *
 *   archetype  the hand-designed presets from dev_presets.c — recognisable anchors that prove
 *              deliberately authored specs render sensibly
 *   sweep      one registry key varied across its declared [minimum, maximum] with everything
 *              else at stock. A row of cars where ONLY the wheelbase changes is the most
 *              direct evidence that appearance is a function of the parameters, and it is
 *              self-documenting in a way a random sample never is.
 *
 * A third `sampled` group joins them in Phase 3, once the parameter expansion has widened the
 * space enough for quasi-random points to be interesting.
 *
 * Raylib-free and I/O-free.
 */
#ifndef DRIFTY_CAR_CORPUS_H
#define DRIFTY_CAR_CORPUS_H

#include <stdbool.h>
#include <stddef.h>

#include "vehicle.h"

typedef enum {
    CAR_CORPUS_ARCHETYPE = 0,
    CAR_CORPUS_SWEEP,
    CAR_CORPUS_SAMPLED,
    CAR_CORPUS_GROUP_COUNT
} CarCorpusGroup;

/* Total vehicles. Stable for a given build. */
int car_corpus_count(void);

/* Write vehicle `index` into `out`. Pure: the same index always yields the same spec, and no
 * ordering of calls changes the result. Returns false for an out-of-range index or NULL out.
 * The result is not guaranteed valid by construction — the `corpus` scenario asserts
 * vehicle_spec_is_valid() over every entry, which is how a bad registry range gets caught. */
bool car_corpus_spec(int index, VehicleSpec *out);

CarCorpusGroup car_corpus_group(int index);
const char    *car_corpus_group_name(CarCorpusGroup group);

/* A stable, filesystem-safe identifier, e.g. "sweep_wheel_radius_2". Formatted into the
 * caller's buffer rather than returned as a pointer so there is no shared static state. */
void car_corpus_id(int index, char *buf, size_t cap);

/* One human-readable line naming what makes this entry what it is, e.g.
 * "body.cg_to_rear = 1.200 (step 2/5)". Used in the contact sheet and in test failures. */
void car_corpus_describe(int index, char *buf, size_t cap);

/* The registry key a sweep entry varies, or NULL for non-sweep entries. Lets a test assert
 * that each sweep axis actually moves the appearance. */
const char *car_corpus_sweep_key(int index);

#endif /* DRIFTY_CAR_CORPUS_H */
