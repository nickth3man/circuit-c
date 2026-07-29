/*
 * car_corpus_internal.h — the seam between the corpus's archetype data and the corpus
 * machinery. Only src/dev/car_corpus.c and src/dev/car_corpus_archetypes.c may include
 * this; the public contract stays in dev/car_corpus.h.
 *
 * The archetype translation unit is the sole owner of the ArchetypeDef table. Invalid
 * indices or null outputs return false / NULL.
 */
#ifndef DRIFTY_CAR_CORPUS_INTERNAL_H
#define DRIFTY_CAR_CORPUS_INTERNAL_H

#include <stdbool.h>

#include "physics/vehicle.h"

int car_corpus_archetype_count(void);
bool car_corpus_archetype_build(int index, VehicleSpec *out);
const char *car_corpus_archetype_name(int index);
const char *car_corpus_archetype_description(int index);

#endif /* DRIFTY_CAR_CORPUS_INTERNAL_H */
