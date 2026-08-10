/*
 * car_selection.h — the headless car-selection model (issue #32): what the menu UI can offer
 * and how a stored choice is recovered, shared with the headless tests. No raylib here.
 *
 * WHY SELECTION IS AN INDEX OVER A SORTED VIEW. The roster catalog is id-sorted after
 * discovery (see vehicle_manifest.h), so selection order is a pure function of the manifests
 * themselves, not of filesystem enumeration order or platform directory iteration: the same
 * content yields the same menu on every machine. The model therefore exposes selection entries
 * in roster order and resolves choices by stable id, never by raw roster position — a saved
 * choice is "rwd_grip", not "slot 4", so reordering or removing content degrades to a
 * deterministic fallback (index 0) instead of silently selecting a different car. Indexes into
 * this view and the manifest pointers it hands out are only valid while the catalog is
 * unchanged; the pointer lives in the roster cache and must never be kept across
 * car_roster_reload().
 *
 * This translation unit calls no raylib function and links into the headless test executable.
 */
#ifndef CIRCUIT_CAR_SELECTION_H
#define CIRCUIT_CAR_SELECTION_H

#include <stdbool.h>
#include <stddef.h>

#include "content/vehicle_manifest.h"

/*
 * The ids this model carries are content ids copied straight out of VehicleDefinition.id, so
 * the capacity is taken from there rather than restated. A future widening of the content-id
 * capacity would otherwise silently truncate menu and recall ids against a stale 64.
 */
#define CAR_SELECTION_ID_CHARS VEHICLE_CONTENT_ID_CAPACITY
#define CAR_SELECTION_RECALL_PATH "data/input/last_car.txt"

typedef struct {
    char id[CAR_SELECTION_ID_CHARS];
    const VehicleManifest *manifest; /* roster-cache pointer; do not store across reload */
    int rosterIndex;                 /* index into car_roster_* accessors */
} CarSelectionEntry;

/* Selectable = player-selectable (roster already filters) AND eligibleHuman. Roster order (id-sorted). */
int car_selection_count(void);
bool car_selection_entry(int index, CarSelectionEntry *out);
int car_selection_resolve(const char *id); /* selection index or -1 */
int car_selection_index_or_default(
    const char *id); /* resolve, else 0 (deterministic fallback) */
/* Persistence of the last valid selection by stable id. load returns false when the file is
 * absent, unreadable, empty, or the id fails vehicle_manifest_id_is_valid. save returns false
 * for an invalid id or I/O failure. Format: the id, one line. */
bool car_selection_load_recall(char *idBuf, size_t cap);
bool car_selection_save_recall(const char *id);

#endif /* CIRCUIT_CAR_SELECTION_H */
