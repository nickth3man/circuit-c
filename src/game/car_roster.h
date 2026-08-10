/*
 * car_roster.h — the six cars Milestone 1 validates, as pure functions of an index.
 *
 * WHY SIX, AND WHY THESE. Two per drivetrain layout, so that any claim about RWD, FWD or AWD
 * rests on more than one data point, and so that a difference between the two members of a
 * pair can only come from the things that actually differ between them. The roster exists to
 * make the validation suite argue with evidence: a lap time is only interesting next to
 * another lap time from a car that differs in a named way.
 *
 * MANIFEST-DRIVEN. The roster is data-driven, loaded dynamically from data/vehicles/ manifest files
 * via vehicle_manifest_load_dir() (Issue #30). Editing a manifest updates vehicle behavior, and
 * calling car_roster_reload() refreshes the catalog cache.
 *
 * These specs are NOT normalised to make any driver's job easier. A car that cannot be got
 * round the circuit is a result, not a bug in the roster.
 *
 * Raylib-free; headless-compatible.
 */
#ifndef CIRCUIT_CAR_ROSTER_H
#define CIRCUIT_CAR_ROSTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "content/vehicle_manifest.h"
#include "physics/vehicle.h"

/* Where the reviewed class-rule files live. Every class tag naming a file here is binding. */
#define CAR_ROSTER_CLASS_DIR "data/vehicles/classes"

#define CAR_ROSTER_MAX_REJECTIONS 32
/* Wide enough for the longest reason either gate can produce: a promotion check name
 * (PROMOTION_CHECK_NAME_CHARS) plus its evidence (PROMOTION_CHECK_DETAIL_CHARS) plus framing. */
#define CAR_ROSTER_REJECT_REASON_CHARS 256

/*
 * One manifest the roster refused, and why. A car that declares itself player-selectable but
 * fails the issue #31 promotion checklist or an issue #33 class rule is kept out of gameplay —
 * this is how an author finds out, rather than noticing an absence. Rejections are recorded in
 * catalog (stable-id) order, so the log is as deterministic as the roster itself.
 */
typedef struct {
    char id[VEHICLE_CONTENT_ID_CAPACITY];
    char reason[CAR_ROSTER_REJECT_REASON_CHARS];
} CarRosterRejection;

/* Number of manifests the current roster load refused. Loads the roster if needed. */
int car_roster_rejection_count(void);

/* Rejection `index`, or NULL when out of range. Points into module state: fine for immediate
 * reads, never to be stored across car_roster_reload(). */
const CarRosterRejection *car_roster_rejection(int index);

/* Total cars in the roster. Stable for a given build. */
int car_roster_count(void);

/*
 * Write car `index` into `out`. Pure: the same index always yields the same spec, whatever
 * order the calls come in. Returns false for an out-of-range index or a NULL out.
 *
 * Validity is asserted by the `roster` scenario rather than guaranteed here, which is how a
 * bad override gets caught loudly instead of producing a subtly broken car.
 */
bool car_roster_spec(int index, VehicleSpec *out);

/* Roster-cache pointer to the manifest at `index`, or NULL when the roster is unloaded or
 * `index` is out of range. Points into the internal catalog cache: fine for immediate reads,
 * never to be stored across car_roster_reload(). */
const VehicleManifest *car_roster_manifest(int index);

/* Stable, filesystem-safe identifier, e.g. "rwd_grip". Written into the caller's buffer so
 * there is no shared static state to go stale across a reload. */
void car_roster_id(int index, char *buf, size_t cap);

/* Human-readable name for a HUD or a report, e.g. "RWD Grip". Points at module rodata: fine
 * for immediate printing, never to be stored in Game (see hotreload.h). */
const char *car_roster_display_name(int index);

/* One line naming what this car is for, e.g. "high grip, moderate power — the reference". */
const char *car_roster_describe(int index);

/* "RWD" / "FWD" / "AWD", derived from the built spec rather than from a second table, so it
 * cannot disagree with what the car actually does. */
const char *car_roster_layout_name(int index);

/* FNV-1a over the built spec's bytes. Two runs quoting the same hash drove the same car, even
 * if someone edited a preset in between without bumping anything. */
uint32_t car_roster_spec_hash(int index);

/* Index of the car with this id, or -1. Lets `--car rwd_grip` resolve without the caller
 * knowing the table order. */
int car_roster_find(const char *id);
/* Reload or reset the cached manifest catalog from data/vehicles/. */
void car_roster_reload(void);

#endif /* CIRCUIT_CAR_ROSTER_H */
