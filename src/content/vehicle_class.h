/*
 * vehicle_class.h — versioned class-rule manifests + eligibility diagnostics (issue #33).
 *
 * WHY THIS EXISTS. A class tag on a manifest ("road", "race") is a display string, not a
 * contract: nothing stops a typo from tagging a 1600 kg race car as "road". A class file is
 * the reviewed, versioned statement of what a class means numerically — optional bounds on
 * mass, peak torque, tire grip, and drivetrain layout. Eligibility is then a deterministic
 * check that a manifest carries the class id in its classTags AND that every rule the class
 * constrains is satisfied by the manifest's derived spec. The tag alone never grants
 * membership; the rules alone never grant it either (two classes may share an overlap, and
 * the tag is what a roster UI groups by).
 *
 * SCHEMA (version 1). The format is JSON, parsed by the strict reader in core/json.h, and
 * mirrors the vehicle manifest's conventions: unknown top-level keys are rejected, `field:
 * reason` errors, and a catalog is sorted by stable id after discovery with duplicate ids
 * rejected. All rule bounds are optional — an absent rule is unconstrained, so a class can
 * be as strict or as loose as its author decides:
 *
 *   {
 *     "schema": "circuit/vehicle-class", "version": 1,
 *     "id": "road", "displayName": "Road", "description": "...",
 *     "rules": {
 *       "mass_kg": [900, 1600],
 *       "peak_torque_nm": [100, 420],
 *       "max_tire_mu": 1.6,
 *       "layouts": ["rwd", "fwd"]
 *     }
 *   }
 *
 *   rules.mass_kg / rules.peak_torque_nm  inclusive [min, max] bound pairs.
 *   rules.max_tire_mu                     inclusive upper bound on max(lat mu front, rear).
 *   rules.layouts                         whitelist of "rwd"/"fwd"/"awd"; absent = any layout.
 *
 * ELIGIBILITY. vehicle_class_check_eligibility reads the manifest's derived spec: mass is
 * spec.massKg (the sum of the authored mass particles), peak torque is the max of the engine
 * torque curve, tire mu is max(tireMuLatFront, tireMuLatRear), and layout is spec.drivetrain-
 * Layout cast to the DrivetrainLayout enum (0=RWD, 1=FWD, 2=AWD). It never consults display
 * strings — displayName/description are human review text and play no part in membership.
 *
 * DETERMINISM. Parsing is a pure function of the text, and eligibility is a pure function of
 * a class and a manifest: identical inputs, identical verdict and evidence, on every platform.
 * Floating-point comparisons are plain inclusive bounds against derived floats; the class
 * files are authored with comfortable margins so float rounding cannot flip a verdict.
 *
 * This translation unit calls no raylib function and links into the headless test executable.
 */
#ifndef CIRCUIT_VEHICLE_CLASS_H
#define CIRCUIT_VEHICLE_CLASS_H

#include <stdbool.h>
#include <stddef.h>

#include "content/vehicle_manifest.h"

#define VEHICLE_CLASS_SCHEMA "circuit/vehicle-class"
#define VEHICLE_CLASS_VERSION 1

#define VEHICLE_CLASS_ID_CHARS 64    /* stable class id (63 chars + NUL) */
#define VEHICLE_CLASS_TEXT_CHARS 129 /* displayName (128 chars + NUL) */
#define VEHICLE_CLASS_DESC_CHARS 385 /* description (384 chars + NUL) */
#define VEHICLE_CLASS_MAX_LAYOUTS 3  /* "rwd", "fwd", "awd" — all three at most */

/* One loaded class. Plain data with fixed arrays, so it survives a hot reload and needs no
 * destructor beyond its own callers' lifetime. Rule fields are only meaningful when their
 * has* flag is set; layoutCount == 0 means any layout. */
typedef struct {
    char id[VEHICLE_CLASS_ID_CHARS];
    char displayName[VEHICLE_CLASS_TEXT_CHARS];
    char description[VEHICLE_CLASS_DESC_CHARS];
    bool hasMassRange;
    float minMassKg, maxMassKg;
    bool hasTorqueRange;
    float minPeakTorqueNm, maxPeakTorqueNm;
    bool hasMaxTireMu;
    float maxTireMu;
    char layouts[VEHICLE_CLASS_MAX_LAYOUTS][4]; /* "rwd"/"fwd"/"awd" */
    int layoutCount;                            /* 0 = any layout */
} VehicleClass;

/* Parse a class from memory. On failure returns false and writes a `field: reason` (or a JSON
 * `line N, column M: reason`) message. *out is left zeroed on failure. */
bool vehicle_class_parse(const char *text, size_t length, VehicleClass *out, char *error,
                         size_t errorCap);

/* Load one class file. Same error contract as vehicle_class_parse. */
bool vehicle_class_load(const char *path, VehicleClass *out, char *error, size_t errorCap);

/* A discovered, id-sorted set of classes. Owned by the caller; free with
 * vehicle_class_catalog_free. */
typedef struct {
    VehicleClass *items;
    int count;
} VehicleClassCatalog;

/* Discover every "*.vehicle-class.json" under `dir`, parse each, sort the catalog by stable
 * id, and reject duplicate ids. Returns false on any parse, I/O, or duplicate error. *out is
 * zeroed and nothing is allocated on failure. */
bool vehicle_class_load_dir(const char *dir, VehicleClassCatalog *out, char *error,
                            size_t errorCap);

void vehicle_class_catalog_free(VehicleClassCatalog *catalog);

/* Explicit-rules eligibility: the manifest must carry this class in classTags AND satisfy
 * every constrained rule. Returns true when the manifest is a member; on either failure
 * writes a one-line evidence string naming the failed rule (the tag rule, or the numeric
 * rule with the observed value and the bound). On success the detail is a satisfied summary
 * quoting the values that were checked. `detail`/`detailCap` may be NULL/0 to skip evidence.
 * Returns false with detail "no class"/"no manifest" for NULL arguments. */
bool vehicle_class_check_eligibility(const VehicleClass *cls, const VehicleManifest *manifest,
                                     char *detail, size_t detailCap);

#endif /* CIRCUIT_VEHICLE_CLASS_H */
