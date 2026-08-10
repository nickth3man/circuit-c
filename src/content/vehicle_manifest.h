/*
 * vehicle_manifest.h — versioned vehicle content manifest (issue #29).
 *
 * A manifest is the external, human-reviewable form of one immutable VehicleDefinition plus its
 * default VehicleSetup and the content metadata a roster needs (display name, class tags,
 * controller eligibility, provenance). It replaces the hard-coded C roster with reviewed data so
 * a new car is a file, not a recompile.
 *
 * SCHEMA (version 1). The format is JSON, parsed by the strict reader in core/json.h. Every
 * field maps to the issue-12 parameter audit: a "physics" key must be a definition-owned registry
 * parameter, a "setup" key must be a setup-owned one, and a derived or session-rules field is
 * rejected outright. See docs/VEHICLE_MANIFEST.md for the full reference.
 *
 *   {
 *     "schema": "circuit/vehicle", "version": 1,
 *     "id": "rwd_grip", "displayName": "RWD Grip",
 *     "contentVersion": 1, "appearanceId": "rwd_grip",
 *     "contentKind": "player-selectable",
 *     "classTags": ["rwd", "road"], "controllerEligibility": ["human", "ai"],
 *     "provenance": { "source": "roster", "author": "circuit-c" },
 *     "physics": { "body.wheelbase": 2.55, ... },   // definition-owned keys
 *     "setup":   { "brake.bias_front": 0.57, ... }   // setup-owned keys
 *   }
 *
 * DETERMINISM. Loading is a pure function of the text: physics/setup keys apply to a spec that
 * starts from vehicle_spec_set_default(), so an unspecified key has one documented value on
 * every platform, and the manifest hash (json_canonical_hash) is stable across formatting. A
 * catalog is sorted by stable id after discovery, so the order files are enumerated cannot change
 * the roster order or any compatibility checksum.
 *
 * This translation unit calls no raylib function and links into the headless test executable.
 */
#ifndef CIRCUIT_VEHICLE_MANIFEST_H
#define CIRCUIT_VEHICLE_MANIFEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "physics/vehicle.h"

#define VEHICLE_MANIFEST_SCHEMA "circuit/vehicle"
#define VEHICLE_MANIFEST_VERSION 1

#define VEHICLE_MANIFEST_TEXT_CHARS \
    129 /* displayName, appearanceId, provenance fields (128 chars + NUL) */
#define VEHICLE_MANIFEST_DESC_CHARS 385 /* description (384 chars + NUL) */
#define VEHICLE_MANIFEST_MAX_CLASS_TAGS 8
#define VEHICLE_MANIFEST_CLASS_TAG_CHARS 33 /* class tag (32 chars + NUL) */

/* What a manifest is allowed to be used for. The appearance corpus grew samples that must never
 * appear as race cars, so a manifest now declares its own kind instead of every file in
 * data/vehicles/ implicitly being roster content (issue #31). */
typedef enum {
    VEHICLE_CONTENT_VISUAL_SAMPLE = 0, /* appearance corpus sample; never race content */
    VEHICLE_CONTENT_PROTOTYPE,         /* in review; not validated */
    VEHICLE_CONTENT_VALIDATED,         /* passed validation; not player-facing */
    VEHICLE_CONTENT_PLAYER_SELECTABLE  /* validated + listed for players */
} VehicleContentKind;

/* One loaded manifest. Plain data with fixed arrays, so it survives a hot reload and needs no
 * destructor beyond its own callers' lifetime. */
typedef struct {
    VehicleDefinition definition;   /* id/contentVersion/contentHash/appearanceId/spec */
    VehicleSetup defaultSetup;      /* the authored baseline setup */
    uint32_t manifestHash;          /* canonical hash of the whole document */
    VehicleContentKind contentKind; /* what this manifest may be used for */
    char displayName[VEHICLE_MANIFEST_TEXT_CHARS];
    char description[VEHICLE_MANIFEST_DESC_CHARS];
    char classTags[VEHICLE_MANIFEST_MAX_CLASS_TAGS][VEHICLE_MANIFEST_CLASS_TAG_CHARS];
    int classTagCount;
    bool eligibleHuman;
    bool eligibleAi;
    char provenanceSource[VEHICLE_MANIFEST_TEXT_CHARS];
    char provenanceAuthor[VEHICLE_MANIFEST_TEXT_CHARS];
} VehicleManifest;

/* A discovered, id-sorted set of manifests. Owned by the caller; free with vehicle_catalog_free. */
typedef struct {
    VehicleManifest *items;
    int count;
} VehicleCatalog;

/* Stable content id rule: [a-z0-9] then up to 62 of [a-z0-9._-]. Matches the ownership contract. */
bool vehicle_manifest_id_is_valid(const char *id);

/* Canonical JSON name for a content kind ("visual-sample", "prototype", "validated",
 * "player-selectable"). Never returns NULL. */
const char *vehicle_content_kind_name(VehicleContentKind kind);

/* Parse a content kind name into the enum. Returns false for an unknown string. */
bool vehicle_content_kind_parse(const char *text, VehicleContentKind *out);

/* Parse a manifest from memory. On failure returns false and writes a `field: reason` (or a JSON
 * `line N, column M: reason`) message. *out is left zeroed on failure. */
bool vehicle_manifest_parse(const char *text, size_t length, VehicleManifest *out, char *error,
                            size_t errorCap);

/* Load one manifest file. Same error contract as vehicle_manifest_parse. */
bool vehicle_manifest_load(const char *path, VehicleManifest *out, char *error,
                           size_t errorCap);

/* Discover every "*.vehicle.json" under `dir`, parse each, sort the catalog by stable id, and
 * reject duplicate ids. Returns false on any parse, I/O, or duplicate error. *out is zeroed and
 * nothing is allocated on failure. */
bool vehicle_manifest_load_dir(const char *dir, VehicleCatalog *out, char *error,
                               size_t errorCap);

void vehicle_catalog_free(VehicleCatalog *catalog);

/* Write a manifest for a built spec. The export is the human-readable mirror a roster author
 * reviews: every non-derived registry key the spec sets goes into physics or setup according to
 * its issue-12 owner, and the identity fields are taken from `id`/`displayName`/`appearanceId`.
 * Used by the `--generate-roster-manifests` export and asserted to round-trip by a scenario. */
bool vehicle_manifest_write(const VehicleSpec *spec, const char *id, const char *displayName,
                            const char *appearanceId, uint32_t contentVersion,
                            const char *source, FILE *out);

#endif /* CIRCUIT_VEHICLE_MANIFEST_H */
