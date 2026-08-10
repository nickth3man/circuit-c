/*
 * track_manifest.h — versioned external track format (issue #34).
 *
 * A track file is the external, human-reviewable form of one immutable TrackDefinition: the
 * authored centreline, surface bands, ordered checkpoints, and identity metadata. It replaces
 * the hard-coded C layouts with reviewed data so a new circuit is a file, not a recompile.
 *
 * SCHEMA (version 1). JSON, parsed by the strict reader in core/json.h. Version 1 represents
 * exactly what the current built-in tracks need: a closed route of width-banded centreline
 * nodes, off-track/runoff surfaces, an optional parking-lot region, and explicit checkpoints.
 * Grid, pit, environment, and presentation hooks are part of the documented forward-compatibility
 * plan (see docs/TRACK_FORMAT.md) but are not yet accepted by the parser, so v1 cannot be loaded
 * two ways depending on a field nothing acts on yet.
 *
 *   {
 *     "schema": "circuit/track", "version": 1,
 *     "id": "chicane", "displayName": "Chicane Validation Circuit",
 *     "contentVersion": "chicane_v1",
 *     "route": {
 *       "closed": true,
 *       "nodes": [ { "x": -100.0, "y": 0.0, "halfWidth": 8.0,
 *                    "runoffHalfWidth": 12.0, "surface": "asphalt" }, ... ]
 *     },
 *     "surfaces": { "offTrack": "grass", "runoff": "grass" },
 *     "parkingLot": { "minX": -200.0, "maxX": 200.0, "minY": -150.0, "maxY": 150.0 },
 *     "checkpoints": [ { "x": -60.0, "y": 0.0, "forwardX": 1.0, "forwardY": 0.0,
 *                        "halfWidth": 10.0, "required": true }, ... ]
 *   }
 *
 * COORDINATE SYSTEM (fixed for v1, documented rather than carried as a field). World metres, +X
 * east, +Y north; headings are radians counter-clockwise from +X (atan2 convention). Closed
 * routes wind counter-clockwise, so the circuit interior is on the left of the travel direction.
 * Forward vectors are authored as unit vectors and stored verbatim, not renormalized, so a
 * round-trip preserves the exact geometry hash.
 *
 * DETERMINISM. Numbers parse to a double via strtod and are stored as floats; on the supported
 * toolchains this round-trips the authored float bits exactly (see core/json.h). The manifest
 * hash (json_canonical_hash) is stable under formatting-only changes, and the geometry hash
 * (track_geometry_hash) is stable under any change that does not move a node or gate.
 *
 * Raylib-free; links into the headless test executable.
 */
#ifndef CIRCUIT_TRACK_MANIFEST_H
#define CIRCUIT_TRACK_MANIFEST_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "world/track.h"

#define TRACK_MANIFEST_SCHEMA "circuit/track"
#define TRACK_MANIFEST_VERSION 1

#define TRACK_MANIFEST_TEXT_CHARS 128 /* id/displayName fit the definition's fixed arrays */
#define TRACK_MANIFEST_DESC_CHARS 256

/* Parse a track file from memory into a heap-allocated TrackDefinition owned by the caller
 * (free with track_free). On success writes the manifest hash to *manifestHashOut when non-NULL.
 * On failure returns false, leaves *out zeroed, and writes a `field: reason` (or JSON
 * `line N, column M: reason`) message. */
bool track_manifest_parse(const char *text, size_t length, TrackDefinition *out,
                          uint32_t *manifestHashOut, char *error, size_t errorCap);

/* Load one track file. Same contract as track_manifest_parse. */
bool track_manifest_load(const char *path, TrackDefinition *out, uint32_t *manifestHashOut,
                         char *error, size_t errorCap);

/* Write a track file for a compiled TrackDefinition. The export is the human-readable mirror a
 * track author reviews: every node, checkpoint, surface, and the parking-lot bounds (when set),
 * serialized with %.9g so it round-trips the authored float bits. Used by the
 * `--generate-tracks` export and asserted to round-trip by a scenario. */
bool track_manifest_write(const TrackDefinition *track, const char *displayName,
                          const char *description, FILE *out);

/* Surface id <-> lowercase manifest key. Return NULL/(-1) for an unknown name or id. */
const char *track_manifest_surface_name(SurfaceId surface);
SurfaceId track_manifest_surface_id(const char *name);

#endif /* CIRCUIT_TRACK_MANIFEST_H */
